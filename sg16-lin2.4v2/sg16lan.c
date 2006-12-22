/* sg16lan.c:  Sigrand SG-16PCI, SG-16ISA SHDSL modems driver for linux (kernel 2.4.x)
 *
 *	Written 2002-2003 by Denis I.Timofeev
 *
 *	This driver presents SG16-PCI modem	to system as common
 *	ethernet-like netcard.
 *	More info and useful utilities to work with SG16 modems you can find
 *	at http://www.sigrand.ru (Russian)
 *
 *	This software may be used and distributed according to the terms
 *	of the GNU General Public License.
 *
 *
 *	22.11.2002	initial revision
 *	28.01.2003	ported from 2.2.x kernel version
 *	17.07.2003	fixed hung-up problem - Eugene Danilov
 *	13.01.2004  added carrier reporting - Yaroslav Polyakov
 *	04.02.2004 added small packets padding - Maxim Shilov
 *	01.07.2006 Version 1.0 (initial revision)  - Artem U. Polyakov
 *	05.07.2006 Version 2.0 (added control through procfs)  - Artem Plyakov
 *	25.11.2006 Version 3.0 (SG-16ISA card support) - Artem Plyakov
 */
 
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/io.h>
#include <asm/types.h>
#include <asm/byteorder.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/uaccess.h>

#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/isapnp.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
//#include <net/arp.h>

#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/random.h>
#include <linux/timer.h>

#include "sg16lan.h"
#include "cx28975.h"
// Debug parameters
//#define DEBUG_ON
#define DEFAULT_LEV 5
#include "sg16debug.h"


MODULE_DESCRIPTION( "Sigrand SG-16PCI driver Version 3.0\n" );
MODULE_AUTHOR( "Maintainer: Artem U. Polyakov art@sigrand.ru\n" );
MODULE_LICENSE( "GPL" );

/* --------------------------------------------------------------------------
 *      Module initialisation/cleanup
 * -------------------------------------------------------------------------- */
 
int
init_module( void )
{
	sg16_entry=proc_mkdir(sg16_procdir,NULL);
	if( sg16_entry==NULL )
		return -ENOMEM;
	pci_module_init( &sg16_pci_driver );
	PDEBUG(0,"isapnp_register_driver");
	isapnp_register_driver( &sg16_isa_driver );
	return 0;
}

void
cleanup_module( void )
{
	PDEBUG(0,"isapnp_unregister_driver");
	isapnp_unregister_driver( &sg16_isa_driver);
	PDEBUG(0,"pci_unregister_driver");	
	pci_unregister_driver( &sg16_pci_driver );
	PDEBUG(0,"remove_proc_entry(sg16_procdir,NULL)");		
	remove_proc_entry(sg16_procdir,NULL);		

}

void
dsl_init( struct net_device *ndev)
{
        ether_setup(ndev);
	ndev->init = sg16_probe;
}


/* --------------------------------------------------------------------------
 *      PCI adapter initialisation/cleanup
 * -------------------------------------------------------------------------- */
	
static int __devinit
sg16_pci_probe_one( struct pci_dev  *pdev,  const struct pci_device_id  *ent )
{
	struct net_device  *ndev;
	struct net_local *nl;
	int error=0;
	
	PDEBUG(5,"start");
	
	if( pci_enable_device( pdev ) )
		return  -EIO;
	pci_set_master(pdev);
	// allocate memory for network device
	if( !( ndev = alloc_netdev( sizeof(struct net_local),"dsl%d",dsl_init)) ){
		error = -ENOMEM;
		goto err_exit;
	}
	PDEBUG(5,"alloc_netdev");	
	// set pci device data field
	pci_set_drvdata( pdev, ndev );
	// set some net device fields
	ndev->mem_start = pci_resource_start( pdev, 1 );
	ndev->mem_end = pci_resource_end( pdev,1 );
	ndev->irq = pdev->irq;
	// device private data initialisation   
	nl=(struct net_local *)netdev_priv( ndev);
	memset( nl, 0, sizeof(struct net_local) );
	nl->dev_type=sg16pci;
	PDEBUG(5,"before register_netdev");	
	if( register_netdev( ndev ) ) {
		error = -ENODEV;
		goto err_exit1;
	}
	return 0;
	
err_exit1:
	kfree(ndev);
err_exit:
	pci_disable_device(pdev);
	return error;
}


static void __devexit
sg16_remove_one( struct pci_dev  *pdev )
{
	struct net_device  *ndev = pci_get_drvdata( pdev );
	struct net_local  *nl  = (struct net_local *) ndev->priv;
	
	del_sg16_from_procfs(ndev);
	unregister_netdev(ndev);
	iounmap(((struct net_local *)ndev->priv)->mem_base);
	release_mem_region(ndev->mem_start, 0x1000);
	free_irq(ndev->irq, ndev);

	if( nl->dev_type == sg16pci )
		pci_disable_device(pdev);
	else{
		free_dma(ndev->dma);	
		pdev->deactivate(pdev);
	        kfree(nl->rbuf);
	        kfree(nl->xbuf);
	}
	kfree(ndev);
}

/* --------------------------------------------------------------------------
 *      ISA adapter initialisation/cleanup
 * -------------------------------------------------------------------------- */

 
static int __devinit
sg16_isapnp_probe_one( struct pci_dev  *idev,  const struct isapnp_device_id  *ent )
{
	struct net_device  *ndev;
	struct net_local *nl;
	int error=0;
	
	PDEBUG(0,"start");

	if( (error = idev->prepare(idev)) ){
		PDEBUG(0,"prepare fail");
		return error;
	}
	idev->deactivate(idev);
	if( (error = idev->activate(idev) ) ){
		PDEBUG(0,"activate fail, error=%d",error);
		return error;
	}
		
	// allocate memory for network device
	if( !( ndev = alloc_netdev( sizeof(struct net_local),"dsl%d",dsl_init)) ){
		PDEBUG(0,"alloc_netdev fail");
		error = -ENOMEM;
		goto err_exit;
	}

	// set pci device data field
	pci_set_drvdata( idev, ndev );
	// set some net device fields
	int i;
	ndev->mem_start = idev->resource[8].start;
	ndev->mem_end = idev->resource[8].end;
	ndev->irq = idev->irq_resource[0].start;
	ndev->dma = idev->dma_resource[0].start;
	// device private data initialisation   
	nl=(struct net_local *)netdev_priv( ndev);
	memset( nl, 0, sizeof(struct net_local) );
	nl->dev_type=sg16isa;
	
        // allocate ISA DMA able buffers
        nl->rbuf=kmalloc(RQLEN*(sizeof(struct dma_buffer)+10),GFP_DMA | GFP_KERNEL);
	for(i=0;i<RQLEN;i++)
                nl->rbuf[i].data=(u8*)&(nl->rbuf[i].buff)+2;
        nl->xbuf=kmalloc(XQLEN*(sizeof(struct dma_buffer)+10),GFP_DMA | GFP_KERNEL);
	for(i=0;i<XQLEN;i++)
                nl->xbuf[i].data=(u8*)&(nl->xbuf[i].buff)+2;
								
	if( register_netdev( ndev ) ) {
		error = -ENODEV;
		goto err_exit1;
	}

	return  0;
err_exit1:
	kfree(ndev);
err_exit:
	idev->deactivate(idev);
	return error;
}

/*
static void __devexit
sg16_remove_one( struct pci_dev  *pdev )
{
	struct net_device  *ndev = pci_get_drvdata( pdev );

	del_sg16_from_procfs(ndev);
	unregister_netdev(ndev);
	free_irq(dev->irq, ndev);
	iounmap(((struct net_local *)ndev->priv)->mem_base);
	release_mem_region(ndev->mem_start, 0x1000);
	kfree(ndev);
	pci_disable_device(pdev);
}
*/

/* --------------------------------------------------------------------------
 *   Network device functions   
 * -------------------------------------------------------------------------- */


static int __init
sg16_probe( struct net_device  *ndev )
{
	struct net_local  *nl  = (struct net_local *) ndev->priv;
	int error=0;
		
	ndev->open = &sg16_open;
	ndev->stop = &sg16_close;
	ndev->hard_start_xmit = &sg16_start_xmit;
	ndev->get_stats = &sg16_get_stats;
	ndev->set_multicast_list = &set_multicast_list;
	ndev->do_ioctl = &sg16_ioctl;
	ndev->tx_timeout = &sg16_tx_timeout;
	ndev->watchdog_timeo = TX_TIMEOUT;
	init_etherdev( ndev, 0 );
	// generate 'unique' MAC address
	*(u16 *)ndev->dev_addr = htons( 0x00ff );
	*(u32 *)(ndev->dev_addr + 2) = htonl( 0x01a39000 | ( ((long)jiffies ^ (long)ndev->priv ) & 0x00000fff) );

	spin_lock_init( &nl->lock );
	init_waitqueue_head( &nl->wait_for_intr );

	if( !request_mem_region( ndev->mem_start,SG16_IOMEM_LEN,ndev->name) )
		return  -ENODEV;
	nl->mem_base = (void *) ioremap(ndev->mem_start,SG16_IOMEM_LEN);
	nl->state = NOT_LOADED;
	nl->tbd  = (struct hw_descr *) nl->mem_base;
	nl->rbd  = (struct hw_descr *) ((u8 *)nl->mem_base + 0x400);
	nl->regs = (struct sg16_hw_regs *) ((u8 *)nl->mem_base + 0x800);
	nl->cmdp = (struct cx28975_cmdarea *) ((u8 *)nl->mem_base + 0xc00);
	nl->regs->CR = 0;
	nl->regs->CRB = RXDE;	
	nl->regs->SR = 0xff;
	nl->regs->IMR = 0;


	if((error=request_irq(ndev->irq, sg16_interrupt, SA_SHIRQ, ndev->name, ndev))) {
		printk( KERN_ERR "%s: unable to get IRQ %d.\n",
				ndev->name, ndev->irq );
		goto err_mem;
	}
	
	if( nl->dev_type == sg16isa ){
		if( (error=request_dma(ndev->dma,"sg16lan")) ){
			PDEBUG(0,"dma not registered, error=%d",error);
				goto err_irq;
		}
		set_dma_mode(ndev->dma, DMA_MODE_CASCADE);	
		enable_dma(ndev->dma);
	}
		
	if( nl->dev_type == sg16pci )
		printk( KERN_NOTICE "%s: Sigrand SG-16PCI (irq %d, mem %#lx)\n",
			ndev->name, ndev->irq, ndev->mem_start );
	else
		printk( KERN_NOTICE "%s: Sigrand SG-16ISA (irq %d, mem %#lx dma %d)\n",
			ndev->name, ndev->irq, ndev->mem_start, ndev->dma );

	// init device in procfs 
	init_sg16_in_procfs(ndev);

	// timered link chk entire
	nl->link_state.data = ( unsigned long )ndev;
	nl->link_state.function = shdsl_link_chk;
	init_timer( &(nl->link_state) );

	// initialise receiver variavles of net device
	netif_carrier_off(ndev);
	if( !netif_queue_stopped(ndev) )
		netif_stop_queue(ndev);
	
	SET_MODULE_OWNER( ndev );
	return  0;

err_irq:
	free_irq(ndev->irq, ndev);
err_mem:
	iounmap(nl->mem_base);
	release_mem_region(ndev->mem_start,SG16_IOMEM_LEN);
	return error;
}

/* -------------------------------------------------------------------------- */

static void
sg16_interrupt( int  irq,  void  *dev_id,  struct pt_regs  *regs )
{
	struct net_device  *dev = (struct net_device *) dev_id;
	struct net_local  *nl  = (struct net_local *) dev->priv;
	u8  status = nl->regs->SR;

	if( status == 0 )
		return;

	u8  mask = nl->regs->IMR;
	nl->regs->IMR=0;


	if( status & EXT )
		cx28975_interrupt( dev ),
		nl->regs->SR = EXT;
	
	// Whether transmit error is occured, we have to re-enable the
	// transmitter. That's enough, because linux doesn't fragment packets.
	if( status & UFL )
		nl->regs->CR |= TXEN,
		nl->regs->SR = UFL,
		++nl->in_stats.ufl_errs,
		++nl->stats.tx_errors,
		++nl->stats.tx_fifo_errors;

	if( status & RXS )
		indicate_frames( dev ),
		alloc_rx_buffers( dev ),
		nl->regs->SR = RXS;

	if( status & TXS )
		free_sent_buffers( dev ),
		nl->regs->SR = TXS;

	if( status & CRC )
		++nl->in_stats.crc_errs,
		++nl->stats.rx_errors,
		++nl->stats.rx_crc_errors,
		nl->regs->SR = CRC;

	if( status & OFL )
		++nl->in_stats.ofl_errs,
		++nl->stats.rx_errors,
		++nl->stats.rx_over_errors,
		nl->regs->SR = OFL;

	nl->regs->IMR=mask;
}


static void
cx28975_interrupt( struct net_device  *dev )
{
	struct net_local   *nl  = (struct net_local *) dev->priv;
	volatile struct cx28975_cmdarea  *p = nl->cmdp;
	u8  t;

    if( p->intr_host != 0xfe )
	return;
    	
    if( p->out_ack & 0x80 ) {
	nl->link_state.expires = jiffies + HZ/2;
	add_timer( &(nl->link_state) );	    
    } else {
	nl->irqret=p->out_ack;
	wake_up( &nl->wait_for_intr );
    }
    p->out_ack=0;
    // interrupt served
    p->intr_host=0;
    t=p->intr_host;
}


static void shdsl_link_chk( unsigned long data)
{
    struct net_device *ndev=(struct net_device *)data;
    struct net_local  *nl  =(struct net_local *)ndev->priv;
    volatile struct cx28975_cmdarea  *p = nl->cmdp;
		    
    if( *((u8 *)p + 0x3c7) & 2 ) {
	if( (*((u8 *)p + 0x3c0) & 0xc0) == 0x40 )
	    activate( ndev );
	else if( (*((u8 *)p + 0x3c0) & 0xc0) != 0x40 )
	    deactivate( ndev );
    }
}

//---- transmit function ----//

/* -------------------------------------------------------------------------- */

/*
 *	Open/initialize the board. 
 */

static int
sg16_open( struct net_device  *dev )
{
	struct net_local	*nl = (struct net_local *) dev->priv;
	u8  t;

	memset( &nl->stats, 0, sizeof(struct net_device_stats) );
	memset( &nl->in_stats, 0, sizeof(struct sg16_stats) );

	nl->head_xq = nl->tail_xq = nl->head_rq = nl->tail_rq = 0;

	if( nl->state == NOT_LOADED ) {
		printk( KERN_ERR "%s: firmware wasn't loaded\n", dev->name );
		return  -EBUSY;
	}

	nl->regs->IMR = EXT;

	t=0;
	if( issue_cx28975_cmd(nl,_DSL_RESET_SYSTEM,&t,1) )
	    return -EIO;
	if( !shdsl_ready(nl,_ACK_OPER_WAKE_UP) )
	    return -EIO;
	if( preactivation( dev ) )
	    return -EIO;

	t = 2;
	if( issue_cx28975_cmd( nl, _DSL_CLEAR_ERROR_CTRS, &t, 1 ) )
		return  -EIO;
	if( issue_cx28975_cmd( nl, _DSL_ACTIVATION, &t, 1 ) )
		return  -EIO;

	// start transmission through interface
	nl->regs->CTDR = nl->regs->LTDR = nl->regs->CRDR = nl->regs->LRDR = 0;	
	nl->head_tdesc = nl->head_rdesc = 0;
	alloc_rx_buffers( dev );
	netif_wake_queue( dev );	

	return 0;
}

static int
sg16_close( struct net_device  *ndev )
{
	struct net_local *nl = (struct net_local *) ndev->priv;
	u8  t;

	// block receiver interrupts
	nl->regs->IMR = EXT;
	// deactivate connection
	t = 0;
	issue_cx28975_cmd( nl, _DSL_ACTIVATION, &t, 1 );
	t = 1;
	issue_cx28975_cmd( nl, _DSL_FORCE_DEACTIVATE, &t, 1 );
	// first turn off transmission then free buffers
	netif_tx_disable( ndev );
	nl->regs->CTDR = nl->regs->LTDR;
	nl->regs->CRDR = nl->regs->LRDR;
	free_sent_buffers( ndev );
	drop_rq( nl );

	return 0;
}

static struct net_device_stats *
sg16_get_stats( struct net_device  *dev )
{
	return  &((struct net_local *) dev->priv)->stats;
}


static void
set_multicast_list( struct net_device  *dev )
{
	return;		/* sg16 always operate in promiscuos mode */
}

/* -------------------------------------------------------------------------- 
 * ioctl, serve for firmware download
 * --------------------------------------------------------------------------*/

static int
sg16_ioctl( struct net_device  *dev,  struct ifreq  *ifr,  int  cmd )
{
    struct cx28975_fw fw;
    u8   *firmw_image;
    int  error = 0;
    int err;

    PDEBUG(9,"ioctl: start\n");
    if( cmd ==  SIOCDEVLOADFW ){

    	if( current->euid != 0 )	/* root only */
	    return  -EPERM;
    	if( (dev->flags & IFF_UP) == IFF_UP )
    	    return  -EBUSY;
	if( (error = verify_area( VERIFY_READ, ifr->ifr_data,
     		sizeof(struct cx28975_fw) )) != 0 )
	    return  error;

	copy_from_user( &fw, ifr->ifr_data, sizeof fw );
	if( (error = verify_area( VERIFY_READ, fw.firmw_image,
			  fw.firmw_len )) != 0 )
	    return  error;
	if( !(firmw_image = vmalloc( fw.firmw_len )) )
	    return  -ENOMEM;
	copy_from_user( firmw_image, fw.firmw_image, fw.firmw_len );
	if( download_firmware( dev, fw.firmw_len, firmw_image ) )
	    err=-1;
	vfree( firmw_image );
    }
    PDEBUG(9,"ioctl: end\n");
    return  error;
}


/* -------------------------------------------------------------------------
 * Chip operations 
 * ------------------------------------------------------------------------*/

static int
shdsl_ready(struct net_local *nl, u8 expect)
{
    u8 ret_val=1;
    u32 ret;
	
    ret=interruptible_sleep_on_timeout( &nl->wait_for_intr, HZ*10 );
    if( ( nl->irqret & 0x1f) != expect )
        ret_val=0;

    nl->irqret=0;
    return ret_val;
}

static int
download_firmware( struct net_device  *dev,u32 img_len, u8 *img )
{
    struct net_local  *nl = (struct net_local *) dev->priv;
    struct cx28975_cfg *cfg=(struct cx28975_cfg *)&(nl->cfg);    
    volatile struct cx28975_cmdarea  *p = nl->cmdp;
    u32  t;
    int  i;
    u8   cksum = 0;
	
    p->intr_host = 0;
    t = p->intr_host;

    /* reset chip set */
    nl->regs->IMR = EXT;
    nl->regs->CR  = 0;
    nl->regs->SR  = 0xff;
    udelay( 2 );
    nl->regs->CR = XRST;

//---- prepare to download ----//
    if( !shdsl_ready(nl, _ACK_BOOT_WAKE_UP) )
        return  -1;

	for( i = 0;  i < img_len;  ++i )
    		cksum += img[i];
	PDEBUG(9,"download: BOOT_UP\n");
//---- download process ----//
    t = img_len;
    if( issue_cx28975_cmd( nl, _DSL_DOWNLOAD_START, (u8 *) &t, 4 ) )
    	return  -1;
    for( i = 0;  img_len >= 75;  i += 75, img_len -= 75 ) {
    	if( issue_cx28975_cmd( nl, _DSL_DOWNLOAD_DATA, img + i, 75 ) )
	    return  -1;
    }
    if( img_len
	    &&  issue_cx28975_cmd( nl, _DSL_DOWNLOAD_DATA, img + i, img_len ) )
	return  -1;
    t = (cksum ^ 0xff) + 1;
    if( issue_cx28975_cmd( nl, _DSL_DOWNLOAD_END, (u8 *) &t, 1 ) )
	return  -1;
//---- check that download is successfull ----//
    udelay(10);
    if( !shdsl_ready(nl, _ACK_OPER_WAKE_UP) )
	return -1;
//---- check that autorate selection supported ----//
    t=0;
    if( issue_cx28975_cmd(nl,_DSL_VERSIONS,(u8*)&t,1) )
	return -1; 
    
    if( *( (u8*)&(p->out_ack) + 4 ) & 1 ){
	cfg->autob_en=0;
	cfg->autob=0;	
    }
    else
	cfg->autob_en=1;	

    nl->state=LOADED;
PDEBUG(9,"download: end\n");	
//---- change adapter state ----//	
    return  0;
}


static int
preactivation( struct net_device  *dev )
{
    static char  thresh[] = { +8, -4, -16, -40 };
    struct net_local  *nl = (struct net_local *) dev->priv;
    struct cx28975_cfg *cfg=(struct cx28975_cfg *)&(nl->cfg);
    u8   t, parm[ 12 ],iter;
    u16 max_rate=0,min_rate=0,tmp;

    PDEBUG(0,"preact: start\n");
    if( cfg->crc16 )	nl->regs->CR |= CMOD;
    if( cfg->fill_7e )	nl->regs->CR |= FMOD;
    if( cfg->inv )	nl->regs->CR |= PMOD;

    nl->regs->CRB |= RODD | RXDE;
    if( cfg->rburst )	nl->regs->CRB |= RDBE;
    if( cfg->wburst )	nl->regs->CRB |= WTBE;

//----_DSL_SYSTEM_ENABLE----//
    t = cfg->master ? 1 : 9;
    if( issue_cx28975_cmd( nl, _DSL_SYSTEM_ENABLE, &t, 1 ) )
	return  -1;
//----_DSL_SYSTEM_CONFIG----//
    t = 0x63;
    if( issue_cx28975_cmd( nl, _DSL_SYSTEM_CONFIG, &t, 1 ) )
	return  -1;
//---- _DSL_MULTI_RATE_CONFIG----//
    if( cfg->autob ){
        max_rate=MAX_AUTO_RATE;
        min_rate=MIN_AUTO_RATE;
    }else if( cfg->remcfg ){
	if( cfg->master ){
	    if( cfg->annex==ANNEX_F )
    		max_rate=MAX_REMCFGF_RATE;
    	    else if( cfg->master ){
    		max_rate=MAX_REMCFGAB_RATE;
    		min_rate=MIN_REMCFG_RATE;
    	    }
	}
    }else{
    	max_rate=MAX_RATE;
	min_rate=MIN_RATE;
    }
	
    if( max_rate ){
        tmp=(min_rate >> 3) & 0x3ff;
        cfg->lrate= ( cfg->lrate < tmp ) ? tmp : cfg->lrate;
        tmp=(max_rate >> 3) & 0x3ff;
        cfg->lrate= ( cfg->lrate > tmp ) ? tmp : cfg->lrate;
    }
    else
        cfg->lrate=192 >> 3;

    memset( parm, 0, 8 );
    *(u16 *)parm = ( cfg->lrate >> 3 ) & 0x7f ;
    parm[2] = parm[3] = parm[0];
    parm[5] = cfg->lrate & 7;
    parm[4] = parm[7] = 1;
    parm[6] = 0;
    if( issue_cx28975_cmd( nl, _DSL_MULTI_RATE_CONFIG, parm, 8 ) )
    	return  -1;

//----_DSL_PREACT_RATE_LIST----//

    // if DSL configured with automatic rate select
    if( cfg->autob )
    {
	// Set List of aviable Rates
	memset( parm, 0, 36 );
	parm[0]=0;
	parm[1]=( u8 )( ((cfg->lrate - ( (MIN_AUTO_RATE >> 3) & 0x7f) ) >> 3 ) & 0x7f )+ 1;
	for(iter=0; iter<parm[1] ; iter++)
	    parm[iter+2]=3+iter;
	if( issue_cx28975_cmd( nl, _DSL_PREACT_RATE_LIST, parm, iter+2 ) )
	    return  -EIO;
    }
//----_DSL_TRAINING_MODE----//
    if( cfg->autob )
	parm[0] = 0x02 | 0x01<<4;       // In auto rate mode using only TCPAM16
    else if( cfg->remcfg ){		// In Remote rate config using only TCPAM16 & TCPAM32
	if( cfg->master && (cfg->annex==ANNEX_F) && ( cfg->mod==0x00 || cfg->mod==0x01 ) )
    	    parm[0] = 0x02 |(cfg->mod << 4);
	else
	    parm[0] = 0x02 | (0x01 << 4); // If TCPAM8 or TCPAM4 - set TCPAM16
    }
    else
	parm[0] = 0x02 |(cfg->mod << 4);
    parm[1] = 0;
    if( issue_cx28975_cmd( nl, _DSL_TRAINING_MODE, parm, 2 ) )
	return  -EIO;

//----_DSL_PREACTIVATION_CFG----//

    memset( parm, 0, 12 );
    parm[0] = 0x04;     // pre-activation: G.hs
    if( cfg->autob )
        parm[1] = 0x01; // Line probe Enabled
    else
        parm[1] = 0x00; // Line probe Disabled
				
    if( cfg->autob || cfg->remcfg )
        parm[4] = 0x00; // HTU-C send Mode Select message
    else
        parm[4] = 0x04; // No remote configuration
							
    parm[5] = 0x01;     // TPS-TC Config= Clear Channel
    parm[6] = 0x00;
					
    parm[7]=cfg->annex; // annex A,B,F
    parm[8] = 0x01;     // i-bit mask (all bits)
    if( issue_cx28975_cmd( nl, _DSL_PREACTIVATION_CFG, parm, 12 ) )
        return  -EIO;

//----_DSL_THRESHOLDS----//
    parm[0] = 0x03;		/* dying gasp time - 3 frames */
    parm[1] = thresh[ cfg->mod ];	
    parm[2] = 0xff;		/* attenuation */
    parm[3] = 0x04;		/* line probe NMR (+2 dB) */
    parm[4] = 0x00;		/* reserved */
    parm[5] = 0x00;
    if( issue_cx28975_cmd( nl, _DSL_THRESHOLDS, parm, 6 ) )
    	return  -1;
//----_DSL_FR_PCM_CONFIG----//
    t = cfg->master ? 0x23 : 0x21;
    if( issue_cx28975_cmd( nl, _DSL_FR_PCM_CONFIG, &t, 1 ) )
	return  -1;
//----_DSL_INTR_HOST_MASK----//
    t = 0x02;
    if( issue_cx28975_cmd( nl, _DSL_INTR_HOST_MASK, &t, 1 ) )
    	return  -1;
    PDEBUG(0,"preact: end\n");

    return  0;
}


static int
shdsl_get_stat(struct net_local *nl, struct dsl_stats *ds)
{
    u8 t;
    int  i;
    volatile struct cx28975_cmdarea  *p = nl->cmdp;
    
    t = 0;
    if( issue_cx28975_cmd( nl, _DSL_FAR_END_ATTEN, &t, 1 ) )
        return -EIO;
    ds->attenuat = p->out_data[0];
    
    if( issue_cx28975_cmd( nl, _DSL_NOISE_MARGIN, &t, 1 ) )
        return -EIO;
    ds->nmr = p->out_data[0];

    if( issue_cx28975_cmd( nl, _DSL_POWER_BACK_OFF_RESULT, &t, 1 ) )
        return -EIO;
    ds->tpbo = p->out_data[0];
    ds->rpbo = p->out_data[1];
    
    if( !issue_cx28975_cmd( nl, _DSL_SYSTEM_PERF_ERR_CTRS, &t, 1 ) ){
        for( i = 0;  i < 4;  ++i )
            ((u8 *)&(ds->all_atmpt))[i] = p->out_data[i];
    }
    else
        return -EIO;

    if( !issue_cx28975_cmd( nl, _DSL_HDSL_PERF_ERR_CTRS, &t, 1 ) ) {
        for( i = 0;  i < 10;  ++i )
            ((u8 *)&(ds->losw))[i] = p->out_data[i];
    }
    else
        return -EIO;
    return 0;
}

static int
shdsl_clr_stat( struct net_local  *nl )
{
    u8 t;
    memset( &nl->in_stats, 0, sizeof(struct sg16_stats) );
    t =0;
    if( issue_cx28975_cmd( nl, _DSL_CLEAR_ERROR_CTRS, &t, 1 ) )
        return -EIO;
    t =0x04;
    if( issue_cx28975_cmd( nl, _DSL_CLEAR_ERROR_CTRS, &t, 1 ) )
        return -EIO;

    return 0;
}

static int
issue_cx28975_cmd( struct net_local  *nl,  u8  cmd,  u8  *data,  u8  size )
{
	volatile struct cx28975_cmdarea  *p = nl->cmdp;
	u8  *databuf = p->in_data;
	int  i;
	u8  cksum = 0;

	p->in_dest	= 0xf0;
	p->in_opcode	= cmd;
	p->in_zero	= 0;
	p->in_length	= --size;
	p->in_csum	= 0xf0 ^ cmd ^ size ^ 0xaa;

	for( i = 0;  i <= size;  ++i ) {
		cksum ^= *data;
		*databuf++ = *data++;	// only 1 byte per cycle!
	}

	p->in_datasum	= cksum ^ 0xaa;
	p->out_ack	= _ACK_NOT_COMPLETE;
	p->intr_8051	= 0xfe;

	if( !shdsl_ready(nl,_ACK_PASS ) ){
	    PDEBUG(9,"cmd: error, cmd=%x",cmd);
	    return  -1;
	}
	return 0;
}



/* -------------------------------------------------------------------------- 
 * Transmit-receive functions
 * --------------------------------------------------------------------------*/

static void*
xmit_isa_skb_to_dma(struct net_local *nl,struct sk_buff **skb_in,int elem)
{
        struct sk_buff *skb=*skb_in;
        struct dma_buffer *b=(struct dma_buffer *)nl->xbuf+elem;
        PDEBUG(xmit_debug,"xbuf: strt=%08x,offs=%d,adr=%08x",(u32)nl->xbuf,(int)elem,(u32)b);
	memcpy(b->buff,skb->data,skb->len);
        if( skb->len<ETHER_MIN_LEN ){
                memset((u8*)b->buff+skb->len,0,ETHER_MIN_LEN-skb->len);
		skb->len=ETHER_MIN_LEN;
	}
	return (void*)b->buff;
}


static void*
xmit_pci_skb_to_dma(struct net_local *nl,struct sk_buff **skb_in)
{
	struct sk_buff *nskb,*skb=*skb_in;
	unsigned pad;
	PDEBUG(xmit_debug,"start,skb=%08x",skb);
	if( skb->len < ETHER_MIN_LEN ){
		 pad = ETHER_MIN_LEN - skb->len;
	         // if we have enough space just fill it with 0 and resize skb*/
		 // If the skbuff is non linear tailroom is always zero.. */
		PDEBUG(xmit_debug,"need pad");
		if(skb_tailroom(skb) >= pad){
			memset(skb->data + skb->len, 0, pad);
			skb->len = ETHER_MIN_LEN;			
		}else{
			PDEBUG(xmit_debug,"need pad&expand");		 
	        	nskb = skb_copy_expand(skb, skb_headroom(skb), skb_tailroom(skb) + pad, GFP_ATOMIC);
			if(!nskb)
				return NULL;
			dev_kfree_skb_any( skb );
	        	skb = nskb;
                	memset(skb->data + skb->len, 0, pad);
			skb->len = ETHER_MIN_LEN;
		}
	}
	*skb_in=skb;
	PDEBUG(xmit_debug,"end, skb=%08x",skb);
	return (void*)skb->data;
}	 

static void*
xmit_skb_to_dma(struct net_device *ndev,struct sk_buff **skb,int elem)
{
        struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);
	u32 max_mem_addr=virt_to_bus((*skb)->data)+(*skb)->len;
	
        if( (nl->dev_type == sg16isa) &&
		( max_mem_addr > ISA_DMA_MAXADDR) )
	        return xmit_isa_skb_to_dma(nl,skb,elem);
        else
                return xmit_pci_skb_to_dma(nl,skb);
}
									

static int
sg16_start_xmit( struct sk_buff *skb, struct net_device *dev )
{
	struct net_local  *nl = (struct net_local *) dev->priv;
	unsigned long  flags;
	unsigned  cur_tbd;
	void *cpu_addr;


	if ( !netif_carrier_ok(dev) )
		return 1;

	spin_lock_irqsave( &nl->lock, flags );		

	if( nl->tail_xq == ((nl->head_xq - 1) & (XQLEN - 1)) ) {
		netif_stop_queue( dev );
		goto  err_exit;
	}

	cpu_addr=xmit_skb_to_dma(dev,&skb,nl->tail_xq);
	if( !cpu_addr )
                goto  err_exit;

	nl->xq[ nl->tail_xq++ ] = skb;
	nl->tail_xq &= (XQLEN - 1);
	
	cur_tbd = nl->regs->LTDR & 0x7f;
	nl->tbd[ cur_tbd ].address = cpu_to_le32( virt_to_bus( cpu_addr ) );
	nl->tbd[ cur_tbd ].length  = cpu_to_le32( skb->len | LAST_FRAG );
	nl->regs->LTDR = cur_tbd = (cur_tbd + 1) & 0x7f;
			
	/*
	 * Probably, it's the best place to increment statistic counters
	 * though those frames hasn't been actually transferred yet.
	 */
	++nl->in_stats.sent_pkts;
	++nl->stats.tx_packets;
	nl->stats.tx_bytes += skb->len;

	dev->trans_start = jiffies;
	spin_unlock_irqrestore( &nl->lock, flags );
	
	return 0;
err_exit:
				
	spin_unlock_irqrestore( &nl->lock, flags );
	return  1;
}


static void
free_sent_buffers( struct net_device *dev )
{
	struct net_local  *nl = (struct net_local *) dev->priv;

	unsigned  cur_tbd = nl->regs->CTDR;

	spin_lock( &nl->lock );
	if( netif_queue_stopped( dev )  &&  nl->head_tdesc != cur_tbd )
		netif_wake_queue( dev );
	PDEBUG(xmit_debug,"");
	while( nl->head_tdesc != cur_tbd ) {
		/* ...with respect to mapping xq<->descriptors */
		dev_kfree_skb_any( nl->xq[ nl->head_xq++ ] );
		nl->head_xq &= (XQLEN - 1);
		nl->head_tdesc = (nl->head_tdesc + 1) & 0x7f;
	}
	spin_unlock( &nl->lock );
}

/*
 * free_sent_buffers may also be used to drop the queue - just turn
 * the transmitter off, and set CTDR == LTDR
 */

static struct sk_buff*
recv_skb_sync_one(struct net_device *ndev,int len,int elem)
{
        struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);
        struct sk_buff *skb = nl->rq[ elem ];
	PDEBUG(recv_debug,"start,elem=%d",elem);		
        if( (nl->dev_type == sg16isa) &&
                    (virt_to_bus(skb->data)+skb->len > ISA_DMA_MAXADDR) ){
		    PDEBUG(recv_debug,"need to sync");
                    struct dma_buffer *b = (struct dma_buffer *)nl->rbuf + elem;
                    memcpy(skb->data,b->data,len);
        }
        return skb;
}

static void*
recv_skb_to_dma(struct net_device *ndev,struct sk_buff *skb,int elem)
{
        struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);
        struct dma_buffer *b = (struct dma_buffer *)nl->rbuf + elem;
        if( (nl->dev_type == sg16isa) &&
                virt_to_bus(skb->data)+skb->len > ISA_DMA_MAXADDR ){
		PDEBUG(recv_debug,"need to use ext buffer");
                return (void*)b->buff;
	}
	PDEBUG(recv_debug,"use int buffer");	
        return (void*)skb->data;
}
									
static void
alloc_rx_buffers( struct net_device *dev )
{
	struct net_local  *nl = (struct net_local *) dev->priv;
	void *cpu_addr;

	unsigned  cur_rbd = nl->regs->LRDR & 0x7f;

	while( nl->tail_rq != ((nl->head_rq - 1) & (RQLEN - 1)) ) {
		struct sk_buff  *skb = dev_alloc_skb( ETHER_MAX_LEN );
		skb->dev = dev;
		skb_reserve( skb, 2 );	/* align ip on longword boundaries */
		cpu_addr=recv_skb_to_dma(dev,skb,nl->tail_rq);		
		nl->rq[ nl->tail_rq++ ] = skb;
		nl->tail_rq &= (RQLEN - 1);

		nl->rbd[ cur_rbd ].address = cpu_to_le32(
						virt_to_bus( cpu_addr ) );
		nl->rbd[ cur_rbd ].length  = 0;
		nl->regs->LRDR = cur_rbd = (cur_rbd + 1) & 0x7f;
	}
}

static void
indicate_frames( struct net_device *dev )
{
	struct net_local  *nl = (struct net_local *) dev->priv;
	struct sk_buff  *skb;
	unsigned  cur_rbd = nl->regs->CRDR & 0x7f;
	unsigned  len;

	while( nl->head_rdesc != cur_rbd ) {
		len = nl->rbd[ nl->head_rdesc ].length & 0x7ff;
		if( len < ETHER_MIN_LEN )
			len = ETHER_MIN_LEN;
		skb=recv_skb_sync_one(dev,len,nl->head_rq);
		nl->head_rq++;
		nl->head_rq &= (RQLEN - 1);
		
		skb_put( skb, len );
		skb->protocol = eth_type_trans( skb, dev );
		netif_rx( skb );

		++nl->in_stats.rcvd_pkts;
		++nl->stats.rx_packets;
		nl->stats.rx_bytes += len;

		nl->head_rdesc = (nl->head_rdesc + 1) & 0x7f;
	}
}

static void
drop_rq( struct net_local *nl )
{
	while( nl->head_rq != nl->tail_rq ) {
		dev_kfree_skb_any( nl->rq[ nl->head_rq++ ] );
		nl->head_rq &= (RQLEN - 1);
	}
}


static void
activate( struct net_device  *dev )
{
	struct net_local	*nl = (struct net_local *) dev->priv;
	struct timeval		tv;

	nl->regs->SR   = 0xff;		// clear it! 
	nl->regs->CRB &= ~RXDE;
	nl->regs->IMR = EXT | RXS | TXS | OFL | UFL;
	nl->regs->CR  |= TXEN | RXEN;
	++nl->in_stats.attempts;
	do_gettimeofday( &tv );
	nl->in_stats.last_time = tv.tv_sec;
	netif_carrier_on( dev );
}

static void
deactivate( struct net_device  *dev )
{
	struct net_local *nl = (struct net_local *) dev->priv;

	nl->regs->CR  &= ~(RXEN | TXEN);
	nl->regs->CRB |= RXDE;
	nl->regs->IMR  = EXT;
	netif_carrier_off( dev );
}


static void
sg16_tx_timeout( struct net_device  *dev )
{
	struct net_local  *nl = (struct net_local *) dev->priv;

	if( !netif_carrier_ok(dev))
		return;

	printk( KERN_ERR "%s: transmit timeout\n", dev->name );

	if( nl->regs->SR & TXS )
		nl->regs->SR = TXS,
		printk( KERN_ERR "%s: interrupt posted but not delivered\n",
			dev->name );
	free_sent_buffers( dev );
}

/*-----------------------------------------------------------------------------------
 * Control related procfs functions
 *-----------------------------------------------------------------------------------*/

struct dev_entrie{
    char *name;
    int mark;
};
struct dev_entrie entries[]={ 
    { "rate", PFS_SHDSL_RATE},
    { "master", PFS_SHDSL_MASTER},
    { "remcfg", PFS_SHDSL_CFG },
    { "annex", PFS_SHDSL_ANNEX	},
    { "mod", PFS_SHDSL_MOD },
    { "autobaud", PFS_SHDSL_AUTOB },
    { "crc16", PFS_HDLC_CRC16 },
    { "fill_7e", PFS_HDLC_FILL7E },
    { "inv", PFS_HDLC_INV },
    { "rburst", PFS_HDLC_RBURST },
    { "wburst", PFS_HDLC_WBURST },
    { "fwloaded", PFS_STATS_FWOK },
    { "state", PFS_STATS_ONLSTAT },
    { "statistic", PFS_STATS_STATISTIC },
    { "maddr", PFS_NET_MACADDR }, 
    { "bus", PFS_BUS_TYPE },
    { "hdlc_regs", PFS_DEBUG_HREGS }
};

struct ent_data{
    struct net_device *ndev;
    int mark;
};

static int
set_entry(struct proc_dir_entry *ent,read_proc_t *fread,
	    write_proc_t *fwrite,int mark,struct net_device *ndev )
{
    struct ent_data *entdata;
    
    if( !(entdata=(struct ent_data *)kmalloc( sizeof( struct ent_data ),GFP_KERNEL ) ) )
	return -1;
    entdata->ndev=ndev;
    entdata->mark=mark;
    ent->data=(void *)entdata;
    ent->owner=THIS_MODULE;
    ent->read_proc=fread;
    ent->write_proc=fwrite;
    return 0;
}

static int
init_sg16_in_procfs( struct net_device *dev )
{
    struct net_local *nl=(struct net_local *)dev->priv;
    struct proc_dir_entry *dev_ent;
    int i,j=0;    
    
    if( sg16_entry==NULL )
	return -1;

    // create device dir
    if( !(dev_ent=proc_mkdir(dev->name,sg16_entry)) )
	return -1;
    
    // create shdsl ctrl entrys
    for(i=0;i< PFS_SHDSL_ENTS;i++){
	if( !(nl->ents[i]=create_proc_entry(entries[i].name,(S_IRUSR|S_IWUSR),dev_ent) ) )
	    goto err1;
	if( set_entry(nl->ents[i],show_shdsl,store_shdsl,entries[i].mark,dev) )
	    goto err1;
    }
    j=i;
    // create hdlc ctrl entrys
    for(i=0;i<PFS_HDLC_ENTS;i++){
	if( !(nl->ents[j+i]=create_proc_entry(entries[j+i].name,(S_IRUSR|S_IWUSR),dev_ent) ) )
	    goto err1;
	if( set_entry(nl->ents[j+i],show_hdlc,store_hdlc,entries[j+i].mark,dev) )
	    goto err1;
    }
    
    j+=i;
    // create stat entrys ( online status & firmware load status )
    for(i=0;i<PFS_STATS_ENTS-1;i++){
	if( !(nl->ents[j+i]=create_proc_entry(entries[j+i].name,(S_IRUSR|S_IWUSR),dev_ent) ) )
	    goto err1;
	if( set_entry(nl->ents[j+i],show_stats,NULL,entries[j+i].mark,dev) )
	    goto err1;
    }
    
    j+=i; i=0;
    
    // create stat entry ( driver statistic )
    if( !(nl->ents[j+i]=create_proc_entry(entries[j+i].name,(S_IRUSR|S_IWUSR),dev_ent) ) )
        goto err1;
    if( set_entry(nl->ents[j+i],show_stats,store_stats,entries[j+i].mark,dev) )
        goto err1;
    j++;
    // mac addres set entry
    if( !(nl->ents[j+i]=create_proc_entry(entries[j+i].name,(S_IRUSR|S_IWUSR),dev_ent) ) )
        goto err1;
    if( set_entry(nl->ents[j+i],NULL,store_maddr,entries[j+i].mark,dev) )
	goto err1;
    j++;
    // bus type set entry
    if( !(nl->ents[j+i]=create_proc_entry(entries[j+i].name,S_IRUSR,dev_ent) ) )
        goto err1;
    if( set_entry(nl->ents[j+i],show_bus,NULL,entries[j+i].mark,dev) )
	goto err1;
    j++;
    // debug set entry
    if( !(nl->ents[j+i]=create_proc_entry(entries[j+i].name,S_IRUSR,dev_ent) ) )
        goto err1;
    if( set_entry(nl->ents[j+i],show_hdlc_regs,NULL,entries[j+i].mark,dev) )
	goto err1;
        
    return 0;

err1:
    for(j=0;j<=i;j++)
	if( nl->ents[j]->data )
	    kfree(nl->ents[j]->data);
    
    for(j=0;j<=i;j++)
	remove_proc_entry(entries[j].name,dev_ent);    
    remove_proc_entry(dev->name,sg16_entry);
    return -1;
}

static void
del_sg16_from_procfs( struct net_device *dev )
{
    struct net_local *nl=(struct net_local *)dev->priv;
    int j,i=PFS_ENTS;
    int len;
    char ent_path[50];
    
    for(j=0;j<i;j++)
	kfree(nl->ents[j]->data);
    PDEBUG(0,"kfree - ok");
    strcpy(ent_path,dev->name);
    len=strlen(ent_path);
    for(j=0;j<i;j++){
	strcat(ent_path,entries[j].name);
	remove_proc_entry(ent_path,sg16_entry);    
	ent_path[len]=0;
    }
    PDEBUG(0,"remove_proc_entry - ok");    
    remove_proc_entry(dev->name,sg16_entry);
    PDEBUG(0,"all - ok");        
}

//---- shdsl attributes ----//
static int
show_shdsl(char *page,char **start,off_t off,int count,int *eof,void *data)
{
    struct ent_data *ent=(struct ent_data *)data;
    struct net_device *ndev=ent->ndev;
    struct net_local *nl=(struct net_local *)ndev->priv;
    struct cx28975_cfg *cfg=(struct cx28975_cfg *)&(nl->cfg);
    char  *modstr[] = {"TCPAM32", "TCPAM16", "TCPAM8", "TCPAM4"};
    int len=0;
    u16 tmp=0;
    u8 t=0;
    
    MOD_INC_USE_COUNT;
    
    switch( ent->mark ){
//---- Rate show----------------------------------------------
    case PFS_SHDSL_RATE:
        if( !cfg->autob && !( cfg->remcfg && !cfg->master ) ){
	    tmp=cfg->lrate << 3;
	    len=snprintf(page,PAGE_SIZE,"%u of %u\n",tmp,tmp);
	}else{
	    if( cfg->autob  && netif_carrier_ok(ndev) ){
		if( !issue_cx28975_cmd(nl,_DSL_GHS_GET_FINAL_RATE,&t,0) )
    		{
    		    tmp=*( (u8 *)(nl->cmdp->out_data+2));
	    	    tmp=(tmp<<3)+ *( (u8 *)(nl->cmdp->out_data+3) );
		}
		len=snprintf(page,PAGE_SIZE,"%u of %u\n",tmp,(cfg->lrate)<<3);
	    }else if( cfg->remcfg && netif_carrier_ok(ndev) ){
		t=_DSL_DATA_RATE;
		if( !issue_cx28975_cmd(nl,_DSL_READ_CONTROL,&t,0) )
		{
		    tmp=*( (u8 *)(nl->cmdp->out_data+1)) & 0x3;
		    tmp=(tmp<<8)+*( (u8 *)(nl->cmdp->out_data) );
    		    tmp--;
		}

	    }
	    len=snprintf(page,PAGE_SIZE,"%u of %u\n",tmp<<3,tmp<<3);	    
	}																											    	
	break;
//---- Modem role show----------------------------------------------
    case PFS_SHDSL_MASTER:
	if( cfg->master )
	    len=snprintf(page,PAGE_SIZE,"master\n");
	else
	    len=snprintf(page,PAGE_SIZE,"slave\n");
	break;
//---- cfg show ----------------------------------------------------			    
    case PFS_SHDSL_CFG:
	if( cfg->remcfg )
    	    len=snprintf(page,PAGE_SIZE,"preact\n");
	else
    	    len=snprintf(page,PAGE_SIZE,"local\n");
	break;				
//---- annex show---------------------------------------------------
    case PFS_SHDSL_ANNEX:
	t=cfg->annex;
	switch( t ){
        case ANNEX_A:
	    len=snprintf(page,PAGE_SIZE,"Annex=A\n");
	    break;
        case ANNEX_B:
    	    len=snprintf(page,PAGE_SIZE,"Annex=B\n");
    	    break;
	case ANNEX_F:
    	    len=snprintf(page,PAGE_SIZE,"Annex=F\n");
	    break;
	default:
    	    cfg->annex=ANNEX_A;
	    len=snprintf(page,PAGE_SIZE,"Annex=A\n");
	}
	break;
//---- code show---------------------------------------------------	
    case PFS_SHDSL_MOD:

	if( cfg->remcfg )
    	    len=snprintf(page,PAGE_SIZE,"Unknown\n");
	else
	    len=snprintf(page,PAGE_SIZE,"%s\n",modstr[cfg->mod]);
	break;
//---- autorate selection -------------------------------------------
    case PFS_SHDSL_AUTOB:
	if( cfg->autob )
	    len=snprintf(page,PAGE_SIZE,"auto\n");
	else if( cfg->remcfg && !cfg->master )
	    len=snprintf(page,PAGE_SIZE,"remote\n");
	else
	    len=snprintf(page,PAGE_SIZE,"static\n");
	break;
//---- default ------------------------------------------------------
    default:
	len=0;
    }
    MOD_DEC_USE_COUNT;    
    return len;
}


static int
store_shdsl(struct file *file,const char *buffer,unsigned long count,void *data)
{
    char buff[PAGE_SIZE];
    struct ent_data *ent=(struct ent_data *)data;
    struct net_device *ndev=ent->ndev;
    struct net_local *nl=(struct net_local *)ndev->priv;
    struct cx28975_cfg *cfg=(struct cx28975_cfg *)&(nl->cfg);
    int len=count;
    u16 tmp=0;
    char *endp;
    
    MOD_INC_USE_COUNT;    

    if( !count || !data || (ndev->flags & IFF_UP) )
	goto ext;

    len= (count>PAGE_SIZE) ? PAGE_SIZE : count;
    if( copy_from_user(buff,buffer,len) )
	goto ext;

    switch( ent->mark ){
//---- Rate show----------------------------------------------
    case PFS_SHDSL_RATE:
	tmp=simple_strtoul( buff,&endp,0);
	if( !tmp )
	    goto ext;
	cfg->lrate=(tmp >> 3) & 0x3ff;
	break;
//---- Modem role show----------------------------------------------
    case PFS_SHDSL_MASTER:
        if( buff[0]=='0' )
            cfg->master=0;
        else if( buff[0]=='1' )
	    cfg->master=1;
	break;
//---- cfg show ----------------------------------------------------			    
    case PFS_SHDSL_CFG:
	if( buff[0]=='0' )
	    cfg->remcfg=0;
	else if( buff[0]=='1' )
	    cfg->remcfg=1;
	break;				
//---- annex show---------------------------------------------------
    case PFS_SHDSL_ANNEX:
	tmp=buff[0];
        switch(tmp){
	case '0':
    	    cfg->annex=ANNEX_A;
	    break;
	case '1':
	    cfg->annex=ANNEX_B;
	    break;
        case '2':
	    cfg->annex=ANNEX_F;
            break;
	default:
	    cfg->annex=ANNEX_A;
            break;
	}
	break;
//---- code show---------------------------------------------------	
    case PFS_SHDSL_MOD:
	tmp=buff[0];
	switch(tmp){
	case '0':
	    cfg->mod=0;
	    break;
	case '1':
    	    cfg->mod=1;
            break;
	case '2':
    	    cfg->mod=2;
	    break;
	case '3':
	    cfg->mod=3;
    	    break;
	}
	break;
//---- autorate selection -------------------------------------------
    case PFS_SHDSL_AUTOB:
	/* check that autobaud is aviable in this firmware */
        if( !cfg->autob_en ){
	    cfg->autob=0;
	    goto ext;
	}
	/* change auto rate state */
	tmp=buff[0];
	switch(tmp){
	case '0':
	    cfg->autob=0;
	    break;
	case '1':
	    cfg->autob=1;
	    break;
	}
	break;
//---- default ------------------------------------------------------
    }

ext:
    MOD_DEC_USE_COUNT;    
    return len;
}

//---- hdlc attributes ----//
static int
show_hdlc(char *page,char **start,off_t off,int count,int *eof,void *data)
{
    struct ent_data *ent=(struct ent_data *)data;
    struct net_device *ndev=ent->ndev;
    struct net_local *nl=(struct net_local *)ndev->priv;
    struct cx28975_cfg *cfg=(struct cx28975_cfg *)&(nl->cfg);
    int len=0;
    
    MOD_INC_USE_COUNT;
    
    switch( ent->mark ){
//---- CRC show -----------------------------------------------------------    
    case PFS_HDLC_CRC16:
	if( cfg->crc16 )
	    len=snprintf(page,PAGE_SIZE,"crc16\n");
	else
	    len=snprintf(page,PAGE_SIZE,"crc32\n");	
	break;
//---- channel fill show --------------------------------------------------    	
    case PFS_HDLC_FILL7E:
	if( cfg->fill_7e )
	    len=snprintf(page,PAGE_SIZE,"0x7E\n");
	else
	    len=snprintf(page,PAGE_SIZE,"0xFF\n");	
	break;
//---- Data Inversion type show --------------------------------------------    		
    case PFS_HDLC_INV:
	if( cfg->inv )
	    len=snprintf(page,PAGE_SIZE,"on\n");
	else
	    len=snprintf(page,PAGE_SIZE,"off\n");	
	break;
//---- rburst mode show -----------------------------------------------------    		
    case PFS_HDLC_RBURST:
	if( cfg->rburst )
	    len=snprintf(page,PAGE_SIZE,"on\n");
	else
	    len=snprintf(page,PAGE_SIZE,"off\n");	
	break;
//----wburst mode show -----------------------------------------------------    	
    case PFS_HDLC_WBURST:
	if( cfg->wburst )
	    len=snprintf(page,PAGE_SIZE,"on\n");
	else
	    len=snprintf(page,PAGE_SIZE,"off\n");	
	break;
    }
    
    MOD_DEC_USE_COUNT;    
    return len;

}

static int
store_hdlc(struct file *file,const char *buffer,unsigned long count,void *data)
{
    struct ent_data *ent=(struct ent_data *)data;
    struct net_device *ndev=ent->ndev;
    struct net_local *nl=(struct net_local *)ndev->priv;
    struct cx28975_cfg *cfg=(struct cx28975_cfg *)&(nl->cfg);
    char buff[PAGE_SIZE];
    int len=count;
    u8 flag=0;
    
    
    MOD_INC_USE_COUNT;    

    if( !count || !data || (ndev->flags & IFF_UP) )
	goto ext;
    len= (count>PAGE_SIZE) ? PAGE_SIZE : count;
    if( copy_from_user(buff,buffer,len) )
	goto ext;

    switch(buff[0]){
	case '1':
	    flag=1;    break;
	case '0':
	    flag=0;    break;
	default:
	    goto ext;
    }
    
    switch( ent->mark ){
    case PFS_HDLC_CRC16:
	cfg->crc16=flag;
	if( flag )
	    nl->regs->CR |= CMOD;
	else
	    nl->regs->CR &= ~CMOD;	    
	break;
    case PFS_HDLC_FILL7E:
	cfg->fill_7e=flag;
	if( flag )
	    nl->regs->CR |= FMOD;
	else
	    nl->regs->CR &= ~FMOD;	    
	break;
    case PFS_HDLC_INV:
	cfg->inv=flag;
	if( flag )
	    nl->regs->CR |= PMOD;
	else
	    nl->regs->CR &= ~PMOD;	    
	break;
    case PFS_HDLC_RBURST:
	cfg->rburst=flag;
	if( flag )
	    nl->regs->CRB |= RDBE;
	else
	    nl->regs->CRB &= ~RDBE;	    
	break;
    case PFS_HDLC_WBURST:    
	cfg->wburst=flag;
	if( flag )
	    nl->regs->CRB |= WTBE;
	else
	    nl->regs->CRB &= ~WTBE;	    
	break;
    }
ext:
    MOD_DEC_USE_COUNT;    
    return len;

}


//---- stats ----//
static int
show_stats(char *page,char **start,off_t off,int count,int *eof,void *data)
{
    struct ent_data *ent=(struct ent_data *)data;
    struct net_device *ndev=ent->ndev;
    struct net_local *nl=(struct net_local *)ndev->priv;
    volatile struct cx28975_cmdarea  *p = nl->cmdp;    
    int len=0;
    u8 stat_1,stat_3;
    char ret_ad[30];
    struct sg16_stats *sb_stat=&(nl->in_stats);
    struct dsl_stats ds;
    
    MOD_INC_USE_COUNT;
    
    switch( ent->mark ){
    case PFS_STATS_FWOK:
	if( nl->state!=NOT_LOADED )
	    len=snprintf(page,PAGE_SIZE,"1\n");
	else
	    len=snprintf(page,PAGE_SIZE,"0\n");
	break;
    case PFS_STATS_ONLSTAT:
	stat_1=*((u8*)p+0x3c0);
	stat_3=*((u8*)p+0x3c2);
	switch( (stat_1 >> 6) ) {
	case  0 :
    	    *ret_ad=0;
	    switch( stat_3 & 0x0f ) {
    	    case  1 :
		snprintf(ret_ad,30,"(Bad NMR)" );
    		break;
	    case  2 :
    		snprintf(ret_ad,30, "(Frequency lock failed)" );
        	break;
	    case  3 :
		snprintf(ret_ad,30, "(Pre-activation failed)" );
    		break;
    	    case  4 :
		snprintf(ret_ad,30, "(Sync word detect failed)" );
                break;
	        }
    	    len=snprintf(page,PAGE_SIZE,"not ready %s\n",ret_ad);
	    break;
	case  1 :
    	    len=snprintf(page,PAGE_SIZE,"online\n");
	    break;
	default:
	    len=snprintf(page,PAGE_SIZE,"offline\n");
	    break;
	}
        break;
    case PFS_STATS_STATISTIC:
	if( shdsl_get_stat(nl,&ds) )
	    len=snprintf(page,PAGE_SIZE,"err");
	else
	    len=snprintf(page,PAGE_SIZE,
        	"%u %u %u %u  %u %u %u %u  %u %u",
    		sb_stat->sent_pkts,sb_stat->rcvd_pkts,
                sb_stat->crc_errs,ds.atmpt,ds.all_atmpt,
		ds.losw,ds.crc, ds.attenuat,ds.nmr,
		sb_stat->last_time );
	break;																	
    }
    
    MOD_DEC_USE_COUNT;    
    return len;
}

static int
store_stats(struct file *file,const char *buffer,unsigned long count,void *data)
{
    struct ent_data *ent=(struct ent_data *)data;
    struct net_device *ndev=ent->ndev;
    struct net_local *nl=(struct net_local *)ndev->priv;
    char buff[PAGE_SIZE];
    int len=count;
    
    MOD_INC_USE_COUNT;    

    if( !count || !data || (ndev->flags & IFF_UP) )
	goto ext;
    len= (count>PAGE_SIZE) ? PAGE_SIZE : count;
    if( copy_from_user(buff,buffer,len) )
	goto ext;

    if( buff[0]=='1')
        shdsl_clr_stat(nl);

    return len;
ext:
    MOD_DEC_USE_COUNT;    
    return len;

}		
//---- MAC addtess set ----//				

static int
store_maddr(struct file *file,const char *buffer,unsigned long count,void *data)
{
    struct ent_data *ent=(struct ent_data *)data;
    struct net_device *ndev=ent->ndev;
    char buff[PAGE_SIZE];
    int len=count;
    u16 tmp;
    char *endp;
    
    MOD_INC_USE_COUNT;    

    if( !count || !data || (ndev->flags & IFF_UP) )
	goto ext;
    len= (count>PAGE_SIZE) ? PAGE_SIZE : count;
    if( copy_from_user(buff,buffer,len) )
	goto ext;

    tmp=simple_strtoul( buff,&endp,16) & 0xfff;
    *(u16 *)ndev->dev_addr = htons( 0x00ff ),
    *(u32 *)(ndev->dev_addr + 2) = htonl( 0x014aa000 | tmp );
ext:
    MOD_DEC_USE_COUNT;    
    return len;
}		

static int
show_bus(char *page,char **start,off_t off,int count,int *eof,void *data)
{
    struct ent_data *ent=(struct ent_data *)data;
    struct net_device *ndev=ent->ndev;
    struct net_local *nl=(struct net_local *)ndev->priv;
    int len=0;
    
    MOD_INC_USE_COUNT;
    
    if( nl->dev_type == sg16pci )
	    len=snprintf(page,PAGE_SIZE,"PCI" );
    else if( nl->dev_type == sg16isa )
	    len=snprintf(page,PAGE_SIZE,"ISA" );    
    
    MOD_DEC_USE_COUNT;    
    return len;
}

// debug
static int
show_hdlc_regs(char *page,char **start,off_t off,int count,int *eof,void *data)
{
    struct ent_data *ent=(struct ent_data *)data;
    struct net_device *ndev=ent->ndev;
    struct net_local *nl=(struct net_local *)ndev->priv;
    int len=0;
    
    MOD_INC_USE_COUNT;
    
    len=snprintf(page,PAGE_SIZE,"CRA=%02x CRB=%02x SR=%02x IMR=%02x\nCRDR=%02x LRDR=%02x CTDR=%02x LTDR=%02x",
		nl->regs->CR,nl->regs->CRB,nl->regs->SR,nl->regs->IMR,
		nl->regs->CRDR,nl->regs->LRDR,nl->regs->CTDR,nl->regs->LTDR);

    MOD_DEC_USE_COUNT;    
    return len;
}



   