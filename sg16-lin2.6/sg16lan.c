/* sg16lan.c:  Sigrand SG-16PCI SHDSL modem driver for linux (kernel 2.6.x)
 *
 *	Written 2005-2006 by Artem U. Polyakov <art@sigrand.ru>
 *
 *	This driver presents SG-16PCI modem 
 *	to system as common ethernet-like netcard.
 *
 *	This software may be used and distributed according to the terms
 *	of the GNU General Public License.
 *
 *
 *	10.11.2005	initial revision of Granch SBNI16 modem driver v1.0 
 *                      wtitten by Denis I. Timofeev - Artem U. Polyakov <art@sigrand.ru>
 *	11.12.2005	Version 1.0 (sysfs, firmware hotplug) - Artem U. Polyakov <art@sigrand.ru>
 *	05.11.2006	Version 2.0 (ISA adapter support) - Artem U. Polyakov <art@sigrand.ru>
 *	02.07.2007	Version 2.1 (port to 2.6.19 changes) - Artem U. Polyakov <art@sigrand.ru>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/vermagic.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#	include <linux/config.h>
#endif


#include <asm/io.h>
#include <asm/types.h>
#include <asm/byteorder.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/uaccess.h>

#include <linux/ptrace.h>
#include <linux/fcntl.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>

#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/random.h>
#include <linux/firmware.h>

#include <linux/pci.h>
#include <linux/pnp.h>
#include <linux/isapnp.h>

#include <linux/sched.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <net/arp.h>

#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/dma-mapping.h>

#include "cx28975.h"
#include "sg16lan.h"
// Debug parameters
//#define DEBUG_ON
#define DEFAULT_LEV 20
#include "sg16debug.h"


MODULE_DESCRIPTION( "Sigrand SG-16PCI,SG-16ISA driver Version 2.0\n" );
MODULE_AUTHOR( "Maintainer: Artem U. Polyakov art@sigrand.ru\n" );
MODULE_LICENSE( "GPL" );
MODULE_VERSION("2.0");

/* --------------------------------------------------------------------------
 *      Module initialisation/cleanup
 * -------------------------------------------------------------------------- */

int
sg16_init( void ){
    
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	pci_module_init( &sg16_driver );
#else 
	pci_register_driver( &sg16_driver );
#endif
	pnp_register_driver(&sg16_isapnp_driver);	
	return 0;
}

void
sg16_exit( void ){
	pci_unregister_driver( &sg16_driver );
	pnp_unregister_driver(&sg16_isapnp_driver);
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
	struct net_local * nl=NULL;
	u8 err;

	if( pci_enable_device( pdev ) )
		return  -EIO;
	pci_set_master(pdev);
        
	// allocate network device 
	if( !( ndev = alloc_netdev( sizeof(struct net_local),"dsl%d",dsl_init)) )
		return  -ENOMEM;
	// set pci device data field
	pci_set_drvdata( pdev, ndev );
	// set some net device fields
	ndev->mem_start = pci_resource_start( pdev, 1 );
	ndev->mem_end = pci_resource_end( pdev,1 );
	ndev->irq = pdev->irq;
	// device private data initialisation
	nl=(struct net_local *)netdev_priv( ndev);
	memset( nl, 0, sizeof(struct net_local) );	
	nl->dev=&(pdev->dev);
	nl->dev_type = sg16pci;	

	// network interface registration
	if( register_netdev( ndev ) ) {
		err=ENODEV;
		goto err1;
	}
	return  0;
 err1:	    
	free_netdev(ndev);    
	return err;
}

static void __devexit
sg16_pci_remove_one( struct pci_dev  *pdev )
{
	struct net_device  *ndev = pci_get_drvdata( pdev );
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);    
	struct device *dev=nl->dev;
	struct device_driver *drv=dev->driver;

	// timer entry 
	del_timer_sync( &(nl->link_state) );

	// shutdown device
	hdlc_shutdown(nl);
    
	// Remove symlink on device from driver dir in sysfs */
	sysfs_remove_link(&(drv->kobj),ndev->name);
	// Remove sysfs entires 
	del_sg16_from_sysfs(dev);

	// Remove network device from OS
	unregister_netdev( ndev );
	free_irq( ndev->irq, ndev );
	release_mem_region( ndev->mem_start, SG16_IOMEM_LEN );
	iounmap( nl->mem_base );
	free_netdev( ndev );
}

/* --------------------------------------------------------------------------
 *      ISA adapter initialisation/cleanup
 * -------------------------------------------------------------------------- */



static int
sg16_isapnp_probe_one(struct pnp_dev *idev,const struct pnp_device_id *dev_id)
{
	struct net_device *ndev;
	struct net_local *nl;
	int error=-ENODEV;
	int i;
	PDEBUG(8,"found idev=%08x",(long)idev);
	// check device
	if( !pnp_irq_valid(idev, 0) ){
		PDEBUG(0,"irq not valid");
		goto fail1;
	}
	PDEBUG(8,"irq valid. irq#=%d, irqflags=%08x",pnp_irq(idev,0),pnp_irq_flags(idev,0));		

	if( !pnp_mem_valid(idev,0) ){
		PDEBUG(0,"iomem not valid");
		goto fail1;
	}
	if( pnp_mem_len(idev,0) <  SG16_IOMEM_LEN ){
		PDEBUG(0,"iomem length = %d less than %d",
		       (int)pnp_mem_len(idev,1),SG16_IOMEM_LEN);
		goto fail1;
	}
	// TODO: chk flags: (IORESOURCE_MEM_WRITEABLE | IORESOURCE_MEM_16BIT | (more??) )
	PDEBUG(8,"iomem valid. memstart=%08x,memflags=%08x",pnp_mem_start(idev,0),pnp_mem_flags(idev,0));				

	if( !pnp_dma_valid(idev,0) ){
		PDEBUG(0,"dma not valid");
		goto fail1;
	}
	// (??) IORESOURCE_DMA_16BIT | IORESOURCE_DMA_MASTER | IORESOURCE_DMA_WORD | IORESOURCE_DMA_TYPEB
	if( (pnp_dma_flags(idev,0)&(IORESOURCE_DMA_16BIT | IORESOURCE_DMA_MASTER)) !=
	    (IORESOURCE_DMA_16BIT | IORESOURCE_DMA_MASTER) ){
		PDEBUG(0,"bad dma flags: %08x",pnp_dma_flags(idev,0));
		goto fail1;
	}
	PDEBUG(8,"dma valid. dma#=%d, flags=%08x",pnp_dma(idev,0),pnp_dma_flags(idev,0));				
	// allocate net_device
	if( !( ndev = alloc_netdev( sizeof(struct net_local),"dsl%d",dsl_init)) ){
		PDEBUG(0,"Fail to allocate memory for private data");
		error=-ENOMEM;
		goto fail1;
	}
	PDEBUG(0,"memory for private data allocated");
	// init private data depends from isapnp subsystem
	nl=(struct net_local *)netdev_priv( ndev);
	memset( nl, 0, sizeof(struct net_local) );
	nl->dev=(struct device*)&(idev->dev);
	nl->dev_type = sg16isa;
	// allocate ISA DMA able buffers
	nl->rbuf=kmalloc(RQLEN*(sizeof(struct dma_buffer)+10),GFP_DMA | GFP_KERNEL);			
	for(i=0;i<RQLEN;i++)
		nl->rbuf[i].data=(u8*)&(nl->rbuf[i].buff)+2;
	nl->xbuf=kmalloc(XQLEN*(sizeof(struct dma_buffer)+10),GFP_DMA | GFP_KERNEL);			
	for(i=0;i<XQLEN;i++)
		nl->xbuf[i].data=(u8*)&(nl->xbuf[i].buff)+2;
		
	// init netdevice data depends from isapnp subsystem
	pnp_set_drvdata(idev,(void*)ndev);
	ndev->mem_start = pnp_mem_start(idev,0);
	ndev->mem_end = pnp_mem_end(idev,0);
	ndev->irq = pnp_irq(idev, 0);
	ndev->dma = pnp_dma(idev,0);

	PDEBUG(0,"start registering netdev");
	// network interface initialisation
	if( (error=register_netdev(ndev) ) ) {
		PDEBUG(0,"Fail to register network device");
		goto fail2;
	}
	PDEBUG(0,"end");	
	return 0;	
 fail2:
	free_netdev(ndev);
 fail1:
	pnp_device_detach( idev );
	return error;
}
static void
sg16_isapnp_remove_one(struct pnp_dev *idev)
{
	struct net_device *ndev;
	struct net_local *nl;
	struct device_driver *drv;

	// get current isapnp net device	
	ndev=(struct net_device*)pnp_get_drvdata(idev);
	nl=(struct net_local *)netdev_priv(ndev);
	drv=(struct device_driver*)nl->dev->driver;
	// timer entry 
	del_timer_sync( &(nl->link_state) );
	// shutdown device 
	hdlc_shutdown(nl);
	// Remove sysfs entires 
	del_sg16_from_sysfs(nl->dev);
	sysfs_remove_link(&(drv->kobj),ndev->name);
	// Remove network device from OS 
	kfree(nl->rbuf);
	kfree(nl->xbuf);	
	disable_dma(ndev->dma);
	free_dma(ndev->dma);
	free_irq( ndev->irq, ndev );
	iounmap( nl->mem_base );
	release_mem_region( ndev->mem_start, SG16_IOMEM_LEN );
	unregister_netdev( ndev );
	free_netdev( ndev );
	pnp_device_detach( idev );
}

/* --------------------------------------------------------------------------
 *   Network device functions   
 * -------------------------------------------------------------------------- */

static int __init
sg16_probe( struct net_device  *ndev )
{
	struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);
	struct device *dev = nl->dev;
	struct device_driver *drv = dev->driver;    
	int err=-ENODEV;
	u8 t;

	// Carrier off
	netif_carrier_off( ndev );
	netif_stop_queue(ndev);
	PDEBUG(8,"start");	
    
	// generate 'unique' MAC address
	*(u16 *)ndev->dev_addr = htons( 0x00ff );
	*(u32 *)(ndev->dev_addr + 2) = htonl( 0x01a39000 | ((u32)ndev->priv & 0x00000fff) );

	// Init net device handler functions 
	ndev->open = &sg16_open;
	ndev->stop = &sg16_close;
	ndev->hard_start_xmit = &sg16_start_xmit;
	ndev->get_stats	= &sg16_get_stats;
	ndev->set_multicast_list= &set_multicast_list;
	ndev->tx_timeout = &sg16_tx_timeout;
	ndev->watchdog_timeo = TX_TIMEOUT;

	PDEBUG(0,"start registering mem region");
	if( !request_mem_region( ndev->mem_start, 0x1000, ndev->name ) )
		return  -ENODEV;
	PDEBUG(8,"request_mem_region - ok");	
	
	// set network device private data 
	spin_lock_init( &nl->rlock );
	spin_lock_init( &nl->xlock );
	init_waitqueue_head( &nl->wait_for_intr );
	nl->mem_base = (void *) ioremap( ndev->mem_start, SG16_IOMEM_LEN );
	
	nl->tbd  = (struct hw_descr *) nl->mem_base + SG16_TBD_OFFSET;
	nl->rbd  = (struct hw_descr *) ((u8 *)nl->mem_base + SG16_RBD_OFFSET);
	nl->regs = (struct sg16_hw_regs *) ((u8 *)nl->mem_base + SG16_REGS_OFFSET);
	nl->cmdp = (struct cx28975_cmdarea *) ((u8 *)nl->mem_base + SG16_CMDP_OFFSET);

	PDEBUG(0,"start registering irq");
	if( request_irq(ndev->irq, sg16_interrupt, SA_SHIRQ, ndev->name, ndev) ){
		printk( KERN_ERR "%s: unable to get IRQ %d.\n",
			ndev->name, ndev->irq );
		goto err_exit;
	}

	PDEBUG(8,"request_irq - ok");		
	
	if(nl->dev_type == sg16isa){
		if( (err=request_dma(ndev->dma,"sg16lan")) ){
			PDEBUG(0,"dma not registered, error=%d",err);
			goto err_exit1;
		}
		set_dma_mode(ndev->dma, DMA_MODE_CASCADE);	
		enable_dma(ndev->dma);
	}
	
	if( nl->dev_type == sg16pci )	
		printk( KERN_NOTICE "%s: Sigrand SG-16PCI SHDSL (irq %d, mem %#lx)\n",
			ndev->name, ndev->irq, ndev->mem_start );
	else 
		printk( KERN_NOTICE "%s: Sigrand SG-16ISA SHDSL (irq %d, mem %#lx, dma %d)\n",
			ndev->name, ndev->irq, ndev->mem_start, ndev->dma );
		
	SET_MODULE_OWNER( ndev );

	// shutdown device before startup
	hdlc_shutdown(nl);

	// Create symlink from driver dir to device dir in sysfs 
	sysfs_create_link( &(drv->kobj),&(dev->kobj),ndev->name );

	// Create sysfs entires 
	if( init_sg16_in_sysfs( dev ) ){
		printk( KERN_ERR "%s: unable to create sysfs entires\n",ndev->name);
		err=-EPERM;	    
		goto err_exit2;
	}
	// timered link chk entire
	nl->link_state.data = ( unsigned long )ndev;
	nl->link_state.function = shdsl_link_chk;
	init_timer( &(nl->link_state) );

	nl->shdsl_cfg.need_preact=1;
#ifndef AUTOSTART_OFF
	if( shdsl_dload_fw(dev)  )
		printk(KERN_NOTICE"%s: cannot download firmware\n",ndev->name);
	else{
		// check that autobaud is aviable in this firmware 
		if( shdsl_preactivation(nl) )
			printk(KERN_ERR"%s, I/O error\n",ndev->name);
		else{
			// Starting device activation 
			t=0x42;
			if( (err=shdsl_issue_cmd(nl,_DSL_ACTIVATION,&t,1)) )
				goto err_exit2;
			nl->shdsl_cfg.need_preact=0;
		}
	}
#endif	

	return  0;
 err_exit2:    
	if(nl->dev_type == sg16isa)
		free_dma(ndev->dma);
 err_exit1:
	free_irq( ndev->irq, ndev );    
 err_exit:
	iounmap( nl->mem_base );
	release_mem_region( ndev->mem_start, 0x1000 );
	return  err;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t
sg16_interrupt( int  irq,  void  *dev_id,  struct pt_regs  *regs )
#else
	static irqreturn_t
	sg16_interrupt( int  irq,  void  *dev_id)
#endif
{
	struct net_device *dev = (struct net_device *) dev_id;
	struct net_local  *nl  = (struct net_local *)netdev_priv(dev);		
	u8 status = ioread8((iotype)&(nl->regs->SR));
	u8 mask;

	if( status == 0 )
		return IRQ_NONE;

	mask = ioread8((iotype)&(nl->regs->IMR));
	iowrite8( 0, (iotype)&(nl->regs->IMR));
	status= (status & mask);

	if( status & EXT )
		{
			shdsl_interrupt( dev ),
				iowrite8( EXT, (iotype)&(nl->regs->SR));
		}
	//  Whether transmit error is occured, we have to re-enable the
	//  transmitter. That's enough, because linux doesn't fragment
	//  packets.

	if( status & UFL )
		{
			PDEBUG(15"%s: UFL, CRA=%02x\n",dev->name,ioread8((iotype)&(nl->regs->CRA)));	
			iowrite8( UFL,(iotype)&(nl->regs->SR));				
			iowrite8( ioread8((iotype)&(nl->regs->CRA)) | TXEN,
				  (iotype)&(nl->regs->CRA) );
			++nl->in_stats.ufl_errs;
			++nl->stats.tx_errors;
			++nl->stats.tx_fifo_errors;
		}
	if( status & RXS )
		{
			if( spin_trylock( &(nl->rlock) ) )
				{
					recv_init_frames( dev );
					recv_alloc_buffs( dev );
					spin_unlock( &(nl->rlock) );
				}
			iowrite8( RXS,(iotype)&(nl->regs->SR));
		}
	if( status & TXS )
		{
			xmit_free_buffs( dev );
			iowrite8( TXS,(iotype)&(nl->regs->SR));		
		}
	if( status & CRC )
		{
			++nl->in_stats.crc_errs;
			++nl->stats.rx_errors;
			++nl->stats.rx_crc_errors;
			iowrite8( CRC,(iotype)&(nl->regs->SR));				
		}
	if( status & OFL )
		{
			++nl->in_stats.ofl_errs;
			++nl->stats.rx_errors;
			++nl->stats.rx_over_errors;
			iowrite8( OFL,(iotype)&(nl->regs->SR));
		}
	
	iowrite8( mask,(iotype)&(nl->regs->IMR));	
	
	return IRQ_HANDLED;
}


static int
sg16_open( struct net_device  *ndev )
{
	struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);		
	int err;
	u8 t;    
	// shdsl preactivation 
	if( nl->shdsl_cfg.need_preact ){
		iowrite8( EXT, (iotype)&(nl->regs->IMR) ); 
		iowrite8( 0xff, (iotype)&(nl->regs->SR) ); 
		iowrite8( XRST , (iotype)&(nl->regs->CRA));       			    
		iowrite8( RXDE , (iotype)&(nl->regs->CRB));       			    
		t=0;
		if( shdsl_issue_cmd( nl, _DSL_RESET_SYSTEM, &t, 1 ) )
			return -EBUSY;

		if( !shdsl_ready(nl, _ACK_OPER_WAKE_UP )  ){
			printk(KERN_NOTICE"%s, firmware wasn't loaded\n",
			       ndev->name);	    
			return -EBUSY;
		}
		if( shdsl_preactivation(nl) ){
			printk(KERN_ERR"%s, I/O error\n",ndev->name);
			return -EBUSY;
		}
		// Starting device activation 
		t=0x42;
		if( ( err=shdsl_issue_cmd(nl,_DSL_ACTIVATION,&t,1) ) )
			return -EIO;
		nl->shdsl_cfg.need_preact=0;	
	}
    
	// init descripts, allocate receiving buffers 
	nl->head_xq = nl->tail_xq = nl->head_rq = nl->tail_rq = 0;
	nl->head_tdesc = nl->head_rdesc = 0;
	iowrite8( 0, (iotype)&(nl->regs->CTDR));
	iowrite8( 0, (iotype)&(nl->regs->LTDR));
	iowrite8( 0, (iotype)&(nl->regs->CRDR));
	iowrite8( 0, (iotype)&(nl->regs->LRDR));
	recv_alloc_buffs( ndev );
	// enable receive/transmit functions 
	hdlc_init(nl);
	memset( &nl->stats, 0, sizeof(struct net_device_stats) );
	netif_wake_queue( ndev );		
	return 0;
}

static int
sg16_close( struct net_device  *ndev )
{
	struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);
	unsigned long  flags;    

	// disable receive/transmit functions
	iowrite8( XRST ,(iotype)&(nl->regs->CRA));
	netif_tx_disable(ndev);
	
	// drop receive/transmit queries 
	spin_lock_irqsave( &nl->rlock, flags );
	recv_free_buffs( ndev );
	spin_unlock_irqrestore( &nl->rlock, flags );

	iowrite8( ioread8( (iotype)&(nl->regs->LTDR) ),
		  ( iotype)&(nl->regs->CTDR) );
	xmit_free_buffs( ndev );
	return 0;
}


static struct net_device_stats *
sg16_get_stats( struct net_device  *dev )
{
	struct net_local  *nl  = (struct net_local *)netdev_priv(dev);		
	return  &(nl)->stats;
}

static void
set_multicast_list( struct net_device  *dev )
{
	return;		// SG-16PCI always operate in promiscuos mode 
}

/* --------------------------------------------------------------------------
 *   Hardware control functions   
 * -------------------------------------------------------------------------- */

static void
hdlc_shutdown(struct net_local *nl)
{
	iowrite8( 0, (iotype)&( nl->regs->CRA));    
	iowrite8( RXDE , (iotype)&( nl->regs->CRB));
	iowrite8( 0, (iotype)&( nl->regs->IMR));
	iowrite8( 0xff, (iotype)&( nl->regs->SR));
}

static void
hdlc_init( struct net_local *nl)
{
	u8 cfg_byte;

	cfg_byte=XRST | RXEN | TXEN;			
	if( nl->hdlc_cfg.crc16 )		
		cfg_byte|=CMOD;			
	if( nl->hdlc_cfg.fill_7e )		
		cfg_byte|=FMOD;			
	if( nl->hdlc_cfg.inv )		
		cfg_byte|=PMOD;			
	iowrite8(cfg_byte,(iotype)&(nl->regs->CRA));

	cfg_byte=ioread8( (iotype)&(nl->regs->CRB)) | RODD;		
	if( nl->hdlc_cfg.rburst )		
		cfg_byte|=RDBE;			
	if( nl->hdlc_cfg.wburst )		
		cfg_byte|=WTBE;			
	iowrite8(cfg_byte,(iotype)&(nl->regs->CRB)); 
}

static int 
shdsl_ready( struct net_local *nl, u16 expect_state)
{
#ifdef DEBUG_ON
	volatile struct cx28975_cmdarea  *p = nl->cmdp;
#endif
	u8 ret_val=1;
	u32 ret;

	if( (nl->irqret & 0x1f) != expect_state ){
		ret=interruptible_sleep_on_timeout( &nl->wait_for_intr, HZ*10 );
		if( ( nl->irqret & 0x1f) != expect_state ){
			PDEBUG(5,"fail wait, irqret=%02x expect=%02x",
			       nl->irqret & 0x1f,0xff);
			ret_val=0;
		}
	}
	nl->irqret=0;
	return ret_val;
}

static int
shdsl_dload_fw(struct device *dev)
{
	struct net_device *ndev= (struct net_device *)dev_get_drvdata(dev);
	struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);			
	struct shdsl_config *cfg = (struct shdsl_config *)&(nl->shdsl_cfg);
	volatile struct cx28975_cmdarea  *p = nl->cmdp;
	struct firmware *fw;
	size_t img_len;
	u32   t;
	int  i;
	u8 cksum = 0;

	// 1.Get firmware from userspace 
	if( request_firmware((const struct firmware **)&fw,cfg->fw_name,dev) ){
		printk(KERN_ERR"%s: firmware file not found\n",ndev->name); 
		release_firmware(fw);	
		return -ENOENT;
	}
	for(i=0;i< fw->size;i++)
		cksum += fw->data[i];

	PDEBUG(9,"fw requested");
	// 2.Prepare to download process 
	hdlc_shutdown(nl);
	udelay(10);
	iowrite8( 0, (iotype)&(p->intr_host) );
	ioread8( (iotype)&(p->intr_host) );    
	iowrite8(XRST,(iotype)&(nl->regs->CRA));
	iowrite8(EXT,(iotype)&(nl->regs->IMR)); 
	udelay(10);
	if( !shdsl_ready(nl,_ACK_BOOT_WAKE_UP) )
		goto err_exit;

	// 3.Download process 
	t = fw->size;
	if( shdsl_issue_cmd( nl, _DSL_DOWNLOAD_START, (u8 *) &t, 4 ) )
		goto err_exit;
	PDEBUG(9,"dload start");
	for( i = 0, img_len=fw->size;  img_len >= 75;  i += 75, img_len -= 75 )
		if( shdsl_issue_cmd( nl, _DSL_DOWNLOAD_DATA, fw->data + i, 75 ) )
			goto err_exit;
	if( img_len
	    &&  shdsl_issue_cmd(nl,_DSL_DOWNLOAD_DATA,(fw->data+i),img_len) )
		goto err_exit;
	t = (cksum ^ 0xff) + 1;
	if( shdsl_issue_cmd( nl, _DSL_DOWNLOAD_END, (u8 *) &t, 1 ) )
		goto err_exit;
	PDEBUG(9,"dload end");
	// 4.Check that donload is successfull 
	udelay(10);
	if( !shdsl_ready(nl,_ACK_OPER_WAKE_UP) )
		goto err_exit;
    
	// 5. Check that auto rate selection supported 
	udelay(10);
	t=0;
	if( shdsl_issue_cmd( nl,_DSL_VERSIONS, (u8 *)&t, 1 ) )
		goto err_exit;    

	if( !( ioread8( (iotype)&(p->out_data) + 4 ) & 1 ) ){
		cfg->autob_en=0;
		cfg->autob=0;
	}else
		cfg->autob_en=1;

	release_firmware(fw);
	return  0;
 err_exit:
	release_firmware(fw);
	return -EIO;
}

static int
shdsl_preactivation(struct net_local *nl)
{
	static char  thresh[] = { +8, -4, -16, -40 };
	struct shdsl_config *cfg = (struct shdsl_config*)&(nl->shdsl_cfg);
	u8 t, parm[ 36 ];
	u8 iter;
	u16 tmp;
	u32 max_rate=0 ,min_rate=0;

	// Start preactivation process

	// _DSL_SYSTEM_ENABLE
	t = cfg->master ? 1 : 9;
	if( shdsl_issue_cmd( nl, _DSL_SYSTEM_ENABLE, &t, 1 ) )
		return  -EIO;
	
	// _DSL_SYSTEM_CONFIG
	t = 0x63;
	if( shdsl_issue_cmd( nl, _DSL_SYSTEM_CONFIG, &t, 1 ) )
		return  -EIO;

	// _DSL_MULTI_RATE_CONFIG
	if( cfg->autob ){
		max_rate=MAX_AUTO_RATE;
		min_rate=MIN_AUTO_RATE;
	}else if( cfg->remcfg ){
		if( cfg->master ){
			if( cfg->annex==ANNEX_F )
				max_rate=MAX_REMCFGF_RATE;
			else if( cfg->master )
				max_rate=MAX_REMCFGAB_RATE;
			min_rate=MIN_REMCFG_RATE;
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
	}else
		cfg->lrate= 192 >> 3;

	memset( parm, 0, 8 );
	*(u16 *)parm = ( cfg->lrate >> 3 ) & 0x7f ;
	parm[2] = parm[3] = parm[0];
	parm[5] = cfg->lrate & 7;
	parm[4] = parm[7] = 1;
	parm[6] = 0;
	if( shdsl_issue_cmd( nl, _DSL_MULTI_RATE_CONFIG, parm, 8 ) )
		return  -EIO;    

	// _DSL_PREACT_RATE_LIST
	if( cfg->autob ){
		// Set List of aviable Rates	
		memset( parm, 0, 36 );
		parm[0]=0;
		parm[1]=( u8 )( ((cfg->lrate - ( (MIN_AUTO_RATE >> 3) & 0x7f) ) >> 3 ) & 0x7f )+ 1;
		for(iter=0; iter<parm[1] ; iter++)
			parm[iter+2]=3+iter;	    
		if( shdsl_issue_cmd( nl, _DSL_PREACT_RATE_LIST, parm, iter+2 ) )
			return  -EIO;
	}

	// _DSL_TRAINING_MODE

	if( cfg->autob )
		parm[0] = 0x02 | 0x01<<4;	// In auto rate mode using only TCPAM16
	else if( cfg->remcfg ){
		if( ( cfg->mod==0x00 || cfg->mod==0x01 ) &&
		    cfg->master && cfg->annex==ANNEX_F )
			parm[0] = 0x02 |(cfg->mod << 4);
		else
			parm[0] = 0x02 | 0x01 << 4;
	
	}else
		parm[0] = 0x02 | (cfg->mod << 4);
	parm[1] = 0;

	if( shdsl_issue_cmd( nl, _DSL_TRAINING_MODE, parm, 2 ) )
		return  -EIO;

	// _DSL_PREACTIVATION_CFG

	memset( parm, 0, 12 );
	parm[0] = 0x04;	// pre-activation: G.hs
	if( cfg->autob )
		parm[1] = 0x01;	// Line probe Enabled
	else	
		parm[1] = 0x00;	// Line probe Disabled	
	if( cfg->autob || cfg->remcfg )
		parm[4] = 0x00;	// HTU-C send Mode Select message
	else
		parm[4] = 0x04;	// No remote configuration
	parm[5] = 0x01;	// TPS-TC Config= Clear Channel
	parm[6] = 0x00;
	parm[7]=cfg->annex; // annex A,B,F
	parm[8] = 0xff;	// i-bit mask (all bits)
	if( shdsl_issue_cmd( nl, _DSL_PREACTIVATION_CFG, parm, 12 ) )
		return  -EIO;

	// _DSL_THRESHOLDS
	parm[0] = 0x03;	// dying gasp time - 3 frames
	parm[1] = thresh[ cfg->mod ];	
	parm[2] = 0xff;	// attenuation
	parm[3] = 0x04;	// line probe NMR (+2 dB)
	parm[4] = 0x00;	// reserved 
	parm[5] = 0x00;
	if( shdsl_issue_cmd( nl, _DSL_THRESHOLDS, parm, 6 ) )
		return  -EIO;

	// _DSL_FR_PCM_CONFIG
	t = cfg->master ? 0x23 : 0x21;
	if( shdsl_issue_cmd( nl, _DSL_FR_PCM_CONFIG, &t, 1 ) )
		return  -EIO;

	// _DSL_INTR_HOST_MASK
	t = 0x02;
	if( shdsl_issue_cmd( nl, _DSL_INTR_HOST_MASK, &t, 1 ) )
		return  -EIO;

	return 0;
}

static int
shdsl_get_stat(struct net_local *nl, struct dsl_stats *ds)
{
	u8 t;
	int  i;    
	volatile struct cx28975_cmdarea  *p = nl->cmdp;    

	t = 0;
	if( shdsl_issue_cmd( nl, _DSL_FAR_END_ATTEN, &t, 1 ) )
		return -EIO;
	ds->attenuat = ioread8( (iotype)&(p->out_data) );

	if( shdsl_issue_cmd( nl, _DSL_NOISE_MARGIN, &t, 1 ) )
		return -EIO;
	ds->nmr = ioread8( (iotype)&(p->out_data) );

	if( shdsl_issue_cmd( nl, _DSL_POWER_BACK_OFF_RESULT, &t, 1 ) )
		return -EIO;
	ds->tpbo = ioread8( (iotype)&(p->out_data) );
	ds->rpbo = ioread8( (iotype)&(p->out_data) + 1 );

	if( !shdsl_issue_cmd( nl, _DSL_SYSTEM_PERF_ERR_CTRS, &t, 1 ) ) {
		for( i = 0;  i < 4;  ++i )
			((u8 *)&(ds->all_atmpt))[i] = 
				ioread8( (iotype)&(p->out_data) + i );
	}else
		return -EIO;
	if( !shdsl_issue_cmd( nl, _DSL_HDSL_PERF_ERR_CTRS, &t, 1 ) ) {
		for( i = 0;  i < 10;  ++i )
			((u8 *)&(ds->losw))[i] = 
				ioread8( (iotype)&(p->out_data) + i );
	}else
		return -EIO;

	return 0;
}

static int
shdsl_clr_stat( struct net_local  *nl )
{
	u8 t;
	memset( &nl->in_stats, 0, sizeof(struct sg16_stats) );
	t =0;
	if( shdsl_issue_cmd( nl, _DSL_CLEAR_ERROR_CTRS, &t, 1 ) )
		return -EIO;
	t =0x04;
	if( shdsl_issue_cmd( nl, _DSL_CLEAR_ERROR_CTRS, &t, 1 ) )
		return -EIO;
	return 0;
}

static int
shdsl_issue_cmd( struct net_local  *nl,  u8  cmd,  u8  *data,  u8  size )
{
	volatile struct cx28975_cmdarea  *p = nl->cmdp;
	int  i;
	u8  cksum = 0, tmp;
	u8 *databuf = (u8*)p->in_data;

	iowrite8( 0xf0, (iotype)&(p->in_dest) );
	iowrite8( cmd, (iotype)&(p->in_opcode ) );
	iowrite8( 0, (iotype)&(p->in_zero ) );
	iowrite8(--size, (iotype)&(p->in_length ) );
	iowrite8( ( 0xf0 ^ cmd ^ size ^ 0xaa ),(iotype)&(p->in_csum) ); 
	for( i = 0;  i <= size;  ++i ){
		cksum ^= *data;
		tmp=(u8)*data++;
		iowrite8( tmp,(iotype)(databuf++)); //only 1 byte per cycle!
	}

	iowrite8( cksum^0xaa, (iotype)&(p->in_datasum) );
	iowrite8( _ACK_NOT_COMPLETE, (iotype)&(p->out_ack) );
	iowrite8( 0xfe, (iotype)&(p->intr_8051) );

	if( shdsl_ready(nl,_ACK_PASS ) )
		return  0;
	else
		return  -EIO;
}

static void
shdsl_interrupt( struct net_device  *ndev )
{
	struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);	
	volatile struct cx28975_cmdarea  *p = nl->cmdp;
    
	if(  ioread8( (iotype)&(p->intr_host) ) != 0xfe )
		return;

	if( ioread8( (iotype)&(p->out_ack) ) & 0x80  ){
		nl->link_state.expires = jiffies + HZ/2;    
		add_timer( &(nl->link_state) );    
	}else{ // inquiry answer     
		nl->irqret=ioread8( (iotype)&(p->out_ack) );
		wake_up( &nl->wait_for_intr );	
	}
	// Clear acknoledgement register
	iowrite8(0, (iotype)&(p->out_ack));
	// interrupt served
	iowrite8( 0, (u8 *)&(p->intr_host) );
	ioread8( (u8 *)&(p->intr_host) );
}


static void
shdsl_link_chk( unsigned long data )
{
	struct net_device *ndev=(struct net_device *)data;
	struct net_local  *nl  =(struct net_local *)netdev_priv(ndev);	
	volatile struct cx28975_cmdarea  *p = nl->cmdp;
	struct timeval tv;    
	u8  cksum = 0;

	// Link state
	if( ioread8( (iotype)((u8*)p + 0x3c7) ) & 2 ){
		// link up
		if( ( ioread8( (iotype)( (u8*)p + 0x3c0) ) & 0xc0) == 0x40 ){
			PDEBUG(9,"Activate");
			// set hdlc registers
			iowrite8( 0xff, (iotype)&(nl->regs->SR) );    				    
			iowrite8( ioread8( (iotype)&(nl->regs->CRB) ) & ~RXDE,
				  (iotype)&(nl->regs->CRB) );
			iowrite8( EXT | UFL | OFL | RXS | TXS ,
				  (iotype)&(nl->regs->IMR) );    			
			do_gettimeofday( &tv );
			nl->in_stats.last_time = tv.tv_sec;
			//reset Rx FIFO	    
			iowrite8( 0xf0, (iotype)&(p->in_dest) );
			iowrite8( _DSL_FR_RX_RESET, (iotype)&(p->in_opcode ) );
			iowrite8( 0, (iotype)&(p->in_zero ) );
			iowrite8(0, (iotype)&(p->in_length ) );
			iowrite8( (0xf0^_DSL_FR_RX_RESET^0^0xaa),
				  (iotype)&(p->in_csum) ); 
			cksum ^= 0x8;
			iowrite8( 0x8,(iotype)&(p->in_data));	// only 1 byte per cycle!
			iowrite8( cksum^0xaa, (iotype)&(p->in_datasum) );
			iowrite8( _ACK_NOT_COMPLETE, (iotype)&(p->out_ack) );
			iowrite8( 0xfe, (iotype)&(p->intr_8051) );
			// enable packet receiving-transmitting

			netif_carrier_on( ndev );
		}	// link down
		else if( ( ioread8( (iotype)((u8*)p + 0x3c0) ) & 0xc0) != 0x40 )	{
			PDEBUG(9,"Deactivate");
			iowrite8( ioread8( (iotype)&(nl->regs->CRB) ) | RXDE,
				  (iotype)&(nl->regs->CRB) );
			iowrite8( EXT, (iotype)&(nl->regs->IMR) );    			
			netif_carrier_off( ndev );
		}
	}
}

/* --------------------------------------------------------------------------
 *   Functions, serving transmit-receive process   
 * -------------------------------------------------------------------------- */
 
//---------------tarnsmit-----------------------------------------
 
static void*
xmit_isa_skb_to_dma(struct net_local *nl,struct sk_buff **skb_in,int elem)
{
	struct sk_buff *skb=*skb_in;
	struct dma_buffer *b=(struct dma_buffer *)nl->xbuf+elem;
	PDEBUG(0,"xbuf: strt=%08x,offs=%d,adr=%08x",(u32)nl->xbuf,(int)elem,(u32)b);
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
	
	if( skb->len < ETHER_MIN_LEN ){
		pad = ETHER_MIN_LEN - skb->len;
		// if we have enough space just fill it with 0 and resize skb
		// If the skbuff is non linear tailroom is always zero.. 
		if(skb_tailroom(skb) >= pad) {
			memset(skb->data + skb->len, 0, pad);
			skb->len = ETHER_MIN_LEN;			
		}else{
			nskb = skb_copy_expand(skb,skb_headroom(skb), 
					       (skb_tailroom(skb)+pad), GFP_ATOMIC | GFP_DMA);
			if(!nskb)
				return NULL;
			dev_kfree_skb_any( skb );
			skb = nskb;
			memset(skb->data + skb->len, 0, pad);
			skb->len = ETHER_MIN_LEN;
		}
	}
	*skb_in=skb;
	return (void*)skb->data;
}

static void*
xmit_skb_to_dma(struct net_device *ndev,struct sk_buff **skb,int elem)
{
	struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);	
	
	if( (nl->dev_type == sg16isa) && 
	    (isa_virt_to_bus((*skb)->data)+(*skb)->len > ISA_DMA_MAXADDR) )
		return xmit_isa_skb_to_dma(nl,skb,elem);
	else
		return xmit_pci_skb_to_dma(nl,skb);
}

static int
sg16_start_xmit( struct sk_buff *skb, struct net_device *ndev )
{
	struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);	
	unsigned long  flags;
	dma_addr_t bus_addr;
	void *cpu_addr;
	unsigned  cur_tbd;
	int error=0;
	if ( !netif_carrier_ok(ndev) ){
		dev_kfree_skb_any( skb );
		return 0;
	}

	// fix concurent racing in transmit 
	spin_lock_irqsave( &nl->xlock, flags );
	if( nl->tail_xq == ((nl->head_xq - 1) & (XQLEN - 1)) ) {
		netif_stop_queue( ndev );
		error=-EBUSY;
		goto  err_exit;
	}	
	cpu_addr = xmit_skb_to_dma(ndev,&skb,nl->tail_xq);
	if( !cpu_addr ){
		error=-ENOMEM;
		goto  err_exit;
	}
	// we don't have to check if the descriptor queue was overflowed,
	// because of XQLEN < 128
	nl->xq[ nl->tail_xq++ ] = skb;
	nl->tail_xq &= (XQLEN - 1);
	
	// Map the buffer for DMA 
	bus_addr = dma_map_single(nl->dev,cpu_addr, skb->len , DMA_TO_DEVICE );
	cur_tbd = ioread8( (iotype)&(nl->regs->LTDR)) & 0x7f;
	iowrite32( cpu_to_le32(bus_addr),
		   (iotype)&(nl->tbd[ cur_tbd ].address) ) ;
	iowrite32(cpu_to_le32(skb->len | LAST_FRAG),
		  (iotype)&(nl->tbd[ cur_tbd ].length ) ) ;
	cur_tbd = (cur_tbd + 1) & 0x7f;
	iowrite8(cur_tbd,(iotype)&(nl->regs->LTDR));

	// Probably, it's the best place to increment statistic counters
	// though those frames hasn't been actually transferred yet.
	++nl->in_stats.sent_pkts;
	++nl->stats.tx_packets;
	nl->stats.tx_bytes += skb->len;
	ndev->trans_start = jiffies;

 err_exit:
	spin_unlock_irqrestore( &nl->xlock, flags );
	return  error;
}

// xmit_free_buffs may also be used to drop the queue - just turn
// the transmitter off, and set CTDR == LTDR
static void
xmit_free_buffs( struct net_device *dev )
{
	struct net_local  *nl  = (struct net_local *)netdev_priv(dev);		
	unsigned  cur_tbd = ioread8((iotype)&(nl->regs->CTDR));
	dma_addr_t bus_addr;
	unsigned long flags;

	spin_lock_irqsave(&nl->xlock,flags);

	if( netif_queue_stopped( dev )  &&  nl->head_tdesc != cur_tbd )
		netif_wake_queue( dev );
		    
	while( nl->head_tdesc != cur_tbd ){
		// unmap DMA memory 
		bus_addr=le32_to_cpu( ioread32( (u32*)&(nl->tbd[ nl->head_tdesc ].address)) );
		dma_unmap_single(nl->dev,bus_addr, nl->xq[ nl->head_xq]->len, DMA_TO_DEVICE );
		dev_kfree_skb_any( nl->xq[ nl->head_xq++ ] );
		nl->head_xq &= (XQLEN - 1);
		nl->head_tdesc = (nl->head_tdesc + 1) & 0x7f;
	}
	spin_unlock_irqrestore(&nl->xlock,flags);
}

//---------------receive-----------------------------------------

static struct sk_buff*
recv_skb_sync_one(struct net_device *ndev,int len,int elem)
{
	struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);		
	struct sk_buff *skb = nl->rq[ elem ];
	
	if( (nl->dev_type == sg16isa) && 
	    (isa_virt_to_bus(skb->data)+skb->len > ISA_DMA_MAXADDR) ){
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
	    isa_virt_to_bus(skb->data)+skb->len > ISA_DMA_MAXADDR )
		return (void*)b->buff;
	return (void*)skb->data;
}

static void
recv_init_frames( struct net_device *ndev )
{
	struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);		
	unsigned  cur_rbd = ioread8( (iotype)&(nl->regs->CRDR) ) & 0x7f;
	dma_addr_t bus_addr;    
	struct sk_buff  *skb;
	unsigned  len;

	while( nl->head_rdesc != cur_rbd ) {
		bus_addr=le32_to_cpu( 
				     ioread32((u32*)&(nl->rbd[nl->head_rdesc].address)) );
		PDEBUG(9,"Get packet bus_addr=%08x",bus_addr);
		dma_unmap_single(nl->dev,bus_addr,ETHER_MAX_LEN,DMA_FROM_DEVICE);
		len = nl->rbd[ nl->head_rdesc ].length & 0x7ff;
		if( len < ETHER_MIN_LEN )
			len = ETHER_MIN_LEN;
		// get secon skb from rx queue
		skb=recv_skb_sync_one(ndev,len,nl->head_rq);
		nl->head_rq++;
		nl->head_rq &= (RQLEN - 1);
		// setup skb & give it to OS
		skb_put( skb, len );
		skb->protocol = eth_type_trans( skb, ndev );
		netif_rx( skb );
		++nl->in_stats.rcvd_pkts;
		++nl->stats.rx_packets;
		nl->stats.rx_bytes += len;
		nl->head_rdesc = (nl->head_rdesc + 1) & 0x7f;
	}
	return;
}

static int
recv_alloc_buffs( struct net_device *ndev )
{
	struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);		
	unsigned  cur_rbd = ioread8((iotype)&(nl->regs->LRDR)) & 0x7f;
	dma_addr_t bus_addr;
	void *cpu_addr;
	struct sk_buff  *skb;
    
	while( nl->tail_rq != ((nl->head_rq -1) & (RQLEN - 1)) ){
		skb = dev_alloc_skb(ETHER_MAX_LEN);
		if( !skb )
			return -ENOMEM;
		skb->dev = ndev;
		skb_reserve( skb, 2 );	// align ip on longword boundaries
		// get dma able address & save skb
		cpu_addr=recv_skb_to_dma(ndev,skb,nl->tail_rq);
		nl->rq[ nl->tail_rq++ ] = skb;
		nl->tail_rq &= (RQLEN - 1);
		// DMA memory 
		bus_addr = dma_map_single(nl->dev,cpu_addr, 
					  ETHER_MAX_LEN , DMA_FROM_DEVICE );
		iowrite32( cpu_to_le32( bus_addr ),
			   (u32 *)&(nl->rbd[ cur_rbd ].address) ) ;
		iowrite32( 0, (u32*)&(nl->rbd[ cur_rbd ].length ) ) ;
		cur_rbd = (cur_rbd + 1) & 0x7f;
		iowrite8(cur_rbd,(iotype)&(nl->regs->LRDR));
	}
	return 0;
}

static void
recv_free_buffs( struct net_device *ndev)
{
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);		
	unsigned  last_rbd = ioread8( (iotype)&(nl->regs->LRDR)) & 0x7f;
	dma_addr_t bus_addr;    
	struct sk_buff  *skb;
    
	iowrite8( ioread8( (iotype)&(nl->regs->CRDR)), (iotype)&(nl->regs->LRDR));
	while( nl->head_rdesc != last_rbd ){
		skb = nl->rq[ nl->head_rq++ ];
		nl->head_rq &= (RQLEN - 1);
		bus_addr=le32_to_cpu( 
				     ioread32((u32*)&(nl->rbd[nl->head_rdesc].address)));
		nl->head_rdesc = (nl->head_rdesc + 1) & 0x7f;
		dma_unmap_single(nl->dev,bus_addr,ETHER_MAX_LEN,DMA_FROM_DEVICE);
		dev_kfree_skb_any( skb );
	}
	return;
}



static void
sg16_tx_timeout( struct net_device  *ndev )
{
	struct net_local  *nl  = (struct net_local *)netdev_priv(ndev);		
	u8 tmp;

	tmp=ioread8((iotype)&(nl->regs->IMR));
	iowrite8( 0,(iotype)&(nl->regs->IMR));	    
	udelay(1);
	if( netif_carrier_ok(ndev) )
		iowrite8(ioread8((iotype)&(nl->regs->CRA)) | TXEN,
			 (iotype)&(nl->regs->CRA));
	iowrite8( tmp,(iotype)&(nl->regs->IMR));		
	
	PDEBUG(5,"%s: transmit timeout\n", ndev->name );
	if( ioread8( (iotype)&(nl->regs->SR)) & TXS ){
		PDEBUG(5,"%s: interrupt posted but not delivered\n",
		       ndev->name );
	}
	xmit_free_buffs( ndev );
}


/*----------------------------------------------------------------------------
 * Sysfs specific functions
 *----------------------------------------------------------------------------*/

static int __devinit 
init_sg16_in_sysfs(struct device *dev)
{
	struct net_local *nl=
		(struct net_local *)netdev_priv(dev_get_drvdata(dev));
	struct shdsl_config *shcfg=&(nl->shdsl_cfg);
	struct hdlc_config *hdcfg=&(nl->hdlc_cfg);

	// initialising default parameters
	//shdsl
	strcpy(shcfg->fw_name,"sg16.bin");
	// set default role
#ifdef SG16_MASTER
	shcfg->master=1;
#else
	shcfg->master=0;
#endif    

	// set remote master configuration mode
#ifdef SG16_REMCFG
	shcfg->remcfg=1;
	shcfg->annex=ANNEX_F;    
#else
	shcfg->remcfg=0;
	shcfg->annex=ANNEX_A;    
#endif    

	// set default rate
#ifdef SG_RATE
	shcfg->lrate=SG16_RATE>>3;
#else    
	shcfg->lrate=128>>3;
#endif

	// set coding type
#ifdef SG16_MOD 
#if SG16_MOD == 1
	shcfg->mod=1;
#elif SG16_MOD == 2
	shcfg->mod=2;
#elif SG16_MOD == 3
	shcfg->mod=3;
#else
	shcfg->mod=0;
#endif
#endif

#ifdef SG16_AUTOB
	shcfg->autob=1;
#else
	shcfg->autob=0;
#endif
    
	//hdlc
#ifdef SG16_NORBURST
	hdcfg->rburst=0;
#else    
	hdcfg->rburst=1;
#endif
    
#ifdef SG16_NOWBURST
	hdcfg->wburst=0;
#else
	hdcfg->wburst=1;
#endif    

#ifdef SG16_CRC16
	hdcfg->crc16=1;
#else
	hdcfg->crc16=0;
#endif
    
#ifdef SG16_FILL7E
	hdcfg->fill_7e=1;
#else    
	hdcfg->fill_7e=0;
#endif

#ifdef SG16_INV
	hdcfg->inv=1;    
#else    
	hdcfg->inv=0;
#endif    

	// creating attributes of device in sysfs 
	//shdsl	
	device_create_file(dev,&dev_attr_rate);	
	device_create_file(dev,&dev_attr_crate);
	device_create_file(dev,&dev_attr_master);
	device_create_file(dev,&dev_attr_remcfg);    
	device_create_file(dev,&dev_attr_annex);
	device_create_file(dev,&dev_attr_mod);
	device_create_file(dev,&dev_attr_autobaud);
	device_create_file(dev,&dev_attr_download);    
	//hdlc
	device_create_file(dev,&dev_attr_crc16);
	device_create_file(dev,&dev_attr_fill_7e);
	device_create_file(dev,&dev_attr_inv);
	device_create_file(dev,&dev_attr_rburst);
	device_create_file(dev,&dev_attr_wburst);
	//addr
	device_create_file(dev,&dev_attr_maddr);    
	//stat
	device_create_file(dev,&dev_attr_state);        
	device_create_file(dev,&dev_attr_statistic);        
	//debug
	device_create_file(dev,&dev_attr_debug);
	device_create_file(dev,&dev_attr_hdlc_regs);

	return 0;
}

static void __devexit
del_sg16_from_sysfs(struct device *dev)
{
	// removing attributes of device from sysfs 
	//shdsl
	device_remove_file(dev,&dev_attr_rate);	
	device_remove_file(dev,&dev_attr_crate);	    
	device_remove_file(dev,&dev_attr_master);
	device_remove_file(dev,&dev_attr_remcfg);    
	device_remove_file(dev,&dev_attr_annex);    
	device_remove_file(dev,&dev_attr_mod);
	device_remove_file(dev,&dev_attr_autobaud);
	device_remove_file(dev,&dev_attr_download);    
	//hdlc
	device_remove_file(dev,&dev_attr_crc16);
	device_remove_file(dev,&dev_attr_fill_7e);
	device_remove_file(dev,&dev_attr_inv);
	device_remove_file(dev,&dev_attr_rburst);
	device_remove_file(dev,&dev_attr_wburst);
	//addr
	device_remove_file(dev,&dev_attr_maddr);        
	//stat
	device_remove_file(dev,&dev_attr_state);        
	device_remove_file(dev,&dev_attr_statistic);        
	//debug
	device_remove_file(dev,&dev_attr_debug);            
	device_remove_file(dev,&dev_attr_hdlc_regs);            	
}

/* rate attributes */
static ssize_t
show_rate( struct device *dev, ADDIT_ATTR char *buf )
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct shdsl_config *cfg=&(nl->shdsl_cfg);
	u8 t=0;
	u16 tmp=0;
    
	if( !(cfg->remcfg && !cfg->master) )
		return snprintf(buf,PAGE_SIZE,"%u\n",(cfg->lrate)<<3);
	else{
		if( netif_carrier_ok(ndev) ){
			t=_DSL_DATA_RATE;
			if( !shdsl_issue_cmd(nl,_DSL_READ_CONTROL,&t,0) ){
				tmp=ioread8( (u8 *)(nl->cmdp->out_data+1)) & 0x3;
				tmp=(tmp<<8)+ioread8( (u8 *)(nl->cmdp->out_data) );
				tmp--;
			} 
		}
	}
	return snprintf(buf,PAGE_SIZE,"%u\n",tmp<<3);    
}

static ssize_t
store_rate( struct device *dev, ADDIT_ATTR const char *buf, size_t size ) 
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct shdsl_config *cfg=&(nl->shdsl_cfg);
	char *endp;
	u16 tmp;
    
	// if interface is up
	if( (ndev->flags & IFF_UP) )
		return size;
	if( !size ) return size;

	tmp=simple_strtoul( buf,&endp,0);
	if( !tmp )
		return size;
	cfg->lrate=(tmp >> 3) & 0x3ff;
	nl->shdsl_cfg.need_preact=1;
	return size;
}


static ssize_t
show_crate( struct device *dev, ADDIT_ATTR char *buf )
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct shdsl_config *cfg=&(nl->shdsl_cfg);
	u8 t=0;
	u32 tmp=0;
    
	if( !cfg->autob && !( cfg->remcfg && !cfg->master ) )
		tmp=cfg->lrate;    	
	else
		{
			if( cfg->autob  && netif_carrier_ok(ndev) ){
				if( !shdsl_issue_cmd(nl,_DSL_GHS_GET_FINAL_RATE,&t,0) ){
					tmp=ioread8( (u8 *)(nl->cmdp->out_data+2));
					tmp=(tmp<<3)+ioread8( (u8 *)(nl->cmdp->out_data+3) );
				}
			}else if( cfg->remcfg && netif_carrier_ok(ndev) ){
				t=_DSL_DATA_RATE;
				if( !shdsl_issue_cmd(nl,_DSL_READ_CONTROL,&t,0) ){
					tmp=ioread8( (u8 *)(nl->cmdp->out_data+1)) & 0x3;
					tmp=(tmp<<8)+ioread8( (u8 *)(nl->cmdp->out_data) );
					tmp--;		
				} 
			}
		}
	return snprintf(buf,PAGE_SIZE,"%u\n",tmp<<3);
}

// master attribute 
static ssize_t
show_master( struct device *dev, ADDIT_ATTR char *buf ) 
{
	struct net_local *nl=(struct net_local *)netdev_priv(dev_get_drvdata(dev));
	struct shdsl_config *cfg=&(nl->shdsl_cfg);

	if( cfg->master )
		return snprintf(buf,PAGE_SIZE,"master");
	else
		return snprintf(buf,PAGE_SIZE,"slave"); 
}

static ssize_t
store_master( struct device *dev, ADDIT_ATTR const char *buf, size_t size ) 
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct shdsl_config *cfg=&(nl->shdsl_cfg);

	// if interface is up 
	if( ndev->flags & IFF_UP )
		return size;

	if( size > 0 ){
		if( buf[0]=='0' )
			cfg->master=0;
		else if( buf[0]=='1' )
			cfg->master=1;
	}    
	nl->shdsl_cfg.need_preact=1;
	return strnlen(buf,PAGE_SIZE);
}


// Remote configuration setup 
static ssize_t
show_remcfg( struct device *dev, ADDIT_ATTR char *buf )
{
	struct net_local *nl=(struct net_local *)netdev_priv(dev_get_drvdata(dev));
	struct shdsl_config *cfg=&(nl->shdsl_cfg);

	if( cfg->remcfg )
		return snprintf(buf,PAGE_SIZE,"preact");
	else
		return snprintf(buf,PAGE_SIZE,"local"); 
}

static ssize_t
store_remcfg( struct device *dev, ADDIT_ATTR const char *buf, size_t size )
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct shdsl_config *cfg=&(nl->shdsl_cfg);

	// if interface is up 
	if( ndev->flags & IFF_UP )
		return size;

	if( size > 0 ){
		if( buf[0]=='0' )
			cfg->remcfg=0;
		else if( buf[0]=='1' )
			cfg->remcfg=1;
	}    
	nl->shdsl_cfg.need_preact=1;
	return strnlen(buf,PAGE_SIZE);
}

// Annex setup 
static ssize_t 
show_annex( struct device *dev, ADDIT_ATTR char *buf )
{
	struct net_local *nl=(struct net_local *)netdev_priv(dev_get_drvdata(dev));
	struct shdsl_config *cfg=&(nl->shdsl_cfg);
	u8 t=cfg->annex;

	switch( t ){
	case ANNEX_A:
		return snprintf(buf,PAGE_SIZE,"Annex=A");
	case ANNEX_B:
		return snprintf(buf,PAGE_SIZE,"Annex=B"); 
	case ANNEX_F:
		return snprintf(buf,PAGE_SIZE,"Annex=F"); 
	default:
		cfg->annex=ANNEX_A;
		return snprintf(buf,PAGE_SIZE,"Annex=A");    
	}
}

static ssize_t
store_annex( struct device *dev, ADDIT_ATTR const char *buf, size_t size )
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct shdsl_config *cfg=&(nl->shdsl_cfg);
	u8 tmp;

	// if interface is up 
	if( ndev->flags & IFF_UP )
		return size;
	if( !size )	return size;

	tmp=buf[0];
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

	nl->shdsl_cfg.need_preact=1;
	return strnlen(buf,PAGE_SIZE);
}

/* mod attribute */
static ssize_t
show_mod( struct device *dev, ADDIT_ATTR char *buf )
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct shdsl_config *cfg=&(nl->shdsl_cfg);
	char  *modstr[] = {"TCPAM32", "TCPAM16", "TCPAM8", "TCPAM4"};    
	u8 t=0;
	u16 tmp=0;

	if( cfg->remcfg ){
		if( !netif_carrier_ok(ndev) )
			return snprintf(buf,PAGE_SIZE,"Unknown\n");	
		else{
			t=_DSL_DATA_RATE;
			if( !shdsl_issue_cmd(nl,_DSL_READ_CONTROL,&t,0) ){
				tmp=ioread8( (u8 *)(nl->cmdp->out_data+1))& 0x3;
				tmp=(tmp<<8)+ioread8((u8*)(nl->cmdp->out_data));
				tmp--;		
			} 
			if( !tmp )
				return snprintf(buf,PAGE_SIZE,"Unknown\n");	
			else if( (tmp<<3) > 2304 )
				return snprintf(buf,PAGE_SIZE,"TCPAM32\n");	
			else
				return snprintf(buf,PAGE_SIZE,"TCPAM16\n");	
		}
	}
	return snprintf(buf,PAGE_SIZE,"%s\n",modstr[cfg->mod]);
}

static ssize_t
store_mod( struct device *dev, ADDIT_ATTR const char *buf, size_t size ) 
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct shdsl_config *cfg=&(nl->shdsl_cfg);
	char tmp;    

	// if interface is up 
	if( ndev->flags & IFF_UP )
		return size;
	if( !size )	return size;
	
	tmp=buf[0];
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
	nl->shdsl_cfg.need_preact=1;
	return size;
}

// autobaud attribute 
static ssize_t
show_autob( struct device *dev, ADDIT_ATTR char *buf )
{
	struct net_local *nl=
		(struct net_local *)netdev_priv(dev_get_drvdata(dev));
	struct shdsl_config *cfg=&(nl->shdsl_cfg);

	if( cfg->autob )
		return snprintf(buf,PAGE_SIZE,"auto\n");
	else if( cfg->remcfg && !cfg->master )
		return snprintf(buf,PAGE_SIZE,"remote\n");    
	else
		return snprintf(buf,PAGE_SIZE,"static\n");
}

static ssize_t
store_autob( struct device *dev, ADDIT_ATTR const char *buf, size_t size )
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct shdsl_config *cfg=&(nl->shdsl_cfg);
	u8 tmp;
    
	if( ndev->flags & IFF_UP )
		return size;
	if( !size )
		return 0;

	// check that autobaud is aviable in this firmware 
	if( !cfg->autob_en ){
		cfg->autob=0;
		return size;
	}
	
	// change auto rate state 
	tmp=buf[0];
	switch(tmp){
	case '0':
		cfg->autob=0;
		break;
	case '1':
		cfg->autob=1;
		break;
	}
	nl->shdsl_cfg.need_preact=1;
	return size;
}


// download attribute 
static ssize_t
show_download( struct device *dev, ADDIT_ATTR char *buf )
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	u8 t;

	// if UP => fw is downloaded 
	if( ndev->flags & IFF_UP )
		return snprintf(buf,PAGE_SIZE,"1");

	// Reset of system 
	udelay(2);
	t=0;
	if( shdsl_issue_cmd( nl, _DSL_RESET_SYSTEM, &t, 1 ) )
		return snprintf(buf,PAGE_SIZE,"0");
	udelay(2);
	if( !shdsl_ready( nl, _ACK_OPER_WAKE_UP ) )
		return snprintf(buf,PAGE_SIZE,"0");

	return snprintf(buf,PAGE_SIZE,"1");
}	

static ssize_t
store_download( struct device *dev, ADDIT_ATTR const char *buf, size_t size )
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
    
	if( !size )	return 0;
	if( ndev->flags & IFF_UP )
		return size;

	if( buf[0] == '1' )
		shdsl_dload_fw(dev);

	nl->shdsl_cfg.need_preact=1;
	return size;
}

// CRC count attribute 
static ssize_t
show_crc16( struct device *dev, ADDIT_ATTR char *buf )
{
	struct net_local *nl=(struct net_local *)netdev_priv(dev_get_drvdata(dev));
	struct hdlc_config *cfg=&(nl->hdlc_cfg);

	if( cfg->crc16 )
		return snprintf(buf,PAGE_SIZE,"crc16");
	else
		return snprintf(buf,PAGE_SIZE,"crc32");    
}

static ssize_t
store_crc16( struct device *dev, ADDIT_ATTR const char *buf, size_t size )
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct hdlc_config *cfg=&(nl->hdlc_cfg);
	u8 cfg_bt;

	if( ndev->flags & IFF_UP )
		return size;

	if( !size )	return 0;
    
	switch(buf[0]){
	case '1':
		if( cfg->crc16 )
			break;
		cfg->crc16=1;
		cfg_bt=ioread8( (iotype)&(nl->regs->CRA)) | CMOD;
		iowrite8( cfg_bt,(iotype)&(nl->regs->CRA));
		break;
	case '0':
		if( !(cfg->crc16) )
			break;
		cfg->crc16=0;
		cfg_bt=ioread8( (iotype)&(nl->regs->CRA)) & ~CMOD;
		iowrite8( cfg_bt,(iotype)&(nl->regs->CRA));
		break;
	}	
	return size;	
}

static ssize_t
show_fill_7e( struct device *dev, ADDIT_ATTR char *buf )
{
	struct net_local *nl=
		(struct net_local *)netdev_priv(dev_get_drvdata(dev));
	struct hdlc_config *cfg=&(nl->hdlc_cfg);

	if( cfg->fill_7e )
		return snprintf(buf,PAGE_SIZE,"0x7E");
	else
		return snprintf(buf,PAGE_SIZE,"0xFF");    
}

static ssize_t
store_fill_7e( struct device *dev, ADDIT_ATTR const char *buf, size_t size ) 
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct hdlc_config *cfg=&(nl->hdlc_cfg);
	u8 cfg_bt;

	if( ndev->flags & IFF_UP )
		return size;
    
	if( !size )	return 0;
    
	switch(buf[0]){
	case '1':
		if( cfg->fill_7e )
			break;
		cfg->fill_7e=1;
		cfg_bt=ioread8( (iotype)&(nl->regs->CRA)) | FMOD;
		iowrite8( cfg_bt,(iotype)&(nl->regs->CRA));
		break;
	case '0':
		if( !(cfg->fill_7e) )
			break;
		cfg->fill_7e=0;
		cfg_bt=ioread8( (iotype)&(nl->regs->CRA)) & ~FMOD;
		iowrite8( cfg_bt,(iotype)&(nl->regs->CRA));
		break;
	}	
	return size;	
}

static ssize_t 
show_inv( struct device *dev, ADDIT_ATTR char *buf )
{
	struct net_local *nl=
		(struct net_local *)netdev_priv(dev_get_drvdata(dev));
	struct hdlc_config *cfg=&(nl->hdlc_cfg);

	if( cfg->inv )
		return snprintf(buf,PAGE_SIZE,"on");
	else
		return snprintf(buf,PAGE_SIZE,"off");    
}

static ssize_t
store_inv( struct device *dev, ADDIT_ATTR const char *buf, size_t size )
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct hdlc_config *cfg=&(nl->hdlc_cfg);
	u8 cfg_bt;

	if( ndev->flags & IFF_UP )
		return size;
	if( !size ) return 0;
    
	switch(buf[0]){
	case '1':
		if( cfg->inv )
			break;
		cfg->inv=1;
		cfg_bt=ioread8( (iotype)&(nl->regs->CRA)) | PMOD;
		iowrite8( cfg_bt,(iotype)&(nl->regs->CRA));
		break;
	case '0':
		if( !(cfg->inv) )
			break;
		cfg->inv=0;
		cfg_bt=ioread8( (iotype)&(nl->regs->CRA)) & ~PMOD;
		iowrite8( cfg_bt,(iotype)&(nl->regs->CRA));
		break;
	}	
	return size;	
}

static ssize_t
show_rburst( struct device *dev, ADDIT_ATTR char *buf )
{
	struct net_local *nl=(struct net_local *)netdev_priv(dev_get_drvdata(dev));
	struct hdlc_config *cfg=&(nl->hdlc_cfg);

	if( cfg->rburst )
		return snprintf(buf,PAGE_SIZE,"on");
	else
		return snprintf(buf,PAGE_SIZE,"off");    
}

static ssize_t
store_rburst( struct device *dev, ADDIT_ATTR const char *buf, size_t size )
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct hdlc_config *cfg=&(nl->hdlc_cfg);
	u8 cfg_bt;

	if( ndev->flags & IFF_UP )
		return size;
	if( !size )	return 0;
    
	switch(buf[0]){
	case '1':
		if( cfg->rburst )
			break;
		cfg->rburst=1;
		cfg_bt=ioread8( (iotype)&(nl->regs->CRB)) | RDBE;
		iowrite8( cfg_bt,(iotype)&(nl->regs->CRB));
		break;
	case '0':
		if( !(cfg->rburst) )
			break;
		cfg->rburst=0;
		cfg_bt=ioread8( (iotype)&(nl->regs->CRB)) & ~RDBE;
		iowrite8( cfg_bt,(iotype)&(nl->regs->CRB));
		break;
	}	

	return size;	
}

static ssize_t
show_wburst( struct device *dev, ADDIT_ATTR char *buf ) 
{
	struct net_local *nl=(struct net_local *)netdev_priv(dev_get_drvdata(dev));
	struct hdlc_config *cfg=&(nl->hdlc_cfg);

	if( cfg->wburst )
		return snprintf(buf,PAGE_SIZE,"on");
	else
		return snprintf(buf,PAGE_SIZE,"off");    
}

static ssize_t
store_wburst( struct device *dev, ADDIT_ATTR const char *buf, size_t size )
{
	struct net_device *ndev=(struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);
	struct hdlc_config *cfg=&(nl->hdlc_cfg);
	u8 cfg_bt;

	if( ndev->flags & IFF_UP )
		return size;
	if( !size )	return 0;

	switch(buf[0]){
	case '1':
		if( cfg->wburst )
			break;
		cfg->wburst=1;
		cfg_bt=ioread8( (iotype)&(nl->regs->CRB)) | WTBE;
		iowrite8( cfg_bt,(iotype)&(nl->regs->CRB));
		break;
	case '0':
		if( !(cfg->wburst) )
			break;
		cfg->wburst=0;
		cfg_bt=ioread8( (iotype)&(nl->regs->CRB)) & ~WTBE;
		iowrite8( cfg_bt,(iotype)&(nl->regs->CRB));
		break;
	}	

	return size;	
}

// MAC address less significant value 
static ssize_t
store_maddr( struct device *dev, ADDIT_ATTR const char *buf, size_t size ) 
{
	struct net_device *ndev= (struct net_device *)dev_get_drvdata(dev);
	u16 tmp;
	char *endp;

	if( ndev->flags & IFF_UP )
		return size;

	if( !size ) return 0;

	tmp=simple_strtoul( buf,&endp,16) & 0xfff;
	*(u16 *)ndev->dev_addr = htons( 0x00ff ),
		*(u32 *)(ndev->dev_addr + 2) = htonl( 0x014aa000 | tmp );     

	return size;
}

// Statistic 

static ssize_t
show_dev_state( struct device *dev, ADDIT_ATTR char *buf ) 
{
	struct net_device *ndev= (struct net_device *)dev_get_drvdata(dev);
	struct net_local *nl=(struct net_local *)netdev_priv(ndev);    
	volatile struct cx28975_cmdarea  *p = nl->cmdp;    
	u8 stat_1,stat_3;
	char ret_ad[30];
    
	stat_1=ioread8((iotype)( (u8*)p+0x3c0) );
	stat_3=ioread8((iotype)( (u8*)p+0x3c2) );
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
		return snprintf(buf,42,"not ready %s",ret_ad);
	case  1 :
		return snprintf(buf,PAGE_SIZE,"online");    
	default:
		return snprintf(buf,PAGE_SIZE,"offline");    
	} 
	return 0;
}

static ssize_t
show_statistic( struct device *dev, ADDIT_ATTR char *buf ) 
{
	struct net_local *nl=(struct net_local *)netdev_priv(dev_get_drvdata(dev));
	struct sg16_stats *sb_stat=&(nl->in_stats);
	struct dsl_stats ds;

	if( shdsl_get_stat(nl,&ds) )
		return snprintf(buf,PAGE_SIZE,"err");
    
	return snprintf(buf,PAGE_SIZE,
			"%u %u %u %u  %u %u %u %u  %u %u",
			sb_stat->sent_pkts,sb_stat->rcvd_pkts,
			sb_stat->crc_errs,
			ds.atmpt,ds.all_atmpt,
			ds.losw,
			ds.crc,	ds.attenuat,
			ds.nmr,	sb_stat->last_time );
    
}

static ssize_t
store_statistic( struct device *dev, ADDIT_ATTR const char *buf, size_t size ) 
{
	struct net_local *nl=(struct net_local *)netdev_priv(dev_get_drvdata(dev));
    
	if( !size )	return size;

	if( buf[0]=='1')
		shdsl_clr_stat(nl);
	return size;
}

// debug 
static u8 mem_ret[PAGE_SIZE]="";

static ssize_t
show_debug( struct device *dev, ADDIT_ATTR char *buf ) 
{
	strncat(buf,mem_ret,PAGE_SIZE);
	return strlen(buf);
}

static ssize_t
store_debug( struct device *dev, ADDIT_ATTR const char *buff, size_t size )
{
	struct net_local *nl=(struct net_local *)netdev_priv(dev_get_drvdata(dev));
	volatile struct cx28975_cmdarea  *p = nl->cmdp;    
	u8 cmd,args[200],tmp1;
	u16 len;
	int i=0;
	char *endp,*ptr,bf[PAGE_SIZE];
    
	if( !size )	return size;
    
	len= (PAGE_SIZE-1 > size) ? size : PAGE_SIZE-1;
	strncpy(bf,buff,len);
	bf[len]=0;

	ptr=bf;
	cmd=simple_strtoul( ptr,&endp,16);
	PDEBUG(10,"DEBUG: cmd=%x",cmd);    
	PDEBUG(10,"DEBUG: endp=%x",endp);    

	while( ( ptr-bf < len) && *endp ){
		ptr=endp;
		while( (ptr-bf <len) && ( *ptr<'0' || *ptr>'9')  )
			ptr++;	
		if( (ptr-bf)>=len )
			break;
		args[i]=(u8)simple_strtoul( ptr,&endp,16);
		PDEBUG(10,"DEBUG: args[i]=%x",args[i]);    	
		i++;
	}	
	PDEBUG(10,"DEBUG: i=%d",i);    		
	if( shdsl_issue_cmd(nl,cmd,args,i) ){
		sprintf(mem_ret,"cmd: error");
		return size;
	}
	sprintf(mem_ret,"cmd: %x",cmd);
	for( i=0;i<(p->out_length)+1;i++){
		tmp1 = ioread8( (iotype)&(p->out_data) +i );
		sprintf(bf," %x",tmp1);
		strcat(mem_ret,bf);
	}
	return size;
}
				
static ssize_t
show_hdlc_regs( struct device *dev, ADDIT_ATTR char *buf ) 
{
	struct net_local *nl=(struct net_local *)netdev_priv(dev_get_drvdata(dev));

	return snprintf(buf,PAGE_SIZE,
			"CRA=%02x CRB=%02x SR=%02x IMR=%02x\nCRDR=%02x LRDR=%02x CTDR=%02x LTDR=%02x",
			nl->regs->CRA,nl->regs->CRB,nl->regs->SR,nl->regs->IMR,
			nl->regs->CRDR,nl->regs->LRDR,nl->regs->CTDR,nl->regs->LTDR);
}
