/* sg16.c:  Sigrand SG16-PCI SHDSL modem driver for linux (kernel 2.2.x)
 *
 *	Written 2002-2003 by Denis I.Timofeev
 *
 *	This driver presents SG16-PCI modem
 *	to system as common ethernet-like netcard.
 *	More info and useful utilities to work with SG16 modems you can find
 *	at http://www.sigrand.ru
 *
 *	This software may be used and distributed according to the terms
 *	of the GNU General Public License.
 *
 *
 *	22.11.2002	initial revision
 *	17.07.2003	fixed hung-up problem (Eugene Danilov)
 */

#include "cx28975.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/fcntl.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/errno.h>

#include <asm/io.h>
#include <asm/types.h>
#include <asm/byteorder.h>
#include <asm/irq.h>
#include <asm/uaccess.h>


#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/proc_fs.h>

#include <net/arp.h>
#include <linux/pci.h>
#include <linux/random.h>


#include <linux/config.h>

MODULE_DESCRIPTION( "Sigrand SG16 driver Version 1.0.1\n" );
MODULE_AUTHOR( "Denis I. Timofeev\n" );

/* -------------------------------------------------------------------------- */

/* CR bits */
#define	TXEN	0x01		/* transmitter enable */
#define	RXEN	0x02		/* receiver  enable */
#define	NCRC	0x04		/* ignore received CRC */
#define	DLBK	0x08		/* digital loopback */
#define CMOD	0x10		/* 0 - use CRC-32, 1 - CRC-16 */
#define FMOD	0x20		/* interframe fill: 0 - all ones, 1 - 0xfe */
#define PMOD	0x40		/* data polarity: 0 - normal, 1 - invert */
#define XRST	0x80		/* reset the transceiver */

/* CRB bits */
#define RDBE	0x01		/* read burst enable */
#define WTBE	0x02		/* write burst enable */
#define RODD	0x04		/* receive 2-byte alignment */
#define RXDE	0x08		/* receive data enable */

/* SR and IMR bits */
#define	TXS	0x01		/* transmit success */
#define	RXS	0x02		/* receive success */
/* SR only */
#define	CRC	0x04		/* CRC error */
#define	OFL	0x08		/* fifo overflow error */
#define	UFL	0x10		/* fifo underflow error */
#define	EXT	0x20		/* interrupt from sk70725 */
/* IMR only */
#define	TSI	0x80		/* generate test interrupt */

#define LAST_FRAG	0x00008000

/* We don't have official vendor id yet... */
#define SG16_PCI_VENDOR 	0x55 
#define SG16_PCI_DEVICE 	0x9d

#define ETHER_MIN_LEN		60
#define SG16_MAX_FRAME	(1536 + 16)

#define TX_TIMEOUT		400

/* SG16 ioctl params */
#define SIOCDEVLOADFW	 	SIOCDEVPRIVATE
#define SIOCDEVGETSTATS	 	SIOCDEVPRIVATE+1
#define SIOCDEVCLRSTATS	 	SIOCDEVPRIVATE+2

enum State { NOT_LOADED, DOWN, ACTIVATION, ACTIVE };

/* -------------------------------------------------------------------------- */

struct sg16_hw_regs {
	u8  CR, CRB, SR, IMR, CTDR, LTDR, CRDR, LRDR;
};

struct hw_descr {
	u32  address;
	u32  length;
};

struct cx28975_cmdarea {
	u8  intr_host;
	u8  intr_8051;
	u8  map_version;

	u8  in_dest;
	u8  in_opcode;
	u8  in_zero;
	u8  in_length;
	u8  in_csum;
	u8  in_data[ 75 ];
	u8  in_datasum;

	u8  out_dest;
	u8  out_opcode;
	u8  out_ack;
	u8  out_length;
	u8  out_csum;
	u8  out_data[ 75 ];
	u8  out_datasum;
};

#define XQLEN	8
#define RQLEN	8

/* device private data */

struct net_local {
	struct enet_statistics	stats;
	struct wait_queue  *wait_for_intr;
	struct timer_list link_state;

	struct sg16_stats {
		/* SG16 controller statistics */
		u32  sent_pkts, rcvd_pkts;
		u32  crc_errs, ufl_errs, ofl_errs, attempts, last_time;
	} in_stats;

	spinlock_t	lock;

	void 		*mem_base;		/* mapped memory address */

	volatile struct sg16_hw_regs	*regs;
	volatile struct hw_descr	*tbd;
	volatile struct hw_descr	*rbd;
	volatile struct cx28975_cmdarea	*cmdp;

	/* transmit and reception queues */
	struct sk_buff	*xq[ XQLEN ], *rq[ RQLEN ];
	unsigned	head_xq, tail_xq, head_rq, tail_rq;

	/* the descriptors mapped onto the first buffers in xq and rq */
	unsigned	head_tdesc, head_rdesc;
	u8		state;
};

struct cx28975_cfg {
	u8   *firmw_image;
	u32  firmw_len;
	u32  lrate: 10;
	u32  master: 1;
	u32  mod: 2;
	u32  crc16: 1;
	u32  fill_7e: 1;
	u32  inv: 1;
	u32  rburst: 1;
	u32  wburst: 1;
	u32  netaddr: 12;
	u32  : 2;
};

/* SHDSL transceiver statistics */
struct dsl_stats {
	u8	status_1, status_3;
	u8	attenuat, nmr, tpbo, rpbo;
	u16	losw, segd, crc, sega, losd;
};

static int __init  sg16_probe( struct device * );
static void  sg16_interrupt( int, void *, struct pt_regs * );
static int  sg16_open( struct device * );
static int  sg16_close( struct device * );
static int  sg16_start_xmit( struct sk_buff *, struct device * );
static int  sg16_ioctl( struct device *, struct ifreq *, int );
static struct enet_statistics  *sg16_get_stats( struct device * );
static void  set_multicast_list( struct device * );

static void  cx28975_interrupt( struct device * );
static void  sg16_link_chk( unsigned long );
static void  indicate_frames( struct device * );
static void  alloc_rx_buffers( struct device * );
static void  free_sent_buffers( struct device * );
static void  drop_rq( struct net_local * );
static void  activate( struct device * );
static void  deactivate( struct device * );
static void  sg16_tx_timeout( struct device * );


int 	init_module( void );
void	cleanup_module( void );
#ifdef CONFIG_PROC_FS
static int  sbal_read_procmem( char *, char **, off_t, int, int *, void * );
#endif

static int  start_cx28975( struct device *, struct cx28975_cfg, u8 * );
static int  download_firmware( struct device *, u8 *, u32 );

static int  issue_cx28975_cmd( struct net_local *, u8, u8 *, u8 );

/* -------------------------------------------------------------------------- */

#define MAX_UNITS	8

static struct device  *units[ MAX_UNITS ];

static int __init
init_device( struct device  **devp )
{
	if( !(*devp = kmalloc(sizeof(struct device) + 6, GFP_KERNEL)) ) {
		printk( KERN_ERR "sg16: unable to allocate device!\n" );
		return  -ENOMEM;
	}

	memset( *devp, 0, sizeof(struct device) );
	(*devp)->name = (char *)*devp + sizeof( struct device );
	strcpy( (*devp)->name, "sg16" );

	return  0;
}


int
init_module( void )
{
	struct device   *dev;
	struct pci_dev  *pdev = NULL;

	int  num = 0;
	u16  subsys;

	if( !pci_present( ) )
		return  -ENODEV;

	while( (pdev = pci_find_class( PCI_CLASS_NETWORK_OTHER << 8, pdev ))
	       != NULL  &&  num < MAX_UNITS )

		if( pdev->vendor == 0x55  &&  pdev->device == 0x9d ) {
			if( init_device( &dev ) )
				return  -ENOMEM;
			units[ num++ ] = dev;
			pci_read_config_word( pdev, PCI_SUBSYSTEM_ID, &subsys );

			dev->mem_start = pdev->base_address[1]
						& PCI_BASE_ADDRESS_MEM_MASK;
			dev->irq = pdev->irq;

			if( sg16_probe( dev ) )
				return  -ENODEV;
			

		}

	return  num ? 0 : -ENODEV;
}

void
cleanup_module( void )
{
	struct device  *dev;
	int  num;

	/* No need to check MOD_IN_USE, as sys_delete_module( ) checks. */
	for( num = 0;  num < MAX_UNITS;  ++num )
		if( (dev = units[ num ]) != NULL ) {
#ifdef CONFIG_PROC_FS
			remove_proc_entry( dev->name, &proc_root );
#endif
			unregister_netdev( dev );
			free_irq( dev->irq, dev );
			iounmap( ((struct net_local *)dev->priv)->mem_base );
			kfree( dev->priv );
			kfree( dev );
		}
}

/* -------------------------------------------------------------------------- */

static int __init
sg16_probe( struct device  *dev )
{
	struct net_local  *nl;
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry  *pde;
#endif

	/* Allocate dev->priv and fill in sg16-specific dev fields. */
	nl = (struct net_local *) kmalloc(sizeof(struct net_local), GFP_KERNEL);
	if( !nl ) {
		printk( KERN_ERR "%s: unable to get memory!\n", dev->name );
		return  -ENODEV;
	}

	dev->priv = nl;
	memset( nl, 0, sizeof(struct net_local) );

	/* generate 'unique' MAC address */
	*(u16 *)dev->dev_addr = htons( 0x00ff );
	*(u32 *)(dev->dev_addr + 2) = htonl( 0x01a39000 | ((u32)dev->priv & 0x00000fff) );

	nl->mem_base = (void *) ioremap( dev->mem_start, 0x10000 );
	dev->mem_end = dev->mem_start + 0x10000;

	dev->open		= &sg16_open;
	dev->stop		= &sg16_close;
	dev->hard_start_xmit	= &sg16_start_xmit;
	dev->get_stats		= &sg16_get_stats;
	dev->set_multicast_list	= &set_multicast_list;
	dev->do_ioctl		= &sg16_ioctl;
	*dev->name = 0;
	init_etherdev( dev, 0 );
	register_netdevice( dev );

	printk( KERN_NOTICE "%s: Sigrand SG16-SHDSL (irq %d, mem %#lx)\n",
		dev->name, dev->irq, dev->mem_start );

	nl->state = NOT_LOADED;
	nl->tbd  = (struct hw_descr *) nl->mem_base;
	nl->rbd  = (struct hw_descr *) ((u8 *)nl->mem_base + 0x400);
	nl->regs = (struct sg16_hw_regs *) ((u8 *)nl->mem_base + 0x800);
	nl->cmdp = (struct cx28975_cmdarea *) ((u8 *)nl->mem_base + 0xc00);

	nl->regs->CR = 0;
	nl->regs->SR = 0xff;
	nl->regs->IMR = 0;

	if( request_irq(dev->irq, sg16_interrupt, SA_SHIRQ, dev->name, dev) ) {
		printk( KERN_ERR "%s: unable to get IRQ %d.\n",
			dev->name, dev->irq );
		iounmap( nl->mem_base );
		kfree( nl );
		return  -ENODEV;
	}
	
	/* timer init */
    	nl->link_state.data=(unsigned long)dev;
	nl->link_state.function=sg16_link_chk;
	init_timer( &(nl->link_state) );

#ifdef CONFIG_PROC_FS
	pde = create_proc_entry( dev->name, S_IFREG | S_IRUGO, &proc_root );
	if( pde )
		pde->read_proc = sbal_read_procmem,
		pde->data = nl,
		pde->nlink = 1;
#endif
	return  0;
}

/* -------------------------------------------------------------------------- */

#ifdef CONFIG_PROC_FS
static int
sbal_read_procmem( char  *buf,  char  **start,  off_t  offset,  int  count,
		   int  *eof,  void  *data )
{
	struct net_local  *nl = (struct net_local *) data;

	*eof = 1;
	return  sprintf( buf, "%02x %02x %02x %02x %02x (%u)\n",
			 nl->regs->CR,
			 nl->regs->SR,
			 nl->regs->IMR,
			 *((u8 *)nl->cmdp + 0x3c0),
			 *((u8 *)nl->cmdp + 0x3c2),
			 nl->state
			 );
}
#endif

/* -------------------------------------------------------------------------- */

static void
sg16_interrupt( int  irq,  void  *dev_id,  struct pt_regs  *regs )
{
	struct device	  *dev = (struct device *) dev_id;
	struct net_local  *nl  = (struct net_local *) dev->priv;
	u8  status = nl->regs->SR;

	if( status == 0 )
		return;

	if( dev->interrupt ) {
		printk( KERN_ERR "%s: interrupt handler reentered\n",
			dev->name );
		return;
	}

	dev->interrupt = 1; 

	if( status & EXT )
	{
		cx28975_interrupt( dev );
		nl->regs->SR = EXT;
	}
	/*
	 * Whether transmit error is occured, we have to re-enable the
	 * transmitter. That's enough, because linux doesn't fragment
	 * packets.
	 */
	if( status & UFL )
		nl->regs->CR |= TXEN,
		nl->regs->SR = UFL,
		++nl->in_stats.ufl_errs,
		++nl->stats.tx_errors,
		++nl->stats.tx_fifo_errors;

	if( status & RXS )
	{
		indicate_frames( dev ),
		alloc_rx_buffers( dev ),
		nl->regs->SR = RXS;
	}
	if( status & TXS )
	{
		free_sent_buffers( dev ),
		nl->regs->SR = TXS;
	}
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

	dev->interrupt = 0; 
}


static void
cx28975_interrupt( struct device  *dev )
{
	struct net_local   *nl  = (struct net_local *) dev->priv;
	volatile struct cx28975_cmdarea  *p = nl->cmdp;
	u8  t;

	if( p->intr_host != 0xfe )
		return;

	if( p->out_ack & 0x80 ) {
    	nl->link_state.expires=jiffies+HZ/2;
		add_timer(&(nl->link_state));
		p->intr_host = 0;
		t = p->intr_host;
		p->out_ack = 0;
	} else {
		wake_up( &nl->wait_for_intr );
		p->intr_host = 0;
		t = p->intr_host;
	}
}

static void
sg16_link_chk( unsigned long data)
{
	struct device  *dev= (struct device*)data;
	struct net_local   *nl  = (struct net_local *) dev->priv;
	volatile struct cx28975_cmdarea  *p = nl->cmdp;

	if( *((u8 *)p + 0x3c7) & 2 ) {
	    if( nl->state != ACTIVE  &&  (*((u8 *)p + 0x3c0) & 0xc0) == 0x40 )
		activate( dev ),
		printk( KERN_NOTICE "%s: connected to peer\n",
					dev->name );
	    else if( nl->state == ACTIVE	
    		    &&  (*((u8 *)p + 0x3c0) & 0xc0) != 0x40 )
		deactivate( dev ),
		printk( KERN_NOTICE "%s: carrier lost\n",
					dev->name );
	}
}

static int
sg16_start_xmit( struct sk_buff *skb, struct device *dev )
{
	struct net_local  *nl = (struct net_local *) dev->priv;
	unsigned long  flags;
	struct sk_buff *nskb;
	unsigned pad;
	unsigned  cur_tbd;

	spin_lock_irqsave( &nl->lock, flags );

	if( dev->tbusy ) {
		if( nl->state == ACTIVE
		    &&  (jiffies - dev->trans_start) > TX_TIMEOUT )
			sg16_tx_timeout( dev );
		goto  err_exit;
	}

	if( nl->tail_xq == ((nl->head_xq - 1) & (XQLEN - 1)) ) {
		dev->tbusy = 1;
		goto  err_exit;
	}

	nl->xq[ nl->tail_xq++ ] = skb;
	nl->tail_xq &= (XQLEN - 1);

	if( skb->len < ETHER_MIN_LEN )
	{	
	    pad= ETHER_MIN_LEN - skb->len;
	    
	    if( skb_tailroom(skb) >=pad )
	    {
			memset(skb->data+skb->len,0,pad);
			skb->len=ETHER_MIN_LEN;
	    }
	    else
	    {
			nskb=dev_alloc_skb(ETHER_MIN_LEN+1);
			memcpy(skb->data,nskb->data,skb->len);
			dev_kfree_skb(skb);
			skb=nskb;		
			if(skb)
		    	memset( skb->data+skb->len,0,pad);
			skb->len=ETHER_MIN_LEN;
	    }
	}
		
	/*
	 * we don't have to check if the descriptor queue was overflowed,
	 * because of XQLEN < 128
	 */
	cur_tbd = nl->regs->LTDR & 0x7f;
	nl->tbd[ cur_tbd ].address = cpu_to_le32( virt_to_bus( skb->data ) );
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
	return  0;
err_exit:
	spin_unlock_irqrestore( &nl->lock, flags );
	return  1;
}

/* -------------------------------------------------------------------------- */

/*
 *	Open/initialize the board. 
 */

static int
sg16_open( struct device  *dev )
{
	struct net_local	*nl = (struct net_local *) dev->priv;
	u8  t;

	dev->tbusy = 1;
	memset( &nl->stats, 0, sizeof(struct enet_statistics) );
	memset( &nl->in_stats, 0, sizeof(struct sg16_stats) );

	nl->head_xq = nl->tail_xq = nl->head_rq = nl->tail_rq = 0;

	if( nl->state == NOT_LOADED ) {
		printk( KERN_ERR "%s: firmware wasn't loaded\n", dev->name );
		return  -EBUSY;
	}

	nl->regs->IMR = EXT;
	t = 2;
	if( issue_cx28975_cmd( nl, _DSL_CLEAR_ERROR_CTRS, &t, 1 ) )
		return  -EIO;
	if( issue_cx28975_cmd( nl, _DSL_ACTIVATION, &t, 1 ) )
		return  -EIO;

	nl->state = ACTIVATION;
	dev->start = 1;

	MOD_INC_USE_COUNT;
	return 0;
}

static int
sg16_close( struct device  *dev )
{
	struct net_local	*nl = (struct net_local *) dev->priv;
	u8  t;

	nl->regs->IMR = EXT;

	t = 0;
	issue_cx28975_cmd( nl, _DSL_ACTIVATION, &t, 1 );
	if( nl->state == ACTIVE ) {
		t = 1;
		issue_cx28975_cmd( nl, _DSL_FORCE_DEACTIVATE, &t, 1 );
		/* FIX! activation manager state */

		deactivate( dev );
	}
	
	nl->regs->IMR = 0;

	nl->state  = DOWN;
	dev->start = 0;
	MOD_DEC_USE_COUNT;
	return 0;
}


static int
start_cx28975( struct device  *dev,  struct cx28975_cfg  cfg,  u8  *img )
{
	static char  thresh[] = { +8, -4, -16, -40 };

	struct net_local  *nl = (struct net_local *) dev->priv;
	volatile struct cx28975_cmdarea  *p = nl->cmdp;
	u8   t, parm[ 12 ];

	p->intr_host = 0;
	t = p->intr_host;

	/* reset chip set */
	nl->regs->IMR = EXT;
	nl->regs->CR  = 0;
	nl->regs->SR  = 0xff;
	udelay( 2 );
	nl->regs->CR = XRST;
	if( cfg.crc16 )		nl->regs->CR |= CMOD;
	if( cfg.fill_7e )	nl->regs->CR |= FMOD;
	if( cfg.inv )		nl->regs->CR |= PMOD;

	nl->regs->CRB |= RODD | RXDE;
	if( cfg.rburst )	nl->regs->CRB |= RDBE;
	if( cfg.wburst )	nl->regs->CRB |= WTBE;

	interruptible_sleep_on_timeout( &nl->wait_for_intr, HZ*10 );
	if( (p->out_ack & 0x1f) != _ACK_BOOT_WAKE_UP )
		return  -1;

	if( download_firmware( dev, img, cfg.firmw_len ) )
		return  -1;

	interruptible_sleep_on_timeout( &nl->wait_for_intr, HZ*10 );
	if( (p->out_ack & 0x1f) != _ACK_OPER_WAKE_UP )
		return  -1;

	t = cfg.master ? 1 : 9;
	if( issue_cx28975_cmd( nl, _DSL_SYSTEM_ENABLE, &t, 1 ) )
		return  -1;

	t = 0x63;
	if( issue_cx28975_cmd( nl, _DSL_SYSTEM_CONFIG, &t, 1 ) )
		return  -1;

	*(u16 *)parm = cfg.lrate >> 3;
	parm[2] = parm[3] = parm[0];
	parm[5] = cfg.lrate & 7;
	parm[4] = parm[7] = 1;
	parm[6] = 0;
	if( issue_cx28975_cmd( nl, _DSL_MULTI_RATE_CONFIG, parm, 8 ) )
		return  -1;

	parm[0] = 0x02 | (cfg.mod << 4);
	parm[1] = 0;
	if( issue_cx28975_cmd( nl, _DSL_TRAINING_MODE, parm, 2 ) )
		return  -1;

	memset( parm, 0, 12 );
	parm[0] = 0x04;		/* pre-activation: G.hs */
	parm[4] = 0x04;		/* no remote configuration */
	parm[7] = 0x01;		/* annex A (default) */
	parm[8] = 0xff;		/* i-bit mask (all bits) */
	if( issue_cx28975_cmd( nl, _DSL_PREACTIVATION_CFG, parm, 12 ) )
		return  -1;

	parm[0] = 0x03;		/* dying gasp time - 3 frames */
	parm[1] = thresh[ cfg.mod ];	
	parm[2] = 0xff;		/* attenuation */
	parm[3] = 0x04;		/* line probe NMR (+2 dB) */
	parm[4] = 0x00;		/* reserved */
	parm[5] = 0x00;
	if( issue_cx28975_cmd( nl, _DSL_THRESHOLDS, parm, 6 ) )
		return  -1;

	t = cfg.master ? 0x23 : 0x21;
	if( issue_cx28975_cmd( nl, _DSL_FR_PCM_CONFIG, &t, 1 ) )
		return  -1;

	t = 0x02;
	if( issue_cx28975_cmd( nl, _DSL_INTR_HOST_MASK, &t, 1 ) )
		return  -1;

	nl->state = DOWN;
	return  0;
}


static int
download_firmware( struct device  *dev,  u8  *img,  u32  img_len )
{
	struct net_local  *nl = (struct net_local *) dev->priv;
	u32  t;
	int  i;
	u8   cksum = 0;

	for( i = 0;  i < img_len;  ++i )
		cksum += img[i];

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

	return  0;
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

	i = interruptible_sleep_on_timeout( &nl->wait_for_intr, HZ*10 );
	if( i == 0 )
		return  -1;

	while( p->out_ack == _ACK_NOT_COMPLETE )
		;

	if( (p->out_ack & 0x1f) == _ACK_PASS ) {
		p->out_ack = 0;
		return  0;
	} else {
		p->out_ack = 0;
		return  -1;
	}
}

/* -------------------------------------------------------------------------- */

static int
sg16_ioctl( struct device  *dev,  struct ifreq  *ifr,  int  cmd )
{
	struct net_local  *nl = (struct net_local *) dev->priv;
	struct cx28975_cfg  cfg;
	struct dsl_stats    ds;
	u8   *firmw_image;
	int  error = 0;
	u8   t;

	switch( cmd ) {
	case  SIOCDEVLOADFW :
		if( current->euid != 0 )	/* root only */
			return  -EPERM;
		if( (dev->flags & IFF_UP) == IFF_UP )
			return  -EBUSY;
		if( (error = verify_area( VERIFY_READ, ifr->ifr_data,
		     			  sizeof(struct cx28975_cfg) )) != 0 )
			return  error;

		copy_from_user( &cfg, ifr->ifr_data, sizeof cfg );
		if( (error = verify_area( VERIFY_READ, cfg.firmw_image,
		     			  cfg.firmw_len )) != 0 )
			return  error;
		if( !(firmw_image = vmalloc( cfg.firmw_len )) )
			return  -ENOMEM;
		copy_from_user( firmw_image, cfg.firmw_image, cfg.firmw_len );

		if( cfg.netaddr )
			*(u16 *)dev->dev_addr = htons( 0x00ff ),
			*(u32 *)(dev->dev_addr + 2) = htonl( 0x014aa000 | cfg.netaddr );

		if( start_cx28975( dev, cfg, firmw_image ) == 0 ) {
			static char  *modstr[] = {
				"TCPAM32", "TCPAM16", "TCPAM8", "TCPAM4" };
			printk( KERN_NOTICE "%s: %s, rate %d, %s\n",
				dev->name,
				cfg.master ? "master" : "slave",
				cfg.lrate << 3, modstr[ cfg.mod ] );
		} else
			printk( KERN_ERR "%s: unable to load firmware\n",
				dev->name ),
			error = -EIO;

		vfree( firmw_image );
		break;

	case  SIOCDEVGETSTATS :
		if( (error = verify_area( VERIFY_WRITE, ifr->ifr_data,
					  sizeof(struct sg16_stats)
					  + sizeof(struct dsl_stats) )) != 0 )
			return  error;

		t = 0;
		if( issue_cx28975_cmd( nl, _DSL_FAR_END_ATTEN, &t, 1 ) )
			error = -EIO;
		ds.attenuat = nl->cmdp->out_data[ 0 ];

		if( issue_cx28975_cmd( nl, _DSL_NOISE_MARGIN, &t, 1 ) )
			error = -EIO;
		ds.nmr = nl->cmdp->out_data[ 0 ];

		if( issue_cx28975_cmd( nl, _DSL_POWER_BACK_OFF_RESULT, &t, 1 ) )
			error = -EIO;
		ds.tpbo = nl->cmdp->out_data[ 0 ];
		ds.rpbo = nl->cmdp->out_data[ 1 ];

		if( !issue_cx28975_cmd( nl, _DSL_HDSL_PERF_ERR_CTRS, &t, 1 ) ) {
			int  i;
			for( i = 0;  i < 10;  ++i )
				((u8 *) &ds.losw)[i] = nl->cmdp->out_data[i];
		} else
			error = -EIO;

		ds.status_1 = ((u8 *)nl->cmdp)[ 0x3c0 ];
		ds.status_3 = ((u8 *)nl->cmdp)[ 0x3c2 ];

		copy_to_user( ifr->ifr_data, &nl->in_stats,
		     		sizeof(struct sg16_stats) );
		copy_to_user( ifr->ifr_data + sizeof(struct sg16_stats), &ds,
		     		sizeof(struct dsl_stats) );
		break;

	case  SIOCDEVCLRSTATS :
		memset( &nl->in_stats, 0, sizeof(struct sg16_stats) );
		t = 2;
		if( issue_cx28975_cmd( nl, _DSL_CLEAR_ERROR_CTRS, &t, 1 ) )
			error = -EIO;
		break;

	default :
		return  -EOPNOTSUPP;
	}

	return  error;
}


static struct enet_statistics *
sg16_get_stats( struct device  *dev )
{
	return  &((struct net_local *) dev->priv)->stats;
}


static void
set_multicast_list( struct device  *dev )
{
	return;		/* sg16 always operate in promiscuos mode */
}

/* -------------------------------------------------------------------------- */

static void
free_sent_buffers( struct device *dev )
{
	struct net_local  *nl = (struct net_local *) dev->priv;

	unsigned  cur_tbd = nl->regs->CTDR;

	spin_lock( &nl->lock );
	if( dev->tbusy  &&  nl->head_tdesc != cur_tbd ) {
		dev->tbusy = 0;
		mark_bh( NET_BH );
	}

	while( nl->head_tdesc != cur_tbd ) {
		/* ...with respect to mapping xq<->descriptors */
		dev_kfree_skb( nl->xq[ nl->head_xq++ ] );
		nl->head_xq &= (XQLEN - 1);
		nl->head_tdesc = (nl->head_tdesc + 1) & 0x7f;
	}
	spin_unlock( &nl->lock );
}

/*
 * free_sent_buffers may also be used to drop the queue - just turn
 * the transmitter off, and set CTDR == LTDR
 */


static void
alloc_rx_buffers( struct device *dev )
{
	struct net_local  *nl = (struct net_local *) dev->priv;

	unsigned  cur_rbd = nl->regs->LRDR & 0x7f;

	while( nl->tail_rq != ((nl->head_rq - 1) & (RQLEN - 1)) ) {
		struct sk_buff  *skb = dev_alloc_skb( SG16_MAX_FRAME );
		skb->dev = dev;

		skb_reserve( skb, 2 );	/* align ip on longword boundaries */

		nl->rq[ nl->tail_rq++ ] = skb;
		nl->tail_rq &= (RQLEN - 1);

		nl->rbd[ cur_rbd ].address = cpu_to_le32(
						virt_to_bus( skb->data ) );
		nl->rbd[ cur_rbd ].length  = 0;
		nl->regs->LRDR = cur_rbd = (cur_rbd + 1) & 0x7f;
	}
}

static void
indicate_frames( struct device *dev )
{
	struct net_local  *nl = (struct net_local *) dev->priv;

	unsigned  cur_rbd = nl->regs->CRDR & 0x7f;
	unsigned  len;

	while( nl->head_rdesc != cur_rbd ) {
		struct sk_buff  *skb = nl->rq[ nl->head_rq++ ];
		nl->head_rq &= (RQLEN - 1);

		len = nl->rbd[ nl->head_rdesc ].length & 0x7ff;
		if( len < ETHER_MIN_LEN )
			len = ETHER_MIN_LEN;
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
		dev_kfree_skb( nl->rq[ nl->head_rq++ ] );
		nl->head_rq &= (RQLEN - 1);
	}
}


static void
activate( struct device  *dev )
{
	struct net_local	*nl = (struct net_local *) dev->priv;
	struct timeval		tv;

	nl->regs->SR   = 0xff;		/* clear it! */
	nl->regs->CTDR = nl->regs->LTDR = nl->regs->CRDR = nl->regs->LRDR = 0;

	nl->head_tdesc = nl->head_rdesc = 0;
	alloc_rx_buffers( dev );

	nl->regs->CRB &= ~RXDE;
	nl->regs->IMR = EXT | RXS | TXS | OFL | UFL;
	nl->regs->CR  |= TXEN | RXEN;

	nl->state = ACTIVE;
	++nl->in_stats.attempts;
	do_gettimeofday( &tv );
	nl->in_stats.last_time = tv.tv_sec;

	dev->tbusy = 0;
	mark_bh( NET_BH );
}

static void
deactivate( struct device  *dev )
{
	struct net_local	*nl = (struct net_local *) dev->priv;

	nl->regs->CR  &= ~(RXEN | TXEN);
	nl->regs->CRB |= RXDE;
	nl->regs->IMR  = EXT;
	nl->regs->CTDR = nl->regs->LTDR;
	nl->regs->CRDR = nl->regs->LRDR;
	nl->state = ACTIVATION;

	dev->tbusy = 0;
	/* use free_sent_buffers for dropping the xmit queue */
	free_sent_buffers( dev );
	drop_rq( nl );
	dev->tbusy = 1;
}


static void
sg16_tx_timeout( struct device  *dev )
{
	struct net_local  *nl = (struct net_local *) dev->priv;

	printk( KERN_ERR "%s: transmit timeout\n", dev->name );

	if( nl->regs->SR & TXS )
		nl->regs->SR = TXS,
		printk( KERN_ERR "%s: interrupt posted but not delivered\n",
			dev->name );
	free_sent_buffers( dev );
}
