/* sg16lan.h:  Sigrand SG-16PCI SHDSL modem driver for linux (kernel 2.6.x)
 *
 *	Written 2005-2006 by Artem U. Polyakov <art@sigrand.ru>
 *
 *	This driver presents SG-16PCI modem 
 *	to system as common ethernet-like netcard.
 *
 */

#ifndef SG16LAN_H
#define SG16LAN_H

#define SG16_IOMEM_LEN 0x1000
#define SG16_TBD_OFFSET 0x0
#define SG16_RBD_OFFSET 0x400
#define SG16_REGS_OFFSET 0x800
#define SG16_CMDP_OFFSET 0xc00
// CRA bits 
#define	TXEN	0x01		// transmitter enable 
#define	RXEN	0x02		// receiver  enable 
#define	NCRC	0x04		// ignore received CRC 
#define	DLBK	0x08		// digital loopback 
#define CMOD	0x10		// 0 - use CRC-32, 1 - CRC-16 
#define FMOD	0x20		// interframe fill: 0 - all ones, 1 - 0xfe 
#define PMOD	0x40		// data polarity: 0 - normal, 1 - invert 
#define XRST	0x80		// reset the transceiver 

// CRB bits 
#define RDBE	0x01		// read burst enable 
#define WTBE	0x02		// write burst enable 
#define RODD	0x04		// receive 2-byte alignment 
#define RXDE	0x08		// receive data enable 

// SR and IMR bits 
#define	TXS	0x01		// transmit success 
#define	RXS	0x02		// receive success 
// SR only 
#define	CRC	0x04		// CRC error 
#define	OFL	0x08		// fifo overflow error 
#define	UFL	0x10		// fifo underflow error 
#define	EXT	0x20		// interrupt from sk70725 
// IMR only 
#define	TSI	0x80		// generate test interrupt 

#define LAST_FRAG 0x00008000

#define ISA_DMA_MAXADDR 0x1000000

#define ETHER_MIN_LEN	64
#define ETHER_MAX_LEN	(1536 + 16)

#define TX_TIMEOUT	400

// SHDSL parameters
#define MAX_AUTO_RATE	2304
#define MIN_AUTO_RATE	192
#define MAX_RATE	6016
#define MIN_RATE	64
#define MAX_REMCFGF_RATE	5696
#define MAX_REMCFGAB_RATE	2304
#define MIN_REMCFG_RATE		192


#define ANNEX_A		0x01
#define ANNEX_B		0x02
#define ANNEX_F		0x03


// Internal consts 
#define EFWDLOAD 0x20
#define iotype u8*

// Portability 
#ifdef NO_IO_READ_WRITE
#       define iowrite8(val,addr)  writeb(val,addr)
#       define iowrite32(val,addr)  writel(val,addr)
#       define ioread8(addr) readb(addr)
#       define ioread32(addr) readl(addr)
#endif

// -------------------------------------------------------------------------- //

enum sg16_dev_type { sg16pci,sg16isa };

struct sg16_hw_regs {
	u8  CRA, CRB, SR, IMR, CTDR, LTDR, CRDR, LRDR;
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

// net_device private data
#define FW_NAME_SIZE 255
struct shdsl_config
{
	char fw_name[FW_NAME_SIZE];
	u16 lrate:	10;
	u16 master:	1;
	u16 mod:	2;
	u16 autob:	1;
	u16 autob_en: 1;    
	u16 need_preact: 1;
	u8 remcfg :1;
	u8 annex :2;
u8 :5;
};

struct hdlc_config
{
	u8  crc16: 1;
	u8  fill_7e: 1;
	u8  inv: 1;
	u8  rburst: 1;
	u8  wburst: 1;
};

struct dma_buffer{
	u32 pad;
	u8 buff[ETHER_MAX_LEN+3];
	u8 *data;
};

struct net_local{
    
	struct net_device_stats	stats;
	wait_queue_head_t  wait_for_intr;
	// debug purpose
	wait_queue_head_t  wait_test;
	//--------------------------------

	struct device *dev;
	enum sg16_dev_type dev_type;

	// Configuration structures
	struct hdlc_config hdlc_cfg;
	struct shdsl_config shdsl_cfg;
	u8 irqret;

	// SG-16PCI controller statistics
	struct sg16_stats {
		u32  sent_pkts, rcvd_pkts;
		u32  crc_errs, ufl_errs, ofl_errs, last_time;
	} in_stats;

	spinlock_t rlock,xlock;
    
	void *mem_base;		// mapped memory address

	volatile struct sg16_hw_regs	*regs;
	volatile struct hw_descr	*tbd;
	volatile struct hw_descr	*rbd;
	volatile struct cx28975_cmdarea	*cmdp;

	// transmit and reception queues 
	struct sk_buff *xq[ XQLEN ], *rq[ RQLEN ];
	struct dma_buffer *rbuf,*xbuf;
	unsigned head_xq, tail_xq, head_rq, tail_rq;


	// the descriptors mapped onto the first buffers in xq and rq 
	unsigned head_tdesc, head_rdesc;
	u8 fw_state;
    
	// timered link check entire 
	struct timer_list link_state;
};

// SHDSL transceiver statistics
struct dsl_stats {
	u8	status_1, status_3;
	u8	attenuat, nmr, tpbo, rpbo;
	u16	losw, segd, crc, sega, losd;
	u16	all_atmpt,atmpt;
};

//---- Driver initialisation ----//
static int  sg16_init( void );
static void sg16_exit( void );
static void __devinit dsl_init( struct net_device *ndev);

//---- PCI adapter related ----//
// We don't have official vendor id yet... 
#define SG16_PCI_VENDOR 	0x55 
#define SG16_PCI_DEVICE 	0x9d

static int __devinit sg16_pci_probe_one(struct pci_dev *,
					const struct pci_device_id *);
static void __devexit  sg16_pci_remove_one(struct pci_dev *);

static struct pci_device_id  __devinitdata sg16_pci_tbl[]  = {
{ PCI_DEVICE(SG16_PCI_VENDOR,SG16_PCI_DEVICE) },
{ 0 }
};
MODULE_DEVICE_TABLE( pci, sg16_pci_tbl );

static struct pci_driver  sg16_driver = {
 name:		"sg16lan",
 probe:		sg16_pci_probe_one,
 remove:		sg16_pci_remove_one,
 id_table:	sg16_pci_tbl
};


//---- ISA adapter related ----//
static struct pnp_device_id __devinitdata sg16_pnp_tbl[]  = {
{"AAA0016",(long)"Sigrand SG-16ISA"},
{"",0}     /* terminate list */
};
MODULE_DEVICE_TABLE(pnp, sg16_pnp_tbl);

static int __devinit sg16_isapnp_probe_one(struct pnp_dev *idev,
					   const struct pnp_device_id *dev_id);
static void __devexit sg16_isapnp_remove_one(struct pnp_dev *idev);

static struct pnp_driver sg16_isapnp_driver = {
 name :		"sg16lan",
 id_table:       sg16_pnp_tbl,
 probe:          sg16_isapnp_probe_one,
 remove:         sg16_isapnp_remove_one,
};

#ifndef CONFIG_ISAPNP
#warning ISA PnP subsystem does not configured in kernel
#warning enable it first if you use ISA card
#endif

//---- Net device specific functions ----//
static int __devinit  sg16_probe( struct net_device * );

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t  sg16_interrupt( int, void *, struct pt_regs * );
#else
static irqreturn_t  sg16_interrupt( int, void *);
#endif

static int  sg16_open( struct net_device * );
static int  sg16_close( struct net_device * );
static int  sg16_start_xmit( struct sk_buff *, struct net_device * );
static struct net_device_stats  *sg16_get_stats( struct net_device * );
static void  set_multicast_list( struct net_device * );

//---- Functions serving SG-16PCI control ----//

static void hdlc_init( struct net_local *nl);
static void hdlc_shutdown( struct net_local *nl );

static int  shdsl_ready( struct net_local *nl, u16 expect_state);
static int  shdsl_dload_fw(struct device *dev);
static int  shdsl_preactivation(struct net_local *nl);
static int  shdsl_get_stat(struct net_local *nl, struct dsl_stats *ds);
static int  shdsl_clr_stat( struct net_local  *nl );
static int  shdsl_issue_cmd( struct net_local *, u8, u8 *, u8 );
static void shdsl_interrupt( struct net_device * );
static void shdsl_link_chk( unsigned long );

//---- Functions, serving transmit-receive process ----//
static void  recv_init_frames( struct net_device * );
static int   recv_alloc_buffs( struct net_device * );
static void  recv_free_buffs( struct net_device * );
static void  xmit_free_buffs( struct net_device * );
static void  sg16_tx_timeout( struct net_device * );

//---- Sysfs specific functions ----//
static void sg16_defcfg(struct net_device *ndev);
static int sg16_sysfs_register( struct net_device *);
static void sg16_sysfs_remove(struct net_device *dev);

#endif
