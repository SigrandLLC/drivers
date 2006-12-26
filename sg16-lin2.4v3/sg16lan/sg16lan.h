#ifndef SG16LAN_H
#define SG16LAN_H


// -------------------------------------------------------------------------- //

#define SG16_IOMEM_LEN 0x1000
// CR bits 
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

#define LAST_FRAG	0x00008000

// We don't have official vendor id yet... 
#define SG_PCI_VENDOR 	0x55 
#define SG_PCI_DEVICE 	0x9d


#define ISA_DMA_MAXADDR 0x1000000
#define ETHER_MIN_LEN 64
#define ETHER_MAX_LEN (1536 + 16)

#define TX_TIMEOUT		400


// SG16 preactivation params 
#define MAX_AUTO_RATE   2304
#define MIN_AUTO_RATE   192
#define MAX_RATE        6016
#define MIN_RATE        64
#define MAX_REMCFGF_RATE        5696
#define MAX_REMCFGAB_RATE       2304
#define MIN_REMCFG_RATE         192


#define ANNEX_A         0x01
#define ANNEX_B         0x02
#define ANNEX_F         0x03

// procfs related constants 
#define PFS_SHDSL_ENTS		6
#define PFS_SHDSL_RATE		1
#define PFS_SHDSL_MASTER	2
#define PFS_SHDSL_CFG		3
#define PFS_SHDSL_ANNEX		4
#define PFS_SHDSL_MOD		5
#define PFS_SHDSL_AUTOB		6

#define PFS_HDLC_ENTS		5
#define PFS_HDLC_CRC16		1
#define PFS_HDLC_FILL7E		2
#define PFS_HDLC_INV		3
#define PFS_HDLC_RBURST		4
#define PFS_HDLC_WBURST		5

#define PFS_STATS_ENTS		3
#define PFS_STATS_FWOK		1
#define PFS_STATS_ONLSTAT	2
#define PFS_STATS_STATISTIC	3

#define PFS_NET_ENTS		1
#define PFS_NET_MACADDR		1

#define PFS_BUS_ENTS		1
#define PFS_BUS_TYPE		1

#define PFS_DEBUG_HREGS		1
#define PFS_DEBUG_ENTS		1

#define PFS_ENTS PFS_HDLC_ENTS+PFS_SHDSL_ENTS+PFS_NET_ENTS+PFS_STATS_ENTS+\
	PFS_BUS_ENTS+PFS_DEBUG_ENTS

// SG16 ioctl cmds 
#define SIOCDEVLOADFW	 	SIOCDEVPRIVATE

enum State { NOT_LOADED, LOADED };
enum sg16_dev_type { sg16pci,sg16isa };
// -------------------------------------------------------------------------- //

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

struct cx28975_fw {
    u8   *firmw_image;
    u32  firmw_len;
};

struct cx28975_cfg {
	u32  lrate: 10;
	u32  master: 1;
	u32  mod: 2;
	u32  crc16: 1;
	u32  fill_7e: 1;
	u32  inv: 1;
	u32  rburst: 1;
	u32  wburst: 1;
	u32  netaddr: 12;
	u32  remcfg: 1;
	u32  autob: 1;
	u8  autob_en:1;
	u8   annex: 2;
	u8   :5;
};

struct dma_buffer{
        u32 pad;
        u8 buff[ETHER_MAX_LEN+3];
        u8 *data;
};
			

// net_device private data 
struct net_local {
        struct net_device_stats	stats;
	wait_queue_head_t  wait_for_intr;

        struct sg16_stats {
		// SG16 controller statistics 
		u32  sent_pkts, rcvd_pkts;
		u32  crc_errs, ufl_errs, ofl_errs, attempts, last_time;
	} in_stats;

        spinlock_t	lock;
	enum sg16_dev_type dev_type;
	void 		*mem_base;		// mapped memory address 
        volatile struct sg16_hw_regs	*regs;
	volatile struct hw_descr	*tbd;
        volatile struct hw_descr	*rbd;
	volatile struct cx28975_cmdarea	*cmdp;

        // transmit and reception queues 
	struct sk_buff	*xq[ XQLEN ], *rq[ RQLEN ];
        unsigned	head_xq, tail_xq, head_rq, tail_rq;
        struct dma_buffer *rbuf,*xbuf;
	// the descriptors mapped onto the first buffers in xq and rq 
        unsigned	head_tdesc, head_rdesc;
	u8		state;
	
        // interrupt related data 
	u8 irqret;
        struct timer_list link_state;
    
	// procfs related data 
        struct proc_dir_entry *ents[PFS_ENTS];
	struct cx28975_cfg cfg;
};

// SHDSL transceiver statistics 
struct dsl_stats {
	u8	status_1, status_3;
	u8	attenuat, nmr, tpbo, rpbo;
	u16	losw, segd, crc, sega, losd;
	u16	all_atmpt,atmpt;
};

//---- module initialisation ----//
int 	init_module( void );
void	cleanup_module( void );

//---- PCI card specific functions ----//
#define SG16_PCI_VENDOR 	0x55 
#define SG16_PCI_DEVICE 	0x9d
static int __devinit  sg16_pci_probe_one( struct pci_dev *,
				const struct pci_device_id * );
static void __devexit  sg16_remove_one( struct pci_dev * );

static struct pci_device_id  sg16_pci_tbl[] __devinitdata = {
	{ SG16_PCI_VENDOR, SG16_PCI_DEVICE, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE( pci, sg16_pci_tbl );

static struct pci_driver  sg16_pci_driver = {
	name:		"sg16lan",
	probe:		sg16_pci_probe_one,
	remove:		sg16_remove_one,
	id_table:	sg16_pci_tbl
};

//---- ISA card specific functions ----//
static struct isapnp_device_id sg16_isa_tbl[] = {
	{	ISAPNP_ANY_ID, ISAPNP_ANY_ID,
		ISAPNP_VENDOR('A','A','A'), ISAPNP_FUNCTION(0x0016), 0 },
	{0}
};
MODULE_DEVICE_TABLE(isapnp, sg16_isa_tbl);

static int __devinit  sg16_isapnp_probe_one(struct pci_dev *,
				const struct isapnp_device_id * );

struct isapnp_driver sg16_isa_driver = {
	name:	"sg16lan",
	id_table: sg16_isa_tbl,
	probe: sg16_isapnp_probe_one,
	remove: sg16_remove_one
};

//---- Network device functions ----//
static int __init  sg16_probe( struct net_device * );
static void  sg16_interrupt( int, void *, struct pt_regs * );
static int  sg16_open( struct net_device * );
static int  sg16_close( struct net_device * );
static int  sg16_start_xmit( struct sk_buff *, struct net_device * );
static int  sg16_ioctl( struct net_device *, struct ifreq *, int );
static struct net_device_stats  *sg16_get_stats( struct net_device * );
static void  set_multicast_list( struct net_device * );

//---- Chipset functions ----//
static void  cx28975_interrupt( struct net_device * );
static void  shdsl_link_chk( unsigned long );
static void  indicate_frames( struct net_device * );
static void  alloc_rx_buffers( struct net_device * );
static void  free_sent_buffers( struct net_device *, int );
static void  drop_rq( struct net_local * );
static void  activate( struct net_device * );
static void  deactivate( struct net_device * );
static void  sg16_tx_timeout( struct net_device * );

//---- procfs control ----//
char sg16_procdir[]="net/drivers/sg16lan";
struct proc_dir_entry *sg16_entry;
static int init_sg16_in_procfs( struct net_device *dev );
static void del_sg16_from_procfs( struct net_device *dev );
static int show_shdsl(char *,char **,off_t,int ,int *,void *);
static int store_shdsl(struct file *,const char*,unsigned long,void*);
static int show_hdlc(char *,char **,off_t,int,int *,void *);
static int store_hdlc(struct file *,const char*,unsigned long,void*);
static int store_stats(struct file *,const char*,unsigned long,void*);
static int store_maddr(struct file *,const char *,unsigned long,void*);
static int show_stats(char *,char **,off_t,int,int *,void *);
static int show_bus(char *,char **,off_t,int,int *,void *);
static int show_hdlc_regs(char *,char **,off_t,int,int *,void *);

//---- shdsl functions ----//
static int  shdsl_ready(struct net_local *, u8 expect);
static int  download_firmware( struct net_device* , u32 ,u8* );
static int  preactivation( struct net_device * );
static int  issue_cx28975_cmd( struct net_local *, u8, u8 *, u8 );
static int  shdsl_get_stat(struct net_local *nl, struct dsl_stats *ds);
static int  shdsl_clr_stat( struct net_local  *nl );

#endif // SG16LAN_H
