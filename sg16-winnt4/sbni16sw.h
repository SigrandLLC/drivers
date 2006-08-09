
#ifndef SBNI16SW_DEF
#define SBNI16SW_DEF

#define SBNI16_NDIS_MAJOR_VERSION        0x4
#define SBNI16_NDIS_MINOR_VERSION        0x0

#define SBNI16_DRIVER_VERSION ((SBNI16_NDIS_MAJOR_VERSION*0x100) + SBNI16_NDIS_MINOR_VERSION)

#define SBNI16_MAX_FRAME	(1536 + 16)
#define ETHER_MAX_FRAME		1500
#define ETHERNET_HEADER_SIZE	14

#define MAP_REGS		128

struct sbni16_hw_regs {
	UCHAR  CR, CRB, SR, IMR, CTDR, LTDR, CRDR, LRDR;
};

struct hw_descr {
	UINT  address;
	UINT  length;
};

struct cx28975_cmdarea {
	UCHAR  intr_host;
	UCHAR  intr_8051;
	UCHAR  map_version;

	UCHAR  in_dest;
	UCHAR  in_opcode;
	UCHAR  in_zero;
	UCHAR  in_length;
	UCHAR  in_csum;
	UCHAR  in_data[ 75 ];
	UCHAR  in_datasum;

	UCHAR  out_dest;
	UCHAR  out_opcode;
	UCHAR  out_ack;
	UCHAR  out_length;
	UCHAR  out_csum;
	UCHAR  out_data[ 75 ];
	UCHAR  out_datasum;
};

struct sw_rbd {
	struct sw_rbd	*link;
	PVOID		virt_addr;
	ULONG		phys_addr;
	PNDIS_PACKET	pkt_ptr;
	PNDIS_BUFFER	buf_ptr;
};

#define XQLEN	8
#define RQLEN	8

#define SHARED_MEM_SIZE	((RQLEN + 1) * SBNI16_MAX_FRAME)
#define MULTICAST_LIST_SIZE	8

enum State { DOWN, ACTIVATION, ACTIVE };

struct cx28975_cfg {
	UINT  rate;
	UINT  master: 1;
	UINT  mod: 2;
	UINT  crc16: 1;
	UINT  fill_7e: 1;
	UINT  inv: 1;
	UINT  rburst: 1;
	UINT  wburst: 1;
};

typedef struct _SBNI16_ADAPTER {

	NDIS_HANDLE  mport;	/* Miniport handler */

	NDIS_MINIPORT_INTERRUPT  Interrupt;

	NDIS_SPIN_LOCK  Lock;

	/* be used while waiting for interrupt */
	NDIS_EVENT	cx_intr;

	NDIS_HANDLE	PacketPool, BufPool;
	PVOID		sw_rbds;

	/* locked memory used for receive buffers */
	PVOID		virt_shared;
	NDIS_PHYSICAL_ADDRESS  phys_shared;

	/* adapter memory */
	PUCHAR	MemPAddr;

	volatile struct sbni16_hw_regs	*regs;
	volatile struct hw_descr	*tbd;
	volatile struct hw_descr	*rbd;
	volatile struct cx28975_cmdarea	*cmdp;

	struct sbni16_stats {
		/* SBNI16 controller statistics */
		ULONG  sent_pkts, rcvd_pkts;
		ULONG  crc_errs, ufl_errs, ofl_errs, attempts, copied;
		LARGE_INTEGER  last_time;

		/* CX28975 transceiver statistics */
		UCHAR  attenuat, nmr, tpbo, rpbo, status_1, status_3;
		USHORT losw, segd, crc, sega, losd;
	} in_stats;

	struct cx28975_cfg	cfg;

	/* transmit and reception queues */
	NDIS_PACKET	*xq[ XQLEN ];
	struct sw_rbd	*rq[ RQLEN ];
	UINT		head_xq, tail_xq, head_rq, tail_rq;

	/* the descriptors mapped onto the first buffers in xq and rq */
	UINT		head_tdesc, head_rdesc;
	NDIS_PACKET	*xw_head, *xw_tail;

	/* used for physical map registers tracking */
	UINT		free_map_reg, old_map_reg;

	/* mapped receive buffers queue */
	struct sw_rbd	*swr_head, *swr_tail;

	/* contiguos page used for sending packets with <5 bytes frag */
	PVOID		cont_pg_virt;
	ULONG		cont_pg_phys;

	/* standard statistic counters */
	ULONG	XmitGood, RcvGood, XmitBad, CrcErrors;
	UINT	MulticastCount;

	/* resources - memory address and interrupt vector */
	ULONG MemBase;
	CHAR  InterruptNumber;

	UCHAR state;
	UCHAR free_rb;
	UCHAR rdstat;
	UCHAR cont_pg_busy;

	/* текущий MAC-адрес, список групповых адресов и режим фильтрации */
	UCHAR MACAddress[ ETH_LENGTH_OF_ADDRESS ];
	UCHAR MulticastList[ MULTICAST_LIST_SIZE ][ ETH_LENGTH_OF_ADDRESS ];
	UCHAR FilterMode;

} SBNI16_ADAPTER, * PSBNI16_ADAPTER;


#define PKT_LINK( p )	(*(PNDIS_PACKET *)(p)->MiniportReserved)
#define SW_RBD_PTR( p )	(*(struct sw_rbd **)(p)->MiniportReserved)
#define COPY_FLAG( p )	(*(UCHAR *)((p)->MiniportReserved + 4))

#define FIRMW_FNAME	"\\??\\C:\\cx28975_f4_2.bin"

VOID	SB16Isr( OUT PBOOLEAN, OUT PBOOLEAN, IN NDIS_HANDLE );
VOID	SB16HandleInterrupt( IN NDIS_HANDLE );
VOID	SB16MultipleSend( NDIS_HANDLE, PPNDIS_PACKET, UINT );
VOID	SB16GetReturnedPackets( NDIS_HANDLE, PNDIS_PACKET );

BOOLEAN		SB16CheckForHang( NDIS_HANDLE );
NDIS_STATUS	SB16QueryInformation( IN NDIS_HANDLE, IN NDIS_OID, IN PVOID,
					IN ULONG, OUT PULONG, OUT PULONG );
NDIS_STATUS	SB16SetInformation( IN NDIS_HANDLE, IN NDIS_OID, IN PVOID,
					IN ULONG, OUT PULONG, OUT PULONG );
NDIS_STATUS	SB16Initialize( PNDIS_STATUS, PUINT, PNDIS_MEDIUM, UINT,
				NDIS_HANDLE, NDIS_HANDLE );
VOID		SB16Halt( NDIS_HANDLE );
NDIS_STATUS	SB16Reset( PBOOLEAN, NDIS_HANDLE );
VOID		SB16Shutdown( IN NDIS_HANDLE );

NDIS_STATUS	CardStart( IN PSBNI16_ADAPTER );

void	cx28975_interrupt( PSBNI16_ADAPTER );
int	start_cx28975( PSBNI16_ADAPTER, PUCHAR, UINT );
int	issue_cx28975_cmd( PSBNI16_ADAPTER, UCHAR, PUCHAR, UCHAR );
void	alloc_rx_buffers( PSBNI16_ADAPTER );
void	activate( PSBNI16_ADAPTER );
void	deactivate( PSBNI16_ADAPTER );
void	drop_queues( PSBNI16_ADAPTER );
void	reclaim( PSBNI16_ADAPTER, struct sw_rbd * );
void	start_rdstat( PSBNI16_ADAPTER );
void	do_rdstat( PSBNI16_ADAPTER );

/*
NTKERNELAPI
VOID
NTAPI
KeBugCheck (
    IN ULONG BugCheckCode
    );

NTKERNELAPI
VOID
KeBugCheckEx(
    IN ULONG BugCheckCode,
    IN ULONG BugCheckParameter1,
    IN ULONG BugCheckParameter2,
    IN ULONG BugCheckParameter3,
    IN ULONG BugCheckParameter4
    );
*/
#endif
