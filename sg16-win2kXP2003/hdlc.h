#define NUM_HLDC_RING_ELEMS 128 /* Number of ring descriptors */

/* Frame buffer size for HLDC. Must be multiple of 4 */
#define DMA_BUFFER_SIZE ( (ETHERNET_MAX_SIZE + 3) &~3 )

/* -----------------------------------------------------------------------------
 *    HDLC register definitions
 ------------------------------------------------------------------------------- */
struct HldcRegs
{	/* HLDC registers */
	BYTE volatile	CRA;	/* Control register A */
#define TXEN	0x01		/* transmitter enable */
#define RXEN	0x02		/* receiver enable */
#define NCRC	0x04		/* ignore received CRC */
#define DLBK	0x08		/* digital loopback */
#define CMOD	0x10		/* 0 - use CRC-32, 1 - CRC-16 */
#define FMOD	0x20		/* interframe fill: 0 - all ones, 1 - 0xfe */
#define PMOD	0x40		/* data polarity: 0 - normal, 1 - invert */
#define XRST	0x80		/* reset the transceiver */
	BYTE volatile	CRB;	/* Control register B */
#define RDBE	0x01		/* read burst enable */
#define WTBE	0x02		/* write burst enable */
#define RODD	0x04		/* receive 2-byte alignment */
#define RXDE	0x08		/* receive data enabled if 0 */
	BYTE volatile	SR;		/* Status register */
#define TXS 0x01			/* transmit success */
#define RXS 0x02			/* receive success */
#define CRC 0x04			/* CRC error */
#define OFL 0x08			/* fifo overflow error */
#define UFL 0x10			/* fifo underflow error */
#define EXT 0x20			/* interrupt from sk70725 */
#define ALL 0xFF			/* All bits */
	BYTE volatile	IMR;	/* Interrupt mask register */
#define TSI 0x80			/* generate test interrupt */
	BYTE volatile	CTDR;	/* Current transfer descriptor register */
	BYTE volatile	LTDR;	/* Last transfer descriptor register */
	BYTE volatile	CRDR;	/* Current recieve descriptor register */
	BYTE volatile	LRDR;	/* Last recieve descriptor register */

	/* -------------------------------------------------------------------------
	 *
	 --------------------------------------------------------------------------- */
	BYTE
	TestStatusBits( BYTE const Mask ) const
	{
		return BYTE( SR & Mask );
	}

	/* -------------------------------------------------------------------------
	 *
	 --------------------------------------------------------------------------- */
	void
	ResetStatusBits( BYTE const Mask )
	{
		SR=Mask;
	}

	/* -------------------------------------------------------------------------
	 *
	 --------------------------------------------------------------------------- */
	void
	SetIntMask( BYTE const Mask )
	{
		IMR=Mask;
	}

	/* -------------------------------------------------------------------------
	 *
	 --------------------------------------------------------------------------- */
	void
	EnableInts( BYTE const Mask )
	{
		SetBits( IMR, Mask );
	}

	/* -------------------------------------------------------------------------
	 *
	 --------------------------------------------------------------------------- */
	void
	DisableInts( BYTE const Mask )
	{
		ResetBits( IMR, Mask );
	}

	/* -------------------------------------------------------------------------
	 *
	 --------------------------------------------------------------------------- */
	BYTE
	GetCTDR( void ) const
	{
		return CTDR /* % NUM_HLDC_RING_ELEMS */ ;
	}

	/* -------------------------------------------------------------------------
	 *
	 --------------------------------------------------------------------------- */
	BYTE
	GetLTDR( void ) const
	{
		return LTDR /* % NUM_HLDC_RING_ELEMS */ ;
	}

	/* -------------------------------------------------------------------------
	 *
	 --------------------------------------------------------------------------- */
	BYTE
	GetCRDR( void ) const
	{
		return CRDR /* % NUM_HLDC_RING_ELEMS */ ;
	}

	/* -------------------------------------------------------------------------
	 *
	 --------------------------------------------------------------------------- */
	BYTE
	GetLRDR( void ) const
	{
		return LRDR /* % NUM_HLDC_RING_ELEMS */ ;
	}

	/* -------------------------------------------------------------------------
	 *
	 --------------------------------------------------------------------------- */
	static UINT
	NextBD( UINT const BD )
	{
		return( BD + 1 ) % NUM_HLDC_RING_ELEMS;
	}
};
Assume( offsetof(HldcRegs, CRA) == 0 );
Assume( offsetof(HldcRegs, CRB) == 1 );
Assume( offsetof(HldcRegs, SR) == 2 );
Assume( offsetof(HldcRegs, IMR) == 3 );
Assume( offsetof(HldcRegs, CTDR) == 4 );
Assume( offsetof(HldcRegs, LTDR) == 5 );
Assume( offsetof(HldcRegs, CRDR) == 6 );
Assume( offsetof(HldcRegs, LRDR) == 7 );
class	DmaBufferDesc
{	/* DMA buffer descriptor */
public:
	DWORD volatile	PhysAddr;	/* Physical address */

	/* TD: any address RD: bits 0..1 aren't used */
	DWORD volatile	Length;		/* Packet length and flags */
#define DB_LENMASK	0x7FF		/* Length field mask */
#define DB_LAST		0x8000		/* Last fragment of transmitting packet */
#define DB_ERROR	0x40000000	/* Error in received packet */
#define DB_DONE		0x80000000	/* Packet receive is complete */
};
class	DmaBuffer
{	/* DMA buffer structure */
public:
	DmaBufferDesc	BdArray[NUM_HLDC_RING_ELEMS];
};
class	Hdlc_Cfg
{
public:
	bool	crc16;
	bool	fill_7e;
	bool	inv;
	bool	rburst;
	bool	wburst;
	Hdlc_Cfg( void )
	{
		crc16=Hdlc_Crc16Def;
		fill_7e=Hldc_Fill7eDef;
		inv=Hldc_InvertedDef;
		rburst=Hldc_ReadBurstDef;
		wburst=Hldc_WriteBurstDef;
	}
};
