#define PCI_VENDOR_ID_REGISTER		0
#define PCI_DEVICE_ID_REGISTER		2
#define PCI_SUBSYSTEM_ID_REGISTER	0x2E
#define PCI_MEM_BASE_REGISTER		0x14
#define PCI_INTERRUPT_REGISTER		0x3C
#define SG16_MEMSIZE				4096	/* Size of memory window */
class	AdapterDesc
{	/* Main adapter structure */
public:
	NDIS_HANDLE DriverHandle;							/* Miniport handle */
	DWORD	MemoryWindowPhysAddr;						/* Adapter memory
														 * window physical
														 * address */
	PVOID	MemoryWindowAddr;							/* Virtual address */
	HldcRegs	*HLDC;									/* HLDC register set
														 * address */
	DmaBuffer	*TxDmaBuffer;							/* TBD area address */
	DmaBuffer	*RxDmaBuffer;							/* RBD area address */
	cx28975_cmdarea *cmdp;								/* Modem command window
														 * address */
	bool	MapRegsAllocated;							/* Tracking flag */
	NDIS_HANDLE PacketPool;								/* Received packet pool
														 * handle */
	NDIS_HANDLE BufPool;								/* Received packet
														 * buffers pool handle */
	bool	PacketPoolAllocated, BufferPoolAllocated;	/* Tracking flags */
	CHAR	InterruptNumber;						/* Adapter IRq number */
	NDIS_MINIPORT_INTERRUPT Interrupt;				/* Interrupt descriptor */
	bool	InterruptRegistered;					/* Tracking flag */
	NDIS_EVENT	cx_intr;							/* Be used while waiting
													 * for interrupt */

	/*
	 * Modem command completion routine prototype If called with Arg
	 * DWORD_PTR (-1), an error was occured
	 */
	typedef void ( AdapterDesc:: *ModemCompletionRoutine ) ( DWORD_PTR Arg );
	ModemCompletionRoutine volatile ModemComplRtn;	/* Modem command completion
													 * routine */
	DWORD volatile	ModemComplRtnArg;				/* Completion routine
													 * argument */
	PVOID	QI_InfoBuffer;					/* QueryInformation buffer address */
	PULONG	QI_BytesWritten;				/* BytesWritten reference */
	PULONG	QI_BytesNeeded;					/* BytesNeeded reference */
	PVOID	DmaMemAddr;						/* Locked memory used for DMA
											 * buffers */
	NDIS_PHYSICAL_ADDRESS	DmaMemPhysAddr; /* Physical address */
	ULONG	DmaMemSize;						/* DMA memory size */
	DmaQueue	XmtDmaPool;					/* Transmit DMA packet pool
											 * (available desc list) */
	DmaQueue	XmtDmaQueue;				/* Transmit DMA packet queue
											 * (active desc list) */
	UINT	XmtQueueLen;					/* Length in packets */
	UINT	TxRingRemovePos;				/* HLDC tx ring remove position */
	DmaQueue	RcvDmaPool;					/* Receive DMA packet pool
											 * (available desc list) */
	DmaQueue	RcvDmaQueue;				/* Receive DMA packet queue (active
											 * desc list) */
	UINT	RcvQueueLen;					/* Length in packets */
	UINT	RxRingRemovePos;				/* HLDC rx ring remove position */
	PacketList	PacketsFromNdis;			/* Primary queue for holding
											 * packets submitted by NDIS */
	cx28975_cfg ModemCfg;					/* Modem configuration */
	Hdlc_Cfg	HdlcCfg;					/* HDLC configuration */
	StatDesc	Stat;						/* Statistics data */
	SpinLock	SL;						/* Protection spin lock */

	/* Standard statistic counters */
	ULONG64 volatile	XmitGood;
	ULONG64 volatile	XmitBad;
	ULONG64 volatile	RcvdGood;
	ULONG64 volatile	CrcErrors;

	/* Additional counters */
	ULONG64 volatile	IntCount;		/* Interrupt counter */
#ifdef _DEBUG
	LONG volatile	IntDepth;			/* Prevents interrupt handler reentrance */
	LONG volatile	InCompleteTxDepth;	/* Prevents CompleteTxPackets reentrance */
	LONG volatile	InCompleteRxDepth;	/* Prevents CompleteRxPackets reentrance */
	ULONG	TotalReceivedBytes;			/* Total received bytes counter */
#endif
	BYTE volatile	LastIntEvents;		/* Status event bits at last interrupt */
	ModemStates volatile	ModemState; /* State of the modem */
	BYTE volatile	RdStatStage;		/* Read statistics stage number */

	/* Current MAC address, multicast address list and filtering mode */
	BYTE	MACAddress[ETH_LENGTH_OF_ADDRESS];
	BYTE	MulticastList[MULTICAST_LIST_SIZE][ETH_LENGTH_OF_ADDRESS];
	UINT	MulticastCount;
	BYTE	FilterMode;
	AdapterDesc( void );
	~AdapterDesc( void );
	void
	Lock( void )	{ SL.Acquire(); }

	void
	Unlock( void )	{ SL.Release(); }

	bool
	IsLocked( void ) const
	{
		return SL.IsOwned();
	}

	void	Reset( void );
	NDIS_STATUS CreatePools( void );
	void	DestroyPools( void );
	NDIS_STATUS InitDmaAndInt( void );
	void	TermDmaAndInt( void );
	NDIS_STATUS Init( NDIS_HANDLE MPH, NDIS_HANDLE WrapperConfigurationContext );
	void	Term( void );
	void	SendModemMessage( BYTE Cmd, PCVOID Data=NULL, UINT Size=0 );
	bool	WaitForModemReply( void );
	bool	DoModemCmd( BYTE Cmd, PCVOID Data=NULL, UINT Size=0,
						ModemCompletionRoutine ComplRtn=NULL, DWORD_PTR RtnArg=0 );
	void	cx28975_interrupt( void );
	bool	download_firmware( PCBYTE img, UINT img_len );
	bool	start_cx28975( PCVOID firmw_img, UINT firmw_len );
	void	ShutdownModem( void );
	void	Indicate( NDIS_STATUS const Stat ) const;
	void
	DisableInterrupts( void )	{ HLDC->SetIntMask( 0 ); }

	void
	EnableInterrupts( void )	{ HLDC->SetIntMask( EXT | RXS | TXS | OFL | UFL ); }

	void	DisableReceiver( void );
	void	EnableReceiver( void );
	void	ResetTransceiver( void );
	void	StartTransceiver( void );
	void	FillXmtQueue( void );
	void	CompleteTxPackets( void );
	void	CancelTxPackets( void );
	void	FillRcvQueue( void );
	void	CompleteRxPackets( void );
	void	CancelRxPackets( void );
	NDIS_STATUS ReadConfiguration( NDIS_HANDLE ConfigHandle );
	NDIS_STATUS CardStart( void );
	void	ReadModemStat( DWORD_PTR Stage );
	void	ReadModemStatComplete( bool Success=true );
	void	ResetModemStat( DWORD_PTR Stage );
	void	ResetModemStatComplete( bool Success=true );
	VOID	MiniportIsr( PBOOLEAN InterruptRecognized, PBOOLEAN QueueDPC );
	VOID	MiniportHandleInterrupt( void );
	VOID	MiniportSendPackets( PPNDIS_PACKET p, UINT count );
	VOID	MiniportReturnPacket( PNDIS_PACKET pkt );
	NDIS_STATUS MiniportQueryInformation( NDIS_OID Oid, PVOID InformationBuffer,
										  ULONG InformationBufferLength,
										  PULONG BytesWritten, PULONG BytesNeeded );
	NDIS_STATUS MiniportSetInformation( NDIS_OID Oid, PVOID InformationBuffer,
										ULONG OidLength, PULONG BytesRead,
										PULONG BytesNeeded );
	VOID	MiniportHalt( void );
	NDIS_STATUS MiniportReset( PBOOLEAN need_set_info );
	VOID	MiniportShutdown( void );
	BOOLEAN MiniportCheckForHang( void );
	void
	Check( void ) const
	{
#ifdef _DEBUG
		Assert( IsValidPtr(this, sizeof(*this)) );
		if( TxDmaBuffer || RxDmaBuffer || HLDC || cmdp )
		{
			Assert( PVOID(TxDmaBuffer) == MemoryWindowAddr );
			Assert( PVOID(RxDmaBuffer) == PBYTE(MemoryWindowAddr) + 0x400 );
			Assert( PVOID(HLDC) == PBYTE(MemoryWindowAddr) + 0x800 );
			Assert( PVOID(cmdp) == PBYTE(MemoryWindowAddr) + 0xC00 );
		}

		Assert( IntDepth == 0 || IntDepth == 1 );
		Assert( MapRegsAllocated == false || MapRegsAllocated == true );
		Assert( PacketPoolAllocated == false || PacketPoolAllocated == true );
		Assert( BufferPoolAllocated == false || BufferPoolAllocated == true );
		Assert( InterruptRegistered == false || InterruptRegistered == true );
		if( DmaMemAddr )
		{
			Assert( DmaMemSize == (XmtQueueLen + RcvQueueLen) * DMA_BUFFER_SIZE );
			Assert( XmtQueueLen <= NUM_HLDC_RING_ELEMS &&
					TxRingRemovePos < NUM_HLDC_RING_ELEMS );
			Assert( RcvQueueLen <= NUM_HLDC_RING_ELEMS &&
					RxRingRemovePos < NUM_HLDC_RING_ELEMS );
		}

		SL.Check();
		XmtDmaPool.Check();
		XmtDmaQueue.Check();
		RcvDmaPool.Check();
		RcvDmaQueue.Check();
		PacketsFromNdis.Check();
#endif
	}
};
typedef AdapterDesc SG16_ADAPTER, *PSG16_ADAPTER;
void	MiniportShutdownOuter( NDIS_HANDLE Context );
