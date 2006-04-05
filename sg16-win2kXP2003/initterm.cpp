#include "common.h"





// ###################################################################
//
// AdapterDesc class functions
// ===========================
//
// ###################################################################





//
// Constructor
// ===========
//

AdapterDesc::AdapterDesc (void)

#ifdef _DEBUG

  : SL (this),
  XmtDmaPool (this), XmtDmaQueue (this),
  RcvDmaPool (this), RcvDmaQueue (this),
  PacketsFromNdis (this)

#endif

{

  DriverHandle = NULL;

#ifdef _DEBUG

  IntDepth = 0;

  InCompleteTxDepth = 0;
  InCompleteRxDepth = 0;

  TotalReceivedBytes = 0;

#endif

  LastIntEvents = 0;

  PacketPoolAllocated = BufferPoolAllocated = false;
  InterruptRegistered = MapRegsAllocated = false;

  PacketPool = BufPool = NULL;

  HLDC = NULL;
  TxDmaBuffer = RxDmaBuffer = NULL;
  cmdp = NULL;

  NdisZeroMemory (&Stat, sizeof (Stat));

  XmitGood = RcvdGood = XmitBad = CrcErrors = MulticastCount = 0;

  IntCount = 0;

  MemoryWindowAddr = NULL;
  MemoryWindowPhysAddr = 0;

  DmaMemAddr = NULL;
  DmaMemSize = 0;

  XmtQueueLen = Driver_XmtQueueLenDef;
  RcvQueueLen = Driver_RcvQueueLenDef;

  TxRingRemovePos = RxRingRemovePos = 0;

  InterruptNumber = 0;

  ModemState = DOWN;

  ModemComplRtn = NULL;

  QI_InfoBuffer = NULL;

  NdisZeroMemory (MACAddress, sizeof (MACAddress));
  NdisZeroMemory (MulticastList, sizeof (MulticastList));

  FilterMode = 0;

  NdisInitializeEvent (&cx_intr);

  Reset ();

  Debug (7, this, "Created adapter object %X", this);

}





//
// Destructor
// ==========
//

AdapterDesc::~AdapterDesc (void) {

  Debug (7, this, "Destroying adapter object %X", this);

  Assert (XmtDmaPool.IsEmpty () && XmtDmaQueue.IsEmpty ());
  Assert (RcvDmaPool.IsEmpty () && RcvDmaQueue.IsEmpty ());
  Assert (PacketsFromNdis.IsEmpty ());

}



//
// Reset to initial state
// ======================
//

void AdapterDesc::Reset (void) {

}


//
// Initialize the modem hardware
// =============================
//

NDIS_STATUS AdapterDesc::CardStart (void) {

  Debug (0, this, "CardStart:ENTER");

  Check ();

  Debug (7, this, "Initializing the modem");

  HLDC->CRA = 0;
  HLDC->ResetStatusBits (ALL);
  DisableInterrupts ();

  // read the 8051 program code

  NDIS_STRING file_name = NDIS_STRING_CONST ("sg16.bin");

  NDIS_HANDLE handle;

  bool FileOpen = false;
  PVOID firmw_img = NULL;
  NDIS_STATUS Status;

	do {
		UINT firmw_len;
		Assert (KeGetCurrentIrql () == PASSIVE_LEVEL);
		NdisOpenFile (&Status, &handle, &firmw_len, &file_name, HighestAddr);

		if (Status != NDIS_STATUS_SUCCESS) {
			Debug (9, this, "CardStart:ERROR - NdisOpenFile (%X)", Status);
			break;
		}

		FileOpen = true;

		NdisMapFile (&Status, &firmw_img, handle);

		if (Status != NDIS_STATUS_SUCCESS) {
			Debug (9, this, "CardStart:ERROR - NdisMapFile (%X)", Status);
			break;

		}

    if (!start_cx28975 (firmw_img, firmw_len)) {
		Status = NDIS_STATUS_FAILURE;
		break;
    }


    StartTransceiver ();

    Status = NDIS_STATUS_SUCCESS;

  } while (False);

  if (FileOpen) {

    if (firmw_img) {

      NdisUnmapFile (handle);

    }

    NdisCloseFile (handle);

  }

  Debug (0, this, "CardStart:EXIT");

  Check ();

  return Status;

}





//
// Create packet pools/queues
// ==========================
//

NDIS_STATUS AdapterDesc::CreatePools (void) {

  Debug (0, this, "CreatePools: ENTER");

  Check ();

  NDIS_STATUS Status = NDIS_STATUS_FAILURE;

  do {

    NdisAllocatePacketPool (&Status, &PacketPool, RcvQueueLen, 16);

    //NdisAllocatePacketPoolEx (&Status, &PacketPool, NUMBER_OF_MAP_REGS, 2, 16);

    if (Status != NDIS_STATUS_SUCCESS) {

      Debug (9, this, "MiniportInitialize:ERROR - NdisAllocatePacketPool (%X)", Status);

      break;

    }

    PacketPoolAllocated = true;

    Debug (7, this, "Allocated packet pool %X", PacketPool);


    NdisAllocateBufferPool (&Status, &BufPool, RcvQueueLen);

    if (Status != NDIS_STATUS_SUCCESS) {

      Debug (9, this, "MiniportInitialize:ERROR - NdisAllocateBufferPool (%X)", Status);

      break;

    }

    BufferPoolAllocated = true;

    Debug (7, this, "Allocated buffer pool %X", BufPool);


    // Initialize DMA pools

    PBYTE VA = PBYTE (DmaMemAddr);
    DWORD PA = NdisGetPhysicalAddressLow (DmaMemPhysAddr);

    for (UINT i = 0; i < XmtQueueLen; i++) {

      Lock ();

      DmaPacketDesc * const DPD = XmtDmaPool.CreateElem (VA, PA, NULL);

      Unlock ();

      if (!DPD) {

        break;

      }

      VA += DMA_BUFFER_SIZE;
      PA += DMA_BUFFER_SIZE;

    }

    if (i < XmtQueueLen) {

      Status = NDIS_STATUS_RESOURCES;

      break;

    }

    Debug (7, this, "Created transmission DMA pool of %u elems", XmtQueueLen);


    for (i = 0; i < RcvQueueLen; i++) {

      PNDIS_PACKET Pkt;

      NdisAllocatePacket (&Status, &Pkt, PacketPool);

      Assert (Status == NDIS_STATUS_SUCCESS);

      Debug (7, this, "Allocated packet desc %X", Pkt);

      PNDIS_BUFFER Buf;

      NdisAllocateBuffer (
        &Status,
        &Buf,
        BufPool,
        VA,
        ETHERNET_MAX_SIZE
      );

      Assert (Status == NDIS_STATUS_SUCCESS);

      Debug (5, this, "Allocated buffer desc %X (VA %X)", Buf, VA);

      NdisChainBufferAtFront (Pkt, Buf);

      Lock ();

      DmaPacketDesc * const DPD = RcvDmaPool.CreateElem (VA, PA, Pkt);

      Unlock ();

      if (!DPD) {

        NdisFreeBuffer (Buf);
        NdisFreePacket (Pkt);

        break;

      }

      NDIS_SET_PACKET_HEADER_SIZE (Pkt, ETHERNET_HEADER_SIZE);

      PacketAddon::FromPkt (Pkt)->DPD = DPD;

      VA += DMA_BUFFER_SIZE;
      PA += DMA_BUFFER_SIZE;

    }

    if (i < RcvQueueLen) {

      Status = NDIS_STATUS_RESOURCES;

      break;

    }

    Debug (7, this, "Created reception DMA pool of %u elems", RcvQueueLen);

    Assert (VA == PBYTE (DmaMemAddr) + DmaMemSize);
    Assert (PA == NdisGetPhysicalAddressLow (DmaMemPhysAddr) + DmaMemSize);

    Status = NDIS_STATUS_SUCCESS;

  } while (False);

  if (Status != NDIS_STATUS_SUCCESS) {

    DestroyPools ();

  }

  Debug (0, this, "CreatePools: EXIT");

  return Status;

}





//
// Destroy packet pools/queues
// ===========================
//

void AdapterDesc::DestroyPools (void) {

  Check ();

  Assert (HLDC->IMR == 0);

  Assert (!IsLocked ());

  Lock ();

  XmtDmaQueue.DestroyAll ();
  XmtDmaPool.DestroyAll ();

  Unlock ();

  Assert (RcvDmaQueue.IsEmpty ());

  while (!RcvDmaPool.IsEmpty ()) {

    Lock ();

    DmaPacketDesc * const DPD = RcvDmaPool.RemoveFirst ();

    Unlock ();

    PNDIS_PACKET const Pkt = DPD->Pkt;

    Assert (Pkt);

    delete DPD;

    PNDIS_BUFFER Buf;

    NdisQueryPacket (Pkt, NULL, NULL, &Buf, NULL);

    Assert (Buf);

    Debug (7, this, "Freeing buffer desc %X", Buf);

    NdisFreeBuffer (Buf);

    Debug (7, this, "Freeing packet %X", Pkt);

    NdisFreePacket (Pkt);

  }



  if (BufferPoolAllocated) {

    Debug (7, this, "Freeing buffer pool %X", BufPool);

    NdisFreeBufferPool (BufPool);

    BufferPoolAllocated = false;

  }

  if (PacketPoolAllocated) {

    Debug (7, this, "Freeing packet pool %X", PacketPool);

    NdisFreePacketPool (PacketPool);

    PacketPoolAllocated = false;

  }


  Check ();

}





//
// Initialize the DMA/interrupt support
// ====================================
//

NDIS_STATUS AdapterDesc::InitDmaAndInt (void) {

	Debug (0, this, "InitDmaAndInt: ENTER");

	Check ();

	NDIS_STATUS Status = NDIS_STATUS_FAILURE;

	do{
		LARGE_INTEGER PhysAddr = { 0, 0 };
		PhysAddr.u.LowPart = MemoryWindowPhysAddr;
		Status = NdisMMapIoSpace (&MemoryWindowAddr,DriverHandle,
									PhysAddr,SG16_MEMSIZE);
		
		if (Status != NDIS_STATUS_SUCCESS) {
			Debug (9, this, "RegisterAdapter:ERROR - NdisMMapIoSpace (%X)", Status);
			break;
		}

		Debug (7, this, "I/O space mapped to %X", MemoryWindowAddr);

		TxDmaBuffer = (DmaBuffer *) (MemoryWindowAddr);
		RxDmaBuffer = (DmaBuffer *) (PBYTE (MemoryWindowAddr) + 0x400);
		HLDC = (HldcRegs *) (PBYTE (MemoryWindowAddr) + 0x800);
		cmdp = (cx28975_cmdarea *) (PBYTE (MemoryWindowAddr) + 0xC00);

		DmaMemSize = (XmtQueueLen + RcvQueueLen) * DMA_BUFFER_SIZE;

#ifdef _DEBUG
		NdisZeroMemory (TxDmaBuffer->BdArray, sizeof (TxDmaBuffer->BdArray));
		NdisZeroMemory (RxDmaBuffer->BdArray, sizeof (RxDmaBuffer->BdArray));
#endif
 
 
    // We must allocate map registers to use shared memory
		Status = NdisMAllocateMapRegisters ( DriverHandle,0,NDIS_DMA_32BITS,
												1,DmaMemSize );

		if (Status != NDIS_STATUS_SUCCESS) {
			Debug (9, this, "Cannot allocate map registers (%X)", Status);
			break;
		}

		MapRegsAllocated = true;
		Debug (7, this, "Allocated map registers");
		NdisMAllocateSharedMemory ( DriverHandle, DmaMemSize, TRUE,
									&DmaMemAddr, &DmaMemPhysAddr );

		if (!DmaMemAddr) {
			Debug (9, this, "Cannot allocate %u bytes of DMA memory", DmaMemSize);
			Status = NDIS_STATUS_RESOURCES;
			break;
		}

		Debug (7,this,"Allocated %u bytes of DMA mem (virt %X, phys %X)",DmaMemSize,
						DmaMemAddr,DmaMemPhysAddr);

		Status = CreatePools ();

		if (Status != NDIS_STATUS_SUCCESS) {
			break;
		}


		// interrupts are disabled until CardStart completed
		Status = NdisMRegisterInterrupt ( &Interrupt,DriverHandle,InterruptNumber,
										InterruptNumber,TRUE,TRUE,NdisInterruptLevelSensitive);

		if (Status != NDIS_STATUS_SUCCESS) {
			Debug (9, this, "Cannot register interrupt (%X)", Status);
			NdisWriteErrorLogEntry (DriverHandle,NDIS_ERROR_CODE_INTERRUPT_CONNECT,0);
			break;
		}

		Debug (7, this, "Registered interrupt %u", InterruptNumber);

		InterruptRegistered = true;

		Status = CardStart ();
		if (Status != NDIS_STATUS_SUCCESS) {
			NdisWriteErrorLogEntry (DriverHandle, Status, 0);
			break;
		}
		HLDC->CTDR = HLDC->LTDR = HLDC->CRDR = HLDC->LRDR = 0;
		Assert (Status == NDIS_STATUS_SUCCESS);
	}while (False);
	
	if (Status != NDIS_STATUS_SUCCESS) {
		TermDmaAndInt ();
	}

	Debug (0, this, "InitDmaAndInt: EXIT");

	return Status;
}





//
// Terminate the DMA/Interrupt support
// ===================================
//

void AdapterDesc::TermDmaAndInt (void) {

  Debug (0, this, "TermDmaAndInt: ENTER");


  if (InterruptRegistered) {

    Debug (7, this, "Deregistering interrupt %u", InterruptNumber);

    NdisMDeregisterInterrupt (&Interrupt);

    InterruptRegistered = false;

  }


  DestroyPools ();


  if (DmaMemAddr) {

    Debug (
      7,
      this,
      "Freeing rcv DMA mem (virt %X, phys %X)",
      DmaMemAddr,
      DmaMemPhysAddr
    );

    NdisMFreeSharedMemory (
      DriverHandle,
      DmaMemSize,
      FALSE,
      DmaMemAddr,
      DmaMemPhysAddr
    );

    DmaMemAddr = NULL;

  }


  if (MapRegsAllocated) {

    NdisMFreeMapRegisters (DriverHandle);

    MapRegsAllocated = false;

  }


  if (MemoryWindowAddr) {

    Debug (7, this, "Unmapping I/O space");

    NdisMUnmapIoSpace (DriverHandle, MemoryWindowAddr, SG16_MEMSIZE);

    MemoryWindowAddr = NULL;

    TxDmaBuffer = RxDmaBuffer = NULL;
    HLDC = NULL;
    cmdp = NULL;

  }

  Debug (0, this, "TermDmaAndInt: EXIT");

}





//
// Initialization
// ==============
//

NDIS_STATUS AdapterDesc::Init (	NDIS_HANDLE MPH,
								NDIS_HANDLE WrapperConfigurationContext)
{

	DriverHandle = MPH;
	
	NdisMRegisterAdapterShutdownHandler ( DriverHandle,this,
										MiniportShutdownOuter);

	NdisMSetAttributesEx (DriverHandle,NDIS_HANDLE (this),0,
				NDIS_ATTRIBUTE_DESERIALIZE | NDIS_ATTRIBUTE_BUS_MASTER,
				NdisInterfacePci);

	NDIS_STATUS Status = NDIS_STATUS_FAILURE;

	do {

		NDIS_HANDLE ConfigHandle;
		NdisOpenConfiguration (&Status, &ConfigHandle, WrapperConfigurationContext);

		if (Status != NDIS_STATUS_SUCCESS) {
			Debug (9, this, "Cannot open configuration (%X)", Status);
			Status = NDIS_STATUS_FAILURE;
			break;
		}

		Status = ReadConfiguration (ConfigHandle);
		NdisCloseConfiguration (ConfigHandle);
		if (Status != NDIS_STATUS_SUCCESS) {
			Debug (9, this, "Cannot read configuration (%X)", Status);
			break;
		}

		Status = InitDmaAndInt ();

		if (Status != NDIS_STATUS_SUCCESS) {
DbgPrint("Init:cannot init DMA & INT\n");      
		break;
		}

		Reset ();

		FillRcvQueue ();

		Status = NDIS_STATUS_SUCCESS;

	}while (False);

	if (Status != NDIS_STATUS_SUCCESS) {
		Term ();
	}

  return Status;

}





//
// Termination
// ===========
//

void AdapterDesc::Term (void) {

  Assert (!IsLocked ());

  if (HLDC) {                   // If HLDC initialized

    Lock ();

    DisableInterrupts ();
    HLDC->CRA = 0;

    CancelRxPackets ();

    Unlock ();

  }

  NdisMDeregisterAdapterShutdownHandler (DriverHandle);

  TermDmaAndInt ();

}
