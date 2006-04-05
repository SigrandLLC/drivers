//$T 1.c GC 1.138 05/26/04 13:02:29

//
// SG16PCI.C Kernel-mode driver for sg16 SHDSL modem
// Last Updated 04/06/2004
//
// Notes
// =====
//
// One channel version!
//
//
// Known bugs
// ==========
//
//
//
// Changes
// =======
//
// Formatted by GC & Eugene Muzychenko (27/05/2004)
//
// Modified to C++ by Eugene Muzychenko (01/06/2004)
//
// Possibly fixed flood/hang problem by Eugene Muzychenko (04/06/2004)
//

#include "common.h"

#include "config.h"





// ###################################################################
//
// Compiler pragmas
// ================
//
// ###################################################################



EXTERN_C NTSTATUS DriverEntry (PDRIVER_OBJECT,PUNICODE_STRING);

#pragma NDIS_INIT_FUNCTION (DriverEntry)





// ###################################################################
//
// AdapterDesc class functions
// ===========================
//
// ###################################################################





//
// Disable receiver
// ================
//

void AdapterDesc::DisableReceiver (void) {

  SetBits (HLDC->CRB, RXDE);

  Debug (2, this, "HLDC receiver disabled");

}





//
// Enable receiver
// ===============
//

void AdapterDesc::EnableReceiver (void) {

  ResetBits (HLDC->CRB, RXDE);

  Debug (2, this, "HLDC receiver enabled");

}





//
// Reset entire transceiver
// ========================
//

void AdapterDesc::ResetTransceiver (void) {

  ResetBits (HLDC->CRA, TXEN | RXEN);
  DisableReceiver ();

}





//
// Enanle transceiver
// ==================
//

void AdapterDesc::StartTransceiver (void) {

  Assert (!TestBits (HLDC->CRA, TXEN | RXEN));

  HLDC->CTDR = HLDC->LTDR = HLDC->CRDR = HLDC->LRDR = 0;

  SetBits (HLDC->CRA, TXEN | RXEN);

}





//
// Indicate status change
// ======================
//

void AdapterDesc::Indicate (NDIS_STATUS const Stat) const {

  Check ();

  NdisMIndicateStatus (DriverHandle, Stat, NULL, 0);

  NdisMIndicateStatusComplete (DriverHandle);

}





//
// Fill transmitter DMA queue
// ==========================
//
// MUST be called at spin lock acquired
//

void AdapterDesc::FillXmtQueue (void) {

  Check ();

  Assert (IsLocked ());

  Assert (InterruptRegistered); // We are in initialized state

  Assert (TestBits (HLDC->CRA, TXEN));

  UINT Tbd = HLDC->GetLTDR ();

  while (
    !PacketsFromNdis.IsEmpty () // Have queued packets from NDIS
    && !XmtDmaPool.IsEmpty ()   // Have free DMA packet descriptors
    && HLDC->NextBD (Tbd) != TxRingRemovePos // Have room in HLDC ring
  ) {

    NDIS_PACKET * const Pkt = PacketsFromNdis.RemoveFirst ();

    PNDIS_BUFFER Buf;

    NdisQueryPacket (Pkt, NULL, NULL, &Buf, NULL);
    
    //!!! Check if packet is empty?

    DmaPacketDesc * const DPD = XmtDmaPool.RemoveFirst ();


    // Copy the packet buffer chain to contiguous DMA buffer

    UINT DmaBufferPos = 0;

    while (Buf) {

      PVOID PktBufferAddr;
      UINT PktBufferLen;

      NdisQueryBuffer (Buf, &PktBufferAddr, &PktBufferLen);

      Assert (PktBufferLen <= ETHERNET_MAX_SIZE - DmaBufferPos);

      NdisMoveMemory (
        PBYTE (DPD->VirtAddr) + DmaBufferPos,
        PktBufferAddr,
        PktBufferLen
      );

      DmaBufferPos += PktBufferLen;

      NdisGetNextBuffer (Buf, &Buf);

    }


    // Pad short packets to minimal allowed length

    if (DmaBufferPos < ETHERNET_MIN_SIZE) {

      UINT const PadBytes = ETHERNET_MIN_SIZE - DmaBufferPos;

      NdisZeroMemory (PBYTE (DPD->VirtAddr) + DmaBufferPos, PadBytes);

      DmaBufferPos = ETHERNET_MIN_SIZE;

    }


    // Fill current TBD with packet data

    DmaBufferDesc * const BD = TxDmaBuffer->BdArray + Tbd;

    Assert (!BD->PhysAddr && !BD->Length);

    BD->PhysAddr = DPD->PhysAddr;
    BD->Length = DmaBufferPos | DB_LAST; // Packet never can be fragmented

    Debug (
      3,
      this,
      "Tx pkt %X VA %X Sz %4u added",
      Pkt, DPD->VirtAddr, DmaBufferPos
    );


    // Save packet pointer to further access

    Assert (!DPD->Pkt);

    DPD->Pkt = Pkt;


    // Save corresponding BD index

    IfDebug (DPD->BdIndex = Tbd);


    // Append to DMA queue

    XmtDmaQueue.Append (DPD);

    Tbd = HLDC->NextBD (Tbd);

  } // main loop


  // Pass updated TBD pos to the HLDC

  HLDC->LTDR = BYTE (Tbd);

}





//
// Complete packets in tx queue
// ============================
//

void AdapterDesc::CompleteTxPackets (void) {

  Check ();

  Assert (IsLocked ());

  Assert (NdisInterlockedIncrement (&InCompleteTxDepth) == 1);

  while (TxRingRemovePos != HLDC->GetCTDR ()) {

    Assert (TxRingRemovePos != HLDC->GetLTDR ());

    IfDebug (DmaBufferDesc * const BD = TxDmaBuffer->BdArray + TxRingRemovePos);

    Assert (TestBits (BD->Length, DB_DONE)); // Check if really done

    Assert (!XmtDmaQueue.IsEmpty ());

    DmaPacketDesc * const DPD = XmtDmaQueue.RemoveFirst ();

    Assert (TxRingRemovePos == DPD->BdIndex); // Check if synchronized
    
    PNDIS_PACKET const Pkt = DPD->Pkt;

    IfDebug (DPD->Pkt = NULL);  // Clear the packet pointer

#ifdef _DEBUG

    UINT Size;

    NdisQueryPacket (Pkt, NULL, NULL, NULL, &Size);

    AssertIf (Size >= ETHERNET_MIN_SIZE, (BD->Length & DB_LENMASK) == Size); // Check if size matches

    BD->PhysAddr = BD->Length = 0; // Clear the descriptor

#endif

    Debug (
      4,
      this,
      "Tx pkt %X VA %X Sz %4u completed, rest %2u",
      Pkt, DPD->VirtAddr, Size, XmtDmaQueue.GetCount ()
    );

    XmitGood++;

    XmtDmaPool.Append (DPD);

    TxRingRemovePos = HLDC->NextBD (TxRingRemovePos);

    Unlock ();

    NdisMSendComplete (DriverHandle, Pkt, NDIS_STATUS_SUCCESS);

    Lock ();

  }

  Assert (NdisInterlockedDecrement (&InCompleteTxDepth) == 0);

}





//
// Cancel all packets in tx queue
// ==============================
//

void AdapterDesc::CancelTxPackets (void) {

  Check ();

  Assert (IsLocked ());

  Assert (!TestBits (HLDC->IMR, TXS) && !TestBits (HLDC->CRA, TXEN));

  Assert (NdisInterlockedIncrement (&InCompleteTxDepth) == 1);

  while (!XmtDmaQueue.IsEmpty ()) {

    DmaPacketDesc * const DPD = XmtDmaQueue.RemoveFirst ();

#ifdef _DEBUG

    DmaBufferDesc * const BD = TxDmaBuffer->BdArray + DPD->BdIndex;

    BD->PhysAddr = BD->Length = 0; // Clear the descriptor

#endif

    PNDIS_PACKET const Pkt = DPD->Pkt;

    IfDebug (DPD->Pkt = NULL);

    Debug (3, this, "Canceling Tx pkt %X", Pkt);

    Unlock ();

    NdisMSendComplete (DriverHandle, Pkt, NDIS_STATUS_REQUEST_ABORTED);

    Lock ();

    XmtDmaPool.Append (DPD);

  }

  HLDC->CTDR = HLDC->LTDR = 0;

  TxRingRemovePos = 0;

  Assert (NdisInterlockedDecrement (&InCompleteTxDepth) == 0);

}





//
// Fill reception queue with the packets from packet pool
// ======================================================
//

void AdapterDesc::FillRcvQueue (void) {

  Check ();

  Lock ();

  Assert (InterruptRegistered); // We are in initialized state

  UINT Rbd = HLDC->GetLRDR ();

  while (
    !RcvDmaPool.IsEmpty ()      // Have free packet descs
    && HLDC->NextBD (Rbd) != RxRingRemovePos // Have room in the HLDC ring
  ) {

    DmaPacketDesc * const DPD = RcvDmaPool.RemoveFirst ();

    PNDIS_PACKET const Pkt = DPD->Pkt;

    Assert (PacketAddon::FromPkt (Pkt)->DPD == DPD);

    PNDIS_BUFFER Buf;

    NdisQueryPacket (Pkt, NULL, NULL, &Buf, NULL);

    NdisAdjustBufferLength (Buf, ETHERNET_MAX_SIZE);


    // Fill current RBD with packet data

    DmaBufferDesc * const BD = RxDmaBuffer->BdArray + Rbd;

    Assert (!BD->PhysAddr && !BD->Length);

    BD->PhysAddr = DPD->PhysAddr;

    Debug (
      3,
      this,
      "Rx pkt %X VA %X posted",
      Pkt, DPD->VirtAddr
    );


    // Save corresponding BD index

    IfDebug (DPD->BdIndex = Rbd);


    // Advance positions in the rings

    RcvDmaQueue.Append (DPD);

    Rbd = HLDC->NextBD (Rbd);

  } // main loop


  // Pass updated RBD pos to the HLDC

  HLDC->LRDR = BYTE (Rbd);

  Unlock ();

}





//
// Complete packets in rx queue
// ============================
//

void AdapterDesc::CompleteRxPackets (void) {

  Check ();

  Assert (IsLocked ());

  Assert (NdisInterlockedIncrement (&InCompleteRxDepth) == 1);

  PNDIS_PACKET PktArray [NUM_HLDC_RING_ELEMS];

  UINT PktCount = 0;

  UINT FirstResIdx = UINT (-1); // First packet with STATUS_RESOURCES

  while (RxRingRemovePos != HLDC->GetCRDR ()) {

    Assert (RxRingRemovePos != HLDC->GetLRDR ());

    Assert (!RcvDmaQueue.IsEmpty ());

    DmaPacketDesc * const DPD = RcvDmaQueue.RemoveFirst ();

    Assert (RxRingRemovePos == DPD->BdIndex); // Check if synchronized
    
    DmaBufferDesc * const BD = RxDmaBuffer->BdArray + RxRingRemovePos;

    Assert (TestBits (BD->Length, DB_DONE)); // Check if really done
    Assert (!TestBits (BD->Length, DB_ERROR)); // Check if there are no error

    UINT const ReceivedBytes = BD->Length & DB_LENMASK;

    Assert (ReceivedBytes <= ETHERNET_MAX_SIZE); // Check if no overflow

    IfDebug (BD->PhysAddr = BD->Length = 0); // Clear the descriptor

    PNDIS_PACKET const Pkt = DPD->Pkt;

    Assert (PacketAddon::FromPkt (Pkt)->DPD == DPD);

    if (FilterMode) {           // We must ignore all packets if FM=0

      NdisMUpdateSharedMemory(
        DriverHandle,
        ReceivedBytes,
        DPD->VirtAddr,
        DPD->PhysAddr
      );

      PNDIS_BUFFER Buf;

      NdisQueryPacket (Pkt, NULL, NULL, &Buf, NULL);
  
#ifdef _DEBUG

      PVOID PktBufferAddr;
      UINT PktBufferLen;

      NdisQueryBuffer (Buf, &PktBufferAddr, &PktBufferLen);

      Assert (PktBufferAddr == DPD->VirtAddr);

#endif

      Unlock ();

      NdisAdjustBufferLength (Buf, ReceivedBytes);

      Lock ();

      Assert (NDIS_GET_PACKET_HEADER_SIZE (Pkt) == ETHERNET_HEADER_SIZE);

      UINT const RestPackets = RcvDmaQueue.GetCount () + RcvDmaPool.GetCount ();

      bool const Enough = (RestPackets >= (RcvQueueLen + 2) / 3);

      if (Enough) {             // if not less than 30% left

        NDIS_SET_PACKET_STATUS (Pkt, NDIS_STATUS_SUCCESS);

      } else {

        NDIS_SET_PACKET_STATUS (Pkt, NDIS_STATUS_RESOURCES);

        if (FirstResIdx != UINT (-1)) {

          FirstResIdx = PktCount;

        }

      }

      Debug (
        4,
        this,
        "Rx pkt %X VA %X Sz %4u completed, rest %2u (%s)",
        Pkt, DPD->VirtAddr, ReceivedBytes, RestPackets, Enough? "succ" : "res"
      );

      //Debug (7, this, "Received %u bytes", ReceivedBytes);

      IfDebug (TotalReceivedBytes += ReceivedBytes);

      PktArray [PktCount] = Pkt;

      PktCount++;

      Assert (PktCount <= RcvQueueLen);

    } else {

      Debug (3, this, "Rx pkt %X Sz %4u dropped", Pkt, ReceivedBytes);

      RcvDmaPool.Append (DPD);

    }

    RcvdGood++;

    RxRingRemovePos = HLDC->NextBD (RxRingRemovePos);

  }

  Assert (NdisInterlockedDecrement (&InCompleteRxDepth) == 0);

  if (PktCount) {

    Debug (3, this, "Indicating %u received packets", PktCount);

    NdisMIndicateReceivePacket (DriverHandle, PktArray, PktCount);

    if (FirstResIdx != UINT (-1)) {

      Debug (3, this, "Self-returning %u critical packets", PktCount - FirstResIdx);

      for (UINT i = FirstResIdx; i < PktCount; i++) {

        RcvDmaPool.Append (PacketAddon::FromPkt (PktArray [i])->DPD);

      }

    }

  }

}





//
// Cancel packets in rx queue
// ==========================
//

void AdapterDesc::CancelRxPackets (void) {

  Check ();

  Assert (IsLocked ());

  Assert (NdisInterlockedIncrement (&InCompleteRxDepth) == 1);

  Assert (!TestBits (HLDC->IMR, RXS) && !TestBits (HLDC->CRA, RXEN));

  while (!RcvDmaQueue.IsEmpty ()) {

    DmaPacketDesc * const DPD = RcvDmaQueue.RemoveFirst ();

#ifdef _DEBUG

    DmaBufferDesc * const BD = RxDmaBuffer->BdArray + DPD->BdIndex;

    BD->PhysAddr = BD->Length = 0; // Clear the descriptor

    PNDIS_PACKET const Pkt = DPD->Pkt;

    Assert (PacketAddon::FromPkt (Pkt)->DPD == DPD);

    Debug (3, this, "Canceling Rx pkt %X", Pkt);

#endif

    RcvDmaPool.Append (DPD);

  }

  HLDC->CRDR = HLDC->LRDR = 0;

  RxRingRemovePos = 0;

  Assert (NdisInterlockedDecrement (&InCompleteRxDepth) == 0);

}





// ###################################################################
//
// NDIS interface functions
// ========================
//
// ###################################################################





//
// Serve the hardware interrupts at DIRQL
// ======================================
//

inline VOID AdapterDesc::MiniportIsr (
  PBOOLEAN InterruptRecognized,
  PBOOLEAN QueueDPC
) {

  BYTE Events = HLDC->TestStatusBits (HLDC->IMR); // Save event bits

  HLDC->ResetStatusBits (Events);

  do {

    if (Events == 0) {

      *InterruptRecognized = FALSE;
      *QueueDPC = FALSE;

      Debug (0, this, "Interrupt not recognized");

      break;

    }

    Debug (0, this, "Interrupt recognized: %02X", Events);

    Assert (IntDepth == 0);

    *InterruptRecognized = TRUE;

    IntCount++;

    if (TestBits (Events, CRC)) {

      Debug (9, this, "Receiver CRC error");

      ++CrcErrors;

      ResetBits (Events, CRC);

    }

    if (TestBits (Events, OFL)) {

      Debug (9, this, "Receiver overflow");

      ++Stat.ofl_errs;

      ResetBits (Events, OFL);

    }

    if (Events) {

      Assert (LastIntEvents == 0); // Check if all previous events are processed

      LastIntEvents = Events;   // Save event bits for DPC

      DisableInterrupts ();

      *QueueDPC = TRUE;

    }

  } while (False);

}

VOID MiniportIsrOuter (
  PBOOLEAN InterruptRecognized,
  PBOOLEAN QueueDPC,
  NDIS_HANDLE Context
) {

  PSG16_ADAPTER (Context)->MiniportIsr (InterruptRecognized, QueueDPC);

}






//
// Serve the hardware interrupt in deferred mode at DISPATCH_LEVEL
// ===============================================================
//

inline VOID AdapterDesc::MiniportHandleInterrupt (void) {

  Debug (0, this, "MiniportHandleInterrupt:ENTER");

  Assert (KeGetCurrentIrql () == DISPATCH_LEVEL);

  Check ();

  Assert (NdisInterlockedIncrement (&IntDepth) == 1);

  Lock ();

  if (TestBits (LastIntEvents, EXT)) {

    Debug (1, this, "Modem event");

    ModemStates const PrevState = ModemState;

    cx28975_interrupt ();

    if (ModemState != PrevState) {

      if (ModemState == ACTIVE) {

        HLDC->ResetStatusBits (ALL);

        EnableReceiver ();

        Stat.attempts++;

        NdisGetCurrentSystemTime (&Stat.last_time);

        //!!!Unlock ();

        if (!ModemCfg.AlwaysConnected) {

          Indicate (NDIS_STATUS_MEDIA_CONNECT);

        }

        //!!!Lock ();

      } else if (PrevState == ACTIVE) {

        DisableReceiver ();

        //!!!Unlock ();

        if (!ModemCfg.AlwaysConnected) {

          Indicate (NDIS_STATUS_MEDIA_DISCONNECT);

        }

        //!!!Lock ();

      }

    }

    ResetBits (LastIntEvents, EXT);

  }

  if (TestBits (LastIntEvents, UFL)) {

    Debug (2, this, "Transmitter FIFO underflow event");

    HLDC->CRA |= TXEN;

    ++XmitBad;
    ++Stat.ufl_errs;

    ResetBits (LastIntEvents, UFL);

  }

  if (TestBits (LastIntEvents, RXS)) {

    Debug (2, this, "Receiver event");

    CompleteRxPackets ();

    FillRcvQueue ();

    ResetBits (LastIntEvents, RXS);

  }

  if (TestBits (LastIntEvents, TXS)) {

    Debug (2, this, "Transmitter event");

    CompleteTxPackets ();

    FillXmtQueue ();

    ResetBits (LastIntEvents, TXS);

  }

  Assert (LastIntEvents == 0);

  Unlock ();

  Assert (NdisInterlockedDecrement (&IntDepth) == 0);

  EnableInterrupts ();

  Debug (0, this, "MiniportHandleInterrupt:EXIT");

}

VOID MiniportHandleInterruptOuter (NDIS_HANDLE Context) {

  PSG16_ADAPTER (Context)->MiniportHandleInterrupt ();

}





//
// Queue a set of packets to send
// ==============================
//

inline VOID AdapterDesc::MiniportSendPackets (PPNDIS_PACKET Packets, UINT Count) {

  Debug (0, this, "MiniportSendPackets:ENTER");

  Check ();

  Lock ();

  for (UINT i = 0; i < Count; i++) {

    NDIS_PACKET * const Pkt = Packets [i];

#ifdef _DEBUG

    UINT PktLen;

    NdisQueryPacket (Pkt, NULL, NULL, NULL, &PktLen);

    Assert (PktLen <= ETHERNET_MAX_SIZE);

    Debug (3, this, "Packet from NDIS: A=%X, L=%u", Pkt, PktLen);

#endif

    NDIS_SET_PACKET_STATUS (Pkt, NDIS_STATUS_PENDING);

    PacketsFromNdis.Append (Pkt);

  }

  FillXmtQueue ();

  Unlock ();

  Debug (0, this, "MiniportSendPackets:EXIT");

}

VOID MiniportSendPacketsOuter (
  NDIS_HANDLE Context,
  PPNDIS_PACKET p,
  UINT count
) {

  PSG16_ADAPTER (Context)->MiniportSendPackets (p, count);

}





//
// Dequeue next received packet
// ============================
//

inline VOID AdapterDesc::MiniportReturnPacket (PNDIS_PACKET Pkt) {

  Debug (0, this, "MiniportReturnPacket:ENTER");

  Check ();

  Debug (3, this, "NDIS returned packet %X", Pkt);

  Lock ();

  RcvDmaPool.Append (PacketAddon::FromPkt (Pkt)->DPD);

  FillRcvQueue ();

  Unlock ();

  Debug (0, this, "MiniportReturnPacket:EXIT");

}

VOID MiniportReturnPacketOuter (
  NDIS_HANDLE Context,
  PNDIS_PACKET pkt
) {

  PSG16_ADAPTER (Context)->MiniportReturnPacket (pkt);

}





//
// Issue a hardware reset to the NIC and/or reset the driver's software state
// ==========================================================================
//

inline NDIS_STATUS AdapterDesc::MiniportReset (PBOOLEAN need_set_info) {

  Debug (0, this, "MiniportReset:ENTER");

  Check ();

  //IfDebug (Break ());

  Debug (7, this, "Resetting");

  Lock ();

  DisableInterrupts ();
  ResetTransceiver ();

  //!!!Unlock ();

  //Indicate (NDIS_STATUS_MEDIA_DISCONNECT);

  //!!!Lock ();

  //ModemState = DOWN;

  CancelTxPackets ();
  CancelRxPackets ();

  while (!PacketsFromNdis.IsEmpty ()) {

    NDIS_PACKET * const Pkt = PacketsFromNdis.RemoveFirst ();

    NdisMSendComplete (DriverHandle, Pkt, NDIS_STATUS_REQUEST_ABORTED);

  }

  //CardStart ();

  //Reset ();

  FillRcvQueue ();

  StartTransceiver ();

  if (ModemState == ACTIVE) {

    EnableReceiver ();

  }

  EnableInterrupts ();

  Unlock ();

  *need_set_info = FALSE;

  Debug (0, this, "MiniportReset:EXIT");

  return NDIS_STATUS_SUCCESS;

}

NDIS_STATUS MiniportResetOuter (
  PBOOLEAN need_set_info,
  NDIS_HANDLE Context
) {

  return PSG16_ADAPTER (Context)->MiniportReset (need_set_info);

}





//
// restores a NIC to its initial state when the system is shut down,
// whether by the user or because an unrecoverable system error occurred.
// 
// ===================================================================

inline VOID AdapterDesc::MiniportShutdown (void) {

  Debug (0, this, "MiniportShutdown:ENTER");

  Debug (7, this, "Shutting down the NIC");

  DisableInterrupts ();
  HLDC->CRB = RXDE;
  HLDC->CRA = 0;
  HLDC->ResetStatusBits (ALL);

  Debug (0, this, "MiniportShutdown:EXIT");

}

VOID MiniportShutdownOuter (NDIS_HANDLE Context) {

  PSG16_ADAPTER (Context)->MiniportShutdown ();

}





//
// Check the NIC for hangup
// ========================
//

inline BOOLEAN AdapterDesc::MiniportCheckForHang (void) {

  Debug (0, this, "MiniportCheckForHang:ENTER");

  Check ();

  Debug ( 5,this,"Checking for hang: CRA=%X, CRB=%X, SR=%X, IMR=%X, TR=%u",
				HLDC->CRA, HLDC->CRB, HLDC->SR,
				HLDC->IMR, TotalReceivedBytes);

  return FALSE;
  /*
  Lock ();

  if (ModemState != ACTIVE) {

    drop_queues ();

  }

  Unlock ();

  Debug (0, this, "MiniportCheckForHang:EXIT");

  return FALSE;
  */
}

BOOLEAN
MiniportCheckForHangOuter (NDIS_HANDLE Context)
{
  return PSG16_ADAPTER (Context)->MiniportCheckForHang ();
}





//
// Initializing the driver
// =======================
//

EXTERN_C NDIS_STATUS
MiniportInitialize (PNDIS_STATUS OpenErrorStatus, PUINT SelectedMediumIndex,
					PNDIS_MEDIUM MediumArray,UINT MediumArraySize, NDIS_HANDLE MPH,
					NDIS_HANDLE WrapperConfigurationContext)
{

  UNUSED (OpenErrorStatus);

  Debug (0, NULL, "MiniportInitialize for %X: ENTER", MPH);

  NDIS_STATUS Status;

  do {

    for (UINT i = 0; i < MediumArraySize && MediumArray [i] != NdisMedium802_3; ++i);

    if (i == MediumArraySize) {

      Status = NDIS_STATUS_UNSUPPORTED_MEDIA;
      break;

    }

    *SelectedMediumIndex = i;

    PSG16_ADAPTER const Adapter = new AdapterDesc;

    if (!Adapter) {

      Status = NDIS_STATUS_RESOURCES;
      break;

    }

    Status = Adapter->Init (MPH, WrapperConfigurationContext);
    if (Status != NDIS_STATUS_SUCCESS) {

      delete Adapter;

    }

  } while (False);

  Debug (0, NULL, "MiniportInitialize for %X: EXIT", MPH);
  return Status;

}





//
// ===================================================================
// ===================================================================
//

inline VOID AdapterDesc::MiniportHalt (void) {

	Debug (0, this, "MiniportHalt: ENTER");
	Check ();
	Debug (7, this, "Halting the adapter");
	Lock ();
	ResetTransceiver ();
	DisableInterrupts ();
	Unlock ();
	HLDC->CRA = 0;
	HLDC->ResetStatusBits (ALL);
	ModemState = DOWN;
	ShutdownModem ();
	Term ();
	delete this;
	Debug (0, NULL, "MiniportHalt: EXIT");
}

VOID MiniportHaltOuter (NDIS_HANDLE Context) {

  PSG16_ADAPTER (Context)->MiniportHalt ();

}





//
// ===================================================================
// ===================================================================
//

#ifdef _DEBUG

static void DriverUnload (PDRIVER_OBJECT drvObj) {

  UNUSED (drvObj);

  Debug (7, NULL, "Unloading driver module for driver object %X", drvObj);

  Assert (!MemoryBlocksAllocated);

}

#endif





//
// ===================================================================
// ===================================================================
//

EXTERN_C NTSTATUS DriverEntry (
  PDRIVER_OBJECT drvObj,
  PUNICODE_STRING registryPath
) {

#ifdef _DEBUG
	DbgPrint ("\n\n\n");
#endif

	ULONG SysTime;
	NdisGetSystemUpTime (&SysTime);
	srand (SysTime);
	NDIS_HANDLE WrapperHandle;
	NdisMInitializeWrapper (&WrapperHandle, drvObj, registryPath, NULL);
	NDIS_MINIPORT_CHARACTERISTICS sg16;
	NdisZeroMemory (&sg16, sizeof (sg16));
	sg16.Ndis40Chars.Ndis30Chars.MajorNdisVersion = NDIS_MAJOR_VERSION;
	sg16.Ndis40Chars.Ndis30Chars.MinorNdisVersion = NDIS_MINOR_VERSION;
	sg16.Ndis40Chars.Ndis30Chars.CheckForHangHandler = MiniportCheckForHangOuter;
	sg16.Ndis40Chars.Ndis30Chars.HaltHandler = MiniportHaltOuter;
	sg16.Ndis40Chars.Ndis30Chars.HandleInterruptHandler = MiniportHandleInterruptOuter;
	sg16.Ndis40Chars.Ndis30Chars.InitializeHandler = MiniportInitialize;
	sg16.Ndis40Chars.Ndis30Chars.ISRHandler = MiniportIsrOuter;
	sg16.Ndis40Chars.Ndis30Chars.QueryInformationHandler = MiniportQueryInformationOuter;
	sg16.Ndis40Chars.Ndis30Chars.ResetHandler = MiniportResetOuter;
	sg16.Ndis40Chars.Ndis30Chars.SetInformationHandler = MiniportSetInformationOuter;
	sg16.Ndis40Chars.SendPacketsHandler = MiniportSendPacketsOuter;
	sg16.Ndis40Chars.ReturnPacketHandler = MiniportReturnPacketOuter;

	Debug (7, NULL, "Registering miniport for driver object %X", drvObj);

	NDIS_STATUS const Status = NdisMRegisterMiniport( WrapperHandle,&sg16,
															sizeof (sg16) );
	if (Status == NDIS_STATUS_SUCCESS) {
		IfDebug (NdisMRegisterUnloadHandler (WrapperHandle, DriverUnload));
	}else {
	    Debug (9, NULL, "Cannot register miniport (%X)", Status);
	    NdisTerminateWrapper (WrapperHandle, NULL);
	}
	return Status;
}
