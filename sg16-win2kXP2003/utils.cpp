#include "common.h"





// #####################################################################
//
// Constants
// =========
//
// #####################################################################





NDIS_PHYSICAL_ADDRESS const HighestAddr = NDIS_PHYSICAL_ADDRESS_CONST (-1, -1);





// #####################################################################
//
// Variables
// =========
//
// #####################################################################





IfDebug (UINT MemoryBlocksAllocated = 0);





// #####################################################################
//
// Dynamic memory functions
// ========================
//
// #####################################################################





void * _cdecl operator new (size_t Size) {

  void *Block;

  static char Tag [] = "sg16";

  NDIS_STATUS const Status = NdisAllocateMemoryWithTag (
    &Block,
    Size,
    *PULONG (Tag)
  );

  if (Status == NDIS_STATUS_SUCCESS) {

    Debug (2, NULL, "Allocated block %X of %u bytes, total %u", Block, Size, MemoryBlocksAllocated);

#ifdef _DEBUG

    MemoryBlocksAllocated++;

    NdisFillMemory (Block, Size, 0xCC);

#endif

    return Block;

  } else {

    Debug (9, NULL, "Cannot allocate %u bytes (%X)", Status);

    return NULL;

  }

}





void _cdecl operator delete (void *Block) {

  Assert (MemoryBlocksAllocated);

  NdisFreeMemory (Block, 0, 0);

  IfDebug (MemoryBlocksAllocated--);

  Debug (2, NULL, "Released block %X, rest %u", Block, MemoryBlocksAllocated);

}





// #####################################################################
//
// class SpinLock functions
// ========================
//
// #####################################################################





SpinLock::SpinLock (

#ifdef _DEBUG

  AdapterDesc const * const A

#else

  void

#endif

)

#ifdef _DEBUG

  : Adapter (A)

#endif

{

  NdisAllocateSpinLock (&Lock);

  Owner = ULONG (-1);

  Depth = 0;

}





SpinLock::~SpinLock (void) {

  Assert (Owner == ULONG (-1) && Depth == 0);

  NdisFreeSpinLock (&Lock);

}





void SpinLock::Acquire (void) {

  if (!IsOwned ()) {

    // Spin lock is free or owned by other processor

#ifdef _DEBUG

    bool Waiting = false;

    ULONG const O = Owner;

    if (O != ULONG (-1)) {

      Debug (3, Adapter, "Waiting for SL %X held by %u", this, O);

      Waiting = true;

    }

#endif // _DEBUG

    NdisAcquireSpinLock (const_cast <NDIS_SPIN_LOCK *> (&Lock));

    Assert (Depth == 0);

    ULONG Idle, KUT, Processor;  

    NdisGetCurrentProcessorCounts (&Idle, &KUT, &Processor); // Processor can change before spin lock is acquired

    Owner = Processor;

    Debug (Waiting? 3 : 1, Adapter, "Entered SL %X", this);

  }

  Depth++;

}




void SpinLock::Release (void) {

  Assert (IsOwned ());

  if (--Depth == 0) {

    Owner = ULONG (-1);

    Debug (1, Adapter, "Releasing SL %X", this);

    NdisReleaseSpinLock (const_cast <NDIS_SPIN_LOCK *> (&Lock));

  }

}





bool SpinLock::IsOwned (void) const {

  ULONG Idle, KUT, Processor;  

  NdisGetCurrentProcessorCounts (&Idle, &KUT, &Processor);

  bool const Our = (Owner == Processor);

  AssertIf (Our, KeGetCurrentIrql () == DISPATCH_LEVEL && Depth != 0);

  return Our;

}





// #####################################################################
//
// class DmaPacketDesc functions
// =============================
//
// #####################################################################





//
// Construction
// ============
//

DmaPacketDesc::DmaPacketDesc (PVOID VA, DWORD PA, PNDIS_PACKET P) {

  VirtAddr = VA;
  PhysAddr = PA;

  Pkt = P;

  IfDebug (BdIndex = 0);

  Next = NULL;

}





//
// Destruction
// ===========
//

DmaPacketDesc::~DmaPacketDesc (void) {

  Assert (!Next);

}





// #####################################################################
//
// class DmaQueue functions
// ========================
//
// #####################################################################





//
// Construction
// ============
//

DmaQueue::DmaQueue (

#ifdef _DEBUG

  AdapterDesc const * const A

#else

  void

#endif

)

#ifdef _DEBUG

  : Adapter (A)

#endif

{

  Head = Tail = NULL;
  Count = 0;
  
  Check ();

}




 
//
// Destruction
// ===========
//

DmaQueue::~DmaQueue (void) {

  Check ();

  Assert (!Head && !Tail && !Count);

}





//
// Create and add new element
// ==========================
//
// Creates a descriptor pointed to given memory block and packet desc
//

DmaPacketDesc *DmaQueue::CreateElem (PVOID VA, DWORD PA, PNDIS_PACKET Pkt) {

  DmaPacketDesc *DPD = NULL;

  do {

    DPD = new DmaPacketDesc (VA, PA, Pkt);

    if (!DPD) {

      Debug (9, NULL, "Cannot create DMA packet desc for DmaQueue %X", this);

      break;

    }

    Append (DPD);

  } while (False);

  Check ();

  return DPD;

}
 
 
 


//
// Destroy the queue
// =================
//
// Deletes all queue elements
//

void DmaQueue::DestroyAll (void) {

  Check ();

  while (!IsEmpty ()) {

    DmaPacketDesc * const DPD = RemoveFirst ();

    DPD->Check ();

    delete DPD;

  }

  Check ();

}





//
// Append element to the queue
// ===========================
//

void DmaQueue::Append (DmaPacketDesc * const Elem) {

  Assert (Adapter->IsLocked ());

  Check ();

  Elem->Check ();

  if (Tail) {

    Tail->Next = Elem;

  }

  Tail = Elem;

  if (!Head) {

    Head = Elem;

  }

  Count++;

  Debug (2, Adapter, "Elem %X added to DQ %X, total %u", Elem, this, Count);

  Check ();

}





//
// Remove first element from the queue
// ===================================
//

DmaPacketDesc *DmaQueue::RemoveFirst (void) {

  Assert (Adapter->IsLocked ());

  Check ();

  Assert (Head && Tail && Count);

  DmaPacketDesc * const Elem = GetFirst ();

  Elem->Check ();

  Head = Elem->Next;

  if (Tail == Elem) {

    Tail = NULL;

  }

  Elem->Next = NULL;

  Count--;

  Debug (2, Adapter, "Elem %X removed from DQ %X, rest %u", Elem, this, Count);

  Check ();

  return Elem;

}





// #####################################################################
//
// class PacketList functions
// ==========================
//
// #####################################################################





//
// Construction
// ============
//

PacketList::PacketList (

#ifdef _DEBUG

  AdapterDesc const * const A

#else

  void

#endif

)

#ifdef _DEBUG

  : Adapter (A)

#endif

{


  Head = Tail = NULL;
  Count = 0;
  
  Check ();

}





//
// Destruction
// ===========
//

PacketList::~PacketList (void) {

  Check ();

  Assert (!Head && !Tail && !Count);
  
}





//
// Append packet to the list
// =========================
//

void PacketList::Append (NDIS_PACKET * const Pkt) {

  Assert (Adapter->IsLocked ());

  Check ();

  PacketAddon::FromPkt (Pkt)->Next = NULL;

  if (Tail) {

    PacketAddon::FromPkt (Tail)->Next = Pkt;

  }

  Tail = Pkt;

  if (!Head) {

    Head = Pkt;

  }

  Count++;

  Debug (2, Adapter, "Packet %X added to PL %X, total %u", Pkt, this, Count);

}





//
// Remove first packet from the list
// =================================
//

NDIS_PACKET *PacketList::RemoveFirst (void) {

  Assert (Adapter->IsLocked ());

  Check ();

  Assert (Head && Tail && Count);

  NDIS_PACKET * const Pkt = GetFirst ();

  PacketAddon * const PA = PacketAddon::FromPkt (Pkt);

  Head = PA->Next;

  if (Tail == Pkt) {

    Tail = NULL;

  }

  PA->Next = NULL;

  Count--;

  Debug (2, Adapter, "Packet %X removed from PL %X, rest %u", Pkt, this, Count);

  return Pkt;

}
