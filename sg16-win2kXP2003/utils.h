class AdapterDesc;





class SpinLock {

  NDIS_SPIN_LOCK Lock;

  ULONG volatile Owner;
  LONG volatile Depth;

#ifdef _DEBUG

  AdapterDesc const * const Adapter;

#endif

  public:

  SpinLock (

#ifdef _DEBUG

    AdapterDesc const * const A

#else

    void

#endif
      
  );

  ~SpinLock (void);

  SpinLock const & operator = (SpinLock const &Src) { Break (); return Src; }

  void Acquire (void);
  void Release (void);

  bool IsOwned (void) const;

  void Check (void) const {

#ifdef _DEBUG

    Assert (Depth >= 0 || Depth <= 1);

#endif

  }

};





class DmaPacketDesc {           // Packet/buffer/DMA descriptor

  public:

  PVOID VirtAddr;               // Virtual address of DMA buffer
  DWORD PhysAddr;               // Physical address of DMA buffer

  PNDIS_PACKET Pkt;             // Corresponding packet descriptor pointer
                                // For tx: original packet got from MiniportSendPackets
                                // For rx: our own packet got from NdisAllocatePacket

  IfDebug (UINT BdIndex);       // Corresponding BD index in HLDC ring

  DmaPacketDesc * volatile Next; // Forward link

  DmaPacketDesc (PVOID VA, DWORD PA, PNDIS_PACKET Pkt);
  ~DmaPacketDesc (void);

  void Check (void) const {

#ifdef _DEBUG

    Assert (IsValidPtr (this, sizeof (*this)));

    Assert (IsValidPtr (VirtAddr, DMA_BUFFER_SIZE));

    Assert ((BYTE (VirtAddr) & 3) == 0 && (PhysAddr & 3) == 0);

    AssertIf (Pkt, IsValidPtr (Pkt, sizeof (*Pkt)));

    Assert (BdIndex < NUM_HLDC_RING_ELEMS);

#endif

  }

};





//
// DMA packet queue descriptor
// ===========================
//
// Queue is made in ring style.
// Maintained in correspondence with related TBD/RBD ring.
//

class DmaQueue {

  DmaPacketDesc * volatile Head; // Head pointer
  DmaPacketDesc * volatile Tail; // Tail pointer

  UINT Count;                    // Element count

#ifdef _DEBUG

  AdapterDesc const * const Adapter;

#endif

  public:

  DmaQueue (

#ifdef _DEBUG

    AdapterDesc const * const A

#else

    void

#endif

  );

  ~DmaQueue (void);

  DmaQueue const & operator = (DmaQueue const &Src) { Break (); return Src; }

  DmaPacketDesc *CreateElem (PVOID VA, DWORD PA, PNDIS_PACKET Pkt);
  void DestroyAll (void);

  bool IsEmpty (void) const { return !Head; }

  UINT GetCount (void) const { return Count; }

  DmaPacketDesc *GetFirst (void) const { return Head; }

  void Append (DmaPacketDesc * const Elem);
  DmaPacketDesc *RemoveFirst (void);

  void Check (void) const {

#ifdef _DEBUG

    Assert (IsValidPtr (this, sizeof (*this)));

    Assert (!!Head == !!Tail && !!Head == !!Count);

#endif

  }

};





class PacketAddon {             // MiniportReservedEx data structure

  public:

  NDIS_PACKET *Next;            // Next packet link
  DmaPacketDesc *DPD;           // Corresponding DmaPacketDesc object

  static PacketAddon *FromPkt (PNDIS_PACKET Pkt) {

    return (PacketAddon *) (&Pkt->MiniportReservedEx);

  }

};

//!!!Assume (sizeof (PacketAddon) <= sizeof (_NDIS_PACKET::MiniportReservedEx));





class PacketList {              // Linked packet list

  NDIS_PACKET * volatile Head;
  NDIS_PACKET * volatile Tail;

  UINT Count;

#ifdef _DEBUG

  AdapterDesc const * const Adapter;

#endif

  public:

  PacketList (

#ifdef _DEBUG

    AdapterDesc const * const A

#else

    void

#endif

  );

  ~PacketList (void);

  PacketList const & operator = (PacketList const &Src) { Break (); return Src; }

  bool IsEmpty (void) const { return !Head; }

  NDIS_PACKET *GetFirst (void) const { return Head; }

  void Append (NDIS_PACKET * const Pkt);
  NDIS_PACKET *RemoveFirst (void);

  void Check (void) const {

#ifdef _DEBUG

    Assert (IsValidPtr (this, sizeof (*this)));

    Assert (!!Head == !!Tail && !!Head == !!Count);

#endif

  }

};





void * _cdecl operator new (size_t Size);
void _cdecl operator delete (void *Block);





//
// Inc/Dec versions for volatile args
//

inline LONG NdisInterlockedIncrement (LONG volatile *Var) {

  return NdisInterlockedIncrement (const_cast <LONG *> (Var));

}

inline LONG NdisInterlockedDecrement (LONG volatile *Var) {

  return NdisInterlockedDecrement (const_cast <LONG *> (Var));

}




inline UINT Abs (int a) {

  return (a >= 0)? a : -a;

}





extern NDIS_PHYSICAL_ADDRESS const HighestAddr;

IfDebug (extern UINT MemoryBlocksAllocated);
