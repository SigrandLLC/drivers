
#include "precomp.h"


NTSTATUS
DriverEntry( PDRIVER_OBJECT drvObj, PUNICODE_STRING registryPath )
{
	NDIS_HANDLE	wrapperHandle;

	NDIS_MINIPORT_CHARACTERISTICS	sbni16;

	NdisMInitializeWrapper( &wrapperHandle, drvObj, registryPath, NULL );
	NdisZeroMemory( &sbni16, sizeof sbni16 );

	sbni16.MajorNdisVersion       = SBNI16_NDIS_MAJOR_VERSION;
	sbni16.MinorNdisVersion       = SBNI16_NDIS_MINOR_VERSION;
	sbni16.CheckForHangHandler    = SB16CheckForHang;
	sbni16.DisableInterruptHandler= NULL;
	sbni16.EnableInterruptHandler = NULL;
	sbni16.HaltHandler            = SB16Halt;
	sbni16.HandleInterruptHandler = SB16HandleInterrupt;
	sbni16.InitializeHandler      = SB16Initialize;
	sbni16.ISRHandler             = SB16Isr;
	sbni16.QueryInformationHandler= SB16QueryInformation;
	sbni16.ReconfigureHandler     = NULL;
	sbni16.ResetHandler           = SB16Reset;
	sbni16.SetInformationHandler  = SB16SetInformation;
	sbni16.SendHandler            = NULL;
	sbni16.SendPacketsHandler     = SB16MultipleSend;
	sbni16.ReturnPacketHandler    = SB16GetReturnedPackets;
	sbni16.TransferDataHandler    = NULL;
	sbni16.AllocateCompleteHandler= NULL;

	return  NdisMRegisterMiniport( wrapperHandle, &sbni16,
					sizeof(NDIS_MINIPORT_CHARACTERISTICS) );
}


VOID
SB16Halt( NDIS_HANDLE context )
{
	PSBNI16_ADAPTER  Adapter = (PSBNI16_ADAPTER) context;

	struct sw_rbd	*p;
	UCHAR  t = 0;

	NdisAcquireSpinLock( &Adapter->Lock );
	Adapter->regs->CR  &= ~(RXEN | TXEN);
	Adapter->regs->CRB |= RXDE;
	Adapter->regs->IMR  = EXT;
	Adapter->regs->CTDR = Adapter->regs->LTDR;
	Adapter->regs->CRDR = Adapter->regs->LRDR;
	Adapter->state = ACTIVATION;

	drop_queues( Adapter );
	NdisReleaseSpinLock( &Adapter->Lock );

	NdisMIndicateStatus( Adapter->mport,
			     NDIS_STATUS_MEDIA_DISCONNECT, NULL, 0 );
	NdisMIndicateStatusComplete( Adapter->mport );

	issue_cx28975_cmd( Adapter, _DSL_ACTIVATION, &t, 1 );
	if( Adapter->state == ACTIVE ) {
		t = 1;
		issue_cx28975_cmd( Adapter, _DSL_FORCE_DEACTIVATE, &t, 1 );
		/* FIX! activation manager state */
	}
	
	Adapter->regs->IMR = 0;
	Adapter->regs->CR  = 0;
	Adapter->regs->SR  = 0xff;
	Adapter->state = DOWN;

	for( p = Adapter->swr_head;  p;  p = p->link ) {
		NdisFreeBuffer( p->buf_ptr );
		NdisFreePacket( p->pkt_ptr );
	}

	NdisMDeregisterAdapterShutdownHandler( Adapter->mport );
	NdisFreeSpinLock( &Adapter->Lock );
	NdisMDeregisterInterrupt( &Adapter->Interrupt );

	NdisMFreeSharedMemory( Adapter->mport, SHARED_MEM_SIZE, FALSE,
				Adapter->virt_shared, Adapter->phys_shared );
	NdisMFreeMapRegisters( Adapter->mport );
	NdisMUnmapIoSpace( Adapter->mport,
			   (PVOID)Adapter->MemPAddr, SBNI16_MEMSIZE );

	NdisFreeMemory( Adapter->sw_rbds, RQLEN*sizeof(struct sw_rbd), 0 );
	NdisFreeBufferPool( Adapter->BufPool );
	NdisFreePacketPool( Adapter->PacketPool );
	NdisFreeMemory( Adapter, sizeof(SBNI16_ADAPTER), 0 );
}


NDIS_STATUS
SB16Reset( PBOOLEAN need_set_info, NDIS_HANDLE context )
{
	PSBNI16_ADAPTER  Adapter = (PSBNI16_ADAPTER) context;

	NdisAcquireSpinLock( &Adapter->Lock );

	Adapter->regs->CR   = 0;
	Adapter->regs->CRB  = RXDE;
	Adapter->regs->IMR  = 0;
	Adapter->regs->CTDR = Adapter->regs->LTDR;
	Adapter->regs->CRDR = Adapter->regs->LRDR;

	NdisReleaseSpinLock( &Adapter->Lock );
	NdisMIndicateStatus( Adapter->mport,
			     NDIS_STATUS_MEDIA_DISCONNECT, NULL, 0 );
	NdisMIndicateStatusComplete( Adapter->mport );
	NdisAcquireSpinLock( &Adapter->Lock );

	Adapter->state = DOWN;

	drop_queues( Adapter );
	CardStart( Adapter );

	NdisReleaseSpinLock( &Adapter->Lock );
	*need_set_info = FALSE;
	return  NDIS_STATUS_SUCCESS;
}


VOID
SB16Shutdown( IN NDIS_HANDLE  Context )
{
	PSBNI16_ADAPTER  Adapter = (PSBNI16_ADAPTER) Context;

	Adapter->regs->IMR = 0;
	Adapter->regs->CRB = RXDE;
	Adapter->regs->CR  = 0;
	Adapter->regs->SR  = 0xff;
}


BOOLEAN
SB16CheckForHang( NDIS_HANDLE  Context )
{
	PSBNI16_ADAPTER  Adapter = (PSBNI16_ADAPTER) Context;

	NdisAcquireSpinLock( &Adapter->Lock );
	if( Adapter->state != ACTIVE )
		drop_queues( Adapter );
	NdisReleaseSpinLock( &Adapter->Lock );
	return  FALSE;
}

