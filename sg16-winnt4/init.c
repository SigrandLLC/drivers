
#include "precomp.h"


static NDIS_STATUS	ReadConfiguration( PSBNI16_ADAPTER, NDIS_HANDLE );
static NDIS_STATUS	RegisterAdapter( PSBNI16_ADAPTER );
static VOID		ReadMACAddr( IN OUT PSBNI16_ADAPTER, IN PWCHAR );

#pragma NDIS_PAGEABLE_FUNCTION( SB16Initialize )
#pragma NDIS_PAGEABLE_FUNCTION( ReadConfiguration )
#pragma NDIS_PAGEABLE_FUNCTION( RegisterAdapter )
#pragma NDIS_PAGEABLE_FUNCTION( ReadMACAddr )


const NDIS_PHYSICAL_ADDRESS	high_addr = NDIS_PHYSICAL_ADDRESS_CONST(-1,-1);

NDIS_STATUS
SB16Initialize(PNDIS_STATUS OpenErrorStatus,
               PUINT SelectedMediumIndex,
               PNDIS_MEDIUM MediumArray,
               UINT MediumArraySize,
               NDIS_HANDLE mport,
               NDIS_HANDLE WrapperConfigurationContext)
{
	ULONG		i;
	NDIS_STATUS	Status;
	PSBNI16_ADAPTER	Adapter;
	NDIS_HANDLE	ConfigHandle;

	for( i = 0;
	     i < MediumArraySize  &&  MediumArray[ i ] != NdisMedium802_3;
	     ++i );

	if( i == MediumArraySize )
		return NDIS_STATUS_UNSUPPORTED_MEDIA;

	*SelectedMediumIndex = i;

	Status = NdisAllocateMemory( &Adapter, sizeof(SBNI16_ADAPTER),
					0, high_addr );

	if( Status != NDIS_STATUS_SUCCESS )
		return  Status;

	NdisZeroMemory( Adapter, sizeof(SBNI16_ADAPTER) );
	Adapter->mport = mport;

	NdisOpenConfiguration( &Status, &ConfigHandle, WrapperConfigurationContext );
	if( Status != NDIS_STATUS_SUCCESS ) {
		NdisFreeMemory( Adapter, sizeof(SBNI16_ADAPTER), 0 );
		return  NDIS_STATUS_FAILURE;
	}

	Status = ReadConfiguration( Adapter, ConfigHandle );
	NdisCloseConfiguration( ConfigHandle );

	if( Status != NDIS_STATUS_SUCCESS ) {
		NdisFreeMemory( Adapter, sizeof(SBNI16_ADAPTER), 0 );
		return  Status;
	}

	NdisAllocatePacketPool( &Status, &Adapter->PacketPool, MAP_REGS, 16 );
	if( Status != NDIS_STATUS_SUCCESS ) {
		NdisFreeMemory( Adapter, sizeof(SBNI16_ADAPTER), 0 );
		return  Status;
	}

	NdisAllocateBufferPool( &Status, &Adapter->BufPool, MAP_REGS );
	if( Status != NDIS_STATUS_SUCCESS ) {
		NdisFreePacketPool( Adapter->PacketPool );
		NdisFreeMemory( Adapter, sizeof(SBNI16_ADAPTER), 0 );
		return  Status;
	}

	if( (Status = NdisAllocateMemory( &Adapter->sw_rbds,
				RQLEN*sizeof(struct sw_rbd), 0, high_addr ))
	    != NDIS_STATUS_SUCCESS ) {
		NdisFreeBufferPool( Adapter->BufPool );
		NdisFreePacketPool( Adapter->PacketPool );
		NdisFreeMemory( Adapter, sizeof(SBNI16_ADAPTER), 0 );
		return  Status;
	}

	NdisMSetAttributesEx( Adapter->mport, (NDIS_HANDLE) Adapter, 0,
		(ULONG) NDIS_ATTRIBUTE_BUS_MASTER, NdisInterfacePci );

	if( (Status = RegisterAdapter( Adapter ) )
	    != NDIS_STATUS_SUCCESS ) {
		NdisFreeMemory( Adapter->sw_rbds, RQLEN*sizeof(struct sw_rbd), 0 );
		NdisFreeBufferPool( Adapter->BufPool );
		NdisFreePacketPool( Adapter->PacketPool );
		NdisFreeMemory( Adapter, sizeof(SBNI16_ADAPTER), 0 );
		return  Status;
	}

	/* initialize reception buffers queue */
	NdisZeroMemory( Adapter->sw_rbds, RQLEN*sizeof(struct sw_rbd) );
	for( i = 0;  i < RQLEN;  ++i ) {
		struct sw_rbd  *p = (struct sw_rbd *) ((PUCHAR)Adapter->sw_rbds
						+ i*sizeof(struct sw_rbd));

		p->virt_addr = (PUCHAR)Adapter->virt_shared + i*SBNI16_MAX_FRAME;
		p->phys_addr = NdisGetPhysicalAddressLow( Adapter->phys_shared )
				+ i*SBNI16_MAX_FRAME;

		NdisAllocatePacket( &Status, &p->pkt_ptr, Adapter->PacketPool );
		NDIS_SET_PACKET_HEADER_SIZE( p->pkt_ptr, ETHERNET_HEADER_SIZE );
		NDIS_SET_PACKET_STATUS( p->pkt_ptr, NDIS_STATUS_SUCCESS );
		NdisAllocateBuffer( &Status, &p->buf_ptr, Adapter->BufPool,
				    p->virt_addr, SBNI16_MAX_FRAME );
		NdisChainBufferAtFront( p->pkt_ptr, p->buf_ptr );
		SW_RBD_PTR( p->pkt_ptr ) = p;

		reclaim( Adapter, p );
	}

	Adapter->cont_pg_virt = (PUCHAR)Adapter->virt_shared
				+ RQLEN*SBNI16_MAX_FRAME;
	Adapter->cont_pg_phys = NdisGetPhysicalAddressLow(Adapter->phys_shared)
				+ RQLEN*SBNI16_MAX_FRAME;

	NdisMIndicateStatus( Adapter->mport,
			     NDIS_STATUS_MEDIA_DISCONNECT, NULL, 0 );
	NdisMIndicateStatusComplete( Adapter->mport );
	return  NDIS_STATUS_SUCCESS;
}



static NDIS_STATUS
ReadConfiguration( PSBNI16_ADAPTER Adapter, NDIS_HANDLE ConfigHandle )
{
	NDIS_STATUS	Status;
	UINT		i;

	NDIS_STRING	SlotNumStr    = NDIS_STRING_CONST( "SlotNumber" ),
			NetAddressStr = NDIS_STRING_CONST( "NetworkAddress" ),
			ModeStr	      = NDIS_STRING_CONST( "IsItMaster" ),
			LineRateStr   = NDIS_STRING_CONST( "LineRate" ),
			DSLModStr     = NDIS_STRING_CONST( "LineCode" ),
			Crc16Str      = NDIS_STRING_CONST( "CRC16" ),
			FillByteStr   = NDIS_STRING_CONST( "FillByte" ),
			ReadBurstStr  = NDIS_STRING_CONST( "ReadBurst" ),
			WriteBurstStr = NDIS_STRING_CONST( "WriteBurst" );

	PNDIS_CONFIGURATION_PARAMETER  Parm;
	PNDIS_RESOURCE_LIST  AssignedResources;
	UINT    id = -1;
	USHORT  SlotNumber = -1;


/*
 * some pretty ugly macros for code shortening
 */

#define	READ_PARM( ParmNameStr ) NdisReadConfiguration( &Status, &Parm,	\
		ConfigHandle, (ParmNameStr), NdisParameterHexInteger )

#define	READ_IPARM( ParmNameStr ) NdisReadConfiguration( &Status, &Parm,\
		ConfigHandle, (ParmNameStr), NdisParameterInteger )

#define	INT_DATA( Parm )	( (Parm)->ParameterData.IntegerData )


	READ_PARM( &SlotNumStr );
	if( Status == NDIS_STATUS_SUCCESS )
		SlotNumber = (USHORT) INT_DATA( Parm );

	NdisReadPciSlotInformation( Adapter->mport,
				    SlotNumber, PCI_VENDOR_ID_REGISTER,
				    &id, sizeof id );

	if( (SBNI16_VENDOR | (SBNI16_DEVICE << 16)) != id )
		return  NDIS_STATUS_FAILURE;

	NdisReadPciSlotInformation( Adapter->mport,
		SlotNumber, PCI_MEM_BASE_REGISTER,
		&Adapter->MemBase, 4 );
	Adapter->MemBase &= ~15;
	NdisReadPciSlotInformation( Adapter->mport,
		SlotNumber, PCI_INTERRUPT_REGISTER,
		&Adapter->InterruptNumber, 1 );

	if( Adapter->InterruptNumber == 0xFF
	    || Adapter->MemBase == 0 )
		return  NDIS_STATUS_FAILURE;

	/* read line rate */
	Adapter->cfg.rate = 256;
	READ_IPARM( &LineRateStr );
	if( Status == NDIS_STATUS_SUCCESS )
		Adapter->cfg.rate = INT_DATA( Parm );

	/* read Master/Slave mode */
	READ_IPARM( &ModeStr );
	if( Status == NDIS_STATUS_SUCCESS )
		Adapter->cfg.master = INT_DATA( Parm );

	Adapter->cfg.mod = 1;		/* default line code TCPAM-16 */
	READ_IPARM( &DSLModStr );
	if( Status == NDIS_STATUS_SUCCESS )
		Adapter->cfg.mod = INT_DATA( Parm );

	Adapter->cfg.crc16 = 0;
	READ_IPARM( &Crc16Str );
	if( Status == NDIS_STATUS_SUCCESS )
		Adapter->cfg.crc16 = INT_DATA( Parm );

	Adapter->cfg.fill_7e = 0;
	READ_IPARM( &FillByteStr );
	if( Status == NDIS_STATUS_SUCCESS )
		Adapter->cfg.fill_7e = INT_DATA( Parm );

	Adapter->cfg.rburst = 1;
	READ_IPARM( &ReadBurstStr );
	if( Status == NDIS_STATUS_SUCCESS )
		Adapter->cfg.rburst = INT_DATA( Parm );

	Adapter->cfg.wburst = 1;
	READ_IPARM( &WriteBurstStr );
	if( Status == NDIS_STATUS_SUCCESS )
		Adapter->cfg.wburst = INT_DATA( Parm );

	/* read MAC-address */
	Adapter->MACAddress[ 0 ] = 0x00;
	Adapter->MACAddress[ 1 ] = 0xFF;
	Adapter->MACAddress[ 2 ] = 0x01;
	Adapter->MACAddress[ 3 ] = 0x2D;
	Adapter->MACAddress[ 4 ] = 0x00;
	Adapter->MACAddress[ 5 ] = 0x00;

	NdisReadConfiguration( &Status, &Parm, ConfigHandle, &NetAddressStr,
			       NdisParameterString );
	if( Status == NDIS_STATUS_SUCCESS )
		ReadMACAddr( Adapter, Parm->ParameterData.StringData.Buffer );

	return  NDIS_STATUS_SUCCESS;
}


static NDIS_STATUS  RegisterAdapter( PSBNI16_ADAPTER  Adapter )
{
        LARGE_INTEGER  PhysAddr = { 0, 0 };
	NDIS_STATUS  Status;

	PhysAddr.u.LowPart = Adapter->MemBase;
	if( (Status = NdisMMapIoSpace( (PVOID *)&Adapter->MemPAddr,
					Adapter->mport,
					PhysAddr,
					SBNI16_MEMSIZE )
            ) != NDIS_STATUS_SUCCESS )
		return  Status;

	if( (Status = NdisMAllocateMapRegisters( Adapter->mport, 0, TRUE,
						 MAP_REGS, SBNI16_MAX_FRAME )
            ) != NDIS_STATUS_SUCCESS )
		goto  fail4;

	Adapter->free_map_reg = Adapter->old_map_reg = 0;

	NdisMAllocateSharedMemory( Adapter->mport, SHARED_MEM_SIZE, FALSE,
			&Adapter->virt_shared, &Adapter->phys_shared );

	if( !Adapter->virt_shared ) {
		Status = NDIS_STATUS_RESOURCES;
		goto  fail3;
	}

	Status = NdisMRegisterInterrupt( &Adapter->Interrupt,
			Adapter->mport,
			Adapter->InterruptNumber,
			Adapter->InterruptNumber,
			TRUE,
			TRUE,
			NdisInterruptLevelSensitive );

	/* interrupts are disabled until CardStart completed */

	if( Status != NDIS_STATUS_SUCCESS ) {
		NdisWriteErrorLogEntry( Adapter->mport,
			NDIS_ERROR_CODE_INTERRUPT_CONNECT, 0 );
		goto  fail2;
	}

	NdisAllocateSpinLock( &Adapter->Lock );
	NdisInitializeEvent( &Adapter->cx_intr );

	NdisMRegisterAdapterShutdownHandler( Adapter->mport,
					     Adapter, SB16Shutdown );

	if( (Status = CardStart( Adapter )) != NDIS_STATUS_SUCCESS ) {
		NdisWriteErrorLogEntry( Adapter->mport,
			NDIS_ERROR_CODE_ADAPTER_NOT_FOUND, 0 );
		goto  fail;
	}

	return  NDIS_STATUS_SUCCESS;

fail:
	NdisMDeregisterAdapterShutdownHandler( Adapter->mport );
	NdisFreeSpinLock( &Adapter->Lock );
	NdisMDeregisterInterrupt( &Adapter->Interrupt );

fail2:
	NdisMFreeSharedMemory( Adapter->mport, SHARED_MEM_SIZE, FALSE,
				Adapter->virt_shared, Adapter->phys_shared );
fail3:
	NdisMFreeMapRegisters( Adapter->mport );
fail4:
	NdisMUnmapIoSpace( Adapter->mport,
			   (PVOID)Adapter->MemPAddr, SBNI16_MEMSIZE );
	return  Status;
}


static VOID
ReadMACAddr( IN OUT PSBNI16_ADAPTER Adapter, IN PWCHAR SBNetAddress )
{
	UCHAR  StrBuf[ 12 ];
        UINT   i;

	for( i = 0;  i < 12;  i++ )
		if( SBNetAddress[i] >= '0' && SBNetAddress[i] <= '9' )
			StrBuf[ i ] = SBNetAddress[i] - '0';
		else if( SBNetAddress[i] >= 'a' && SBNetAddress[i] <= 'f' )
			StrBuf[ i ] = SBNetAddress[i] - 'a' + 10;
		else if( SBNetAddress[i] >= 'A' && SBNetAddress[i] <= 'F' )
			StrBuf[ i ] = SBNetAddress[i] - 'A' + 10;
		else
			StrBuf[ i ] = 0;

	for( i = 0;  i < 6;  i++ )
		Adapter->MACAddress[ i ] =
			(StrBuf[ i<<1 ] << 4) | StrBuf[ (i<<1)+1 ];
}


NDIS_STATUS
CardStart( IN PSBNI16_ADAPTER Adapter )
{
	NDIS_STATUS	status;
	NDIS_HANDLE	handle;
	PVOID		firmw_img;
	UINT		firmw_len;
	NDIS_STRING	file_name = NDIS_STRING_CONST("cxfw.bin");

	NDIS_PHYSICAL_ADDRESS  highest = NDIS_PHYSICAL_ADDRESS_CONST( -1, -1 );

	Adapter->tbd  = (struct hw_descr *) Adapter->MemPAddr;
	Adapter->rbd  = (struct hw_descr *) (Adapter->MemPAddr + 0x400);
	Adapter->regs = (struct sbni16_hw_regs *) (Adapter->MemPAddr + 0x800);
	Adapter->cmdp = (struct cx28975_cmdarea *) (Adapter->MemPAddr + 0xc00);

	Adapter->regs->CR = 0;
	Adapter->regs->SR = 0xff;
	Adapter->regs->IMR = 0;

	/* read the 8051 program code */
	NdisOpenFile( &status, &handle, &firmw_len, &file_name, highest );
	if( status != NDIS_STATUS_SUCCESS )
		return  status;

	NdisMapFile( &status, &firmw_img, handle );
	if( status != NDIS_STATUS_SUCCESS )
		goto  done;

	Adapter->head_xq = Adapter->tail_xq = Adapter->head_rq = Adapter->tail_rq = 0;
	Adapter->head_tdesc = Adapter->head_rdesc = 0;

	status = start_cx28975( Adapter, (PUCHAR)firmw_img, firmw_len ) != -1
		 ?  NDIS_STATUS_SUCCESS  :  NDIS_STATUS_ADAPTER_NOT_FOUND;
	NdisUnmapFile( handle );
done:
	NdisCloseFile( handle );
	return  status;
}
