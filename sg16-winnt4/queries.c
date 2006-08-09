
#include "precomp.h"


static UINT  SbniSupportedOids[] = {
        OID_GEN_SUPPORTED_LIST,
        OID_GEN_HARDWARE_STATUS,
        OID_GEN_MEDIA_SUPPORTED,
        OID_GEN_MEDIA_IN_USE,
        OID_GEN_MAXIMUM_LOOKAHEAD,
        OID_GEN_MAXIMUM_FRAME_SIZE,
        OID_GEN_MAXIMUM_TOTAL_SIZE,
        OID_GEN_MAC_OPTIONS,
        OID_GEN_MEDIA_CONNECT_STATUS,
        OID_GEN_LINK_SPEED,
        OID_GEN_TRANSMIT_BUFFER_SPACE,
        OID_GEN_RECEIVE_BUFFER_SPACE,
        OID_GEN_TRANSMIT_BLOCK_SIZE,
        OID_GEN_RECEIVE_BLOCK_SIZE,
        OID_GEN_VENDOR_DESCRIPTION,
        OID_GEN_VENDOR_ID,
        OID_GEN_DRIVER_VERSION,
        OID_GEN_CURRENT_PACKET_FILTER,
        OID_GEN_CURRENT_LOOKAHEAD,
        OID_GEN_MAXIMUM_SEND_PACKETS,
        OID_GEN_TRANSMIT_QUEUE_LENGTH,
        OID_GEN_XMIT_OK,
        OID_GEN_RCV_OK,
        OID_GEN_XMIT_ERROR,
        OID_GEN_RCV_ERROR,
        OID_GEN_RCV_NO_BUFFER,

        OID_802_3_PERMANENT_ADDRESS,
        OID_802_3_CURRENT_ADDRESS,
        OID_802_3_MULTICAST_LIST,
        OID_802_3_MAXIMUM_LIST_SIZE,
        OID_802_3_RCV_ERROR_ALIGNMENT,
        OID_802_3_XMIT_ONE_COLLISION,
        OID_802_3_XMIT_MORE_COLLISIONS,

        0xFF020210,
        0xFF020211,
        0xFF020212
};



NDIS_STATUS  SB16QueryInformation( IN NDIS_HANDLE  Context,
                                   IN NDIS_OID  Oid,
                                   IN PVOID  InformationBuffer,
                                   IN ULONG  InformationBufferLength,
                                   OUT PULONG  BytesWritten,
                                   OUT PULONG  BytesNeeded )
{
        PSBNI16_ADAPTER  Adapter = (PSBNI16_ADAPTER) Context;

        PUCHAR  InfoBuffer = (PUCHAR)InformationBuffer;
        UINT    BytesLeft = InformationBufferLength;

        NDIS_STATUS  StatusToReturn = NDIS_STATUS_SUCCESS;
        NDIS_HARDWARE_STATUS  HardwareStatus = NdisHardwareStatusReady;
        NDIS_MEDIUM  Medium = NdisMedium802_3;

        ULONG   GenericULong;
        USHORT  GenericUShort;
        UCHAR   GenericArray[ 6 ],t;
        UINT    MoveBytes = sizeof( ULONG );
        PVOID   MoveSource = (PVOID)&GenericULong;

        switch( Oid ) {
        case OID_GEN_MAC_OPTIONS :
                GenericULong = NDIS_MAC_OPTION_TRANSFERS_NOT_PEND  |
                        NDIS_MAC_OPTION_COPY_LOOKAHEAD_DATA |
/*                      NDIS_MAC_OPTION_FULL_DUPLEX |*/
                        NDIS_MAC_OPTION_NO_LOOPBACK;
                break;

        case OID_GEN_SUPPORTED_LIST :
                MoveSource = SbniSupportedOids;
                MoveBytes  = sizeof SbniSupportedOids;
                break;

        case OID_GEN_HARDWARE_STATUS :
                HardwareStatus = NdisHardwareStatusReady;
                MoveSource = &HardwareStatus;
                MoveBytes  = sizeof( NDIS_HARDWARE_STATUS );
                break;

        case OID_GEN_MEDIA_SUPPORTED :
        case OID_GEN_MEDIA_IN_USE :
                MoveSource = &Medium;
                MoveBytes  = sizeof( NDIS_MEDIUM );
                break;

        case OID_GEN_MEDIA_CONNECT_STATUS :
                GenericULong = Adapter->state == ACTIVE  ?
                        NdisMediaStateConnected : NdisMediaStateDisconnected;
                break;

        case OID_GEN_MAXIMUM_LOOKAHEAD :
        case OID_GEN_CURRENT_LOOKAHEAD :
        case OID_GEN_MAXIMUM_FRAME_SIZE :
                GenericULong = 1500;
                break;

        case OID_GEN_MAXIMUM_TOTAL_SIZE :
                GenericULong = 1514;
                break;

        case OID_GEN_LINK_SPEED :
                GenericULong = Adapter->cfg.rate * 10;
                break;

        case OID_GEN_TRANSMIT_BUFFER_SPACE :
        case OID_GEN_RECEIVE_BUFFER_SPACE :
        case OID_GEN_RECEIVE_BLOCK_SIZE :
                GenericULong = 1514;
                break;

        case OID_GEN_TRANSMIT_BLOCK_SIZE :
                GenericULong = 12;
                break;

        case OID_GEN_TRANSMIT_QUEUE_LENGTH :
                GenericULong = 128;
                break;

        case OID_GEN_MAXIMUM_SEND_PACKETS :
                GenericULong = 32;
                break;

        case OID_GEN_VENDOR_ID :
                NdisMoveMemory( &GenericULong, Adapter->MACAddress, 3 );
                GenericULong &= 0xFFFFFF00;
                break;

        case OID_GEN_VENDOR_DESCRIPTION :
                MoveSource = "Granch SBNI16 SHDSL Modem";
                MoveBytes  = 24;
                break;

        case OID_GEN_DRIVER_VERSION :
                GenericUShort = (USHORT)(SBNI16_NDIS_MAJOR_VERSION << 8)
                                | SBNI16_NDIS_MINOR_VERSION;
                MoveSource = &GenericUShort;
                MoveBytes  = sizeof( USHORT );
                break;

        case OID_802_3_PERMANENT_ADDRESS :
        case OID_802_3_CURRENT_ADDRESS :
                NdisMoveMemory( GenericArray, Adapter->MACAddress,
                                ETH_LENGTH_OF_ADDRESS );
                MoveSource = GenericArray;
                MoveBytes  = sizeof Adapter->MACAddress;
                break;

        case OID_802_3_MAXIMUM_LIST_SIZE:
                GenericULong = MULTICAST_LIST_SIZE;
                break;

        case OID_GEN_XMIT_OK :
                GenericULong = Adapter->XmitGood;
                break;

        case OID_GEN_RCV_OK :
                GenericULong = Adapter->RcvGood;
                break;

        case OID_GEN_XMIT_ERROR :
                GenericULong = Adapter->XmitBad;
                break;

        case OID_GEN_RCV_ERROR :
                GenericULong = Adapter->CrcErrors;
                break;

        /* we must support these OIDs though ones aren't actual for SBNI16 */
        case OID_GEN_RCV_NO_BUFFER :
        case OID_802_3_RCV_ERROR_ALIGNMENT :
        case OID_802_3_XMIT_ONE_COLLISION :
        case OID_802_3_XMIT_MORE_COLLISIONS :
                GenericULong = 0;
                break;

        case 0xFF020210 :       /* internal statistics read - start */
                start_rdstat( Adapter );
                MoveBytes = 0;
                break;

        case 0xFF020211 :       /* internal statistics read - finalize */
                if( Adapter->rdstat )
                        StatusToReturn = NDIS_STATUS_FAILURE;
                MoveSource = &Adapter->in_stats;
                MoveBytes  = sizeof( struct sbni16_stats );
                break;

        case 0xFF020212 :       /* clear statistics */
         t=0x00;
         issue_cx28975_cmd(Adapter,_DSL_CLEAR_ERROR_CTRS,&t,1);
         t=0x04;
         issue_cx28975_cmd(Adapter,_DSL_CLEAR_ERROR_CTRS,&t,1);
         NdisZeroMemory( &Adapter->in_stats, sizeof(struct sbni16_stats) );

                MoveBytes = 0;
                break;

        default:
                StatusToReturn = NDIS_STATUS_INVALID_OID;
        }

        if( StatusToReturn == NDIS_STATUS_SUCCESS ) {
                if( MoveBytes > BytesLeft )
                        *BytesNeeded = MoveBytes,
                        StatusToReturn = NDIS_STATUS_INVALID_LENGTH;

                else
                        NdisMoveMemory( InfoBuffer, MoveSource, MoveBytes ),
                        *BytesWritten += MoveBytes;
        }

/*      if( StatusToReturn != NDIS_STATUS_SUCCESS )
                NdisWriteErrorLogEntry( Adapter->MiniportAdapterHandle,
                        NDIS_ERROR_CODE_DRIVER_FAILURE,
                        3, 0, Oid, StatusToReturn );
*/      return  StatusToReturn;
}

/* ------------------------------------------------------------------ */

NDIS_STATUS  SB16SetInformation( IN NDIS_HANDLE  Context,
                                 IN NDIS_OID  Oid,
                                 IN PVOID  InformationBuffer,
                                 IN ULONG  InformationBufferLength,
                                 OUT PULONG  BytesRead,
                                 OUT PULONG  BytesNeeded )
{
        PSBNI16_ADAPTER  Adapter = (PSBNI16_ADAPTER) Context;
        NDIS_STATUS  StatusToReturn = NDIS_STATUS_SUCCESS;

        UINT   BytesLeft  = InformationBufferLength;
        PUCHAR InfoBuffer = InformationBuffer;

        UINT   OidLength  = BytesLeft;
        ULONG  LookAhead;
        ULONG  Filter;

        switch( Oid ) {
        case OID_GEN_CURRENT_LOOKAHEAD :
                if( OidLength != 4 ) {
                        StatusToReturn = NDIS_STATUS_INVALID_LENGTH;
                        *BytesRead     = 0;
                        *BytesNeeded   = 0;
                        break;
                }
                *BytesRead = 4;
                break;

        case OID_802_3_MULTICAST_LIST :
                *BytesRead   = 0;
                *BytesNeeded = 0;
                if( OidLength % ETH_LENGTH_OF_ADDRESS )
                        StatusToReturn = NDIS_STATUS_INVALID_LENGTH;
                else if( OidLength > MULTICAST_LIST_SIZE * ETH_LENGTH_OF_ADDRESS )
                        StatusToReturn = NDIS_STATUS_MULTICAST_FULL;
                else
                        NdisMoveMemory( Adapter->MulticastList, InfoBuffer, OidLength ),
                        Adapter->MulticastCount = OidLength / ETH_LENGTH_OF_ADDRESS,
                        *BytesRead = OidLength;
                break;

        case OID_GEN_CURRENT_PACKET_FILTER :
                if( OidLength != 4 ) {
                        StatusToReturn = NDIS_STATUS_INVALID_LENGTH;
                        *BytesRead     = 0;
                        *BytesNeeded   = 0;
                        break;
                }

                NdisMoveMemory( &Filter, InfoBuffer, 4 );

                if( Filter & ~(NDIS_PACKET_TYPE_DIRECTED    |
                               NDIS_PACKET_TYPE_BROADCAST   |
                               NDIS_PACKET_TYPE_PROMISCUOUS |
                               NDIS_PACKET_TYPE_MULTICAST   |
                               NDIS_PACKET_TYPE_ALL_MULTICAST)
                  )
                        StatusToReturn = NDIS_STATUS_NOT_SUPPORTED,
                        *BytesRead     = 4,
                        *BytesNeeded   = 0;
                else
                        /* все поддерживаемые флаги - в младшем байте */
                        Adapter->FilterMode = (UCHAR)Filter;
                break;

        default:
                StatusToReturn = NDIS_STATUS_INVALID_OID;
                *BytesRead = 0;
                *BytesNeeded = 0;
        }

/*      if( StatusToReturn != NDIS_STATUS_SUCCESS )
                NdisWriteErrorLogEntry( Adapter->MiniportAdapterHandle,
                        NDIS_ERROR_CODE_DRIVER_FAILURE,
                        3, 1, Oid, StatusToReturn );
*/      return  StatusToReturn;
}
