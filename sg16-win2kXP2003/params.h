#define VersionMajor 1
#define VersionMinor 10



// We don't have official vendor id yet...

#define VendorId 0x55
#define DeviceId 0x9d
#define SubDevId 3



#define NDIS_MAJOR_VERSION 0x5
#define NDIS_MINOR_VERSION 0x0



#define ETHERNET_HEADER_SIZE 14
#define ETHERNET_MIN_SIZE  60
#define ETHERNET_MAX_SIZE 1514

#define MULTICAST_LIST_SIZE 32


//
// Parameter defaults
// ==================
//
// Modulation type has no default value. It must be specified explicitly or
// chosen from the rate value.
//

#define Modem_LineRateDef 256
#define Modem_MasterModeDef false
#define Modem_AlwaysConnectedDef false

#define Hdlc_Crc16Def false
#define Hldc_Fill7eDef false
#define Hldc_InvertedDef false
#define Hldc_ReadBurstDef true
#define Hldc_WriteBurstDef true

#define Driver_XmtQueueLenDef 8
#define Driver_RcvQueueLenDef 16
