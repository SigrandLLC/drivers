#include "common.h"

/* Constants */
static char const	DeviceNameString[] = DeviceName;

/* -----------------------------------------------------------------------------
 *    FunctionsRead configuration parameters
 ------------------------------------------------------------------------------- */
NDIS_STATUS
AdapterDesc::ReadConfiguration(NDIS_HANDLE ConfigHandle)
{
	Debug(0, this, "ReadConfiguration:ENTER");
	Check();

	NDIS_STATUS						Status;
	PNDIS_CONFIGURATION_PARAMETER	Parm;

	/* PNDIS_RESOURCE_LIST AssignedResources; */
	UINT							i, id;
	WORD							SlotNumber = 0;
#define READ_XPARM(ParmNameStr) \
	NdisReadConfiguration(&Status, &Parm, ConfigHandle, \
						  const_cast < NDIS_STRING * > (ParmNameStr), \
						  NdisParameterHexInteger)
#define READ_IPARM(ParmNameStr) \
	NdisReadConfiguration(&Status, &Parm, ConfigHandle, \
						  const_cast < NDIS_STRING * > (ParmNameStr), \
						  NdisParameterInteger)
#define INT_DATA(Parm)	(Parm->ParameterData.IntegerData)
	NDIS_STRING const	SlotNumStr = NDIS_STRING_CONST("SlotNumber");
	READ_XPARM(&SlotNumStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		SlotNumber = (WORD) INT_DATA(Parm);
		Debug(7, this, "Using PCI slot %u", SlotNumber);
	}

	NdisReadPciSlotInformation(DriverHandle, SlotNumber, PCI_VENDOR_ID_REGISTER,
							   &id, sizeof id);
	if((VendorId | (DeviceId << 16)) != id)
	{
		Debug(9, this, "Invalid device id %X", id);
		return NDIS_STATUS_FAILURE;
	}

	NdisReadPciSlotInformation(DriverHandle, SlotNumber, PCI_MEM_BASE_REGISTER,
							   &MemoryWindowPhysAddr, 4);
	MemoryWindowPhysAddr &= ~15;
	NdisReadPciSlotInformation(DriverHandle, SlotNumber, PCI_INTERRUPT_REGISTER,
							   &InterruptNumber, 1);
	if(InterruptNumber == 0xFF || MemoryWindowPhysAddr == 0)
	{
		return NDIS_STATUS_FAILURE;
	}

	NDIS_STRING const	LineRateStr = NDIS_STRING_CONST("LineRate");
	READ_IPARM(&LineRateStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		ModemCfg.rate = INT_DATA(Parm) &~0x1F;
	}

	NDIS_STRING const	TerminalTypeStr = NDIS_STRING_CONST("TerminalType");
	READ_IPARM(&TerminalTypeStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		ModemCfg.master = !!INT_DATA(Parm);
	}

	/* Adding by me */
	NDIS_STRING const	CfgTypeStr = NDIS_STRING_CONST("ConfigType");
	READ_IPARM(&CfgTypeStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		ModemCfg.remcfg = !!INT_DATA(Parm);
	}

	NDIS_STRING const	AnnexTypeStr = NDIS_STRING_CONST("AnnexType");
	READ_IPARM(&AnnexTypeStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		ModemCfg.annex = Annexes(INT_DATA(Parm));
	}

	/* End */
	NDIS_STRING const	AlwaysConnectedStr = NDIS_STRING_CONST("AlwaysConnected");
	READ_IPARM(&AlwaysConnectedStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		ModemCfg.AlwaysConnected = !!INT_DATA(Parm);
	}

	NDIS_STRING const	DSLModStr = NDIS_STRING_CONST("LineCode");
	READ_IPARM(&DSLModStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		if(INT_DATA(Parm) && INT_DATA(Parm) - 1 < Mod_Auto)
		{
			ModemCfg.mod = Modulations(INT_DATA(Parm) - 1);
		} else
		{
			if(ModemCfg.rate < 128)
			{
				ModemCfg.mod = Mod_TCPAM4;
			} else if(ModemCfg.rate < 1152)
			{
				ModemCfg.mod = Mod_TCPAM8;
			} else if(ModemCfg.rate < 2560)
			{
				ModemCfg.mod = Mod_TCPAM16;
			} else
			{
				ModemCfg.mod = Mod_TCPAM32;
			}
		}
	}

	NDIS_STRING const	CrcStr = NDIS_STRING_CONST("CRC");
	READ_IPARM(&CrcStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		HdlcCfg.crc16 = !!INT_DATA(Parm);
	}

	NDIS_STRING const	FillByteStr = NDIS_STRING_CONST("FillByte");
	READ_IPARM(&FillByteStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		HdlcCfg.fill_7e = !!INT_DATA(Parm);
	}

	NDIS_STRING const	DataPolarityStr = NDIS_STRING_CONST("DataPolarity");
	READ_IPARM(&DataPolarityStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		HdlcCfg.inv = !!INT_DATA(Parm);
	}

	NDIS_STRING const	ReadBurstStr = NDIS_STRING_CONST("ReadBurst");
	READ_IPARM(&ReadBurstStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		HdlcCfg.rburst = !!INT_DATA(Parm);
	}

	NDIS_STRING const	WriteBurstStr = NDIS_STRING_CONST("WriteBurst");
	READ_IPARM(&WriteBurstStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		HdlcCfg.wburst = !!INT_DATA(Parm);
	}

	NDIS_STRING const	XmtQueueLenStr = NDIS_STRING_CONST("XmtQueueLen");
	READ_IPARM(&XmtQueueLenStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		XmtQueueLen = INT_DATA(Parm);
	}

	NDIS_STRING const	RcvQueueLenStr = NDIS_STRING_CONST("RcvQueueLen");
	READ_IPARM(&RcvQueueLenStr);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		RcvQueueLen = INT_DATA(Parm);
	}

	memset(MACAddress, 0, sizeof(MACAddress));

	NDIS_STRING const	NetAddressStr = NDIS_STRING_CONST("NetworkAddress");
	NdisReadConfiguration(&Status, &Parm, ConfigHandle,
						  const_cast < NDIS_STRING * > (&NetAddressStr),
						  NdisParameterString);
	if(Status == NDIS_STATUS_SUCCESS &&
	   Parm->ParameterData.StringData.Length == sizeof(WCHAR) * 12)
	{
		PWCHAR	AddrStr = Parm->ParameterData.StringData.Buffer;
		for(i = 0; i < 12; i++)
		{
			WCHAR const wc = *AddrStr++;
			BYTE		Digit;
			if(wc >= L'0' && wc <= L'9')
			{
				Digit = BYTE(wc - L'0');
			} else if(wc >= L'a' && wc <= L'f')
			{
				Digit = BYTE(wc - L'a' + 10);
			} else if(wc >= L'A' && wc <= L'F')
			{
				Digit = BYTE(wc - L'A' + 10);
			} else
			{
				Digit = 0;
			}

			if(!TestBits(i, 1))
			{
				Digit <<= 4;
			}

			MACAddress[i / 2] |= Digit;
		}
	}

	if(!*PDWORD(MACAddress) && !*PWORD(MACAddress + 4))
	{
		MACAddress[0] = 0x00;
		MACAddress[1] = 0xFF;
		*PWORD(MACAddress + 2) = WORD(rand());
		*PWORD(MACAddress + 4) = WORD(rand());

		WCHAR	Addr[12];
		for(i = 0; i < 12; i++)
		{
			BYTE	Digit = MACAddress[i / 2];
			if(!TestBits(i, 1))
			{
				Digit >>= 4;
			} else
			{
				Digit &= 0xF;
			}

			Addr[i] = WCHAR((Digit < 10) ? Digit + L'0' : Digit - 10 + L'A');
		}

		NDIS_CONFIGURATION_PARAMETER	ParmDesc;
		ParmDesc.ParameterType = NdisParameterString;
		ParmDesc.ParameterData.StringData.Buffer = Addr;
		ParmDesc.ParameterData.StringData.Length = sizeof(Addr);
		ParmDesc.ParameterData.StringData.MaximumLength = sizeof(Addr);
		NdisWriteConfiguration(&Status, ConfigHandle,
							   const_cast < NDIS_STRING * > (&NetAddressStr),
							   &ParmDesc);
		if(Status != NDIS_STATUS_SUCCESS)
		{
			Debug(9, this, "Cannot write random MAC address to registry (%X)",
				  Status);
		}
	}

	Debug(7, this, "Adapter: MAC=%02X-%02X-%02X-%02X-%02X-%02X", MACAddress[0],
		  MACAddress[1], MACAddress[2], MACAddress[3], MACAddress[4],
		  MACAddress[5]);
	Debug(7, this, "HLDC: CRC%s, Fill=%s, Pol=%s, RB=%s, WB=%s, XQL=%u, RQL=%u",
		  HdlcCfg.crc16 ? "16" : "32", HdlcCfg.fill_7e ? "7E" : "FF",
		  HdlcCfg.inv ? "Neg" : "Pos", HdlcCfg.rburst ? "On" : "Off",
		  HdlcCfg.wburst ? "On" : "Off", XmtQueueLen, RcvQueueLen);
	Debug(7, this, "Modem: Rate=%u, %s, Mod=TCPAM%u, AC=%s", ModemCfg.rate,
		  ModemCfg.master ? "Master" : "Slave", 4 << (3 - ModemCfg.mod),
		  ModemCfg.AlwaysConnected ? "On" : "Off");
	Debug(0, this, "ReadConfiguration:EXIT");
	Assert(ModemCfg.mod < Mod_Auto);
	return NDIS_STATUS_SUCCESS;
}

/* -----------------------------------------------------------------------------
 *    Query adapter/driver capabilities
 ------------------------------------------------------------------------------- */
inline NDIS_STATUS
AdapterDesc::MiniportQueryInformation(NDIS_OID Oid, PVOID InformationBuffer,
									  ULONG InformationBufferLength,
									  PULONG BytesWritten, PULONG BytesNeeded)
{
	Debug(0, this, "MiniportQueryInformation:ENTER");
	Check();
	union
	{
		WORD	U16;
		DWORD	U32;
		ULONG64 U64;
	} Data;
	Data.U64 = 0;

	PCVOID		DataAddr = &Data;
	UINT		DataSize = sizeof(Data.U32);
	UINT		MinimalSize = 0;
	NDIS_STATUS StatusToReturn = NDIS_STATUS_SUCCESS;
	switch(Oid)
	{
	case OID_GEN_SUPPORTED_LIST:
		{
			/* OIDs must go in ascending order */
			static UINT const	SupportedOids[] =
			{
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
				OID_GEN_VENDOR_DRIVER_VERSION,
				OID_GEN_PHYSICAL_MEDIUM,
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
				OID_PRIVATE_READ_STAT,
				OID_PRIVATE_CLEAR_STAT,
			};
			DataAddr = SupportedOids;
			DataSize = sizeof(SupportedOids);
			break;
		}

	case OID_GEN_MAC_OPTIONS:
		Data.U32 = NDIS_MAC_OPTION_TRANSFERS_NOT_PEND |
			NDIS_MAC_OPTION_COPY_LOOKAHEAD_DATA |
			NDIS_MAC_OPTION_TRANSFERS_NOT_PEND |
			NDIS_MAC_OPTION_FULL_DUPLEX |
			NDIS_MAC_OPTION_NO_LOOPBACK;
		break;

	case OID_GEN_HARDWARE_STATUS:
		Data.U32 = NdisHardwareStatusReady;
		break;

	case OID_GEN_MEDIA_SUPPORTED:
	case OID_GEN_MEDIA_IN_USE:
		Data.U32 = NdisMedium802_3;
		break;

	case OID_GEN_PHYSICAL_MEDIUM:
		Data.U32 = NdisPhysicalMediumDSL;
		break;

	case OID_GEN_MEDIA_CONNECT_STATUS:
		Data.U32 = (ModemCfg.AlwaysConnected || ModemState == ACTIVE) ? (NdisMediaStateConnected) : (NdisMediaStateDisconnected);
		break;

	case OID_GEN_MAXIMUM_LOOKAHEAD:
	case OID_GEN_CURRENT_LOOKAHEAD:
	case OID_GEN_MAXIMUM_FRAME_SIZE:
		Data.U32 = ETHERNET_MAX_SIZE - ETHERNET_HEADER_SIZE;
		break;

	case OID_GEN_MAXIMUM_TOTAL_SIZE:
		Data.U32 = ETHERNET_MAX_SIZE;
		break;

	case OID_GEN_LINK_SPEED:
		Data.U32 = ModemCfg.rate * 10;
		break;

	case OID_GEN_TRANSMIT_BUFFER_SPACE:
	case OID_GEN_RECEIVE_BUFFER_SPACE:
	case OID_GEN_RECEIVE_BLOCK_SIZE:
		Data.U32 = 1514;
		break;

	case OID_GEN_TRANSMIT_BLOCK_SIZE:
		Data.U32 = 12;
		break;

	case OID_GEN_TRANSMIT_QUEUE_LENGTH:
		Data.U32 = 128;
		break;

	case OID_GEN_MAXIMUM_SEND_PACKETS:
		Data.U32 = 32;
		break;

	case OID_GEN_VENDOR_ID:
		NdisMoveMemory(&Data.U32, MACAddress, 3);
		Data.U32 &= 0xFFFFFF00;
		break;

	case OID_GEN_VENDOR_DESCRIPTION:
		DataAddr = DeviceNameString;
		DataSize = sizeof(DeviceNameString);
		break;

	case OID_GEN_VENDOR_DRIVER_VERSION:
		Data.U32 = ((VersionMajor) << 16 | VersionMinor);
		break;

	case OID_GEN_DRIVER_VERSION:
		Data.U16 = WORD(NDIS_MAJOR_VERSION << 8) | NDIS_MINOR_VERSION;
		DataSize = sizeof(Data.U16);
		break;

	case OID_802_3_PERMANENT_ADDRESS:
	case OID_802_3_CURRENT_ADDRESS:
		DataAddr = MACAddress;
		DataSize = sizeof(MACAddress);
		break;

	case OID_802_3_MAXIMUM_LIST_SIZE:
		Data.U32 = MULTICAST_LIST_SIZE;
		break;

	case OID_802_3_MULTICAST_LIST:
		DataAddr = MulticastList;
		DataSize = MulticastCount * ETH_LENGTH_OF_ADDRESS;
		break;

	/* case OID_GEN_CURRENT_PACKET_FILTER: { Data.U32 = FilterMode; break; } */
	case OID_GEN_XMIT_OK:
		Data.U64 = XmitGood;
		MinimalSize = sizeof(Data.U32);
		DataSize = sizeof(Data.U64);
		break;

	case OID_GEN_RCV_OK:
		Data.U64 = RcvdGood;
		MinimalSize = sizeof(Data.U32);
		DataSize = sizeof(Data.U64);
		break;

	case OID_GEN_XMIT_ERROR:
		Data.U64 = XmitBad;
		MinimalSize = sizeof(Data.U32);
		DataSize = sizeof(Data.U64);
		break;

	case OID_GEN_RCV_ERROR:
		Data.U64 = CrcErrors;
		MinimalSize = sizeof(Data.U32);
		DataSize = sizeof(Data.U64);
		break;

	/* * we must support these OIDs though ones aren't actual for * SG16-PCI */
	case OID_GEN_RCV_NO_BUFFER:
	case OID_802_3_RCV_ERROR_ALIGNMENT:
	case OID_802_3_XMIT_ONE_COLLISION:
	case OID_802_3_XMIT_MORE_COLLISIONS:
		Data.U32 = 0;
		break;

	case OID_PRIVATE_READ_STAT:
		{					/* internal statistics read - start */
			if(InformationBufferLength >= 4 &&
			   *PDWORD(InformationBuffer) == StatKey)
			{
				if(InformationBufferLength < sizeof(Stat))
				{
					DataSize = sizeof(Stat);
					break;	/* Status will be set before exit */
				}

				Assert(!QI_InfoBuffer);
				QI_InfoBuffer = InformationBuffer;
				QI_BytesWritten = BytesWritten;
				QI_BytesNeeded = BytesNeeded;
				ReadModemStat(0);
				StatusToReturn = NDIS_STATUS_PENDING;
			} else
			{
				StatusToReturn = NDIS_STATUS_FAILURE;
			}

			break;
		}

	case OID_PRIVATE_CLEAR_STAT:
		{					/* clear statistics */
			if(InformationBufferLength == 4 &&
			   *PDWORD(InformationBuffer) == StatKey)
			{
				Debug(5, this, "Resetting modem stat counters");
				Assert(!QI_InfoBuffer);
				ResetModemStat(0);
				StatusToReturn = NDIS_STATUS_PENDING;
				*BytesNeeded = *BytesWritten = 0;
			} else
			{
				StatusToReturn = NDIS_STATUS_FAILURE;
			}

			break;
		}

	default:
		{
			StatusToReturn = NDIS_STATUS_INVALID_OID;
		}
	}

	if(!MinimalSize)
	{
		MinimalSize = DataSize;
	}

	*BytesNeeded = DataSize;
	if(StatusToReturn == NDIS_STATUS_SUCCESS)
	{
		if(MinimalSize <= InformationBufferLength)
		{
			UINT const	ActualSize = min(InformationBufferLength, DataSize);
			NdisMoveMemory(InformationBuffer, DataAddr, ActualSize);
			*BytesWritten = ActualSize;
		} else
		{
			StatusToReturn = NDIS_STATUS_INVALID_LENGTH;
			*BytesWritten = 0;
		}
	}

	Debug(0, this, "MiniportQueryInformation:EXIT");
	return StatusToReturn;
}

/* -----------------------------------------------------------------------------
 *
 ------------------------------------------------------------------------------- */
NDIS_STATUS
MiniportQueryInformationOuter(NDIS_HANDLE Context, NDIS_OID Oid,
							  PVOID InformationBuffer,
							  ULONG InformationBufferLength, PULONG BytesWritten,
							  PULONG BytesNeeded)
{
	return PSG16_ADAPTER(Context)->MiniportQueryInformation(Oid,
															InformationBuffer, InformationBufferLength,
															BytesWritten, BytesNeeded);
}

/* -----------------------------------------------------------------------------
 *    Set adapter/driver parameters
 ------------------------------------------------------------------------------- */
inline NDIS_STATUS
AdapterDesc::MiniportSetInformation(NDIS_OID Oid, PVOID InformationBuffer,
									ULONG InformationBufferLength,
									PULONG BytesRead, PULONG BytesNeeded)
{
	Debug(0, this, "MiniportSetInformation:ENTER");
	Check();

	NDIS_STATUS StatusToReturn = NDIS_STATUS_SUCCESS;
	*BytesRead = 0;
	*BytesNeeded = 0;
	switch(Oid)
	{
	case OID_GEN_CURRENT_LOOKAHEAD:
		if(InformationBufferLength != 4)
		{
			StatusToReturn = NDIS_STATUS_INVALID_LENGTH;
			*BytesNeeded = 4;
			break;
		}

		*BytesRead = 4;
		break;

	case OID_802_3_MULTICAST_LIST:
		if(InformationBufferLength % ETH_LENGTH_OF_ADDRESS)
		{
			StatusToReturn = NDIS_STATUS_INVALID_LENGTH;
		} else if(InformationBufferLength > MULTICAST_LIST_SIZE *
				ETH_LENGTH_OF_ADDRESS)
		{
			StatusToReturn = NDIS_STATUS_MULTICAST_FULL;
		} else
		{
			NdisMoveMemory(MulticastList, InformationBuffer,
						   InformationBufferLength);
			MulticastCount = InformationBufferLength / ETH_LENGTH_OF_ADDRESS;
			*BytesRead = InformationBufferLength;
		}

		break;

	case OID_GEN_CURRENT_PACKET_FILTER:
		{
			if(UINT(InformationBufferLength) != 4)
			{
				StatusToReturn = NDIS_STATUS_INVALID_LENGTH;
				break;
			}

			DWORD const Filter = *PDWORD(InformationBuffer);
			Debug(5, this, "Set: OID_GEN_CURRENT_PACKET_FILTER=%X", Filter);
			if(Filter &~(NDIS_PACKET_TYPE_DIRECTED | NDIS_PACKET_TYPE_BROADCAST |
			   NDIS_PACKET_TYPE_PROMISCUOUS | NDIS_PACKET_TYPE_MULTICAST |
			   NDIS_PACKET_TYPE_ALL_MULTICAST))
			{
				StatusToReturn = NDIS_STATUS_NOT_SUPPORTED;
			} else
			{
				/* All supported flags are in LSB */
				FilterMode = BYTE(Filter);
				*BytesRead = 4;
			}

			break;
		}

	default:
		{
			StatusToReturn = NDIS_STATUS_INVALID_OID;
		}
	}

	Debug(0, this, "MiniportSetInformation:EXIT");
	return StatusToReturn;
}

/* -----------------------------------------------------------------------------
 *
 ------------------------------------------------------------------------------- */
NDIS_STATUS
MiniportSetInformationOuter(NDIS_HANDLE Context, NDIS_OID Oid,
							PVOID InformationBuffer,
							ULONG InformationBufferLength, PULONG BytesRead,
							PULONG BytesNeeded)
{
	return PSG16_ADAPTER(Context)->MiniportSetInformation(Oid,
														  InformationBuffer, InformationBufferLength,
														  BytesRead, BytesNeeded);
}

/* -----------------------------------------------------------------------------
 *    Complete read stat data
 ------------------------------------------------------------------------------- */
void
AdapterDesc::ReadModemStatComplete(bool Success)
{
	NDIS_STATUS Status = NDIS_STATUS_FAILURE;
	if(Success)
	{
		Assert(QI_InfoBuffer && QI_BytesWritten && QI_BytesNeeded);
		Debug(5, this, "Returning stat data");
		Stat.sent_pkts = XmitGood;
		Stat.rcvd_pkts = RcvdGood;
		Stat.crc_errs = CrcErrors;

		static char const	NMR_offsets[] = { -12, 0, +12, +36 };
		Assert(ModemCfg.mod < NumArrayElems(NMR_offsets));
		Stat.nmr = BYTE(Stat.nmr + NMR_offsets[ModemCfg.mod]);
		NdisMoveMemory(QI_InfoBuffer, &Stat, sizeof(Stat));
		*QI_BytesWritten = *QI_BytesNeeded = sizeof(Stat);
		QI_InfoBuffer = NULL;
		Status = NDIS_STATUS_SUCCESS;
	}

	NdisMQueryInformationComplete(DriverHandle, Status);
}

/* -----------------------------------------------------------------------------
 *    Complete reset stat data
 ------------------------------------------------------------------------------- */
void
AdapterDesc::ResetModemStatComplete(bool Success)
{
	NDIS_STATUS Status = NDIS_STATUS_FAILURE;
	if(Success)
	{
		Assert(!QI_InfoBuffer);
		Debug(5, this, "Reset modem stat done");

		/* Clear software stats */
		NdisZeroMemory(&Stat, sizeof(Stat));
		Status = NDIS_STATUS_SUCCESS;
	}

	NdisMSetInformationComplete(DriverHandle, Status);
}
