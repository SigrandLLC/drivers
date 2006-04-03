/* transceiver chip set definitions */

struct cx28975_cmdarea {        // Modem command area

  BYTE volatile intr_host;      // Write generates interrupt, read clears
  BYTE volatile intr_8051;      // The same as above
  BYTE volatile map_version;

  BYTE volatile in_dest;
  BYTE volatile in_opcode;
  BYTE volatile in_zero;
  BYTE volatile in_length;
  BYTE volatile in_csum;
  BYTE volatile in_data [75];
  BYTE volatile in_datasum;

  BYTE volatile out_dest;
  BYTE volatile out_opcode;

  BYTE volatile out_ack;

    #define MACK_UnsolInt 0x80
    #define MACK_ApiResponse 0x20
    #define MACK_Status 0x1F

  BYTE volatile out_length;
  BYTE volatile out_csum;
  BYTE volatile out_data [75];
  BYTE volatile out_datasum;             // Marked as "reserved" in API docs

  struct { BYTE volatile Data [0x1E8 - 0xA5 + 1]; } Reserved1;

  BYTE volatile E1Framer [0x5C];

  struct { BYTE volatile Data [0x3BF - 0x245 + 1]; } Reserved2;

  BYTE volatile Status_1;

    #define MST1_LOS 0x1
    #define MST1_DyingGasp 0x2
    #define MST1_LOSW 0x4
    #define MST1_NTR_Lock 0x8
    #define MST1_NMR_OK 0x10
    #define MST1_FatalError 0x20
    #define MST1_AsmStatus 0xC0

    #define _ASM_STAT_IDLE 0x00
    #define _ASM_STAT_SUCCESS 0x40
    #define _ASM_STAT_DEACTIVATED 0x80
    #define _ASM_STAT_IN_PROGRESS 0xC0

  BYTE volatile Status_2;
  BYTE volatile Status_3;
  BYTE volatile Status_4;
  BYTE volatile Status_5;
  BYTE volatile Status_6;
  BYTE volatile Status_7;
  BYTE volatile Status_8;

    #define MST8_UnsolApiCmd 0x1
    #define MST8_AsmTransition 0x2
    #define MST8_DyingGasp 0x8

  struct { BYTE volatile Data [0x3FF - 0x3C8 + 1]; } Reserved3;

};

Assume (offsetof (cx28975_cmdarea, in_data) == 0x8);
Assume (offsetof (cx28975_cmdarea, in_datasum) == 0x53);
Assume (offsetof (cx28975_cmdarea, out_data) == 0x59);
Assume (offsetof (cx28975_cmdarea, out_datasum) == 0xA4);
Assume (offsetof (cx28975_cmdarea, E1Framer) == 0x1E9);
Assume (offsetof (cx28975_cmdarea, Status_1) == 0x3C0);
Assume (offsetof (cx28975_cmdarea, Status_8) == 0x3C7);

Assume (sizeof (cx28975_cmdarea) == 0x400);





enum Modulations { Mod_TCPAM32, Mod_TCPAM16, Mod_TCPAM8, Mod_TCPAM4, Mod_Auto };
enum Annexes { AnnexA=1, AnnexB=2, AnnexF=3 };


/* SHDSL parameters*/
#define MAX_AUTO_RATE	2304
#define MIN_AUTO_RATE	192
#define MAX_RATE	6016
#define MIN_RATE	64
#define MAX_REMCFGF_RATE	5696
#define MAX_REMCFGAB_RATE	2304
#define MIN_REMCFG_RATE		192

class cx28975_cfg { // Chipset configuration

  public:

  UINT rate;
  Modulations mod;
  bool master;
  bool AlwaysConnected;

/* -------- Added by me ------------*/
  bool remcfg;
  Annexes annex;
/*--------- End --------------------*/
  

  cx28975_cfg (void) {

    rate = Modem_LineRateDef;
    mod = Mod_Auto;
    master = Modem_MasterModeDef;
    AlwaysConnected = Modem_AlwaysConnectedDef;
	remcfg=0;
	annex=AnnexA;

  }

};





enum CX28975_API_commands {

  _DSL_RESET_SYSTEM	       = 0x00,
  _DSL_SYSTEM_ENABLE	       = 0x01,
  _DSL_AFE_CONFIG		   = 0x02,
  _DSL_TRAINING_MODE	       = 0x03,
  _DSL_CLOCK_CONFIG	       = 0x04,
  _DSL_PCM_MF_LEN		   = 0x05,
  _DSL_SYSTEM_CONFIG	       = 0x06,
  _DSL_LOOPBACK		       = 0x09,
  _DSL_ACTIVATION		   = 0x0b,
  _DSL_FORCE_DEACTIVATE	   = 0x0c,
  _DSL_TEST_MODE		       = 0x0d,
  _DSL_DATA_RATE		       = 0x0e,
  _DSL_PREACTIVATION_CFG	   = 0x0f,
  _DSL_FR_PCM_CONFIG	       = 0x10,
  _DSL_FR_HDSL_CONFIG	   = 0x11,
  _DSL_PCM_CLK_CONF	       = 0x12,
  _AFE_TX_GAIN		       = 0x13,
  _DSL_PREACT_USER_INFO	   = 0x14,
  _DSL_PREACT_RATE_LIST	   = 0x15,
  _DSL_TX_ISO_PULSE	       = 0x16,
  _BP_ERLE_TEST_MODE	       = 0x18,
  _DSL_MULTI_PAIR_CONFIG	   = 0x19,
  _DSL_NB_MULTI_RATE_CONFIG = 0x1a,
  _DSL_MULTI_RATE_CONFIG	   = 0x1b,
  _ATM_PHY_MODE		       = 0x1c,
  _ATM_PHY_UTOPIA_CONFIG	   = 0x1d,
  _ATM_PHY_IF_MODE	       = 0x1e,
  _ATM_PHY_INJECT_HEC_ERROR = 0x1f,
  _ATM_PHY_CONFIG		   = 0x20,
  _DSL_TNB_BER_STATE	       = 0x21,
  _DSL_RNB_BER_STATE        = 0x22,
  _DSL_TP_BER_STATE	       = 0x23,
  _DSL_RP_BER_STATE	       = 0x24,
  _DSL_PRBS_CONFIGURE	   = 0x25,
  _DSL_CONST_FILL		   = 0x26,
  _DSL_DBANK		           = 0x27,
  _DSL_NB_CONFIG		       = 0x28,
  _DSL_TNB_FRM_OFST	       = 0x29,
  _DSL_RNB_FRM_OFST	       = 0x2a,
  _DSL_TP_FRM_OFST	       = 0x2e,
  _DSL_RP_FRM_OFST	       = 0x2f,
  _DSL_TP_MAPPER_VALUE	   = 0x30,
  _DSL_TP_MAPPER_WRITE	   = 0x31,
  _DSL_RP_MAPPER_VALUE	   = 0x32,
  _DSL_RP_MAPPER_WRITE	   = 0x33,
  _DSL_TH_MAPPER_VALUE	   = 0x34,
  _DSL_TH_MAPPER_WRITE	   = 0x35,
  _DSL_RH_MAPPER_VALUE	   = 0x36,
  _DSL_RH_MAPPER_WRITE	   = 0x37,
  _DSL_TNB_MAPPER_VALUE	   = 0x38,
  _DSL_TNB_MAPPER_WRITE	   = 0x39,
  _DSL_RNB_MAPPER_VALUE	   = 0x3a,
  _DSL_RNB_MAPPER_WRITE	   = 0x3b,
  _DSL_CLEAR_ERROR_CTRS	   = 0x40,
  _DSL_INJECT_CRC_ERROR	   = 0x41,
  _DSL_THRESHOLDS		   = 0x43,
  _DSL_FR_SET_STATE_MACHINE = 0x4a,
  _DSL_FR_TNB_RESET	       = 0x4b,
  _DSL_FR_RNB_RESET	       = 0x4c,
  _EOC_RESET		           = 0x4d,
  _DSL_FR_TX_RESET	       = 0x4e,
  _DSL_FR_RX_RESET	       = 0x4f,
  _DSL_INTR_HOST_MASK	   = 0x50,
  _DSL_INTR_API_SUBMASK	   = 0x51,
  _DSL_DOWNLOAD_START	   = 0x53,
  _DSL_DOWNLOAD_DATA	       = 0x54,
  _DSL_DOWNLOAD_END	       = 0x55,
  _DSL_DPLL_CLOCK_GEN	   = 0x58,
  _DSL_NB_DPLL_CLOCK_GEN	   = 0x59,
  _DSL_WRITE_REG		       = 0x75,
  _DSL_WRITE_AFE		       = 0x76,

  _DSL_NO_COMMAND          = 0xFF,

};





enum CX28975_status_commands {

  _DSL_READ_CONTROL	           = 0x80,
  _DSL_FAR_END_ATTEN	           = 0x82,
  _DSL_NOISE_MARGIN	           = 0x83,
  _DSL_STATUS		           = 0x85,
  _DSL_PREACT_GET_FE_CAPS	   = 0x88,
  _DSL_PREACT_GET_OPT_DATA_RATE = 0x89,
  _DSL_VERSIONS		           = 0x8a,
  _DSL_TP_BER_RESULTS	       = 0x8c,
  _DSL_RP_BER_RESULTS	       = 0x8d,
  _DSL_STAGE_NUMBER	           = 0x8f,
  _DSL_AFE_SETTING	           = 0x90,
  _DSL_TNB_BER_RESULTS	       = 0x91,
  _DSL_RNB_BER_RESULTS	       = 0x92,
  _BP_ERLE_RESULTS	           = 0x93,
  _DSL_POWER_BACK_OFF_RESULT	   = 0x94,
  _DSL_OPER_ERR_CTRS	           = 0x9c,
  _DSL_TIME		               = 0x9d,
  _DSL_HDSL_PERF_ERR_CTRS	   = 0x9e,
  _DSL_READ_REG		           = 0xa0,
  _DSL_READ_AFE		           = 0xa1,
  _DSL_SYSTEM_PERF_ERR_CTRS	   = 0xa2,
  _DSL_TP_MAPPER_READ	       = 0xa3,
  _DSL_RP_MAPPER_READ	       = 0xa4,
  _DSL_TH_MAPPER_READ	       = 0xa5,
  _DSL_RH_MAPPER_READ	       = 0xa6,
  _DSL_TNB_MAPPER_READ	       = 0xa7,
  _DSL_RNB_MAPPER_READ	       = 0xa8,
  _EOC_RX_GET_STATS	           = 0xae,
  _EOC_TX_SEND_COMMAND	       = 0xb0,
  _EOC_RX_GET_MSG		       = 0xb1,
  _EOC_TX_GET_MSG_STATUS	       = 0xb2,
  _EOC_TX_DELETE_MSG	           = 0xb3,
  _ATM_PHY_OPER_ERR_CTRS	       = 0xb8,
  _ATM_PHY_PERF_ERR_CTRS	       = 0xb9,
  _ATM_PHY_CELL_CTRS	           = 0xba

};





enum CX28975_acknowledge_status_codes {

  _ACK_NOT_COMPLETE	 = 0x00,
  _ACK_PASS		     = 0x01,
  _ACK_BUSY		     = 0x02,
  _ACK_NOT_APPLICABLE = 0x03,
  _ACK_INVALID_DEST	 = 0x04,
  _ACK_INVALID_OPCODE = 0x05,
  _ACK_INVALID_LENGTH = 0x06,
  _ACK_INVALID_DATA	 = 0x07,
  _ACK_INVALID_CHKSUM = 0x08,
  _ACK_NO_RESULT		 = 0x09,
  _ACK_NOT_AVAILABLE	 = 0x0a,
  _ACK_BOOT_WAKE_UP	 = 0x0d,
  _ACK_OPER_WAKE_UP	 = 0x0e

};




enum {                          // _DSL_CLEAR_ERROR_CTRS modes

  _CLEAR_ALL_COUNTERS = 0,
  _CLEAR_SYSTEM_ERR_CTRS = 4,

};





// Terminal types
// ==============

#define _DSL_HTUC (0x00 << 3)
#define _DSL_HTUR (0x01 << 3)
#define _DSL_REGC (0x02 << 3)
#define _DSL_REGR (0x03 << 3)





enum ModemStates { DOWN, ACTIVATION, ACTIVE };