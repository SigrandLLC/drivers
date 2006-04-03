#define DeviceName "Sigrand SG16-PCI SHDSL Modem"

#define OID_PRIVATE_READ_STAT 0xFF020210
#define OID_PRIVATE_CLEAR_STAT 0xFF020211

#define StatKey 'stat'

struct StatDesc {               // Statistics

  /* SG16 controller statistics */

  ULONG64  sent_pkts;
  ULONG64  rcvd_pkts;
  ULONG64  crc_errs;

  ULONG  ufl_errs;
  ULONG  ofl_errs;
  ULONG  attempts;
  LARGE_INTEGER last_time;

  /* CX28975 transceiver statistics */

  BYTE  attenuat;
  BYTE  nmr;
  BYTE  tpbo;
  BYTE  rpbo;
  BYTE  status_1;
  BYTE  status_3;

  WORD losw;                    // These 5 fields are set indirectly by first
  WORD segd;                    // Order cannot be changed
  WORD crc;
  WORD sega;
  WORD losd;

};
