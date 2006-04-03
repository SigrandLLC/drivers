//$T main.cpp GC 1.138 06/24/04 01:00:47


//$6
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//


#include <windows.h>
#include <setupapi.h>

#pragma warning (disable: 4201)

#include <winioctl.h>
#include <stdio.h>
#include <string.h>
#include <math.h>


#include <ntddndis.h>

#include "global.h"

#include "api.h"




#pragma comment (lib, "setupapi")
#pragma comment (lib, "uuid")




#ifdef _DEBUG

  #define Debug DebugPrint

#else

  #define Debug(x)
  #pragma warning (disable: 4002 /* too many actual parameters for macro */)

#endif




// #####################################################################
//
// Macros
// ======
//
// #####################################################################





char inline CharToUpper (TCHAR const c) {

  return char (CharUpper (PTSTR (WORD (c))));
  
}





static char const NetworkCardsKeyName [] = "SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion\\NetworkCards";

static char const *AdapterNameTpl;

static bool DoReset = false;

static bool Found = false;





//
// Place error text
// ================
//

PCSTR _UniCall ErrorText (DWORD Code) {

  static char Text [256];

  wsprintf (Text, "%u/%u - ", HRESULT_FACILITY (Code), HRESULT_CODE (Code));

  UINT Offset = lstrlen (Text);

  LPSTR cp = Text + Offset;
  UINT Avail = sizeof (Text) - Offset - 1;

  FormatMessage (
    FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS | FORMAT_MESSAGE_MAX_WIDTH_MASK,
    NULL,
    Code,
    MAKELANGID (LANG_NEUTRAL, SUBLANG_DEFAULT),
    cp,
    Avail,
    NULL 
  );

  cp += strlen (cp) - 1;

  if (*cp == ' ') {

    *cp = '\0';

  }

  return Text;

}





//
// Retrieve last error information
// ===============================
//

PCSTR _UniCall LastErrorText (void) {

  return ErrorText (GetLastError ());

}





//
// Check for template matching
// ===========================
//
// '?' - any character
// '*' - any character sequence or empty
//

bool _UniCall IsTemplateMatching (PCSTR Template, PCSTR String) {

  while (*Template || *String) {

    switch (*Template) {

      case '?': {

        if (*String) {

          Template++;
          String++;

          break;

        } else {

          return false;

        }

      }

      case '*': {

        if (*++Template) {

          while (*String) {

            if (
              CharToUpper (*String) == CharToUpper (*Template)
              && IsTemplateMatching (Template, String)
            ) {

              return true;

            }

            String++;

          }

          return false;

        } else {

          return true;

        }

      }

      default: {

        if (CharToUpper (*Template++) != CharToUpper (*String++)) {

          return false;

        }

      }

    }

  }

  return true;

}





//
// Debugging output
// ================
//

#ifdef _DEBUG

void _cdecl DebugPrint (PCSTR Fmt, ...) {

  char Str [256];

  strcpy (Str, "sg16stat: ");

  va_list Args;

  va_start (Args, Fmt);

  wvsprintf (Str + lstrlen (Str), Fmt, Args);

  va_end (Args);

  lstrcat (Str, "\n");

  OutputDebugString (Str);

}

#endif // _DEBUG





//
// Print with newline
// ==================
//

inline void _UniCall Print (FILE *Stream, PCSTR Fmt, va_list Args) {

  vfprintf (Stream, Fmt, Args);

  fputc ('\n', Stream);

}





//
// Output to stdout
// ================
//

void _cdecl StdOut (PCSTR Fmt, ...) {

  va_list Args;

  va_start (Args, Fmt);

  Print (stdout, Fmt, Args);

  va_end (Args);

}





//
// Output to stderr
// ================
//

void _cdecl StdErr (PCSTR Fmt, ...) {

  va_list Args;

  va_start (Args, Fmt);

  Print (stdout, Fmt, Args);

  va_end (Args);

}





//
// Get friendly name of the adapter
// ================================
//

bool GetFriendlyName (PCSTR const Guid, PSTR const Name, DWORD NameLen) {

  HKEY NetKey = NULL;

  bool Success = false;

  do {

    #define GUID_DEVCLASS_NET_A "{4D36E972-E325-11CE-BFC1-08002BE10318}"

    char NetKeyName [256] = "SYSTEM\\CurrentControlSet\\Control\\Network\\" GUID_DEVCLASS_NET_A "\\";

    strcat (NetKeyName, Guid);

    strcat (NetKeyName, "\\Connection");

    LONG Res = RegOpenKeyEx (
      HKEY_LOCAL_MACHINE,
      NetKeyName,
      0,
      KEY_READ,
      &NetKey
    );

    if (Res != ERROR_SUCCESS) {

      StdErr ("Cannot open %s (%s)", NetKeyName, ErrorText (Res));

      break;

    }

    static char const NameValueName [] = "Name";

    DWORD ValueType;

    Res = RegQueryValueEx (
      NetKey,
      NameValueName,
      NULL,
      &ValueType,
      PBYTE (Name),
      &NameLen
    );

    if (Res != ERROR_SUCCESS) {

      StdErr ("Cannot get %s value (%s)", NameValueName, ErrorText (Res));

      break;

    }

    Success = true;

  } while (False);

  if (NetKey) {

    RegCloseKey (NetKey);

  }

  return Success;

}





// #####################################################################
// #####################################################################

static void Format (StatDesc const * const stat) {

  StdOut (
    "\n"
    "Received frames: %I64u\n"
    "Sent frames:     %I64u\n"
    "CRC errors:      %I64u\n"
    "FIFO overflows:  %u\n"
    "FIFO underflows: %u",
    stat->rcvd_pkts,
    stat->sent_pkts,
    stat->crc_errs,
    stat->ufl_errs,
    stat->ofl_errs
  );

  //
  // if( stat->last_time ) printf( "\tLast connect at %s\n", ctime(
  // &stat->last_time ) );
  //

  switch (stat->status_1 >> 6) {

    case 0: {

      static char const *szStatus3 [] = {
        "Bad NMR",
        "Frequency lock failed",
        "Pre-activation failed",
        "Sync word detect failed"
      };

      StdOut ("\nInactive (%s)", szStatus3 [stat->status_3 & 0x0F]);

      break;

    }

    case 1:

      StdOut (
        "\nLink established\n"
        "\n"
        "Line attenuation: %5.1f dB\n"
        "Noise Margin:     %+5.1f dB\n"
        "Power back-off:   near-end - %u dB, far-end - %u dB\n"
        "\n"
        "SHDSL counters:\n"
        "  Out-of-sync:     %u\n"
        "  Loss of signal:  %u\n"
        "  Bad CRC:         %u\n"
        "  Segment defect:  %u\n"
        "  Segment anomaly: %u\n"
        "",
        ((double) stat->attenuat) / 2.0,
        ((double) stat->nmr) / 2.0,
        stat->tpbo,
        stat->rpbo,
        stat->losw,
        stat->losd,
        stat->crc,
        stat->segd,
        stat->sega
      );

      break;

    case 2:

      StdOut ("\nNo link");
      break;

    case 3:

      StdOut ("\nLink negotiation in progress");
      break;

  }

}





//
// #####################################################################
// #####################################################################
//

static void PrintStat (PCSTR const Guid) {

  NDIS_OID OidCode;
  DWORD ReturnedCount;
  char DevName [64], Vendor [64];
  BYTE MACAddr [6];

  strcpy (DevName, "\\\\.\\");
  strcat (DevName, Guid);

  HANDLE handle = INVALID_HANDLE_VALUE;

  do {

    handle = CreateFile (
      DevName,
      GENERIC_READ,
      FILE_SHARE_READ | FILE_SHARE_WRITE,
      NULL,
      OPEN_EXISTING,
      0,
      INVALID_HANDLE_VALUE
    );

    if (handle == INVALID_HANDLE_VALUE) {

      if (GetLastError () != ERROR_FILE_NOT_FOUND) {

        StdErr ("Cannot open driver %s (%s)", Guid, LastErrorText ());

      }

      break;

    }

    OidCode = OID_GEN_VENDOR_DESCRIPTION;

    if (
      !DeviceIoControl (
        handle,
        IOCTL_NDIS_QUERY_GLOBAL_STATS,
        &OidCode,
        sizeof OidCode,
        Vendor,
        sizeof (Vendor),
        &ReturnedCount,
        NULL
      )
    ) {

      StdErr ("Cannot query vendor desc (%s)", LastErrorText ());

      break;

    }
      
    if (strncmp (Vendor, "Sigrand", 6)) {

      break;

    }

    char Name [256];

    if (!GetFriendlyName (Guid, Name, sizeof (Name))) {

      strcpy (Name, "?");

    }

    if (!IsTemplateMatching (AdapterNameTpl, Name)) {

      break;

    }

    Found = true;

    StdOut ("\n\n%s (%s)", Name, Guid);

    if (DoReset) {

      OidCode = OID_PRIVATE_CLEAR_STAT;

      static DWORD KeyId = StatKey;

      if (
        DeviceIoControl (
          handle,
          IOCTL_NDIS_QUERY_GLOBAL_STATS,
          &OidCode,
          sizeof OidCode,
          &KeyId,
          sizeof (KeyId),
          &ReturnedCount,
          NULL
        )
      ) {

        StdOut ("Modem stat counters are resetted");

      } else {

        StdErr ("Cannot clear stat (%s)", LastErrorText ());

      }

      break;

    }

    OidCode = OID_802_3_CURRENT_ADDRESS;

    if (
      !DeviceIoControl (
        handle,
        IOCTL_NDIS_QUERY_GLOBAL_STATS,
        &OidCode,
        sizeof OidCode,
        MACAddr,
        sizeof (MACAddr),
        &ReturnedCount,
        NULL
      )
    ) {

      StdErr ("Cannot get MAC address (%s)", LastErrorText ());

      break;

    }

    if (ReturnedCount != 6) {

      StdErr ("Incorrect MAC address length (%u)", ReturnedCount);

      break;

    }

    StdOut (
      "Network address: %02X-%02X-%02X-%02X-%02X-%02X",
      MACAddr [0],
      MACAddr [1],
      MACAddr [2],
      MACAddr [3],
      MACAddr [4],
      MACAddr [5]
    );


    StatDesc StatBuffer;

    OidCode = OID_PRIVATE_READ_STAT;

    *PDWORD (&StatBuffer) = StatKey;

    if (
      !DeviceIoControl (
        handle,
        IOCTL_NDIS_QUERY_GLOBAL_STATS,
        &OidCode,
        sizeof (OidCode),
        &StatBuffer,
        sizeof (StatBuffer),
        &ReturnedCount,
        NULL
      )
    ) {

      StdErr ("Cannot read stat (%s)", LastErrorText ());

      break;

    }

    if (ReturnedCount != sizeof (StatDesc)) {

      StdErr ("Returned %u bytes instead of %u", ReturnedCount, sizeof (StatDesc));

      break;

    }

    Format (&StatBuffer);

  } while (False);

  if (handle != INVALID_HANDLE_VALUE) {

    CloseHandle (handle);

  }

}





//
// #####################################################################
// #####################################################################
//
int _cdecl main (int ArgC, char *ArgV []) {

  int ExitCode = 1;

  HKEY MainKey = NULL;

  do {

    if (ArgC == 1) {

      StdOut (
        "\n"
        "usage: sg16stat [-r] <Name>\n"
        "\n"
        "  <Name> - adapter name template (? and * allowed)\n"
        "  -r - reset modem stat counters\n"
        ""
      );

      ExitCode = 1;

      break;

    }

    for (int ArgN = 1; ArgN < ArgC && ArgV [ArgN] [0] == '-'; ArgN++) {

      if (!strcmp (ArgV [ArgN] + 1, "r")) {

        DoReset = true;

      }

    }

    if (ArgN == ArgC) {

      StdErr ("No adapter name specified");

      ExitCode = 2;

      break;

    }

    AdapterNameTpl = ArgV [ArgN];

    LONG Res = RegOpenKeyEx (
      HKEY_LOCAL_MACHINE,
      NetworkCardsKeyName,
      0,
      KEY_READ,
      &MainKey
    );

    if (Res != ERROR_SUCCESS) {

      StdErr ("Cannot open %s (%s)", NetworkCardsKeyName, ErrorText (Res));

      ExitCode = 3;

      break;

    }

    for (UINT i = 0; ; i++) {

      char CardKeyName [64];

      Res = RegEnumKey (MainKey, i, CardKeyName, sizeof (CardKeyName));
      
      if (Res != ERROR_SUCCESS) {

        if (Res != ERROR_NO_MORE_ITEMS) {

          StdErr (
            "Cannot enumerate %s (%s)",
            NetworkCardsKeyName, ErrorText (Res)
          );

        }

        break;

      }

      HKEY SrvKey = NULL;

      do {

        Res = RegOpenKeyEx (MainKey, CardKeyName, 0, KEY_READ, &SrvKey);

        if (Res != ERROR_SUCCESS) {

          StdErr ("Cannot open %s (%s)", CardKeyName, ErrorText (Res));

          break;

        }

        static char const ServiceValueName [] = "ServiceName";

        char Data [128];

        DWORD DataLen = sizeof (Data);

        DWORD ValueType;

        Res = RegQueryValueEx (
          SrvKey,
          ServiceValueName,
          NULL,
          &ValueType,
          PBYTE (Data),
          &DataLen
        );


        if (Res != ERROR_SUCCESS) {

          StdErr ("Cannot get %s value (%s)", ServiceValueName, ErrorText (Res));

          break;

        }

        PrintStat (Data);

      } while (False);

      if (SrvKey) {

        RegCloseKey (SrvKey);

      }

    }

    if (!Found) {

      StdErr ("Adapter %s not found", AdapterNameTpl);

    }
      
  } while (False);

  if (MainKey) {

    RegCloseKey (MainKey);

  }

  return ExitCode;

}
