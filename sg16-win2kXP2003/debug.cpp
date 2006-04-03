#include "common.h"

#include <stdio.h>
#include <stdarg.h>



#ifdef _DEBUG



UINT DebugLevel = 5;



void _UniCall DebugVPrintf (
  UINT const Level,
  AdapterDesc const * const Adapter,
  PCSTR const Fmt,
  va_list const Args
) {

  if (Level >= DebugLevel) {

    char Str [128];

  	vsprintf (Str, Fmt, Args);

    LARGE_INTEGER Time;

    NdisGetCurrentSystemTime (&Time);

    UINT const MS = UINT (Time.QuadPart / 10000) % 100000;

    PCSTR const Type = (Level < 9)? " " : "***** ";

    if (Adapter && Adapter->MemoryWindowAddr) {

      DbgPrint (
        "sbni%c%05u %X:%s%s\n",
        Type [0],
        MS,
        Adapter->MemoryWindowAddr,
        Type,
        Str
      );

  	} else {

      DbgPrint ("sbni %05u:%s%s\n", MS, Type, Str);

    }


  }

}




void _cdecl DebugPrintf (UINT const Level, AdapterDesc const * const Adapter, PCSTR const Fmt, ...) {

  if (Level >= DebugLevel) {

    va_list Args;

    va_start (Args, Fmt);

  	DebugVPrintf (Level, Adapter, Fmt, Args);

  }

}





//
// Verify pointer validity
// =======================
//

bool _UniCall IsValidPtr (PCVOID Block, ULONG Size) {

  Assert (KeGetCurrentIrql () <= DISPATCH_LEVEL);

  return (
    Block
    && MmIsAddressValid (PVOID (Block))
    && MmIsAddressValid (PBYTE (Block) + Size - 1)
  );

}





#endif // _DEBUG
