// some defines for compatibility
// why _PTR??
#define PULONG_PTR unsigned long *
#define ULONG_PTR unsigned long 
#define DWORD_PTR ULONG_PTR
// end of defines

#pragma warning (disable:\
  4115 /* named type definition in parentheses */ \
  4127 /* conditional expression is constant */ \
  4131 /* uses old-style declarator */ \
  4201 /* nonstandard extension used : nameless struct/union */ \
  4214 /* nonstandard extension used : bit field types other than int */ \
  4355 /* 'this' : used in base member initializer list */ \
  4514 /* unreferenced inline function has been removed */ \
)



#ifndef _DEBUG

  //#pragma warning (disable: 4711 /* function selected for automatic inline expansion */)

#endif



#define _X86_
#define STRICT
#define IS_32
//#define USE_KLOCKS 1            // Use kernel spin locks instead of ndis

#define NDIS50_MINIPORT
#define NDIS_MINIPORT_DRIVER

#ifdef _DEBUG

  #define DEBUG_LEVEL DEBUGLVL_BLAB

  #define DBG 1

#endif


