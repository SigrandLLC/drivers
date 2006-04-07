#ifdef _DEBUG
#define Assert( Condition ) \
	if( !(Condition) ) \
	{ \
		Break(); \
	}

/*
 * #define Assert(Condition) \ if (!(Condition)) { \ if (KdDebuggerEnabled) {
 * \ Break (); \ } else { \ Debug (9, NULL, "%s (%s, %u)", #Condition,
 * __FILE__, __LINE__); \ } \ } \#include <excpt.h> #define Assert(Condition)
 * \ if (!(Condition)) { \ __try { \ Break (); \ } __except (GetExceptionCode
 * () == EXCEPTION_BREAKPOINT) { \ Debug (9, NULL, "%s (%s, %u)", #Condition,
 * __FILE__, __LINE__); \ } \ } \
 */
#define Verify				Assert
#define Assume( Condition ) extern char _AssumeDummyVariable[!!( Condition )]
extern "C"
{
extern BOOLEAN const	KdDebuggerEnabled;
BOOLEAN MmIsAddressValid( IN PVOID VirtualAddress );
}

#define Debug	DebugPrintf
class	AdapterDesc;
void _cdecl DebugPrintf( UINT const Level, AdapterDesc const *const Adapter,
						 PCSTR const Fmt, ... );
bool _UniCall	IsValidPtr( PCVOID Block, ULONG Size=1 );
#else
#define Assert( Condition )
#define Verify( x ) x
#define Assume( Condition )
#define Debug( x )
#pragma warning( disable : 4002 /* too many actual parameters for macro */ )
#endif
#define AssertIf( Cond1, Cond2 )	Assert( !(Cond1) || (Cond2) )
#define DRIVER_VERSION				( (NDIS_MAJOR_VERSION << 8) + NDIS_MINOR_VERSION )
