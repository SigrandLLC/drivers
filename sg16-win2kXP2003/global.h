#define UNUSED (void)

#define Break() _asm { int 3 }

#define SetBits(v,m) (v |= m)
#define ResetBits(v,m) (v &= ~(m))
#define TestBits(e,m) ((e) & (m))




#define NumArrayElems(a) (sizeof (a) / sizeof (*(a)))




#ifdef _DEBUG

  #define _UniCall _stdcall

  #define IfDebug(x) x

  // Define Zero, False and True as variables to prevent C4127 warning

  static int Zero = 0, False = 0, True = 1;

#else

  #define _UniCall _fastcall

  #define IfDebug(x)

  // These warnings are enabled only during debug

  #pragma warning (disable: \
    4127 /* conditional expression is constant */ \
    4727 /* conditional expression is constant */ \
  )

  #define Zero 0
  #define False false
  #define True true

#endif




typedef UCHAR BYTE;
typedef USHORT WORD;
typedef ULONG DWORD;

typedef BYTE *PBYTE;
typedef BYTE const *PCBYTE;
typedef WORD *PWORD;
typedef WORD const *PCWORD;
typedef DWORD *PDWORD;
typedef DWORD const *PCDWORD;

typedef void const *PCVOID;
