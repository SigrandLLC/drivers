#pragma warning \
		( \
			disable : 4115 /* named type definition in parentheses */ 4127	/* conditional
																			 * expression
																			 * is
																			 * constant
																			 * */ \
				4131 /* uses old-style declarator */ 4201					/* nonstandard
																			 * extension
																			 * used
																			 * :
																			 * nameless
																			 * struct/union
																			 * */ \
					4214			/* nonstandard extension used : bit field
									 * types other than int */ \
						4355		/* 'this' : used in base member initializer
									 * list */ \
							4514	/* unreferenced inline function has been
									 * removed */ \
		)
#ifdef DBG
#undef NDEBUG
#ifndef _DEBUG
#define _DEBUG
#endif
#else
#ifndef NDEBUG
#define NDEBUG
#endif
#endif
#ifndef _X86_
#define _X86_
#endif
#ifndef STRICT
#define STRICT
#endif
#ifndef IS_32
#define IS_32
#endif

/* define USE_KLOCKS 1 // Use kernel spin locks instead of ndis */
#define NDIS50_MINIPORT
#define NDIS_MINIPORT_DRIVER
#ifdef _DEBUG
#define DEBUG_LEVEL DEBUGLVL_BLAB
#else

/*
 * pragma warning (disable: 4711 /* function selected for automatic inline
 * expansion
 */
#endif
