/*
 * This source has been formatted by an unregistered SourceFormatXIf you want
 * to remove this info, please register this sharewarePlease visit
 * http://www.textrush.com to get more information
 */
#include "prefix.h"
extern "C"
{
#include <ndis.h>
}

#include <stddef.h>
#include <stdlib.h>
#include <minmax.h>
#if !USE_KLOCKS
#define PASSIVE_LEVEL	0
#define DISPATCH_LEVEL	2
#endif
extern "C"
{
ULONG	KeGetCurrentProcessorNumber( VOID );
NTHALAPI KIRQL	KeGetCurrentIrql( VOID );
}

#include "params.h"
#include "global.h"
#include "api.h"
#include "debug.h"
#include "cx28975.h"
#include "hdlc.h"
#include "utils.h"
#include "sg16pci.h"
