/* sg17device.h:
 *
 * SDFE4 Library
 *
 *	Target device configuration
 *
 * Authors:
 *      Artem Polyakov <art@sigrand.ru>
 *      Ivan Neskorodev <ivan@sigrand.ru>
 */
       
#ifndef SG17_DEVICE_H
#define SG17_DEVICE_H

#include "sg17config.h"

#ifdef SG17_PCI_MODULE
#include "sg17_pci_module.h"
#else
#include "sg17_repeater.h"
#endif

#endif
