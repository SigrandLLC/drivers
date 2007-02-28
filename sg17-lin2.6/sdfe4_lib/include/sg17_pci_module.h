/* sg17_pci_module.h:
 * 
 * SDFE4 Library
 *
 *	Architecture dependent interface to device
 *	specific functions
 *
 * Authors: 
 *	Artem Polyakov <art@sigrand.ru>
 *	Ivan Neskorodev <ivan@sigrand.ru>
 */
#ifndef SIGRAND_PCI_MODULE_H
#define SIGRAND_PCI_MODULE_H
#include "sg17hw.h"

int sdfe4_hdlc_xmit(u8 *msg,u16 len,struct sdfe4 *hwdev);
int sdfe4_hdlc_wait_intr(int to,struct sdfe4 *hwdev);
int sdfe4_hdlc_recv(u8 *buf,int *len,struct sdfe4 *hwdev);
void sdfe4_clear_channel(struct sdfe4 *hwdev);
void wait_ms(int x);
int sdfe4_hdlc_regs(struct sdfe4 *hwdev);
int sdfe4_link_led_up(int i,struct sdfe4 *hwdev);
int sdfe4_link_led_down(int i,struct sdfe4 *hwdev);
int sdfe4_link_led_blink(int i, struct sdfe4 *hwdev);
int sdfe4_link_led_fast_blink(int i,struct sdfe4 *hwdev);

#endif
