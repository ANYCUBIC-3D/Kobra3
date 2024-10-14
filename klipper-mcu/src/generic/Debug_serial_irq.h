#ifndef __GENERIC_DEBUG_SERIAL_IRQ_H
#define __GENERIC_DEBUG_SERIAL_IRQ_H

#include <stdint.h> // uint32_t

// callback provided by board specific code
void Debug_serial_enable_tx_irq(void);

// serial_irq.c
void Debug_serial_rx_byte(uint_fast8_t data);
int Debug_serial_get_tx_byte(uint8_t *pdata);
void Debug_sendf(char *format, ...);
#endif // serial_irq.h
