// Generic interrupt based serial uart helper code
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memmove
#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/io.h" // readb
#include "board/irq.h" // irq_save
#include "board/misc.h" // console_sendf
#include "board/pgm.h" // READP
#include "command.h" // DECL_CONSTANT
#include "sched.h" // sched_wake_tasks
#include "Debug_serial_irq.h" // serial_enable_tx_irq
#include <stdarg.h> // va_start
#include "stm32/printf.h"

#define RX_BUFFER_SIZE 192

static uint8_t receive_buf[RX_BUFFER_SIZE], receive_pos;
static uint8_t transmit_buf[1024], transmit_pos, transmit_max;

DECL_CONSTANT("SERIAL_BAUD", CONFIG_SERIAL_BAUD);
DECL_CONSTANT("RECEIVE_WINDOW", RX_BUFFER_SIZE);

// Rx interrupt - store read data
void
Debug_serial_rx_byte(uint_fast8_t data)
{
    if (data == MESSAGE_SYNC)
        sched_wake_tasks();
    if (receive_pos >= sizeof(receive_buf))
        // Serial overflow - ignore it as crc error will force retransmit
        return;
    receive_buf[receive_pos++] = data;
}

// Tx interrupt - get next byte to transmit
int
Debug_serial_get_tx_byte(uint8_t *pdata)
{
    if (transmit_pos >= transmit_max)
        return -1;
    *pdata = transmit_buf[transmit_pos++];
    return 0;
}

// Encode and transmit a "response" message
void
Debug_sendf(char *format, ...)
{
    uint8_t temp_buf[96];
     va_list args;
     va_start(args, format);
     memset(temp_buf, 0x00, sizeof( temp_buf));
     uint16_t len = vsnprintf(temp_buf, sizeof(temp_buf), format, args);
     va_end(args);

    // Verify space for message
    uint_fast8_t tpos = readb(&transmit_pos), tmax = readb(&transmit_max);
    if (tpos >= tmax) {
        tpos = tmax = 0;
        writeb(&transmit_max, 0);
        writeb(&transmit_pos, 0);
    }

    if (tmax + len > sizeof(transmit_buf)) {
        if (tmax + len - tpos > sizeof(transmit_buf))
            // Not enough space for message
            return;
        // Disable TX irq and move buffer
        writeb(&transmit_max, 0);
        tpos = readb(&transmit_pos);
        tmax -= tpos;
        memmove(&transmit_buf[0], &transmit_buf[tpos], tmax);
        writeb(&transmit_pos, 0);
        writeb(&transmit_max, tmax);
        Debug_serial_enable_tx_irq();
    }

    // Start message transmit
    memmove(&transmit_buf[tmax],temp_buf,len);
    writeb(&transmit_max, tmax + len);
    Debug_serial_enable_tx_irq();
}
