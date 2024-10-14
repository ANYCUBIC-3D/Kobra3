// 

// Support for gathering acceleration data from lis2dw12 chip
//
// Copyright (C) 2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "spicmds.h" // spidev_transfer

// Chip registers
#define AR_POWER_CTL   0x20
#define AD_OUT_X_L      0x28
#define AR_FIFO_STATUS 0x2F
#define AM_READ  0x80
#define AM_MULTI 0x40
#define AD_FIFO_CTRL 0x2E

#define SET_FIFO_CTL 0xC0

struct lis2dw12 {
    struct timer timer;
    uint32_t rest_ticks;
    struct spidev_s *spi;
    uint16_t sequence, limit_count;
    uint8_t flags, data_count;
    uint8_t data[50];
};

enum {
    AX_HAVE_START = 1<<0, AX_RUNNING = 1<<1, AX_PENDING = 1<<2,
};

static struct task_wake lis2dw12_wake;

// Event handler that wakes lis2dw12_task() periodically
static uint_fast8_t
lis2dw12_event(struct timer *timer)
{
    struct lis2dw12 *ax = container_of(timer, struct lis2dw12, timer);
    ax->flags |= AX_PENDING;
    sched_wake_task(&lis2dw12_wake);
    return SF_DONE;
}

void
command_config_lis2dw12(uint32_t *args)
{
    struct lis2dw12 *ax = oid_alloc(args[0], command_config_lis2dw12
                                   , sizeof(*ax));
    ax->timer.func = lis2dw12_event;
    ax->spi = spidev_oid_lookup(args[1]);
}
DECL_COMMAND(command_config_lis2dw12, "config_lis2dw12 oid=%c spi_oid=%c");

// Report local measurement buffer
static void
adxl_report(struct lis2dw12 *ax, uint8_t oid)
{
    sendf("lis2dw12_data oid=%c sequence=%hu data=%*s"
          , oid, ax->sequence, ax->data_count, ax->data);
    ax->data_count = 0;
    ax->sequence++;
}

// Report buffer and fifo status
static void
adxl_status(struct lis2dw12 *ax, uint_fast8_t oid
            , uint32_t time1, uint32_t time2, uint_fast8_t fifo)
{
    sendf("lis2dw12_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
          " buffered=%c fifo=%c limit_count=%hu"
          , oid, time1, time2-time1, ax->sequence
          , ax->data_count, fifo&0x1F, ax->limit_count);
}

// Helper code to reschedule the lis2dw12_event() timer
static void
adxl_reschedule_timer(struct lis2dw12 *ax)
{
    irq_disable();
    ax->timer.waketime = timer_read_time() + ax->rest_ticks;
    sched_add_timer(&ax->timer);
    irq_enable();
}


// Query accelerometer data
static void
adxl_query(struct lis2dw12 *ax, uint8_t oid)
{
    // Read data
    uint8_t msg[7] = { AD_OUT_X_L | AM_READ , 0, 0, 0, 0, 0, 0 };
    spidev_transfer(ax->spi, 1, sizeof(msg), msg);
    uint8_t fifo[3] = {AD_FIFO_CTRL | AM_READ,0,0};
    spidev_transfer(ax->spi, 1, sizeof(fifo), fifo);
    // sendf("fifo_data fifo=%*s",3,fifo);
    // Extract x, y, z measurements
    uint_fast8_t fifo_status = fifo[2] & ~0xE0; // Ignore trigger bit
    uint8_t *d = &ax->data[ax->data_count];
    if (((fifo[1] & 0xE0) != SET_FIFO_CTL)|| (fifo_status > 32)) {
        // Data error - may be a CS, MISO, MOSI, or SCLK glitch
        // d[0] = d[1] = d[2] = d[3] = d[4] = d[5]= 0xff;
        d[0] = d[1] = d[2] = d[3] = d[4] = d[5] = 0xff;
        fifo_status = 0;
    } else {
        // Copy data
        d[0] = msg[1]; // x low bits
        d[1] = msg[3]; // y low bits
        d[2] = msg[5]; // z low bits
        d[3] = msg[2]; // x high bits 
        d[4] = msg[4]; // y high bits
        d[5] = msg[6]; // z high bits
    }
    ax->data_count += 6;
    if ((ax->data_count + 6) > ARRAY_SIZE(ax->data)){
        adxl_report(ax, oid);
    }
        
    // Check fifo status
    if (fifo_status >= 31)
        ax->limit_count++;
    if (fifo_status > 1 && fifo_status <= 32) {
        // More data in fifo - wake this task again
        sched_wake_task(&lis2dw12_wake);
    } else if (ax->flags & AX_RUNNING) {
        // Sleep until next check time
        sched_del_timer(&ax->timer);
        ax->flags &= ~AX_PENDING;
        adxl_reschedule_timer(ax);
    }
}

// Startup measurements
static void
adxl_start(struct lis2dw12 *ax, uint8_t oid)
{
    sched_del_timer(&ax->timer);
    ax->flags = AX_RUNNING;
    // uint8_t msg[2] = { AR_POWER_CTL,  0x94 };
    // spidev_transfer(ax->spi, 0, sizeof(msg), msg);
    adxl_reschedule_timer(ax);
}

// End measurements
static void
adxl_stop(struct lis2dw12 *ax, uint8_t oid)
{
    // Disable measurements
    sched_del_timer(&ax->timer);
    ax->flags = 0;
    uint8_t msg[2] = { AR_FIFO_STATUS | AM_READ, 0x00 };
    // uint32_t end1_time = timer_read_time();
    // spidev_transfer(ax->spi, 0, sizeof(msg), msg);
    // uint32_t end2_time = timer_read_time();
    // Drain any measurements still in fifo
    uint_fast8_t i;
    for (i=0; i<33; i++) {
        // msg[0] = AR_FIFO_STATUS | AM_READ;
        // msg[1] = 0x00;
        spidev_transfer(ax->spi, 1, sizeof(msg), msg);
        uint_fast8_t fifo_status = msg[1] & ~0x80;
        if (!fifo_status)
            break;
        if (fifo_status <= 32)
            adxl_query(ax, oid);
    }
    uint8_t fifo = msg[1];
    msg[0] = AR_POWER_CTL;
    msg[1] = 0x00;
    uint32_t end1_time = timer_read_time();
    spidev_transfer(ax->spi, 0, sizeof(msg), msg);
    uint32_t end2_time = timer_read_time();
    // Report final data
    if (ax->data_count)
        adxl_report(ax, oid);
    adxl_status(ax, oid, end1_time, end2_time, fifo);
}

void
command_query_lis2dw12(uint32_t *args)
{
    struct lis2dw12 *ax = oid_lookup(args[0], command_config_lis2dw12);

    if (!args[2]) {
        // End measurements
        adxl_stop(ax, args[0]);
        return;
    }
    // Start new measurements query
    sched_del_timer(&ax->timer);
    ax->timer.waketime = args[1];
    ax->rest_ticks = args[2];
    ax->flags = AX_HAVE_START;
    ax->sequence = ax->limit_count = 0;
    ax->data_count = 0;
    sched_add_timer(&ax->timer);
}
DECL_COMMAND(command_query_lis2dw12,
             "query_lis2dw12 oid=%c clock=%u rest_ticks=%u");

void
command_query_lis2dw12_status(uint32_t *args)
{
    struct lis2dw12 *ax = oid_lookup(args[0], command_config_lis2dw12);
    uint8_t msg[2] = { AR_FIFO_STATUS | AM_READ, 0x00 };
    uint32_t time1 = timer_read_time();
    spidev_transfer(ax->spi, 1, sizeof(msg), msg);
    uint32_t time2 = timer_read_time();
    adxl_status(ax, args[0], time1, time2, msg[1]);
}
DECL_COMMAND(command_query_lis2dw12_status, "query_lis2dw12_status oid=%c");

void
lis2dw12_task(void)
{
    if (!sched_check_wake(&lis2dw12_wake))
        return;
    uint8_t oid;
    struct lis2dw12 *ax;
    foreach_oid(oid, ax, command_config_lis2dw12) {
        uint_fast8_t flags = ax->flags;
        if (!(flags & AX_PENDING))
            continue;
        if (flags & AX_HAVE_START)
            adxl_start(ax, oid);
        else
            adxl_query(ax, oid);
    }
}
DECL_TASK(lis2dw12_task);
