#ifndef __GENERIC_ARMCM_TIMER_H
#define __GENERIC_ARMCM_TIMER_H

#include <stdint.h> // uint32_t

void udelay(uint32_t usecs);
void timer_pause(void);
void timer_resume(void);
#endif // armcm_timer.h
