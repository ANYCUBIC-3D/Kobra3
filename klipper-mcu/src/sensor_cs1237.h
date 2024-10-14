#ifndef __SENSOR_CS1237_H
#define __SENSOR_CS1237_H

struct cs1237_sensor {
    struct timer timer;
    uint8_t speed;           // 00:10hz 01:40hz 10:64hz 11:1280hz 
    uint32_t sensitivity;   //
    uint8_t check_enable;
    int32_t adc32;
    int32_t avr_adc32;
    int32_t adc_diff;
    uint8_t homing_flag;
    int32_t homing_adc;
    uint32_t report_tick;
};

#define REG32(addr)                  (*(volatile uint32_t *)(uint32_t)(addr))
#define BIT(x)                       ((uint32_t)((uint32_t)0x01U<<(x)))
/* registers definitions */
#define EXTI_INTEN                   REG32(EXTI + 0x00U)      /*!< interrupt enable register */
#define EXTI_EVEN                    REG32(EXTI + 0x04U)      /*!< event enable register */
#define EXTI_RTEN                    REG32(EXTI + 0x08U)      /*!< rising edge trigger enable register */
#define EXTI_FTEN                    REG32(EXTI + 0x0CU)      /*!< falling trigger enable register */
#define EXTI_SWIEV                   REG32(EXTI + 0x10U)      /*!< software interrupt event register */
#define EXTI_PD                      REG32(EXTI + 0x14U)      /*!< pending register */


#endif