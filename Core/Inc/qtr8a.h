#ifndef QTR8A_H_
#define QTR8A_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    FRONT = 0,
    BACK
} qtr8a_instance_e;

void qtr8a_power_on(qtr8a_instance_e instance, uint8_t dutyCycle);
void qtr8a_power_off(qtr8a_instance_e instance);
void qtr8a_change_duty_cycle(qtr8a_instance_e instance, uint8_t dutyCycle);
bool qtr8a_get_readings(uint16_t *dataArr, uint8_t size, uint32_t timeout);

#endif /* QTR8A_H_*/
