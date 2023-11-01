#include "qtr8a.h"

extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern volatile bool adcReady;

void qtr8a_power_on(qtr8a_instance_e instance, uint8_t dutyCycle) {
    if (dutyCycle > 100)
        return;

    if (instance == FRONT) {
        htim3.Instance->CCR1 = dutyCycle;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    } else if (instance == BACK) {
        htim3.Instance->CCR3 = dutyCycle;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    }
}

void qtr8a_power_off(qtr8a_instance_e instance) {
    if (instance == FRONT) {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    } else if (instance == BACK) {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
    }
}

void qtr8a_change_duty_cycle(qtr8a_instance_e instance, uint8_t dutyCycle) {
    qtr8a_power_off(instance);
    qtr8a_power_on(instance, dutyCycle);
}

bool qtr8a_get_readings(uint16_t *dataArr, uint8_t size, uint32_t timeout) {
    if (dataArr == NULL) {
        return false;
    }
    uint32_t count = 0;
    HAL_ADC_Start_DMA(&hadc1, dataArr, size);
    while (!adcReady && count < timeout) {
      HAL_Delay(1);
      count++;
    }

    if (count == timeout) {
        return false;
    }

    adcReady = false;
    return true;
}
