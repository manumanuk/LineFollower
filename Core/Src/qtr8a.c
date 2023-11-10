#include "qtr8a.h"
#include "main.h"

#define IR_ARRAY_DUTY_CYCLE 100U

extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern volatile bool adcReady;

uint16_t frontColourCalibratedLevels[2][FRONT_IR_ARRAY_SENSORS];
uint16_t backColourCalibratedLevels[2][BACK_IR_ARRAY_SENSORS];

void qtr8a_power_on(qtr8a_instance_e instance) {
    if (instance == FRONT) {
        htim3.Instance->CCR1 = IR_ARRAY_DUTY_CYCLE;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    } else if (instance == BACK) {
        htim3.Instance->CCR3 = IR_ARRAY_DUTY_CYCLE;
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
    if (instance == FRONT)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, dutyCycle);
    else
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, dutyCycle);
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

void qtr8a_set_levels(qtr8a_instance_e instance, line_colour_e colour, double *calibReadings) {
    uint16_t *levelsArr = (instance == FRONT) ? frontColourCalibratedLevels[colour] : backColourCalibratedLevels[colour];
    uint8_t size = (instance == FRONT) ? FRONT_IR_ARRAY_SENSORS : BACK_IR_ARRAY_SENSORS;

    for (int i=0; i<size; i++) {
        levelsArr[i] = ((uint16_t)(calibReadings[i]));
    }
}
