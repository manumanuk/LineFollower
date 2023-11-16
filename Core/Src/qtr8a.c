#include "qtr8a.h"
#include "main.h"

#define IR_ARRAY_DUTY_CYCLE 90U
#if DEBUG_MODE
#define NUM_IR_READINGS 14U
#else
#define NUM_IR_READINGS 16U
#endif

extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern volatile bool adcReady;
extern UART_HandleTypeDef huart2;

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

bool qtr8a_get_readings(qtr8a_instance_e instance, uint16_t *dataArr, uint8_t size, uint32_t timeout) {
    if (dataArr == NULL) {
        return false;
    }
    uint32_t count = 0;
    uint16_t readings[NUM_IR_READINGS] = {0};
    adcReady = false;
    HAL_ADC_Start_DMA(&hadc1, readings, NUM_IR_READINGS);

    while (!adcReady && count < timeout) {
      HAL_Delay(1);
      count++;
    }

    if (count == timeout) {
        return false;
    }

    if (instance == FRONT) {
        for(int i=0; i<size; i++) {
            dataArr[i] = readings[i];
        }
    } else {
        for(int i=0; i<size; i++) {
            dataArr[i] = readings[i+FRONT_IR_ARRAY_SENSORS];
        }
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

#define min(a, b) (((a) < (b)) ? (a) : (b))

double get_position_from_readings(qtr8a_instance_e instance, uint16_t *qtrReadings, uint16_t numReadings) {
    double position = 0;
    uint16_t *redCalib = (instance == FRONT) ? frontColourCalibratedLevels[0] : backColourCalibratedLevels[0];
    uint16_t *brownCalib = (instance == FRONT) ? frontColourCalibratedLevels[1] : backColourCalibratedLevels[1];

    int16_t minVal = 1000;
    for (uint8_t i=0; i<numReadings; i++) {
    	minVal = min(minVal, (int16_t)qtrReadings[i]-brownCalib[i]);
    }

    uint32_t multiplier = 100;
    double sumOfWeights = 0.0;

    for (uint32_t i=0; i<numReadings; i++) {
    	double range = redCalib[i]-brownCalib[i];
    	double weightedValue = (-minVal)+1+qtrReadings[i]-brownCalib[i];
    	weightedValue = weightedValue/range;
    	sumOfWeights += weightedValue;
        position += multiplier*(i+1)*weightedValue;
    }

    position /= sumOfWeights;
    /*
    char buf[100] = {0};
    uint16_t n = snprintf(buf, 100, "%f\r\n", position);
    HAL_UART_Transmit(&huart2, buf, n, HAL_MAX_DELAY);
    HAL_Delay(100);
    */

    return position;
}
