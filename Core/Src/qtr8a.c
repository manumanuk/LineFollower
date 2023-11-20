#include "common.h"
#include "qtr8a.h"

// Red/brown: 50%
// Black/brown: 100%
#define IR_ARRAY_DUTY_CYCLE 100U
#if DEBUG_MODE
#define NUM_IR_READINGS 14U
#else
#define NUM_IR_READINGS 16U
#endif

extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern volatile bool adcReady;

static uint16_t frontColourCalibratedLevels[2][FRONT_IR_ARRAY_SENSORS];
static uint16_t backColourCalibratedLevels[2][BACK_IR_ARRAY_SENSORS];

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

float prevReadings[NUM_IR_READINGS] = {0U};
float alpha = 0;
bool qtr8a_get_readings(qtr8a_instance_e instance, uint16_t *dataArr, uint8_t size, uint32_t timeout) {
    if (dataArr == NULL) {
        return false;
    }
    uint32_t count = 0;
    uint16_t readings[NUM_IR_READINGS] = {0U};
    adcReady = false;
    HAL_ADC_Start_DMA(&hadc1, readings, NUM_IR_READINGS);

    while (!adcReady && count < timeout) {
      HAL_Delay(1);
      count++;
    }

    if (count == timeout) {
        return false;
    }
    
    for (uint8_t i=0; i<NUM_IR_READINGS; i++) {
        prevReadings[i] = alpha*prevReadings[i]+(1-alpha)*((float)readings[i]);
        readings[i] = (uint16_t)prevReadings[i];
    }

    if (instance == FRONT) {
        for(uint8_t i=0; i<size; i++) {
            dataArr[i] = readings[i];
        }
    } else {
        for(uint8_t i=0; i<size; i++) {
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

double get_position_from_readings(qtr8a_instance_e instance, uint16_t *qtrReadings, uint16_t numReadings) {
    double position = 0;
    uint16_t *redCalib = (instance == FRONT) ? frontColourCalibratedLevels[RED] : backColourCalibratedLevels[RED];
    uint16_t *brownCalib = (instance == FRONT) ? frontColourCalibratedLevels[BROWN] : backColourCalibratedLevels[BROWN];

    uint32_t multiplier = 100;
    double sumOfWeights = 0.0;
    
    uint32_t i=0;
    
    for (; i<numReadings; i++) {
    	double range = redCalib[i]-brownCalib[i];
    	double weightedValue = max(0, qtrReadings[i]-brownCalib[i]);
    	weightedValue = weightedValue/range;

    	sumOfWeights += weightedValue;
        position += multiplier*(i+1)*weightedValue;
    }
    sumOfWeights += 0.0001;

    position /= sumOfWeights;

    return position;
}
