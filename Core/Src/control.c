#include "control.h"
#include "stm32f4xx_hal.h"

#define BANG_BANG_POS_THRESH 500

static uint8_t currentLeftPwm;
static uint8_t currentRightPwm;

extern TIM_HandleTypeDef htim4;

void init_motors() {
    currentLeftPwm = 0;
    currentRightPwm = 0;

    htim4.Instance->CCR1 = currentLeftPwm;
    htim4.Instance->CCR2 = currentRightPwm;

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

bool ctrl_bang_bang_get_motor_cmd(uint16_t *qtrReadings, uint8_t numReadings, uint8_t *lMotorPwm, uint8_t *rMotorPwm) {
    float position = 0;
    uint32_t multiplier = 100;
    for (uint32_t i=1; i<numReadings+1; i++) {
        position += multiplier*i*(((float)(qtrReadings[i]))/4096);
    }
    if (position > BANG_BANG_POS_THRESH) {
        *lMotorPwm = 100;
        *rMotorPwm = 50;
    } else {
        *lMotorPwm = 50;
        *rMotorPwm = 100;
    }
    return true;
}

void motor_command(uint8_t lMotorPwm, uint8_t rMotorPwm) {
    currentLeftPwm = lMotorPwm;
    currentRightPwm = rMotorPwm;

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, lMotorPwm);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, rMotorPwm);
    
    return;
}
