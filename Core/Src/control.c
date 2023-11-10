#include "control.h"
#include "stm32f4xx_hal.h"

#define BANG_BANG_POS_THRESH 1050
#define BANG_BANG_SPEED 70

static uint8_t currentLeftPwm;
static uint8_t currentRightPwm;

extern TIM_HandleTypeDef htim1;

static motor_dir_e direction;

void init_motors() {
	direction = FORWARD;
    currentLeftPwm = 0;
    currentRightPwm = 0;

    htim1.Instance->CCR1 = currentLeftPwm;
    htim1.Instance->CCR2 = currentRightPwm;
    htim1.Instance->CCR3 = 0;
    htim1.Instance->CCR4 = 0;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void ctrl_bang_bang_get_motor_cmd(uint16_t *qtrReadings, uint8_t numReadings, uint8_t *lMotorPwm, uint8_t *rMotorPwm) {
    float position = 0;
    uint32_t multiplier = 100;
    
    for (uint32_t i=1; i<numReadings+1; i++) {
        position += multiplier*i*(((float)(qtrReadings[i]))/4096.0);
    }

    if (position > BANG_BANG_POS_THRESH) {
        *lMotorPwm = BANG_BANG_SPEED;
        *rMotorPwm = 0;
    } else {
        *lMotorPwm = 0;
        *rMotorPwm = BANG_BANG_SPEED;
    }
}

void motor_command(uint8_t lMotorPwm, uint8_t rMotorPwm) {
    currentLeftPwm = lMotorPwm;
    currentRightPwm = rMotorPwm;
    
    uint32_t lChannel = (direction == FORWARD) ? TIM_CHANNEL_1 : TIM_CHANNEL_3;
    uint32_t rChannel = (direction == FORWARD) ? TIM_CHANNEL_2 : TIM_CHANNEL_4;
    __HAL_TIM_SET_COMPARE(&htim1, lChannel, lMotorPwm);
    __HAL_TIM_SET_COMPARE(&htim1, rChannel, rMotorPwm);
    
    return;
}

void motor_switch_directions(motor_dir_e dir) {
	direction = dir;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}
