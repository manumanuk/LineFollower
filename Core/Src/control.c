#include "control.h"
#include "tcs3472.h"
#include "qtr8a.h"
#include "stm32f4xx_hal.h"
#include "main.h"

#define BANG_BANG_POS_THRESH 350U

extern uint16_t frontColourCalibratedLevels[2][FRONT_IR_ARRAY_SENSORS];

extern UART_HandleTypeDef huart2;

static uint8_t currentLeftPwm;
static uint8_t currentRightPwm;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

static motor_dir_e direction;

#define GRIPPER_GRIP_PWM 4U
#define GRIPPER_RELEASE_PWM 11U
#define GRIPPER_DELAY 3000U

#define GRPR_SENSE_THRESH 5U
#define GRPR_SPEED 50U
#define GRPR_REVERSE_TIME 1000U

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

void gripper_grip() {
	htim2.Instance->CCR1 = GRIPPER_GRIP_PWM;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_Delay(GRIPPER_DELAY);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void gripper_release() {
	htim2.Instance->CCR1 = GRIPPER_RELEASE_PWM;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_Delay(GRIPPER_DELAY);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void halt_gripper() {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
}

#define min(a, b) (((a) < (b)) ? (a) : (b))

void ctrl_bang_bang_get_motor_cmd(uint16_t *qtrReadings, uint8_t numReadings, uint8_t *lMotorPwm, uint8_t *rMotorPwm, uint8_t speed) {
    int16_t minVal = 1000;
    for (uint8_t i=0; i<numReadings; i++) {
    	minVal = min(minVal, (int16_t)qtrReadings[i]-frontColourCalibratedLevels[1][i]);
    }

    double position = 0;
    uint32_t multiplier = 100;
    double sumOfWeights = 0.0;

    for (uint32_t i=0; i<numReadings; i++) {
    	double range = frontColourCalibratedLevels[0][i]-frontColourCalibratedLevels[1][i];
    	double weightedValue = (-minVal)+1+qtrReadings[i]-frontColourCalibratedLevels[1][i];
    	weightedValue = weightedValue/range;
    	sumOfWeights += weightedValue;
        position += multiplier*(i+1)*weightedValue;
    }

    position /= sumOfWeights;

    if (position > BANG_BANG_POS_THRESH) {
        *lMotorPwm = speed;
        *rMotorPwm = 0;
    } else {
        *lMotorPwm = 0;
        *rMotorPwm = speed;
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

void call_grpg_sequence() {
    gripper_grip();
}

void call_grpr_sequence() {
    motor_command(GRPR_SPEED, GRPR_SPEED);
    uint8_t greenCount = 0;
    uint16_t frontQtr8aReadings[FRONT_IR_ARRAY_SENSORS];
    uint16_t backQtr8aReadings[FRONT_IR_ARRAY_SENSORS];
    uint8_t lMotorPwm = 0U;
    uint8_t rMotorPwm = 0U;

    while (greenCount < GRPR_SENSE_THRESH) {
        if (check_green()) {
            greenCount++;
        }
        qtr8a_get_readings(BACK, backQtr8aReadings, BACK_IR_ARRAY_SENSORS, IR_ARRAY_ADC_TIMEOUT);
        ctrl_bang_bang_get_motor_cmd(backQtr8aReadings, BACK_IR_ARRAY_SENSORS, &lMotorPwm, &rMotorPwm, GRPR_SPEED);
        motor_command(lMotorPwm, rMotorPwm);
    }

    uint32_t currTime = HAL_GetTick();
    while(currTime - HAL_GetTick() < GRPR_REVERSE_TIME) {
        qtr8a_get_readings(BACK, backQtr8aReadings, BACK_IR_ARRAY_SENSORS, IR_ARRAY_ADC_TIMEOUT);
        ctrl_bang_bang_get_motor_cmd(backQtr8aReadings, BACK_IR_ARRAY_SENSORS, &lMotorPwm, &rMotorPwm, GRPR_SPEED);
        motor_command(lMotorPwm, rMotorPwm);
    }

    motor_command(0, 0);
    gripper_release();
}
