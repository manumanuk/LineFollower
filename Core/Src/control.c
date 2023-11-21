#include "control.h"
#include "tcs3472.h"
#include "qtr8a.h"
#include "stm32f4xx_hal.h"
#include "common.h"

uint32_t MOTOR_MAX_PWM = 600U;
#define MOTOR_UNSTALL_TIME 50U
#define MOTOR_UNSTALL_SPEED 850U
#define MOTOR_BALANCE_BIAS 0U

#define LF_MOTOR_CHANNEL TIM_CHANNEL_2
#define RF_MOTOR_CHANNEL TIM_CHANNEL_1
#define LB_MOTOR_CHANNEL TIM_CHANNEL_3
#define RB_MOTOR_CHANNEL TIM_CHANNEL_4

#define BANG_BANG_SPEED 650U
#define BANG_BANG_POS_THRESH 450U

uint32_t PID_BASE_SPEED = 425U;
#define PID_DELTA_V_RANGE (MOTOR_MAX_PWM-PID_BASE_SPEED)
uint32_t PID_DESIRED_POS = 480U;
float PID_K_P = 1.0;
float PID_K_D = 3.0;
float PID_K_I = 0;

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

float globalPosition = 0.0;

typedef struct {
    motor_dir_e direction;
    uint32_t lMotorPwm;
    uint32_t rMotorPwm;
} motor_state_t;

static motor_state_t state;

#define GRIPPER_GRIP_PWM 4U
#define GRIPPER_RELEASE_PWM 11U
#define GRIPPER_DELAY 1200U

#define GRPR_SENSE_THRESH 5U
#define GRPR_SPEED 50U
#define GRPR_REVERSE_TIME 1000U

void init_motors() {
	state.direction = FORWARD;
    state.lMotorPwm = 0;
    state.rMotorPwm = 0;

    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;
    htim1.Instance->CCR4 = 0;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void gripper_grip() {
	htim2.Instance->CCR1 = GRIPPER_RELEASE_PWM;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_Delay(GRIPPER_DELAY);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void gripper_release() {
	htim2.Instance->CCR1 = GRIPPER_GRIP_PWM;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_Delay(GRIPPER_DELAY);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void halt_gripper() {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
}


void ctrl_bang_bang_get_motor_cmd(double position, uint32_t *lMotorPwm, uint32_t *rMotorPwm) {
    if (position > BANG_BANG_POS_THRESH) {
        *lMotorPwm = BANG_BANG_SPEED;
        *rMotorPwm = 0;
    } else {
        *lMotorPwm = 0;
        *rMotorPwm = BANG_BANG_SPEED;
    }
}

void ctrl_pid_get_motor_cmd(double position, uint32_t *lMotorPwm, uint32_t *rMotorPwm) {
    static double errorPrior = 0.0;
    static double integralPrior = 0.0;
    // When robot is too much to the left, error will be negative. activate the left motor
    double error = PID_DESIRED_POS-position;
    double integral = integralPrior + error;
    double deltaV = error*PID_K_P + (error-errorPrior)*PID_K_D + (integral)*PID_K_I;
    integralPrior = integral;
    errorPrior = error;
    

    if (deltaV > 0) {
        *lMotorPwm = PID_BASE_SPEED;
        uint16_t controlVal = ((uint16_t)deltaV) < PID_DELTA_V_RANGE ? ((uint16_t)deltaV) : PID_DELTA_V_RANGE;
        *rMotorPwm = PID_BASE_SPEED + controlVal;
    } else {
        uint16_t controlVal = ((uint16_t)(-deltaV)) < PID_DELTA_V_RANGE ? ((uint16_t)(-deltaV)) : PID_DELTA_V_RANGE;
        *lMotorPwm = PID_BASE_SPEED + controlVal;
        *rMotorPwm = PID_BASE_SPEED;
    }
}

void motor_command(uint32_t lMotorPwm, uint32_t rMotorPwm) {        
    uint32_t lChannel = (state.direction == FORWARD) ? LF_MOTOR_CHANNEL : LB_MOTOR_CHANNEL;
    uint32_t rChannel = (state.direction == FORWARD) ? RF_MOTOR_CHANNEL : RB_MOTOR_CHANNEL;

    __HAL_TIM_SET_COMPARE(&htim1, lChannel, lMotorPwm);
    __HAL_TIM_SET_COMPARE(&htim1, rChannel, rMotorPwm);
    
    state.lMotorPwm = lMotorPwm;
    state.rMotorPwm = rMotorPwm;
    return;
}

void motor_switch_directions(motor_dir_e dir) {
	state.direction = dir;
    state.lMotorPwm = 0;
    state.rMotorPwm = 0;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

void call_lf_sequence() {
    uint16_t readings[MAX_IR_ARRAY_SENSORS];

    qtr8a_instance_e instance = (state.direction == FORWARD) ? FRONT : BACK;
    uint8_t numSensors = (state.direction == FORWARD) ? FRONT_IR_ARRAY_SENSORS : BACK_IR_ARRAY_SENSORS;

    if (!qtr8a_get_readings(instance, readings, numSensors, IR_ARRAY_ADC_TIMEOUT))
        return;
        
    double position = get_position_from_readings(instance, readings, numSensors);
    globalPosition = (float)position;

    uint32_t lMotorPwm = 0;
    uint32_t rMotorPwm = 0;

    ctrl_pid_get_motor_cmd(position, &lMotorPwm, &rMotorPwm);
    motor_command(lMotorPwm, rMotorPwm);
}

void call_grpg_sequence() {
    gripper_grip();
}

void call_grpr_sequence() {
    /*
    motor_command(GRPR_SPEED, GRPR_SPEED);
    uint8_t greenCount = 0;
    uint16_t backQtr8aReadings[FRONT_IR_ARRAY_SENSORS];
    uint32_t lMotorPwm = 0U;
    uint32_t rMotorPwm = 0U;

    while (greenCount < GRPR_SENSE_THRESH) {
        if (check_green()) {
            greenCount++;
        }
        call_lf_sequence();
    }

    uint32_t currTime = HAL_GetTick();
    while(currTime - HAL_GetTick() < GRPR_REVERSE_TIME) {
        call_lf_sequence();
    }
    */

    motor_command(0, 0);
    gripper_release();
}
