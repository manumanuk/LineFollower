#include "control.h"
#include "tcs3472.h"
#include "stm32f4xx_hal.h"
#include "common.h"

uint32_t MOTOR_MAX_PWM = 600U;
#define MOTOR_UNSTALL_TIME 50U
#define MOTOR_UNSTALL_SPEED 850U
#define MOTOR_BALANCE_BIAS 0U

#define LF_MOTOR_CHANNEL TIM_CHANNEL_1
#define LB_MOTOR_CHANNEL TIM_CHANNEL_3

#define RF_MOTOR_CHANNEL TIM_CHANNEL_1
#define RB_MOTOR_CHANNEL TIM_CHANNEL_2

#define BANG_BANG_SPEED 650U
#define BANG_BANG_POS_THRESH 450U

uint32_t PID_BASE_SPEED = 425U;
#define PID_DELTA_V_RANGE (MOTOR_MAX_PWM-PID_BASE_SPEED)
uint32_t PID_DESIRED_POS = 480U;
float PID_K_P = 1.0;
float PID_K_D = 3.0;
float PID_K_I = 0;

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim2;

float globalPosition = 0.0;

typedef struct {
    int32_t lMotorPwm;
    int32_t rMotorPwm;
} motor_state_t;

static motor_state_t state;

#define GRIPPER_GRIP_PWM 4U
#define GRIPPER_RELEASE_PWM 11U
#define GRIPPER_DELAY 1200U

void init_motors() {
    state.lMotorPwm = 0;
    state.rMotorPwm = 0;

    htim3.Instance->CCR1 = 0;
    htim3.Instance->CCR3 = 0;
    htim4.Instance->CCR1 = 0;
    htim4.Instance->CCR2 = 0;

    HAL_TIM_PWM_Start(&htim3, LF_MOTOR_CHANNEL);
    HAL_TIM_PWM_Start(&htim3, LB_MOTOR_CHANNEL);
    HAL_TIM_PWM_Start(&htim4, RF_MOTOR_CHANNEL);
    HAL_TIM_PWM_Start(&htim4, RB_MOTOR_CHANNEL);
}

void motor_command(int32_t lMotorPwm, int32_t rMotorPwm) {
    if (lMotorPwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, LF_MOTOR_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim3, LB_MOTOR_CHANNEL, -lMotorPwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, LF_MOTOR_CHANNEL, lMotorPwm);
        __HAL_TIM_SET_COMPARE(&htim3, LB_MOTOR_CHANNEL, 0);
    }

    if (rMotorPwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim4, RF_MOTOR_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim4, RB_MOTOR_CHANNEL, -rMotorPwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim4, RF_MOTOR_CHANNEL, rMotorPwm);
        __HAL_TIM_SET_COMPARE(&htim4, RB_MOTOR_CHANNEL, 0);
    }
    
    state.lMotorPwm = lMotorPwm;
    state.rMotorPwm = rMotorPwm;
    return;
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

void ctrl_bang_bang_get_motor_cmd(double position, int32_t *lMotorPwm, int32_t *rMotorPwm) {
    if (position > BANG_BANG_POS_THRESH) {
        *lMotorPwm = BANG_BANG_SPEED;
        *rMotorPwm = 0;
    } else {
        *lMotorPwm = 0;
        *rMotorPwm = BANG_BANG_SPEED;
    }
}

void ctrl_pid_get_motor_cmd(double position, int32_t *lMotorPwm, int32_t *rMotorPwm) {
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

void call_lf_sequence() {
    /*
    double position = get_position_from_readings(instance, readings, numSensors);
    globalPosition = (float)position;

    int32_t lMotorPwm = 0;
    int32_t rMotorPwm = 0;

    ctrl_pid_get_motor_cmd(position, &lMotorPwm, &rMotorPwm);
    motor_command(lMotorPwm, rMotorPwm);
    */
}

void call_grpg_sequence() {
    gripper_grip();
}
