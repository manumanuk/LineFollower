#include "control.h"
#include "tcs3472.h"
#include "qtr8a.h"
#include "stm32f4xx_hal.h"
#include "main.h"

#define MOTOR_MAX_PWM 1000U
#define MOTOR_UNSTALL_TIME 50U
#define MOTOR_UNSTALL_SPEED 700U
#define MOTOR_RIGHT_BIAS 50U

#define BANG_BANG_SPEED 650U
#define BANG_BANG_POS_THRESH 300U

#define PID_BASE_SPEED 400U
#define PID_DELTA_V_RANGE (MOTOR_MAX_PWM-PID_BASE_SPEED)
#define PID_DESIRED_POS 410U
#define PID_K_P 2.4
#define PID_K_D 3.3
#define PID_K_I 0

extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

typedef struct {
    motor_dir_e direction;
    uint32_t lMotorPwm;
    uint32_t rMotorPwm;
} motor_state_t;

static motor_state_t state = {
    FORWARD,
    0,
    0
};

#define GRIPPER_GRIP_PWM 4U
#define GRIPPER_RELEASE_PWM 11U
#define GRIPPER_DELAY 3000U

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
        *rMotorPwm = PID_BASE_SPEED + controlVal - MOTOR_RIGHT_BIAS;
    } else {
        uint16_t controlVal = ((uint16_t)(-deltaV)) < PID_DELTA_V_RANGE ? ((uint16_t)(-deltaV)) : PID_DELTA_V_RANGE;
        *lMotorPwm = PID_BASE_SPEED + controlVal;
        *rMotorPwm = PID_BASE_SPEED - MOTOR_RIGHT_BIAS;
    }
    char buf[50] = {0};
    // uint16_t n = snprintf(buf, 50, "%f\r\n", deltaV);
    // uint16_t n = snprintf(buf, 50, "%i, %i\r\n", *lMotorPwm, *rMotorPwm);
    // HAL_UART_Transmit(&huart2, buf, n, HAL_MAX_DELAY);
    // HAL_Delay(100);
}

void motor_command(uint32_t lMotorPwm, uint32_t rMotorPwm) {        
    uint32_t lChannel = (state.direction == FORWARD) ? TIM_CHANNEL_1 : TIM_CHANNEL_4;
    uint32_t rChannel = (state.direction == FORWARD) ? TIM_CHANNEL_2 : TIM_CHANNEL_3;

    if (state.lMotorPwm == 0 && lMotorPwm != 0) {
        __HAL_TIM_SET_COMPARE(&htim1, lChannel, MOTOR_UNSTALL_SPEED);
        HAL_Delay(MOTOR_UNSTALL_TIME);
    }
    if (state.rMotorPwm == 0 && rMotorPwm != 0) {
        __HAL_TIM_SET_COMPARE(&htim1, rChannel, MOTOR_UNSTALL_SPEED);
        HAL_Delay(MOTOR_UNSTALL_TIME);
    }

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

void call_grpg_sequence() {
    gripper_grip();
}

void call_grpr_sequence() {
    motor_command(GRPR_SPEED, GRPR_SPEED);
    uint8_t greenCount = 0;
    uint16_t backQtr8aReadings[FRONT_IR_ARRAY_SENSORS];
    uint32_t lMotorPwm = 0U;
    uint32_t rMotorPwm = 0U;

    while (greenCount < GRPR_SENSE_THRESH) {
        if (check_green()) {
            greenCount++;
        }
        double position = 0.0;
        qtr8a_get_readings(BACK, backQtr8aReadings, BACK_IR_ARRAY_SENSORS, IR_ARRAY_ADC_TIMEOUT);
        position = get_position_from_readings(BACK, backQtr8aReadings, BACK_IR_ARRAY_SENSORS);
        ctrl_bang_bang_get_motor_cmd(position, &lMotorPwm, &rMotorPwm);
        motor_command(lMotorPwm, rMotorPwm);
    }

    uint32_t currTime = HAL_GetTick();
    while(currTime - HAL_GetTick() < GRPR_REVERSE_TIME) {
        double position = 0.0;
        qtr8a_get_readings(BACK, backQtr8aReadings, BACK_IR_ARRAY_SENSORS, IR_ARRAY_ADC_TIMEOUT);
        position = get_position_from_readings(BACK, backQtr8aReadings, BACK_IR_ARRAY_SENSORS);
        ctrl_bang_bang_get_motor_cmd(position, &lMotorPwm, &rMotorPwm);
        motor_command(lMotorPwm, rMotorPwm);
    }

    motor_command(0, 0);
    gripper_release();
}
