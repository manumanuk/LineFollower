#include "control.h"
#include "tcs3472.h"
#include "stm32f4xx_hal.h"
#include "common.h"

#define MOTOR_MAX_PWM 1000U
#define MOTOR_UNSTALL_TIME 50U
#define MOTOR_UNSTALL_SPEED 850U
#define MOTOR_BALANCE_BIAS 0U

#define LF_MOTOR_CHANNEL TIM_CHANNEL_3
#define LB_MOTOR_CHANNEL TIM_CHANNEL_1

#define RF_MOTOR_CHANNEL TIM_CHANNEL_2
#define RB_MOTOR_CHANNEL TIM_CHANNEL_1

#define BANG_BANG_OUTER_MOTOR_SPEED 400
#define BANG_BANG_INNER_MOTOR_SPEED 200
#define BANG_BANG_POS_THRESH 100U

#define GRIPPER_GRIP_PWM 4U
#define GRIPPER_RELEASE_PWM 11U
#define GRIPPER_DELAY 1200U

#define PID_BASE_SPEED 350
#define PID_MIN_SPEED -350
#define PID_DESIRED_POS 100U
float PID_K_P = 3.5;
float PID_K_D = 6.0;
float PID_K_I = 0.0;

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

static void ctrl_bang_bang_get_motor_cmd(float position, int32_t *lMotorPwm, int32_t *rMotorPwm);
static void ctrl_pid_get_motor_cmd(float position, int32_t *lMotorPwm, int32_t *rMotorPwm);
static float get_position_from_readings(float *lRGB, float *rRGB);

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


static void ctrl_bang_bang_get_motor_cmd(float position, int32_t *lMotorPwm, int32_t *rMotorPwm) {
    if (position > 125) {
        *lMotorPwm = BANG_BANG_OUTER_MOTOR_SPEED;
        *rMotorPwm = BANG_BANG_INNER_MOTOR_SPEED;
    } else if (position < 75) {
        *lMotorPwm = BANG_BANG_INNER_MOTOR_SPEED;
        *rMotorPwm = BANG_BANG_OUTER_MOTOR_SPEED;
    } else {
        *lMotorPwm = 300;
        *rMotorPwm = 300;
    }
}

static void ctrl_pid_get_motor_cmd(float position, int32_t *lMotorPwm, int32_t *rMotorPwm) {
    static float errorPrior = 0.0;
    static float integralPrior = 0.0;
    // When robot is too much to the left, error will be negative. activate the left motor
    float error = PID_DESIRED_POS-position;
    float integral = integralPrior + error;
    float deltaV = error*PID_K_P + (error-errorPrior)*PID_K_D + integral*PID_K_I;
    integralPrior = integral;
    errorPrior = error;
    

    if (deltaV > 0) {
        // Slow down left motor
        *rMotorPwm = PID_BASE_SPEED;
        int32_t controlValue = PID_BASE_SPEED-deltaV;
        *lMotorPwm = max(PID_MIN_SPEED, controlValue);
    } else {
        // Slow down right motor
        *lMotorPwm = PID_BASE_SPEED;
        int32_t controlValue = PID_BASE_SPEED+deltaV;
        *rMotorPwm = max(PID_MIN_SPEED, controlValue);
    }
}

static float get_position_from_readings(float *lRGB, float *rRGB) {
    float lRed = lRGB[0];
    float rRed = rRGB[0];
    
    return 100.0 + 100.0*((rRed-RIGHT_BROWN_R)/(RIGHT_RED_R-RIGHT_BROWN_R) - (lRed-LEFT_BROWN_R)/(LEFT_RED_R-LEFT_BROWN_R));
}

void call_lf_sequence() {
    float rgbLeft[3];
    float rgbRight[3];
    tcs3472_get_colour_data(LEFT_COLOUR_SENSOR, rgbLeft);
    tcs3472_get_colour_data(RIGHT_COLOUR_SENSOR, rgbRight);

    int32_t lMotorPwm = 0;
    int32_t rMotorPwm = 0;
    float position = get_position_from_readings(rgbLeft, rgbRight);
    globalPosition = position;
    ctrl_pid_get_motor_cmd(position, &lMotorPwm, &rMotorPwm);
    motor_command(lMotorPwm, rMotorPwm);
}

void call_grpg_sequence() {
    gripper_grip();
}
