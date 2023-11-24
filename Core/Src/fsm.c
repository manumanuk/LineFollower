#include "fsm.h"
#include "common.h"
#include "control.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define SENSOR_CALIB_TIME 100U

static void blink_ld2_n_times(uint8_t n, uint32_t delay);
static void wait_for_button_press();

bool transition_state(volatile robot_state_e *state, robot_event_e event) {
    switch (*state) {
        case IDLE: {
            switch(event) {
                case SW:
                    motor_command(0, 0);
                    *state = LFF;
                    return true;
                default:
                    return false;
            }
        }
        case CALIB: {
            switch(event) {
                case CALIB_CMPL:
                    *state = IDLE;
                    return true;
                default:
                    return false;
            }
        }
        case LFF: {
            switch(event) {
                case SW:
                    *state = IDLE;
                    motor_command(0, 0);
                    return true;
                case BLUE_EVT:
                    motor_command(0, 0);
                    *state = GRPG;
                default:
                    return false;
            }
        }
        case GRPG: {
            switch(event) {
                case SW:
                    *state = IDLE;
                    motor_command(0, 0);
                    halt_gripper();
                    return true;
                case GRPG_CMPL:
                    motor_command(0, 0);
                    halt_gripper();
                    *state = LFF;
                    return true;
                default:
                    return false;
            }
        }
        default:
            return false;
    }
}

static void wait_for_button_press() {
    while (1) {
        while(HAL_GPIO_ReadPin(SW_PORT, SW_PIN));
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(SW_PORT, SW_PIN) == 0)
            return;
    }
}

static void blink_ld2_n_times(uint8_t n, uint32_t delay) {
    for (int i=0; i<n; i++) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(delay);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(delay);
    }
}

void calibration_sequence() {
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    wait_for_button_press();
    HAL_Delay(1000);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
