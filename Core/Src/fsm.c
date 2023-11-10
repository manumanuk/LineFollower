#include "fsm.h"
#include "main.h"
#include "qtr8a.h"
#include "control.h"

#define SENSOR_CALIB_TIME 100U

bool transition_state(volatile robot_state_e *state, robot_event_e event) {
    switch (*state) {
        case IDLE: {
            switch(event) {
                case SW:
                    motor_command(0, 0);
                    motor_switch_directions(FORWARD);
                    qtr8a_power_on(FRONT);
                    *state = LFF;
                    return true;
                case SWLONG:
                    *state = CALIB;
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
                    motor_command(0, 0);
                    qtr8a_power_off(FRONT);
                    *state = IDLE;
                    return true;
                case BLUE_EVT:
                    motor_command(0, 0);
                    qtr8a_power_off(FRONT);
                    *state = GRPG;
                default:
                    return false;
            }
        }
        case LFR: {
            switch(event) {
                case SW:
                    motor_command(0, 0);
                    qtr8a_power_off(BACK);
                    *state = IDLE;
                    return true;
                case GREEN_EVT:
                    motor_command(0, 0);
                    qtr8a_power_off(BACK);
                    *state = GRPR;
                    return true;
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
                    motor_switch_directions(BACKWARD);
                    qtr8a_power_on(BACK);
                    *state = LFR;
                    return true;
                default:
                    return false;
            }
        }
        case GRPR: {
            switch(event) {
                case SW:
                    *state = IDLE;
                    motor_command(0, 0);
                    halt_gripper();
                    return true;
                case GRPR_CMPL:
                    motor_command(0, 0);
                    motor_switch_directions(BACKWARD);
                    qtr8a_power_on(BACK);
                    *state = LFR;
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

static void blink_ld2_n_times(uint8_t n) {
    for (int i=0; i<n; i++) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(1000);
    }
}

static void read_calibration_data(uint16_t *readings, double *levels, uint8_t numSensors) {
    uint32_t startTime = HAL_GetTick();
    uint32_t readingsTaken = 0;

    double tempLevels[MAX_IR_ARRAY_SENSORS];

    while(HAL_GetTick()-startTime < SENSOR_CALIB_TIME) {
        if (qtr8a_get_readings(readings, numSensors, IR_ARRAY_ADC_TIMEOUT)) {
            for (uint8_t i=0; i<numSensors; i++) {
                tempLevels[i] += readings[i];
            }
            readingsTaken++;
        }
    }

    for (uint8_t i=0; i<numSensors; i++) {
        if (levels[i] != 0)
            levels[i] = (levels[i] + tempLevels[i]/readingsTaken)/2;
        else
            levels[i] = tempLevels[i]/readingsTaken;
    }
}

static void calibrate_qtr8a_position_colour(qtr8a_instance_e position,
                                            line_colour_e colour,
                                            uint16_t *readings,
                                            double *levels,
                                            uint8_t numSensors,
                                            uint8_t blinkIndicator,
                                            uint8_t repetitions) {
    for (uint8_t i=0; i<repetitions; i++) {
        blink_ld2_n_times(blinkIndicator);
        wait_for_button_press();
        read_calibration_data(readings, levels, numSensors);
    }
}

void calibration_sequence() {
    uint16_t frontReadings[FRONT_IR_ARRAY_SENSORS] = {0U};
    uint16_t backReadings[BACK_IR_ARRAY_SENSORS] = {0U};

    double frontLevels[FRONT_IR_ARRAY_SENSORS] = {0.0F};
    double backLevels[BACK_IR_ARRAY_SENSORS] = {0.0F};

    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

    // Front red calibration
    calibrate_qtr8a_position_colour(FRONT, RED, frontReadings, frontLevels, FRONT_IR_ARRAY_SENSORS, 1, 3);
    qtr8a_set_levels(FRONT, RED, frontLevels);

    // Back red calibration
    calibrate_qtr8a_position_colour(BACK, RED, backReadings, backLevels, BACK_IR_ARRAY_SENSORS, 2, 3);
    qtr8a_set_levels(BACK, RED, backLevels);

    // Front brown calibration
    calibrate_qtr8a_position_colour(FRONT, BROWN, frontReadings, frontLevels, FRONT_IR_ARRAY_SENSORS, 1, 3);
    qtr8a_set_levels(FRONT, BROWN, frontLevels);

    // Back brown calibration
    calibrate_qtr8a_position_colour(BACK, BROWN, backReadings, backLevels, BACK_IR_ARRAY_SENSORS, 2, 3);
    qtr8a_set_levels(BACK, BROWN, backLevels);

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    wait_for_button_press();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
