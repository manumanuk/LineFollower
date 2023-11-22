#include "fsm.h"
#include "common.h"
#include "qtr8a.h"
#include "control.h"
#include <string.h>

#define SENSOR_CALIB_TIME 100U

static void blink_ld2_n_times(uint8_t n, uint32_t delay);
static void wait_for_button_press();
static void calibrate_qtr8a_position_colour(qtr8a_instance_e position,
                                            double *levels,
                                            uint8_t numSensors,
                                            uint8_t blinkIndicator,
                                            uint8_t repetitions);

bool transition_state(volatile robot_state_e *state, robot_event_e event) {
    switch (*state) {
        case IDLE: {
            switch(event) {
                case SWLONG:
                    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
                    motor_command(0, 0);
                    motor_switch_directions(FORWARD);
                    qtr8a_power_on(FRONT);
                    *state = LFF;
                    return true;
                case SW:
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
                    qtr8a_power_off(FRONT);
                    *state = IDLE;
                    motor_command(0, 0);
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

static void blink_ld2_n_times(uint8_t n, uint32_t delay) {
    for (int i=0; i<n; i++) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(delay);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(delay);
    }
}

static void collect_calibration_statistics(qtr8a_instance_e instance, double *avg, double *stdev, uint8_t numSensors) {
    uint16_t readings[MAX_IR_ARRAY_SENSORS] = {0U};

    uint32_t startTime = HAL_GetTick();
    uint32_t readingsTaken = 0;

    double m2[MAX_IR_ARRAY_SENSORS] = {0U};
    while (HAL_GetTick()-startTime < SENSOR_CALIB_TIME) {
        if (!qtr8a_get_readings(instance, readings, numSensors, IR_ARRAY_ADC_TIMEOUT))
            continue;

        for (uint8_t i=0; i<numSensors; i++) {
            if (readingsTaken == 0) {
                avg[i] = readings[i];
                continue;
            }

            double oldAvg = avg[i];
            avg[i] = avg[i]*((double)readingsTaken/(readingsTaken+1)) + readings[i]/(readingsTaken+1);
            // Calculate stdev with Welford's online algorithm
            m2[i] = m2[i] + (readings[i] - avg[i])*(readings[i] - oldAvg);
        }

        readingsTaken++;
    }
    for (uint8_t i=0; i<numSensors; i++)
        stdev[i] = m2[i]/(readingsTaken-1);
}

static void read_calibration_data(qtr8a_instance_e instance, double *levels, uint8_t numSensors) {
    uint16_t readings[MAX_IR_ARRAY_SENSORS] = {0U};
    double tempLevels[MAX_IR_ARRAY_SENSORS] = {0.0F};

    uint32_t startTime = HAL_GetTick();
    uint32_t readingsTaken = 0;

    while(HAL_GetTick()-startTime < SENSOR_CALIB_TIME) {
        if (qtr8a_get_readings(instance, readings, numSensors, IR_ARRAY_ADC_TIMEOUT)) {
            for (uint8_t i=0; i<numSensors; i++) {
            	if (readingsTaken > 0) {
					tempLevels[i] = tempLevels[i]*((double)readingsTaken/(readingsTaken+1)) + readings[i]/(readingsTaken+1);
            	} else {
                    tempLevels[i] += readings[i];
            	}
            }
            readingsTaken++;
        }
    }

    for (uint8_t i=0; i<numSensors; i++) {
        if (levels[i] != 0)
            levels[i] = (levels[i] + tempLevels[i])/2;
        else
            levels[i] = tempLevels[i];
    }
}

static void alternate_calibrate_qtr8a(qtr8a_instance_e position,
                                      double *avg, double *stdev,
                                      uint8_t numSensors,
                                      uint8_t blinkIndicator,
                                      uint8_t repetitions) {
    memset(avg, 0.0F, numSensors*sizeof(double));
    memset(stdev, 0.0F, numSensors*sizeof(double));

    for (uint8_t i=0; i<repetitions; i++) {
        blink_ld2_n_times(blinkIndicator, 1000);
        wait_for_button_press();
        collect_calibration_statistics(position, avg, stdev, numSensors);
    }
}

static void calibrate_qtr8a_position_colour(qtr8a_instance_e position,
                                            double *levels,
                                            uint8_t numSensors,
                                            uint8_t blinkIndicator,
                                            uint8_t repetitions) {
    memset(levels, 0.0F, numSensors*sizeof(double));

    for (uint8_t i=0; i<repetitions; i++) {
        blink_ld2_n_times(blinkIndicator, 1000);
        wait_for_button_press();
        read_calibration_data(position, levels, numSensors);
    }
}

void calibration_sequence() {
    double frontLevels[FRONT_IR_ARRAY_SENSORS];
    double backLevels[BACK_IR_ARRAY_SENSORS];

    // double frontStdev[FRONT_IR_ARRAY_SENSORS];
    // double backStdev[BACK_IR_ARRAY_SENSORS];

    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

    /*
    alternate_calibrate_qtr8a(FRONT, frontLevels, frontStdev, FRONT_IR_ARRAY_SENSORS, 1, 1);
    qtr8a_set_levels(FRONT, RED, frontLevels);
    qtr8a_set_range(FRONT, RED, frontStdev);
    */

    // Front red calibration
    calibrate_qtr8a_position_colour(FRONT, frontLevels, FRONT_IR_ARRAY_SENSORS, 1, 1);
    qtr8a_set_levels(FRONT, RED, frontLevels);

    // Front brown calibration
    calibrate_qtr8a_position_colour(FRONT, frontLevels, FRONT_IR_ARRAY_SENSORS, 1, 1);
    qtr8a_set_levels(FRONT, BROWN, frontLevels);

    // Back red calibration
    //calibrate_qtr8a_position_colour(BACK, backLevels, BACK_IR_ARRAY_SENSORS, 2, 1);
    //qtr8a_set_levels(BACK, RED, backLevels);

    // Back brown calibration
    //calibrate_qtr8a_position_colour(BACK, backLevels, BACK_IR_ARRAY_SENSORS, 2, 1);
    //qtr8a_set_levels(BACK, BROWN, backLevels);

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    wait_for_button_press();
    HAL_Delay(1000);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
