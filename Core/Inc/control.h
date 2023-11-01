#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

bool ctrl_bang_bang_get_motor_cmd(uint16_t *qtrReadings, uint8_t numReadings, uint8_t *lMotorPwm, uint8_t *rMotorPwm);
void motor_command(uint8_t lMotorPwm, uint8_t rMotorPwm);
void init_motors();

#endif /* CONTROL_H_ */
