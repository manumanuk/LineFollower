#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	FORWARD=0,
	BACKWARD
} motor_dir_e;

typedef enum {
	RED = 0,
	BROWN,
	GREEN,
	BLUE
} line_colour_e;

void ctrl_bang_bang_get_motor_cmd(uint16_t *qtrReadings, uint8_t numReadings, uint8_t *lMotorPwm, uint8_t *rMotorPwm);
void motor_command(uint8_t lMotorPwm, uint8_t rMotorPwm);
void init_motors();
void motor_switch_directions(motor_dir_e dir);

#endif /* CONTROL_H_ */
