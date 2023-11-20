#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	FORWARD=0,
	BACKWARD
} motor_dir_e;

void ctrl_pid_get_motor_cmd(double position, uint32_t *lMotorPwm, uint32_t *rMotorPwm);
void ctrl_bang_bang_get_motor_cmd(double position, uint32_t *lMotorPwm, uint32_t *rMotorPwm);
void motor_command(uint32_t lMotorPwm, uint32_t rMotorPwm);
void init_motors();
void motor_switch_directions(motor_dir_e dir);
void gripper_grip();
void gripper_release();
void call_grpr_sequence();
void call_grpg_sequence();
void halt_gripper();

#endif /* CONTROL_H_ */
