#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

void motor_command(int32_t lMotorPwm, int32_t rMotorPwm);
void init_motors();
void gripper_grip();
void gripper_release();
void halt_gripper();
void call_lf_sequence();
void call_grpg_sequence();

#endif /* CONTROL_H_ */
