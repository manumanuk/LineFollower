#ifndef FSM_H_
#define FSM_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    IDLE,
    CALIB,
    LFF,
    LFR,
    GRPG,
    GRPR
} robot_state_e;

typedef enum {
    SW,
    SWLONG,
    BLUE,
    GREEN,
    GRPR_CMPL,
    GRPG_CMPL,
    CALIB_CMPL
} robot_event_e;

bool transition_state(robot_state_e *state, robot_event_e event);
#endif /* FSM_H_ */
