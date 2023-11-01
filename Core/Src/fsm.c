#include "fsm.h"

bool transition_state(robot_state_e *state, robot_event_e event) {
    switch (*state) {
        case IDLE: {
            switch(event) {
                case SW:
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
                case SWLONG:
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
                    return true;
                case BLUE:
                    *state = GRPG;
                default:
                    return false;
            }
        }
        case LFR: {
            switch(event) {
                case SW:
                    *state = IDLE;
                    return true;
                case GREEN:
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
                    return true;
                case GRPG_CMPL:
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
                    return true;
                case GRPR_CMPL:
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
