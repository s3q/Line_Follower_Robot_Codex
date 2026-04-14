#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>

typedef enum {
    CONTROL_STATE_LINE_FOLLOW = 0,
    CONTROL_STATE_OBSTACLE_AVOID = 1,
    CONTROL_STATE_TORQUE_RAMP = 2,
    CONTROL_STATE_STOP = 3,
    CONTROL_STATE_OBSTACLE_APPROACH_CONFIRM = 4,
    CONTROL_STATE_EMERGENCY_BUMP = 5,
    CONTROL_STATE_OBSTACLE_BYPASS = 6,
    CONTROL_STATE_LINE_REACQUIRE_GATED = 7,
    CONTROL_STATE_COLLISION = 8,
    CONTROL_STATE_EMERGENCY = 9
} ControlState;

extern volatile ControlState gControlState;

void Control_Init(void);
void Control_Update(void);

#endif /* CONTROL_H_ */
