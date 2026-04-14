#include "control.h"
#include "mission.h"

volatile ControlState gControlState = CONTROL_STATE_LINE_FOLLOW;

void Control_Init(void)
{
    Mission_Init();
    gControlState = Mission_GetMode();
}

void Control_Update(void)
{
    Mission_Update();
    gControlState = Mission_GetMode();
}
