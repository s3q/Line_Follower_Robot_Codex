#include "reaction.h"
#include "collision.h"
#include "motor.h"

/*
 * Conservative reaction timings in control-loop update cycles.
 * With the current 10 ms loop period, these are short, safe motions.
 */
#define REACTION_STOP_CYCLES            5U
#define REACTION_BACK_CYCLES            20U
#define REACTION_TURN_CYCLES            15U

/*
 * Conservative reaction motor duties.
 * Keep these modest because reaction is a safety maneuver, not a navigation
 * behavior.
 */
#define REACTION_BACK_DUTY              110U
#define REACTION_TURN_DUTY              110U

typedef enum {
    REACTION_PHASE_IDLE = 0,
    REACTION_PHASE_STOP,
    REACTION_PHASE_BACK,
    REACTION_PHASE_TURN
} ReactionPhase;

static ReactionPhase gReactionPhase = REACTION_PHASE_IDLE;
static uint8_t gReactionZone = COLLISION_NONE;
static uint32_t gReactionPhaseCycles = 0U;

void Reaction_Init(void)
{
    gReactionPhase = REACTION_PHASE_IDLE;
    gReactionZone = COLLISION_NONE;
    gReactionPhaseCycles = 0U;
}

bool Reaction_IsActive(void)
{
    return (gReactionPhase != REACTION_PHASE_IDLE);
}

void Reaction_Execute(uint8_t collisionZone)
{
    /*
     * Collision reactions must be immediate because a bumper press means the
     * robot has already made physical contact.
     *
     * Left/right impacts steer away from the impacted side after backing up.
     * Center impacts back straight away. Multi-zone impacts use the most
     * conservative response: emergency stop.
     */
    if (collisionZone == COLLISION_MULTI) {
        Motor_EmergencyStop();
        gReactionPhase = REACTION_PHASE_IDLE;
        gReactionZone = COLLISION_MULTI;
        gReactionPhaseCycles = 0U;
        return;
    }

    if ((collisionZone == COLLISION_NONE) && (gReactionPhase == REACTION_PHASE_IDLE)) {
        gReactionPhase = REACTION_PHASE_IDLE;
        gReactionZone = COLLISION_NONE;
        gReactionPhaseCycles = 0U;
        return;
    }

    if ((collisionZone != COLLISION_NONE) &&
        ((gReactionPhase == REACTION_PHASE_IDLE) || (collisionZone != gReactionZone))) {
        gReactionZone = collisionZone;
        gReactionPhase = REACTION_PHASE_STOP;
        gReactionPhaseCycles = 0U;
    }

    switch (gReactionPhase) {
    case REACTION_PHASE_STOP:
        Motor_Stop();
        if (++gReactionPhaseCycles >= REACTION_STOP_CYCLES) {
            gReactionPhase = REACTION_PHASE_BACK;
            gReactionPhaseCycles = 0U;
        }
        break;

    case REACTION_PHASE_BACK:
        Motor_Backward(REACTION_BACK_DUTY, REACTION_BACK_DUTY);
        if (++gReactionPhaseCycles >= REACTION_BACK_CYCLES) {
            if (gReactionZone == COLLISION_CENTER) {
                gReactionPhase = REACTION_PHASE_IDLE;
                gReactionZone = COLLISION_NONE;
                gReactionPhaseCycles = 0U;
                Motor_Stop();
            } else {
                gReactionPhase = REACTION_PHASE_TURN;
                gReactionPhaseCycles = 0U;
            }
        }
        break;

    case REACTION_PHASE_TURN:
        if (gReactionZone == COLLISION_LEFT) {
            Motor_Right(REACTION_TURN_DUTY, REACTION_TURN_DUTY);
        } else if (gReactionZone == COLLISION_RIGHT) {
            Motor_Left(REACTION_TURN_DUTY, REACTION_TURN_DUTY);
        } else {
            Motor_Stop();
        }

        if (++gReactionPhaseCycles >= REACTION_TURN_CYCLES) {
            gReactionPhase = REACTION_PHASE_IDLE;
            gReactionZone = COLLISION_NONE;
            gReactionPhaseCycles = 0U;
            Motor_Stop();
        }
        break;

    case REACTION_PHASE_IDLE:
    default:
        break;
    }
}
