#include "collision.h"

/*
 * Zone grouping for the TI-RSLK MAX front bump switches:
 * - left zone   : Bump0, Bump1 -> bit0, bit1
 * - center zone : Bump2, Bump3 -> bit2, bit3
 * - right zone  : Bump4, Bump5 -> bit4, bit5
 *
 * If future mechanical testing shows a different physical grouping is better,
 * only these masks need to be adjusted.
 */
#define COLLISION_LEFT_MASK              ((uint8_t)0x03U)
#define COLLISION_CENTER_MASK            ((uint8_t)0x0CU)
#define COLLISION_RIGHT_MASK             ((uint8_t)0x30U)

void Collision_Init(void)
{
    /* No hardware state to initialize for bump-mask interpretation. */
}

uint8_t Collision_FromBumpMask(uint8_t bumpMask)
{
    uint8_t zonesActive;

    zonesActive = 0U;

    if ((bumpMask & COLLISION_LEFT_MASK) != 0U) {
        zonesActive |= COLLISION_LEFT_MASK;
    }
    if ((bumpMask & COLLISION_CENTER_MASK) != 0U) {
        zonesActive |= COLLISION_CENTER_MASK;
    }
    if ((bumpMask & COLLISION_RIGHT_MASK) != 0U) {
        zonesActive |= COLLISION_RIGHT_MASK;
    }

    if (zonesActive == 0U) {
        return COLLISION_NONE;
    }
    if (zonesActive == COLLISION_LEFT_MASK) {
        return COLLISION_LEFT;
    }
    if (zonesActive == COLLISION_CENTER_MASK) {
        return COLLISION_CENTER;
    }
    if (zonesActive == COLLISION_RIGHT_MASK) {
        return COLLISION_RIGHT;
    }
    return COLLISION_MULTI;
}
