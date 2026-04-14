#include "bump.h"
#include "robot_pins.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

void Bump_Init(void)
{
    /*
     * Configure bump pins as GPIO inputs with internal pull-ups.
     * Hardware behavior:
     * - not pressed: line held high (~3.3V) by pull-up
     * - pressed: switch connects line to GND (0V)
     */
    GPIO_setAsInputPinWithPullUpResistor(BUMP_PORT, BUMP_ALL_PINS);
}

uint8_t Bump_Read(void)
{
    uint8_t inP4;
    uint8_t activeLow;
    uint8_t packed;

    /* Read raw P4 input bits, then invert for active-low logic. */
    inP4 = ((GPIO_getInputPinValue(BUMP_PORT, BUMP0_PIN) != 0U) ? 1U : 0U) << 0;
    inP4 |= ((GPIO_getInputPinValue(BUMP_PORT, BUMP1_PIN) != 0U) ? 1U : 0U) << 2;
    inP4 |= ((GPIO_getInputPinValue(BUMP_PORT, BUMP2_PIN) != 0U) ? 1U : 0U) << 3;
    inP4 |= ((GPIO_getInputPinValue(BUMP_PORT, BUMP3_PIN) != 0U) ? 1U : 0U) << 5;
    inP4 |= ((GPIO_getInputPinValue(BUMP_PORT, BUMP4_PIN) != 0U) ? 1U : 0U) << 6;
    inP4 |= ((GPIO_getInputPinValue(BUMP_PORT, BUMP5_PIN) != 0U) ? 1U : 0U) << 7;

    activeLow = (uint8_t)(~inP4);

    /* Pack sparse port bits into compact bitmask: bit0..bit5 = Bump0..Bump5 */
    packed = 0U;
    packed |= ((activeLow >> 0) & 0x01U) << 0;  /* P4.0 -> bit0 */
    packed |= ((activeLow >> 2) & 0x01U) << 1;  /* P4.2 -> bit1 */
    packed |= ((activeLow >> 3) & 0x01U) << 2;  /* P4.3 -> bit2 */
    packed |= ((activeLow >> 5) & 0x01U) << 3;  /* P4.5 -> bit3 */
    packed |= ((activeLow >> 6) & 0x01U) << 4;  /* P4.6 -> bit4 */
    packed |= ((activeLow >> 7) & 0x01U) << 5;  /* P4.7 -> bit5 */

    return packed;
}
