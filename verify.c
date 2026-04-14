#include "verify.h"
#include "bump.h"
#include "ir.h"

volatile uint8_t gVerifyBumpMask = 0U;
volatile int8_t gVerifyBumpIndex = -1;
volatile bool gVerifyMultipleBumps = false;
volatile uint16_t gVerifyFrontIrRaw = 0U;

static int8_t Verify_ExtractBumpIndex(uint8_t bumpMask, bool *multiplePressed);

void Verify_Init(void)
{
    /*
     * Front verification stage:
     * - bump switches report front collision contact
     * - center IR reports front-looking analog distance/reflection level
     * - this module only verifies raw interpretation and decoding
     */
    Bump_Init();
    IR_Init();

    gVerifyBumpMask = 0U;
    gVerifyBumpIndex = -1;
    gVerifyMultipleBumps = false;
    gVerifyFrontIrRaw = 0U;
}

void Verify_Update(void)
{
    gVerifyBumpMask = Bump_Read();
    gVerifyBumpIndex = Verify_ExtractBumpIndex(gVerifyBumpMask, (bool *)&gVerifyMultipleBumps);
    gVerifyFrontIrRaw = IR_ReadRaw();
}

static int8_t Verify_ExtractBumpIndex(uint8_t bumpMask, bool *multiplePressed)
{
    uint8_t bit;
    int8_t foundIndex;

    *multiplePressed = false;
    foundIndex = -1;

    for (bit = 0U; bit < 6U; bit++) {
        if ((bumpMask & (uint8_t)(1U << bit)) != 0U) {
            if (foundIndex >= 0) {
                *multiplePressed = true;
                return -1;
            }
            foundIndex = (int8_t)bit;
        }
    }

    return foundIndex;
}
