#ifndef VERIFY_H_
#define VERIFY_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Verification/debug watch variables for front-sensor interpretation.
 * - bump switches are front collision sensors
 * - center IR is front-looking distance sensing on P6.1
 * - this stage only checks sensor interpretation, not full robot behavior
 */
extern volatile uint8_t gVerifyBumpMask;
extern volatile int8_t gVerifyBumpIndex;
extern volatile bool gVerifyMultipleBumps;
extern volatile uint16_t gVerifyFrontIrRaw;

void Verify_Init(void);
void Verify_Update(void);

#endif /* VERIFY_H_ */
