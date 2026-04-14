#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include <stdint.h>

/*
 * TI-RSLK MAX ultrasonic pin mapping
 *
 * Left ultrasonic:
 * TRIG = P6.3 (AUXL)
 * ECHO = P9.1 (DISTL)
 *
 * Right ultrasonic:
 * TRIG = P6.2 (AUXR)
 * ECHO = P9.0 (DISTR)
 *
 * Electrical note:
 * HC-SR04 style ECHO is typically 5V logic. It must be level-shifted to 3.3V
 * before connecting to the MSP432 input pins.
 *
 * Practical setup notes for downward-looking ground sensing:
 * - Mount one sensor on the front-left and one on the front-right.
 * - Aim about 35 to 40 degrees downward from horizontal.
 * - Aim about 10 to 15 degrees outward.
 * - Keep both sensors at the same height and mounted symmetrically.
 * - Point both sensors at similar left/right ground zones.
 *
 * Practical behavior notes:
 * - On flat floor, left and right should be close to each other.
 * - A normal reading may be around 8 cm to 20 cm, depending on mounting.
 * - Ultrasonic readings can be noisy when aimed at the ground at an angle.
 * - Surface material and texture matter.
 * - Readings should be filtered or averaged in higher-level code.
 * - Never trigger left and right at the same time; read one sensor at a time.
 */
void Ultrasonic_Init(void);
float Ultrasonic_ReadLeftCm(void);
float Ultrasonic_ReadRightCm(void);

#endif /* ULTRASONIC_H_ */
