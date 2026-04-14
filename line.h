#ifndef LINE_H_
#define LINE_H_

#include <stdint.h>

/*
 * TI-RSLK MAX line sensor array:
 * Emitter controls: P5.3 and P9.2
 * Sensor signals:   P7.7..P7.0
 *
 * Returned bit order from Line_Read():
 * bit0=P7.0, bit1=P7.1, ... bit7=P7.7
 *
 * Logic convention:
 * 0 = white, 1 = black
 */
void Line_Init(void);
uint8_t Line_Read(uint32_t sample_us);

#endif /* LINE_H_ */

