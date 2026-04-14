#ifndef IR_H_
#define IR_H_

#include <stdint.h>

/*
 * TI-RSLK MAX IR analog inputs:
 * Center IR = P6.1 -> ADC14 input A14
 * Left IR   = P9.1
 * Right IR  = P9.0
 *
 * This module currently supports center IR only.
 */
void IR_Init(void);
uint16_t IR_ReadRaw(void);

#endif /* IR_H_ */
