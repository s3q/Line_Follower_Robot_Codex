#ifndef BUMP_H_
#define BUMP_H_

#include <stdint.h>

/*
 * TI-RSLK MAX bump switch map (active-low hardware):
 * bit0=Bump0(P4.0), bit1=Bump1(P4.2), bit2=Bump2(P4.3),
 * bit3=Bump3(P4.5), bit4=Bump4(P4.6), bit5=Bump5(P4.7)
 *
 * Return convention for Bump_Read():
 * 1 = pressed, 0 = not pressed
 */
void Bump_Init(void);
uint8_t Bump_Read(void);

#endif /* BUMP_H_ */
