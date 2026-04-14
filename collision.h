#ifndef COLLISION_H_
#define COLLISION_H_

#include <stdint.h>

/*
 * High-level front collision zones derived from the 6-bit bump mask.
 */
#define COLLISION_NONE                   0U
#define COLLISION_LEFT                   1U
#define COLLISION_CENTER                 2U
#define COLLISION_RIGHT                  3U
#define COLLISION_MULTI                  4U

void Collision_Init(void);
uint8_t Collision_FromBumpMask(uint8_t bumpMask);

#endif /* COLLISION_H_ */
