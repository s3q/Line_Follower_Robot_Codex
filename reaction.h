#ifndef REACTION_H_
#define REACTION_H_

#include <stdbool.h>
#include <stdint.h>

void Reaction_Init(void);
bool Reaction_IsActive(void);
void Reaction_Execute(uint8_t collisionZone);

#endif /* REACTION_H_ */
