#ifndef EMERGENCY_H_
#define EMERGENCY_H_

#include <stdbool.h>

void Emergency_Init(void);
bool Emergency_IsActive(void);
void Emergency_Update(void);

#endif /* EMERGENCY_H_ */
