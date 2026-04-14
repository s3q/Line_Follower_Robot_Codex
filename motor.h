#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include "robot_config.h"

/*
 * TI-RSLK MAX motor mapping (MSP432P401R):
 * Left:  DIRL=P5.4, PWML=P2.7(TA0.4), SLPL=P3.7
 * Right: DIRR=P5.5, PWMR=P2.6(TA0.3), SLPR=P3.6
 */

#define MOTOR_PWM_PERIOD_TICKS   ROBOT_CFG_MOTOR_PWM_PERIOD_TICKS

void Motor_Init(void);
void Motor_SetDuty(uint16_t leftDuty, uint16_t rightDuty);
void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty);
void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty);
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty);
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty);
void Motor_Stop(void);
void Motor_EmergencyStop(void);

#endif /* MOTOR_H_ */
