#include "motor.h"
#include "robot_pins.h"
#include <stdbool.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#define MOTOR_PWM_TIMER_BASE             TIMER_A0_BASE
#define MOTOR_PWM_LEFT_CC_REG            TIMER_A_CAPTURECOMPARE_REGISTER_4
#define MOTOR_PWM_RIGHT_CC_REG           TIMER_A_CAPTURECOMPARE_REGISTER_3
#define MOTOR_DUTY_MIN                   0U
#define MOTOR_DUTY_MAX                   (MOTOR_PWM_PERIOD_TICKS - 1U)

static bool gMotorInitDone = false;

static uint16_t Motor_ClampDuty(uint16_t duty);
static void Motor_PwmPinsToTimer(void);
static void Motor_PwmPinsToGpioLow(void);
static void Motor_SetSleep(bool leftOn, bool rightOn);
static void Motor_SetDirection(bool leftBackward, bool rightBackward);

static uint16_t Motor_ClampDuty(uint16_t duty)
{
    if (duty < MOTOR_DUTY_MIN) {
        return MOTOR_DUTY_MIN;
    }
    if (duty > MOTOR_DUTY_MAX) {
        return MOTOR_DUTY_MAX;
    }
    return duty;
}

static void Motor_PwmPinsToTimer(void)
{
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        MOTOR_LEFT_PWM_PORT, MOTOR_LEFT_PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        MOTOR_RIGHT_PWM_PORT, MOTOR_RIGHT_PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

static void Motor_PwmPinsToGpioLow(void)
{
    GPIO_setAsOutputPin(MOTOR_LEFT_PWM_PORT, MOTOR_LEFT_PWM_PIN);
    GPIO_setAsOutputPin(MOTOR_RIGHT_PWM_PORT, MOTOR_RIGHT_PWM_PIN);
    GPIO_setOutputLowOnPin(MOTOR_LEFT_PWM_PORT, MOTOR_LEFT_PWM_PIN);
    GPIO_setOutputLowOnPin(MOTOR_RIGHT_PWM_PORT, MOTOR_RIGHT_PWM_PIN);
}

static void Motor_SetSleep(bool leftOn, bool rightOn)
{
    if (leftOn) {
        GPIO_setOutputHighOnPin(MOTOR_LEFT_SLP_PORT, MOTOR_LEFT_SLP_PIN);
    } else {
        GPIO_setOutputLowOnPin(MOTOR_LEFT_SLP_PORT, MOTOR_LEFT_SLP_PIN);
    }

    if (rightOn) {
        GPIO_setOutputHighOnPin(MOTOR_RIGHT_SLP_PORT, MOTOR_RIGHT_SLP_PIN);
    } else {
        GPIO_setOutputLowOnPin(MOTOR_RIGHT_SLP_PORT, MOTOR_RIGHT_SLP_PIN);
    }
}

static void Motor_SetDirection(bool leftBackward, bool rightBackward)
{
    if (leftBackward) {
        GPIO_setOutputHighOnPin(MOTOR_LEFT_DIR_PORT, MOTOR_LEFT_DIR_PIN);
    } else {
        GPIO_setOutputLowOnPin(MOTOR_LEFT_DIR_PORT, MOTOR_LEFT_DIR_PIN);
    }

    if (rightBackward) {
        GPIO_setOutputHighOnPin(MOTOR_RIGHT_DIR_PORT, MOTOR_RIGHT_DIR_PIN);
    } else {
        GPIO_setOutputLowOnPin(MOTOR_RIGHT_DIR_PORT, MOTOR_RIGHT_DIR_PIN);
    }
}

void Motor_SetDuty(uint16_t leftDuty, uint16_t rightDuty)
{
    if (!gMotorInitDone) {
        return;
    }

    Timer_A_setCompareValue(
        MOTOR_PWM_TIMER_BASE, MOTOR_PWM_LEFT_CC_REG, Motor_ClampDuty(leftDuty));
    Timer_A_setCompareValue(
        MOTOR_PWM_TIMER_BASE, MOTOR_PWM_RIGHT_CC_REG, Motor_ClampDuty(rightDuty));
}

void Motor_Init(void)
{
    Timer_A_UpModeConfig upCfg = {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_8,
        MOTOR_PWM_PERIOD_TICKS,
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
        TIMER_A_DO_CLEAR
    };

    Timer_A_CompareModeConfig lPwmCfg = {
        MOTOR_PWM_LEFT_CC_REG,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0U
    };

    Timer_A_CompareModeConfig rPwmCfg = {
        MOTOR_PWM_RIGHT_CC_REG,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0U
    };

    GPIO_setAsOutputPin(MOTOR_LEFT_DIR_PORT, MOTOR_LEFT_DIR_PIN);
    GPIO_setAsOutputPin(MOTOR_RIGHT_DIR_PORT, MOTOR_RIGHT_DIR_PIN);
    GPIO_setAsOutputPin(MOTOR_LEFT_SLP_PORT, MOTOR_LEFT_SLP_PIN);
    GPIO_setAsOutputPin(MOTOR_RIGHT_SLP_PORT, MOTOR_RIGHT_SLP_PIN);

    GPIO_setOutputLowOnPin(MOTOR_LEFT_DIR_PORT, MOTOR_LEFT_DIR_PIN);
    GPIO_setOutputLowOnPin(MOTOR_RIGHT_DIR_PORT, MOTOR_RIGHT_DIR_PIN);
    GPIO_setOutputLowOnPin(MOTOR_LEFT_SLP_PORT, MOTOR_LEFT_SLP_PIN);
    GPIO_setOutputLowOnPin(MOTOR_RIGHT_SLP_PORT, MOTOR_RIGHT_SLP_PIN);

    Motor_PwmPinsToTimer();
    Timer_A_configureUpMode(MOTOR_PWM_TIMER_BASE, &upCfg);
    Timer_A_initCompare(MOTOR_PWM_TIMER_BASE, &lPwmCfg);
    Timer_A_initCompare(MOTOR_PWM_TIMER_BASE, &rPwmCfg);
    Timer_A_startCounter(MOTOR_PWM_TIMER_BASE, TIMER_A_UP_MODE);

    gMotorInitDone = true;
    Motor_EmergencyStop();
}

void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty)
{
    if (!gMotorInitDone) {
        Motor_Init();
    }

    Motor_PwmPinsToTimer();
    Motor_SetDirection(false, false);
    Motor_SetDuty(leftDuty, rightDuty);
    Timer_A_clearTimer(MOTOR_PWM_TIMER_BASE);
    Timer_A_startCounter(MOTOR_PWM_TIMER_BASE, TIMER_A_UP_MODE);
    Motor_SetSleep(true, true);
}

void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty)
{
    if (!gMotorInitDone) {
        Motor_Init();
    }

    Motor_PwmPinsToTimer();
    Motor_SetDirection(true, true);
    Motor_SetDuty(leftDuty, rightDuty);
    Timer_A_clearTimer(MOTOR_PWM_TIMER_BASE);
    Timer_A_startCounter(MOTOR_PWM_TIMER_BASE, TIMER_A_UP_MODE);
    Motor_SetSleep(true, true);
}

void Motor_Left(uint16_t leftDuty, uint16_t rightDuty)
{
    if (!gMotorInitDone) {
        Motor_Init();
    }

    Motor_PwmPinsToTimer();
    Motor_SetDirection(true, false);
    Motor_SetDuty(leftDuty, rightDuty);
    Timer_A_clearTimer(MOTOR_PWM_TIMER_BASE);
    Timer_A_startCounter(MOTOR_PWM_TIMER_BASE, TIMER_A_UP_MODE);
    Motor_SetSleep(true, true);
}

void Motor_Right(uint16_t leftDuty, uint16_t rightDuty)
{
    if (!gMotorInitDone) {
        Motor_Init();
    }

    Motor_PwmPinsToTimer();
    Motor_SetDirection(false, true);
    Motor_SetDuty(leftDuty, rightDuty);
    Timer_A_clearTimer(MOTOR_PWM_TIMER_BASE);
    Timer_A_startCounter(MOTOR_PWM_TIMER_BASE, TIMER_A_UP_MODE);
    Motor_SetSleep(true, true);
}

void Motor_Stop(void)
{
    if (!gMotorInitDone) {
        return;
    }

    Motor_SetDuty(0U, 0U);
    Motor_SetSleep(false, false);
    Timer_A_stopTimer(MOTOR_PWM_TIMER_BASE);
    Motor_PwmPinsToGpioLow();
    Motor_SetDirection(false, false);
}

void Motor_EmergencyStop(void)
{
    if (!gMotorInitDone) {
        return;
    }

    __disable_irq();
    Motor_SetDuty(0U, 0U);
    Timer_A_stopTimer(MOTOR_PWM_TIMER_BASE);
    Motor_PwmPinsToGpioLow();
    Motor_SetSleep(false, false);
    Motor_SetDirection(false, false);
    __enable_irq();
}
