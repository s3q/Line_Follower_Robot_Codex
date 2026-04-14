#ifndef ROBOT_PINS_H_
#define ROBOT_PINS_H_

/*
 * Onboard LaunchPad LEDs
 */
#define LED_RED_PORT                        GPIO_PORT_P1
#define LED_RED_PIN                         GPIO_PIN0
#define LED_GREEN_PORT                      GPIO_PORT_P2
#define LED_GREEN_PIN                       GPIO_PIN1
#define LED_BLUE_PORT                       GPIO_PORT_P2
#define LED_BLUE_PIN                        GPIO_PIN2

/*
 * LaunchPad XDS110 backchannel UART
 */
#define DEBUG_UART_MODULE                   EUSCI_A0_BASE
#define DEBUG_UART_TX_PORT                  GPIO_PORT_P1
#define DEBUG_UART_TX_PIN                   GPIO_PIN3
#define DEBUG_UART_RX_PORT                  GPIO_PORT_P1
#define DEBUG_UART_RX_PIN                   GPIO_PIN2

/*
 * Motor pins: TI-RSLK MAX / MSP432P401R
 */
#define MOTOR_LEFT_DIR_PORT                 GPIO_PORT_P5
#define MOTOR_LEFT_DIR_PIN                  GPIO_PIN4
#define MOTOR_LEFT_PWM_PORT                 GPIO_PORT_P2
#define MOTOR_LEFT_PWM_PIN                  GPIO_PIN7
#define MOTOR_LEFT_SLP_PORT                 GPIO_PORT_P3
#define MOTOR_LEFT_SLP_PIN                  GPIO_PIN7

#define MOTOR_RIGHT_DIR_PORT                GPIO_PORT_P5
#define MOTOR_RIGHT_DIR_PIN                 GPIO_PIN5
#define MOTOR_RIGHT_PWM_PORT                GPIO_PORT_P2
#define MOTOR_RIGHT_PWM_PIN                 GPIO_PIN6
#define MOTOR_RIGHT_SLP_PORT                GPIO_PORT_P3
#define MOTOR_RIGHT_SLP_PIN                 GPIO_PIN6

/*
 * Bump switch pins: bit order remains Bump0..Bump5
 */
#define BUMP_PORT                           GPIO_PORT_P4
#define BUMP0_PIN                           GPIO_PIN0
#define BUMP1_PIN                           GPIO_PIN2
#define BUMP2_PIN                           GPIO_PIN3
#define BUMP3_PIN                           GPIO_PIN5
#define BUMP4_PIN                           GPIO_PIN6
#define BUMP5_PIN                           GPIO_PIN7
#define BUMP_ALL_PINS                       (BUMP0_PIN | BUMP1_PIN | BUMP2_PIN | \
                                             BUMP3_PIN | BUMP4_PIN | BUMP5_PIN)

/*
 * Line sensor array pins
 */
#define LINE_EMIT_LEFT_PORT                 GPIO_PORT_P5
#define LINE_EMIT_LEFT_PIN                  GPIO_PIN3
#define LINE_EMIT_RIGHT_PORT                GPIO_PORT_P9
#define LINE_EMIT_RIGHT_PIN                 GPIO_PIN2

#define LINE_SENSOR_PORT                    GPIO_PORT_P7
#define LINE_SENSOR_BIT0_PIN                GPIO_PIN0
#define LINE_SENSOR_BIT1_PIN                GPIO_PIN1
#define LINE_SENSOR_BIT2_PIN                GPIO_PIN2
#define LINE_SENSOR_BIT3_PIN                GPIO_PIN3
#define LINE_SENSOR_BIT4_PIN                GPIO_PIN4
#define LINE_SENSOR_BIT5_PIN                GPIO_PIN5
#define LINE_SENSOR_BIT6_PIN                GPIO_PIN6
#define LINE_SENSOR_BIT7_PIN                GPIO_PIN7
#define LINE_SENSOR_ALL_PINS                (LINE_SENSOR_BIT0_PIN | LINE_SENSOR_BIT1_PIN | \
                                             LINE_SENSOR_BIT2_PIN | LINE_SENSOR_BIT3_PIN | \
                                             LINE_SENSOR_BIT4_PIN | LINE_SENSOR_BIT5_PIN | \
                                             LINE_SENSOR_BIT6_PIN | LINE_SENSOR_BIT7_PIN)

/*
 * IR analog sensor pins
 */
#define IR_CENTER_PORT                      GPIO_PORT_P6
#define IR_CENTER_PIN                       GPIO_PIN1
#define IR_CENTER_ADC_INPUT                 ADC_INPUT_A14

/*
 * Ultrasonic sensor pins
 */
//#define ULTRA_LEFT_TRIG_PORT                GPIO_PORT_P6
//#define ULTRA_LEFT_TRIG_PIN                 GPIO_PIN3

#define ULTRA_LEFT_TRIG_PORT                GPIO_PORT_P6
#define ULTRA_LEFT_TRIG_PIN                 GPIO_PIN3

#define ULTRA_LEFT_ECHO_PORT                GPIO_PORT_P9
#define ULTRA_LEFT_ECHO_PIN                 GPIO_PIN1

#define ULTRA_RIGHT_TRIG_PORT               GPIO_PORT_P6
#define ULTRA_RIGHT_TRIG_PIN                GPIO_PIN2
#define ULTRA_RIGHT_ECHO_PORT               GPIO_PORT_P9
#define ULTRA_RIGHT_ECHO_PIN                GPIO_PIN0

/*
 * MPU6500 I2C pins
 * P6.4/P6.5 are used for the IMU I2C bus.
 */
#define MPU6500_I2C_MODULE                  EUSCI_B1_BASE
#define MPU6500_I2C_SDA_PORT                GPIO_PORT_P6
#define MPU6500_I2C_SDA_PIN                 GPIO_PIN4
#define MPU6500_I2C_SCL_PORT                GPIO_PORT_P6
#define MPU6500_I2C_SCL_PIN                 GPIO_PIN5

#endif /* ROBOT_PINS_H_ */
