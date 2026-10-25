#ifndef BOARD_C10B_H
#define BOARD_C10B_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* C10B schematic: U6/CN3 = motor A, U5/CN2 = motor B. */
#define BOARD_MOTOR_A_PORT       GPIOA
#define BOARD_MOTOR_A_IN1_PIN    GPIO_PIN_6  /* TIM3_CH1 */
#define BOARD_MOTOR_A_IN2_PIN    GPIO_PIN_7  /* TIM3_CH2 */
#define BOARD_MOTOR_B_PORT       GPIOB
#define BOARD_MOTOR_B_IN1_PIN    GPIO_PIN_0  /* TIM3_CH3 */
#define BOARD_MOTOR_B_IN2_PIN    GPIO_PIN_1  /* TIM3_CH4 */
#define BOARD_BUTTON_PORT        GPIOA
#define BOARD_BUTTON_PIN         GPIO_PIN_0  /* USER, high when pressed */
#define BOARD_ENABLE_PORT        GPIOC
#define BOARD_ENABLE_PIN         GPIO_PIN_13 /* low permits motion */
#define BOARD_LED_PORT           GPIOC
#define BOARD_LED_RUN_PIN        GPIO_PIN_2  /* active low */
#define BOARD_LED_FAULT_PIN      GPIO_PIN_3  /* active low */
#define BOARD_GRAY_PORT          GPIOC
#define BOARD_GRAY_TX_PIN        GPIO_PIN_10 /* CN6.3 -> module RX */
#define BOARD_GRAY_RX_PIN        GPIO_PIN_11 /* CN6.4 <- module TX */
#define BOARD_MPU_PORT           GPIOB
#define BOARD_MPU_SCL_PIN        GPIO_PIN_14
#define BOARD_MPU_SDA_PIN        GPIO_PIN_15

#define BOARD_ENCODER_A_PORT     GPIOB
#define BOARD_ENCODER_A_CH1_PIN  GPIO_PIN_6
#define BOARD_ENCODER_A_CH2_PIN  GPIO_PIN_7
#define BOARD_ENCODER_B_PORT     GPIOC
#define BOARD_ENCODER_B_CH1_PIN  GPIO_PIN_6
#define BOARD_ENCODER_B_CH2_PIN  GPIO_PIN_7
#define BOARD_BUZZER_PORT        GPIOA
#define BOARD_BUZZER_PIN         GPIO_PIN_15

/* Original C10B OLED: four-wire software SPI, CS tied on module. */
#define BOARD_OLED_PORT         GPIOB
#define BOARD_OLED_DC_PIN       GPIO_PIN_3
#define BOARD_OLED_RST_PIN      GPIO_PIN_4
#define BOARD_OLED_DATA_PIN     GPIO_PIN_5
#define BOARD_OLED_CLOCK_PORT   GPIOC
#define BOARD_OLED_CLOCK_PIN    GPIO_PIN_14

void Board_SetBuzzer(bool on);
void Board_Init(void);
bool Board_ButtonPressed(void);
bool Board_MotionEnabled(void);
void Board_MotorApply(uint16_t left, uint16_t right);
bool Board_MotorBrake(void);
void Board_SetLeds(bool running, bool fault);
void Board_EmergencyStop(void);

#endif
