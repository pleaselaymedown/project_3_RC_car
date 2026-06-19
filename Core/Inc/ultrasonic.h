#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H
#include "main.h"

extern void delay_us(uint32_t us);

#define Front_Trigger()  Ultrasonic_Trigger(FRONT_TRIG_PORT, FRONT_TRIG_PIN)
#define Left_Trigger()   Ultrasonic_Trigger(LEFT_TRIG_PORT, LEFT_TRIG_PIN)
#define Right_Trigger()  Ultrasonic_Trigger(RIGHT_TRIG_PORT, RIGHT_TRIG_PIN)

// 범용 트리거 함수
void Ultrasonic_Trigger(GPIO_TypeDef* PORT, uint16_t PIN);

uint16_t Calculate_EchoTime(uint16_t Value1, uint16_t Value2);

// ECHO(Input Capture)
#define FRONT_ECHO_PORT     GPIOA
#define FRONT_ECHO_PIN      GPIO_PIN_6

#define LEFT_ECHO_PORT      GPIOA
#define LEFT_ECHO_PIN       GPIO_PIN_7

#define RIGHT_ECHO_PORT     GPIOB
#define RIGHT_ECHO_PIN      GPIO_PIN_0


// TRIG(Output Capture)
#define FRONT_TRIG_PORT     GPIOB
#define FRONT_TRIG_PIN      GPIO_PIN_13

#define LEFT_TRIG_PORT      GPIOB
#define LEFT_TRIG_PIN       GPIO_PIN_14

#define RIGHT_TRIG_PORT     GPIOB
#define RIGHT_TRIG_PIN      GPIO_PIN_15

#endif
