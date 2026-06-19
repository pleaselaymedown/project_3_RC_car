#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define MOTOR_A_FORWARD_PIN GPIO_PIN_5
#define MOTOR_A_BACKWARD_PIN GPIO_PIN_6
#define MOTOR_B_FORWARD_PIN GPIO_PIN_8
#define MOTOR_B_BACKWARD_PIN GPIO_PIN_9
#define MOTOR_GPIO_PORT GPIOC

#include "main.h"

typedef enum
{
    DIR_STOP,
    DIR_FORWARD,
    DIR_BACKWARD
}MotorDir_t;


void MotorA_Control(uint16_t speed, MotorDir_t direction);
void MotorB_Control(uint16_t speed, MotorDir_t direction);

#endif
