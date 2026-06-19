#include "ultrasonic.h"

void Ultrasonic_Trigger(GPIO_TypeDef* PORT, uint16_t PIN)
{
    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_SET);
    delay_us(100);
    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_RESET);
}

