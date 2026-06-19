#include "bluetooth.h"
#include "motor.h"
#include "usart.h"

volatile uint8_t Rx_data;
extern volatile uint8_t Drive_mode;

void Bluetooth_Init(void)
{
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rx_data, 1); //블루투스 모듈과 연결된 uart1에서 데이터 받아오기
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//현재 최고 속도는 999
{
  if(huart->Instance == USART1)
  {
    switch(Rx_data)
    {
      case 'A': // 자율주행 시작
        Drive_mode = 1;
        break;

      case 'M': // 자율주행 중지
        Drive_mode = 0;
        MotorA_Control(0, DIR_STOP);
        MotorB_Control(0, DIR_STOP);
        break;

      case 'F':
        MotorA_Control(700, DIR_FORWARD);
        MotorB_Control(700, DIR_FORWARD);
        break;

      case 'B':
        MotorA_Control(700, DIR_BACKWARD);
        MotorB_Control(700, DIR_BACKWARD);
        break;

      case 'R':
        MotorA_Control(700, DIR_FORWARD);
        MotorB_Control(700, DIR_BACKWARD);
        break;

      case 'L':
        MotorA_Control(700, DIR_BACKWARD);
        MotorB_Control(700, DIR_FORWARD);
        break;

      case 'S':
        MotorA_Control(0, DIR_STOP);
        MotorB_Control(0, DIR_STOP);
        break;
    }

    HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rx_data, 1);
  }
}
