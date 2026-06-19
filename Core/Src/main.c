/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "motor.h"
#include "bluetooth.h"
#include "ultrasonic.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DANGER_DIST     13
#define RECOVER_DIST    25
#define BASE_SPEED      300
#define TURN_SPEED      300
#define BACK_SPEED      200
#define FWD_SPEED       BASE_SPEED
#define ACTION_TIME     300     // 0.3초

typedef enum {
  STATE_FORWARD = 0,
  STATE_BACKWARD,
  STATE_TURN_LEFT,
  STATE_TURN_RIGHT
} CarState;

CarState currentState = STATE_FORWARD;

uint32_t stateStartTime = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#ifdef __GNUC__
/* With GCC small printf (option LD Linker->Libraries->Small printf
 * set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int  __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int  fputc(int ch, FILE *f)
#endif /* __GNUC__*/

/** @brief Retargets the C library printf function to the USART.
 *  @param None
 *  @retval None
 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop
     until the end of transmission */
  if(ch == '\n')
    HAL_UART_Transmit(&huart2, (uint8_t*) "\r", 1, 0xFFFF);
  HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rxData;

uint16_t Front_Value1 = 0;       // Echo Pin Rising Check
uint16_t Front_Value2 = 0;       // Echo Pin Falling Check
uint16_t Right_Value1 = 0;
uint16_t Right_Value2 = 0;
uint16_t Left_Value1 = 0;
uint16_t Left_Value2 = 0;

uint16_t Front_echoTime = 0;
uint16_t Left_echoTime  = 0;
uint16_t Right_echoTime = 0;

uint16_t Front_captureFlag = 0;
uint16_t Left_captureFlag  = 0;
uint16_t Right_captureFlag = 0;

uint16_t Front_flag = 0;
uint16_t Left_flag  = 0;
uint16_t Right_flag = 0;

volatile uint16_t Front_distance = 0;
volatile uint16_t Left_distance  = 0;
volatile uint16_t Right_distance = 0;

uint8_t Drive_mode = 0;    // 0 = 수동주행, 1 = 자율주행

uint32_t baseTime = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM3)
  {
    // ================= FRONT (CH1) =================
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      if(Front_captureFlag == 0)
      {
        Front_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        Front_captureFlag = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
      }
      else
      {
        Front_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if(Front_Value2 > Front_Value1)
          Front_echoTime = Front_Value2 - Front_Value1;
        else
          Front_echoTime = (0xFFFF - Front_Value1) + Front_Value2;

        Front_distance = Front_echoTime / 58;

        Front_captureFlag = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      }
    }

    // ================= LEFT (CH2) =================
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      if(Left_captureFlag == 0)
      {
        Left_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        Left_captureFlag = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
      }
      else
      {
        Left_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

        if(Left_Value2 > Left_Value1)
          Left_echoTime = Left_Value2 - Left_Value1;
        else
          Left_echoTime = (0xFFFF - Left_Value1) + Left_Value2;

        Left_distance = Left_echoTime / 58;

        Left_captureFlag = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
      }
    }

    // ================= RIGHT (CH3) =================
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
      if(Right_captureFlag == 0)
      {
        Right_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        Right_captureFlag = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
      }
      else
      {
        Right_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

        if(Right_Value2 > Right_Value1)
          Right_echoTime = Right_Value2 - Right_Value1;
        else
          Right_echoTime = (0xFFFF - Right_Value1) + Right_Value2;

        Right_distance = Right_echoTime / 58;

        Right_captureFlag = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
      }
    }
  }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);   // Front
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);   // Left
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);   // Right

  Bluetooth_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (Drive_mode == 1) // 자율주행 모드
    {
      // 1. 센서 트리거링 (순차적으로 측정하여 간섭 방지)
      Front_Trigger();
      HAL_Delay(10);
      Left_Trigger();
      HAL_Delay(10);
      Right_Trigger();
      HAL_Delay(10);

      // 0cm로 읽히는 오류(센서 튐) 방지: 0은 매우 먼 거리나 오류이므로 무시
      uint16_t fd = (Front_distance <= 3) ? 999 : Front_distance;
      uint16_t ld = (Left_distance <= 3) ? 999 : Left_distance;
      uint16_t rd = (Right_distance <= 3) ? 999 : Right_distance;

      switch(currentState)
      {
        case STATE_FORWARD:

          if(fd <= DANGER_DIST)
          {
            currentState = STATE_BACKWARD;
            stateStartTime = HAL_GetTick();
          }
          else if(ld <= DANGER_DIST)
          {
            currentState = STATE_TURN_RIGHT;
            stateStartTime = HAL_GetTick();
          }
          else if(rd <= DANGER_DIST)
          {
            currentState = STATE_TURN_LEFT;
            stateStartTime = HAL_GetTick();
          }
               // 정면과 오른쪽이 모두 안전거리 이상인가?
          else if((fd >= RECOVER_DIST) && (rd > (ld + 15)))
          {
            currentState = STATE_TURN_RIGHT;    //오른쪽이 뚫렸다면 우회전
            stateStartTime = HAL_GetTick();
          }
               // 정면과 왼쪽이 모두 안전거리 이상인가?
          else if((fd >= RECOVER_DIST) && (ld > (rd + 15)))
          {
            currentState = STATE_TURN_LEFT;   // 왼쪽이 뚫렸다면 좌회전
            stateStartTime = HAL_GetTick();
          }
             // 3. 그 외 (모두 뚫려있거나 모두 적당한 거리일 때)
          MotorA_Control(BASE_SPEED, DIR_FORWARD);
          MotorB_Control(BASE_SPEED, DIR_FORWARD);
        break;


        case STATE_BACKWARD:

          MotorA_Control(BACK_SPEED, DIR_BACKWARD);
          MotorB_Control(BACK_SPEED, DIR_BACKWARD);

          if (HAL_GetTick() - stateStartTime >= ACTION_TIME)
          {
            // 후진 시간이 끝났으므로 일단 정지
            MotorA_Control(0, DIR_STOP);
            MotorB_Control(0, DIR_STOP);

            if(fd > RECOVER_DIST)    // 전방이 뚫렸다면 직진
            {
              currentState = STATE_FORWARD;
            }
            else    // 전방이 막혀있다면, 좌우 중 더 넓은 쪽으로 회전
            {
              if (ld > rd)
              {
                currentState = STATE_TURN_LEFT;
              }
              else
              {
                currentState = STATE_TURN_RIGHT;
              }
            }

            // 상태가 바뀌었으므로 시간 기록을 초기화 (회전 시간을 재기 위해)
            stateStartTime = HAL_GetTick();
          }
          break;

        case STATE_TURN_LEFT:

          MotorA_Control(TURN_SPEED, DIR_BACKWARD);
          MotorB_Control(TURN_SPEED, DIR_FORWARD);

          // 전방 회피용은 0.5초만 회전
          if(fd <= DANGER_DIST)
          {
            if(HAL_GetTick() - stateStartTime >= ACTION_TIME)
              currentState = STATE_FORWARD;
          }
          else if(rd >= RECOVER_DIST && fd >= RECOVER_DIST)
          {
            currentState = STATE_FORWARD;
          }
          break;

        case STATE_TURN_RIGHT:

          MotorA_Control(TURN_SPEED, DIR_FORWARD);
          MotorB_Control(TURN_SPEED, DIR_BACKWARD);

          if(fd <= DANGER_DIST)
          {
            if(HAL_GetTick() - stateStartTime >= ACTION_TIME)
              currentState = STATE_FORWARD;
          }
          else if(ld >= RECOVER_DIST && fd >= RECOVER_DIST)
          {
            currentState = STATE_FORWARD;
          }
          break;
      }

      printf("State:%d F:%d L:%d R:%d\r\n",
             currentState, fd, ld, rd);
    }
    else
    {
      HAL_Delay(10);
    }
  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
      |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
