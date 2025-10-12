/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "pid.h"
#include "Caculate.h"
#include "DJI.h"
#include "wtr_can.h"
#include "encoder.h"
//#include "bsp_can.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATAS_LEN 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// PID_TypeDef motor_pid[4];
int32_t set_spd = 0;
// static int key_sta = 0;
// int speed_step_sign = +1;

// uint16_t TIM_COUNT[2];
// #define SpeedStep 500
  uint8_t rxBuffer[DATAS_LEN] = {0};  //串口接收的数据缓冲区，最后一字节为\n
  int32_t errorNum = 0;
  uint8_t dataReady = 0;

  Motor_t hmotor[4] ={ {{&htim9,TIM_CHANNEL_1},&htim2,{AIN1_GPIO_Port,AIN1_Pin},{AIN2_GPIO_Port,AIN2_Pin},0,0,0,0},
                      {{&htim9,TIM_CHANNEL_2},&htim3,{BIN1_GPIO_Port,BIN1_Pin},{BIN2_GPIO_Port,BIN2_Pin},0,0,0,0},
                      {{&htim12,TIM_CHANNEL_1},&htim4,{CIN1_GPIO_Port,CIN1_Pin},{CIN2_GPIO_Port,CIN2_Pin},0,0,0,0},
                      {{&htim12,TIM_CHANNEL_2},&htim5,{DIN1_GPIO_Port,DIN1_Pin},{DIN2_GPIO_Port,DIN2_Pin},0,0,0,0}};
  PID_t hpid_speed[4];
  PID_t hpid_position[4];
  float motor_out_posi = 0;
  float motor_out_speed = 0;

  uint8_t jumpFlag = 0; //起跳的标志位
  uint8_t runFlag = 0;  //移动标志位，为0，不移动；为1，向前移动
  int counter = 0;  //起跳时间，counter*10ms
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}   //使用huart1可以正常输出，但是使用huart2不能正常输出

//用于蓝牙通信
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
  if (huart == &huart1){
    
    //HAL_UART_Transmit(&huart1,rxBuffer,DATAS_LEN,0xFFFF);
    //printf("into rx\n");
    if (rxBuffer[0] == 0xAA && rxBuffer[DATAS_LEN-2] == 0xBB){
      if (rxBuffer[1] == 0x11){
        runFlag = 1;  //移动
        printf("run\n");
      }
      else {
        runFlag = 0;  //不移动
        printf("stop\n");
      }

      if (rxBuffer[2] == 0xFF){
        jumpFlag = 1; //跳越
        counter = 200 ;
        printf("jump\n");
      }
      else{
        jumpFlag = 0; //不跳越
      }
    }

    HAL_UART_Receive_IT(&huart1,(uint8_t*)rxBuffer,DATAS_LEN);
  }
}

/**
 * @brief  错误中断回调
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    jumpFlag = 0;
    runFlag = 0;
    // 清除错误标志
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
    // 重启接收
    HAL_UART_Receive_IT(&huart1,(uint8_t*)rxBuffer,DATAS_LEN);
    printf("error\n");
  }
}

void CAN_Init()
{
    const CAN_FilterTypeDef sFilterConfig = {.FilterIdHigh         = 0x0000,
                                             .FilterIdLow          = 0x0000,
                                             .FilterMaskIdHigh     = 0x0000,
                                             .FilterMaskIdLow      = 0x0000,
                                             .FilterFIFOAssignment = CAN_FILTER_FIFO0,
                                             .FilterBank           = 0,
                                             .FilterMode           = CAN_FILTERMODE_IDMASK,
                                             .FilterScale          = CAN_FILTERSCALE_32BIT,
                                             .FilterActivation     = ENABLE,
                                             .SlaveStartFilterBank = 14};
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    //HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
      Error_Handler();
    }

    // 启动中断接收，监听 FIFO 0
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
      Error_Handler();
    }
}
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // if (HAL_CAN_Start(&hcan1) != HAL_OK)
  // {
  //     Error_Handler();
  // }

  // // 启动中断接收，监听 FIFO 0
  // if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  // {
  //     Error_Handler();
  // }
  //HAL_CAN_Receive_IT(&hcan1, CAN_RX_FIFO0);   //启动CAN接收中断
  /* 麦轮电机初始化*/
  for (int i=0;i<4;i++){
    PID_Init(&hpid_speed[i],
          2.81364 / MAX_SPEED, 0.83787 / MAX_SPEED, 0.68851 / MAX_SPEED, 0.45070, 0.39677,
          3.04182 / MAX_SPEED, 0.71963 / MAX_SPEED, 0.09108 / MAX_SPEED, 0.00000, 0.74526, 0,
          1, 20, 0);
    PID_Init(&hpid_position[i],
          1.72742, 0.14087, 0.35868, 0.57931, 0.51708,
          2.47420, 0.02883, 0.22799, 0.60006, 0.60421, 0.02,
          1, 2, 0);
    Motor_Start(&hmotor[i]);
    Motor_SetSpeed(&hmotor[i], 1);
    Encoder_Start(&hmotor[i]);
  }

  HAL_TIM_Base_Start_IT(&htim6);  //用于编码器数据处理和控制麦轮电机
  for (int i=0;i<8;i++){
    hDJI[i].motorType = M2006;
  }
  DJI_Init();
  // printf("1\n");
  //CANFilterInit(&hcan1);
  // printf("2\n");
  // ENCODER_CANFilterInit(&hcan1);
  // printf("3\n");
  // HAL_CAN_Start(&hcan1);
  // printf("4\n");
  // HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  CAN_Init();
  //my_can_filter_init_recv_all(&hcan1);     //配置CAN过滤器
  //HAL_CAN_Start(&hcan1);
  //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);   //启动CAN接收中断
  HAL_TIM_Base_Start_IT(&htim6);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE); //清除可能残余的标志位
  HAL_UART_Receive_IT(&huart1, (uint8_t*)rxBuffer, DATAS_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // speedServo(100,&hDJI[0]);
    // set_moto_current(&hcan1, hDJI[0].speedPID.output,   //将PID的计算结果通过CAN发送到电机
    //                     hDJI[0].speedPID.output,
    //                     hDJI[0].speedPID.output,
    //                     hDJI[0].speedPID.output);
    // //CanTransmit_DJI_1234(&hcan1, hDJI[0].speedPID.output, hDJI[1].speedPID.output,hDJI[2].speedPID.output, hDJI[3].speedPID.output);
    //printf("%d,%d\n",(int)(hDJI[0].FdbData.rpm), (int)(hDJI[0].speedPID.ref));
    // HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM6){
    if (jumpFlag == 1){   //跳越
      counter--;
      if (counter <= 0){
        jumpFlag = 0;
      }
      //DJI2006电机控制
      //hDJI[0].FdbData.rpm = moto_chassis[0].speed_rpm;
      //hDJI[0].FdbData.rpm = encoderData.angularSpeed;
      speedServo(500,&hDJI[0]);
      speedServo(500,&hDJI[1]);
      speedServo(-500,&hDJI[2]);
      //positionServo(180, &hDJI[0]);
      // set_moto_current(&hcan1, hDJI[0].speedPID.output,   //将PID的计算结果通过CAN发送到电机
      //                     hDJI[1].speedPID.output,
      //                     hDJI[2].speedPID.output,
      //                     hDJI[3].speedPID.output);
      CanTransmit_DJI_1234(&hcan1, hDJI[0].speedPID.output, hDJI[1].speedPID.output,hDJI[2].speedPID.output, hDJI[3].speedPID.output);
      //printf("%d,%d\n",(int)(hDJI[0].FdbData.rpm), (int)(hDJI[0].speedPID.ref));
      //printf("%d,%d\n",(int)(hDJI[0].posPID.fdb), (int)(hDJI[0].posPID.ref));
    }
    else {  //不跳越
      CanTransmit_DJI_1234(&hcan1,0,0,0,0);
    }

    // //DJI2006电机控制
    // //hDJI[0].FdbData.rpm = moto_chassis[0].speed_rpm;
    // //hDJI[0].FdbData.rpm = encoderData.angularSpeed;
    // speedServo(200,&hDJI[0]);
    // //positionServo(180, &hDJI[0]);
    // // set_moto_current(&hcan1, hDJI[0].speedPID.output,   //将PID的计算结果通过CAN发送到电机
    // //                     hDJI[1].speedPID.output,
    // //                     hDJI[2].speedPID.output,
    // //                     hDJI[3].speedPID.output);
    // CanTransmit_DJI_1234(&hcan1, hDJI[0].speedPID.output, hDJI[1].speedPID.output,hDJI[2].speedPID.output, hDJI[3].speedPID.output);
    // printf("%d,%d\n",(int)(hDJI[0].FdbData.rpm), (int)(hDJI[0].speedPID.ref));
    // //printf("%d,%d\n",(int)(hDJI[0].posPID.fdb), (int)(hDJI[0].posPID.ref));

    if (runFlag == 1){  //前进
      //运动电机控制
      for (int i=0;i<2;i++){
        hpid_speed[i].target = 500;
        Encoder_Progress(&hmotor[i]);
        // motor_out_posi = PID_Calculate(&hpid_position,hmotor.real_round);
        // hpid_speed.target = motor_out_posi;
        motor_out_speed = PID_Calculate(&hpid_speed[i],hmotor[i].real_speed);
        //printf("%d,%d\n",(int)(hmotor.real_speed*100),(int)hpid_speed.target*100);
        Motor_SetSpeed(&hmotor[i],motor_out_speed);
      }
      for (int i=2;i<4;i++){
        hpid_speed[i].target = -100;
        Encoder_Progress(&hmotor[i]);
        // motor_out_posi = PID_Calculate(&hpid_position,hmotor.real_round);
        // hpid_speed.target = motor_out_posi;
        motor_out_speed = PID_Calculate(&hpid_speed[i],hmotor[i].real_speed);
        //printf("%d,%d\n",(int)(hmotor.real_speed*100),(int)hpid_speed.target*100);
        Motor_SetSpeed(&hmotor[i],motor_out_speed);
      }
    }
    else {  //  停止
      for (int i=0;i<4;i++){
        Motor_SetSpeed(&hmotor[i],0);
      }
    }
    // //运动电机控制
    // for (int i=0;i<2;i++){
    //   hpid_speed[i].target = 100;
    //   Encoder_Progress(&hmotor[i]);
    //   // motor_out_posi = PID_Calculate(&hpid_position,hmotor.real_round);
    //   // hpid_speed.target = motor_out_posi;
    //   motor_out_speed = PID_Calculate(&hpid_speed[i],hmotor[i].real_speed);
    //   //printf("%d,%d\n",(int)(hmotor.real_speed*100),(int)hpid_speed.target*100);
    //   Motor_SetSpeed(&hmotor[i],motor_out_speed);
    // }
    // for (int i=2;i<4;i++){
    //   hpid_speed[i].target = -100;
    //   Encoder_Progress(&hmotor[i]);
    //   // motor_out_posi = PID_Calculate(&hpid_position,hmotor.real_round);
    //   // hpid_speed.target = motor_out_posi;
    //   motor_out_speed = PID_Calculate(&hpid_speed[i],hmotor[i].real_speed);
    //   //printf("%d,%d\n",(int)(hmotor.real_speed*100),(int)hpid_speed.target*100);
    //   Motor_SetSpeed(&hmotor[i],motor_out_speed);
    // }
    //printf("run\n");
  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
