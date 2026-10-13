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
#include "fdcan.h"
#include "memorymap.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "m2006_poll.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



char tx_buf[128] = "Hello STM32 UART!\r\n";
uint8_t rx_buf[10]; 
volatile HAL_StatusTypeDef tx_status;

uint8_t can_tx_data[8] = {
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88
};

uint8_t can_rx_data[8] = {0};
FDCAN_RxHeaderTypeDef can_rx_header;

/* 0：未完成；1：通过；2：收到但内容不一致 */
volatile uint8_t can_test_result = 0;


M2006_Feedback motor_feedback;
volatile HAL_StatusTypeDef motor_rx_status;
volatile HAL_StatusTypeDef motor_tx_status;
volatile uint32_t motor_request = 0; /* 1=尝试使能一次；2=停止 */
volatile uint8_t motor_enable_ok = 0;
volatile float motor_target_rpm = 0.0f;

extern volatile uint32_t can_raw_rx_count;
extern volatile uint32_t can_last_rx_id;
int motor_current =0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




//	void CAN_Init(FDCAN_HandleTypeDef *hfdcan)
//	{
//		FDCAN_FilterTypeDef CAN_FilterConfig = {0};
//		HAL_StatusTypeDef HAL_Status;

//		if (hfdcan->Instance == FDCAN1) 
//			{
//				// ---- 配置 FDCAN1 的过滤器 ----
//				CAN_FilterConfig.IdType = FDCAN_STANDARD_ID;              // 标准 ID (11位)
//				CAN_FilterConfig.FilterIndex = 0;                         // 使用第 0 个过滤器
//				CAN_FilterConfig.FilterType = FDCAN_FILTER_MASK;          // 掩码模式
//				CAN_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  // 匹配的帧放入 RX FIFO0
//				CAN_FilterConfig.FilterID1 = 0x000;                       // 期望匹配的 ID (这里写0)
//				CAN_FilterConfig.FilterID2 = 0x000;                       // 掩码 (全0表示放行所有ID)
//				
//				// 应用过滤器配置
//				HAL_Status = HAL_FDCAN_ConfigFilter(hfdcan, &CAN_FilterConfig);
//				if (HAL_Status != HAL_OK) { Error_Handler(); }

//				// 配置全局过滤规则 (拒绝未匹配的帧，拒绝远程帧)
//				HAL_Status = HAL_FDCAN_ConfigGlobalFilter(hfdcan, 
//														  FDCAN_REJECT, FDCAN_REJECT, 
//														  FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
//				if (HAL_Status != HAL_OK) { Error_Handler(); }

//				// 启动 FDCAN1
//				HAL_Status = HAL_FDCAN_Start(hfdcan);
//				if (HAL_Status != HAL_OK) { Error_Handler(); }
//			}
//			
//		 else
//			{
//				// 未知的 CAN 外设
//				Error_Handler();
//			}
//	}

///**
// * @brief  FDCAN 发送数据函数
// * @param  hfdcan: FDCAN 句柄指针（例如 &hfdcan1）
// * @param  pData:  要发送的数据数组指针
// * @param  ID:     标准帧 ID (11位，范围 0x000 ~ 0x7FF)
// * @retval 0: 成功加入发送队列; 2: 发送失败
// */
//uint8_t CAN_SendData(FDCAN_HandleTypeDef *hfdcan, uint8_t *pData, uint16_t ID)
//{
//    HAL_StatusTypeDef HAL_RetVal = HAL_ERROR;
//    FDCAN_TxHeaderTypeDef TxHeader = {0};

//    // 1. 配置发送帧头 (对应经典 CAN 的 TxMessage)
//    TxHeader.Identifier = ID;                         // 设置标准 ID
//    TxHeader.IdType = FDCAN_STANDARD_ID;              // 标准帧 (11位 ID)
//    TxHeader.TxFrameType = FDCAN_DATA_FRAME;          // 数据帧
//    TxHeader.DataLength = FDCAN_DLC_BYTES_8;          // 数据长度 8 字节 (FDCAN 必须用宏)
//    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 错误状态指示 (通常填 ACTIVE)
//    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           // 不切换位速率 (经典 CAN 模式)
//    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            // 经典 CAN 格式 (非 FD)
//    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 不保存发送事件
//    TxHeader.MessageMarker = 0;                       // 消息标记 (用于追踪，这里不用)

//    // 2. 检查发送队列是否已满 (对应经典 CAN 的 GetTxMailboxesFreeLevel)
//    // FDCAN 通常有 3 个发送队列元素，这里检查是否有空位
//    // 如果队列满了，HAL_FDCAN_AddMessageToTxFifoQ 会返回 HAL_ERROR 或 HAL_BUSY
//    // 为了安全，我们可以加一个带超时的等待，防止卡死
//    uint32_t tickstart = HAL_GetTick();
//    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0) 
//    {
//        // 如果队列满了，等待最多 100ms
//        if ((HAL_GetTick() - tickstart) > 100) 
//        {
//            return 2; // 超时，发送失败
//        }
//    }

//    // 3. 将消息加入发送队列 (对应经典 CAN 的 AddTxMessage)
//    HAL_RetVal = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, pData);

//    if (HAL_RetVal != HAL_OK)
//    {
//        return 2; // 发送失败
//    }

//    return 0; // 成功加入发送队列
//}










/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

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
  MX_USART1_UART_Init();
  MX_FDCAN1_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */

// 先全部塞在这里，后面再分。
//FDCAN_HandleTypeDef  hfdcan1


//	CAN_Init(&hfdcan1);

//	if (CAN_SendData(&hfdcan1, can_tx_data, 0x123) != 0)
//	{
//		Error_Handler();
//	}
	if (M2006_Init(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}


   uint32_t control_tick = HAL_GetTick();
	static uint8_t started = 0;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 
	motor_rx_status = M2006_Poll();
	M2006_ReadFeedback(&motor_feedback);
	  
	  
	if (!started && M2006_IsOnline())
    {
        if (M2006_Enable())          /* 在线才使能 */
        {
            started = 1;
        }
    }

    if (started)
    {
		    M2006_Set_I(motor_current);            /* 原始值 300 = 0.3A，小电流试 */	
    }  
	  
	  
	
	sprintf(tx_buf, "%d, %d\r\n", motor_feedback.RotorSpeedRPM,motor_feedback.Encoder );  
	tx_status = HAL_UART_Transmit(&huart8, (uint8_t *)tx_buf, strlen(tx_buf), 100);  
	
	//HAL_Delay(1);
//	uint32_t start_time = HAL_GetTick();

//	/* 等待接收队列中出现一帧 */
//	while (HAL_FDCAN_GetRxFifoFillLevel(
//			   &hfdcan1, FDCAN_RX_FIFO0) == 0U)
//	{
//		if ((uint32_t)(HAL_GetTick() - start_time) >= 100U)
//		{
//			Error_Handler();   // 等了 100 ms 仍然没有收到
//		}
//	}

//	/* 从接收队列取出这一帧 */
//	if (HAL_FDCAN_GetRxMessage(
//			&hfdcan1,
//			FDCAN_RX_FIFO0,
//			&can_rx_header,
//			can_rx_data) != HAL_OK)
//	{
//		Error_Handler();
//	}

//	/* 比较帧类型、ID、长度和数据 */
//	if (can_rx_header.IdType == FDCAN_STANDARD_ID &&
//		can_rx_header.RxFrameType == FDCAN_DATA_FRAME &&
//		can_rx_header.Identifier == 0x123U &&
//		can_rx_header.DataLength == FDCAN_DLC_BYTES_8 &&
//		memcmp(can_tx_data, can_rx_data, 8) == 0)
//	{
//		can_test_result = 1;
//	}
//	else
//	{
//		can_test_result = 2;
//	}
	  
	  //HAL_UART_Receive(&huart8, rx_buf, 10, 10000);  
//	tx_status = HAL_UART_Transmit(&huart8, (uint8_t *)tx_buf, strlen(tx_buf), 100);
//	HAL_Delay(10);
		  
	   
	  
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

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

#ifdef  USE_FULL_ASSERT
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
