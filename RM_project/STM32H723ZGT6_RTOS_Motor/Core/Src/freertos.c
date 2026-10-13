/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "m2006_simple.h"
#include <math.h>
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
/* USER CODE BEGIN Variables */



extern char tx_buf[164];
int  my_cnt  = 0;


extern volatile HAL_StatusTypeDef motor_rx_status;
extern volatile HAL_StatusTypeDef motor_tx_status;
extern volatile uint32_t motor_request;
extern volatile uint8_t motor_enable_ok;
extern volatile float motor_target_rpm;


extern volatile uint32_t can_raw_rx_count;
extern volatile uint32_t can_last_rx_id;
uint8_t cmd;

float motor_current =0;
float speed = 0;
float target_position_deg = 0.0f;


//加减速


/* Output-shaft units: degree, degree/second, degree/second^2. */
volatile float trajectory_target_deg = 0.0f;
volatile float trajectory_degree = 0.0f;  /* Passed to the motor task. */
volatile uint8_t trajectory_start = 0;

float trajectory_vmax = 500.0f;
float trajectory_amax = 180.0f;

float trajectory_v_ref = 0.0f;
uint8_t trajectory_busy = 0;

typedef struct
{
    float degree;       /* Output-shaft degrees. */
    float velocity;     /* Output-shaft degrees/second. */
    uint32_t time_ms;
    uint8_t valid;
} TrajectoryReference;

static TrajectoryReference trajectory_ref = {0};

/* For plotting: target ROTOR rpm sent to the speed PI. */
float motor_speed_cmd = 0.0f;


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Motor_task */
osThreadId_t Motor_taskHandle;
const osThreadAttr_t Motor_task_attributes = {
  .name = "Motor_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LQR_control_Task */
osThreadId_t LQR_control_TaskHandle;
const osThreadAttr_t LQR_control_Task_attributes = {
  .name = "LQR_control_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Motor_control(void *argument);
void LQR_control(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Motor_task */
  Motor_taskHandle = osThreadNew(Motor_control, NULL, &Motor_task_attributes);

  /* creation of LQR_control_Task */
  LQR_control_TaskHandle = osThreadNew(LQR_control, NULL, &LQR_control_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	
	 uint32_t tick = osKernelGetTickCount();
	
	
  for(;;)
  {
//	 snprintf(tx_buf, sizeof(tx_buf),
//		 "<rpm,f,t,pos,t,trq,cur>:%d,%.2f,%.2f,%.2f,%.2f,%d,%.3f\r\n",
//        (int)m2006_simple_feedback.rotor_rpm,
//			m2006_speed_filtered, speed,
//		m2006_position_deg,target_position_deg,           
//        (int)m2006_simple_feedback.torque_raw,
//        motor_current);
	  
	 snprintf(tx_buf, sizeof(tx_buf),
		 "<rpm,f,t,pos,t,trq,cur>:%d,%.2f,%.2f,%.2f,%.2f,%d,%.3f\r\n",
        (int)m2006_simple_feedback.rotor_rpm,
			m2006_speed_filtered, motor_speed_cmd,
		m2006_position_deg,trajectory_degree,           
        (int)m2006_simple_feedback.torque_raw,
        motor_current);	  
	 
	HAL_UART_Transmit(&huart8, (uint8_t *)tx_buf, strlen(tx_buf),1);

    tick += 1;                    // 下一个绝对时间点
    osDelayUntil(tick);
    //osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Motor_control */
/**
* @brief Function implementing the Motor_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_control */
void Motor_control(void *argument)
{
  /* USER CODE BEGIN Motor_control */
  /* Infinite loop */
	

	
     uint32_t tick = osKernelGetTickCount();
	 m2006_simple_pi.kp=15.0f;
	 m2006_simple_pi.ki = 3.0f;
	
  for(;;)
  {
	  
	motor_rx_status = M2006Simple_Poll();	  
	  
	motor_tx_status = M2006Simple_SetSpeedRotorRPM(speed,0.001f,&motor_current);
	  
	//motor_tx_status = M2006Simple_SetPositionDeg(target_position_deg,0.001f);
	//motor_tx_status = M2006Simple_SetPositionDeg(trajectory_degree,0.001f);
	  
	  
//	  TrajectoryReference ref;

//		taskENTER_CRITICAL();
//		ref = trajectory_ref;
//		taskEXIT_CRITICAL();

//		if (ref.valid)
//		{
//			float elapsed_s;
//			float reference_degree;
//			float position_error;
//			float speed_limit = m2006_position_speed_limit;

//			elapsed_s = (HAL_GetTick() - ref.time_ms) * 0.001f;

//			/* Approximate the reference between 50 Hz updates. */
//			if (elapsed_s > 0.020f)
//				elapsed_s = 0.020f;

//			reference_degree = ref.degree + ref.velocity * elapsed_s;

//			position_error = reference_degree - m2006_position_deg;

//			/*
//			 * Output-shaft degrees/second -> rotor rpm: multiply by 6.
//			 * Existing position Kp unit: rotor rpm / output-shaft degree.
//			 */
//			motor_speed_cmd =
//				ref.velocity * 6.0f +
//				m2006_position_kp * position_error;

//			if (motor_speed_cmd > speed_limit)
//				motor_speed_cmd = speed_limit;

//			if (motor_speed_cmd < -speed_limit)
//				motor_speed_cmd = -speed_limit;

//			motor_tx_status = M2006Simple_SetSpeedRotorRPM(
//				motor_speed_cmd, 0.001f, &motor_current);
//		}
//	  
	  
	  
	  
	  
	  
	  
	tick += 1;
    osDelayUntil(tick);
  }
  /* USER CODE END Motor_control */
}

/* USER CODE BEGIN Header_LQR_control */
/**
* @brief Function implementing the LQR_control_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LQR_control */
void LQR_control(void *argument)
{
  /* USER CODE BEGIN LQR_control */
	//uint8_t cmd;
    uint32_t tick = osKernelGetTickCount();
    uint32_t start_ms = 0;

    float q_start = 0.0f;
    float q_end = 0.0f;
    float direction = 1.0f;
    float distance = 0.0f;
    float acceleration = 0.0f;

    float peak_speed = 0.0f;
    float t_acc = 0.0f;
    float t_cruise = 0.0f;
    float t_total = 0.0f;
    float d_acc = 0.0f;

    uint8_t initialized = 0;	
	
  /* Infinite loop */
//  for(;;)
//  {  
//	HAL_UART_Receive(&huart8,&cmd,1,0);  
//	  
//	switch (cmd)
//	{
//	  case '0': target_position_deg =   0.0f; break;
//	  case '1': target_position_deg =  100.0f; break;
//	  case '2': target_position_deg =  200.0f; break;
//	  case '3': target_position_deg =  300.0f; break;
//	  default: break;
//	}
//    osDelay(10);
//  }

    for (;;)
    {
//		
//		if (HAL_UART_Receive(&huart8, &cmd, 1, 0) == HAL_OK)
//		{
//			if (!trajectory_busy)
//			{
//				uint8_t valid_cmd = 1;

//				switch (cmd)
//				{
//					case '0': trajectory_target_deg =    0.0f; break;
//					case '1': trajectory_target_deg = 1800.0f; break;
//					case '2': trajectory_target_deg = 3600.0f; break;
//					case '3': trajectory_target_deg = 7200.0f; break;
//					default:  valid_cmd = 0; break;
//				}

//				if (valid_cmd)
//					trajectory_start = 1;
//			}
//		}
//		
//		  
//        /* The motor task receives feedback and updates actual position. */
//        if (!initialized && m2006_simple_feedback.rx_count > 0)
//        {
//            trajectory_degree = m2006_position_deg;
//            initialized = 1;
//        }
//        if (initialized)
//        {
//            if (trajectory_start)
//            {
//                trajectory_start = 0;

//                /* First version: accept new targets only when idle. */
//                if (!trajectory_busy &&
//                    trajectory_vmax > 0.0f &&
//                    trajectory_amax > 0.0f)
//                {
//                    q_start = m2006_position_deg;
//                    q_end = trajectory_target_deg;
//                    distance = fabsf(q_end - q_start);
//                    direction = (q_end >= q_start) ? 1.0f : -1.0f;
//                    acceleration = trajectory_amax;

//                    if (distance >=
//                        trajectory_vmax * trajectory_vmax / acceleration)
//                    {
//                        peak_speed = trajectory_vmax;
//                        t_acc = peak_speed / acceleration;
//                        t_cruise =
//                            (distance - peak_speed * peak_speed / acceleration)
//                            / peak_speed;
//                    }
//                    else
//                    {
//                        peak_speed = sqrtf(distance * acceleration);
//                        t_acc = peak_speed / acceleration;
//                        t_cruise = 0.0f;
//                    }

//                    d_acc = 0.5f * acceleration * t_acc * t_acc;
//                    t_total = 2.0f * t_acc + t_cruise;

//                    start_ms = HAL_GetTick();
//                    trajectory_busy = 1;
//					
//                }
//            }

//            if (trajectory_busy)
//            {
//                float t = (HAL_GetTick() - start_ms) * 0.001f;
//                float displacement;
//                float velocity;

//                if (t >= t_total)
//                {
//                    trajectory_degree = q_end;
//                    trajectory_v_ref = 0.0f;
//                    trajectory_busy = 0;
//                }
//                else
//                {
//                    if (t < t_acc)
//                    {
//                        velocity = acceleration * t;
//                        displacement = 0.5f * acceleration * t * t;
//                    }
//                    else if (t < t_acc + t_cruise)
//                    {
//                        velocity = peak_speed;
//                        displacement = d_acc + peak_speed * (t - t_acc);
//                    }
//                    else
//                    {
//                        float remaining = t_total - t;

//                        velocity = acceleration * remaining;
//                        displacement =
//                            distance - 0.5f * acceleration *
//                            remaining * remaining;
//                    }

//                    trajectory_degree = q_start + direction * displacement;
//                    trajectory_v_ref = direction * velocity;
//                }
//            }
//			
//					taskENTER_CRITICAL();

//					trajectory_ref.degree = trajectory_degree;
//					trajectory_ref.velocity = trajectory_v_ref;
//					trajectory_ref.time_ms = HAL_GetTick();
//					trajectory_ref.valid = 1;

//					taskEXIT_CRITICAL();
//			
//			
//       }
		
        tick += 20;       /* 50 Hz, assuming a 1000 Hz kernel tick. */
        osDelayUntil(tick);
    }



  /* USER CODE END LQR_control */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

