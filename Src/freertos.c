/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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


#include "led_flow_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId testHandle;
osThreadId LaserTaskHandle;
osThreadId FricTaskHandle;
osThreadId LedFlowTaskHandle;
osThreadId GimbalTaskHandle;
osThreadId GimbalLockedTasHandle;
osThreadId TranslateTaskHandle;
osThreadId AdvanceTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
  
/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);
void laser_task(void const * argument);
void fric_task(void const * argument);
void led_RGB_flow_task(void const * argument);
void gimbal_task(void const * argument);
void gimbal_locked_task(void const * argument);
void translate_task(void const * argument);
void advance_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* definition and creation of test */
  osThreadDef(test, test_task, osPriorityBelowNormal, 0, 128);
  testHandle = osThreadCreate(osThread(test), NULL);

  /* definition and creation of LaserTask */
  osThreadDef(LaserTask, laser_task, osPriorityNormal, 0, 128);
  LaserTaskHandle = osThreadCreate(osThread(LaserTask), NULL);

  /* definition and creation of FricTask */
  osThreadDef(FricTask, fric_task, osPriorityNormal, 0, 128);
  FricTaskHandle = osThreadCreate(osThread(FricTask), NULL);

  /* definition and creation of LedFlowTask */
  osThreadDef(LedFlowTask, led_RGB_flow_task, osPriorityNormal, 0, 128);
  LedFlowTaskHandle = osThreadCreate(osThread(LedFlowTask), NULL);

  /* definition and creation of GimbalTask */
  osThreadDef(GimbalTask, gimbal_task, osPriorityNormal, 0, 128);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

  /* definition and creation of GimbalLockedTas */
  osThreadDef(GimbalLockedTas, gimbal_locked_task, osPriorityNormal, 0, 128);
  GimbalLockedTasHandle = osThreadCreate(osThread(GimbalLockedTas), NULL);

  /* definition and creation of TranslateTask */
  osThreadDef(TranslateTask, translate_task, osPriorityNormal, 0, 128);
  TranslateTaskHandle = osThreadCreate(osThread(TranslateTask), NULL);

  /* definition and creation of AdvanceTask */
  osThreadDef(AdvanceTask, advance_task, osPriorityIdle, 0, 128);
  AdvanceTaskHandle = osThreadCreate(osThread(AdvanceTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{

  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_task */
}

/* USER CODE BEGIN Header_laser_task */
/**
* @brief Function implementing the LaserTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_laser_task */
__weak void laser_task(void const * argument)
{
  /* USER CODE BEGIN laser_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END laser_task */
}

/* USER CODE BEGIN Header_fric_task */
/**
* @brief Function implementing the FricTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fric_task */
__weak void fric_task(void const * argument)
{
  /* USER CODE BEGIN fric_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END fric_task */
}

/* USER CODE BEGIN Header_led_RGB_flow_task */
/**
* @brief Function implementing the LedFlowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_RGB_flow_task */
__weak void led_RGB_flow_task(void const * argument)
{
  /* USER CODE BEGIN led_RGB_flow_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_RGB_flow_task */
}

/* USER CODE BEGIN Header_gimbal_task */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}

/* USER CODE BEGIN Header_gimbal_locked_task */
/**
* @brief Function implementing the GimbalLockedTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_locked_task */
__weak void gimbal_locked_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_locked_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_locked_task */
}

/* USER CODE BEGIN Header_translate_task */
/**
* @brief Function implementing the TranslateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_translate_task */
__weak void translate_task(void const * argument)
{
  /* USER CODE BEGIN translate_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END translate_task */
}

/* USER CODE BEGIN Header_advance_task */
/**
* @brief Function implementing the AdvanceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_advance_task */
__weak void advance_task(void const * argument)
{
  /* USER CODE BEGIN advance_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END advance_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
