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
osThreadId ServoTaskHandle;
osThreadId GpioTaskHandle;
osThreadId FricTaskHandle;
osThreadId RunnerTaskHandle;
osThreadId LedFlowTaskHandle;
osThreadId GimbalTaskHandle;
osThreadId GimbalLockedTasHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);
void servo_task(void const * argument);
void gpio_task(void const * argument);
void fric_task(void const * argument);
void runner_task(void const * argument);
void led_RGB_flow_task(void const * argument);
void gimbal_task(void const * argument);
void gimbal_locked_task(void const * argument);

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

  /* definition and creation of ServoTask */
  osThreadDef(ServoTask, servo_task, osPriorityNormal, 0, 128);
  ServoTaskHandle = osThreadCreate(osThread(ServoTask), NULL);

  /* definition and creation of GpioTask */
  osThreadDef(GpioTask, gpio_task, osPriorityNormal, 0, 128);
  GpioTaskHandle = osThreadCreate(osThread(GpioTask), NULL);

  /* definition and creation of FricTask */
  osThreadDef(FricTask, fric_task, osPriorityNormal, 0, 128);
  FricTaskHandle = osThreadCreate(osThread(FricTask), NULL);

  /* definition and creation of RunnerTask */
  osThreadDef(RunnerTask, runner_task, osPriorityNormal, 0, 128);
  RunnerTaskHandle = osThreadCreate(osThread(RunnerTask), NULL);

  /* definition and creation of LedFlowTask */
  osThreadDef(LedFlowTask, led_RGB_flow_task, osPriorityNormal, 0, 128);
  LedFlowTaskHandle = osThreadCreate(osThread(LedFlowTask), NULL);

  /* definition and creation of GimbalTask */
  osThreadDef(GimbalTask, gimbal_task, osPriorityNormal, 0, 128);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

  /* definition and creation of GimbalLockedTas */
  osThreadDef(GimbalLockedTas, gimbal_locked_task, osPriorityIdle, 0, 128);
  GimbalLockedTasHandle = osThreadCreate(osThread(GimbalLockedTas), NULL);

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

/* USER CODE BEGIN Header_servo_task */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo_task */
__weak void servo_task(void const * argument)
{
  /* USER CODE BEGIN servo_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END servo_task */
}

/* USER CODE BEGIN Header_gpio_task */
/**
* @brief Function implementing the GpioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gpio_task */
__weak void gpio_task(void const * argument)
{
  /* USER CODE BEGIN gpio_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gpio_task */
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

/* USER CODE BEGIN Header_runner_task */
/**
* @brief Function implementing the RunnerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runner_task */
__weak void runner_task(void const * argument)
{
  /* USER CODE BEGIN runner_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END runner_task */
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
