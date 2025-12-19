/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fc_sys.h"
#include "queue.h"
#include "usart.h"
#include "vofa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REMOTE_NRF // 指定遥控设备REMOTE_NRF/REMOTE_UART
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for xSensorTask */
osThreadId_t xSensorTaskHandle;
const osThreadAttr_t xSensorTask_attributes = {
    .name = "xSensorTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};
/* Definitions for xAttitudeTask */
osThreadId_t xAttitudeTaskHandle;
const osThreadAttr_t xAttitudeTask_attributes = {
    .name = "xAttitudeTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for xControlTask */
osThreadId_t xControlTaskHandle;
const osThreadAttr_t xControlTask_attributes = {
    .name = "xControlTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal7,
};
/* Definitions for xUartTxTask */
osThreadId_t xUartTxTaskHandle;
const osThreadAttr_t xUartTxTask_attributes = {
    .name = "xUartTxTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal6,
};
/* Definitions for xRemoteTask */
osThreadId_t xRemoteTaskHandle;
const osThreadAttr_t xRemoteTask_attributes = {
    .name = "xRemoteTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal2,
};
/* Definitions for xNrfReceiveTask */
osThreadId_t xNrfReceiveTaskHandle;
const osThreadAttr_t xNrfReceiveTask_attributes = {
    .name = "xNrfReceiveTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal2,
};
/* Definitions for xSensorDataQueue */
osMessageQueueId_t xSensorDataQueueHandle;
const osMessageQueueAttr_t xSensorDataQueue_attributes = {
    .name = "xSensorDataQueue"};
/* Definitions for xAttitudeDataQueue */
osMessageQueueId_t xAttitudeDataQueueHandle;
const osMessageQueueAttr_t xAttitudeDataQueue_attributes = {
    .name = "xAttitudeDataQueue"};
/* Definitions for xRemoteDataQueue */
osMessageQueueId_t xRemoteDataQueueHandle;
const osMessageQueueAttr_t xRemoteDataQueue_attributes = {
    .name = "xRemoteDataQueue"};
/* Definitions for xTelemetryDataQueue */
osMessageQueueId_t xTelemetryDataQueueHandle;
const osMessageQueueAttr_t xTelemetryDataQueue_attributes = {
    .name = "xTelemetryDataQueue"};
/* Definitions for xRemoteDataMutex */
osMutexId_t xRemoteDataMutexHandle;
const osMutexAttr_t xRemoteDataMutex_attributes = {.name = "xRemoteDataMutex"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void xSensorTaskFunction(void *argument);
void xAttitudeTaskFunction(void *argument);
void xControlTaskFunction(void *argument);
void xUartTxTaskFunction(void *argument);
void xRemoteTaskFunction(void *argument);
void xNrfReceiveTaskFunction(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of xRemoteDataMutex */
  xRemoteDataMutexHandle = osMutexNew(&xRemoteDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xSensorDataQueue */
  xSensorDataQueueHandle =
      osMessageQueueNew(1, sizeof(SensorData_t), &xSensorDataQueue_attributes);

  /* creation of xAttitudeDataQueue */
  xAttitudeDataQueueHandle = osMessageQueueNew(1, sizeof(AttitudeData_t),
                                               &xAttitudeDataQueue_attributes);

  /* creation of xRemoteDataQueue */
  xRemoteDataQueueHandle =
      osMessageQueueNew(1, sizeof(RemoteData_t), &xRemoteDataQueue_attributes);

  /* creation of xTelemetryDataQueue */
  xTelemetryDataQueueHandle = osMessageQueueNew(
      1, sizeof(TelemetryData_t), &xTelemetryDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of xSensorTask */
  xSensorTaskHandle =
      osThreadNew(xSensorTaskFunction, NULL, &xSensorTask_attributes);

  /* creation of xAttitudeTask */
  xAttitudeTaskHandle =
      osThreadNew(xAttitudeTaskFunction, NULL, &xAttitudeTask_attributes);

  /* creation of xControlTask */
  xControlTaskHandle =
      osThreadNew(xControlTaskFunction, NULL, &xControlTask_attributes);

  /* creation of xUartTxTask */
  xUartTxTaskHandle =
      osThreadNew(xUartTxTaskFunction, NULL, &xUartTxTask_attributes);

  /* creation of xRemoteTask */
  xRemoteTaskHandle =
      osThreadNew(xRemoteTaskFunction, NULL, &xRemoteTask_attributes);

  /* creation of xNrfReceiveTask */
  xNrfReceiveTaskHandle =
      osThreadNew(xNrfReceiveTaskFunction, NULL, &xNrfReceiveTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_xSensorTaskFunction */
/**
 * @brief  Function implementing the xSensorTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_xSensorTaskFunction */
void xSensorTaskFunction(void *argument) {
  /* USER CODE BEGIN xSensorTaskFunction */
  const TickType_t xDelay = pdMS_TO_TICKS(2); // 2ms周期
  /* Infinite loop */
  for (;;) {
    task_SensorGetData();
    xQueueOverwrite(xSensorDataQueueHandle, &sensorData);
    vTaskDelay(xDelay);
  }
  /* USER CODE END xSensorTaskFunction */
}

/* USER CODE BEGIN Header_xAttitudeTaskFunction */
/**
 * @brief Function implementing the xAttitudeTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_xAttitudeTaskFunction */
void xAttitudeTaskFunction(void *argument) {
  /* USER CODE BEGIN xAttitudeTaskFunction */
  /* Infinite loop */
  for (;;) {
    if (xQueueReceive(xSensorDataQueueHandle, &sensorData, portMAX_DELAY) ==
        pdPASS) {

      task_CalPosture(125);

      // 继承传感器数据
      attitudeData.sensordata = sensorData;
      // 写入队列
      xQueueOverwrite(xAttitudeDataQueueHandle, &attitudeData);
    }
  }
  /* USER CODE END xAttitudeTaskFunction */
}

/* USER CODE BEGIN Header_xControlTaskFunction */
/**
 * @brief Function implementing the xControlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_xControlTaskFunction */
void xControlTaskFunction(void *argument) {
  /* USER CODE BEGIN xControlTaskFunction */
  TickType_t pidTimeStampLast = xTaskGetTickCount(); // 获取系统节拍数，1节拍/ms
  float pidDT = 0;
  /* Infinite loop */
  for (;;) {
    // 等待姿态数据
    if (xQueueReceive(xAttitudeDataQueueHandle, &attitudeData, portMAX_DELAY) ==
        pdPASS) {

      // 翻转停机
      if ((attitudeData.mahony_pitch > 45 || attitudeData.mahony_pitch < -45) ||
          (attitudeData.mahony_roll > 45 || attitudeData.mahony_roll < -45)) {
        MotorSet(0, 0, 0, 0);
      } else {
        // 计算实际间隔时间dt
        pidDT = (xTaskGetTickCount() - pidTimeStampLast) / 1000.0f;

        // 获取锁
        if (osMutexAcquire(xRemoteDataMutexHandle, 10) == 0) {

          // 控制任务
          task_Control(pidDT);

          // 释放锁
          osMutexRelease(xRemoteDataMutexHandle);
        }

        // 更新旧值
        pidTimeStampLast = xTaskGetTickCount();
      }

      // 继承当前数据
      telemetryData.attitude = attitudeData;
      telemetryData.remote = remoteData;
      // 打包数据放入回传队列
      xQueueOverwrite(xTelemetryDataQueueHandle, &telemetryData);
    }
  }
  /* USER CODE END xControlTaskFunction */
}

/* USER CODE BEGIN Header_xUartTxTaskFunction */
/**
 * @brief Function implementing the xUartTxTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_xUartTxTaskFunction */
void xUartTxTaskFunction(void *argument) {
  /* USER CODE BEGIN xUartTxTaskFunction */
  /* Infinite loop */
  for (;;) {
    // 回传数据队列接收数据
    if (xQueueReceive(xTelemetryDataQueueHandle, &telemetryData,
                      portMAX_DELAY) == pdPASS) {
      task_VofaTx();
    }
  }
  /* USER CODE END xUartTxTaskFunction */
}

/* USER CODE BEGIN Header_xRemoteTaskFunction */
/**
 * @brief Function implementing the xRemoteTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_xRemoteTaskFunction */
void xRemoteTaskFunction(void *argument) {
  /* USER CODE BEGIN xRemoteTaskFunction */
  /* Infinite loop */
  // 等待中断的通知，并判断该解析任务优先级是否高于当前运行任务优先级
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  // 等待锁。因为赋值和读取均操作同一个变量。释放了才能修改
  if (osMutexAcquire(xRemoteDataMutexHandle, portMAX_DELAY)) {
    // 串口dma接收后开始对原始数据解码。解码后的数据直接传入remoteData
    task_VofaRxDecode();

    // 解锁
    osMutexRelease(xRemoteDataMutexHandle);
  }

  // 控制数据装入队列
  xQueueOverwrite(xRemoteDataQueueHandle, &remoteData);

  // 重新开启接收中断
  HAL_UART_Receive_DMA(&huart2, rxVofaBuff, 6);
  /* USER CODE END xRemoteTaskFunction */
}

/* USER CODE BEGIN Header_xNrfReceiveTaskFunction */
/**
 * @brief Function implementing the xNrfReceiveTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_xNrfReceiveTaskFunction */
void xNrfReceiveTaskFunction(void *argument) {
  /* USER CODE BEGIN xNrfReceiveTaskFunction */
  /* Infinite loop */
  for (;;) {
    // 等待中断的通知，并判断该解析任务优先级是否高于当前运行任务优先级
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // 等待锁。因为赋值和读取均操作同一个变量。释放了才能修改
    if (osMutexAcquire(xRemoteDataMutexHandle, portMAX_DELAY)) {

      // 查看是否接收数据并转移
      task_NrfLoadData();

      // 解锁
      osMutexRelease(xRemoteDataMutexHandle);
    }

    // 控制数据装入队列
    xQueueOverwrite(xRemoteDataQueueHandle, &remoteData);
  }
  /* USER CODE END xNrfReceiveTaskFunction */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

#ifdef REMOTE_UART
/*串口dma接收中断*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // 通知解码任务
    vTaskNotifyGiveFromISR(xRemoteTaskHandle, &xHigherPriorityTaskWoken);
  }
}
#elif defined(REMOTE_NRF)
/*gpio外部中断*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == NRF_IRQ_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // 通知解码任务
    vTaskNotifyGiveFromISR(xNrfReceiveTaskHandle, &xHigherPriorityTaskWoken);
  }
}
#endif

/* USER CODE END Application */
