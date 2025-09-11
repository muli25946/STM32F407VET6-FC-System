/**
 * @file fc_sys.h
 * @author Muli25946 (2687025869@qq.com)
 * @brief
 * 作为驱动+模块和任务的中间层，向上层提供任务函数和类型。主要分为数据采集，姿态解算，串口发数据/遥控，pid控制，接收数据解析5个任务。
 * 该文件期望将所有的全局变量和任务流程控制留在此文件内，以保证上层的整洁
 * @version 0.1
 * @date 2025-08-23
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef FC_SYS_H
#define FC_SYS_H

/*驱动层*/
#include "ist8310.h"
#include "motor.h"
#include "mpu6050.h"
#include "nrf24l01.h"
#include "oled.h"


/*模块层*/
#include "FreeRTOS.h"
#include "mahony.h"
#include "pid.h"
#include "task.h"
#include "vofa.h"

/*调试模式控制，用于调参*/
typedef enum {
  RELEASE,
  ONLY_PITCH_RATE,
  ONLY_ROLL_RATE,
  ONLY_YAW_RATE,
  ONLT_PITCH,
  ONLY_ROLL,
  ONLY_YAW,
} SysDebugMode;

/*传感器原始数据队列前结构体*/
typedef struct {
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
} SensorData_t;

/*Mahony解算结果数据队列前结构体*/
typedef struct {
  SensorData_t sensordata;
  float mahony_pitch, mahony_roll, mahony_yaw;
} AttitudeData_t;

/*串口发送数据对列前结构体*/
typedef struct {
  AttitudeData_t attitude;
  float pidPitchCmd, pidRollCmd, pidYawCmd; // 存储pid 计算结果
  RemoteData_t remote;
} TelemetryData_t;

extern SensorData_t sensorData;
extern AttitudeData_t attitudeData;
extern RemoteData_t remoteData;
extern TelemetryData_t telemetryData;

extern VofaRxFloatFrame vofaRxFrame;

void FCSysInit(void);
void task_SensorGetData(void);
void task_CalPosture(float dt);
void task_Control(float dt);
void task_VofaTx(void);
void task_VofaRxDecode(void);
void task_NrfLoadData(void);

#endif
