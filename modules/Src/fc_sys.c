/*
2逆 X 1顺
   口  Y
4顺   3逆
*/

/*依赖*/
#include "ist8310_bsp.h"
#include "nrf24l01_bsp.h"
#include "vofa.h"
/*HAL*/
#include "cmsis_os2.h"
#include "fc_sys.h"
#include "main.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>

#define NRFREMOTE // 选择遥控方式:UARTREMOTE/NRFREMOTE;同时只能使用一个
#define ONLY_PITCH_RATE

float vofaBuffer[12]; // vofa发送数据前用到的临时存放数组

/*设备对象*/
mpu6050_struct mpu6050;
NRF24L01ObjectType nrf24;
IST8310ObjectType ist8310;
/*通信数据*/
VofaRxFloatFrame vofaRxFrame;
VofaTxFloatFrame vofaTxFrame = {.pFloat = vofaBuffer};
/*控制数据*/
PIDControllerType pidPitchRate, pidRollRate, pidYawRate; // pid角度环控制器
PIDControllerType pidPitch, pidRoll, pidYaw;             // pid角度控制器
PIDOutType pidRes;       // PID计算得到的占空比存放结构体
PIDTargetType pidTarget; // 期望PID达到的目标值
MahonyType mahony;       // mahony计算的结果
/*数据传递通道*/
SensorData_t sensorData; // 用于将传感器数据装入队列的结构体,定义在fc_sys.c中
AttitudeData_t attitudeData; // 用于将姿态数据装入队列的结构体,定义在fc_sys.c中
RemoteData_t remoteData; // 用于将遥控接收数据装入队列的结构体,定义在fc_sys.c中
TelemetryData_t
    telemetryData; // 用于将要回传的数据装入队列的结构体,定义在fc_sys.c中

/**
 * @brief 初始化飞控系统
 *
 */
void FCSysInit(void) {
  MPU6050_Init();
  IST8310ObjectInit(&ist8310, BspIST8310ReadBuf, BspIST8310WriteBuf,
                    BspIST8310Delayms);
  NRF24L01Initialization(&nrf24, NRF24ReadWriteByte, NRF24ChipSelect,
                         NRF24ChipEnable, NRF24GetIRQ, NRF24Delayms);

  MahonyInit();
  // OLED_Init();
  MotorInit();
  PID_Init(&pidPitch, 1, 0, 0, 200);
  PID_Init(&pidPitchRate, 6, 0.024, 1.5, 350);

#if defined(UARTREMOTE)
  HAL_UART_Receive_DMA(&huart2, rxVofaBuff, 6);
#elif defined(NRFREMOTE)
// 已在gpio.c中使能中断通道
#endif
}

/**
 * @brief 传感器进行数据采集,同时装入队列
 *
 */
void task_SensorGetData(void) {
  MPU6050GetRedirectValue(&mpu6050);
  IST8310_GET_MEG_VAL(&ist8310);
  sensorData.acc_x = mpu6050.redirectAcc.x;
  sensorData.acc_y = mpu6050.redirectAcc.y;
  sensorData.acc_z = mpu6050.redirectAcc.z;
  sensorData.gyro_x = mpu6050.redirectGyro.x;
  sensorData.gyro_y = mpu6050.redirectGyro.y;
  sensorData.gyro_z = mpu6050.redirectGyro.z;
  sensorData.mag_x = ist8310.data.redirect_x;
  sensorData.mag_y = ist8310.data.redirect_y;
  sensorData.mag_z = ist8310.data.redirect_z;
}

/**
 * @brief 计算出姿态并入队
 *
 */
void task_CalPosture(float dt) {
  MahonyGetEuler(dt, &mpu6050, &ist8310, &mahony);
  attitudeData.mahony_pitch = mahony.FRD.pitch;
  attitudeData.mahony_roll = mahony.FRD.roll;
  attitudeData.mahony_yaw = mahony.FRD.yaw;
}

/**
 * @brief
 * 从xRemoteDataQueueHandle队列更新参数，判断控制状态计算pid并控制pwm输出。最后打包pid计算数据以准备回传给上位机
 *
 */
void task_Control(float dt) {
  /*接收控制数据，可以是vofa，也可以是遥控数据*/
  pidTarget.targetPitch = remoteData.target;
  pidTarget.baseThrottle = remoteData.remote_throttle;

  /*模式判断*/
  // 油门校准或者未起飞时
  if (remoteData.vofa_remote == 1 || pidTarget.baseThrottle < 1400) {
    MotorSet(pidTarget.baseThrottle, pidTarget.baseThrottle,
             pidTarget.baseThrottle, pidTarget.baseThrottle);
  }
  // 姿态控制(只在30%-70%的近似油门区间控制)
  else if (pidTarget.baseThrottle >= 1400 &&
           remoteData.vofa_remote == 2) { // pid自控制,保证油门在
    PIDControlPosture(dt, &pidTarget, &mahony, &mpu6050, &pidRoll, &pidPitch,
                      &pidYaw, &pidRollRate, &pidPitchRate, &pidYawRate,
                      &pidRes);
    MotorSet(pidRes.outFR, pidRes.outFL, pidRes.outBR, pidRes.outBL);
  }
  // 直接停机
  else if (remoteData.vofa_remote >= 3) { // 停机
    MotorSet(1000, 1000, 1000, 1000);
  }

  /*装载回传数据*/
  telemetryData.pidPitchCmd = pitch_cmd;
  telemetryData.pidRollCmd = roll_cmd;
  telemetryData.pidYawCmd = yaw_cmd;
}

/**
 * @brief vofa发送任务
 *
 */
void task_VofaTx(void) {
  /*mahony姿态角*/
  vofaBuffer[0] = telemetryData.attitude.mahony_pitch;
  vofaBuffer[1] = telemetryData.attitude.mahony_roll;
  vofaBuffer[2] = telemetryData.attitude.mahony_yaw;
  /*传感器实际值*/
  vofaBuffer[3] = telemetryData.attitude.sensordata.gyro_x;
  vofaBuffer[4] = telemetryData.attitude.sensordata.gyro_y;
  vofaBuffer[5] = telemetryData.attitude.sensordata.gyro_z;
  /*pid计算结果*/
  vofaBuffer[6] = telemetryData.pidPitchCmd;
  vofaBuffer[7] = telemetryData.pidRollCmd;
  vofaBuffer[8] = telemetryData.pidYawCmd;
  /*遥控数据返回*/
  vofaBuffer[9] = telemetryData.remote.remote_throttle;
  vofaBuffer[10] = telemetryData.remote.target;
  vofaBuffer[11] = remoteData.vofa_remote;
  VofaTxFloat(&vofaTxFrame, 12);
}

/**
 * @brief
 * vofa解析接收帧同时更新pid参数(调试模式下)，接收操作由中断接收到定长数据处理
 *
 */
void task_VofaRxDecode(void) {

  if (VofaRxFloat(&vofaRxFrame, rxVofaBuff)) {
    // 将接收到的数组数据解析并装入remoteData队列缓冲结构体
    VofaDecodetoRemoteData(&vofaRxFrame, &remoteData);
  }

// 将解析后的参数直接装入指定的pid控制器
#if defined(ONLY_PITCH_RATE)
  // PID_Init(&pidPitchRate, remoteData.kp, remoteData.ki, remoteData.kd, 350);
#elif defined(ONLY_ROLL_RATE)
  PID_Init(&pidRollRate, remoteData.kp, remoteData.ki, remoteData.kd, 200);
#elif defined(ONLY_YAW_RATE)
  PID_Init(&pidYawRate, remoteData.kp, remoteData.ki, remoteData.kd, 200);
#elif defined(ONLT_PITCH)
  PID_Init(&pidPitch, remoteData.kp, remoteData.ki, remoteData.kd, 200);
#elif defined(ONLY_ROLL)
  PID_Init(&pidRoll, remoteData.kp, remoteData.ki, remoteData.kd, 200);
#elif defined(ONLY_YAW)
  PID_Init(&pidYaw, remoteData.kp, remoteData.ki, remoteData.kd, 200);
#endif
}

/**
 * @brief 判断中断原因，若是接收完成则将数据移植rxvofaBuff
 *
 */
void task_NrfLoadData(void) {
  uint8_t rxData[32];
  // 接收数据
  NRF24L01ReceivePacket(&nrf24, rxData);
  // 数据拷贝
  memcpy(rxVofaBuff, rxData, sizeof(rxVofaBuff));
}
