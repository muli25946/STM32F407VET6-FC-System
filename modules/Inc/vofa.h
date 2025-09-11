#ifndef VOFA_H
#define VOFA_H

#define VOFA_UART huart2

#include "mahony.h"
#include "pid.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include <stdint.h>

typedef enum VofaID {
  /*pid调节*/
  K_P = 0xA1,
  K_I = 0xA2,
  K_D = 0xA3,
  /*目标值设置*/
  TARGET_VALUE = 0xB1,
  /*状态控制*/
  VOFA_FC_STATUS = 0xC0,
  /*油门控制*/
  REMOTE = 0xE0,
} VofaIDEnum;

typedef enum DecodeStatus {
  DECODE_READY,
  DECODE_ERR,
} DecodeStatusEnum;

typedef struct VofaTxFloat {
  float *pFloat; // 指向float数组的指针
} VofaTxFloatFrame;

typedef struct VofaRxFloat {
  uint8_t Head;      // 接收数据的帧头
  uint8_t ID;        // 接收数据的ID号,0xFF为错误码
  float *pFloat;     // 指向float数组的指针
  uint8_t dataReady; // 数据接收就绪标志
} VofaRxFloatFrame;

/*遥控数据接收队列前结构体*/
typedef struct {
  float vofa_remote;
  float remote_throttle;
  float target;
  float kp, ki, kd;
} RemoteData_t;

extern uint8_t rxVofaBuff[6]; // 定长串口接收数组

void VofaTxFloat(VofaTxFloatFrame *floatFrame, uint8_t num);
uint8_t VofaRxFloat(VofaRxFloatFrame *rxFloatFrame, uint8_t *rxBuffer);
void VofaDecodeFrame(VofaRxFloatFrame *frame, PIDControllerType *pid,
                     PIDTargetType *pidTarget, float pidTargetData,
                     uint8_t *vofa_remote);
void VofaDecodetoRemoteData(VofaRxFloatFrame *frame,
                            RemoteData_t *rxRemoteData);

#endif // !VOFA_H
