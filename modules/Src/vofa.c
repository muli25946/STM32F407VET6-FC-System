#include "vofa.h"
#include "fc_sys.h"
#include "oled.h"
#include "pid.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>

const uint8_t floatTail[4] = {0x00, 0x00, 0x80, 0x7F};
uint8_t rxVofaBuff[6] = {0xFF}; // 定长串口接收数组，vofa.c中定义的全局数组

/**
 * @brief 调用串口向vofa发送一个数据
 * @param  floatFrame 一个VofaTxFloatFrame类型的地址
 * @param num 数组长度
 * @note
 * 使用前需使用floatFrame->pFloat指向一个float数组
 */
void VofaTxFloat(VofaTxFloatFrame *floatFrame, uint8_t num) {
  uint8_t txTempBuff[4 * num + 4];

  memcpy(txTempBuff, floatFrame->pFloat, 4 * num);
  memcpy(&txTempBuff[4 * num], floatTail, 4);

  HAL_UART_Transmit(&VOFA_UART, txTempBuff, (4 * num + 4), 10);
}

/**
 * @brief 将传入的6字节数组转为VofaRxFloatFrame帧
 *
 * @param rxFloatFrame 转换得到的vofa接收帧
 * @param rxBuffer 需要转换的数组
 * @return true 数据无误
 * @return false 帧头错误
 * @note
 * 该数组使用自定义协议，格式为(固定帧头)[6C]+(标识)[ID]+(float数据)[4字节]
 */
uint8_t VofaRxFloat(VofaRxFloatFrame *rxFloatFrame, uint8_t *rxBuffer) {
  if (rxBuffer[0] == 0x6C) {
    rxFloatFrame->Head = 0x6C;
    rxFloatFrame->ID = rxBuffer[1];
    rxFloatFrame->pFloat = (float *)&rxBuffer[2];
    rxFloatFrame->dataReady = 1;
  } else {
    rxFloatFrame->dataReady = 0;
  }

  return rxFloatFrame->dataReady;
}

/**
 * @brief
 *
 * @param frame 解析接收的VofaRxFloatFrame帧并自动装入对应的参数中
 * @param pid VofaRxFloatFrame帧地址
 * @param pidTarget 装入解析值的pid控制器
 * @param pidTargetData 期望指定的pid目标值
 * @param vofa_remote 状态控制量
 */
void VofaDecodeFrame(VofaRxFloatFrame *frame, PIDControllerType *pid,
                     PIDTargetType *pidTarget, float pidTargetData,
                     uint8_t *vofa_remote) {
  float tempFloat; // 中转数据

  memcpy(&tempFloat, frame->pFloat, sizeof(float));
  switch (frame->ID) {
  case K_P:
    memcpy(&pid->kp, &tempFloat, sizeof(float));
    break;
  case K_I:
    memcpy(&pid->ki, &tempFloat, sizeof(float));
    break;
  case K_D:
    memcpy(&pid->kd, &tempFloat, sizeof(float));
    break;
  case TARGET_VALUE:
    memcpy(&pidTargetData, &tempFloat, sizeof(float));
    break;
  case STATUS:
    *vofa_remote += 1; // vofa控制状态值
    break;
  case REMOTE:
    memcpy(&pidTarget->baseThrottle, &tempFloat, sizeof(float));
    break;
  default:
    break;
  }
}

/**
 * @brief 用于freeRTOS的解析函数，将接收到的数据帧解码并放入队列前缓冲区
 *
 * @param frame 接收到的数据帧
 * @param rxRemoteData 要装入的RemoteData_t结构体。该结构体用于装入队列
 */
void VofaDecodetoRemoteData(VofaRxFloatFrame *frame,
                            RemoteData_t *rxRemoteData) {
  float tempFloat; // 中转数据

  memcpy(&tempFloat, frame->pFloat, sizeof(float));
  switch (frame->ID) {
  case K_P:
    memcpy(&rxRemoteData->kp, &tempFloat, sizeof(float));
    break;
  case K_I:
    memcpy(&rxRemoteData->ki, &tempFloat, sizeof(float));
    break;
  case K_D:
    memcpy(&rxRemoteData->kd, &tempFloat, sizeof(float));
    break;
  case TARGET_VALUE:
    memcpy(&rxRemoteData->target, &tempFloat, sizeof(float));
    break;
  case VOFA_FC_STATUS:
    rxRemoteData->vofa_remote++; // vofa控制状态值
    break;
  case REMOTE: // 远控油门
    memcpy(&rxRemoteData->remote_throttle, &tempFloat, sizeof(float));
    break;
  default:
    break;
  }
}
