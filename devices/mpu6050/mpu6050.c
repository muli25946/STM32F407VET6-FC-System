/*依赖*/
#include "mpu6050.h"
/*HAL*/
#include "i2c.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include <math.h>
#include <stdint.h>

static uint8_t MPU6050WriteReg(MPU6050ObjectType *mpu6050, uint8_t reg,
                               uint8_t data); // 写一个寄存器值
static uint8_t MPU6050ReadReg(MPU6050ObjectType *mpu6050, uint8_t reg,
                              uint8_t *rxData); // 读一个寄存器值
static uint8_t MPU6050WriteBuff(MPU6050ObjectType *mpu6050, uint8_t reg,
                                uint8_t *pTxBuf, uint8_t len); // 写多个数据
static uint8_t MPU6050ReadBuff(MPU6050ObjectType *mpu6050, uint8_t reg,
                               uint8_t *pRxBuf, uint8_t len);
static uint8_t MPU6050Check(MPU6050ObjectType *mpu6050); // 读多个数据
static void gyro_lowpass_filter(mpu6050_struct *mpu6050, float alpha,
                                float deadband); // 一阶低通滤波器

MPU6050ErrorType MPU6050ObjectInit(MPU6050ObjectType *mpu6050,
                                   MPU6050ReadBuf read, MPU6050WriteBuf write,
                                   MPU6050Delayms delay) {
  uint8_t retry = 0;
  uint8_t status = 0;

  /*检查注入函数是否空缺*/
  if ((read == NULL) || (write == NULL) || (delay = NULL)) {
    return MPU6050_InitError;
  }

  /*注入函数*/
  mpu6050->ReadBuf = read;
  mpu6050->WriteBuf = write;
  mpu6050->Delayms = delay;

  /*设备检查*/
  while ((retry < 5) && MPU6050Check(mpu6050)) {
    return MPU6050_Absent;
  }

  /*寄存器配置*/
  status += MPU6050WriteReg(mpu6050, MPU6050_PWR_MGMT_1, 0x01);
  status += MPU6050WriteReg(mpu6050, MPU6050_SMPLRT_DIV, 0x04);
  status += MPU6050WriteReg(mpu6050, MPU6050_CONFIG, 0x06);
  status += MPU6050WriteReg(mpu6050, MPU6050_GYRO_CONFIG, 0x18);
  status += MPU6050WriteReg(mpu6050, MPU6050_ACCEL_CONFIG, 0x00);

  if (status != 0) {
    return MPU6050_InitError;
  }

  return MPU6050_NoError;
}

/**
 * @brief
 * mpu6050自动获取原始数据转换为物理量(float)同时按照机体系(FRD)映射。加速度单位:m/s²;陀螺仪单位:°/s;
 * @param  mpu6050 指向mpu6050_struct的地址。用于存储数据
 */
void MPU6050GetRedirectValue(MPU6050ObjectType *mpu6050) {
  uint8_t mpu6050RxBuff[14];
  // 获取原始数据
  MPU6050ReadBuff(mpu6050, MPU6050_ACCEL_XOUT_H, mpu6050RxBuff, 14);

  // 解析加速度数据
  mpu6050->data.acc[0] = (int16_t)(mpu6050RxBuff[0] << 8 | mpu6050RxBuff[1]);
  mpu6050->data.acc[1] = (int16_t)(mpu6050RxBuff[2] << 8 | mpu6050RxBuff[3]);
  mpu6050->data.acc[2] = (int16_t)(mpu6050RxBuff[4] << 8 | mpu6050RxBuff[5]);
  // 解析温度数据
  mpu6050->data.temp = (int16_t)(mpu6050RxBuff[6] << 8 | mpu6050RxBuff[7]);
  // 解析陀螺仪数据
  mpu6050->data.gyro[0] = (int16_t)(mpu6050RxBuff[8] << 8 | mpu6050RxBuff[9]);
  mpu6050->data.gyro[1] = (int16_t)(mpu6050RxBuff[10] << 8 | mpu6050RxBuff[11]);
  mpu6050->data.gyro[2] = (int16_t)(mpu6050RxBuff[12] << 8 | mpu6050RxBuff[13]);

  /*转为物理量*/
  mpu6050->data.redirectAcc.x = -(float)mpu6050->data.acc[0] / 16384 * 9.8f;
  mpu6050->data.redirectAcc.y = (float)mpu6050->data.acc[1] / 16384 * 9.8f;
  mpu6050->data.redirectAcc.z = (float)mpu6050->data.acc[2] / 16384 * 9.8f;
  // 1.0f为观测得到的零漂
  mpu6050->data.redirectGyro.x = (float)mpu6050->data.gyro[0] / 16.4f + 1.7;
  mpu6050->data.redirectGyro.y = -(float)mpu6050->data.gyro[1] / 16.4f;
  mpu6050->data.redirectGyro.z = (float)mpu6050->data.gyro[2] / 16.4f + 0.8;

  // 二阶低通滤波
  gyro_lowpass_filter(&mpu6050->data, 0.3, 0.6);
}

/**
 * @brief 为mpu6050特化的二阶低通 + 死区滤波
 * @param mpu6050  MPU6050数据结构
 * @param alpha    一阶低通系数 (0~1，越接近1滤波越强)
 * @param deadband 死区阈值 (deg/s)，小于该值直接置0
 */
static void gyro_lowpass_filter(mpu6050_struct *mpu6050, float alpha,
                                float deadband) {
  // 历史状态（两个缓存用于二阶滤波）
  static float gyro_lpf_last1[3] = {0};
  static float gyro_lpf_last2[3] = {0};
  static int gyro_lpf_init = 0;
  // 原始输入
  float gx = mpu6050->redirectGyro.x;
  float gy = mpu6050->redirectGyro.y;
  float gz = mpu6050->redirectGyro.z;

  // 初始化：第一次直接赋值，避免跳变
  if (!gyro_lpf_init) {
    gyro_lpf_last1[0] = gx;
    gyro_lpf_last1[1] = gy;
    gyro_lpf_last1[2] = gz;
    gyro_lpf_last2[0] = gx;
    gyro_lpf_last2[1] = gy;
    gyro_lpf_last2[2] = gz;
    gyro_lpf_init = 1;
  }

  // === 一阶低通1 ===
  float out1x = alpha * gyro_lpf_last1[0] + (1.0f - alpha) * gx;
  float out1y = alpha * gyro_lpf_last1[1] + (1.0f - alpha) * gy;
  float out1z = alpha * gyro_lpf_last1[2] + (1.0f - alpha) * gz;
  gyro_lpf_last1[0] = out1x;
  gyro_lpf_last1[1] = out1y;
  gyro_lpf_last1[2] = out1z;

  // === 一阶低通2 ===
  float out2x = alpha * gyro_lpf_last2[0] + (1.0f - alpha) * out1x;
  float out2y = alpha * gyro_lpf_last2[1] + (1.0f - alpha) * out1y;
  float out2z = alpha * gyro_lpf_last2[2] + (1.0f - alpha) * out1z;
  gyro_lpf_last2[0] = out2x;
  gyro_lpf_last2[1] = out2y;
  gyro_lpf_last2[2] = out2z;

  // === 死区处理 ===
  if (fabsf(out2x) < deadband)
    out2x = 0.0f;
  if (fabsf(out2y) < deadband)
    out2y = 0.0f;
  if (fabsf(out2z) < deadband)
    out2z = 0.0f;

  // 输出回结构体
  mpu6050->redirectGyro.x = out2x;
  mpu6050->redirectGyro.y = out2y;
  mpu6050->redirectGyro.z = out2z;
}

/**
 * @brief 检测MPU6050是否存在
 *
 * @param mpu6050 MPU6050对象
 * @return uint8_t 0，成功;1，失败
 */
static uint8_t MPU6050Check(MPU6050ObjectType *mpu6050) {
  uint8_t devID;
  MPU6050ReadReg(mpu6050, MPU6050_WHO_AM_I, &devID);
  if (devID != MPU6050_DEVICE_ID) {
    return 1;
  } else {
    return 0;
  }
}

/**
 * @brief 向mpu6050的某个寄存器写数组
 *
 * @param mpu6050 mpu6050对象
 * @param reg 要写入的寄存器
 * @param data 要写入的值
 * @return uint8_t 状态值
 */
static uint8_t MPU6050WriteReg(MPU6050ObjectType *mpu6050, uint8_t reg,
                               uint8_t data) {
  uint8_t status;
  status = mpu6050->WriteBuf(reg, &data, 1);
  return status;
}

/**
 * @brief 向mpu6050的某个寄存器写数组
 *
 * @param mpu6050 mpu6050对象
 * @param reg 要写入的寄存器
 * @param rxData 接收数据的变量地址
 * @return uint8_t 状态值
 */
static uint8_t MPU6050ReadReg(MPU6050ObjectType *mpu6050, uint8_t reg,
                              uint8_t *rxData) {
  uint8_t status;
  status = mpu6050->ReadBuf(reg, rxData, 1);
  return status;
}

/**
 * @brief 向MPU6050的某个寄存器写数组
 *
 * @param mpu6050 MPU6050对象
 * @param reg 要写入的寄存器
 * @param pTxBuf 要写入数组的首地址
 * @param len 写入数据长度
 * @return uint8_t 状态值
 */
static uint8_t MPU6050WriteBuff(MPU6050ObjectType *mpu6050, uint8_t reg,
                                uint8_t *pTxBuf, uint8_t len) {
  uint8_t status;
  status = mpu6050->WriteBuf(reg, pTxBuf, len);
  return status;
}

/**
 * @brief 从MPU6050某个寄存器开始读取指定长度数据
 *
 * @param mpu6050 MPU6050对象
 * @param reg 要读取的寄存器
 * @param pRxBuf 接收数据的数组首地址
 * @param len 接收数据长度
 * @return uint8_t 状态值
 */
static uint8_t MPU6050ReadBuff(MPU6050ObjectType *mpu6050, uint8_t reg,
                               uint8_t *pRxBuf, uint8_t len) {
  uint8_t status;
  status = mpu6050->ReadBuf(reg, pRxBuf, len);
  return status;
}
