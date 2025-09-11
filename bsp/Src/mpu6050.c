#include "mpu6050.h"
#include "i2c.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_i2c.h"
#include <math.h>
#include <stdint.h>

#define MPU6050_IIC hi2c2

// 一阶低通滤波器
static void gyro_lowpass_filter(mpu6050_struct *mpu6050, float alpha,
                                float deadband);
uint8_t mpu6050RxBuff[16];

/**
 * @brief 向指定寄存器写入数据
 *
 * @param regAddr 寄存器地址
 * @param data 写入的数据
 */
void MPU6050WriteReg(uint8_t regAddr, uint8_t data) {
  // if (mpu6050TxFlag == HAL_I2C_STATE_READY) {
  //   mpu6050TxFlag = HAL_I2C_STATE_BUSY_TX;
  //   HAL_I2C_Mem_Write_DMA(&MPU6050_IIC, MPU6050_ADDR, regAddr, 1,
  //                         (uint8_t[]){data}, 1);
  // }
  HAL_I2C_Mem_Write(&MPU6050_IIC, MPU6050_ADDR, regAddr, 1, (uint8_t[]){data},
                    1, 10);
}

/**
 * @brief 读取指定寄存器的值
 *
 * @param regAddr 寄存器地址
 * @param rxData 接收用的数组
 * @param num 接收的字节数
 */
void MPU6050ReadRegs(uint8_t regAddr, uint8_t *rxData, uint8_t num) {
  // if (mpu6050RxFlag == HAL_I2C_STATE_READY) {
  //   mpu6050RxFlag = HAL_I2C_STATE_BUSY_RX;
  //   HAL_I2C_Mem_Read_DMA(&MPU6050_IIC, MPU6050_ADDR, regAddr, 1, rxData,
  //   num);
  // }
  HAL_I2C_Mem_Read(&MPU6050_IIC, MPU6050_ADDR, regAddr, 1, rxData, num, 10);
}

/**
 * @brief
 * mpu6050初始化配置函数，正常则会返回1，失败返回读取的设备ID(更多返回值查看HAL_StatusTypeDef类型)
 *
 * @param hi2c2 该设备是挂载在IIC2上的设备
 * @return uint8_t
 */
uint8_t MPU6050_Init(void) {
  uint8_t devID = 0;
  MPU6050WriteReg(MPU6050_PWR_MGMT_1, 0x01);
  MPU6050WriteReg(MPU6050_SMPLRT_DIV, 0x04);   // 1KHz采样率
  MPU6050WriteReg(MPU6050_CONFIG, 0x06);       // 低通滤波，20Hz
  MPU6050WriteReg(MPU6050_GYRO_CONFIG, 0x18);  //+-2000°/s
  MPU6050WriteReg(MPU6050_ACCEL_CONFIG, 0x00); //+-2g
  MPU6050ReadRegs(MPU6050_WHO_AM_I, &devID, 1);
  if (devID != 0x68) {
    return devID;
  }
  return 0;
}

/**
 * @brief
 * mpu6050自动获取原始数据转换为物理量(float)同时按照机体系(FRD)映射。加速度单位:m/s²;陀螺仪单位:°/s;
 * @param  mpu6050 指向mpu6050_struct的地址。用于存储数据
 */
void MPU6050GetRedirectValue(mpu6050_struct *mpu6050) {
  // 获取原始数据
  MPU6050ReadRegs(MPU6050_ACCEL_XOUT_H, mpu6050RxBuff, 14);

  // 解析加速度数据
  mpu6050->acc[0] = (int16_t)(mpu6050RxBuff[0] << 8 | mpu6050RxBuff[1]);
  mpu6050->acc[1] = (int16_t)(mpu6050RxBuff[2] << 8 | mpu6050RxBuff[3]);
  mpu6050->acc[2] = (int16_t)(mpu6050RxBuff[4] << 8 | mpu6050RxBuff[5]);
  // 解析温度数据
  mpu6050->temp = (int16_t)(mpu6050RxBuff[6] << 8 | mpu6050RxBuff[7]);
  // 解析陀螺仪数据
  mpu6050->gyro[0] = (int16_t)(mpu6050RxBuff[8] << 8 | mpu6050RxBuff[9]);
  mpu6050->gyro[1] = (int16_t)(mpu6050RxBuff[10] << 8 | mpu6050RxBuff[11]);
  mpu6050->gyro[2] = (int16_t)(mpu6050RxBuff[12] << 8 | mpu6050RxBuff[13]);

  /*转为物理量*/
  mpu6050->redirectAcc.x = -(float)mpu6050->acc[0] / 16384 * 9.8f;
  mpu6050->redirectAcc.y = (float)mpu6050->acc[1] / 16384 * 9.8f;
  mpu6050->redirectAcc.z = (float)mpu6050->acc[2] / 16384 * 9.8f;
  // 1.0f为观测得到的零漂
  mpu6050->redirectGyro.x = (float)mpu6050->gyro[0] / 16.4f + 1.7;
  mpu6050->redirectGyro.y = -(float)mpu6050->gyro[1] / 16.4f;
  mpu6050->redirectGyro.z = (float)mpu6050->gyro[2] / 16.4f + 0.8;

  // // 二阶低通滤波
  gyro_lowpass_filter(mpu6050, 0.3, 0.6);
}

/**
 * @brief 二阶低通 + 死区滤波
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
