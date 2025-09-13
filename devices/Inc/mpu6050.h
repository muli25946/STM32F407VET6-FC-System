/**
 * @file mpu6050.h
 * @brief MPU6050挂在IIC2上，数据存储使用mpu6050_struct结构体。
 * @author Muli25946 (2687025869@qq.com)
 * @version 1.0
 * @date 2025-08-03
 *
 * @copyright Copyright (c) 2025  GPL
 *
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

// MPU6050地址
#define MPU6050_ADDR 0x68 << 1
#define MPU6050_IIC hi2c2

typedef enum {
  MPU6050_NoError,
  MPU6050_InitError,
  MPU6050_Absent
} MPU6050ErrorType;

/*物理值结构体*/
typedef struct physicalValue {
  float x;
  float y;
  float z;
} PhysicalType;

/*数据结构体*/
typedef struct MPU6050 {
  int16_t acc[3];  // 对应xyz
  int16_t gyro[3]; // 对应xyz
  uint16_t temp;
  PhysicalType redirectAcc;
  PhysicalType redirectGyro;
} mpu6050_struct;

/*MPU6050对象类型*/
typedef struct {
  /*数据*/
  mpu6050_struct data;
  /*方法*/
  uint8_t (*ReadBuf)(uint8_t reg, uint8_t *pRxBuf,
                     uint8_t len); // 声明读取指定寄存器上多个字节的函数
  uint8_t (*WriteBuf)(uint8_t reg, uint8_t *pTxBuf,
                      uint8_t len); // 声明写入指定寄存器指定长度数据的函数
  void (*Delayms)(uint32_t nTime);  // 声明毫秒延迟函数
} MPU6050ObjectType;

typedef uint8_t (*MPU6050ReadBuf)(
    uint8_t reg, uint8_t *pRxBuf,
    uint8_t len); // 声明读取指定寄存器上多个字节的函数
typedef uint8_t (*MPU6050WriteBuf)(
    uint8_t reg, uint8_t *pTxBuf,
    uint8_t len); // 声明写入指定寄存器指定长度数据的函数
typedef void (*MPU6050Delayms)(uint32_t nTime); // 声明毫秒延迟函数

MPU6050ErrorType MPU6050ObjectInit(MPU6050ObjectType *mpu6050,
                                   MPU6050ReadBuf read, MPU6050WriteBuf write,
                                   MPU6050Delayms delay);
void MPU6050GetRedirectValue(MPU6050ObjectType *mpu6050);

/*MPU6050 寄存器*/
#define MPU6050_SMPLRT_DIV                                                     \
  0x19 // 指定陀螺仪输出频率的分频值以作为 MPU-6050 的采样率
#define MPU6050_CONFIG                                                         \
  0x1A // 加速度计和陀螺仪的外部帧同步(FSYNC)引脚采样以及设置数字低通滤波器
#define MPU6050_GYRO_CONFIG 0x1B  // 触发陀螺仪的自检以及设定陀螺仪的满测量范围
#define MPU6050_ACCEL_CONFIG 0x1C // 触发加速度计的自检并设定其满量程范围

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

#define MPU6050_PWR_MGMT_1 0x6B // 用户配置电源模式和时钟源
#define MPU6050_WHO_AM_I 0x75   // 这个寄存器用来查证该设备的身份,默认0x68
#define MPU6050_DEVICE_ID 0x68

#endif // !MPU6050_H
