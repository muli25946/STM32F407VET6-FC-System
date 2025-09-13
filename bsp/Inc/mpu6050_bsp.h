#ifndef MPU6050_BSP_H
#define MPU6050_BSP_H

/*依赖*/
#include "mpu6050.h"
/*HAL*/
#include "i2c.h"

#define MPU6050_IIC hi2c2

uint8_t BspMPU6050ReadBuf(uint8_t reg, uint8_t *pRxBuf,
                          uint8_t len); // 声明读取指定寄存器上多个字节的函数
uint8_t BspMPU6050WriteBuf(uint8_t reg, uint8_t *pTxBuf,
                           uint8_t len); // 声明写入指定寄存器指定长度数据的函数
void BspMPU6050Delayms(uint32_t nTime);  // 声明毫秒延迟函数

#endif
