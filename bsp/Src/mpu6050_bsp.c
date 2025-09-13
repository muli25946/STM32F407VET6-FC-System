/*依赖*/
#include "mpu6050_bsp.h"
#include "mpu6050.h"
/*HAL*/
#include "stm32f4xx_hal_i2c.h"
#include <stdint.h>

/**
 * @brief 向ist8310设备驱动提供iic读取多个字节的硬件驱动接口
 *
 * @param reg 要读取的寄存器
 * @param pRxBuf 用于接收数据的数组
 * @param len 接收的数据长度
 * @return uint8_t 状态值
 */
uint8_t BspMPU6050ReadBuf(uint8_t reg, uint8_t *pRxBuf, uint8_t len) {
  uint8_t status;
  status = HAL_I2C_Mem_Read(&MPU6050_IIC, MPU6050_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, pRxBuf, len, 10);
  return status;
}

/**
 * @brief 向ist8310设备驱动提供iic写入多个字节的硬件驱动接口
 *
 * @param reg 要写入的寄存器
 * @param pTxBuf 发送数据的数组首地址
 * @param len 发送的数据长度
 * @return uint8_t 状态值
 */
uint8_t BspMPU6050WriteBuf(uint8_t reg, uint8_t *pTxBuf, uint8_t len) {
  uint8_t status;
  status = HAL_I2C_Mem_Write(&MPU6050_IIC, MPU6050_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, pTxBuf, len, 10);
  return status;
}

/**
 * @brief 向ist8310设备驱动提供延迟的函数接口
 *
 * @param nTime 延迟时间ms
 */
void BspMPU6050Delayms(uint32_t nTime) { HAL_Delay(nTime); }
