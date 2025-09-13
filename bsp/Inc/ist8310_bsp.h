#ifndef IST8310_BSP_H
#define IST8310_BSP_H

/*依赖*/
#include "ist8310.h"
/*HAL*/
#include "i2c.h"
#include <stdint.h>

#define IST8310_IIC hi2c1

uint8_t BspIST8310ReadBuf(uint8_t reg, uint8_t *pRxBuf,
                          uint8_t len); // 声明读取指定寄存器上
uint8_t BspIST8310WriteBuf(uint8_t reg, uint8_t *pTxBuf,
                           uint8_t len);   // 声明写入指定寄存
void BspIST8310Delayms(uint32_t nTime); // 声明毫秒延迟函数

#endif
