/**
 * @file ist8310.h
 * @brief 该设备挂载在IIC1上。IIC速率400KHz
 * @author Muli25946 (2687025869@qq.com)
 * @version 1.0
 * @date 2025-08-05
 *
 * @copyright Copyright (c) 2025  GPL
 *
 */
#ifndef IST8310_H
#define IST8310_H

#include <stdint.h>

/*整形向uT转换*/
#define MAG_SEN 0.3f
#define IST8310_I2C_ADDR 0x0E
#define IST8310_IIC hi2c1

/*定义IST8310错误枚举*/
typedef enum {
  IST8310_NoError,
  IST8310_InitError,
  IST8310_Absent
} IST8310ErrorType;

/*ist8310存储数据结构体*/
typedef struct IST8310 {
  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;
  float x;
  float y;
  float z;
  float redirect_x;
  float redirect_y;
  float redirect_z;
} ist8310_struct;

/*IST8310对象类型*/
typedef struct {
  /*数据*/
  ist8310_struct data;
  /*方法*/
  uint8_t (*ReadBuf)(uint8_t reg, uint8_t *pRxBuf,
                     uint8_t len); // 声明读取指定寄存器上多个字节的函数
  uint8_t (*WriteBuf)(uint8_t reg, uint8_t *pTxBuf,
                      uint8_t len); // 声明写入指定寄存器指定长度数据的函数
  void (*Delayms)(uint32_t nTime);  // 声明毫秒延迟函数
} IST8310ObjectType;

typedef uint8_t (*IST8310ReadBuf)(
    uint8_t reg, uint8_t *pRxBuf,
    uint8_t len); // 声明读取指定寄存器上多个字节的函数
typedef uint8_t (*IST8310WriteBuf)(
    uint8_t reg, uint8_t *pTxBuf,
    uint8_t len); // 声明写入指定寄存器指定长度数据的函数
typedef void (*IST8310Delayms)(uint32_t nTime); // 声明毫秒延迟函数

IST8310ErrorType IST8310ObjectInit(IST8310ObjectType *ist, IST8310ReadBuf read,
                                   IST8310WriteBuf write, IST8310Delayms delay);
void IST8310_GET_RAW_DATA(IST8310ObjectType *ist);
void IST8310_GET_MEG_VAL(IST8310ObjectType *ist);

/*IST8310寄存器地址*/
#define IST8310_CHIP_ID_ADDR 0x00
#define IST8310_CHIP_ID_VAL 0x10
#define IST8310_STAT1_ADDR 0x02
/*3轴磁力计数据*/
#define IST8310_DATA_XL_ADDR 0x03
#define IST8310_DATA_XH_ADDR 0x04
#define IST8310_DATA_YL_ADDR 0x05
#define IST8310_DATA_YH_ADDR 0x06
#define IST8310_DATA_ZL_ADDR 0x07
#define IST8310_DATA_ZH_ADDR 0x08
/*寄存器*/
#define IST8310_STAT2_ADDR 0x09
#define IST8310_CNTL1_ADDR 0x0A
#define IST8310_CNTL1_SLEEP 0x00
#define IST8310_CNTL1_SINGLE 0x01
#define IST8310_CNTL1_CONTINUE 0x0B
#define IST8310_CNTL2_ADDR 0x0B
#define IST8310_STAT2_NONE_ALL 0x00
#define IST8310_SELF_CHECK_ADDR 0x0C
#define IST8310_TEMPL_ADDR 0x1C
#define IST8310_TEMPH_ADDR 0x1D
/*平均采样次数*/
#define IST8310_AVGCNTL_ADDR 0x41
#define IST8310_AVGCNTL_TWICE 0x09
#define IST8310_AVGCNTL_FOURTH 0x12

#endif // !IST8310_H
