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
typedef enum IST8310Init {
  INIT_OK,
  INIT_ERROR,
} IST8310InitType;

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

/*-----整形向uT转换-----*/
#define MAG_SEN 0.3f

#define IST8310_I2C_ADDR 0x0E
#define IST8310_IIC hi2c1

uint8_t IST8310_Init(void);
void IS8310_GET_RAW_DATA(ist8310_struct *ist8310);
void IST8310_GET_MEG_VAL(ist8310_struct *ist8310);

/*-----IST8310寄存器地址-----*/
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
