#include "ist8310.h"

#include <stdint.h>

#include "stddef.h"

static uint8_t IST8310WriteReg(IST8310ObjectType *ist, uint8_t reg,
                               uint8_t data); // 写一个寄存器值
static uint8_t IST8310ReadReg(IST8310ObjectType *ist, uint8_t reg,
                              uint8_t *rxData); // 读一个寄存器值
static uint8_t IST8310WriteBuff(IST8310ObjectType *ist, uint8_t reg,
                                uint8_t *pTxBuf, uint8_t len); // 写多个数据
static uint8_t IST8310ReadBuff(IST8310ObjectType *ist, uint8_t reg,
                               uint8_t *pRxBuf, uint8_t len); // 读多个数据
static uint8_t IST8310Check(IST8310ObjectType *ist);          // 一阶低通滤波器

/**
 * @brief ist8310对象初始化函数
 *
 * @param ist ist8310对象
 * @param read iic读操作函数指针
 * @param write iic写操作函数指针
 * @param irq 获取中断函数指针
 * @param delay 毫秒延迟函数指针
 * @return IST8310ErrorType
 */
IST8310ErrorType IST8310ObjectInit(IST8310ObjectType *ist, IST8310ReadBuf read,
                                   IST8310WriteBuf write,
                                   IST8310Delayms delay) {
  uint8_t retry = 0;

  /*检查注入函数是否空缺*/
  if ((ist == NULL) || (read == NULL) || (write == NULL) || 
      (delay == NULL)) {
    return IST8310_InitError;
  }

  /*注入函数*/
  ist->ReadBuf = read;
  ist->WriteBuf = write;
  ist->Delayms = delay;

  /*设备检查*/
  while (IST8310Check(ist) && (retry < 5)) {
    retry++;
  }

  if (retry >= 5) {
    return IST8310_Absent;
  }

  /*设置基础配置*/
  // 不设置中断
  IST8310WriteReg(ist, IST8310_CNTL2_ADDR, IST8310_STAT2_NONE_ALL);
  // 平均采样4次
  IST8310WriteReg(ist, IST8310_AVGCNTL_ADDR, IST8310_AVGCNTL_FOURTH);
  // 输出频率
  IST8310WriteReg(ist, IST8310_CNTL1_ADDR, IST8310_CNTL1_CONTINUE);

  return IST8310_NoError;
}

/**
 * @brief 获得传感器测量得到的初始值
 *
 * @param ist ist8310对象
 */
void IST8310_GET_RAW_DATA(IST8310ObjectType *ist) {
  uint8_t temp[6];
  IST8310ReadBuff(ist, IST8310_DATA_XL_ADDR, temp, 6);
  ist->data.raw_x = (int16_t)(temp[1] << 8 | temp[0]);
  ist->data.raw_y = (int16_t)(temp[3] << 8 | temp[2]);
  ist->data.raw_z = (int16_t)(temp[5] << 8 | temp[4]);
}

/**
 * @brief 获得传感器测量经转换后的磁力值。并进行映射(转为机体的FRD)
 *
 * @param ist ist8310对象
 */
void IST8310_GET_MEG_VAL(IST8310ObjectType *ist) {
  IST8310_GET_RAW_DATA(ist);
  ist->data.x = ist->data.raw_x * MAG_SEN;
  ist->data.y = ist->data.raw_y * MAG_SEN;
  ist->data.z = ist->data.raw_z * MAG_SEN;

  // 由于imu为zyx方向，磁力计为zxy。现交换xy轴磁力计
  ist->data.redirect_x = ist->data.y;
  ist->data.redirect_y = -ist->data.x;
  ist->data.redirect_z = -ist->data.z;
}

/**
 * @brief 检测IST8310是否存在
 *
 * @param ist IST8310对象
 * @return uint8_t 0，成功;1，失败
 */
static uint8_t IST8310Check(IST8310ObjectType *ist) {
  uint8_t rxData;
  IST8310ReadReg(ist, IST8310_CHIP_ID_ADDR, &rxData);
  if (rxData != IST8310_CHIP_ID_VAL) {
    return 1;
  } else {
    return 0;
  }
}

/**
 * @brief 向IST8310的某个寄存器写值
 *
 * @param ist ist8310对象
 * @param reg 要写入的寄存器
 * @param data 要写入的值
 * @return uint8_t 状态值
 */
static uint8_t IST8310WriteReg(IST8310ObjectType *ist, uint8_t reg,
                               uint8_t data) {
  uint8_t status;
  status = ist->WriteBuf(reg, &data, 1);
  return status;
}

/**
 * @brief 读IST8310的某个寄存器值
 *
 * @param ist ist8310对象
 * @param reg 要读取的寄存器
 * @param rxData 接受数据的变量地址
 * @return uint8_t 状态值
 */
static uint8_t IST8310ReadReg(IST8310ObjectType *ist, uint8_t reg,
                              uint8_t *rxData) {
  uint8_t status;
  status = ist->ReadBuf(reg, rxData, 1);
  return status;
}

/**
 * @brief 向IST8310的某个寄存器写数组
 *
 * @param ist ist8310对象
 * @param reg 要写入的寄存器
 * @param pTxBuf 要写入数组的首地址
 * @param len 写入数据长度
 * @return uint8_t 状态值
 */
static uint8_t IST8310WriteBuff(IST8310ObjectType *ist, uint8_t reg,
                                uint8_t *pTxBuf, uint8_t len) {
  uint8_t status;
  status = ist->WriteBuf(reg, pTxBuf, len);
  return status;
}

/**
 * @brief 从ist8310某个寄存器开始读取指定长度数据
 *
 * @param ist ist8310对象
 * @param reg 要读取的寄存器
 * @param pRxBuf 接收数据的数组首地址
 * @param len 接收数据长度
 * @return uint8_t 状态值
 */
static uint8_t IST8310ReadBuff(IST8310ObjectType *ist, uint8_t reg,
                               uint8_t *pRxBuf, uint8_t len) {
  uint8_t status;
  status = ist->ReadBuf(reg, pRxBuf, len);
  return status;
}
