#include "ist8310.h"
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdint.h>

/**
 * @brief 向IST8310的某个寄存器写值
 *
 * @param regAddr 寄存器地址
 * @param data 写入的数据
 */
void IST8310WriteReg(uint8_t regAddr, uint8_t data) {
  HAL_I2C_Mem_Write(&IST8310_IIC, (IST8310_I2C_ADDR << 1), regAddr,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t[]){data}, 1, 10);
}

/**
 * @brief 从IST8310的某个寄存器开始读值
 *
 * @param regAddr 要读的寄存器地址
 * @param rxData 接受数据的uint8_t数组
 * @param num 接收的字节数
 */
void IST8310ReadRegs(uint8_t regAddr, uint8_t *rxData, uint8_t num) {
  HAL_I2C_Mem_Read(&IST8310_IIC, (IST8310_I2C_ADDR << 1), regAddr,
                   I2C_MEMADD_SIZE_8BIT, rxData, num, 10);
}

/**
 * @brief IST8310初始化函数。关闭中断，4次采样。连续输出频率200Hz
 * @return uint8_t
 */
uint8_t IST8310_Init(void) {
  uint8_t txData[1];
  uint8_t rxData[1];
  /*重启磁力计*/
  HAL_GPIO_WritePin(IST8310_RESET_GPIO_Port, IST8310_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(IST8310_RESET_GPIO_Port, IST8310_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(50);

  /*检查IIC通信*/
  IST8310ReadRegs(IST8310_CHIP_ID_ADDR, rxData, 1);
  if (IST8310_CHIP_ID_VAL != rxData[0]) {
    return rxData[0];
  }
  /*设置基础配置*/
  // 不设置中断
  txData[0] = IST8310_STAT2_NONE_ALL;
  IST8310WriteReg(IST8310_CNTL2_ADDR, txData[0]);
  // 平均采样4次
  txData[0] = IST8310_AVGCNTL_FOURTH;
  IST8310WriteReg(IST8310_AVGCNTL_ADDR, txData[0]);

  // 输出频率
  txData[0] = IST8310_CNTL1_CONTINUE;
  IST8310WriteReg(IST8310_CNTL1_ADDR, txData[0]);

  return INIT_OK;
}

/**
 * @brief 获得传感器测量得到的初始值
 * @param  ist8310
 * ist8310_struct结构体。结果在ist8310_struct->raw_x,raw_y,raw_z中
 */
void IST8310_GET_RAW_DATA(ist8310_struct *ist8310) {
  uint8_t temp[6];
  IST8310ReadRegs(IST8310_DATA_XL_ADDR, temp, 6);
  ist8310->raw_x = (int16_t)(temp[1] << 8 | temp[0]);
  ist8310->raw_y = (int16_t)(temp[3] << 8 | temp[2]);
  ist8310->raw_z = (int16_t)(temp[5] << 8 | temp[4]);
}

/**
 * @brief 获得传感器测量经转换后的磁力值。并进行映射(转为机体的FRD)
 * @param  ist8310 ist8310_struct结构体。结果在ist8310_struct->x,y,z中
 */
void IST8310_GET_MEG_VAL(ist8310_struct *ist8310) {
  IST8310_GET_RAW_DATA(ist8310);
  ist8310->x = ist8310->raw_x * MAG_SEN;
  ist8310->y = ist8310->raw_y * MAG_SEN;
  ist8310->z = ist8310->raw_z * MAG_SEN;

  // 由于imu为zyx方向，磁力计为zxy。现交换xy轴磁力计
  ist8310->redirect_x = ist8310->y;
  ist8310->redirect_y = -ist8310->x;
  ist8310->redirect_z = -ist8310->z;
}
