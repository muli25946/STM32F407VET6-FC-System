/*自定义依赖*/
#include "nrf24l01_bsp.h"
#include "nrf24l01.h"
/*官方库*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include <stdint.h>

/**
 * @brief 向nrf24设备驱动提供spi交换字节的硬件驱动接口
 *
 * @param TxData 发送的数据
 * @return uint8_t 返回接收到的数据
 */
uint8_t NRF24ReadWriteByte(uint8_t TxData) {
  uint8_t rxData;
  HAL_SPI_TransmitReceive(&NRF_SPI, &TxData, &rxData, 1, 10);
  return rxData;
}

/**
 * @brief 向nrf24设备驱动提供控制CS引脚的硬件驱动接口
 *
 * @param cs NRF24L01CSType类型
 */
void NRF24ChipSelect(NRF24L01CSType cs) {
  HAL_GPIO_WritePin(NRF_CS_GPIO_Port, NRF_CS_Pin,
                    (cs == NRF24L01CS_Enable) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief 向nrf24设备驱动提供控制CE引脚的硬件驱动接口
 *
 * @param en NRF24L01CEType类型
 */
void NRF24ChipEnable(NRF24L01CEType en) {
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin,
                    (en == NRF24L01CE_Enable) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 向nrf24设备驱动提供获取IRQ引脚电平的硬件驱动接口
 *
 * @return uint8_t 返回引脚电平值.有中断返回0，无中断返回1
 */
uint8_t NRF24GetIRQ(void) {
  GPIO_PinState pinState = HAL_GPIO_ReadPin(NRF_IRQ_GPIO_Port, NRF_IRQ_Pin);
  return (pinState == GPIO_PIN_RESET) ? 0 : 1;
}

/**
 * @brief 向nrf24设备驱动提供延时的硬件驱动接口
 *
 * @param nTime 延时时间，ms
 */
void NRF24Delayms(volatile uint32_t nTime) { HAL_Delay(nTime); }
