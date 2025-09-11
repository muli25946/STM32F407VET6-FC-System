#include "motor.h"
#include "main.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include <stdint.h>

/**
 * @brief 解锁电调(100Hz，占空比应为)
 */
void MotorUnLock(void) {
  TIM4->CCR1 = 2000;
  TIM4->CCR2 = 2000;
  TIM4->CCR3 = 2000;
  TIM4->CCR4 = 2000;
  TIM4->EGR = TIM_EGR_UG;
  HAL_Delay(2000);
  TIM4->CCR1 = 1000;
  TIM4->CCR2 = 1000;
  TIM4->CCR3 = 1000;
  TIM4->CCR4 = 1000;
  TIM4->EGR = TIM_EGR_UG;
}

/**
 * @brief 油门PWM通道初始化
 *
 */
void MotorInit(void) {
  HAL_TIM_PWM_Start(&ESC_PWM, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&ESC_PWM, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&ESC_PWM, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&ESC_PWM, TIM_CHANNEL_4);
}

/**
 * @brief 电调的油门值。(1000-2000的占空比值)
 * @param  frThrottle FR的电机油门大小
 * @param  flThrottle FL的电机油门大小
 * @param  brThrottle BR的电机油门大小
 * @param  blThrottle BL的电机油门大小
 */
void MotorSet(float frThrottle, float flThrottle, float brThrottle,
              float blThrottle) {
  // 输入值合法性检查
  frThrottle = frThrottle > 2000 ? 2000 : frThrottle < 1000 ? 1000 : frThrottle;
  flThrottle = flThrottle > 2000 ? 2000 : flThrottle < 1000 ? 1000 : flThrottle;
  brThrottle = brThrottle > 2000 ? 2000 : brThrottle < 1000 ? 1000 : brThrottle;
  blThrottle = blThrottle > 2000 ? 2000 : blThrottle < 1000 ? 1000 : blThrottle;
  // 线性映射
  TIM4->CCR1 = (uint16_t)frThrottle; // 测试得到ccr范围
  TIM4->CCR2 = (uint16_t)flThrottle; // 测试得到ccr范围
  TIM4->CCR3 = (uint16_t)brThrottle; // 测试得到ccr范围
  TIM4->CCR4 = (uint16_t)blThrottle; // 测试得到ccr范围
  // pwm输出
  TIM4->EGR = TIM_EGR_UG;
}
