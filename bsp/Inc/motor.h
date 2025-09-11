#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

void MotorUnLock(void);
void MotorInit(void);
void MotorSet(float frThrottle, float flThrottle, float brThrottle,
              float blThrottle);

#endif // !MOTOR_H
