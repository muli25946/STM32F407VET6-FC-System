#ifndef MAHONY_H
#define MAHONY_H

#include "ist8310.h"
#include "mpu6050.h"
#include "stm32f407xx.h"

// 载体系下的欧拉角
typedef struct LoadSystem {
  float pitch;
  float roll;
  float yaw;
  float pitchRad;
  float rollRad;
  float yawRad;
} LoadSystemType;

typedef struct Navigation_System {
  float pitch;
  float roll;
  float yaw;
} NavigationSystemType;

// mahony解算结果
typedef struct Mahony {
  LoadSystemType FRD;       // 载体系用前右下
  NavigationSystemType NED; // 导航系用北东地
} MahonyType;

typedef struct MahonyFilter {
  float twoKp; // 2 * 比例增益 (Kp)
  float twoKi; // 2 * 积分增益 (Ki)
  float q0, q1, q2,
      q3; // 传感器帧相对于辅助帧的四元数
  float integralFBx, integralFBy,
      integralFBz; // 按 Ki 缩放的积分误差项
} MahonyFilterType;

void MahonyInit(void);
void MahonyUpdateAHRS(float dt, mpu6050_struct *mpu6050,
                      ist8310_struct *ist8310);
void MahonyUpdateAHRSIMU(float dt, mpu6050_struct *mpu6050);
void MahonyGetEuler(float dt, mpu6050_struct *mpu6050, ist8310_struct *ist8310,
                    MahonyType *mahony);

#endif // !MAHONY_H
