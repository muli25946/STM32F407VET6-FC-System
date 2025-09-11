#include "mahony.h"
#include "arm_math.h" //硬件fpu
#include "ist8310.h"
#include "mpu6050.h"
#include <math.h>

// vofa上位机调参
#define twoKpDef (2.0f * 4.3f) // 2 * proportional gain
#define twoKiDef (2.0f * 0.1f) // 2 * integral gain

static MahonyFilterType mahonyFilter;

float InvSqrt(float x);

/**
 * @brief 给定一个用于mahony计算用的数据存储结构体
 */
void MahonyInit(void) {
  mahonyFilter.twoKp = twoKpDef; // 2 * proportional gain (Kp)
  mahonyFilter.twoKi = twoKiDef; // 2 * integral gain (Ki)
  mahonyFilter.q0 = 1.0f;
  mahonyFilter.q1 = 0.0f;
  mahonyFilter.q2 = 0.0f;
  mahonyFilter.q3 = 0.0f;
  mahonyFilter.integralFBx = 0.0f;
  mahonyFilter.integralFBy = 0.0f;
  mahonyFilter.integralFBz = 0.0f;
}

/**
 * @brief 使用mahony进行一次计算求得四元数
 * @param  mpu6050          指向mpu6050_struct的地址，用于读取数据。
 * @param  ist8310          指向ist8310_struct的地址，用于读取数据。
 * @param  filter           指向MahonyFilterType的地址，用于数据计算。
 */
void MahonyUpdateAHRS(float dt, mpu6050_struct *mpu6050,
                      ist8310_struct *ist8310) {
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // 将陀螺仪度/秒转换为弧度/秒
  mpu6050->redirectGyro.x *= 0.0174533f;
  mpu6050->redirectGyro.y *= 0.0174533f;
  mpu6050->redirectGyro.z *= 0.0174533f;

  // 如果磁强计测量无效，则使用 IMU 算法 （避免在磁强计归一化中出现 NaN）
  if ((ist8310->redirect_x == 0.0f) && (ist8310->redirect_y == 0.0f) &&
      (ist8310->redirect_z == 0.0f)) {
    MahonyUpdateAHRSIMU(dt, mpu6050);
    return;
  }

  // 仅在加速度计测量有效时计算反馈（避免加速度计归一化中出现 NaN）
  if (!((mpu6050->redirectAcc.x == 0.0f) && (mpu6050->redirectAcc.y == 0.0f) &&
        (mpu6050->redirectAcc.z == 0.0f))) {

    // 加速度计测量归一化
    recipNorm = InvSqrt(mpu6050->redirectAcc.x * mpu6050->redirectAcc.x +
                        mpu6050->redirectAcc.y * mpu6050->redirectAcc.y +
                        mpu6050->redirectAcc.z * mpu6050->redirectAcc.z);
    mpu6050->redirectAcc.x *= recipNorm;
    mpu6050->redirectAcc.y *= recipNorm;
    mpu6050->redirectAcc.z *= recipNorm;

    // 将磁强计测量归一化
    recipNorm = InvSqrt(ist8310->redirect_x * ist8310->redirect_x +
                        ist8310->redirect_y * ist8310->redirect_y +
                        ist8310->redirect_z * ist8310->redirect_z);
    ist8310->redirect_x *= recipNorm;
    ist8310->redirect_y *= recipNorm;
    ist8310->redirect_z *= recipNorm;

    // 辅助变量避免重复运算
    q0q0 = mahonyFilter.q0 * mahonyFilter.q0;
    q0q1 = mahonyFilter.q0 * mahonyFilter.q1;
    q0q2 = mahonyFilter.q0 * mahonyFilter.q2;
    q0q3 = mahonyFilter.q0 * mahonyFilter.q3;
    q1q1 = mahonyFilter.q1 * mahonyFilter.q1;
    q1q2 = mahonyFilter.q1 * mahonyFilter.q2;
    q1q3 = mahonyFilter.q1 * mahonyFilter.q3;
    q2q2 = mahonyFilter.q2 * mahonyFilter.q2;
    q2q3 = mahonyFilter.q2 * mahonyFilter.q3;
    q3q3 = mahonyFilter.q3 * mahonyFilter.q3;

    // 地球磁场的参考方向
    hx = 2.0f * (ist8310->redirect_x * (0.5f - q2q2 - q3q3) +
                 ist8310->redirect_y * (q1q2 - q0q3) +
                 ist8310->redirect_z * (q1q3 + q0q2));
    hy = 2.0f * (ist8310->redirect_x * (q1q2 + q0q3) +
                 ist8310->redirect_y * (0.5f - q1q1 - q3q3) +
                 ist8310->redirect_z * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (ist8310->redirect_x * (q1q3 - q0q2) +
                 ist8310->redirect_y * (q2q3 + q0q1) +
                 ist8310->redirect_z * (0.5f - q1q1 - q2q2));

    // 重力和磁场的估计方向
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // 误差是估计方向与测量的场矢量方向之和
    halfex =
        (mpu6050->redirectAcc.y * halfvz - mpu6050->redirectAcc.z * halfvy) +
        (ist8310->redirect_y * halfwz - ist8310->redirect_z * halfwy);
    halfey =
        (mpu6050->redirectAcc.z * halfvx - mpu6050->redirectAcc.x * halfvz) +
        (ist8310->redirect_z * halfwx - ist8310->redirect_x * halfwz);
    halfez =
        (mpu6050->redirectAcc.x * halfvy - mpu6050->redirectAcc.y * halfvx) +
        (ist8310->redirect_x * halfwy - ist8310->redirect_y * halfwx);

    // 如果启用，计算并应用积分反馈
    if (mahonyFilter.twoKi > 0.0f) {
      // 按 Ki 缩放的积分误差
      mahonyFilter.integralFBx += mahonyFilter.twoKi * halfex / dt;
      mahonyFilter.integralFBy += mahonyFilter.twoKi * halfey / dt;
      mahonyFilter.integralFBz += mahonyFilter.twoKi * halfez / dt;
      mpu6050->redirectGyro.x += mahonyFilter.integralFBx; // 应用积分反馈
      mpu6050->redirectGyro.y += mahonyFilter.integralFBy;
      mpu6050->redirectGyro.z += mahonyFilter.integralFBz;
    } else {
      mahonyFilter.integralFBx = 0.0f; // 防止整体卷绕
      mahonyFilter.integralFBy = 0.0f;
      mahonyFilter.integralFBz = 0.0f;
    }

    // 应用比例反馈
    mpu6050->redirectGyro.x += mahonyFilter.twoKp * halfex;
    mpu6050->redirectGyro.y += mahonyFilter.twoKp * halfey;
    mpu6050->redirectGyro.z += mahonyFilter.twoKp * halfez;
  }

  // 积分四元数的变化率
  mpu6050->redirectGyro.x *= (0.5f / dt); // 预乘公因数
  mpu6050->redirectGyro.y *= (0.5f / dt);
  mpu6050->redirectGyro.z *= (0.5f / dt);
  qa = mahonyFilter.q0;
  qb = mahonyFilter.q1;
  qc = mahonyFilter.q2;
  mahonyFilter.q0 +=
      (-qb * mpu6050->redirectGyro.x - qc * mpu6050->redirectGyro.y -
       mahonyFilter.q3 * mpu6050->redirectGyro.z);
  mahonyFilter.q1 +=
      (qa * mpu6050->redirectGyro.x + qc * mpu6050->redirectGyro.z -
       mahonyFilter.q3 * mpu6050->redirectGyro.y);
  mahonyFilter.q2 +=
      (qa * mpu6050->redirectGyro.y - qb * mpu6050->redirectGyro.z +
       mahonyFilter.q3 * mpu6050->redirectGyro.x);
  mahonyFilter.q3 +=
      (qa * mpu6050->redirectGyro.z + qb * mpu6050->redirectGyro.y -
       qc * mpu6050->redirectGyro.x);

  // 四元数规范化
  recipNorm = InvSqrt(
      mahonyFilter.q0 * mahonyFilter.q0 + mahonyFilter.q1 * mahonyFilter.q1 +
      mahonyFilter.q2 * mahonyFilter.q2 + mahonyFilter.q3 * mahonyFilter.q3);
  mahonyFilter.q0 *= recipNorm;
  mahonyFilter.q1 *= recipNorm;
  mahonyFilter.q2 *= recipNorm;
  mahonyFilter.q3 *= recipNorm;
}

/**
 * @brief 使用mahony进行一次计算求得四元数。(当磁力计不可靠时)
 * @param  mpu6050          指向mpu6050_struct的地址，用于读取数据。
 * @param  filter           指向MahonyFilterType的地址，用于数据计算。
 */
void MahonyUpdateAHRSIMU(float dt, mpu6050_struct *mpu6050) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // 仅在加速度计测量有效时计算反馈（避免加速度计归一化中出现 NaN）
  if (!((mpu6050->redirectAcc.x == 0.0f) && (mpu6050->redirectAcc.y == 0.0f) &&
        (mpu6050->redirectAcc.z == 0.0f))) {

    // 使加速度计测量归一化
    recipNorm = InvSqrt(mpu6050->redirectAcc.x * mpu6050->redirectAcc.x +
                        mpu6050->redirectAcc.y * mpu6050->redirectAcc.y +
                        mpu6050->redirectAcc.z * mpu6050->redirectAcc.z);
    mpu6050->redirectAcc.x *= recipNorm;
    mpu6050->redirectAcc.y *= recipNorm;
    mpu6050->redirectAcc.z *= recipNorm;

    // 估计重力方向
    halfvx =
        mahonyFilter.q1 * mahonyFilter.q3 - mahonyFilter.q0 * mahonyFilter.q2;
    halfvy =
        mahonyFilter.q0 * mahonyFilter.q1 + mahonyFilter.q2 * mahonyFilter.q3;
    halfvz = mahonyFilter.q0 * mahonyFilter.q0 - 0.5f +
             mahonyFilter.q3 * mahonyFilter.q3;

    // 误差是估计重力方向 与测量的重力方向的乘积之和
    halfex =
        (mpu6050->redirectAcc.y * halfvz - mpu6050->redirectAcc.z * halfvy);
    halfey =
        (mpu6050->redirectAcc.z * halfvx - mpu6050->redirectAcc.x * halfvz);
    halfez =
        (mpu6050->redirectAcc.x * halfvy - mpu6050->redirectAcc.y * halfvx);

    // 如果启用，计算并应用积分反馈
    if (mahonyFilter.twoKi > 0.0f) {
      // 按 Ki 缩放的积分误差
      mahonyFilter.integralFBx += mahonyFilter.twoKi * halfex / dt;
      mahonyFilter.integralFBy += mahonyFilter.twoKi * halfey / dt;
      mahonyFilter.integralFBz += mahonyFilter.twoKi * halfez / dt;
      mpu6050->redirectGyro.x += mahonyFilter.integralFBx; // 应用积分反馈
      mpu6050->redirectGyro.y += mahonyFilter.integralFBy;
      mpu6050->redirectGyro.z += mahonyFilter.integralFBz;
    } else {
      mahonyFilter.integralFBx = 0.0f; // 防止整体卷绕
      mahonyFilter.integralFBy = 0.0f;
      mahonyFilter.integralFBz = 0.0f;
    }

    // 应用比例反馈
    mpu6050->redirectGyro.x += mahonyFilter.twoKp * halfex;
    mpu6050->redirectGyro.y += mahonyFilter.twoKp * halfey;
    mpu6050->redirectGyro.z += mahonyFilter.twoKp * halfez;
  }

  // 积分四元数的变化率
  mpu6050->redirectGyro.x *= (0.5f / dt); // 预乘公因数
  mpu6050->redirectGyro.y *= (0.5f / dt);
  mpu6050->redirectGyro.z *= (0.5f / dt);
  qa = mahonyFilter.q0;
  qb = mahonyFilter.q1;
  qc = mahonyFilter.q2;
  mahonyFilter.q0 +=
      (-qb * mpu6050->redirectGyro.x - qc * mpu6050->redirectGyro.y -
       mahonyFilter.q3 * mpu6050->redirectGyro.z);
  mahonyFilter.q1 +=
      (qa * mpu6050->redirectGyro.x + qc * mpu6050->redirectGyro.z -
       mahonyFilter.q3 * mpu6050->redirectGyro.y);
  mahonyFilter.q2 +=
      (qa * mpu6050->redirectGyro.y - qb * mpu6050->redirectGyro.z +
       mahonyFilter.q3 * mpu6050->redirectGyro.x);
  mahonyFilter.q3 +=
      (qa * mpu6050->redirectGyro.z + qb * mpu6050->redirectGyro.y -
       qc * mpu6050->redirectGyro.x);

  // 四元数规范化
  recipNorm = InvSqrt(
      mahonyFilter.q0 * mahonyFilter.q0 + mahonyFilter.q1 * mahonyFilter.q1 +
      mahonyFilter.q2 * mahonyFilter.q2 + mahonyFilter.q3 * mahonyFilter.q3);
  mahonyFilter.q0 *= recipNorm;
  mahonyFilter.q1 *= recipNorm;
  mahonyFilter.q2 *= recipNorm;
  mahonyFilter.q3 *= recipNorm;
}

/**
 * @brief 快速平方根倒数算法
 * @param  x               要求算的数字
 * @return float
 */
float InvSqrt(float x) {
  float halfx = 0.5f * x;
  union {
    float f;
    long l;
  } i;
  i.f = x;
  i.l = 0x5f3759df - (i.l >> 1);
  float y = i.f;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/**
 * @brief 使用Mahony算法解算欧拉角(FRD载体系)
 * @param  mpu6050          指向mpu6050_struct的地址，用于读取数据。
 * @param  ist8310          指向ist8310_struct的地址，用于读取数据。
 * @param  filter           指向MahonyFilterType的地址，用于数据计算。
 * @param  mahony           指向MahonyType的地址，用于存储计算结果。
 */
void MahonyGetEuler(float dt, mpu6050_struct *mpu6050, ist8310_struct *ist8310,
                    MahonyType *mahony) {

  MahonyUpdateAHRS(dt, mpu6050, ist8310);
  // 四元数结算弧度
  mahony->FRD.rollRad = atan2f(mahonyFilter.q0 * mahonyFilter.q1 +
                                   mahonyFilter.q2 * mahonyFilter.q3,
                               0.5f - mahonyFilter.q1 * mahonyFilter.q1 -
                                   mahonyFilter.q2 * mahonyFilter.q2);
  mahony->FRD.pitchRad = asinf(-2.0f * (mahonyFilter.q1 * mahonyFilter.q3 -
                                        mahonyFilter.q0 * mahonyFilter.q2));
  mahony->FRD.yawRad = atan2f(mahonyFilter.q1 * mahonyFilter.q2 +
                                  mahonyFilter.q0 * mahonyFilter.q3,
                              0.5f - mahonyFilter.q2 * mahonyFilter.q2 -
                                  mahonyFilter.q3 * mahonyFilter.q3);
  // 弧度转角度
  mahony->FRD.roll = mahony->FRD.rollRad * 57.29578f;
  mahony->FRD.pitch = mahony->FRD.pitchRad * 57.29578f;
  mahony->FRD.yaw = mahony->FRD.yawRad * 57.29578f;

  // 限制航向角0-360°
  if (mahony->FRD.yaw < 0.0f) {
    mahony->FRD.yaw += 180.0f;
  }

  // 只保留整数
  mahony->FRD.roll = floor(mahony->FRD.roll);
  mahony->FRD.pitch = floor(mahony->FRD.pitch);
  mahony->FRD.yaw = floor(mahony->FRD.yaw);
}
