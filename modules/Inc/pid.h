#ifndef PID_H
#define PID_H

#include "mahony.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

// 通用PID控制器结构
typedef struct {
  float kp;           // 比例系数
  float ki;           // 积分系数
  float kd;           // 微分系数
  float integral;     // 积分累积值
  float prev_error;   // 上一次误差
  float output_limit; // PID输出限幅
} PIDControllerType;

// PID目标值
typedef struct {
  float targetRoll;
  float targetPitch;
  float targetYaw;
  float targetRollRate;
  float targetPitchRate;
  float targetYawRate;
  float baseThrottle;
} PIDTargetType;

// PID输出油门
typedef struct {
  float outFR; // 前右
  float outFL; // 前左
  float outBR; // 后右
  float outBL; // 后左
} PIDOutType;

// 调试模式选择
typedef enum {
  MODE_ALL = 0,     // 三轴全开
  MODE_PITCH_ONLY,  // 只调Pitch
  MODE_ROLL_ONLY,   // 只调Roll
  MODE_YAW_ONLY,    // 只调Yaw
  MODE_RATE_DIRECT, // 只调内环
} PID_DebugMode;

extern float roll_cmd, pitch_cmd, yaw_cmd; // 调试用

void PID_Init(PIDControllerType *pid, float kp, float ki, float kd,
              float limit);
void PIDControlPosture(float dt, PIDTargetType *PIDTarget, MahonyType *mahony,
                       mpu6050_struct *mpu6050,
                       PIDControllerType *roll_angle_pid,
                       PIDControllerType *pitch_angle_pid,
                       PIDControllerType *yaw_angle_pid,
                       PIDControllerType *roll_rate_pid,
                       PIDControllerType *pitch_rate_pid,
                       PIDControllerType *yaw_rate_pid, PIDOutType *pidOut);

#endif // !PID_H
