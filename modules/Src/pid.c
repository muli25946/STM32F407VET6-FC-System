#include "pid.h"
#include "arm_math.h"
#include "mahony.h"
#include "oled.h"
#include <math.h>
#include <stdint.h>

// 模式设置，调试?单环?运行?
#define MODE_RATE_DIRECT
#define MODE_PITCH_ONLY
// 动态增益参数定义
#define THROTTLE_RC_MIN 500    // 电机起转油门值
#define THROTTLE_RC_HOVER 5600 // 悬停油门值
#define GAIN_MAX 3.0f          // 增益上限，防止低油门时增益过大
#define GAIN_MIN 0.5f          // 增益下限，防止高油门时增益过小
#define MIX_GAIN 100           // 输出增益比例

float roll_cmd = 0, pitch_cmd = 0, yaw_cmd = 0;

/**
 * @brief 初始化pid控制器
 * @param  pid pid环
 * @param  kp 填入初始化值
 * @param  ki 填入初始化值
 * @param  kd 填入初始化值
 * @param  limit 填入初始化值
 */
void PID_Init(PIDControllerType *pid, float kp, float ki, float kd,
              float limit) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->output_limit = limit;
}

/**
 * @brief PID计算
 * @param  pid pid环
 * @param  error 误差输入
 * @param  dt 时间间隔
 * @return float
 */
float PIDCompute(PIDControllerType *pid, float error, float dt) {
  // 积分项
  pid->integral += error * dt;

  // 积分限幅
  if (pid->ki > 0) {
    float max_integral = pid->output_limit / pid->ki;
    if (pid->integral > max_integral)
      pid->integral = max_integral;
    if (pid->integral < -max_integral)
      pid->integral = -max_integral;
  }

  // 微分项
  float derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;

  // PID输出
  float output =
      pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

  // 限幅
  if (output > pid->output_limit)
    return pid->output_limit;
  if (output < -pid->output_limit)
    return -pid->output_limit;
  return output;
}

/**
 * @brief pid计算(串级)
 * @param  dt 计算间隔/s
 * @param  PIDTarget 期望pid达到的目标
 * @param  mahony 姿态角输入
 * @param  mpu6050 陀螺仪输入
 * @param  roll_angle_pid roll角度环
 * @param  pitch_angle_pid pitch角度环
 * @param  yaw_angle_pid yaw角度环
 * @param  roll_rate_pid roll角速率环
 * @param  pitch_rate_pid pitch角速率环
 * @param  yaw_rate_pid yaw角速率环
 * @param  pidOut pid输出结构体
 */
void PIDControlPosture(float dt, PIDTargetType *PIDTarget, MahonyType *mahony,
                       mpu6050_struct *mpu6050,
                       PIDControllerType *roll_angle_pid,
                       PIDControllerType *pitch_angle_pid,
                       PIDControllerType *yaw_angle_pid,
                       PIDControllerType *roll_rate_pid,
                       PIDControllerType *pitch_rate_pid,
                       PIDControllerType *yaw_rate_pid, PIDOutType *pidOut) {
  // 外环：姿态角误差
  float roll_error = PIDTarget->targetRoll - mahony->FRD.roll;
  float pitch_error = PIDTarget->targetPitch - mahony->FRD.pitch;
  float yaw_error = PIDTarget->targetYaw - mahony->FRD.yaw;

  // 航向角归一化到 [-180, 180]
  if (yaw_error > 180.0f)
    yaw_error -= 360.0f;
  else if (yaw_error < -180.0f)
    yaw_error += 360.0f;

  // 外环PID输出 -> 目标角速度（deg/s）
  PIDTarget->targetRollRate = PIDCompute(roll_angle_pid, roll_error, dt);
  PIDTarget->targetPitchRate = PIDCompute(pitch_angle_pid, pitch_error, dt);
  PIDTarget->targetYawRate = PIDCompute(yaw_angle_pid, yaw_error, dt);

  //  内环：角速度误差deg/s
  float roll_rate_error = PIDTarget->targetRollRate - mpu6050->redirectGyro.x;
  float pitch_rate_error = PIDTarget->targetPitchRate - mpu6050->redirectGyro.y;
  float yaw_rate_error = PIDTarget->targetYawRate - mpu6050->redirectGyro.z;

  roll_cmd = PIDCompute(roll_rate_pid, roll_rate_error, dt);
  pitch_cmd = PIDCompute(pitch_rate_pid, pitch_rate_error, dt);
  yaw_cmd = PIDCompute(yaw_rate_pid, yaw_rate_error, dt);

  // === 调试模式限制 ===
#if defined(MODE_PITCH_ONLY)
  roll_cmd = 0.0f;
  yaw_cmd = 0.0f;
#elif defined(MODE_ROLL_ONLY)
  pitch_cmd = 0.0f;
  yaw_cmd = 0.0f;
#elif defined(MODE_YAW_ONLY)
  roll_cmd = 0.0f;
  pitch_cmd = 0.0f;
#elif defined(MODE_ALL)
#endif

  // === X型混控 ===
  pidOut->outFR =
      PIDTarget->baseThrottle + pitch_cmd + roll_cmd + yaw_cmd; // 前右
  pidOut->outFL =
      PIDTarget->baseThrottle + pitch_cmd - roll_cmd - yaw_cmd; // 前左
  pidOut->outBR =
      PIDTarget->baseThrottle - pitch_cmd + roll_cmd - yaw_cmd; // 后右
  pidOut->outBL =
      PIDTarget->baseThrottle - pitch_cmd - roll_cmd + yaw_cmd; // 后左

  // 限幅（30%-70%油门,始终保持在近似线性区间）
  pidOut->outFR = pidOut->outFR > 1700
                      ? 1700
                      : (pidOut->outFR < 1400 ? 1400 : pidOut->outFR);
  pidOut->outFL = pidOut->outFL > 1700
                      ? 1700
                      : (pidOut->outFL < 1400 ? 1400 : pidOut->outFL);
  pidOut->outBR = pidOut->outBR > 1700
                      ? 1700
                      : (pidOut->outBR < 1400 ? 1400 : pidOut->outBR);
  pidOut->outBL = pidOut->outBL > 1700
                      ? 1700
                      : (pidOut->outBL < 1400 ? 1400 : pidOut->outBL);
}
