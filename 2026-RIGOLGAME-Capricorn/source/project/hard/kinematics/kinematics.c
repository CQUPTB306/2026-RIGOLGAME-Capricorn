/**
 * @file    kinematics.c
 * @brief   三自由度机械臂逆运动学解算实现
 *Arm_Move
 *  模型：
 *    底座旋转(RZ) + 大臂俯仰(RY) + 小臂俯仰(RY)
 *    L1 = 80mm, L2 = 115mm
 *
 *  坐标系：
 *    原点在底座中心，X+ = 正前方，Y+ = 正左方，Z+ = 向上
 *    底座舵机0° = 正前方(X+)，90° = 正左方(Y+)
 *
 *  用法：
 *    Point3D target = {100.0f, 0.0f, 30.0f};  // 前方100mm, 高度30mm
 *    float s1, s2, s3;
 *    if (IK_Solve(target, &s1, &s2, &s3) == 0) {
 *        Servo_SetAngle(TIM_CHANNEL_1, s1);
 *        Servo_SetAngle(TIM_CHANNEL_2, s2);
 *        Servo_SetAngle(TIM_CHANNEL_3, s3);
 *    }
 *
 *  注意：
 *    - atan2f/sinf/cosf/acosf/sqrtf 需要链接 math.h (-lm)
 *    - 角度制输出，直接可用于 Servo_SetAngle
 *    - θ2>0 = 肘部向下弯折，小臂绝对角度 = θ1-θ2
 */

#include "kinematics.h"
#include <math.h>

/* ==================== 常量 ==================== */
#define PI_F        3.14159265f
#define RAD2DEG     (180.0f / PI_F)    /* 弧度转角度 */
#define DEG2RAD     (PI_F / 180.0f)    /* 角度转弧度 */
#define EPSILON     1e-6f              /* 浮点比较容差 */

/* ==================== 逆运动学解算 ==================== */

int IK_Solve(Point3D target, float *s1, float *s2, float *s3)
{
    float r, d, cos_elbow, elbow_rad, shoulder_rad;
    float base_deg, upper_deg, lower_deg;

    /* ---------- 第一步：计算底座旋转角 θ0 ---------- */
    r = sqrtf(target.x * target.x + target.y * target.y);

    if (r < EPSILON && target.z < 0.0f) {
        /* 目标在原点正下方 → 不可达 */
        return -1;
    }

    if (r < EPSILON) {
        /* 目标在原点正上方（r≈0）→ 奇异点
         * 此时底座角度无意义，保持当前角度或默认0 */
        base_deg = 0.0f;  /* 调用者可覆盖此值 */
    } else {
        /* 用户坐标系：底座0°=正前方(X+)，90°=正左方(Y+)
         * atan2(y, x) 映射：x>0且y=0→0°(前), x>0且y>0→90°(左) */
        base_deg = atan2f(target.y, target.x) * RAD2DEG;
    }

    /* 映射到 0~180° 范围（底座舵机范围限制） */
    if (base_deg < 0.0f) {
        base_deg += 180.0f;
    }
    if (base_deg > BASE_SERVO_MAX) base_deg = BASE_SERVO_MAX;
    if (base_deg < BASE_SERVO_MIN) base_deg = BASE_SERVO_MIN;

    *s1 = base_deg;

    /* ---------- 第二步：检查总臂长是否可达 ---------- */
    d = sqrtf(r * r + target.z * target.z);

    /* 最大伸展距离 = L1 + L2，最小距离 = |L1 - L2| */
    if (d > (ARM_L1 + ARM_L2) * 1.001f) {
        return -1;  /* 超出最大伸展距离 */
    }
    if (d < fabsf(ARM_L1 - ARM_L2) * 0.999f) {
        return -1;  /* 太近，肘部无法弯曲 */
    }

    /* ---------- 第三步：肘部角 θ2（余弦定理） ----------
     *  cos(θ2) = (d² - L1² - L2²) / (2*L1*L2)
     *  θ2 = acos(cosθ2)    → 0~π 范围
     *  正θ2 = 肘部向下弯曲（"elbow-down" 构型） */
    cos_elbow = (d * d - ARM_L1 * ARM_L1 - ARM_L2 * ARM_L2)
              / (2.0f * ARM_L1 * ARM_L2);

    /* 数值保护：防止浮点误差导致 acos 越界 */
    if (cos_elbow > 1.0f) cos_elbow = 1.0f;
    if (cos_elbow < -1.0f) cos_elbow = -1.0f;

    elbow_rad = acosf(cos_elbow);

    /* ---------- 第四步：肩部角 θ1 ----------
     *  ［肘部向下构型］小臂向下弯折（正θ2使末端下沉）
     *  θ1 = atan2(z, r) + atan2(L2*sinθ2, L1 + L2*cosθ2) */
    shoulder_rad = atan2f(target.z, r)
                 + atan2f(ARM_L2 * sinf(elbow_rad),
                          ARM_L1 + ARM_L2 * cosf(elbow_rad));

    /* ---------- 第五步：弧度转角度并限幅 ---------- */
    upper_deg = shoulder_rad * RAD2DEG;
    lower_deg = elbow_rad * RAD2DEG;

    /* 大臂限幅 0~120°（0°=水平） */
    if (upper_deg < UPPER_ARM_SERVO_MIN) upper_deg = UPPER_ARM_SERVO_MIN;
    if (upper_deg > UPPER_ARM_SERVO_MAX) upper_deg = UPPER_ARM_SERVO_MAX;

    /* 小臂限幅 0~120°（0°=伸直） */
    if (lower_deg < LOWER_ARM_SERVO_MIN) lower_deg = LOWER_ARM_SERVO_MIN;
    if (lower_deg > LOWER_ARM_SERVO_MAX) lower_deg = LOWER_ARM_SERVO_MAX;

    *s2 = upper_deg;
    *s3 = lower_deg;

    return 0;  /* IK 成功 */
}

/* ==================== 正运动学解算（验证用） ==================== */

Point3D FK_Solve(float s1, float s2, float s3)
{
    Point3D result = {0, 0, 0};
    float theta0 = s1 * DEG2RAD;
    float theta1 = s2 * DEG2RAD;
    float theta2 = s3 * DEG2RAD;

    /* 大臂 + 小臂（肘部向下构型：正θ2使小臂从大臂方向向下弯折）
     * 小臂绝对角度 = θ1 - θ2 */
    float arm_reach = ARM_L1 * cosf(theta1)
                    + ARM_L2 * cosf(theta1 - theta2);

    /* 分解到三维坐标 */
    result.x = arm_reach * cosf(theta0);
    result.y = arm_reach * sinf(theta0);
    result.z = ARM_L1 * sinf(theta1) + ARM_L2 * sinf(theta1 - theta2);

    return result;
}

/* ==================== 可达性检查 ==================== */

int IK_IsReachable(Point3D target)
{
    float s1, s2, s3;
    return (IK_Solve(target, &s1, &s2, &s3) == 0);
}

float IK_Margin(Point3D target)
{
    float r = sqrtf(target.x * target.x + target.y * target.y);
    float d = sqrtf(r * r + target.z * target.z);

    float max_reach = ARM_L1 + ARM_L2;
    float min_reach = fabsf(ARM_L1 - ARM_L2);

    if (d > max_reach) return max_reach - d;   /* 负值：超出 */
    if (d < min_reach) return d - min_reach;   /* 负值：太近 */

    return (d - min_reach) < (max_reach - d)
           ? (d - min_reach) : (max_reach - d); /* 正值：余量 */
}
