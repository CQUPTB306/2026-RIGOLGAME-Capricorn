#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <stdint.h>

/*
 * ============================================================
 *  三自由度机械臂逆运动学解算
 * ============================================================
 *
 *  坐标系定义：
 *    原点 O 位于底座舵机转轴中心
 *    X轴：指向小车正前方（行进方向）
 *    Y轴：指向小车正左方
 *    Z轴：竖直向上
 *
 *  机械臂结构：
 *    底座舵机 (servo1)：绕 Z 轴旋转，角度 0~180°
 *      0° = 指向 X+ (正前方)，180° = 指向 X- (正后方)
 *
 *    大臂舵机 (servo2)：控制大臂俯仰，长度 L1=80mm，角度 0~120°
 *      0° = 大臂水平（与底座平面平行）
 *      正角度 = 向上抬起
 *
 *    小臂舵机 (servo3)：控制小臂俯仰，长度 L2=115mm，角度 0~120°
 *      0° = 小臂与大臂共线（完全伸直）
 *      正角度 = 向下弯曲（趋向地面方向）
 *
 *  ========== 运动学模型 ==========
 *
 *  正运动学（角度 → 末端坐标）：
 *    reach = L1*cos(θ1) + L2*cos(θ1-θ2)    // θ2>0 小臂向下弯折
 *    x = reach * cos(θ0)
 *    y = reach * sin(θ0)
 *    z = L1*sin(θ1) + L2*sin(θ1-θ2)
 *
 *  逆运动学（坐标 → 角度）：
 *    θ0 = atan2(y, x)
 *    cos(θ2) = (r² + z² - L1² - L2²) / (2*L1*L2)    // r = sqrt(x²+y²)
 *    θ2 = acos(cosθ2)   → 小臂角度（肘部向下弯折）
 *    θ1 = atan2(z, r) + atan2(L2*sinθ2, L1+L2*cosθ2) → 大臂角度
 *
 *  ============================================================
 */

/* ==================== 机械臂尺寸参数 ==================== */
#define ARM_L1         80.0f    /* 大臂长度 (mm) */
#define ARM_L2        115.0f    /* 小臂长度 (mm) */

/* ==================== 舵机角度限位 ==================== */
#define BASE_SERVO_MIN      0.0f
#define BASE_SERVO_MAX    180.0f
#define UPPER_ARM_SERVO_MIN 0.0f
#define UPPER_ARM_SERVO_MAX 120.0f
#define LOWER_ARM_SERVO_MIN 0.0f
#define LOWER_ARM_SERVO_MAX 120.0f

/* ==================== 三维坐标点 ==================== */
typedef struct {
    float x;   /* 前后方向 (mm)，前方为正 */
    float y;   /* 左右方向 (mm)，左方为正 */
    float z;   /* 上下方向 (mm)，向上为正 */
} Point3D;

/**
 * @brief  逆运动学解算
 * @param  target  目标末端三维坐标（单位：mm）
 * @param  s1  输出：底座舵机角度 (0~180)
 * @param  s2  输出：大臂舵机角度 (0~120, 0=水平)
 * @param  s3  输出：小臂舵机角度 (0~120, 0=伸直)
 * @return 0=可达, -1=超出工作空间不可达, -2=奇异点
 */
int IK_Solve(Point3D target, float *s1, float *s2, float *s3);

/**
 * @brief  正运动学解算（验证用）
 * @param  s1  底座舵机角度
 * @param  s2  大臂舵机角度
 * @param  s3  小臂舵机角度
 * @return 末端坐标
 */
Point3D FK_Solve(float s1, float s2, float s3);

/**
 * @brief  检查目标点是否在工作空间内
 * @return 1=可达, 0=不可达
 */
int IK_IsReachable(Point3D target);

/**
 * @brief  计算目标点距工作空间边界的距离
 * @return >0 表示可达时的"余量"，<0 表示超出多少
 */
float IK_Margin(Point3D target);

#endif /* __KINEMATICS_H */
