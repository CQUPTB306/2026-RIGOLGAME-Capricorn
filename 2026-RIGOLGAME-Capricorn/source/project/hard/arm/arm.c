/**
 * @file    arm.c
 * @brief   机械臂控制（逆运动学解算 + TIM8直驱PWM版）
 *
 *  相较于旧版本的变化：
 *    - 删去 PCA9685 I2C 驱动，改用 STM32 TIM8 直接输出 PWM(PC6/7/8)
 *    - 点位用 XYZ 坐标定义，通过逆运动学自动解算舵机角度
 *    - 调整点位只需改 XYZ 三个数
 *
 *  坐标系：
 *    原点在底座中心，X向前(行进方向)，Y向左，Z向上
 *    大臂长80mm，小臂长115mm
 *    底座0°=正前方(X+)，90°=正左方(Y+)
 */

#include "arm.h"
#include "kinematics.h"
#include "servo.h"
#include "arm_poses.h"
#include <math.h>

/* ==================== 速度调参宏 ==================== */
#define ARM_INTERPOLATION_STEPS  50   /* 插值步数（越大越平滑） */
#define ARM_STEP_DELAY_MS        15   /* 每步延时ms（越大越慢） */
#define ARM_TARGET_HOLD_MS      100   /* 到位保持ms */
#define ARM_PICK_DELAY_MS       100   /* 拾取等待ms */
#define ARM_DROP_DELAY_MS       100   /* 卸载等待ms */
#define ARM_MAGNET_ON_DELAY_MS  100   /* 吸合保持ms */
#define ARM_MAGNET_OFF_DELAY_MS 80    /* 释放保持ms */

/* ==================== 舵机通道映射 ====================
 *  实际接线：PC6 → 底座舵机
 *            PC7 → 大臂舵机
 *            PC8 → 小臂舵机
 *  ============================================================ */
#define SERVO_BASE    TIM_CHANNEL_1   /* PC6 = TIM8 CH1 */
#define SERVO_UPPER   TIM_CHANNEL_2   /* PC7 = TIM8 CH2 */
#define SERVO_LOWER   TIM_CHANNEL_3   /* PC8 = TIM8 CH3 */

/* ==================== 点位定义（XYZ毫米坐标） ====================
 *
 *  坐标系（以底座中心为原点）：
 *    X+ = 小车正前方（行进方向，编码器正方向）
 *    Y+ = 小车正左方
 *    Z+ = 竖直向上（底座离地面约85mm，地面在Z≈-85mm）
 *
 *  底座舵机角度映射：
 *    0°  = 指向X+（正前方）
 *    90° = 指向Y+（正左方）
 *    180°= 指向X-（正后方）
 *
 *  约束：
 *    大臂角度 0~120°，小臂角度 0~120°
 *    最大伸展距离 195mm (L1+L2)，最小 35mm (|L1-L2|)
 *    Y<0(右方)不可达（底座0~180°只能覆盖前+左+后半圆）
 *
 *  注意：以下坐标为估算值，实际需根据地图微调
 *  ============================================================ */
static Point3D arm_points_xyz[] = {
    /* RESET     */  {  0.0f,  88.0f,  75.0f },  /* 蜷缩：左方88，上方75 */
    /* ALIGN     */  {120.0f,   0.0f, -10.0f },  /* 对准位：前方120，近地 */
    /* PICK_A    */  {130.0f,  30.0f, -55.0f },  /* 拾取A：前130偏左30，地面 */
    /* PICK_B    */  {140.0f,  15.0f, -55.0f },  /* 拾取B：前140偏左15，地面 */
    /* PICK_C    */  {130.0f,   0.0f, -55.0f },  /* 拾取C：正前130，地面 */
    /* STORE     */  {  0.0f,  80.0f,  80.0f },  /* 搬运：左方80，上方80 */
    /* DROP      */  {140.0f,   0.0f, -55.0f },  /* 卸货位：前方140，地面 */
};

/* IK解算缓存：当前三个舵机的实际角度 */
static float g_current_s1 = 0.0f;
static float g_current_s2 = 0.0f;
static float g_current_s3 = 0.0f;

/* 完成标志与回调 */
static uint8_t arm_done = 0;
static void (*done_callback)(void) = NULL;

/* ==================== 初始化 ==================== */

void Arm_Init(void)
{
    /* Servo_Init 已在 Init_ALL() 中调用，此处仅设初始角度 */
    Point3D reset = arm_points_xyz[ARM_POINT_RESET];
    IK_Solve(reset, &g_current_s1, &g_current_s2, &g_current_s3);
    Servo_SetAngle(SERVO_BASE,  g_current_s1);
    Servo_SetAngle(SERVO_UPPER, g_current_s2);
    Servo_SetAngle(SERVO_LOWER, g_current_s3);
    
}

/* ==================== IK驱动的插值运动 ==================== */

void Arm_MoveToXYZ(Point3D target)
{
    float t_s1, t_s2, t_s3;

    if (IK_Solve(target, &t_s1, &t_s2, &t_s3) != 0)
        return;  /* 不可达，保持原位 */

    /* 关节空间线性插值 */
    for (int i = 0; i <= ARM_INTERPOLATION_STEPS; i++)
    {
        float ratio = (float)i / (float)ARM_INTERPOLATION_STEPS;
        float o0 = g_current_s1 + (t_s1 - g_current_s1) * ratio;
        float o1 = g_current_s2 + (t_s2 - g_current_s2) * ratio;
        float o2 = g_current_s3 + (t_s3 - g_current_s3) * ratio;

        Servo_SetAngle(SERVO_BASE,  o0);
        Servo_SetAngle(SERVO_UPPER, o1);
        Servo_SetAngle(SERVO_LOWER, o2);
        HAL_Delay(ARM_STEP_DELAY_MS);
    }

    g_current_s1 = t_s1;
    g_current_s2 = t_s2;
    g_current_s3 = t_s3;

    HAL_Delay(ARM_TARGET_HOLD_MS);
}

/* ==================== 预设点位运动 ==================== */

void Arm_MoveToPoint(ArmPoint point)
{
    if ((int)point >= 0 && (int)point < 7)
        Arm_MoveToXYZ(arm_points_xyz[point]);
}

/* ==================== 修改预设点位 ==================== */

void Arm_SetPointXYZ(ArmPoint point, Point3D xyz)
{
    if ((int)point >= 0 && (int)point < 7)
        arm_points_xyz[point] = xyz;
}

/* ==================== 拾取序列 ==================== */

void Arm_PickSequence(uint8_t goods_index)
{
    ArmPoint pick_point = ARM_POINT_PICK_A + goods_index;

    Arm_MoveToPoint(pick_point);
    HAL_Delay(ARM_PICK_DELAY_MS);

    
    HAL_Delay(ARM_MAGNET_ON_DELAY_MS);

    Arm_MoveToPoint(ARM_POINT_STORE);

    arm_done = 1;
    if (done_callback) done_callback();
}

/* ==================== 卸载序列 ==================== */

void Arm_DropSequence(void)
{
    Arm_MoveToPoint(ARM_POINT_DROP);
    HAL_Delay(ARM_DROP_DELAY_MS);

    
    HAL_Delay(ARM_MAGNET_OFF_DELAY_MS);

    /* 抖动防粘连 */
    for (int k = 0; k < 2; k++) {
           HAL_Delay(10);
          HAL_Delay(30);
    }

    Arm_MoveToPoint(ARM_POINT_RESET);

    arm_done = 1;
    if (done_callback) done_callback();
}

/* ==================== 电磁铁控制 ==================== */

void Magnet_On(void)  { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET); }
void Magnet_Off(void) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); }


/* ==================== 回调 ==================== */

void Arm_SetDoneCallback(void (*cb)(void)) { done_callback = cb; }
uint8_t Arm_IsDone(void) { return arm_done; }
void Arm_ClearDoneFlag(void) { arm_done = 0; }

/* ==================== 校准模式角度同步 ==================== */

void Arm_SyncFromCalib(float s1, float s2, float s3)
{
    /* 校准模式下用户用按键逐度调了舵机角度，但 arm.c 内部的
     * g_current_s1/s2/s3 仍为旧值。
     * 退出校准后若直接调用 Arm_MoveToPoint()，插值从旧角度开始，
     * 会导致舵机突跳。
     * 此函数将内部角度缓存同步到实际舵机角度。 */
    g_current_s1 = s1;
    g_current_s2 = s2;
    g_current_s3 = s3;
}

/* ==================== 直控舵机角度（绕过IK） ==================== */

void Arm_MoveToAngles(float base_deg, float upper_deg, float lower_deg)
{
    /* 限幅 */
    if (base_deg  < 0.0f) base_deg  = 0.0f;
    if (base_deg  > 180.0f) base_deg  = 180.0f;
    if (upper_deg < 0.0f) upper_deg = 0.0f;
    if (upper_deg > 180.0f) upper_deg = 180.0f;
    if (lower_deg < 0.0f) lower_deg = 0.0f;
    if (lower_deg > 180.0f) lower_deg = 180.0f;

    /* 使用 params.h 定义的插值参数 */
    int steps = ASTEPS;
    for (int i = 0; i <= steps; i++)
    {
        float r = (float)i / (float)steps;
        float b = g_current_s1 + (base_deg  - g_current_s1) * r;
        float u = g_current_s2 + (upper_deg - g_current_s2) * r;
        float l = g_current_s3 + (lower_deg - g_current_s3) * r;

        Servo_SetAngle(SERVO_BASE,  b);
        Servo_SetAngle(SERVO_UPPER, u);
        Servo_SetAngle(SERVO_LOWER, l);
        HAL_Delay(ADELAY_MS);
    }

    g_current_s1 = base_deg;
    g_current_s2 = upper_deg;
    g_current_s3 = lower_deg;

    HAL_Delay(100);  /* 到位保持 */
}

/* ==================== 按宏定义点位走位 ==================== */

void Arm_GotoPose(ArmPose pose)
{
    float b, u, l;
    switch (pose)
    {
        case ARM_POSE_RESET:
            b = POS_RESET_BASE; u = POS_RESET_UPPER; l = POS_RESET_LOWER; break;
        case ARM_POSE_PICK:
            b = POS_PICK_BASE; u = POS_PICK_UPPER; l = POS_PICK_LOWER; break;
        case ARM_POSE_BOUNCE_A:
            b = POS_BOUNCE_A_BASE; u = POS_BOUNCE_A_UPPER; l = POS_BOUNCE_A_LOWER; break;
        case ARM_POSE_BOUNCE_B:
            b = POS_BOUNCE_B_BASE; u = POS_BOUNCE_B_UPPER; l = POS_BOUNCE_B_LOWER; break;
        case ARM_POSE_BOUNCE_C:
            b = POS_BOUNCE_C_BASE; u = POS_BOUNCE_C_UPPER; l = POS_BOUNCE_C_LOWER; break;
        case ARM_POSE_STORE_A:
            b = POS_STORE_A_BASE; u = POS_STORE_A_UPPER; l = POS_STORE_A_LOWER; break;
        case ARM_POSE_STORE_B:
            b = POS_STORE_B_BASE; u = POS_STORE_B_UPPER; l = POS_STORE_B_LOWER; break;
        case ARM_POSE_STORE_C:
            b = POS_STORE_C_BASE; u = POS_STORE_C_UPPER; l = POS_STORE_C_LOWER; break;
        case ARM_POSE_UNLOAD:
            b = POS_UNLOAD_BASE; u = POS_UNLOAD_UPPER; l = POS_UNLOAD_LOWER; break;
        case ARM_POSE_H1:
            b = POS_H1_BASE; u = POS_H1_UPPER; l = POS_H1_LOWER; break;
        case ARM_POSE_H2:
            b = POS_H2_BASE; u = POS_H2_UPPER; l = POS_H2_LOWER; break;
        default:
            b = POS_RESET_BASE; u = POS_RESET_UPPER; l = POS_RESET_LOWER; break;
    }
    Arm_MoveToAngles(b, u, l);
}
