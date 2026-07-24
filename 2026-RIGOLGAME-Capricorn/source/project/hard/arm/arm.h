#ifndef __ARM_H
#define __ARM_H

#include "main.h"
#include "kinematics.h"

/* ==================== 机械臂点位枚举（IK版，保留兼容） ==================== */
typedef enum {
    ARM_POINT_RESET = 0,
    ARM_POINT_ALIGN,
    ARM_POINT_PICK_A,
    ARM_POINT_PICK_B,
    ARM_POINT_PICK_C,
    ARM_POINT_STORE,
    ARM_POINT_DROP
} ArmPoint;

/* ==================== 点位枚举（直控角度版） ==================== */
typedef enum {
    ARM_POSE_RESET,
    ARM_POSE_PICK,
    ARM_POSE_BOUNCE_A,
    ARM_POSE_BOUNCE_B,  /* 与BOUNCE_A连续，可用BOUNCE_A+idx */
    ARM_POSE_BOUNCE_C,
    ARM_POSE_STORE_A,
    ARM_POSE_STORE_B,
    ARM_POSE_STORE_C,
    ARM_POSE_UNLOAD,
    ARM_POSE_H1,
    ARM_POSE_H2
} ArmPose;

/* ==================== 函数声明 ==================== */
void Arm_Init(void);
void Arm_MoveToPoint(ArmPoint point);
void Arm_PickSequence(uint8_t goods_index);
void Arm_DropSequence(void);
void Arm_SetDoneCallback(void (*cb)(void));
uint8_t Arm_IsDone(void);
void Arm_ClearDoneFlag(void);

void Arm_MoveToXYZ(Point3D target);
void Arm_SetPointXYZ(ArmPoint point, Point3D xyz);

/* 直控舵机角度（绕过IK模型） */
void Arm_MoveToAngles(float base_deg, float upper_deg, float lower_deg);
void Arm_GotoPose(ArmPose pose);

void Magnet_On(void);
void Magnet_Off(void);

/* 校准模式角度同步 */
void Arm_SyncFromCalib(float s1, float s2, float s3);

#endif
