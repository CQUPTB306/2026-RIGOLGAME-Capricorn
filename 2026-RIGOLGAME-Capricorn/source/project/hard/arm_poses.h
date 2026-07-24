#ifndef __ARM_POSES_H
#define __ARM_POSES_H

/**
 * @file    arm_poses.h
 * @brief   机械臂点位角度硬编码（不使用IK模型）
 *
 *  每个点位 = (base_angle, upper_angle, lower_angle) 三角度
 *
 *  坐标系：X+ = 正前方, Y+ = 正右方, Z+ = 向上
 *
 *  ⚠️ 以下角度值占位，需实测后替换为校准模式中读取的实际角度：
 *     进入校准模式 → 调臂到目标位 → 读取 OLED 显示的 BASE/UPPER/LOWER
 *
 *  使用说明：
 *    在 calibration 模式中调好臂位置，
 *    记下 OLED 显示的三个角度值，
 *    替换下面对应的宏定义即可。
 */

/* ==================== 拾取阶段（仓库区） ==================== */

/* 拾取位置（A/B/C共用同一个拾取位） */
#define POS_PICK_BASE    0   /* TODO: 替换为实际校准角度 */
#define POS_PICK_UPPER   0
#define POS_PICK_LOWER   0

/* 中间回弹位置（每次拾取/卸载后回到此位） */
#define POS_BOUNCE_BASE     0   /* TODO */
#define POS_BOUNCE_UPPER    0
#define POS_BOUNCE_LOWER    0

/* ==================== 车上存放位置 ==================== */

/* 存A */
#define POS_STORE_A_BASE    0   /* TODO */
#define POS_STORE_A_UPPER   0
#define POS_STORE_A_LOWER   0

/* 存B */
#define POS_STORE_B_BASE    0   /* TODO */
#define POS_STORE_B_UPPER   0
#define POS_STORE_B_LOWER   0

/* 存C */
#define POS_STORE_C_BASE    0   /* TODO */
#define POS_STORE_C_UPPER   0
#define POS_STORE_C_LOWER   0

/* ==================== 卸载阶段（卸货区） ==================== */

/* 卸载位置（A/B/C共用） */
#define POS_UNLOAD_BASE    0   /* TODO */
#define POS_UNLOAD_UPPER   0
#define POS_UNLOAD_LOWER   0

/* ==================== 初始位 ==================== */
#define POS_RESET_BASE    90   /* 默认蜷缩在右侧 */
#define POS_RESET_UPPER  110
#define POS_RESET_LOWER  110

#endif /* __ARM_POSES_H */
