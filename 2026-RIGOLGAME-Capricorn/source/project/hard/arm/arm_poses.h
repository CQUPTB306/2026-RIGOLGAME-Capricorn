#ifndef __ARM_POSES_H
#define __ARM_POSES_H

/**
 * @file    arm_poses.h
 * @brief   机械臂点位角度（从 params.h 读取）
 *
 *  所有可调参数集中在 hard/params.h
 *  调参只改 params.h 即可
 */

#include "params.h"

/* 从 params.h 映射到点位名称 */
#define POS_PICK_BASE    P_BASE
#define POS_PICK_UPPER   P_UPPER
#define POS_PICK_LOWER   P_LOWER

/* 各物块专属回弹位（对应params.h中的Z1/Z2/Z3） */
#define POS_BOUNCE_A_BASE   Z1_BASE
#define POS_BOUNCE_A_UPPER  Z1_UPPER
#define POS_BOUNCE_A_LOWER  Z1_LOWER

#define POS_BOUNCE_B_BASE   Z2_BASE
#define POS_BOUNCE_B_UPPER  Z2_UPPER
#define POS_BOUNCE_B_LOWER  Z2_LOWER

#define POS_BOUNCE_C_BASE   Z3_BASE
#define POS_BOUNCE_C_UPPER  Z3_UPPER
#define POS_BOUNCE_C_LOWER  Z3_LOWER

#define POS_STORE_A_BASE  SA_BASE
#define POS_STORE_A_UPPER SA_UPPER
#define POS_STORE_A_LOWER SA_LOWER

#define POS_STORE_B_BASE  SB_BASE
#define POS_STORE_B_UPPER SB_UPPER
#define POS_STORE_B_LOWER SB_LOWER

#define POS_STORE_C_BASE  SC_BASE
#define POS_STORE_C_UPPER SC_UPPER
#define POS_STORE_C_LOWER SC_LOWER

#define POS_UNLOAD_BASE   U_BASE
#define POS_UNLOAD_UPPER  U_UPPER
#define POS_UNLOAD_LOWER  U_LOWER

#define POS_H1_BASE   H1_BASE
#define POS_H1_UPPER  H1_UPPER
#define POS_H1_LOWER  H1_LOWER

#define POS_H2_BASE   H2_BASE
#define POS_H2_UPPER  H2_UPPER
#define POS_H2_LOWER  H2_LOWER

#define POS_RESET_BASE    R_BASE
#define POS_RESET_UPPER   R_UPPER
#define POS_RESET_LOWER   R_LOWER

#endif /* __ARM_POSES_H */
