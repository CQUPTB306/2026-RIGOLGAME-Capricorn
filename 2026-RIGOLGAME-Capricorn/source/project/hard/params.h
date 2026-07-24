#ifndef __PARAMS_H
#define __PARAMS_H

/**
 * @file    params.h
 * @brief   项目全局调参文件
 *
 *  所有需要调整的参数集中在此文件，方便修改。
 *  坐标系：X+ = 正前方, Y+ = 正右方, Z+ = 向上
 */

/* ================================================================ */
/*  1. 机械臂点位角度 (BASE, UPPER, LOWER)                         */
/*     进入校准模式 → 调臂到目标位 → OLED显示的BASE/UPPER/LOWER值  */
/* ================================================================ */

/* ----- 拾取位 ----- */
#define P_BASE     180    /* PICK */
#define P_UPPER    158
#define P_LOWER     50


/* ----- 中间回弹位1 ----- */
#define Z1_BASE      55    /* BOUNCE */
#define Z1_UPPER     70
#define Z1_LOWER     88

/* ----- 中间回弹位2 ----- */
#define Z2_BASE      81    /* BOUNCE */
#define Z2_UPPER     70
#define Z2_LOWER     88

/* ----- 中间回弹位3 ----- */
#define Z3_BASE      105    /* BOUNCE */
#define Z3_UPPER     70
#define Z3_LOWER     88

/* ----- 车上存放位 ----- */
#define SA_BASE     55    /* STORE A */
#define SA_UPPER   149.5
#define SA_LOWER    80

#define SB_BASE     81    /* STORE B */
#define SB_UPPER   149.5
#define SB_LOWER    80

#define SC_BASE     103    /* STORE C */
#define SC_UPPER   149.5
#define SC_LOWER    80

/* ----- 缓冲位1 ----- */
#define H1_BASE      180    /* RESET */
#define H1_UPPER     70
#define H1_LOWER     88

/* ----- 缓冲位2（正向） ----- */
#define H2_BASE      0    /* 配合正向卸载U_BASE=0 */
#define H2_UPPER     70
#define H2_LOWER     88

/* ----- 缓冲位2（反向，配合反向卸载UN_BASE=175） ----- */
#define HR_BASE     175
#define HR_UPPER     70
#define HR_LOWER     88

/* ----- 卸载位（正向） ----- */
#define U_BASE      0    /* UNLOAD */
#define U_UPPER    158
#define U_LOWER     50

/* ----- 卸载位（反向，掉头后使用） ----- */
#define UN_BASE     175
#define UN_UPPER    158
#define UN_LOWER     50

/* ----- 初始蜷缩位 ----- */
#define R_BASE      85    /* RESET */
#define R_UPPER    110
#define R_LOWER    110

/* ================================================================ */
/*  2. 行驶距离 (cm)                                               */
/*     Run_To_Distance() 接受厘米参数                               */
/* ================================================================ */
#define D_WAREHOUSE   30     /* 启停区→仓库区 */
#define D_PICK_STEP    4     /* 拾取A→B→C间距 */
#define D_UNLOAD_A    50     /* 仓库→卸货A区 */
#define D_TO_B        60     /* A卸→B卸 */
#define D_TO_C        35     /* B卸→C卸 */
#define D_FINISH      60     /* C卸→终点 */

/* ================================================================ */
/*  6. 各顺序分段距离(Run_To_Distance参数)，标★为需实测校准       */
/* ================================================================ */
/* ---- ABC (test8原版) ---- */
#define ABC_TO_A      59    /* 仓库拾取完→A */
#define ABC_A_TO_B    61    /* A→B */
#define ABC_B_TO_C    49    /* B→C */
#define ABC_C_END    100    /* C→终点 */

/* ---- CBA (已验证) ---- */
#define CBA_TO_C     154    /* ★ 仓库→C */
#define CBA_C_TO_B    55    /* C掉头→B */
#define CBA_B_TO_A    60    /* B→A */
#define CBA_A_END    101    /* A→终点 */

/* ---- CAB ---- */
#define CAB_TO_C     152    /* ★ 仓库→C */
#define CAB_C_TO_A   101.5    /* C掉头→A */
#define CAB_A_TO_B    66    /* A再掉头→B */
#define CAB_B_END    151    /* B→终点 */

/* ---- ACB ---- */
#define ACB_TO_A      59    /* ★ 仓库→A */
#define ACB_A_TO_C   109    /* A→C */
#define ACB_C_TO_B    49    /* C掉头→B */
#define ACB_B_END    165    /* B→终点 */

/* ---- BAC ---- */
#define BAC_TO_B     119    /* ★ 仓库→B */
#define BAC_B_TO_A    60    /* B掉头→A */
#define BAC_A_TO_C   109    /* A再掉头→C */
#define BAC_C_END    100    /* C→终点 */

/* ---- BCA ---- */
#define BCA_TO_B     119    /* ★ 仓库→B */
#define BCA_B_TO_C    50    /* B→C */
#define BCA_C_TO_A   110    /* C掉头→A */
#define BCA_A_END    105    /* A→终点 */

/* ================================================================ */
/*  3. 循迹速度与PID                                                */
/* ================================================================ */
#define SPD_FWD     280     /* 前进PWM基速（原400的70%） */
#define SPD_BACK    200     /* 后退PWM基速 */

/* ================================================================ */
/*  4. 机械臂运动速度                                                */
/* ================================================================ */
#define ASTEPS       56     /* 插值步数（越大越平滑） */
#define ADELAY_MS    30     /* 每步延时ms（越大越慢） */

///* ----- 电磁铁延时常量（与 arm.c 中一致） ----- */
//#define ARM_PICK_DELAY_MS       100   /* 拾取等待ms */
//#define ARM_DROP_DELAY_MS       100   /* 卸载等待ms */
//#define ARM_MAGNET_ON_DELAY_MS  100   /* 吸合保持ms */
//#define ARM_MAGNET_OFF_DELAY_MS 80    /* 释放保持ms */

/* ================================================================ */
/*  5. 编码器校准                                                    */
/* ================================================================ */
#define CM_TO_RAW     76    /* 1cm = 76个编码器脉冲（需实测校准） */

#endif /* __PARAMS_H */
