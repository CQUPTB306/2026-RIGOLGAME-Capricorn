#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#include <stdint.h>
#include "params.h"

/* ==================== 卸载模式枚举 ==================== */
typedef enum {
    MODE_ABC = 0,
    MODE_CBA,
    MODE_CAB,
    MODE_ACB,
    MODE_BAC,
    MODE_BCA
} UnloadMode;

/* ==================== 通用状态枚举（覆盖全部6种模式） ==================== */
typedef enum {
    STATE_IDLE = 0,
    STATE_MODE_SELECT,        /* 选择模式 */
    STATE_READY,
    /* 拾取（通用） */
    STATE_GO_WAREHOUSE,
    STATE_PICK_A,
    STATE_GO_6CM_1,
    STATE_PICK_B,
    STATE_GO_6CM_2,
    STATE_PICK_C,
    /* 卸载（覆盖所有顺序需要） */
    STATE_GO_UNLOAD_A,
    STATE_VISION_ALIGN_A,
    STATE_UNLOAD_A,
    STATE_GO_UNLOAD_B,
    STATE_VISION_ALIGN_B,
    STATE_UNLOAD_B,
    STATE_GO_UNLOAD_C,
    STATE_VISION_ALIGN_C,
    STATE_UNLOAD_C,
    STATE_TURN_B,             /* BAC用 */
    STATE_TURN_A,             /* CAB/BAC用 */
    STATE_TURN_C,             /* CBA/CAB/ACB/BCA用 */
    /* 归位 */
    STATE_GO_FINISH,
    STATE_FINISH
} SystemState;

/* ==================== 全局变量 ==================== */
extern SystemState g_state;
extern UnloadMode g_mode;
extern char g_unload_order[4];
extern uint8_t g_pick_index;
extern uint8_t g_unload_count;
extern int32_t g_precise_dist_A;
extern int32_t g_precise_dist_B;
extern int32_t g_precise_dist_C;

/* ==================== 函数声明 ==================== */
void State_Machine(void);
void Start_Order_Setting(void);
void Vision_Align(void);
void OLED_Display_Status(void);

/* 各模式状态机 */
void State_Machine_ABC(void);
void State_Machine_CBA(void);
void State_Machine_CAB(void);
void State_Machine_ACB(void);
void State_Machine_BAC(void);
void State_Machine_BCA(void);

#endif
