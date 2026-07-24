#ifndef __ARM_H
#define __ARM_H

#include "main.h"


typedef enum {
    ARM_POINT_RESET = 0,
    ARM_POINT_ALIGN,
    ARM_POINT_PICK_A,
    ARM_POINT_PICK_B,
    ARM_POINT_PICK_C,
    ARM_POINT_STORE,
    ARM_POINT_DROP
} ArmPoint;

void Arm_Init(void);
void Arm_MoveToPoint(ArmPoint point);
void Arm_PickSequence(uint8_t goods_index); // 0:A,1:B,2:C
void Arm_DropSequence(void);
void Arm_SetDoneCallback(void (*cb)(void));
uint8_t Arm_IsDone(void);
void Arm_ClearDoneFlag(void);


void Magnet_On(void);
void Magnet_Off(void);

#endif