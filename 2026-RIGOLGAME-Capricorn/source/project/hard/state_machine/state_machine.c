#include "state_machine.h"
#include "oled.h"
#include "motor.h"
#include "track.h"
#include "encoder.h"
#include "arm.h"
#include "arm_poses.h"
#include "button.h"
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "extern.h"

/* ==================== 全局变量 ==================== */
SystemState g_state = STATE_IDLE;
UnloadMode g_mode = MODE_ABC;
char g_unload_order[4] = "ABC";
uint8_t g_pick_index = 0;
uint8_t g_unload_count = 0;
int32_t g_precise_dist_A = 0;
int32_t g_precise_dist_B = 0;
int32_t g_precise_dist_C = 0;
static char oled_buf[21];

static const char* mode_names[] = {"ABC","CBA","CAB","ACB","BAC","BCA"};

/* ==================== 视觉对准 ==================== */
void Vision_Align(void)
{
    OLED_ShowString(4, 1, "Vision Align...");
    for (int tries = 0; tries < 50; tries++)
    {
        if (Color_IsFrameComplete())
        {
            int16_t cx, cy;
            for (uint8_t id = 0; id < 4; id++)
            {
                if (Color_IsDetected(id))
                {
                    Color_GetCoordinates(id, &cx, &cy);
                    int16_t off = cx - 80;
                    sprintf(oled_buf, "Off:%d", off);
                    OLED_ShowString(4, 1, oled_buf);
                    if (off > 10) { Motor_Set(-50, 50); HAL_Delay(50); Motor_Stop(); }
                    else if (off < -10) { Motor_Set(50, -50); HAL_Delay(50); Motor_Stop(); }
                    Color_ClearFrame();
                    return;
                }
            }
            Color_ClearFrame();
        }
        HAL_Delay(10);
    }
    OLED_ShowString(4, 1, "Vision TO ");
}

/* ==================== 陀螺仪原地旋转 ==================== */
static void TurnByAngle(float delta_deg)
{
    OLED_ShowString(4, 1, "Turning...      ");
    MPU6050_UpdateAngles();
    float target = MPU6050_GetYaw() + delta_deg;
    while (target >= 360.0f) target -= 360.0f;
    while (target < 0.0f)   target += 360.0f;

    /* 冲击启动：高PWM短时冲击，突破静摩擦力 */
    Motor(180, -180);
    HAL_Delay(50);

    turn_flag = 1;
    while (1)
    {
        HAL_Delay(10);
        MPU6050_UpdateAngles();
        float yaw = MPU6050_GetYaw();
        float diff = yaw - target;
        if (diff > 180.0f)      diff -= 360.0f;
        else if (diff < -180.0f) diff += 360.0f;
        if (diff > -3.0f && diff < 3.0f)
        {
            turn_flag = 0;
            Motor_Stop();
            OLED_ShowString(4, 1, "Turn OK!        ");
            HAL_Delay(300);
            return;
        }
    }
}

/* ==================== OLED ==================== */
void OLED_Display_Status(void)
{
    static const char* sn[] = {
        "IDLE","MODE","READY",
        "GO_WARE","PICK_A","GO_6CM","PICK_B","GO_6CM","PICK_C",
        "GO_UN_A","VIS_A","UNL_A",
        "GO_UN_B","VIS_B","UNL_B",
        "GO_UN_C","VIS_C","UNL_C",
        "TRN_B","TRN_A","TRN_C",
        "GO_FIN","FINISH"};
    int idx = (int)g_state;
    if (idx >= 0 && idx < 23) {
        sprintf(oled_buf, "S:%s", sn[idx]);
        OLED_ShowString(1, 1, oled_buf);
    }
    sprintf(oled_buf, "M:%s", mode_names[g_mode]);
    OLED_ShowString(2, 1, oled_buf);
    sprintf(oled_buf, "D:%5ld", Encoder_GetDistance() / 766);
    OLED_ShowString(3, 1, oled_buf);
}

/* ==================== 按键选择模式 ==================== */
void Start_Order_Setting(void)
{
    uint8_t key;
    g_mode = MODE_ABC;
    OLED_Clear();
    OLED_ShowString(1, 1, "Select Mode:");
    
    while (1)
    {
        sprintf(oled_buf, "M:%s  K2=ok", mode_names[g_mode]);
        OLED_ShowString(2, 1, oled_buf);
        OLED_ShowString(3, 1, "K1:next K3:back");
        
        key = Key_Scan();
        if (key == 1) {  /* K1: 下一个模式 */
            g_mode = (UnloadMode)((g_mode + 1) % 6);
            OLED_ShowString(1, 1, "Select Mode:   ");
            HAL_Delay(200);
        }
        else if (key == 2) {  /* K2: 确认 */
            sprintf(oled_buf, "Mode:%s ok!", mode_names[g_mode]);
            OLED_ShowString(1, 1, oled_buf);
            strcpy(g_unload_order, mode_names[g_mode]);
            HAL_Delay(500);
            g_state = STATE_READY;
            OLED_Clear();
            return;
        }
        else if (key == 3) {  /* K3: 回到待机 */
            g_state = STATE_IDLE;
            OLED_Clear();
            return;
        }
        HAL_Delay(10);
    }
}

/* ==================== 机械臂动作（通用） ==================== */
static void Do_PickAndStore(uint8_t idx)
{
    Arm_GotoPose(ARM_POSE_H1);
    Arm_GotoPose(ARM_POSE_PICK);
    HAL_Delay(200);
    Magnet_On();
    HAL_Delay(150);
    Arm_GotoPose(ARM_POSE_H1);
    Arm_GotoPose((ArmPose)(ARM_POSE_BOUNCE_A + idx));
    if (idx == 0) Arm_GotoPose(ARM_POSE_STORE_A);
    else if (idx == 1) Arm_GotoPose(ARM_POSE_STORE_B);
    else Arm_GotoPose(ARM_POSE_STORE_C);
    HAL_Delay(200);
    Magnet_Off();
    HAL_Delay(100);
    Arm_GotoPose((ArmPose)(ARM_POSE_BOUNCE_A + idx));
}

static void Do_Unload(uint8_t idx)
{
    if (idx == 0) Arm_GotoPose(ARM_POSE_STORE_A);
    else if (idx == 1) Arm_GotoPose(ARM_POSE_STORE_B);
    else Arm_GotoPose(ARM_POSE_STORE_C);
    Magnet_On();
    HAL_Delay(150);
    Arm_GotoPose((ArmPose)(ARM_POSE_BOUNCE_A + idx));
    Arm_GotoPose(ARM_POSE_H2);
    Arm_GotoPose(ARM_POSE_UNLOAD);
    HAL_Delay(200);
    Magnet_Off();
    HAL_Delay(150);
    for (int k = 0; k < 2; k++) {
        Magnet_On();   HAL_Delay(10);
        Magnet_Off();  HAL_Delay(30);
    }
    Arm_GotoPose(ARM_POSE_H2);
    Arm_GotoPose((ArmPose)(ARM_POSE_BOUNCE_A + idx));
}

static void Do_UnloadReverse(uint8_t idx)
{
    if (idx == 0) Arm_GotoPose(ARM_POSE_STORE_A);
    else if (idx == 1) Arm_GotoPose(ARM_POSE_STORE_B);
    else Arm_GotoPose(ARM_POSE_STORE_C);
    Magnet_On();
    HAL_Delay(150);
    Arm_GotoPose((ArmPose)(ARM_POSE_BOUNCE_A + idx));
    Arm_MoveToAngles((float)HR_BASE, (float)HR_UPPER, (float)HR_LOWER);
    Arm_MoveToAngles((float)UN_BASE, (float)UN_UPPER, (float)UN_LOWER);
    HAL_Delay(200);
    Magnet_Off();
    HAL_Delay(150);
    for (int k = 0; k < 2; k++) {
        Magnet_On();   HAL_Delay(10);
        Magnet_Off();  HAL_Delay(30);
    }
    Arm_MoveToAngles((float)HR_BASE, (float)HR_UPPER, (float)HR_LOWER);
    Arm_GotoPose((ArmPose)(ARM_POSE_BOUNCE_A + idx));
}

/* ==================== 主状态机调度 ==================== */
void State_Machine(void)
{
    OLED_Display_Status();
    
    /* 模式选择阶段由 Start_Order_Setting 处理 */
    if (g_state == STATE_IDLE) {
        uint8_t key = Key_Scan();
        OLED_ShowString(4, 1, "K1:start K3:cal");
        if (key == 1) {
            g_state = STATE_MODE_SELECT;
            Start_Order_Setting();
        }
        return;
    }
    if (g_state == STATE_MODE_SELECT) return;  /* Start_Order_Setting阻塞中 */
    
    /* 根据当前模式调度对应状态机 */
    switch (g_mode)
    {
        case MODE_ABC:  State_Machine_ABC();  break;
        case MODE_CBA:  State_Machine_CBA();  break;
        case MODE_CAB:  State_Machine_CAB();  break;
        case MODE_ACB:  State_Machine_ACB();  break;
        case MODE_BAC:  State_Machine_BAC();  break;
        case MODE_BCA:  State_Machine_BCA();  break;
    }
}

/* ================================================================ */
/*  以下为6种固定顺序状态机                                          */
/* ================================================================ */

/* ==================== ABC（test8原版） ==================== */
void State_Machine_ABC(void)
{
    uint8_t key;
    switch (g_state)
    {
        case STATE_READY:
            OLED_ShowString(4, 1, "K1:Go!         ");
            key = Key_Scan();
            if (key == 1) {
                Encoder_ResetDistance();
                MPU6050_ResetYaw();
                g_pick_index = 0; g_unload_count = 0;
                g_state = STATE_GO_WAREHOUSE;
            }
            break;
        case STATE_GO_WAREHOUSE:
            OLED_ShowString(4, 1, "To Warehouse   ");
            Run_To_Distance(31.2);
            g_state = STATE_PICK_A;
            break;
        case STATE_PICK_A:
            OLED_ShowString(4, 1, "Pick A         ");
            Do_PickAndStore(0);
            g_state = STATE_GO_6CM_1;
            break;
        case STATE_GO_6CM_1:
            OLED_ShowString(4, 1, "Fwd 6cm        ");
            Run_To_Distance(7.39);
            g_state = STATE_PICK_B;
            break;
        case STATE_PICK_B:
            OLED_ShowString(4, 1, "Pick B         ");
            Do_PickAndStore(1);
            g_state = STATE_GO_6CM_2;
            break;
        case STATE_GO_6CM_2:
            OLED_ShowString(4, 1, "Fwd 6cm        ");
            Run_To_Distance(7.39);
            g_state = STATE_PICK_C;
            break;
        case STATE_PICK_C:
            OLED_ShowString(4, 1, "Pick C         ");
            Do_PickAndStore(2);
            g_state = STATE_GO_UNLOAD_A;
            break;
        case STATE_GO_UNLOAD_A:
            OLED_ShowString(4, 1, "To A pos       ");
            Run_To_Distance(ABC_TO_A);
            g_precise_dist_A = Encoder_GetDistance();
            g_state = STATE_UNLOAD_A;
            break;
        case STATE_UNLOAD_A:
            OLED_ShowString(4, 1, "Unload A       ");
            Do_Unload(0);
            g_unload_count = 1;
            g_state = STATE_GO_UNLOAD_B;
            break;
        case STATE_GO_UNLOAD_B:
            OLED_ShowString(4, 1, "To B pos       ");
            Run_To_Distance(ABC_A_TO_B);
            g_precise_dist_B = Encoder_GetDistance();
            g_state = STATE_UNLOAD_B;
            break;
        case STATE_UNLOAD_B:
            OLED_ShowString(4, 1, "Unload B       ");
            Do_Unload(1);
            g_unload_count = 2;
            g_state = STATE_GO_UNLOAD_C;
            break;
        case STATE_GO_UNLOAD_C:
            OLED_ShowString(4, 1, "To C pos       ");
            Run_To_Distance(ABC_B_TO_C);
            g_precise_dist_C = Encoder_GetDistance();
            g_state = STATE_UNLOAD_C;
            break;
        case STATE_UNLOAD_C:
            OLED_ShowString(4, 1, "Unload C       ");
            Do_Unload(2);
            g_unload_count = 3;
            g_state = STATE_GO_FINISH;
            break;
        case STATE_GO_FINISH:
            OLED_ShowString(4, 1, "To Finish      ");
            Run_To_Distance(ABC_C_END);
            g_state = STATE_FINISH;
            break;
        case STATE_FINISH:
            OLED_ShowString(4, 1, "Task Done!     ");
            Motor_Stop();
            HAL_Delay(2000);
            g_state = STATE_IDLE; OLED_Clear();
            break;
        default: g_state = STATE_IDLE; break;
    }
}

/* ==================== CBA ==================== */
void State_Machine_CBA(void)
{
    switch (g_state)
    {
        case STATE_READY:
            OLED_ShowString(4, 1, "K1:Go!         ");
            if (Key_Scan() == 1) {
                Encoder_ResetDistance(); MPU6050_ResetYaw();
                g_pick_index = 0; g_unload_count = 0;
                g_state = STATE_GO_WAREHOUSE;
            }
            break;
        case STATE_GO_WAREHOUSE:
            OLED_ShowString(4, 1, "To Warehouse   ");
            Run_To_Distance(31.2); g_state = STATE_PICK_A; break;
        case STATE_PICK_A:
            OLED_ShowString(4, 1, "Pick A         ");
            Do_PickAndStore(0); g_state = STATE_GO_6CM_1; break;
        case STATE_GO_6CM_1:
            OLED_ShowString(4, 1, "Fwd 6cm        ");
            Run_To_Distance(7.39); g_state = STATE_PICK_B; break;
        case STATE_PICK_B:
            OLED_ShowString(4, 1, "Pick B         ");
            Do_PickAndStore(1); g_state = STATE_GO_6CM_2; break;
        case STATE_GO_6CM_2:
            OLED_ShowString(4, 1, "Fwd 6cm        ");
            Run_To_Distance(7.39); g_state = STATE_PICK_C; break;
        case STATE_PICK_C:
            OLED_ShowString(4, 1, "Pick C         ");
            Do_PickAndStore(2); g_state = STATE_GO_UNLOAD_C; break;
        case STATE_GO_UNLOAD_C:
            OLED_ShowString(4, 1, "To C pos       ");
            Run_To_Distance(CBA_TO_C);
            g_precise_dist_C = Encoder_GetDistance();
            g_state = STATE_TURN_C; break;
        case STATE_TURN_C:
            TurnByAngle(180); g_state = STATE_UNLOAD_C; break;
        case STATE_UNLOAD_C:
            OLED_ShowString(4, 1, "Unload C       ");
            Do_UnloadReverse(2);
            g_unload_count = 1; g_state = STATE_GO_UNLOAD_B; break;
        case STATE_GO_UNLOAD_B:
            OLED_ShowString(4, 1, "To B pos       ");
            Run_To_Distance(CBA_C_TO_B);
            g_precise_dist_B = Encoder_GetDistance();
            g_state = STATE_UNLOAD_B; break;
        case STATE_UNLOAD_B:
            OLED_ShowString(4, 1, "Unload B       ");
            Do_UnloadReverse(1);
            g_unload_count = 2; g_state = STATE_GO_UNLOAD_A; break;
        case STATE_GO_UNLOAD_A:
            OLED_ShowString(4, 1, "To A pos       ");
            Run_To_Distance(CBA_B_TO_A);
            g_precise_dist_A = Encoder_GetDistance();
            g_state = STATE_UNLOAD_A; break;
        case STATE_UNLOAD_A:
            OLED_ShowString(4, 1, "Unload A       ");
            Do_UnloadReverse(0);
            g_unload_count = 3; g_state = STATE_GO_FINISH; break;
        case STATE_GO_FINISH:
            OLED_ShowString(4, 1, "To Finish      ");
            Run_To_Distance(CBA_A_END);
            g_state = STATE_FINISH; break;
        case STATE_FINISH:
            OLED_ShowString(4, 1, "Task Done!     ");
            Motor_Stop(); HAL_Delay(2000);
            g_state = STATE_IDLE; OLED_Clear(); break;
        default: g_state = STATE_IDLE; break;
    }
}

/* ==================== CAB ==================== */
void State_Machine_CAB(void)
{
    switch (g_state)
    {
        case STATE_READY:
            OLED_ShowString(4, 1, "K1:Go!         ");
            if (Key_Scan() == 1) {
                Encoder_ResetDistance(); MPU6050_ResetYaw();
                g_pick_index = 0; g_unload_count = 0;
                g_state = STATE_GO_WAREHOUSE;
            }
            break;
        case STATE_GO_WAREHOUSE:
            Run_To_Distance(31.2); g_state = STATE_PICK_A; break;
        case STATE_PICK_A:
            Do_PickAndStore(0); g_state = STATE_GO_6CM_1; break;
        case STATE_GO_6CM_1:
            Run_To_Distance(7.39); g_state = STATE_PICK_B; break;
        case STATE_PICK_B:
            Do_PickAndStore(1); g_state = STATE_GO_6CM_2; break;
        case STATE_GO_6CM_2:
            Run_To_Distance(7.39); g_state = STATE_PICK_C; break;
        case STATE_PICK_C:
            Do_PickAndStore(2); g_state = STATE_GO_UNLOAD_C; break;
        case STATE_GO_UNLOAD_C:
            OLED_ShowString(4, 1, "To C pos       ");
            Run_To_Distance(CAB_TO_C);
            g_precise_dist_C = Encoder_GetDistance();
            g_state = STATE_TURN_C; break;
        case STATE_TURN_C:
            TurnByAngle(180); g_state = STATE_UNLOAD_C; break;
        case STATE_UNLOAD_C:
            OLED_ShowString(4, 1, "Unload C       ");
            Do_UnloadReverse(2);
            g_unload_count = 1; g_state = STATE_GO_UNLOAD_A; break;
        case STATE_GO_UNLOAD_A:
            OLED_ShowString(4, 1, "To A pos       ");
            Run_To_Distance(CAB_C_TO_A);
            g_precise_dist_A = Encoder_GetDistance();
            g_state = STATE_TURN_A; break;
        case STATE_TURN_A:
            TurnByAngle(180); g_state = STATE_UNLOAD_A; break;
        case STATE_UNLOAD_A:
            OLED_ShowString(4, 1, "Unload A       ");
            Do_Unload(0);
            g_unload_count = 2; g_state = STATE_GO_UNLOAD_B; break;
        case STATE_GO_UNLOAD_B:
            OLED_ShowString(4, 1, "To B pos       ");
            Run_To_Distance(CAB_A_TO_B);
            g_precise_dist_B = Encoder_GetDistance();
            g_state = STATE_UNLOAD_B; break;
        case STATE_UNLOAD_B:
            OLED_ShowString(4, 1, "Unload B       ");
            Do_Unload(1);
            g_unload_count = 3; g_state = STATE_GO_FINISH; break;
        case STATE_GO_FINISH:
            OLED_ShowString(4, 1, "To Finish      ");
            Run_To_Distance(CAB_B_END);
            g_state = STATE_FINISH; break;
        case STATE_FINISH:
            OLED_ShowString(4, 1, "Task Done!     ");
            Motor_Stop(); HAL_Delay(2000);
            g_state = STATE_IDLE; OLED_Clear(); break;
        default: g_state = STATE_IDLE; break;
    }
}

/* ==================== ACB ==================== */
void State_Machine_ACB(void)
{
    switch (g_state)
    {
        case STATE_READY:
            OLED_ShowString(4, 1, "K1:Go!         ");
            if (Key_Scan() == 1) {
                Encoder_ResetDistance(); MPU6050_ResetYaw();
                g_pick_index = 0; g_unload_count = 0;
                g_state = STATE_GO_WAREHOUSE;
            }
            break;
        case STATE_GO_WAREHOUSE:
            Run_To_Distance(31.2); g_state = STATE_PICK_A; break;
        case STATE_PICK_A:
            Do_PickAndStore(0); g_state = STATE_GO_6CM_1; break;
        case STATE_GO_6CM_1:
            Run_To_Distance(7.39); g_state = STATE_PICK_B; break;
        case STATE_PICK_B:
            Do_PickAndStore(1); g_state = STATE_GO_6CM_2; break;
        case STATE_GO_6CM_2:
            Run_To_Distance(7.39); g_state = STATE_PICK_C; break;
        case STATE_PICK_C:
            Do_PickAndStore(2); g_state = STATE_GO_UNLOAD_A; break;
        case STATE_GO_UNLOAD_A:
            OLED_ShowString(4, 1, "To A pos       ");
            Run_To_Distance(ACB_TO_A);
            g_precise_dist_A = Encoder_GetDistance();
            g_state = STATE_UNLOAD_A; break;
        case STATE_UNLOAD_A:
            OLED_ShowString(4, 1, "Unload A       ");
            Do_Unload(0);
            g_unload_count = 1; g_state = STATE_GO_UNLOAD_C; break;
        case STATE_GO_UNLOAD_C:
            OLED_ShowString(4, 1, "To C pos       ");
            Run_To_Distance(ACB_A_TO_C);
            g_precise_dist_C = Encoder_GetDistance();
            g_state = STATE_TURN_C; break;
        case STATE_TURN_C:
            TurnByAngle(180); g_state = STATE_UNLOAD_C; break;
        case STATE_UNLOAD_C:
            OLED_ShowString(4, 1, "Unload C       ");
            Do_UnloadReverse(2);
            g_unload_count = 2; g_state = STATE_GO_UNLOAD_B; break;
        case STATE_GO_UNLOAD_B:
            OLED_ShowString(4, 1, "To B pos       ");
            Run_To_Distance(ACB_C_TO_B);
            g_precise_dist_B = Encoder_GetDistance();
            g_state = STATE_UNLOAD_B; break;
        case STATE_UNLOAD_B:
            OLED_ShowString(4, 1, "Unload B       ");
            Do_UnloadReverse(1);
            g_unload_count = 3; g_state = STATE_GO_FINISH; break;
        case STATE_GO_FINISH:
            OLED_ShowString(4, 1, "To Finish      ");
            Run_To_Distance(ACB_B_END);
            g_state = STATE_FINISH; break;
        case STATE_FINISH:
            OLED_ShowString(4, 1, "Task Done!     ");
            Motor_Stop(); HAL_Delay(2000);
            g_state = STATE_IDLE; OLED_Clear(); break;
        default: g_state = STATE_IDLE; break;
    }
}

/* ==================== BAC ==================== */
void State_Machine_BAC(void)
{
    switch (g_state)
    {
        case STATE_READY:
            OLED_ShowString(4, 1, "K1:Go!         ");
            if (Key_Scan() == 1) {
                Encoder_ResetDistance(); MPU6050_ResetYaw();
                g_pick_index = 0; g_unload_count = 0;
                g_state = STATE_GO_WAREHOUSE;
            }
            break;
        case STATE_GO_WAREHOUSE:
            Run_To_Distance(31.2); g_state = STATE_PICK_A; break;
        case STATE_PICK_A:
            Do_PickAndStore(0); g_state = STATE_GO_6CM_1; break;
        case STATE_GO_6CM_1:
            Run_To_Distance(7.39); g_state = STATE_PICK_B; break;
        case STATE_PICK_B:
            Do_PickAndStore(1); g_state = STATE_GO_6CM_2; break;
        case STATE_GO_6CM_2:
            Run_To_Distance(7.39); g_state = STATE_PICK_C; break;
        case STATE_PICK_C:
            Do_PickAndStore(2); g_state = STATE_GO_UNLOAD_B; break;
        case STATE_GO_UNLOAD_B:
            OLED_ShowString(4, 1, "To B pos       ");
            Run_To_Distance(BAC_TO_B);
            g_precise_dist_B = Encoder_GetDistance();
            g_state = STATE_UNLOAD_B; break;
        case STATE_UNLOAD_B:
            OLED_ShowString(4, 1, "Unload B       ");
            Do_Unload(1);
            g_unload_count = 1; g_state = STATE_TURN_B; break;
        case STATE_TURN_B:
            TurnByAngle(180); g_state = STATE_GO_UNLOAD_A; break;
        case STATE_GO_UNLOAD_A:
            OLED_ShowString(4, 1, "To A pos       ");
            Run_To_Distance(BAC_B_TO_A);
            g_precise_dist_A = Encoder_GetDistance();
            g_state = STATE_UNLOAD_A; break;
        case STATE_UNLOAD_A:
            OLED_ShowString(4, 1, "Unload A       ");
            Do_UnloadReverse(0);
            g_unload_count = 2; g_state = STATE_TURN_A; break;
        case STATE_TURN_A:
            TurnByAngle(180); g_state = STATE_GO_UNLOAD_C; break;
        case STATE_GO_UNLOAD_C:
            OLED_ShowString(4, 1, "To C pos       ");
            Run_To_Distance(BAC_A_TO_C);
            g_precise_dist_C = Encoder_GetDistance();
            g_state = STATE_UNLOAD_C; break;
        case STATE_UNLOAD_C:
            OLED_ShowString(4, 1, "Unload C       ");
            Do_Unload(2);
            g_unload_count = 3; g_state = STATE_GO_FINISH; break;
        case STATE_GO_FINISH:
            OLED_ShowString(4, 1, "To Finish      ");
            Run_To_Distance(BAC_C_END);
            g_state = STATE_FINISH; break;
        case STATE_FINISH:
            OLED_ShowString(4, 1, "Task Done!     ");
            Motor_Stop(); HAL_Delay(2000);
            g_state = STATE_IDLE; OLED_Clear(); break;
        default: g_state = STATE_IDLE; break;
    }
}

/* ==================== BCA ==================== */
void State_Machine_BCA(void)
{
    switch (g_state)
    {
        case STATE_READY:
            OLED_ShowString(4, 1, "K1:Go!         ");
            if (Key_Scan() == 1) {
                Encoder_ResetDistance(); MPU6050_ResetYaw();
                g_pick_index = 0; g_unload_count = 0;
                g_state = STATE_GO_WAREHOUSE;
            }
            break;
        case STATE_GO_WAREHOUSE:
            Run_To_Distance(31.2); g_state = STATE_PICK_A; break;
        case STATE_PICK_A:
            Do_PickAndStore(0); g_state = STATE_GO_6CM_1; break;
        case STATE_GO_6CM_1:
            Run_To_Distance(7.39); g_state = STATE_PICK_B; break;
        case STATE_PICK_B:
            Do_PickAndStore(1); g_state = STATE_GO_6CM_2; break;
        case STATE_GO_6CM_2:
            Run_To_Distance(7.39); g_state = STATE_PICK_C; break;
        case STATE_PICK_C:
            Do_PickAndStore(2); g_state = STATE_GO_UNLOAD_B; break;
        case STATE_GO_UNLOAD_B:
            OLED_ShowString(4, 1, "To B pos       ");
            Run_To_Distance(BCA_TO_B);
            g_precise_dist_B = Encoder_GetDistance();
            g_state = STATE_UNLOAD_B; break;
        case STATE_UNLOAD_B:
            OLED_ShowString(4, 1, "Unload B       ");
            Do_Unload(1);
            g_unload_count = 1; g_state = STATE_GO_UNLOAD_C; break;
        case STATE_GO_UNLOAD_C:
            OLED_ShowString(4, 1, "To C pos       ");
            Run_To_Distance(BCA_B_TO_C);
            g_precise_dist_C = Encoder_GetDistance();
            g_state = STATE_TURN_C; break;
        case STATE_TURN_C:
            TurnByAngle(180); g_state = STATE_UNLOAD_C; break;
        case STATE_UNLOAD_C:
            OLED_ShowString(4, 1, "Unload C       ");
            Do_UnloadReverse(2);
            g_unload_count = 2; g_state = STATE_GO_UNLOAD_A; break;
        case STATE_GO_UNLOAD_A:
            OLED_ShowString(4, 1, "To A pos       ");
            Run_To_Distance(BCA_C_TO_A);
            g_precise_dist_A = Encoder_GetDistance();
            g_state = STATE_UNLOAD_A; break;
        case STATE_UNLOAD_A:
            OLED_ShowString(4, 1, "Unload A       ");
            Do_UnloadReverse(0);
            g_unload_count = 3; g_state = STATE_GO_FINISH; break;
        case STATE_GO_FINISH:
            OLED_ShowString(4, 1, "To Finish      ");
            Run_To_Distance(BCA_A_END);
            g_state = STATE_FINISH; break;
        case STATE_FINISH:
            OLED_ShowString(4, 1, "Task Done!     ");
            Motor_Stop(); HAL_Delay(2000);
            g_state = STATE_IDLE; OLED_Clear(); break;
        default: g_state = STATE_IDLE; break;
    }
}
