#include "arm.h"
#include "i2c.h"


static const float arm_points[][3] = {
    {  0, 45, -80},   // RESET
    { 90,  0, -80},   // ALIGN
    { 90, 30, -30},   // PICK_A
    { 90, 30, -30},   // PICK_B (��΢��)
    { 90, 30, -30},   // PICK_C (��΢��)
    { 90, 30, -30},   // STORE (�ƶ�λ)
    { 90, 80, -30}    // DROP
};

static uint8_t arm_done = 0;
static void (*done_callback)(void) = NULL;

void PCA9685_Write(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, 0x80, reg, 1, &data, 1, 100);
}

void Servo_SetAngle(uint8_t ch, float angle)
{
    if(angle > 180) angle = 180;
    if(angle < 0)   angle = 0;
    uint16_t pulse = 500 + (angle / 180.0f) * 2000;
    uint16_t off = (pulse * 4096) / 20000;
    uint8_t da[4] = {0, 0, off & 0xFF, off >> 8};
    HAL_I2C_Mem_Write(&hi2c1, 0x80, 0x06 + 4*ch, 1, da, 4, 100);
}

void Arm_Init(void)
{
    HAL_Delay(100);
    PCA9685_Write(0x00, 0x10);
    PCA9685_Write(0xFE, 121);
    PCA9685_Write(0x00, 0x20);
    Arm_MoveToPoint(ARM_POINT_RESET);
    Magnet_Off();
}

void Arm_MoveToPoint(ArmPoint point)
{
    float t0 = arm_points[point][0];
    float t1 = arm_points[point][1];
    float t2 = arm_points[point][2];
    static float n0 = 0, n1 = 45, n2 = -80;

    for(int i=0; i<=50; i++)
    {
        float o0 = n0 + (t0 - n0)/50.0f * i;
        float o1 = n1 + (t1 - n1)/50.0f * i;
        float o2 = n2 + (t2 - n2)/50.0f * i;
        Servo_SetAngle(0, o0 + 90);
        Servo_SetAngle(1, 90 - o1);
        Servo_SetAngle(2, -o2);
        HAL_Delay(10);
    }
    n0 = t0; n1 = t1; n2 = t2;
    HAL_Delay(300);
}

void Arm_PickSequence(uint8_t goods_index)
{
    ArmPoint pick_point = ARM_POINT_PICK_A + goods_index;

    Arm_MoveToPoint(pick_point);
    HAL_Delay(200);

    Magnet_On();
    HAL_Delay(200);
 
    Arm_MoveToPoint(ARM_POINT_STORE);
    Magnet_Off();   // �����ݴ���
    HAL_Delay(200);
    arm_done = 1;
    if(done_callback) done_callback();
}

void Arm_DropSequence(void)
{

    Arm_MoveToPoint(ARM_POINT_STORE);
    Magnet_On();
    HAL_Delay(300);
  
    Arm_MoveToPoint(ARM_POINT_DROP);
    HAL_Delay(200);
    Magnet_Off();
    HAL_Delay(150);

    for(int k=0;k<2;k++){
        Magnet_On(); HAL_Delay(10);
        Magnet_Off(); HAL_Delay(30);
    }
    Arm_MoveToPoint(ARM_POINT_RESET);
    arm_done = 1;
    if(done_callback) done_callback();
}

void Magnet_On(void)  { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET); }
void Magnet_Off(void) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); }

void Arm_SetDoneCallback(void (*cb)(void)) 
{ 
    done_callback = cb; 
}
uint8_t Arm_IsDone(void)
{
     return arm_done;
}
void Arm_ClearDoneFlag(void)
{ 
    arm_done = 0; 
}