/*
 * bno080_rvc.h - BNO080 UART-RVC 协议解码驱动
 *
 * 接口: UART0 (gy_INST), PA10/PA11, 115200 8N1
 * 协议: UART-RVC 模式 (PS0=1, PS1=0)
 * 输出频率: 100Hz 固定
 *
 * 帧格式 (19 bytes):
 *   0xAA 0xAA | Index(1) |
 *   Yaw_L Yaw_H | Pitch_L Pitch_H | Roll_L Roll_H |
 *   AccX_L AccX_H | AccY_L AccY_H | AccZ_L AccZ_H |
 *   Reserved(3) | Checksum(1)
 *
 * Yaw/Pitch/Roll: int16 little-endian, 0.01°/bit
 * Acc: int16 little-endian, 单位 mg
 * Checksum: 累加 Index ~ Reserved 共 16 字节
 */

#ifndef BNO080_RVC_H
#define BNO080_RVC_H

#include <stdint.h>
#include <stdbool.h>

#define BNO080_RVC_FRAME_SIZE   19
#define BNO080_RVC_HEADER1      0xAA
#define BNO080_RVC_HEADER2      0xAA

/* 解码后的姿态数据 (定点整数, 无浮点) */
typedef struct {
    int16_t yaw;            /* 偏航角, 厘度 (0.01°/bit), ±18000 */
    int16_t pitch;          /* 俯仰角, 厘度, ±9000 */
    int16_t roll;           /* 横滚角, 厘度, ±18000 */
    int16_t accX;           /* X 轴加速度, mg */
    int16_t accY;           /* Y 轴加速度, mg */
    int16_t accZ;           /* Z 轴加速度, mg */

    uint8_t index;          /* 帧序号 0-255 */
    uint8_t updated;        /* 新数据标志, 读取后清零 */
    bool    crc_ok;         /* 校验结果 */
} BNO080_RVC_t;

/* 初始化 */
void bno080_rvc_init(BNO080_RVC_t *imu);

/*
 * 从 UART RX FIFO 中解析一帧
 * 在主循环中周期性调用
 * 返回 true: 解析到新帧; false: 无完整帧
 */
bool bno080_rvc_parse(BNO080_RVC_t *imu);

/* UART 中断中调用: 将接收字节存入内部 FIFO */
void bno080_rvc_rx_isr(uint8_t byte);

#endif
