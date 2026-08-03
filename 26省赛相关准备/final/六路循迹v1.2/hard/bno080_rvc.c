/*
 * bno080_rvc.c - BNO080 UART-RVC 协议解码实现
 *
 * 内部维护一个 256 字节的环形 FIFO,
 * 由 UART 中断收字节填充, main loop 中解析。
 */

#include "bno080_rvc.h"

/* ========== 内部 FIFO ========== */
#define RX_FIFO_SIZE    256

static volatile uint8_t  rx_fifo[RX_FIFO_SIZE];
static volatile uint16_t rx_fifo_wr = 0;
static volatile uint16_t rx_fifo_rd = 0;

void bno080_rvc_rx_isr(uint8_t byte) {
    uint16_t next = (rx_fifo_wr + 1) % RX_FIFO_SIZE;
    if (next != rx_fifo_rd) {                   // 非满
        rx_fifo[rx_fifo_wr] = byte;
        rx_fifo_wr = next;
    }
}

static bool fifo_empty(void) {
    return (rx_fifo_rd == rx_fifo_wr);
}

static uint8_t fifo_dequeue(void) {
    if (fifo_empty()) return 0;
    uint8_t b = rx_fifo[rx_fifo_rd];
    rx_fifo_rd = (rx_fifo_rd + 1) % RX_FIFO_SIZE;
    return b;
}

/* ========== 帧解析状态机 ========== */
typedef enum {
    RVC_IDLE,
    RVC_HEADER2,
    RVC_DATA,
    RVC_DONE
} rvc_state_t;

static rvc_state_t state = RVC_IDLE;
static uint8_t frame[BNO080_RVC_FRAME_SIZE];
static uint8_t frame_idx = 0;

/* ========== 校验和 ========== */
static uint8_t calc_csum(uint8_t *buf) {
    uint8_t sum = 0;
    for (uint8_t i = 2; i < BNO080_RVC_FRAME_SIZE - 1; i++) {
        sum += buf[i];
    }
    return sum;
}

/* ========== 解码 (纯整数, 无浮点) ========== */
static void decode(BNO080_RVC_t *imu, uint8_t *buf) {
    imu->index = buf[2];
    imu->yaw   = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
    imu->pitch = (int16_t)((uint16_t)buf[5] | ((uint16_t)buf[6] << 8));
    imu->roll  = (int16_t)((uint16_t)buf[7] | ((uint16_t)buf[8] << 8));
    imu->accX  = (int16_t)((uint16_t)buf[9] | ((uint16_t)buf[10] << 8));
    imu->accY  = (int16_t)((uint16_t)buf[11] | ((uint16_t)buf[12] << 8));
    imu->accZ  = (int16_t)((uint16_t)buf[13] | ((uint16_t)buf[14] << 8));
    imu->updated = 1;
}

/* ========== 公开接口 ========== */

void bno080_rvc_init(BNO080_RVC_t *imu) {
    imu->yaw   = 0; imu->pitch = 0; imu->roll = 0;
    imu->accX  = 0; imu->accY  = 0; imu->accZ  = 0;
    imu->index   = 0;
    imu->updated = 0;
    imu->crc_ok  = false;

    /* 重置 FIFO & 状态机 */
    rx_fifo_wr = rx_fifo_rd = 0;
    state = RVC_IDLE;
    frame_idx = 0;
}

bool bno080_rvc_parse(BNO080_RVC_t *imu) {
    while (!fifo_empty()) {
        uint8_t byte = fifo_dequeue();

        switch (state) {
        case RVC_IDLE:
            if (byte == BNO080_RVC_HEADER1) state = RVC_HEADER2;
            break;

        case RVC_HEADER2:
            if (byte == BNO080_RVC_HEADER2) {
                frame[0] = BNO080_RVC_HEADER1;
                frame[1] = BNO080_RVC_HEADER2;
                frame_idx = 2;
                state = RVC_DATA;
            } else if (byte == BNO080_RVC_HEADER1) {
                /* 0xAA 重复, 保持 */
            } else {
                state = RVC_IDLE;
            }
            break;

        case RVC_DATA:
            frame[frame_idx++] = byte;
            if (frame_idx >= BNO080_RVC_FRAME_SIZE) {
                state = RVC_DONE;
            }
            break;

        default:
            break;
        }

        if (state == RVC_DONE) {
            uint8_t crc = calc_csum(frame);
            imu->crc_ok = (crc == frame[18]);
            if (imu->crc_ok) {
                decode(imu, frame);
            }
            state = RVC_IDLE;
            frame_idx = 0;
            return true;    // 完成一帧
        }
    }
    return false;
}
