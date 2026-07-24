import sensor, image, time
from pyb import UART

# ========== 配置 ==========
# 摄像头分辨率（建议用 QQVGA 或 QVGA，便于裁剪）
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)   # 灰度图
sensor.set_framesize(sensor.QQVGA)       # 160x120
sensor.skip_frames(time=2000)

# 二值化阈值（根据实际光线调整，可使用 IDE 的阈值编辑器）
THRESHOLD = (100, 255)   # 灰度值大于 100 的变为白色，其余黑色

# OLED 尺寸
OLED_W = 128
OLED_H = 64

# 数据量：128 * 64 / 8 = 1024 字节
IMG_SIZE_BYTES = (OLED_W * OLED_H) // 8

# 串口初始化（OpenMV Cam H7 的 UART3 对应 P4(TX), P5(RX)）
uart = UART(3, 115200)   # 波特率 115200
uart.init(115200, bits=8, parity=None, stop=1)

# ========== 辅助函数 ==========
def crop_center(img, crop_w, crop_h):
    """裁剪图像中心区域"""
    w, h = img.width(), img.height()
    x = (w - crop_w) // 2
    y = (h - crop_h) // 2
    return img.copy(roi=(x, y, crop_w, crop_h))

def pack_for_oled(img):
    """
    将二值化图像（128x64）打包成字节数组，顺序为：
    先 Page0 所有列（128字节），再 Page1 所有列，...，直到 Page7。
    每个字节的 bit7 对应页内第1行，bit0 对应第8行。
    """
    packed = bytearray()
    for page in range(8):               # 8页
        for col in range(128):          # 每页128列
            byte_val = 0
            for row_in_page in range(8): # 页内行0~7
                y = page * 8 + row_in_page
                pixel = img.get_pixel(col, y)
                if pixel > 0:
                    byte_val |= (1 << (7 - row_in_page))
            packed.append(byte_val)
    return packed

# ========== 主循环 ==========
print("OpenMV 开始发送二值图像（中心 128x64）...")
while True:
    # 1. 拍摄并二值化
    img = sensor.snapshot()
    img.binary([THRESHOLD])

    # 2. 裁剪中心 128x64 区域
    img_cropped = crop_center(img, OLED_W, OLED_H)

    # 3. 打包成字节数组（1024 字节）
    img_bytes = pack_for_oled(img_cropped)

    # 4. 通过串口发送（固定长度，无帧头帧尾）
    uart.write(img_bytes)

    # 5. 适当延时，控制帧率（可根据 STM32 处理速度调整）
    time.sleep_ms(100)
