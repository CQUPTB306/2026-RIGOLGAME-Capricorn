// 在主循环中
while (1)
{
    if (Color_IsFrameComplete())
    {
        // 处理绿色 (ID=0)
        if (Color_IsDetected(0))
        {
            int16_t x, y;
            Color_GetCoordinates(0, &x, &y);
            // 例如：控制舵机、输出坐标等
        }

        // 处理红色 (ID=3)
        if (Color_IsDetected(3))
        {
            int16_t x, y;
            Color_GetCoordinates(3, &x, &y);
            // ...
        }

        // 一帧处理完毕，清除标志准备接收下一帧
        Color_ClearFrame();
    }

    // 其他任务...
    HAL_Delay(1);
}