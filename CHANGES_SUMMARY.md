# 更改摘要 / Changes Summary

## 完成的功能 / Completed Features

### 1. 添加 STOP 串口指令 / Added STOP Serial Command
- **文件**: [Core/Src/freertos.c](Core/Src/freertos.c)
- **功能**: 新增 `STOP` 命令,用于立即停止所有电机并重置PID控制器
- **使用方法**: 通过串口发送 `STOP` 命令
- **响应**: `STOP OK\r\n`

### 2. 修正左侧车轮旋转方向 / Fixed Left Wheel Rotation Directions
- **文件**: [Core/Src/main.c](Core/Src/main.c)
- **问题**: 左前轮和左后轮的旋转方向与预期相反
- **解决方案**: 在电机初始化后设置左前轮和左后轮的反转标志
```c
Motor_SetInversion(MOTOR_LEFT_REAR, 1);
Motor_SetInversion(MOTOR_LEFT_FRONT, 1);
```

### 3. 添加 MPU6050 Pitch 角度校正 / Added MPU6050 Pitch Angle Correction
- **文件**: [Core/Src/freertos.c](Core/Src/freertos.c)
- **功能**: 利用 MPU6050 的 pitch 角度自动校正车辆航向,保持车头正向
- **原理**:
  - Pitch 为正值: 车头左偏,增加右侧轮速,减少左侧轮速
  - Pitch 为负值: 车头右偏,减少右侧轮速,增加左侧轮速
- **校正系数**: `PITCH_CORRECTION_KP = 2.0f` (可根据实际情况调整)

## 技术细节 / Technical Details

### MPU6050 集成 / MPU6050 Integration
1. 在 `StartTask03` (Proccess_Data 任务) 中初始化 MPU6050 DMP
2. 每个控制周期 (10ms) 读取 MPU6050 的欧拉角数据
3. 根据 pitch 角度计算校正值并应用到各个电机的目标速度

### 校正算法 / Correction Algorithm
```c
float pitch_correction = -mpu_euler.pitch * PITCH_CORRECTION_KP;

// 右侧电机增加校正值
Motor_SetTargetPercent(MOTOR_RIGHT_REAR, base_speed + pitch_correction);
Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, base_speed + pitch_correction);

// 左侧电机减少校正值
Motor_SetTargetPercent(MOTOR_LEFT_REAR, base_speed - pitch_correction);
Motor_SetTargetPercent(MOTOR_LEFT_FRONT, base_speed - pitch_correction);
```

## 串口命令列表 / Serial Command List

| 命令 | 参数 | 功能 | 响应 |
|------|------|------|------|
| `RUN` | -100 到 100 | 设置所有电机速度百分比 | `RUN OK <percent>\r\n` |
| `STOP` | 无 | 停止所有电机 | `STOP OK\r\n` |

## 注意事项 / Notes

1. **编译环境**: 需要使用 ARM GCC 交叉编译工具链 (arm-none-eabi-gcc)
2. **MPU6050 初始化**: 如果 MPU6050 初始化失败,系统仍会正常运行,但不会进行航向校正
3. **校正系数调整**: `PITCH_CORRECTION_KP` 可能需要根据实际车辆特性进行调整
4. **控制周期**: 电机控制和 MPU6050 读取都在 10ms 周期内完成

## 测试建议 / Testing Recommendations

1. 先测试 `STOP` 命令是否能正确停止电机
2. 测试左右轮旋转方向是否已修正
3. 在平坦地面上测试车辆直线行驶,观察航向校正效果
4. 如果车辆出现震荡,适当降低 `PITCH_CORRECTION_KP` 值
