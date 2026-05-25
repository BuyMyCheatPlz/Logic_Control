# MPU6050 DMP 移植说明

## 已完成的工作

1. **创建了I2C移植层**
   - `Core/Inc/mpu_port.h` - I2C接口头文件
   - `Core/Src/mpu_port.c` - I2C接口实现（使用I2C1）

2. **修改了DMP库文件**
   - `Core/Src/inv_mpu.c` - 添加了STM32平台支持
   - `Core/Src/inv_mpu_dmp_motion_driver.c` - 添加了STM32平台支持

3. **创建了高层封装**
   - `Core/Inc/mpu6050.h` - MPU6050高层接口
   - `Core/Src/mpu6050.c` - MPU6050功能实现

## 编译配置

### 方法1：使用STM32CubeIDE（推荐）

1. 右键点击项目 -> Properties
2. 进入 C/C++ Build -> Settings -> MCU GCC Compiler -> Preprocessor
3. 在 "Define symbols (-D)" 中添加以下宏定义：
   ```
   EMPL_TARGET_STM32
   MPU6050
   ```

### 方法2：修改代码添加宏定义

在 `Core/Inc/main.h` 文件的开头添加：
```c
#define EMPL_TARGET_STM32
#define MPU6050
```

## 使用示例

### 初始化MPU6050 DMP

```c
#include "mpu6050.h"

// 在main函数或FreeRTOS任务中
int result = MPU6050_DMP_Init();
if (result != 0)
{
    // 初始化失败，检查I2C连接和地址
    printf("MPU6050 DMP Init Failed: %d\r\n", result);
}
```

### 读取姿态数据

```c
MPU6050_Quaternion_t quat;
MPU6050_Euler_t euler;

// 读取四元数和欧拉角
if (MPU6050_DMP_GetData(&quat, &euler) == 0)
{
    printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f\r\n", 
           euler.pitch, euler.roll, euler.yaw);
}
```

### 读取原始数据

```c
MPU6050_Gyro_t gyro;
MPU6050_Accel_t accel;
float temp;

// 读取陀螺仪
MPU6050_GetGyro(&gyro);

// 读取加速度计
MPU6050_GetAccel(&accel);

// 读取温度
MPU6050_GetTemperature(&temp);
```

## 硬件连接

- **I2C1_SCL**: PB6
- **I2C1_SDA**: PB7
- **MPU6050地址**: 0x68 (AD0接GND) 或 0x69 (AD0接VCC)

如果使用0x69地址，需要修改 `Core/Inc/mpu6050.h` 中的 `MPU6050_ADDR` 定义。

## 注意事项

1. **I2C速度**: 当前配置为100kHz，如需更高速度可在CubeMX中修改
2. **DMP更新频率**: 默认100Hz，可通过 `dmp_set_fifo_rate()` 修改
3. **FIFO溢出**: 如果读取不及时可能导致FIFO溢出，建议定时读取
4. **坐标系**: 默认使用标准坐标系，如需修改请调整 `dmp_set_orientation()` 参数

## 返回值说明

- `0`: 成功
- `-1`: MPU初始化失败
- `-2`: DMP固件加载失败
- `-3`: 方向设置失败
- `-4`: 特性使能失败
- `-5`: FIFO速率设置失败
- `-6`: DMP使能失败

## 调试建议

1. 使用示波器或逻辑分析仪检查I2C信号
2. 确认MPU6050供电正常（3.3V）
3. 检查I2C上拉电阻（通常4.7kΩ）
4. 使用串口输出调试信息
