# MPU6050 DMP 完整集成指南

## 文件清单

### 新增文件
1. **I2C移植层**
   - `Core/Inc/mpu_port.h` - I2C接口声明
   - `Core/Src/mpu_port.c` - I2C接口实现

2. **高层封装**
   - `Core/Inc/mpu6050.h` - MPU6050 API
   - `Core/Src/mpu6050.c` - MPU6050实现

3. **示例代码**
   - `Core/Src/mpu6050_example.c` - 使用示例

4. **文档**
   - `MPU6050_README.md` - 基础使用说明
   - `MPU6050_INTEGRATION.md` - 本文件

### 已修改文件
1. `Core/Src/inv_mpu.c` - 添加STM32平台支持
2. `Core/Src/inv_mpu_dmp_motion_driver.c` - 添加STM32平台支持
3. `Core/Inc/main.h` - 添加宏定义

### 已存在的DMP库文件
- `Core/Inc/inv_mpu.h`
- `Core/Inc/inv_mpu_dmp_motion_driver.h`
- `Core/Inc/dmpKey.h`
- `Core/Inc/dmpmap.h`

## 快速开始

### 步骤1：验证硬件连接

```
MPU6050    ->    STM32F4
VCC        ->    3.3V
GND        ->    GND
SCL        ->    PB6 (I2C1_SCL)
SDA        ->    PB7 (I2C1_SDA)
AD0        ->    GND (地址0x68) 或 VCC (地址0x69)
```

### 步骤2：在freertos.c中集成

在 `Core/Src/freertos.c` 文件中添加以下代码：

#### 2.1 添加头文件
```c
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
/* USER CODE END Includes */
```

#### 2.2 添加任务句柄和属性（在文件顶部）
```c
/* Definitions for MPU6050Task */
osThreadId_t MPU6050TaskHandle;
const osThreadAttr_t MPU6050Task_attributes = {
  .name = "MPU6050Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
```

#### 2.3 创建任务（在MX_FREERTOS_Init函数中）
```c
void MX_FREERTOS_Init(void) {
  /* ... 其他代码 ... */
  
  /* creation of MPU6050Task */
  MPU6050TaskHandle = osThreadNew(StartMPU6050Task, NULL, &MPU6050Task_attributes);
  
  /* ... 其他代码 ... */
}
```

#### 2.4 实现任务函数
```c
/* USER CODE BEGIN Header_StartMPU6050Task */
/**
* @brief Function implementing the MPU6050Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMPU6050Task */
void StartMPU6050Task(void *argument)
{
  /* USER CODE BEGIN StartMPU6050Task */
  MPU6050_Euler_t euler;
  char msg[64];
  int result;

  osDelay(100);

  result = MPU6050_DMP_Init();
  if (result != 0)
  {
    snprintf(msg, sizeof(msg), "MPU Init Failed: %d\r\n", result);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    vTaskDelete(NULL);
    return;
  }

  HAL_UART_Transmit(&huart2, (uint8_t *)"MPU6050 OK\r\n", 12, 100);

  for(;;)
  {
    if (MPU6050_DMP_GetData(NULL, &euler) == 0)
    {
      snprintf(msg, sizeof(msg), "P:%.1f R:%.1f Y:%.1f\r\n",
               euler.pitch, euler.roll, euler.yaw);
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    }
    osDelay(100);
  }
  /* USER CODE END StartMPU6050Task */
}
```

### 步骤3：编译项目

1. 在STM32CubeIDE中打开项目
2. 右键项目 -> Properties -> C/C++ Build -> Settings
3. MCU GCC Compiler -> Preprocessor -> Define symbols
4. 确认已添加：`EMPL_TARGET_STM32` 和 `MPU6050`（已在main.h中定义）
5. 点击 Build Project

### 步骤4：测试

1. 烧录程序到STM32
2. 打开串口终端（115200波特率）
3. 应该看到类似输出：
   ```
   MPU6050 OK
   P:0.5 R:-1.2 Y:45.3
   P:0.6 R:-1.1 Y:45.4
   ...
   ```

## API参考

### 初始化函数

```c
int MPU6050_DMP_Init(void);
```
- 返回值：0=成功，非0=失败
- 说明：初始化MPU6050并加载DMP固件

### 数据读取函数

```c
int MPU6050_DMP_GetData(MPU6050_Quaternion_t *quat, MPU6050_Euler_t *euler);
```
- 参数：
  - `quat`: 四元数输出（可为NULL）
  - `euler`: 欧拉角输出（可为NULL）
- 返回值：0=成功，非0=失败

```c
int MPU6050_GetGyro(MPU6050_Gyro_t *gyro);
int MPU6050_GetAccel(MPU6050_Accel_t *accel);
int MPU6050_GetTemperature(float *temp);
```

### 数据结构

```c
typedef struct {
    float pitch;  // 俯仰角（度）
    float roll;   // 横滚角（度）
    float yaw;    // 偏航角（度）
} MPU6050_Euler_t;

typedef struct {
    float w, x, y, z;  // 四元数
} MPU6050_Quaternion_t;

typedef struct {
    int16_t x, y, z;  // 陀螺仪原始值
} MPU6050_Gyro_t;

typedef struct {
    int16_t x, y, z;  // 加速度计原始值
} MPU6050_Accel_t;
```

## 常见问题

### Q1: 初始化失败返回-1
**原因**：I2C通信失败
**解决**：
- 检查硬件连接
- 确认I2C地址（0x68或0x69）
- 检查上拉电阻
- 用示波器检查I2C信号

### Q2: 初始化失败返回-2
**原因**：DMP固件加载失败
**解决**：
- 确认dmpKey.h和dmpmap.h文件存在
- 检查Flash空间是否足够

### Q3: 数据读取返回-2
**原因**：FIFO无数据或溢出
**解决**：
- 增加读取频率
- 检查任务调度是否正常

### Q4: 角度漂移
**原因**：DMP需要校准
**解决**：
- 上电后保持传感器静止几秒
- 启用陀螺仪自动校准功能（已默认启用）

## 性能优化

### 降低CPU占用
```c
// 降低DMP更新率到50Hz
dmp_set_fifo_rate(50);
mpu_set_sample_rate(50);
```

### 减少串口输出
```c
// 每100ms输出一次而不是每10ms
osDelay(100);
```

### 使用中断方式
可以配置MPU6050的INT引脚，在数据就绪时触发中断，避免轮询。

## 坐标系说明

默认坐标系（芯片正面朝上，文字正向）：
- X轴：指向右侧
- Y轴：指向前方
- Z轴：指向上方

如需修改坐标系，在 `MPU6050_DMP_Init()` 中修改：
```c
// 示例：旋转180度
dmp_set_orientation(
    inv_orientation_matrix_to_scalar(
        (signed char[9]){-1, 0, 0,
                         0, -1, 0,
                         0, 0, 1}));
```

## 进阶功能

### 启用手势识别
```c
dmp_enable_feature(DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT);
mpu_register_tap_cb(tap_callback);
```

### 启用低功耗模式
```c
mpu_lp_accel_mode(40);  // 40Hz低功耗加速度计模式
```

## 技术支持

如遇到问题，请检查：
1. 硬件连接是否正确
2. I2C通信是否正常（可用逻辑分析仪）
3. 编译宏定义是否正确
4. FreeRTOS堆栈是否足够

参考文件：
- `MPU6050_README.md` - 基础说明
- `mpu6050_example.c` - 完整示例
- `inv_mpu.h` - 底层API
