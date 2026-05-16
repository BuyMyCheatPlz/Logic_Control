# Logic_Control

简要说明（中文）

概要
- 基于 STM32F4（STM32F407）平台的轮式小车控制固件，使用 FreeRTOS、HAL 驱动和 MPU6050 DMP实现航向（Pitch/Heading）校正与电机控制。

硬件要求
- 主控：STM32F407 系列开发板（工程配置为 STM32F407VET6）。
- IMU：MPU6050（I2C 地址 0x68），建议连接到 I2C1。
- 电机驱动与电源：根据你的底板接线配置。

主要功能
- 电机（Mecanum / 多轮）控制与 PID 调节。
- 使用 MPU6050 DMP 实现姿态（Euler）与航向校正。
- FreeRTOS 任务隔离（MPU6050 读取、控制回路等）。

代码结构（关键文件）
- [Core/Src/main.c](Core/Src/main.c) — 程序入口与任务创建
- [Core/Src/motor.c](Core/Src/motor.c) — 电机控制实现
- [Core/Src/mpu6050.c](Core/Src/mpu6050.c) 与 [Core/Src/mpu_port.c](Core/Src/mpu_port.c) — MPU6050 与移植层
- [Core/Inc/*] — 头文件集合
- [BUILD_SUCCESS.md](BUILD_SUCCESS.md)、[MPU6050_INTEGRATION.md](MPU6050_INTEGRATION.md)、[JITTER_DEBUG_GUIDE.md](JITTER_DEBUG_GUIDE.md) — 项目文档与使用说明

构建（本地）
先确保已安装：`cmake`、`ninja`、`arm-none-eabi` 工具链、`st-flash`（用于刷写）。

示例命令（在仓库根目录）：
```bash
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug
```

刷写固件（示例，按需修改文件名与地址）：
```bash
st-flash write Logic_Control.bin 0x8000000
```

开发与调试
- 本仓库包含 EWARM（Keil）工程文件在 `EWARM/` 目录，可用于使用 IAR/Keil 等 IDE 开发。
- 链接脚本为 `STM32F407XX_FLASH.ld`，启动汇编为 `startup_stm32f407xx.s`。

注意事项
- MPU6050：如果初始化失败，固件仍可运行但不会启用航向校正，详见 [MPU6050_INTEGRATION.md](MPU6050_INTEGRATION.md)。
- 回退与备份：如果需要恢复到当前版本，仓库中已创建备份分支（backup-before-revert-*）。

常见问题与调优
- 惯性传感器数据噪声会影响 PID 行为，参阅 [JITTER_DEBUG_GUIDE.md](JITTER_DEBUG_GUIDE.md) 获取滤波与采样建议。

许可证与作者
- 请根据项目实际情况补充许可证信息与作者联系方式。

更多信息
- 查看项目根目录中的其他文档以获取详细集成与构建步骤。

引脚（Pinout）
按功能列出本固件使用的 MCU 引脚（基于 `Core/Src/*.c` 的初始化配置）：

- MPU6050 (I2C1): PB6 = SCL, PB7 = SDA  （参见 [Core/Src/i2c.c](Core/Src/i2c.c#L70-L79））
- 串口（用于日志/调试）: USART2 — PA2 = TX, PA3 = RX （参见 [Core/Src/usart.c](Core/Src/usart.c#L24-L32) 和 [Core/Src/usart.c](Core/Src/usart.c#L54-L64)）

- 电机 PWM（TIM1）输出：
	- Motor 0 PWM (CH1) -> PE9
	- Motor 1 PWM (CH2) -> PE11
	- Motor 2 PWM (CH3) -> PE13
	- Motor 3 PWM (CH4) -> PE14
	（PWM 引脚映射见 [Core/Src/tim.c](Core/Src/tim.c#L418-L426) 与 `MotorHardware_t` 定义在 [Core/Src/motor.c](Core/Src/motor.c#L34-L44)）

- 电机方向控制 (IN1 / IN2 GPIOs, 在 `motor.c` 中配置):
	- Motor 0: IN1 = PE7, IN2 = PE8
	- Motor 1: IN1 = PA9, IN2 = PA8
	- Motor 2: IN1 = PE12, IN2 = PE10
	- Motor 3: IN1 = PC6, IN2 = PC8
	（参见 [Core/Src/motor.c](Core/Src/motor.c#L36-L44) 与 [Core/Src/gpio.c](Core/Src/gpio.c#L62-L83)）

- 编码器 (TIMx 作为编码器接口，使用 TIM_CH1/CH2 引脚)：
	- Motor 0 encoder (TIM5): PA0 (CH1), PA1 (CH2)  （[Core/Src/tim.c](Core/Src/tim.c#L391-L400)）
	- Motor 1 encoder (TIM2): PA5 (CH1), PB3 (CH2)   （[Core/Src/tim.c](Core/Src/tim.c#L307-L328)）
	- Motor 2 encoder (TIM3): PA6 (CH1), PA7 (CH2)   （[Core/Src/tim.c](Core/Src/tim.c#L343-L352)）
	- Motor 3 encoder (TIM4): PD12 (CH1), PD13 (CH2) （[Core/Src/tim.c](Core/Src/tim.c#L367-L376)）

注：引脚注释来源于 HAL MSPInit 中的 GPIO 注释与 `motor.c` 中的硬件表；在修改硬件连线或使用不同开发板前，请以 `Logic_Control.ioc` 或 STM32CubeMX 配置为准。

串口指令（Serial Commands）
本固件在 `USART2`（115200 8N1）上实现了一组文本命令，用于调试与运动控制。命令以回车或换行结束（CR/LF），命令长度上限为 31 字符，队列深度 4（见 `Core/Src/freertos.c`）。命令使用大写字母。

- 基本格式：单行 ASCII 文本，结尾 CR 或 LF。
- 运动指令（新语法，支持格移动）：
	- `LEFT <n>` — 向左移动 n 格（n 为正整数，缺省为 1）。每格长度 0.30 m。示例：`LEFT 1`。
	- `RIGHT <n>` — 向右移动 n 格（n 为正整数，缺省为 1）。示例：`RIGHT 2`。
	- `FORWARD <n>` — 向前移动 n 格（同上）。示例：`FORWARD 1`。
	- `BACKWARD <n>` 或 `BACK <n>` — 向后移动 n 格（接受 `BACKWARD` 与 `BACK` 两种写法）。示例：`BACKWARD 1`。
	- 行为说明：在收到运动命令后固件会先回传 `<CMD> OK\r\n`（例如 `LEFT OK`），动作完成后会回传 `STOPPED\r\n` 表示已停止并达到目标格数。

- 速度/运行控制：
	- `RUN <percent>` — 设置全车基础速度（-100 到 100），例如 `RUN 30`；返回 `RUN OK <percent>\r\n`。
	- `STOP` — 立即停止所有电机并清除目标速度；返回 `STOP OK\r\n`（同时中断当前格移动）。

- 传感器与 PID：
	- `MPU ON` — 启用 MPU6050 的航向校正，固件会以当前俯仰作为目标并重置航向 PID；返回 `MPU ON\r\n`。
	- `MPU OFF` — 关闭航向校正并重置航向 PID；返回 `MPU OFF\r\n`。
	- `PID <kp> <ki> <kd>` — 设置航向 PID 参数（浮点），例如 `PID 0.5 0.05 0.1`；成功返回 `PID OK <kp> <ki> <kd>\r\n`。

- 其他命令：
  - （无）

- 错误回复说明：
	- `ERR CMD` — 未知命令。
	- `ERR PARAM` — 参数缺失或不合法。
	- `ERR FORMAT` — 参数格式错误。
	- `ERR RANGE` — 数值超出允许范围。

更多实现细节见 `Core/Src/freertos.c`（命令接收、解析与处理逻辑）。

物理与编码器参数（当前实现中使用的参数，用于计算每格对应的编码器计数）：
- 每格长度：0.30 m
- 轮径：0.06 m（60 mm）
- 编码器：13 lines（每转）
- Quadrature 计数：4（软件按四倍计数计算）
- 减速比：20

如果你的硬件编码器或减速比与上述不同，请在 Flash 前调整 `Core/Src/freertos.c` 中的 `ENCODER_LINES`、`ENCODER_QUADRATURE` 或 `GEAR_RATIO` 常量以匹配真实值。
