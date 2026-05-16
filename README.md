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
本固件在 `USART2`（115200 8N1）上实现了一组文本命令，用于调试与运动控制。命令以回车或换行结束（CR/LF），命令长度上限为 31 字符，队列深度 4（见 `Core/Src/freertos.c`）。命令解析对前导空格/制表符容错，但请使用大写命令词以保持一致性。

- 基本格式：单行 ASCII 文本，结尾 CR 或 LF。

- 支持的命令（精确匹配实现）
	- `STOP` — 立即停止所有动作，清除运动队列与位置目标，并发送：`STOPPED\r\n`。
	- `LEFT <n>` — 向左移动 `n` 格（`n` 为正整数，缺省为 `1`）。示例：`LEFT 1`。
	- `RIGHT <n>` — 向右移动 `n` 格（`n` 为正整数，缺省为 `1`）。示例：`RIGHT 2`。
	- `FORWARD <n>` — 向前移动 `n` 格（`n` 为正整数，缺省为 `1`）。示例：`FORWARD 1`。
	- `BACKWARD <n>` 或 `BACK <n>` — 向后移动 `n` 格（等价写法），示例：`BACK 1`。
	- `RUN <percent>` — 以统一基础速度运行，`percent` 为 -100 到 100 的整数或整数字符串，例如 `RUN 30`。此命令设置基础速度并不会返回确认文本。
	- `MPU ON` / `MPU OFF` — 启用或禁用 MPU6050 的航向（pitch）校正；命令不会返回确认文本。
	- `PID <kp> <ki> <kd>` — 设置航向 PID 参数（浮点）；此命令用于航向 PID（非车轮速度 PID），命令执行后不会返回确认文本。

- 注意（重要）：
	- 固件只会在动作完成或 `STOP` 时通过 `USART2` 发送 `STOPPED\r\n`；其它命令默认不发送回复。请不要依赖命令确认文本，除非你检查 `STOPPED\r\n`。
	- 命令参数会被严格解析并检查格式/范围，非法参数会被静默忽略（即不返回错误文本）。

- 命令实现要点（参考 `Core/Src/freertos.c`）：
	- 命令队列深度：4，单条最大长度：31 字符。
	- 移动命令会把请求放入运动队列并由控制循环非阻塞执行；单次移动超时时间见 `Core/Inc/config.h` 的 `POSITION_TIMEOUT_MS`。
	- `PID` 命令调整的是航向 PID（用于产生每轮的差速偏置），而非直接调整每轮速度 PID。要在线调整单轮速度 PID，请使用 `Motor_SetPIDGain()` API（需要在固件中添加对应的 UART 控制命令以实现运行时调参）。

- VOFA / 观测：
	- 固件通过 `USART3` 以 VOFA justfloat 格式周期性发送一个 8 浮点数帧（大小由 `VOFA_JUSTFLOAT_FLOATS` 与 `VOFA_JUSTFLOAT_PERIOD_MS` 控制）。帧内顺序为：
		0: 右后（RIGHT_REAR）当前速度 (counts/s)
		1: 右后目标速度 (counts/s)
		2: 左后（LEFT_REAR）当前速度
		3: 左后目标速度
		4: 右前（RIGHT_FRONT）当前速度
		5: 右前目标速度
		6: 左前（LEFT_FRONT）当前速度
		7: 左前目标速度
	- 使用 VOFA 可以在主机端画图以观察每轮的跟踪性能与调参效果。

更多实现细节见 `Core/Src/freertos.c`（命令接收、解析与处理逻辑）。

物理与编码器参数（当前实现中使用的参数，用于计算每格对应的编码器计数）：
- 每格长度：0.30 m
- 轮径：0.06 m（60 mm）
- 编码器：13 lines（每转）
- Quadrature 计数：4（软件按四倍计数计算）
- 减速比：20

如果你的硬件编码器或减速比与上述不同，请在 Flash 前调整 `Core/Src/freertos.c` 中的 `ENCODER_LINES`、`ENCODER_QUADRATURE` 或 `GEAR_RATIO` 常量以匹配真实值。

按轮速度 PID 调参
-----------------
本固件支持为四轮分别设定速度（内环）PID 初始值，方便针对各轮差异单独调参。

- 宏位置：`Core/Inc/config.h`。
- 宏名（按轮）：
	- `VELOCITY_PID_KP_RR` / `VELOCITY_PID_KI_RR` / `VELOCITY_PID_KD_RR` （右后，RIGHT_REAR）
	- `VELOCITY_PID_KP_RF` / `VELOCITY_PID_KI_RF` / `VELOCITY_PID_KD_RF` （右前，RIGHT_FRONT）
	- `VELOCITY_PID_KP_LF` / `VELOCITY_PID_KI_LF` / `VELOCITY_PID_KD_LF` （左前，LEFT_FRONT）
	- `VELOCITY_PID_KP_LR` / `VELOCITY_PID_KI_LR` / `VELOCITY_PID_KD_LR` （左后，LEFT_REAR）

- 使用方法：
	1. 直接在 `Core/Inc/config.h` 修改对应宏值后重新编译并刷入固件（见“构建（本地）”一节）。
	2. 运行时也可以通过固件提供的 API 修改：固件中实现了 `Motor_SetPIDGain(motor_idx, kp, ki, kd)` 函数，可在调试交互或临时命令中调用以在线微调（当前固件未包含通用的 UART 调参命令，需按需添加）。

- 观测：推荐使用固件在 `USART3` 输出的 VOFA justfloat 流（周期由 `VOFA_JUSTFLOAT_PERIOD_MS` 控制），帧内按电机索引发送每轮的“当前速度, 目标速度”，方便用主机端画图对比响应。

- 调参建议（流程）：先单轮调速度环（KP、KD、再 KI），测试不同目标速度，再回到整车场景微调位置外环与航向环。

