# Logic_Control 代码架构全景

> 本文档以项目领域词汇（Ubiquitous Language）描述各模块职责、调用关系和数据流向，供 AI Agent 导航代码库使用。

---

## 1. 模块地图

```
┌─────────────────────────────────────────────────────────────────┐
│                      入口层 (main.c)                             │
│  HAL_Init → SystemClock_Config → MX_*_Init → Motor_Init         │
│  → osKernelStart (FreeRTOS 调度器接管)                           │
└────────────────────────────┬────────────────────────────────────┘
                             │
        ┌────────────────────┼────────────────────┐
        ▼                    ▼                    ▼
┌───────────────┐  ┌─────────────────┐  ┌──────────────────┐
│  defaultTask  │  │  StartTask02    │  │  StartTask03     │
│  (空闲, 1ms)  │  │  (命令分发,1ms) │  │  (运动控制,10ms) │
└───────────────┘  └────────┬────────┘  └────────┬─────────┘
                            │                     │
                    ┌───────▼────────┐    ┌───────▼──────────┐
                    │ uart2_cmd_queue│    │  Motion_Tick()   │
                    │ (环形缓冲,16深)│    │  Motor_UpdateCtrl│
                    └────────────────┘    └──────────────────┘
```

### 1.1 模块清单

| 模块 | 路径 | 职责 | 关键导出 |
|------|------|------|----------|
| **main** | `Core/Src/main.c` | 硬件初始化，启动调度器 | `main()`, `SystemClock_Config()` |
| **freertos** | `Core/Src/freertos.c` | FreeRTOS 任务、双 UART 协议解析、运动编排、麦克纳姆步进计算 | `HAL_UARTEx_RxEventCallback`, `StartTask02`, `StartTask03` |
| **motor** | `Core/Src/motor.c` | 四轮电机驱动、速度环 PID、位置环 PID、编码器读取、死区补偿 | `Motor_Init()`, `Motor_UpdateControl()`, `Motor_SetPositionTarget()` |
| **soft_filter** | `Core/Src/soft_filter.c` | EMA 低通滤波（带尖峰限幅） | `SoftFilterEMA_Update()` |
| **usart** | `Core/Src/usart.c` | UART2/UART3 硬件初始化（GPIO、DMA、NVIC） | `MX_USART2_UART_Init()`, `MX_USART3_UART_Init()` |
| **tim** | `Core/Src/tim.c` | PWM 定时器 (TIM1) 和编码器定时器 (TIM2-5) 初始化 | `MX_TIM1_Init()` ~ `MX_TIM5_Init()` |
| **gpio/dma/i2c** | `Core/Src/gpio.c` 等 | 外设 GPIO/DMA/I2C 初始化 | `MX_GPIO_Init()` 等 |
| **stm32f4xx_it** | `Core/Src/stm32f4xx_it.c` | 中断向量入口 | `USART2_IRQHandler()`, `USART3_IRQHandler()` |
| **config** | `Core/Inc/config.h` | 所有可调参数（几何、PID、限幅、补偿因子） | `#define` 宏 |

---

## 2. 领域词汇表 (Ubiquitous Language)

### 运动类型

| 术语 | 含义 | 枚举/宏 |
|------|------|---------|
| **MotionKind** | 运动动作类型枚举 | `MOTION_KIND_FORWARD=0`, `CIRCLE=1`, `QUARTER_TURN_LEFT=2`, `QUARTER_TURN_RIGHT=3` |
| **FORWARD / BACKWARD** | 前进/后退 n 格（单格 `GRID_SIZE_M = 0.15m`） | 麦克纳姆平动 |
| **CIRCLE** | 原地转圈 n 圈 | 麦克纳姆自旋 |
| **LEFT / RIGHT** | 90° quarter-turn 转向 + 直行 | `QUARTER_TURN_LEFT` / `QUARTER_TURN_RIGHT` → `Mecanum_StepQuarterTurn()` + `Mecanum_StepForward()` |
| **RUN** | 设置基础速度百分比，速度开环模式 | `base_speed_percent` |
| **STOP** | 立即清除所有运动和队列 | `Mecanum_StepStop()` |

### 运动编排

| 术语 | 含义 |
|------|------|
| **motion_queue** | 运动指令环形缓冲区（深度 16），由 UART 接收端入队，`Motion_Tick()` 逐个出队执行 |
| **motion_active** | 当前是否有运动正在执行（位置环未到达目标时为 1） |
| **Motion_Tick()** | 每 10ms 执行的状态机：IDLE → 出队 → EXECUTING → COMPLETE/TIMEOUT → 下一条 |
| **IsMotionBusy()** | 检查 `motion_active` 标记 |
| **STOPPED** | 所有指令执行完毕或 STOP 后通过对应 UART 通道回复的确认消息 |

### 电机控制

| 术语 | 含义 |
|------|------|
| **外环（位置环）** | 输入位置目标 (counts) vs 编码器累计位置，输出速度目标 (counts/s)，PID 控制 |
| **内环（速度环）** | 输入速度目标 vs 编码器速度反馈，输出 PWM 占空比 (-999 ~ +999)，PID 控制 |
| **反馈低通滤波** | EMA 滤波，α = `MOTOR_FEEDBACK_LPF_ALPHA = 0.2`，平滑编码器速度噪声 |
| **死区补偿** | 额外 PWM 占空比叠加，克服静摩擦力，每轮独立配置 |
| **输出限幅** | `COMMAND_MAX_OUTPUT_PERCENT` 限制所有高层命令的最大 PWM 输出 |
| **位置环输出限幅** | 每轮独立的位置环 PID 输出上限，防止瞬间剧烈修正 |
| **速度死区** | `SPEED_THRESHOLD_COUNTS_PER_SEC`，低于此阈值的速度目标被强制归零 |
| **位置容差** | `POSITION_TOLERANCE_COUNTS`，位置到达判断阈值 |
| **位置超时** | `POSITION_TIMEOUT_MS` / `CIRCLE_TIMEOUT_MS`，超时后强制结束当前运动 |

### 补偿因子

| 术语 | 宏 | 作用 |
|------|-----|------|
| **前进修正因子** | `FORWARD_CORRECTION_FACTOR` | 补偿前进/后退时机械打滑导致的距离偏差 |
| **旋转修正因子** | `TURN_CORRECTION_FACTOR` | 补偿 CIRCLE 原地旋转的编码器偏差 |
| **LEFT quarter-turn 修正因子** | `QUARTER_TURN_LEFT_CORRECTION_FACTOR` | LEFT 命令 90° 转向独立补偿 |
| **RIGHT quarter-turn 修正因子** | `QUARTER_TURN_RIGHT_CORRECTION_FACTOR` | RIGHT 命令 90° 转向独立补偿 |
| **quarter-turn 步长** | `QUARTER_TURN_STEPS` | 单次 quarter-turn 圈数步长（默认 0.25 = 90°） |

### 通信

| 术语 | 含义 |
|------|------|
| **UART2 数组模式** | `{CMD1,CMD2,...}` 花括号包裹，逗号分隔，串行执行 |
| **UART2 单行模式** | `RUN 50` / `STOP`，向后兼容 |
| **UART3 单行模式** | `CIRCLE` / `CIRCLE n` / `STOP`，独立通道 |
| **空闲中断接收** | `HAL_UARTEx_ReceiveToIdle_IT()`，收到完整帧后自动重启接收 |
| **uart2_cmd_queue** | UART2 命令环形缓冲（深度 16），ISR 入队 → StartTask02 出队处理 |
| **uart3_rx_line** | UART3 单行缓冲（32 字节），ISR 中直接处理（不经过任务队列） |

---

## 3. 调用关系图

```
main()
├── HAL_Init()
├── SystemClock_Config()
├── MX_GPIO_Init()
├── MX_DMA_Init()
├── MX_I2C1_Init()
├── MX_TIM1_Init()      ← PWM (4通道)
├── MX_TIM2_Init()      ← 编码器 (Motor LR)
├── MX_TIM3_Init()      ← 编码器 (Motor RF)
├── MX_TIM4_Init()      ← 编码器 (Motor LF)
├── MX_TIM5_Init()      ← 编码器 (Motor RR)
├── MX_USART2_UART_Init()
├── MX_USART3_UART_Init()
├── Motor_Init()        ← 初始化 4 轮 PID + 硬件配置
└── MX_FREERTOS_Init()
    └── osKernelStart()
        ├── defaultTask      (prio Normal, 1ms, 空闲)
        ├── StartTask02      (prio Low, 1ms)
        │   └── 出队 uart2_cmd_queue → UART2_HandleCommand()
        │       ├── FORWARD/BACKWARD → Mecanum_StepForward()
        │       ├── LEFT → Mecanum_StepQuarterTurn(LEFT) + Mecanum_StepForward()
        │       ├── RIGHT → Mecanum_StepQuarterTurn(RIGHT) + Mecanum_StepForward()
        │       ├── CIRCLE → Mecanum_StepCircle()
        │       ├── RUN → 设置 base_speed_percent
        │       └── STOP → Mecanum_StepStop()
        └── StartTask03      (prio Low, 10ms)
            ├── Motion_Tick()           ← 运动编排状态机
            └── Motor_UpdateControl(dt) ← 四轮 PID 更新

UART2 ISR → HAL_UARTEx_RxEventCallback
├── UART2_ProcessRxByte()  逐字节解析
└── 重新 HAL_UARTEx_ReceiveToIdle_IT()

UART3 ISR → HAL_UARTEx_RxEventCallback
├── UART3_ProcessByte()  逐字节解析
│   └── \r\n → UART3_HandleCommand()
│       ├── CIRCLE → Mecanum_StepCircle()
│       └── STOP → Mecanum_StepStop()
└── 重新 HAL_UARTEx_ReceiveToIdle_IT()
```

---

## 4. 数据流向

### 4.1 从串口到电机输出的完整路径

```
┌──────────────────────────────────────────────────────────────────┐
│  上位机                                                            │
│  UART2: {FORWARD 3, LEFT 1, CIRCLE 2}                            │
│  UART3: CIRCLE 1 / STOP                                           │
└────────────┬─────────────────────────────────┬────────────────────┘
             │ UART2                            │ UART3
             ▼                                  ▼
     HAL_UARTEx_RxEventCallback        HAL_UARTEx_RxEventCallback
     ┌───────┴───────┐                 ┌───────┴───────┐
     │ 逐字节解析      │                 │ 逐字节解析      │
     │ 数组/单行模式   │                 │ 遇\r\n直接处理 │
     └───────┬───────┘                 └───────┬───────┘
             │ 入队                             │ 直接调用
             ▼                                  ▼
     uart2_cmd_queue                    UART3_HandleCommand()
             │                                  │
             ▼                                  ▼
     StartTask02 出队                    Mecanum_StepCircle()
     UART2_HandleCommand()              Mecanum_StepStop()
             │
   ┌─────────┼─────────┐
   │         │         │
   ▼         ▼         ▼
StepForward StepQuarterTurn StepCircle
             │
             ▼
     Mecanum_StepForward()
             │
             ▼
     motion_queue (环形缓冲, 深度16)
             │
             ▼
     Motion_Tick() 每10ms
             │
     ┌───────┴───────┐
     │ 出队 → 设置每轮位置目标
     │ 四轮 position_target (counts)
     └───────┬───────┘
             ▼
     Motor_PositionUpdate(dt)
     ┌───────┴───────┐
     │ 位置环 PID (外环)
     │ 输出: speed_target (counts/s)
     └───────┬───────┘
             ▼
     速度环 PID (内环)
     ┌───────┴───────┐
     │ 输入: speed_target vs encoder_feedback
     │ 反馈先经 EMA 低通滤波 (α=0.2)
     │ 输出: pwm_duty (-999 ~ +999)
     └───────┬───────┘
             ▼
     输出限幅 (COMMAND_MAX_OUTPUT_PERCENT)
     死区补偿叠加
     速度死区检查
             │
             ▼
     PWM → TB6612 驱动 → 直流电机
     Encoder → TIM2~5 编码器模式 → 累计计数
```

### 4.2 运动队列串行执行流程

```
motion_queue: [FORWARD3] [LEFT1] [CIRCLE2] ...
                    │
                    ▼
            Motion_Tick() 出队
                    │
            ┌───────▼────────┐
            │ Motion_StartRequest() │
            │ 设置四轮 position_target │
            │ motion_active = 1        │
            └───────┬────────┘
                    │
            ┌───────▼────────┐
            │ 每10ms 轮询      │
            │ all_reached?     │
            ├────┬────────────┤
            │ YES│ NO + 超时   │
            ▼    ▼
        COMPLETE TIMEOUT
            │    │
            └────┼────→ motion_active=0
                 │      检查队列
                 ├─ 还有 → 出队继续
                 └─ 空 → 发送 STOPPED
```

---

## 5. 关键设计决策 (ADR)

### ADR-001: 双 UART 通道分工
- **UART2**: 主运动序列通道，支持数组模式（多指令串行）和单行模式（RUN/STOP）
- **UART3**: 旋转/停止独立通道，ISR 中直接处理，不入任务队列
- **原因**: UART3 的 STOP/CIRCLE 需要快速响应，避免经过队列延迟

### ADR-002: LEFT/RIGHT = 旋转 + 直行（不回正）
- 先做 quarter-turn 90° 转向，再直行，不转回
- 使用独立修正因子 `QUARTER_TURN_LEFT/RIGHT_CORRECTION_FACTOR` 调校

### ADR-003: 位置环 + 速度环 串级 PID
- 外环（位置环）: 统一 PID 参数，每轮独立输出限幅
- 内环（速度环）: 每轮独立 PID 参数 + 死区补偿
- 反馈链路中加入 EMA 低通滤波（α=0.2）

### ADR-004: 空闲中断接收 + 自动重启
- 使用 `HAL_UARTEx_ReceiveToIdle_IT()` 接收不定长帧
- 每次接收完成后在回调中立即重新调用，实现持续接收
- 错误回调中也重新启动，保证管道不中断

### ADR-005: 已移除的硬件
- MPU6050 (I2C1) 已禁用
- 航向 PID 已删除
- `Mecanum_StepStrafe()` 已删除
- `Mecanum_SetMotion()` 已删除
- VOFA 遥测已删除