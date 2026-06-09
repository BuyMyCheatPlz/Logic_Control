# Logic_Control

基于 STM32F407 的四轮麦克纳姆底盘固件，FreeRTOS 多任务架构。

## 架构全景图

### 第一层：任务调度层

```
┌──────────────────────────────────────────────────┐
│               FreeRTOS 调度器                     │
│                                                  │
│  ┌────────────┐ ┌────────────┐ ┌───────────────┐│
│  │defaultTask │ │ Send_Data  │ │ Proccess_Data ││
│  │空闲, 1ms   │ │ 命令分发   │ │ 运动控制      ││
│  │stack 128×4 │ │ stack 256×4│ │ stack 512×4   ││
│  │prio Normal │ │ prio Low   │ │ prio Low      ││
│  └────────────┘ └────────────┘ └───────────────┘│
│                      │ 1ms 轮询     │ 10ms 周期   │
│                      ▼              ▼             │
│               uart2_cmd_queue  Motion_Tick()      │
│               环形缓冲区深度4   Motor_UpdateCtrl() │
└──────────────────────────────────────────────────┘
```

- **defaultTask** — 空闲占位，无实际业务
- **Send_Data** — 从 `uart2_cmd_queue` 出队指令，调用 `UART2_HandleCommand()` 分发
- **Proccess_Data** — 固定 10ms 周期：调用 `Motion_Tick()` 驱动运动队列 + `Motor_UpdateControl()` 驱动四轮 PID，启动时施加每轮独立的位置环输出限幅（`POSITION_OUTPUT_LIMIT_RR/RL/FR/FL`）

### 第二层：通信层（双 UART 通道）

| 通道 | 物理接口 | 职责 | 数据格式 |
|------|----------|------|----------|
| UART2 | USART2 | 主运动序列通道 | 数组：`{FORWARD 3,LEFT 2,BACKWARD 1}` 或单行：`RUN 50` / `STOP` |
| UART3 | USART3 | 旋转/停止独立通道 | 单行：`CIRCLE` / `CIRCLE n` / `STOP` |

**UART2 数据流：**

```
上位机 → UART2 → HAL_UARTEx_RxEventCallback (中断)
                    │
                    ▼
                UART2_ProcessRxByte()  逐字节流式解析
                ├── '{' → 进入数组模式
                ├── '}' → 结束数组模式
                ├── ',' → 分割数组元素 → 入队 uart2_cmd_queue
                └── '\r\n' → 单行结束 → UART2_FinalizeLine → 入队
                    │
                    ▼
                UART2_HandleCommand()
                ├── FORWARD/BACKWARD → Mecanum_StepForward()
                ├── LEFT/RIGHT       → Mecanum_StepStrafe()
                ├── CIRCLE           → Mecanum_StepCircle()
                ├── RUN              → 设置 base_speed_percent
                └── STOP             → 清除所有运动 + 回复 STOPPED
                    │
                    ▼
                motion_queue (环形缓冲区，深度 4)
```

**UART3 数据流：**

```
上位机 → UART3
  │
  │   HAL_UARTEx_RxEventCallback (中断)
  │       │
  │       ▼
  │   UART3_ProcessByte() 逐字节解析，'\r\n' 结束
  │       │
  │       ▼
  │   UART3_HandleCommand()
  │   ├── CIRCLE (带互斥检查) → Mecanum_StepCircle()
  │   └── STOP (无互斥) → 立即清除所有运动
  │       │ 完成后
  │       ▼
  │   UART3_SendText("STOPPED\r\n")
```

### 第三层：运动编排层

**核心状态机 `Motion_Tick()` — 每 10ms 执行一次：**

```
        ┌────────┐
        │  IDLE  │ (motion_active == 0, queue 有数据)
        └───┬────┘
            │ 出队 + Motion_StartRequest()
            ▼
        ┌──────────┐
        │ EXECUTING│ 每 10ms 检查四轮位置目标
        └───┬──────┘
            │
        ┌───┴───────────┐
        │ all_reached?   │
        ├───────────────┤
        │ YES           │ NO + 超时 → 强制结束
        ▼               ▼
    ┌─────────┐   ┌─────────┐
    │COMPLETE │   │TIMEOUT  │
    └────┬────┘   └────┬────┘
         │              │
         │ 检查队列      │
         ├─ 还有指令 → 自动出队执行下一个
         └─ 队列为空 → 发送 STOPPED
```

**互斥规则：**
- 同一时刻仅一个 motion 在执行
- UART3 `CIRCLE` 在 `IsMotionBusy()` 为真时被静默忽略
- `STOP` 命令（无论来自 UART2/UART3）不受互斥限制，立即清除

**运动类型：**
| 类型 | 功能 | 方向 |
|------|------|------|
| MOTION_KIND_FORWARD | 前进/后退 n 格 | +1 = 前, -1 = 后 |
| MOTION_KIND_STRAFE | 左/右横移 n 格 | +1 = 左, -1 = 右 |
| MOTION_KIND_CIRCLE | 原地转圈 n 次 | +1 = 顺时针 |

### 第四层：电机控制层

```
Motor_UpdateControl(dt_s)  ← 每 10ms 调用
│
├── Motor_PositionUpdate(dt_s)    位置环 PID（外环）
│   ├── 输入：位置目标 (counts) vs 编码器位置
│   ├── 输出：速度目标 (counts/s)
│   └── PID：POSITION_PID_KP/KI/KD
│
├── 速度环 PID（内环）
│   ├── 输入：速度目标 vs 编码器速度反馈
│   ├── 输出：PWM 占空比 (-999 ~ +999)
│   ├── 反馈低通滤波：α = 0.2
│   └── PID：每轮独立 VELOCITY_PID_* 参数
│
├── 输出限幅：COMMAND_MAX_OUTPUT_PERCENT（默认 30%）
│
└── 四轮映射：
    ┌─────────┬─────────┐
    │ MOTOR_LF│ MOTOR_RF│  ← 前 (Id 3) (Id 2)
    ├─────────┼─────────┤
    │ MOTOR_LR│ MOTOR_RR│  ← 后 (Id 1) (Id 0)
    └─────────┴─────────┘
```

### 第五层：物理配置层

所有可调参数集中在 `Core/Inc/config.h`：

**机器人几何：**
| 参数 | 值 | 含义 |
|------|-----|------|
| `GRID_SIZE_M` | 0.15 | 单格长度 (m) |
| `WHEEL_DIAM_M` | 0.06 | 轮子直径 (m) |
| `WHEEL_BASE_M` | 0.1226 | 前后轮中心距 (m) |
| `WHEEL_TRACK_M` | 0.175 | 左右轮中心距 (m) |

**编码器：** ENCODER_LINES=13 × QUADRATURE=4 × GEAR_RATIO=20 = **1040 counts/rev（轮端）**

**运动参数：**
| 参数 | 值 | 含义 |
|------|-----|------|
| `DEFAULT_STRAFE_PERCENT` | 30 | 横移默认速度 (%) |
| `DEFAULT_FORWARD_PERCENT` | 30 | 直行默认速度 (%) |
| `COMMAND_MAX_OUTPUT_PERCENT` | 30 | 全局输出限幅 (%) |
| `POSITION_TOLERANCE_COUNTS` | 20 | 位置到达容差 |
| `POSITION_TIMEOUT_MS` | 10000 (10s) | 位置运动超时 (ms) |
| `POSITION_OUTPUT_LIMIT_RR/RL/FR/FL` | 30 | 每轮独立位置环输出限幅 (% of max speed)，防止瞬间剧烈修正 |

### 文件依赖图

```
main.c
├── motor.h ──── motor.c ──── config.h
├── freertos.c (FreeRTOS 任务 + UART 协议 + 运动编排)
│     ├── motor.h / config.h
│     └── HAL UART 驱动
├── usart.c / tim.c / i2c.c / gpio.c / dma.c
└── stm32f4xx_it.c (中断服务)
```

### 已被移除的功能

| 功能 | 状态 | 原因 |
|------|------|------|
| VOFA JustFloat 遥测 | 已删除 | 不再需要实时速度曲线监控 |
| 航向 PID | 已删除 | 未使用 |
| MPU6050 传感器 | 已禁用 | 航向 PID 依赖项 |
| `Mecanum_SetMotion()` | 保留但未使用 | 被分段步进运动替代 |

---

## 串口协议

### UART2 — 数组模式

格式：`{CMD1,CMD2,CMD3,...}`，用花括号包裹，逗号分隔。

支持的数组元素：`FORWARD`、`FORWARD n`、`BACKWARD`、`BACKWARD n`、`BACK`、`BACK n`、`LEFT`、`LEFT n`、`RIGHT`、`RIGHT n`、`STOP`

语义：
- 指令按顺序串行执行，上一个位置环到达目标后才开始下一个
- 数组中遇到 `STOP` 会清除剩余指令并立即停止
- 所有指令执行完毕后，通过 UART2 发送 `STOPPED\r\n`

示例：
```
{FORWARD 3,LEFT 2,BACKWARD 1}   → 前进 3 格 → 左移 2 格 → 后退 1 格 → STOPPED
{FORWARD 4,STOP,RIGHT 2}        → 仅执行前进 4 格，然后停止（RIGHT 2 被丢弃）
```

### UART2 — 单行模式（向后兼容）

`RUN` 和 `STOP` 保持单行模式（无需花括号）：

- `RUN 50` — 设置基础速度 50%
- `STOP` — 立即停止，回复 `STOPPED\r\n`

### UART3

仅支持两条指令，均为单行模式（`\r\n` 结束）：

- `CIRCLE` 或 `CIRCLE n` — 原地顺时针转圈 n 次（默认 1 次）。当前有运动执行中时被忽略
- `STOP` — 立即清除所有运动（无互斥限制）

CIRCLE 执行完成后，通过 UART3 发送 `STOPPED\r\n`。

---

## 构建

```bash
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug
```

或使用项目脚本：

```bash
scripts/build_and_flash.sh Debug
```

---

## 引脚表

### 串口
| 接口 | TX | RX |
|------|----|----|
| USART2 | PA2 | PA3 |
| USART3 | PB10 | PB11 |

### 电机 PWM (TIM1)
| 电机 | 通道 | 引脚 |
|------|------|------|
| Motor 0 (RR) | CH1 | PE9 |
| Motor 1 (LR) | CH2 | PE11 |
| Motor 2 (RF) | CH3 | PE13 |
| Motor 3 (LF) | CH4 | PE14 |

### 方向控制 GPIO
| 电机 | IN1 | IN2 |
|------|-----|-----|
| Motor 0 (RR) | PE7 | PE8 |
| Motor 1 (LR) | PA9 | PA8 |
| Motor 2 (RF) | PE12 | PE10 |
| Motor 3 (LF) | PC6 | PC8 |

### 编码器
| 电机 | 定时器 | CH1 | CH2 |
|------|--------|-----|-----|
| Motor 0 (RR) | TIM5 | PA0 | PA1 |
| Motor 1 (LR) | TIM2 | PA5 | PB3 |
| Motor 2 (RF) | TIM3 | PA6 | PA7 |
| Motor 3 (LF) | TIM4 | PD12 | PD13 |

### MPU6050 (I2C1) — 已禁用
| SCL | SDA |
|-----|-----|
| PB6 | PB7 |

---

## 配置与校准

编辑 `Core/Inc/config.h` 中的宏以匹配硬件。

**距离换算关键参数：** `WHEEL_DIAM_M`、`WHEEL_BASE_M`、`WHEEL_TRACK_M`、`ENCODER_LINES`、`ENCODER_QUADRATURE`、`GEAR_RATIO`

这些参数直接影响 `FORWARD`、`LEFT`、`RIGHT` 和 `CIRCLE` 的距离换算。

**调参建议：** 先使用 `RUN 20` 验证速度闭环，再用 `FORWARD 1` / `LEFT 1` 验证编码器计数。如需校准原地旋转，调整 `WHEEL_BASE_M` / `WHEEL_TRACK_M` 后重建并测试 `CIRCLE 1`。