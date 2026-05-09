# 麦克纳姆轮控制系统 / Mecanum Wheel Control System

## 🎮 串口命令列表 / Serial Commands

### 基本运动控制

| 命令 | 功能 | 速度 | 响应 |
|------|------|------|------|
| `FORWARD` | 前进 | 30% | `FORWARD OK` |
| `BACK` | 后退 | 30% | `BACK OK` |
| `LEFT` | 左平移 | 30% | `LEFT OK` |
| `RIGHT` | 右平移 | 30% | `RIGHT OK` |
| `STOP` | 停止 | 0% | `STOP OK` |

### 航向控制

| 命令 | 功能 | 默认状态 | 响应 |
|------|------|---------|------|
| `MPU ON` | 启用航向校正 | 关闭 | `MPU ON` |
| `MPU OFF` | 禁用航向校正 | **默认** | `MPU OFF` |

### PID 调参

| 命令 | 功能 | 示例 | 响应 |
|------|------|------|------|
| `PID <kp> <ki> <kd>` | 设置航向 PID | `PID 0.5 0.05 0.1` | `PID OK 0.50 0.05 0.10` |

### 自定义速度（原有功能）

| 命令 | 功能 | 示例 | 响应 |
|------|------|------|------|
| `RUN <percent>` | 设置速度 | `RUN 50` | `RUN OK 50` |

## 🚗 麦克纳姆轮运动学 / Mecanum Wheel Kinematics

### 轮子布局
```
    前 Front
  LF ↗  ↖ RF
      ╱╲
      ╲╱
  LR ↖  ↗ RR
    后 Rear
```

### 运动公式

```c
右后轮 (RR) = forward - strafe - rotation
左后轮 (LR) = forward + strafe + rotation
右前轮 (RF) = forward + strafe - rotation
左前轮 (LF) = forward - strafe + rotation
```

### 运动模式

#### 1. 前进 (FORWARD)
```
forward = 30, strafe = 0, rotation = 0

RR = 30    LR = 30
RF = 30    LF = 30

所有轮子同速前进 →
```

#### 2. 后退 (BACK)
```
forward = -30, strafe = 0, rotation = 0

RR = -30   LR = -30
RF = -30   LF = -30

所有轮子同速后退 ←
```

#### 3. 左平移 (LEFT)
```
forward = 0, strafe = -30, rotation = 0

RR = 30    LR = -30
RF = -30   LF = 30

对角轮同向，实现左平移 ⇐
```

#### 4. 右平移 (RIGHT)
```
forward = 0, strafe = 30, rotation = 0

RR = -30   LR = 30
RF = 30    LF = -30

对角轮同向，实现右平移 ⇒
```

## 🧪 测试步骤 / Testing Steps

### 步骤 1: 基本运动测试

```bash
# 前进
FORWARD
# 等待 2 秒
STOP

# 后退
BACK
# 等待 2 秒
STOP

# 左平移
LEFT
# 等待 2 秒
STOP

# 右平移
RIGHT
# 等待 2 秒
STOP
```

### 步骤 2: 测试航向校正

```bash
# 启用航向校正
MPU ON

# 前进测试
FORWARD
# 观察是否保持直线
STOP

# 如果有震荡，调整 PID
PID 0.3 0.05 0.05

# 再次测试
FORWARD
STOP
```

### 步骤 3: 测试不同方向

```bash
MPU ON

# 测试各个方向
FORWARD
STOP

BACK
STOP

LEFT
STOP

RIGHT
STOP
```

## 📊 运动控制函数 / Motion Control Function

### 函数原型

```c
static void Mecanum_SetMotion(float forward, float strafe, float rotation)
```

### 参数说明

| 参数 | 范围 | 说明 |
|------|------|------|
| `forward` | -100 ~ 100 | 前进速度（正值前进，负值后退） |
| `strafe` | -100 ~ 100 | 平移速度（正值右移，负值左移） |
| `rotation` | -100 ~ 100 | 旋转速度（正值顺时针，负值逆时针） |

### 功能特性

1. **自动归一化**: 如果计算出的轮速超过 100%，会自动按比例缩放
2. **PID 重置**: 每次运动切换时自动重置 PID 状态
3. **平滑过渡**: 通过电机 PID 实现平滑加速

### 使用示例

```c
// 前进 30%
Mecanum_SetMotion(30.0f, 0.0f, 0.0f);

// 左平移 30%
Mecanum_SetMotion(0.0f, -30.0f, 0.0f);

// 右平移 30%
Mecanum_SetMotion(0.0f, 30.0f, 0.0f);

// 后退 30%
Mecanum_SetMotion(-30.0f, 0.0f, 0.0f);

// 前进 + 右平移（斜向运动）
Mecanum_SetMotion(30.0f, 30.0f, 0.0f);

// 前进 + 顺时针旋转
Mecanum_SetMotion(30.0f, 0.0f, 20.0f);
```

## 🔧 添加自定义运动 / Adding Custom Motions

### 示例 1: 添加旋转命令

在 `UART2_HandleCommand` 函数中添加：

```c
if (strncmp(cursor, "ROTATE_CW", 9) == 0)
{
  // 顺时针旋转：forward=0, strafe=0, rotation=30
  Mecanum_SetMotion(0.0f, 0.0f, 30.0f);
  UART2_SendText("ROTATE_CW OK\r\n");
  return;
}

if (strncmp(cursor, "ROTATE_CCW", 10) == 0)
{
  // 逆时针旋转：forward=0, strafe=0, rotation=-30
  Mecanum_SetMotion(0.0f, 0.0f, -30.0f);
  UART2_SendText("ROTATE_CCW OK\r\n");
  return;
}
```

### 示例 2: 添加斜向运动

```c
if (strncmp(cursor, "DIAG_FR", 7) == 0)
{
  // 右前斜向：forward=30, strafe=30, rotation=0
  Mecanum_SetMotion(30.0f, 30.0f, 0.0f);
  UART2_SendText("DIAG_FR OK\r\n");
  return;
}

if (strncmp(cursor, "DIAG_FL", 7) == 0)
{
  // 左前斜向：forward=30, strafe=-30, rotation=0
  Mecanum_SetMotion(30.0f, -30.0f, 0.0f);
  UART2_SendText("DIAG_FL OK\r\n");
  return;
}
```

### 示例 3: 添加可变速度命令

```c
if (strncmp(cursor, "MOVE", 4) == 0)
{
  cursor += 4;
  float forward, strafe, rotation;
  if (sscanf(cursor, "%f %f %f", &forward, &strafe, &rotation) == 3)
  {
    Mecanum_SetMotion(forward, strafe, rotation);
    UART2_SendText("MOVE OK\r\n");
    return;
  }
  else
  {
    UART2_SendText("ERR FORMAT\r\n");
    return;
  }
}

// 使用示例：
// MOVE 30 0 0      -> 前进 30%
// MOVE 0 -30 0     -> 左平移 30%
// MOVE 30 30 0     -> 右前斜向 30%
// MOVE 30 0 20     -> 前进 + 旋转
```

## 🎯 航向控制说明 / Heading Control

### 默认状态
- **MPU 航向校正默认关闭**
- 需要手动发送 `MPU ON` 启用

### 工作原理

当 `MPU ON` 启用后：
1. 系统读取 MPU6050 的 pitch 角度
2. 航向 PID 计算需要的差速
3. 应用差速到左右轮，保持直线

### 适用场景

| 运动模式 | 是否需要航向校正 |
|---------|----------------|
| FORWARD | ✅ 推荐启用 |
| BACK | ✅ 推荐启用 |
| LEFT | ❌ 不需要 |
| RIGHT | ❌ 不需要 |
| 旋转 | ❌ 不需要 |

**建议**: 只在前进/后退时启用 MPU 校正，平移和旋转时禁用。

## 📈 性能参数 / Performance Parameters

| 参数 | 值 |
|------|-----|
| 控制周期 | 10 ms (100 Hz) |
| 默认速度 | 30% PWM |
| 最大差速 | ±20% |
| 航向 PID | Kp=0.5, Ki=0.05, Kd=0.1 |
| 电机 PID | Kp=10.0, Ki=0.5, Kd=0.1 |

## 🐛 故障排查 / Troubleshooting

### 问题 1: 平移时车辆不是直线移动

**可能原因**:
1. 轮子方向设置错误
2. 电机 PID 参数不一致
3. 机械安装问题

**解决方案**:
- 检查左侧轮子是否设置了反转
- 确保四个电机 PID 参数相同
- 检查轮子安装角度

### 问题 2: 前进时有偏移

**解决方案**:
```bash
# 启用航向校正
MPU ON
FORWARD
```

### 问题 3: 平移时有震荡

**解决方案**:
```bash
# 平移时不需要航向校正
MPU OFF
LEFT
```

### 问题 4: 速度不够

**解决方案**:
修改命令中的速度值，例如从 30% 改为 50%：

```c
// 在 freertos.c 中修改
Mecanum_SetMotion(50.0f, 0.0f, 0.0f);  // 前进 50%
```

## 📊 内存使用 / Memory Usage

```
代码段 (text):  48,980 字节
数据段 (data):     184 字节
BSS 段 (bss):   22,032 字节
总计:           71,196 字节
```

## 🚀 快速开始 / Quick Start

```bash
# 1. 烧录固件
st-flash write build/Debug/Logic_Control.bin 0x8000000

# 2. 连接串口（波特率 115200）

# 3. 测试基本运动
FORWARD
STOP

LEFT
STOP

RIGHT
STOP

BACK
STOP

# 4. 测试航向校正
MPU ON
FORWARD
STOP

# 5. 如果有震荡，调整 PID
PID 0.3 0.05 0.05
FORWARD
STOP
```

## 📝 命令速查表 / Command Quick Reference

```
基本运动:
  FORWARD  - 前进 30%
  BACK     - 后退 30%
  LEFT     - 左平移 30%
  RIGHT    - 右平移 30%
  STOP     - 停止

航向控制:
  MPU ON   - 启用航向校正
  MPU OFF  - 禁用航向校正（默认）

PID 调参:
  PID 0.5 0.05 0.1  - 设置航向 PID

自定义速度:
  RUN 50   - 设置速度 50%
```

---

**重要提示**: 
1. MPU 航向校正默认**关闭**，需要手动启用
2. 建议只在前进/后退时启用航向校正
3. 平移和旋转时建议禁用航向校正
