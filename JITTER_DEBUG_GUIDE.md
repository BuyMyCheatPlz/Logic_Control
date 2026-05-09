# 电机抖动深度调试指南 / Motor Jitter Deep Debugging Guide

## 🔴 当前状态 / Current Status

修复了目标速度覆盖问题后，电机仍然抖动。

## 🔍 可能的原因分析 / Possible Root Causes

### 1. MPU6050 校正干扰 PID 控制 ⚠️ 最可能

**问题**: 
- 每 10ms 重新设置目标速度会干扰 PID 控制器的积分项
- MPU6050 数据可能有噪声或更新不稳定
- 校正值变化太快

**测试方法**: 
```
1. 发送: MPU OFF    (禁用 MPU 校正)
2. 发送: RUN 30     (测试电机是否平稳)
3. 如果平稳，说明是 MPU 校正的问题
4. 如果仍抖动，说明是其他问题
```

### 2. PID 参数不合适导致震荡

**当前参数**:
```c
Kp = 10.0
Ki = 0.5
Kd = 0.1
```

**症状**:
- Kp 太大 → 快速震荡
- Ki 太大 → 慢速震荡，超调
- Kd 太小 → 无法抑制震荡

**测试方法**: 需要修改代码降低 Kp

### 3. 编码器噪声或机械问题

**可能原因**:
- 编码器信号不稳定
- 电机或轮子有机械卡顿
- 电源电压不稳定

**测试方法**: 
- 检查编码器连接
- 手动转动轮子，检查是否顺畅
- 测量电源电压

### 4. 控制周期问题

**当前**: 10ms (100Hz)

**可能问题**:
- 周期太快，PID 来不及稳定
- 编码器读取频率不匹配

**测试方法**: 需要修改代码增加周期到 20ms

## 🧪 系统化调试步骤 / Systematic Debugging Steps

### 步骤 1: 测试无 MPU 校正的基本运行

```bash
# 烧录新固件
st-flash write build/Debug/Logic_Control.bin 0x8000000

# 串口测试
MPU OFF      # 禁用 MPU 校正
RUN 30       # 低速测试
```

**预期结果**:
- ✅ 如果平稳 → MPU 校正是问题，跳到步骤 4
- ❌ 如果仍抖动 → 继续步骤 2

### 步骤 2: 测试不同速度

```bash
RUN 10       # 极低速
RUN 20       # 低速
RUN 40       # 中速
RUN 60       # 高速
```

**观察**:
- 所有速度都抖动 → PID 参数问题
- 只有高速抖动 → 可能是电源或机械问题
- 只有低速抖动 → 可能是死区或摩擦力问题

### 步骤 3: 检查 PID 参数（需要修改代码）

如果步骤 1-2 确认是 PID 问题，修改 `main.c`:

```c
// 降低 Kp，减少震荡
Motor_SetPIDGain(MOTOR_RIGHT_REAR, 5.0f, 0.3f, 0.05f);
Motor_SetPIDGain(MOTOR_LEFT_REAR, 5.0f, 0.3f, 0.05f);
Motor_SetPIDGain(MOTOR_RIGHT_FRONT, 5.0f, 0.3f, 0.05f);
Motor_SetPIDGain(MOTOR_LEFT_FRONT, 5.0f, 0.3f, 0.05f);
```

重新编译测试。

### 步骤 4: 优化 MPU 校正（如果步骤 1 确认是 MPU 问题）

#### 方案 A: 降低校正频率

修改 `freertos.c`，不要每次都更新：

```c
static uint32_t mpu_update_counter = 0;

// 在循环中
if (mpu_initialized && mpu_correction_enabled && (base_speed_percent != 0.0f))
{
  mpu_update_counter++;
  if (mpu_update_counter >= 5)  // 每 50ms 更新一次，而不是 10ms
  {
    mpu_update_counter = 0;
    if (MPU6050_DMP_GetData(&mpu_quat, &mpu_euler) == 0)
    {
      // ... 校正代码
    }
  }
}
```

#### 方案 B: 添加低通滤波

```c
static float filtered_pitch = 0.0f;
#define FILTER_ALPHA 0.9f

if (MPU6050_DMP_GetData(&mpu_quat, &mpu_euler) == 0)
{
  // 低通滤波
  filtered_pitch = FILTER_ALPHA * filtered_pitch + (1.0f - FILTER_ALPHA) * mpu_euler.pitch;
  float pitch_correction = -filtered_pitch * PITCH_CORRECTION_KP;
  // ...
}
```

#### 方案 C: 进一步降低校正系数

```c
#define PITCH_CORRECTION_KP 0.1f  // 从 0.5 降到 0.1
```

## 📊 诊断表 / Diagnostic Table

| 症状 | 可能原因 | 解决方案 |
|------|---------|---------|
| MPU OFF 后平稳 | MPU 校正干扰 | 降低校正频率或系数 |
| MPU OFF 仍抖动 | PID 参数问题 | 降低 Kp 到 5.0 |
| 所有速度都抖动 | PID 震荡 | 调整 PID 参数 |
| 只有高速抖动 | 电源不足或机械问题 | 检查硬件 |
| 只有低速抖动 | 死区或摩擦力 | 增加最小 PWM 或调整 PID |
| 周期性抖动 | 编码器问题 | 检查编码器连接 |

## 🔧 快速修复建议 / Quick Fix Recommendations

### 修复 1: 禁用 MPU 校正测试（已实现）

```bash
# 串口命令
MPU OFF      # 禁用 MPU 校正
RUN 30       # 测试
MPU ON       # 启用 MPU 校正
```

### 修复 2: 降低 PID 参数（需要修改代码）

在 `main.c` 中修改：

```c
// 原来: Kp=10.0, Ki=0.5, Kd=0.1
// 修改为:
Motor_SetPIDGain(MOTOR_RIGHT_REAR, 5.0f, 0.2f, 0.05f);
Motor_SetPIDGain(MOTOR_LEFT_REAR, 5.0f, 0.2f, 0.05f);
Motor_SetPIDGain(MOTOR_RIGHT_FRONT, 5.0f, 0.2f, 0.05f);
Motor_SetPIDGain(MOTOR_LEFT_FRONT, 5.0f, 0.2f, 0.05f);
```

### 修复 3: 增加控制周期（需要修改代码）

在 `freertos.c` 的 `StartTask03` 中：

```c
osDelay(20);  // 从 10ms 改为 20ms
```

## 📝 数据收集 / Data Collection

如果以上方法都不行，需要收集更多数据。添加调试输出：

```c
// 在控制循环中添加
char debug_msg[128];
snprintf(debug_msg, sizeof(debug_msg), 
         "T:%.1f F:%.1f E:%.1f O:%.1f\r\n",
         motor_state[MOTOR_RIGHT_REAR].pid.target,
         motor_state[MOTOR_RIGHT_REAR].pid.feedback,
         motor_state[MOTOR_RIGHT_REAR].pid.error,
         motor_state[MOTOR_RIGHT_REAR].pid.output);
UART2_SendText(debug_msg);
```

观察：
- target 是否稳定
- feedback 是否震荡
- error 是否持续变化
- output 是否震荡

## 🎯 最可能的解决方案 / Most Likely Solution

基于经验，**最可能的原因是 MPU 校正干扰了 PID 控制**。

**推荐测试顺序**:

1. ✅ **先测试 `MPU OFF` + `RUN 30`**
   - 如果平稳 → 问题确认，优化 MPU 校正
   - 如果仍抖动 → 继续下一步

2. ✅ **降低 PID 的 Kp 到 5.0**
   - 重新编译测试
   - 如果平稳 → 问题解决
   - 如果仍抖动 → 继续下一步

3. ✅ **增加控制周期到 20ms**
   - 重新编译测试
   - 如果平稳 → 问题解决
   - 如果仍抖动 → 检查硬件

## 🚀 新增串口命令 / New Serial Commands

| 命令 | 功能 | 响应 |
|------|------|------|
| `MPU ON` | 启用 MPU 校正 | `MPU ON\r\n` |
| `MPU OFF` | 禁用 MPU 校正 | `MPU OFF\r\n` |
| `RUN <percent>` | 设置速度 | `RUN OK <percent>\r\n` |
| `STOP` | 停止电机 | `STOP OK\r\n` |

## 📌 下一步行动 / Next Actions

1. **立即测试**: 烧录新固件，测试 `MPU OFF` + `RUN 30`
2. **报告结果**: 
   - 如果平稳 → MPU 是问题
   - 如果仍抖动 → 需要调整 PID
3. **根据结果选择修复方案**

---

**重要提示**: 请先测试 `MPU OFF` 模式，这是最快的诊断方法！
