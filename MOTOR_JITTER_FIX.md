# 电机抖动问题分析与修复 / Motor Jitter Issue Analysis and Fix

## 🔴 问题描述 / Problem Description

电机不是正常转动，而是出现抖动现象。

## 🔍 根本原因分析 / Root Cause Analysis

### 问题 1: **目标速度被反复覆盖和累积衰减** ⚠️ 最严重

**原始代码**:
```c
const MotorPid_t *rr_pid = Motor_GetPID(MOTOR_RIGHT_REAR);
Motor_SetTargetPercent(MOTOR_RIGHT_REAR, (rr_pid->target / 30.0f) + pitch_correction);
```

**问题**:
1. 每次循环（10ms）都会读取当前的 `target` 值
2. 然后除以 30.0 再加上校正值，重新设置为新的 `target`
3. 下一次循环又读取这个新的 `target`，再除以 30.0...

**结果**:
```
第 1 次: target = 50 / 30.0 + correction = 1.67 + correction
第 2 次: target = 1.67 / 30.0 + correction = 0.056 + correction
第 3 次: target = 0.056 / 30.0 + correction = 0.002 + correction
...
```

目标速度快速衰减到接近 0，导致电机无法正常转动！

### 问题 2: **除以 30.0f 的逻辑错误**

`Motor_SetTargetPercent()` 函数期望的参数是百分比（-100 到 100），但代码中 `target / 30.0f` 会将目标速度除以 30，这完全没有意义。

### 问题 3: **校正系数过大**

`PITCH_CORRECTION_KP = 2.0f` 太大了。假设 pitch 角度为 5 度（0.087 弧度），校正值就是：
```
correction = -0.087 * 2.0 = -0.174 (约 -17.4%)
```

这个校正值相对于基础速度来说太大，会导致过度校正和震荡。

### 问题 4: **没有保存原始目标速度**

每次都从 PID 结构体中读取 `target`，而这个值已经被修改过了，无法获取用户最初设定的速度。

## ✅ 修复方案 / Solution

### 1. 添加基础速度变量

```c
static float base_speed_percent = 0.0f;  // 保存用户设定的基础速度
```

### 2. 在命令处理中保存基础速度

```c
// STOP 命令
base_speed_percent = 0.0f;

// RUN 命令
base_speed_percent = (float)percent;  // 保存用户设定的速度
Motor_SetTargetPercent(MOTOR_RIGHT_REAR, base_speed_percent);
// ...
```

### 3. 修正控制循环逻辑

**修复后的代码**:
```c
if (mpu_initialized && (base_speed_percent != 0.0f))
{
  if (MPU6050_DMP_GetData(&mpu_quat, &mpu_euler) == 0)
  {
    float pitch_correction = -mpu_euler.pitch * PITCH_CORRECTION_KP;

    // 直接使用基础速度 + 校正值
    Motor_SetTargetPercent(MOTOR_RIGHT_REAR, base_speed_percent + pitch_correction);
    Motor_SetTargetPercent(MOTOR_LEFT_REAR, base_speed_percent - pitch_correction);
    Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, base_speed_percent + pitch_correction);
    Motor_SetTargetPercent(MOTOR_LEFT_FRONT, base_speed_percent - pitch_correction);
  }
}
```

### 4. 降低校正系数

```c
#define PITCH_CORRECTION_KP 0.5f  // 从 2.0 降低到 0.5
```

## 📊 修复前后对比 / Before and After Comparison

### 修复前（错误逻辑）

| 循环次数 | 目标速度计算 | 实际目标值 |
|---------|------------|-----------|
| 1 | 50 / 30.0 + 0.1 | 1.77 |
| 2 | 1.77 / 30.0 + 0.1 | 0.159 |
| 3 | 0.159 / 30.0 + 0.1 | 0.105 |
| 4 | 0.105 / 30.0 + 0.1 | 0.104 |

**结果**: 速度快速衰减，电机抖动

### 修复后（正确逻辑）

| 循环次数 | 目标速度计算 | 实际目标值 |
|---------|------------|-----------|
| 1 | 50 + 0.1 | 50.1 |
| 2 | 50 + 0.15 | 50.15 |
| 3 | 50 + 0.12 | 50.12 |
| 4 | 50 + 0.08 | 50.08 |

**结果**: 速度稳定在基础值附近，只有小幅校正

## 🎯 校正系数调整建议 / Correction Coefficient Tuning

### 当前设置
```c
#define PITCH_CORRECTION_KP 0.5f
```

### 调整指南

| 现象 | 建议调整 |
|------|---------|
| 车辆仍然抖动 | 降低 KP 到 0.3 或 0.2 |
| 车辆偏离严重，校正不足 | 增加 KP 到 0.8 或 1.0 |
| 车辆左右摇摆 | 降低 KP，可能需要添加微分项 |
| 校正响应太慢 | 适当增加 KP |

### 测试步骤

1. **从小值开始**: 先用 `KP = 0.2` 测试
2. **观察行为**: 
   - 车辆是否能保持直线
   - 是否有震荡
   - 校正速度是否合适
3. **逐步调整**: 每次增加 0.1，直到找到最佳值
4. **记录最佳值**: 在不同速度下测试，找到通用的最佳值

## 🔧 其他可能的问题 / Other Potential Issues

### 1. MPU6050 数据噪声

**症状**: 即使修复后仍有轻微抖动

**解决方案**: 添加低通滤波
```c
static float filtered_pitch = 0.0f;
#define FILTER_ALPHA 0.8f

// 在读取 pitch 后
filtered_pitch = FILTER_ALPHA * filtered_pitch + (1.0f - FILTER_ALPHA) * mpu_euler.pitch;
float pitch_correction = -filtered_pitch * PITCH_CORRECTION_KP;
```

### 2. PID 参数不合适

**症状**: 电机响应不稳定

**当前 PID 参数**:
```c
Motor_SetPIDGain(MOTOR_RIGHT_REAR, 10.0f, 0.5f, 0.1f);
```

**调整建议**:
- 如果震荡: 降低 Kp (比如改为 8.0)
- 如果响应慢: 增加 Kp (比如改为 12.0)
- 如果有稳态误差: 增加 Ki (比如改为 1.0)

### 3. 控制周期太快

**当前**: 10ms (100Hz)

**如果 MPU6050 数据更新率低于 100Hz**, 会导致重复使用相同的数据，可能引起问题。

**解决方案**: 检查 MPU6050 的实际更新率，或增加控制周期到 20ms。

## 📝 测试步骤 / Testing Steps

1. **重新编译并烧录固件**
   ```bash
   cd build/Debug
   ninja
   ```

2. **测试基本运行**
   ```
   串口发送: RUN 30
   预期: 电机平稳转动，无抖动
   ```

3. **测试停止**
   ```
   串口发送: STOP
   预期: 电机立即停止
   ```

4. **测试不同速度**
   ```
   RUN 20  -> 低速测试
   RUN 50  -> 中速测试
   RUN 80  -> 高速测试
   ```

5. **观察航向校正**
   - 让车辆直线行驶
   - 轻微倾斜车身
   - 观察是否自动校正方向

## 🎓 经验教训 / Lessons Learned

1. **永远不要修改正在使用的变量**: 不要从 PID 结构体读取 target 然后又修改它
2. **保存原始输入**: 用户的输入应该保存在独立的变量中
3. **校正值应该是增量**: 校正应该是在基础值上的小幅调整，不是替换
4. **从小参数开始**: 控制参数（如 KP）应该从小值开始逐步调整
5. **单位要一致**: 确保所有速度值使用相同的单位（百分比）

## 📌 总结 / Summary

**主要问题**: 目标速度被反复除以 30 并覆盖，导致快速衰减到接近 0

**修复方法**:
1. ✅ 添加 `base_speed_percent` 变量保存原始速度
2. ✅ 移除错误的 `/ 30.0f` 操作
3. ✅ 直接使用 `base_speed + correction` 设置目标
4. ✅ 降低校正系数从 2.0 到 0.5

**预期结果**: 电机平稳转动，pitch 角度校正正常工作
