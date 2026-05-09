# 编译问题解决方案 / Build Issue Resolution

## 问题描述 / Problem Description

VS Code 的 CMake Tools 扩展使用 Ninja 构建系统时出现编译错误：
```
ninja: build stopped: subcommand failed.
```

## 根本原因 / Root Cause

1. **缺少头文件包含**: `mpu6050.c` 中调用了 `inv_orientation_matrix_to_scalar()` 函数，但没有包含 `mpu_port.h`
2. **链接器选项重复**: `-specs=nano.specs` 被指定了两次，导致链接器错误
3. **库链接选项位置错误**: `-lc` 和 `-lm` 不应该在 `target_link_options` 中

## 解决方案 / Solution

### 1. 添加缺失的头文件
**文件**: `Core/Src/mpu6050.c`

```c
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu_port.h"  // 添加这一行
#include <math.h>
```

### 2. 修正 CMakeLists.txt 链接选项
**文件**: `CMakeLists.txt`

移除 `target_link_options` 中的 `-lc` 和 `-lm`：

```cmake
# Linker options
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -specs=nano.specs
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
)
```

数学库通过 `target_link_libraries` 链接：

```cmake
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m  # 数学库
)
```

## 编译结果 / Build Result

✅ **编译成功！**

### 生成的文件 / Generated Files

| 文件 | 大小 | 说明 |
|------|------|------|
| `Logic_Control.elf` | 658 KB | 可执行文件（包含调试信息） |
| `Logic_Control.bin` | 42 KB | 二进制固件文件 |
| `Logic_Control.hex` | 118 KB | Intel HEX 格式 |
| `Logic_Control.map` | 581 KB | 内存映射文件 |

### 内存使用 / Memory Usage

```
   text	   data	    bss	    dec	    hex	filename
  42580	    140	  22028	  64748	   fcec	Logic_Control.elf
```

## 使用 VS Code 编译 / Building with VS Code

### 方法 1: 使用 CMake Tools 扩展

1. 打开 VS Code
2. 按 `Ctrl+Shift+P` 打开命令面板
3. 选择 "CMake: Configure"
4. 选择 "CMake: Build" 或按 `F7`

### 方法 2: 使用终端

在项目根目录执行：

```bash
# 使用 Ninja（VS Code 默认）
cd build/Debug
cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug ../..
ninja

# 或使用 Make
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j4
```

## 两种构建系统的区别 / Difference Between Build Systems

| 特性 | Make | Ninja |
|------|------|-------|
| 速度 | 较慢 | 更快 |
| 并行构建 | 支持 (`-j4`) | 自动并行 |
| VS Code 默认 | 否 | 是 |
| 输出详细程度 | 详细 | 简洁 |

两种构建系统都能成功编译项目，生成相同的固件文件。

## 验证编译 / Verify Build

检查固件文件是否生成：

```bash
ls -lh build/Debug/Logic_Control.*
```

应该看到：
- `Logic_Control.elf` (658 KB)
- `Logic_Control.bin` (42 KB)
- `Logic_Control.hex` (118 KB)
- `Logic_Control.map` (581 KB)

## 下一步 / Next Steps

1. 将 `Logic_Control.bin` 或 `Logic_Control.hex` 烧录到 STM32F407 开发板
2. 通过串口测试 STOP 命令和 RUN 命令
3. 测试 MPU6050 航向校正功能

## 常见问题 / Common Issues

### Q: 编译时出现 "MPU6050 redefined" 警告
**A**: 这是正常的警告，不影响编译。`MPU6050` 宏在 CMake 和 main.h 中都有定义。

### Q: 如何切换构建系统？
**A**: 
- 使用 Make: `cmake -DCMAKE_BUILD_TYPE=Debug ..`
- 使用 Ninja: `cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug ..`

### Q: 如何清理构建？
**A**: 删除 build 目录：`rm -rf build && mkdir build`
