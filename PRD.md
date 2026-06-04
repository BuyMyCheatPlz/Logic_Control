## Problem Statement

当前串口2以逐行文本方式接收单条运动指令（FORWARD/LEFT/RIGHT/BACKWARD/CIRCLE/RUN/STOP），串口3仅用于VOFA遥测输出。每次只能发送一条指令，无法将多个指令打包为序列一次性下发，也无法区分UART2和UART3的职责。

## Solution

- **UART2** 改为接收C风格数组格式 `{FORWARD 3,LEFT 2,BACKWARD 1,RIGHT 1}`，一次性下发指令序列，逐条顺序执行（位置环稳态后自动进入下一条）
- **UART3** 改为独立接收 CIRCLE 和 STOP 指令，与 UART2 数组完全互斥
- 移除 VOFA JustFloat 遥测

## User Stories

1. As a robot operator, I want to send a command sequence like `{FORWARD 3,LEFT 2,BACKWARD 1}` over UART2, so that the robot executes a multi-step path without waiting for step-by-step confirmation.
2. As a robot operator, I want the robot to wait for each position move to reach its target before starting the next, so that moves don't overlap and accumulate errors.
3. As a robot operator, I want to include STOP in the array to abort the remaining sequence immediately, so that I can handle unexpected situations mid-sequence.
4. As a robot operator, I want CIRCLE commands sent over UART3 to execute independently, so that rotational calibration can be triggered on a separate channel.
5. As a robot operator, I want UART2 arrays and UART3 CIRCLE to be mutually exclusive, so that concurrent commands don't conflict.
6. As a robot operator, I want to receive STOPPED over UART3 when any motion sequence completes, so that the host system knows the robot is idle and ready for the next command.
7. As a robot operator, I still want to use RUN xx as a standalone UART2 command for manual open-loop speed control.
8. As a developer, I want VOFA telemetry removed entirely, so that UART3 is fully dedicated to command reception.

## Implementation Decisions

1. **UART2 array format**: `{CMD1,CMD2,...}` with curly braces and comma separators. Each element follows existing command syntax (e.g. `FORWARD 3`, `LEFT 2`, `BACKWARD 1`, `RIGHT 1`, `STOP`). Max 10 elements.
2. **UART2 standalone commands**: RUN and STOP remain as single-line commands (no braces) for backward compatibility.
3. **UART3 commands**: CIRCLE and STOP only. Same format as current single-line UART2 commands.
4. **Mutual exclusion**: A global busy flag prevents UART3 CIRCLE from executing while a UART2 array is in progress, and vice versa. The later command is silently ignored.
5. **STOP semantics**: When STOP appears in an array, all remaining array elements after STOP are discarded, motors stop immediately, and no STOPPED is sent (because the sequence was aborted, not completed).
6. **STOPPED signal**: Sent via UART3 (not UART2). Once when an array completes all elements, once when a CIRCLE completes.
7. **VOFA removal**: Function `VOFA_SendJustFloat()` and its call in StartTask03 are deleted. The `VOFA_JUSTFLOAT_PERIOD_MS` and `VOFA_JUSTFLOAT_FLOATS` macros in config.h are removed.
8. **Idle state**: After any motion completes, the system waits for the next UART2 array or UART3 CIRCLE command.

## Testing Decisions

- Test seams at HAL UART callback level: verify correct array parsing from raw bytes.
- Test motion queue behavior: enqueue multiple commands, verify sequential execution with position-reached gating.
- Test mutual exclusion: send CIRCLE during array execution, verify it is ignored.
- Test STOP-in-array: verify remaining elements are discarded and motors stop.

## Out of Scope

- Changing the motor control PID logic or position target computation
- Adding new motion types beyond the existing FORWARD/BACKWARD/LEFT/RIGHT/CIRCLE
- Changing the physical parameters in config.h (except VOFA macros)
- Mecanum_SetMotion — remains unchanged but unused in new flow
- Heading PID — already removed in prior builds, remains removed

## Further Notes

The refactored architecture cleanly separates UART2 as the primary motion-sequence channel and UART3 as the rotation/stop channel. The existing `motion_queue` mechanism and `Motion_Tick()` state machine remain the execution backbone — only the command ingestion layer changes.