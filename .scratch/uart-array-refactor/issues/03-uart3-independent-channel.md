Status: ready-for-agent

## Parent

PRD: [重写串口收发逻辑：UART2数组指令 + UART3 CIRCLE 独立控制](../../PRD.md)

## What to build

启用 UART3 作为独立的指令通道，接收 `CIRCLE n` 和 `STOP` 命令。

- 实现 `UART3_ProcessByte` 逐行解析逻辑，与 UART2 现有风格一致
- 引入全局互斥标志 `motion_busy`：
  - UART2 数组开始执行时置位，执行期间 UART3 的 CIRCLE 被忽略
  - UART3 CIRCLE 开始执行时置位，执行期间 UART2 新数组被忽略
  - 任何通道执行完毕（队列空）后清除标志
- UART3 的 STOP 作为独立紧急停止，不受互斥限制

完工后，UART2 和 UART3 各自独立接收指令，且保证不会同时执行冲突的运动。

## Acceptance criteria

- [ ] UART3 收到 `CIRCLE 2\r\n` 后正确执行两次原地旋转
- [ ] UART2 数组运行期间，UART3 发送 CIRCLE 被忽略
- [ ] UART3 CIRCLE 运行期间，UART2 发送数组被忽略
- [ ] UART3 收到 `STOP\r\n` 后立即停止所有电机
- [ ] 项目编译通过无报错

## Blocked by

- #02-uart2-array-parsing