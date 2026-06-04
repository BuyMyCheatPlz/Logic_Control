Status: ready-for-agent

## Parent

PRD: [重写串口收发逻辑：UART2数组指令 + UART3 CIRCLE 独立控制](../../PRD.md)

## What to build

改造 UART2 接收逻辑，支持 `{FORWARD 3,LEFT 2,BACKWARD 1,RIGHT 1}` 数组格式的解析与批量入队。

- 在现有 `\n` 逐行解析之上，新增 `{}` 包裹识别状态机：遇到 `{` 进入数组收集模式，`,` 分隔元素，`}` 结束数组
- 数组内每个元素格式与现有单条命令一致（如 `FORWARD 3`、`LEFT 2`），最大支持 10 个元素
- 收到 `}` 后，将解析出的所有指令一次性按顺序批量加入 `motion_queue`
- 独立的 `RUN xx` 行模式保持不变（不带 `{}` 的单行指令）

完工后，上位机可通过 UART2 一次发送 `{FORWARD 3,LEFT 2,BACKWARD 1}` 完成多步路径。

## Acceptance criteria

- [ ] 收到 `{FORWARD 3,LEFT 2}` 后，`motion_queue` 中包含 2 条指令，顺序为 FORWARD 3 → LEFT 2
- [ ] 独立 `RUN 50` 行指令照常工作
- [ ] 超过 10 个元素的数组被拒绝（或截断至 10），不触发溢出
- [ ] 数组执行完后（队列空），系统处于等待状态，可接收下一个数组
- [ ] 项目编译通过无报错

## Blocked by

- #01-remove-vofa-telemetry