Status: ready-for-agent

## Parent

PRD: [重写串口收发逻辑：UART2数组指令 + UART3 CIRCLE 独立控制](../../PRD.md)

## What to build

将运动完成的 "STOPPED" 信号从 UART2 迁移至 UART3，并实现数组中 STOP 的截断语义。

- `Motion_Tick` 检测到队列清空且所有运动到达目标后，通过 UART3 发送 `"STOPPED\r\n"`
- UART2 数组全部执行完毕 → UART3 发一次 STOPPED
- UART3 CIRCLE 执行完毕 → UART3 发一次 STOPPED
- 数组中遇到 STOP 指令时：清空 `motion_queue` 中所有剩余项、立即停止所有电机输出、不发 STOPPED（因为是异常截断而非正常完成）
- 任何通道发送 STOPPED 后，清除 `motion_busy` 互斥标志，恢复空闲状态

完工后，上位机统一通过 UART3 获取运动完成通知。

## Acceptance criteria

- [ ] UART2 数组 `{FORWARD 1,LEFT 1}` 全部执行完毕后，UART3 收到 `STOPPED`
- [ ] UART3 CIRCLE 执行完毕后，UART3 收到 `STOPPED`
- [ ] 数组 `{FORWARD 3,STOP,LEFT 2}`：FORWARD 3 之后 LEFT 2 被丢弃，电机停止，UART3 不收到 STOPPED
- [ ] 收到 STOPPED 后系统允许接收下一个数组或 CIRCLE（互斥标志已清除）
- [ ] 项目编译通过无报错

## Blocked by

- #03-uart3-independent-channel