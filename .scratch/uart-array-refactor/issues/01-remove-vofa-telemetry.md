Status: ready-for-agent

## Parent

PRD: [重写串口收发逻辑：UART2数组指令 + UART3 CIRCLE 独立控制](../../PRD.md)

## What to build

完全移除 VOFA JustFloat 遥测系统，释放 UART3 用于后续指令通道。

- 删除 `VOFA_SendJustFloat()` 函数定义及所有实现代码
- 删除 `StartTask03` 中对 VOFA 发送的周期调用
- 删除 `config.h` 中 `VOFA_JUSTFLOAT_PERIOD_MS` 和 `VOFA_JUSTFLOAT_FLOATS` 宏定义

完工后，UART3 不再有周期性数据发送，完全空闲可用于后续指令接收。

## Acceptance criteria

- [ ] `VOFA_SendJustFloat()` 函数在代码库中不再存在
- [ ] 任何任务中不再调用该函数
- [ ] `config.h` 中 `VOFA_JUSTFLOAT_PERIOD_MS` 和 `VOFA_JUSTFLOAT_FLOATS` 宏已被移除
- [ ] 项目编译通过无报错

## Blocked by

None - can start immediately