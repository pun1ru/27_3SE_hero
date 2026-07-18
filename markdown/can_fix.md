# CAN 接收 FIFO 读空修复说明

## 问题现象

CAN 接收回调原来是每次中断只读 1 帧：

- `HAL_CAN_RxFifo0MsgPendingCallback()`
- `HAL_CAN_RxFifo1MsgPendingCallback()`

在总线流量有突发时，这种写法容易出现下面的问题：

1. 一段时间内连续来了多帧 CAN 数据。
2. 硬件已经把这些数据推进 FIFO。
3. 软件进入回调后只取走 1 帧就退出。
4. 剩余帧继续留在 FIFO 里。
5. 如果后续又来一波突发，FIFO 积压会越来越多，最后出现 FIFO 满甚至丢帧。

所以问题的根因不是“单次 `HAL_CAN_GetRxMessage()` 不够快”，而是“每次回调没有把已经积压的 FIFO 数据读干净”。

## 为什么只读 1 帧不够

`MsgPendingCallback` 的语义只是：

`FIFO 里至少还有 1 帧未读数据`

它并不表示：

`FIFO 里只有 1 帧数据`

因此如果回调里只读 1 帧：

- FIFO 里的旧积压不会被清掉
- 下一次中断到来之前，硬件还可能继续往 FIFO 里塞新帧
- 在总线负载较高、或中断被短时间延迟时，FIFO 深度就会被快速吃满

## 修复思路

回调进入后，不是读 1 帧就退出，而是：

`把当前 FIFO 中已经积压的帧全部读完再退出`

实现方式是先查询 FIFO 当前填充数，再循环读取：

```c
while(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, aData);
    ...
}
```

`FIFO1` 同理。

## 本次代码修改


本次修复包含：

1. `HAL_CAN_RxFifo0MsgPendingCallback()` 改为循环读空 `CAN_RX_FIFO0`
2. `HAL_CAN_RxFifo1MsgPendingCallback()` 改为循环读空 `CAN_RX_FIFO1`
3. `HAL_CAN_GetRxMessage()` 改为使用回调参数 `hcan`，不再硬编码 `&hcan1` / `&hcan2`
4. 加了简短注释，明确说明这里的策略是“单次 ISR 读空 FIFO，避免突发积压”

原有 `switch-case` 解包逻辑保持不变，这次只调整 FIFO 服务方式。

## 预期效果

修复后，预期变化如下：

1. 单次中断可以把当前积压帧一次性清空
2. 突发流量下 FIFO 不再因为“每次只读 1 帧”而持续堆积
3. FIFO 满、overrun、丢帧风险会明显下降

## 仍然要注意的风险

这个修复很重要，但不是所有问题的终点。

即使改成“读空 FIFO”，下面这些情况仍然可能把 FIFO 打满：

1. 总线负载长期过高
2. 中断被屏蔽或被更高优先级任务长时间打断
3. 回调里做了太多重处理，导致 ISR 停留时间过长
4. 太多不同 ID 的报文被塞进同一个 FIFO

所以这个修复解决的是“回调策略错误”，不是所有接收性能问题。

## 建议验证方法

烧录后建议做下面几项确认：

1. 在高流量或突发流量下观察是否还会出现 FIFO 满
2. 如有观测手段，查看 FIFO fill level 是否能在回调后迅速回到 0
3. 确认各个 `switch-case` 分支仍能正常收包，没有因为循环读取而漏处理
4. 如果问题仍在，再继续检查中断优先级、总线负载和 ISR 内部耗时

## 结论

这个问题的正确默认写法应该是：

`进入接收回调 -> 读空当前 FIFO -> 再退出`

而不是：

`进入接收回调 -> 只读 1 帧 -> 退出`
