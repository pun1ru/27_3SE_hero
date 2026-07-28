# 待办事项

这里只保存已经确认、尚未完成或需要持续跟踪的项目事项。临时调试步骤不进入长期待办。

## 当前维护项

- [ ] 新增 `PrivateApplications` 或 `PrivateDrivers` 源文件后，确认对应 Keil 工程已收录，并运行该板输出目录的 `add_missing.py`。
- [ ] 修改上下板共用算法或驱动时，检查两侧实现是否同步，并记录必须保留的板级差异。
- [ ] 修改 Board2Borad 数据定义时，同步核对发送端、接收端、CAN ID、长度、缩放倍率和字节序。

## 已知维护问题

- Keil MDK 导出的 `compile_commands.json` 通常只包含 HAL、Middlewares、Core 和 Tasks，缺失 `PrivateApplications` 与 `PrivateDrivers` 源文件。目前通过各板输出目录的 `add_missing.py` 补全。

## 新增待办格式

```markdown
- [ ] [P1/P2/P3] 事项描述
  - 范围：涉及的板和模块
  - 完成条件：可验证的结果
  - 验证：尚未验证 / 已使用的命令或硬件步骤
```

不要记录密钥、个人信息、未经确认的故障原因或无法验证的结论。
