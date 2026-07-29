# 待办事项

这里只保存已经确认、尚未完成或需要持续跟踪的项目事项。临时调试步骤不进入长期待办。

## 当前维护项

- [ ] [P1] 完成下板 `DecisionAO` 的单一写入所有权迁移。
  - 范围：`hero_down/qp/decision_ao.qm`、生成的 `decision_ao.c/.h` 与 `Tasks/Src/state_task.c`。
  - 完成条件：摩擦轮、拨盘、鼠标锁定、相机目标和 RC 状态均通过 QP 事件在 AO 上下文内修改；AO 外部不再访问 `DecisionAO_inst` 可写实例；模型源与生成代码一致。
  - 状态：TaskMonitor、InitTask、QP 初始化顺序、状态任务死代码清理及台阶检测 `STAIR_OK_SIG` 迁移已完成；其余细粒度 AO 事件迁移尚未完成。
  - 验证：首阶段改动涉及的源文件已使用 ARMClang 6.22.0 完成语法编译；Keil 工程 XML 可解析，新增监控源文件只收录一次，补全后的编译数据库为 148 个唯一源文件。Keil 全量构建因桌面程序审批服务不可用而未启动，仍需完整构建与实机模式切换验证。
- [ ] 新增 `PrivateApplications` 或 `PrivateDrivers` 源文件后，确认对应 Keil 工程已收录，并运行该板输出目录的 `add_missing.py`。
- [ ] 修改上下板共用算法或驱动时，检查两侧实现是否同步，并记录必须保留的板级差异。
- [ ] 修改 Board2Borad 数据定义时，同步核对发送端、接收端、CAN ID、长度、缩放倍率和字节序。
- [ ] [P2] 对下板 `chassisControl` 做行为等价的函数拆分与控制逻辑整理。
  - 范围：`hero_down/PrivateApplications/MainControl/chassisControl.c/.h`，以及 `task_decision.c`、`task_control.c` 中的底盘初始化；首轮不改变控制参数、任务频率、模式行为和 CAN 输出节拍。
  - 初步函数边界：输入源解析与速度斜坡、云台/底盘坐标变换、跟随/自旋角速度生成、底盘状态估计、轮速运动学、功率预算与限幅、轮速 PID、爬坡/缺轮输出分配、最终停机保护、模块初始化。
  - 完成条件：三个公开阶段入口仅负责编排；静态辅助函数输入输出和单位明确；计数器改为带时间语义的状态；移除无效参数与重复初始化前先确认行为；关键控制顺序保持不变。
  - 状态：2026-07-28 至 2026-07-29 已完成首轮函数拆分、初始化收口、周期计数器命名和无效功率估算参数清理；模块私有 `chassis_runtime_t`、`gimbal_runtime_t`、`shoot_runtime_t` 均已内嵌完整控制主体并收纳全部跨周期状态，原独立可写控制实例已删除。三个主要控制实现已移除万能头并使用私有小写叶子字段宏，通用遥控叶子宏集中到 `general_define.h`；鼠标滤波状态迁回 gimbal，Task/模块跨边界写入改为 API 或只读指针，B2B 姿态、云台目标和上位机发送改为局部快照，并删除 shoot 与 music 中仅定义未使用的一批僵尸变量。全部 Task 源文件已改为精确 include，三个控制实现已统一为 `if(condition){` 同行大括号风格。控制参数与执行顺序未调整。
  - 验证：受影响的 chassis、gimbal、stir、joint 及直接相关 Task 源文件已使用 ARMClang 6.22.0 完成语法编译；私有宏引用扫描无僵尸宏，通用小写输入宏未发现跨模块名称冲突，Keil 工程 XML 可解析且三个 internal 头及 `initial_task.h` 均已收录。Keil 全量构建因桌面程序执行审批服务不可用而未启动。仍需完成上下限/零输入/模式切换静态用例，并在实机核对跟随、自旋、上下坡、掉轮降级、功率限制和急停。

## 已知维护问题

- Keil MDK 导出的 `compile_commands.json` 通常只包含 HAL、Middlewares、Core 和 Tasks，缺失 `PrivateApplications` 与 `PrivateDrivers` 源文件。目前通过各板输出目录的 `add_missing.py` 补全。
- 上下板 `PrivateDrivers/LK/LK_driver.h` 中 `LK_SingleLoop_angleControl_limited()` 的 `angle_control` 声明为 `uint32_t`，对应实现为 `uint16_t`；`PrivateApplications/LK` 下还存在另一套重复实现。修复前需确认 LK A6 协议字段宽度、工程实际引用路径，并同步清理上下板重复代码。

## 新增待办格式

```markdown
- [ ] [P1/P2/P3] 事项描述
  - 范围：涉及的板和模块
  - 完成条件：可验证的结果
  - 验证：尚未验证 / 已使用的命令或硬件步骤
```

不要记录密钥、个人信息、未经确认的故障原因或无法验证的结论。
