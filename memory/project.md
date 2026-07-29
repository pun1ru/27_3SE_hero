# 项目与架构

## 项目概述

本项目是 RoboMaster 机甲大师赛 Hero 机器人嵌入式控制代码，平台为 STM32H723VGTx（Cortex-M7），运行 FreeRTOS，使用 Keil MDK-ARM 编译。

双板职责：

- `hero_down`：底盘、云台、拨弹和关节控制，包含 `gimbalControl`、`chassisControl`、`stirControl`、`jointControl`。
- `hero_up`：世界系云台解算和射击控制，包含 `gimbalControl`、`shootControl`、`worldGimbal`。
- 上下板均已完成 Decision、Estimate、Control 三段式任务架构重构。

## 分层结构

```text
Task 层             FreeRTOS 任务、状态机、任务通知和外设收发
    ↓
Application 层      MainControl、ADRC、PID、IMU 解算和滤波
    ↓
Driver 层           CAN、电机、遥控器、IMU 和板间通信
    ↓
HAL 层              STM32H7 HAL、CMSIS 和 CubeMX 生成代码
```

主要目录：

- `Core`：CubeMX 生成的初始化和中断代码。
- `Drivers`、`Middlewares`：CMSIS、HAL、FreeRTOS 和 USB 库。
- `PrivateApplications`：控制算法和应用模块。
- `PrivateDrivers`：CAN、电机、遥控器、IMU 和其他外设驱动。
- `Tasks/Inc`、`Tasks/Src`：FreeRTOS 任务。
- `GeneralHeader`：硬件映射、全局配置和编译开关。
- `MDK-ARM`：Keil 工程和编译输出。

## 三段式控制链

下板：

```text
IMUTask (p7) ──通知──> EstimateTask (p5) ──通知──> ControlTask (p5)
                              ^
DecisionTask (p5) ──共享目标──┘
```

上板：

```text
IMUTask (p5, 2 ms) ──通知──> EstimateTask (p5) ──通知──> ControlTask (p5)
DecisionTask (p5, 10 ms) 独立更新目标输入
```

- `DecisionTask`：解析遥控器、PC 或板间指令，更新目标输入。
- `EstimateTask`：执行 IMU 姿态解算、观测器和滤波，发布估计状态。
- `ControlTask`：读取目标和估计状态，执行 PID/ADRC 并发送电机指令。
- MainControl 模块对应提供 `XxxInputUpdate()`、`XxxEstimateUpdate()`、`XxxControlUpdate()`。
- 下板通过独立 `task_monitor` 模块记录各任务帧数、周期、故障位和 DWT 控制链延迟；任务只调用监控 API，不直接修改监控状态。

## 公共接口与所有权

各板 `Tasks/Inc/general_task_include.h` 是公共头文件入口，集中包含 Task、Application、Driver 和全局配置依赖。

共享状态采用所有权封装：

```c
static gimbal_runtime_t g_gimbal_runtime = {0};
const GimbalControl *const _gimbalControl = &g_gimbal_runtime.control;
```

模块拥有者只保留一个私有可写 runtime 实例，完整控制主体位于 runtime 的 `control` 成员中；其他任务和模块通过 `extern const ... *const` 只读访问该成员，不得再并列定义独立可写控制实例。

下板 `chassis`、`gimbal`、`shoot` 的持续运行状态分别收口在模块私有的 `chassis_runtime_t`、`gimbal_runtime_t`、`shoot_runtime_t` 中。对应 `*_internal.h` 只供本模块实现使用，公共头不暴露运行时状态和简化宏；三个控制实现文件不再包含 `general_task_include.h`。

下板 QP/C 由 `InitTask` 在任务启动前调用 `QpInit()` 完成 `QF_init`、事件池、发布订阅和 `DecisionAO` 启动。遥控事件组同样在启用 UART/CAN 接收前创建，避免中断先于任务资源初始化。

## 上下板差异

| 项目 | hero_down | hero_up |
|---|---|---|
| MainControl | gimbal、chassis、stir、joint | gimbal、shoot、worldGimbal |
| 世界系云台 | 无 | `worldGimbal` 阻尼最小二乘 IK |
| 底盘和关节 | 本板控制 | 无 |
| Board2Borad | 发送下板状态与输入 | 接收下板数据并回传上板状态 |
| MIT 驱动 | 有 | 无 |
| UART2 用途 | `MASTER_485_UART` | `SHOOT_485_UART` |
| ShootControl 类型 | `stirControl.h` | `shootControl.h` |

## 板间通信

上下板通过 FDCAN3 直连，使用 CAN ID `0x220-0x22F`，8 字节单帧封装。

下板到上板：

- `0x220 B2B_DOWN_BODY_STATE`：roll、pitch、yaw、yaw encoder，按 100 倍缩放。
- `0x221 B2B_DOWN_GIMBAL_INPUT`：pitch/yaw 指令和遥控通道输入。
- `0x222 B2B_DOWN_KEYS_SWITCH`：PC 键位、拨杆、通道和血量信息。

上板到下板：

- `0x228 B2B_UP_GIMBAL_POSE`：高频云台姿态。
- `0x229 B2B_UP_GIMBAL_TARGET`：低频云台目标。
- `0x22A/0x22B B2B_UP_FRIC_RPM_A/B`：摩擦轮转速。

## 构建与 clangd

- 下板工程：`hero_down/MDK-ARM/Hero_Reeeee64.uvprojx`。
- 上板工程：`hero_up/MDK-ARM/Hero_Reeeee64.uvprojx`。
- 使用 Keil IDE 或 `UV4.exe -b Hero_Reeeee64.uvprojx` 构建，使用 J-Link 或 ST-Link 调试。
- 根 `.clangd` 只保存全局诊断和索引设置。
- `hero_down/.clangd` 与 `hero_up/.clangd` 分别绑定各自编译数据库和专属 include 路径。
- 工程专属配置不能放到根 `.clangd`，否则会造成上下板同名符号和头文件交叉污染。

编译数据库：

- `hero_down/MDK-ARM/out/Hero_Reeeee64/Hero_Reeeee64/compile_commands.json`
- `hero_up/MDK-ARM/out/Hero_Reeeee64/Hero_Reeeee64/compile_commands.json`

Keil 导出的数据库通常缺失 `PrivateApplications` 和 `PrivateDrivers` 源文件，需要运行对应输出目录中的 `add_missing.py` 补全。

## 任务优先级基线

下板：

- p7：`RemoteRecTask`、`IMUTask`
- p6：`StateMachineTask`
- p5：`DecisionTask`、`EstimateTask`、`ControlTask`、`UpperPCCommTask`
- p4：`MonitorTask`、`DebugTask`
- p3：`UIOperationTask`
- p2：`MusicTask`

上板：

- p7：`MonitorTask`、`StateMachineTask`
- p5：`DecisionTask`、`EstimateTask`、`ControlTask`、`IMUTask`、`UpperPCCommTask`
- p4：`DebugTask`
- p2：`MusicTask`

## 其他稳定信息

- 常用编译开关位于 `general_config_label.h`，包括 `MATCH_MODE`、`SHOOT_OFF`、`CHASSIS_OFF`、`GIMBAL_OFF`、`OLD_CAPACITY`、`CENTRIFUGE_REVOLVE`、`DEBUG_PCB_EN`。
- `hero_down` ADRC 版本包含额外 `isfinite` 安全检查，同步时不能无意移除。
- 调试信息通过 `peripheral_transmit_task.c` 发送至 DT7 上位机。
