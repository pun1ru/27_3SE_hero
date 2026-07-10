# Tasks

## 目标一-规范hero_up的头文件应用 — ✅ 已完成

参考hero_down的`general_task_include.h`，已在hero_up的对应文件中加入所有常用应用和驱动头文件：

**修改文件：** `hero_up/Tasks/Inc/general_task_include.h`
- 移除 `#include "robot_control_task.h"`（改为引用MainControl头文件）
- 新增: `task_estimate.h`, `task_decision.h`, `task_control.h`
- 新增: `algorism.h`, `pid.h`, `adrc.h`, `arm_math.h`, `shoot_speed_best_contrl.h`
- 新增: `DMJ4310.h`, `CAN_driver.h`
- 新增: `extern TaskHandle_t estimateTaskHandle;`

**修改文件：** `hero_up/Tasks/Inc/robot_control_task.h`
- 移除 GimbalControl、ShootControl、WorldGimbal 类型定义（已搬迁至MainControl头文件）
- 保留 ChassisControl 类型定义
- 改为引用 MainControl 头文件（`gimbalControl.h`, `shootControl.h`, `worldGimbal.h`）

## 目标二-参考hero_down的任务调度和任务划分 — ✅ 已完成

将原有的单体`robot_control_task.c`（1135行）拆分为三段式任务架构：

```
IMUTask(p5,2ms) ──通知──▶ EstimateTask(p5,阻塞) ──通知──▶ ControlTask(p5,阻塞)
DecisionTask(p5,10ms) 独立运行
```

**新增/修改的文件：**
- `Tasks/Src/task_decision.c` — DecisionTask（10ms固定周期，调用 GimbalInputUpdate + ShootInputUpdate）
- `Tasks/Src/task_estimate.c` — EstimateTask（通知驱动，等待IMU通知 → WorldGimbalEstimateUpdate → 通知ControlTask）
- `Tasks/Src/task_control.c` — ControlTask（通知驱动，等待Estimate通知 → IK → ADRC → PID → CAN发送）
- `Tasks/Src/initial_task.c` — 新增 EstimateTask 创建，优先调整 Decision(p4→p5), Control(p6→p5)
- `Tasks/Src/peripheral_receive_task.c` — IMUTask 末尾通知 estimateTaskHandle
- `Tasks/Src/peripheral_transmit_task.c` — 移除 ALLHighFreqCal 调用（已拆分到 EstimateTask + GimbalControlUpdate）
- `Tasks/Src/robot_control_task.c` — 清空，保留注释标注搬迁映射

**新增 TaskMonitor 支持：**
- `Tasks/Inc/state_task.h` — TaskFrameCounterPtr/TaskRunPeriodPtr 添加 _estimate_task，新增 ESTIMATE_TASK_MASK
- `GeneralHeader/general_config_label.h` — 新增 ESTIMATE_TASK_NUM(9), CREATE_TASK_NUM(9+1)
- `Tasks/Src/state_task.c` — BlockOrDisturbDetect 添加 EstimateTask 卡死检测

## 目标三-优化掉僵尸变量声明和注释没有用的东西 — ✅ 已完成

清理的僵尸代码：
- `int mardio;` ×2（robot_control_task.c 第258行、第1052行）
- `see_error1/2/3`, `w_d2`, `k_ff`, `wd3`, `wd3p`
- `finish_flag`, `target_torque`, `w_d`, `last_compensation`, `compensation`
- `gravity_compensation`, `fric_compensation`, `lowpass_pitch`
- `last_u`, `fl_u`, `temp_yaw`, `last_temp_yaw`
- 注释掉的旧 PID/ADRC/舵机代码（未搬迁至新文件）
- ALLHighFreqCal 函数（已按要求拆分搬迁）

注意：部分看似僵尸的变量（如 `micro_pitch`, `shit_delay_count`, `shit_temp_pitch_comp` 等）实际上是活跃使用的，已保留并搬迁。

## 目标四-参考hero_down的MainControl文件夹 — ✅ 已完成

### MainControl 模块（新建/填充）

**`PrivateApplications/MainControl/gimbalControl.h/c`** — 云台控制
- GimbalControl 类型定义 + GimbalInputUpdate/PoseUpdate/Init/ControlUpdate/EstimateUpdate

**`PrivateApplications/MainControl/shootControl.h/c`** — 射击控制
- ShootControl 类型定义 + ShootInputUpdate/ControlUpdate/EstimateUpdate + CRC16_Modbus

**`PrivateApplications/MainControl/worldGimbal.h/c`** — 世界系云台控制
- WorldGimbal 类型定义 + 全部 FK/IK/重力计算实现

### gimbalControl.md 文档 ✅

已创建 `gimbalControl.md`，文档内容包括：
1. 云台控制系统总览
2. 控制模式：普通模式 vs 世界系模式（WorldGimbal）
3. WorldGimbal 原理：坐标系定义（B系x前y右z下）、FK/IK 数学、重力方向计算、阻尼最小二乘 IK
4. 控制输入来源：遥控器（CONTROL_FROM_REMOTE）、上位机（CONTROL_FROM_PC）、自动瞄准、Q键预设
5. 控制算法：pitch LTD+ADRC、yaw LTD+LTDPID、摩擦轮 PID、狙击/关节编码器位置控制
6. 观测量：IMU vs 编码器（sniper_on用编码器防漂移）
7. 特殊模式：狙击模式（sniper）、关节爬升模式（joint_climb）、CONTROL_STOP 保护

### 代码搬迁映射表

| 原始位置 (robot_control_task.c) | 新位置 |
|------|------|
| GimbalInputUpdate (行135-285) | MainControl/gimbalControl.c |
| GimbalPoseUpdate (行1029-1076) | MainControl/gimbalControl.c |
| GimbalInit/ControlInit (行825-900) | MainControl/gimbalControl.c GimbalInit() |
| GimbalControlUpdate (行1090-1113) | MainControl/gimbalControl.c |
| ALLHighFreqCal (行927-1010) | MainControl/gimbalControl.c GimbalControlUpdate() |
| GimbalEstimateUpdate (行1014-1016) | MainControl/gimbalControl.c |
| ShootInputUpdate (行329-386) | MainControl/shootControl.c |
| ShootControlUpdate (行1118-1134) | MainControl/shootControl.c |
| ShootEstimateUpdate (行1080-1083) | MainControl/shootControl.c |
| CRC16_Modbus (行303-326) | MainControl/shootControl.c |
| WorldGimbal全部API (行388-768) | MainControl/worldGimbal.c |
| DecisionTask (行85-112) | Tasks/Src/task_decision.c |
| ControlTask (行781-818) | Tasks/Src/task_control.c |

## 注意事项

1. **Keil MDK 工程需要更新**：添加新文件到工程（MainControl/*.c, Tasks/Src/task_*.c），添加 include 路径（MainControl目录）
2. **clangd 配置**：`.clangd` 或 `compile_commands.json` 需要添加 MainControl 目录的 include 路径
3. **hero_down 同步**：hero_down 的 `PrivateApplications/MainControl/` 代码应与此重构后的 hero_up 保持接口一致
