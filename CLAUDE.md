# CLAUDE.md — 27_3SE Hero 机器人项目

## 项目概述

RoboMaster 机甲大师赛 **Hero 机器人** 嵌入式控制代码。基于 **STM32H723VGTx** 平台（Cortex-M7），运行 **FreeRTOS** 实时操作系统，使用 **Keil MDK-ARM** 编译。

项目为双板架构：
- **hero_down（下板）**：底盘 + 云台 + 发射主控逻辑，包含完整的 `MainControl` 模块 ★ 已重构
- **hero_up（上板）**：世界系云台解算 + 射击控制 ★ 已完成三段式重构（2026-07），与 hero_down 架构一致

## 目录结构

```
27_3SE_hero/
├── hero_down/
│   ├── Core/Inc, Core/Src              # STM32CubeMX HAL 初始化（main.c, stm32h7xx_it.c 等）
│   ├── Drivers/                        # CMSIS + STM32H7 HAL 库
│   ├── Middlewares/                    # FreeRTOS + USB Device Library
│   ├── MDK-ARM/                        # Keil MDK 工程（Hero_Reeeee64.uvprojx）
│   ├── PrivateApplications/            # 控制算法层
│   │   ├── ADRC/                       # 自抗扰控制（TD, ESO, LTD, LESF, ADRC, LADRC）
│   │   ├── Algorism/                   # 通用工具函数（信号处理、多项式、矩阵旋转）
│   │   ├── IMU_solver/                 # EKF IMU 姿态解算（ekf_imu_solver + ekf_quaternion）
│   │   ├── kalman_filter/              # 卡尔曼滤波
│   │   ├── LK/                         # LK 电机应用层
│   │   ├── LvBo/                       # 陷波滤波器（Notch_filter）
│   │   ├── MainControl/                # 主控制逻辑 ★ 仅 hero_down
│   │   │   ├── gimbalControl.c/h       # 云台控制（yaw/pitch 输入决策→观测→闭环）
│   │   │   ├── chassisControl.c/h      # 底盘控制
│   │   │   ├── stirControl.c/h         # 拨弹/发射控制（类型定义 + 三段式接口）
│   │   │   └── jointControl.c/h        # 关节控制（爬升/上楼梯）
│   │   ├── Music/                      # 音乐播放
│   │   ├── PID/                        # PID 控制器
│   │   ├── Shoot_Speed_contrl/         # 弹速最优控制
│   │   ├── Song/                       # 乐谱数据定义
│   │   ├── judge_rec/                  # 裁判系统数据接收
│   │   ├── superSee/                   # 超级电容距离检测
│   │   └── UI_operation/               # UI 操作（串口屏 + 裁判系统 UI）
│   ├── PrivateDrivers/                 # 外设驱动层
│   │   ├── Board2Borad/                # ★ 双板 CAN 通信驱动（下板↔上板）
│   │   ├── CAN/                        # CAN 总线驱动（FDCAN 收发封装）
│   │   ├── CBoardIMU/                  # C 板 IMU（BMI088 驱动 + 温度控制）
│   │   ├── DM-IMU/                     # DM 系列 IMU
│   │   ├── DMJ4310/                    # 达妙 J4310 电机
│   │   ├── DT7/                        # DT7 遥控器（DR16 协议）
│   │   ├── DWT/                        # DWT 周期计数器（精确定时）
│   │   ├── LK/                         # LK 电机驱动（485 总线 + 应用层）
│   │   ├── MIT/                        # MIT 电机协议驱动
│   │   ├── VT13A/                      # VT13A 遥控器 + CRC 校验
│   │   ├── WS2812/                     # WS2812 LED 灯带（SPI 驱动）
│   │   ├── WT901IMU/                   # 维特智能 WT901 IMU
│   │   └── WheelTecIMU/                # WheelTec IMU
│   ├── Tasks/Inc, Tasks/Src            # FreeRTOS 任务
│   │   ├── initial_task.c              # 初始化任务（外设初始化 + 创建所有任务）
│   │   ├── task_decision.c/h           # 决策任务 → InputUpdate（遥控器/PC指令→目标值）
│   │   ├── task_estimate.c/h           # 估计任务 → EstimateUpdate（IMU解算/观测器/滤波）
│   │   ├── task_control.c/h            # 控制任务 → ControlUpdate（PID/ADRC闭环计算）
│   │   ├── state_task.c/h              # 状态机任务 + 任务监控器（TaskMonitor）
│   │   ├── peripheral_transmit_task.c/h # 上位机/裁判系统/板间通讯发送
│   │   ├── peripheral_receive_task.c/h  # 遥控器/裁判系统/板间通讯接收
│   │   └── music_task.c/h              # 音乐播放任务
│   ├── GeneralHeader/                  # 全局定义（硬件引脚映射 + 配置标签）
│   │   ├── general_define.h            # 设备端口映射（UART/SPI/TIM/CAN 宏）
│   │   └── general_config_label.h      # 编译开关 + 模式配置标签
│   └── USB_DEVICE/                     # USB CDC 虚拟串口
├── hero_up/                            # 上板（已完成三段式重构 ✅）
├── .gitignore
├── 27_3SE_hero.code-workspace          # VS Code 工作区（clangd 启用，IntelliSense 禁用）
└── CLAUDE.md                           # 本文件
```

## 架构分层

```
Task 层 (FreeRTOS)        →  initial_task → DecisionTask + EstimateTask + ControlTask + StateMachineTask
     │                         三段式分离到三个独立任务，通过 FreeRTOS 通知同步
     ▼
Application 层            →  PrivateApplications/
     │                         ADRC, PID, MainControl(gimbal/chassis/stir/joint), IMU_solver, kalman_filter...
     ▼
Driver 层                 →  PrivateDrivers/
     │                         Board2Borad, CAN, DMJ4310, MIT, DT7, IMU, WS2812...
     ▼
HAL 层                    →  Drivers/ (STM32H7 HAL) + Core/ (CubeMX 生成)
```

### 控制链三段式（重构后 — hero_down）

原始 `robot_control_task.c` 中的单任务三阶段已被拆分为三个独立 FreeRTOS 任务：

```
IMUTask (7)  ──通知──▶  EstimateTask (5)  ──通知──▶  ControlTask (5)
                                ▲
DecisionTask (5) ──写入共享数据──┘
```

1. **DecisionTask** — 遥控器/PC指令解析 → 目标值写入共享结构体（gimbalTargetInput, chassisTargetInput 等）
2. **EstimateTask** — IMU 姿态解算、观测器更新、滤波 → 发布估计状态（gimbalPose, chassisEstimate 等）
3. **ControlTask** — 读入目标值 + 估计值 → PID/ADRC 闭环计算 → CAN 发送电机指令

每个 MainControl 模块（gimbal/chassis/stir/joint）都提供对应的三段式函数：
- `XxxInputUpdate()` — 由 DecisionTask 调用
- `XxxEstimateUpdate()` — 由 EstimateTask 调用
- `XxxControlUpdate()` — 由 ControlTask 调用

**控制链延迟监控**：`state_task.h` 中定义 `CtrlChainTimer` 结构体，通过 DWT->CYCCNT 精确测量 IMU 通知→Estimate→Control 各段延迟（微秒级）。

## 关键公用接口 — general_task_include.h ★

**`hero_down/Tasks/Inc/general_task_include.h`** 是整个下板工程的唯一中枢头文件，所有 `.c` 文件通过它间接包含所有依赖。它是高频调用的核心接口：

```c
// 标准库 + FreeRTOS + HAL
#include <stdint.h> <math.h> <stdlib.h> <string.h>
#include "main.h" "freertos.h" "queue.h" "semphr.h" "task.h" "event_groups.h"

// GeneralHeader
#include "general_define.h"       // 硬件引脚映射
#include "general_config_label.h"  // 编译开关

// 所有 Task 头文件
#include "state_task.h"            // RobotState, TaskMonitor, CtrlChainTimer, 枚举常量
#include "task_decision.h"         // DecisionTask 声明
#include "task_estimate.h"         // EstimateTask 声明
#include "task_control.h"          // ControlTask 声明
#include "peripheral_transmit_task.h"
#include "peripheral_receive_task.h"
#include "music_task.h"

// Application 层头文件
#include "algorism.h"              // 通用算法
#include "pid.h"                   // PID 控制器
#include "adrc.h"                  // 自抗扰控制
#include "arm_math.h"              // CMSIS-DSP 库
#include "gimbalControl.h"         // 云台控制
#include "chassisControl.h"        // 底盘控制
#include "stirControl.h"           // 拨弹/发射控制
#include "jointControl.h"          // 关节控制
#include "shoot_speed_best_contrl.h"
#include "UI_design.h"
#include "judge_receive.h"
#include "distance_check.h"

// Driver 层头文件
#include "DMJ4310.h"               // 达妙电机
#include "MIT.h"                   // MIT 电机
#include "CAN_driver.h"            // CAN 驱动

// 全局只读指针声明（封装模式：const 指针暴露只读访问）
extern const DJIGMotorRec* _chassisMotorRec;
extern const RobotState* _robotState;
extern const GimbalControl* _gimbalControl;
extern const ChassisControl* _chassisControl;
extern const ShootControl* _shootControl;
// ... 等等
```

**注意**：所有模块间数据共享通过 `general_task_include.h` 中声明的 `const` 指针实现，确保只有「拥有者 task」可写，其他 task 只读。

## hero_up 架构（已重构 ✅）

hero_up 已于 2026-07 完成三段式任务架构重构，与 hero_down 保持一致：

```
IMUTask(p5,2ms) ──通知──▶ EstimateTask(p5,阻塞) ──通知──▶ ControlTask(p5,阻塞)
DecisionTask(p5,10ms) 独立运行
```

### hero_up MainControl 模块
- `PrivateApplications/MainControl/gimbalControl.h/c` — 云台控制（GimbalInputUpdate/PoseUpdate/Init/ControlUpdate/EstimateUpdate）
- `PrivateApplications/MainControl/shootControl.h/c` — 射击控制（ShootInputUpdate/ControlUpdate/EstimateUpdate + CRC16_Modbus）
- `PrivateApplications/MainControl/worldGimbal.h/c` — **世界系云台控制**（hero_up 独有，hero_down 无）
  - 虚拟目标指向 f_des_B + 阻尼最小二乘 IK 反解
  - 详细文档：[gimbalControl.md](gimbalControl.md)

### hero_up vs hero_down 差异对照

| 方面 | hero_down（重构后） | hero_up（已重构 ✅） |
|------|---------------------|---------------------|
| Task 架构 | 三段分离：task_decision + task_estimate + task_control | 三段分离：task_decision + task_estimate + task_control |
| MainControl | ✅ gimbal + chassis + stir + joint | ✅ gimbal + shoot + worldGimbal |
| WorldGimbal | ❌ 无（下板不需要世界系） | ✅ 世界系云台 IK（MainControl/worldGimbal） |
| ChassisControl | ✅ 底盘控制（chassisControl.c） | ❌ 无（底盘由 hero_down 控制，B2B 通信） |
| JointControl | ✅ 关节控制（jointControl.c） | ❌ 无（关节由 hero_down 控制） |
| Board2Borad | ✅ 发送（下板→上板） | ✅ 接收 + 发送（上板→下板） |
| MIT 驱动 | ✅ PrivateDrivers/MIT/ | ❌ 无 |
| general_task_include | include task_decision/estimate/control | include task_decision/estimate/control |
| 485 用途 | MASTER_485_UART (huart2) | SHOOT_485_UART (huart2) |
| ShootControl 类型 | stirControl.h 中定义 | shootControl.h 中定义 |

## 板间通信 — Board2Borad

双板直连 CAN（hfdcan1），ID 段 `0x220-0x22F`，8 字节单帧封装：

```
下板→上板 (Send):
  B2B_DOWN_BODY_STATE    0x220  [roll, pitch, yaw, yaw_enc]×100    机体姿态
  B2B_DOWN_GIMBAL_INPUT  0x221  [pitch_cmd, yaw_cmd, ch0, ch1]     云台控制输入
  B2B_DOWN_KEYS_SWITCH   0x222  [PCKey, switch, ch4, HP]           键位开关

上板→下板 (Recv):
  B2B_UP_GIMBAL_POSE     0x228  云台姿态（高频）
  B2B_UP_GIMBAL_TARGET   0x229  云台目标（低频）
  B2B_UP_FRIC_RPM_A/B    0x22A/B 摩擦轮转速
```

## 构建

- **IDE**：STM32CubeIDE 生成初始化代码 + Keil MDK-ARM 编译调试
- **工程文件**：`hero_down/MDK-ARM/Hero_Reeeee64.uvprojx`、`hero_up/MDK-ARM/Hero_Reeeee64.uvprojx`
- **编译命令**：通过 Keil MDK IDE 或命令行 `UV4.exe -b Hero_Reeeee64.uvprojx`
- **目标芯片**：STM32H723VGTx（Cortex-M7, 480MHz）
- **下载调试**：J-Link / ST-Link

## 编码规范

### 命名约定

| 元素 | 风格 | 示例 |
|------|------|------|
| 函数（公开） | CamelCase + 模块前缀 | `GimbalInit()`, `LADRCUpdate()`, `TDInitialize()` |
| 函数（静态） | 小写蛇形 | `controlInit()`, `chassisInputUpdate()` |
| 变量（局部/字段） | snake_case + 单位后缀 | `yaw_angle_d`, `speed_x_mps`, `target_pos_rad` |
| 类型定义 | CamelCase（无 `_t` 后缀） | `GimbalControl`, `PIDStruct`, `ADRC` |
| 宏定义 | UPPER_SNAKE_CASE | `CONTROL_STOP`, `MATCH_MODE`, `P_MIN` |
| 枚举值 | UPPER_SNAKE_CASE | `CHASSIS_FOLLOW`, `STIR_LOCK` |
| 头文件保护 | `_MODULE_H_` | `_GIMBAL_CONTROL_H_` |

### 物理量单位后缀（重要！一致性很高）

| 后缀 | 含义 | 示例 |
|------|------|------|
| `_d` / `_deg` | 角度（度） | `yaw_angle_d`, `pitch_angle_deg` |
| `_rad` | 角度（弧度） | `target_pos_rad` |
| `_dps` | 角速度（度/秒） | `yaw_angular_velocity_dps` |
| `_radps` | 角速度（弧度/秒） | `vel_radps` |
| `_mps` | 线速度（米/秒） | `speed_x_mps` |
| `_rps` | 转速（转/秒） | `speed_w_rps` |
| `_rpm` | 转速（转/分） | `mechanical_speed_rpm` |
| `_nm` | 力矩（牛米） | `torque_cmd_nm` |

### 封装模式

```c
// 全局可写实例（仅模块内可写）
GimbalControl gimbalControl = {0};

// const 只读指针（外部只读访问）
const GimbalControl* _gimbalControl = &gimbalControl;

// 头文件暴露只读指针
extern const GimbalControl* _gimbalControl;
```

### 格式

- **缩进**：4 空格（无 Tab）
- **大括号**：Allman 风格（单独占行）
- **行宽**：不严格限制，长表达式用 `\` 续行
- **`if` / `for` / `while`**：关键字与 `(` 间无空格：`if(condition)`

### 注释

- API 函数头：Doxygen `@brief @param @retval @note` 风格
- 文件头：`@file @brief @note`
- 模组内分区：`/*--- 区域名称 ---*/` 分隔线
- 内联解释：`//` 中文自由注释

### 编译开关

项目大量使用 `#ifdef` 条件编译隔离配置（定义在 `general_config_label.h`）：
```c
#define MATCH_MODE           // 比赛模式
#define SHOOT_OFF            // 关闭发射
#define CHASSIS_OFF          // 关闭底盘
#define GIMBAL_OFF           // 关闭云台
#define OLD_CAPACITY         // 老电容控制板
#define CENTRIFUGE_REVOLVE   // 离心旋转
#define DEBUG_PCB_EN         // 调试 PCB
```

## 新增模块指南

当添加新的控制算法或驱动模块时：

1. 在 `PrivateApplications/` 或 `PrivateDrivers/` 下创建文件夹 `ModuleName/`
2. 包含 `module_name.h`（公开接口）和 `module_name.c`（实现）
3. 头文件模板：
```c
#ifndef _MODULE_NAME_H_
#define _MODULE_NAME_H_

#include "general_task_include.h"

typedef struct {
    // 状态字段
} ModuleName;

/**
 * @brief   初始化
 * @param   module 模块结构体
 * @retval  void
 */
void ModuleNameInitialize(ModuleName* module);

#endif
```
4. **高频模块**：将头文件加入 `Tasks/Inc/general_task_include.h` 的 include 列表，这样所有 task 和模块都能访问
5. 如需暴露只读数据，在 `general_task_include.h` 中声明 `extern const ModuleName* _moduleName;`
6. 初始化函数在 `initial_task.c` 或相应的 task 中调用
7. 遵循标准分层依赖：`Task → Application → Driver → HAL`
8. **同步到 hero_up**：如果模块同时被上下板使用，确认 hero_up 的 `general_task_include.h` 也包含对应头文件

## 注意事项

- **hero_down 和 hero_up 均已重构为三段式任务架构**，通过 IMU→Estimate→Control 通知链实现高频同步
- **`general_task_include.h` 是高频公用接口**，所有模块的数据共享、const 指针暴露、头文件包含都集中在这里；新增模块时必须同步更新
- **Board2Borad** 是双板通信的核心通道，修改 CAN 配置时注意不与电机 ID（0x01-0x08, 0x141）冲突
- **FreeRTOS 任务优先级** 在 `initial_task.c` 创建任务时指定，修改时注意实时性约束：
  - **hero_down**: `7`: RemoteRecTask, IMUTask / `6`: StateMachineTask / `5`: DecisionTask, EstimateTask, ControlTask, UpperPCCommTask / `4`: MonitorTask, DebugTask / `3`: UIOperationTask / `2`: MusicTask
  - **hero_up**: `7`: MonitorTask, StateMachineTask / `5`: DecisionTask, EstimateTask, ControlTask, IMUTask, UpperPCCommTask / `4`: DebugTask / `2`: MusicTask
- **CAN 总线** 是主要的外设通信方式，电机、IMU、裁判系统、板间通信均通过 CAN 连接
- **ADRC 模块** 的 `hero_down` 版本有额外的安全检查（`isfinite`），比 `hero_up` 更成熟
- **调试信息** 通过 `peripheral_transmit_task.c` 向 DT7 上位机发送
- **hero_down 和 hero_up 的 `PrivateApplications/` 和 `PrivateDrivers/` 代码应保持一致**，修改算法模块时应同步两个代码树的对应文件
- **禁止用 PowerShell 编辑含中文的源文件**：PowerShell 5.1 的 `Get-Content` 默认按系统编码（中文 Windows = GBK）读取 UTF-8 文件，`Set-Content -Encoding UTF8` 写回会导致中文乱码。替代方案：Git Bash 的 `sed`/`awk`、Python、或 IDE 的 Edit/Write 工具
- **新增代码前必须逐行检查周围代码的风格**：缩进字符（Tab 还是空格，几级）、注释格式（Doxygen 风格、中文注释风格）、大括号位置、`if(` vs `if (` 等。直接复制粘贴周围函数的骨架再填内容
- **`git checkout --` 前必须确认**：该命令不可逆地丢弃工作区未提交修改。执行前必须告知用户哪些文件会被覆盖，征得同意
