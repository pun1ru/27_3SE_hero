# CLAUDE.md — 27_3SE Hero 机器人项目

## 项目概述

RoboMaster 机甲大师赛 **Hero 机器人** 嵌入式控制代码。基于 **STM32H723/H743** 平台，运行 **FreeRTOS** 实时操作系统，使用 **Keil MDK-ARM** 编译。

项目为双板架构：
- **hero_down（下板）**：底盘 + 云台 + 发射主控逻辑，包含完整的 `MainControl` 模块
- **hero_up（上板）**：hero_down 的镜像精简版，无 `MainControl`，两个代码树的 `PrivateApplications/` 和 `PrivateDrivers/` 共用相同模块

## 目录结构

```
27_3SE_hero/
├── hero_down/
│   ├── Core/Inc, Core/Src          # STM32CubeMX HAL 初始化（main.c, stm32h7xx_it.c 等）
│   ├── Drivers/                    # CMSIS + STM32H7 HAL 库
│   ├── MDK-ARM/                    # Keil MDK 工程（Hero_Reeeee64.uvprojx）
│   ├── PrivateApplications/        # 控制算法层
│   │   ├── ADRC/                   # 自抗扰控制（TD, ESO, LTD, LESF, ADRC, LADRC）
│   │   ├── Algorism/               # 通用工具函数（信号处理、多项式等）
│   │   ├── MainControl/            # 主控制逻辑 ★ 仅 hero_down
│   │   │   ├── gimbalControl.c/h   # 云台控制（yaw/pitch 输入决策→观测→闭环）
│   │   │   ├── chassisControl.c/h  # 底盘控制
│   │   │   ├── shootControl.c/h    # 发射控制
│   │   │   └── jointControl.c/h    # 关节控制
│   │   ├── PID/                    # PID 控制器
│   │   ├── IMU_solver/             # IMU 姿态解算
│   │   ├── kalman_filter/          # 卡尔曼滤波
│   │   ├── judge_rec/              # 裁判系统数据接收
│   │   ├── LK/                     # LK 电机驱动
│   │   ├── LvBo/                   # 滤波模块
│   │   ├── Music/, Song/           # 音乐播放
│   │   ├── Shoot_Speed_contrl/     # 弹速控制
│   │   ├── superSee/               # 超级电容
│   │   └── UI_operation/           # UI 操作
│   ├── PrivateDrivers/             # 外设驱动层
│   │   ├── CAN/                    # CAN 总线驱动
│   │   ├── DMJ4310/                # 达妙电机
│   │   ├── DM-IMU/                 # DM IMU
│   │   ├── CBoardIMU/              # C 板 IMU
│   │   ├── DWT/                    # DWT 定时器
│   │   ├── DT7/                    # DT7 遥控器
│   │   ├── WS2812/                 # LED 灯带
│   │   ├── VT13A/                  # VT13A
│   │   └── WheelTecIMU/            # WheelTec IMU
│   ├── Tasks/Inc, Tasks/Src        # FreeRTOS 任务
│   │   ├── robot_control_task.c    # DecisionTask + ControlTask（核心控制循环）
│   │   ├── peripheral_transmit_task.c # 上位机/裁判系统通讯
│   │   └── ...
│   └── GeneralHeader/              # 全局硬件定义（引脚、外设句柄、task include）
├── hero_up/                        # 上板（结构镜像，无 MainControl）
├── .gitignore
├── 27_3SE_hero.code-workspace      # VS Code 工作区（clangd 启用，IntelliSense 禁用）
└── CLAUDE.md                       # 本文件
```

## 架构分层

```
Task 层 (FreeRTOS)        →  robot_control_task.c, peripheral_transmit_task.c
     │                         决策任务(DecisionTask) + 控制任务(ControlTask)
     ▼
Application 层            →  PrivateApplications/
     │                         ADRC, PID, MainControl, IMU_solver, kalman_filter...
     ▼
Driver 层                 →  PrivateDrivers/
     │                         CAN, DMJ4310, DT7, IMU...
     ▼
HAL 层                    →  Drivers/ (STM32H7 HAL) + Core/
```

**控制循环三段式**（核心设计模式，如 `gimbalControl.c`）：
1. **InputUpdate** — 输入决策（遥控器指令解析、模式切换）
2. **EstimateUpdate** — 状态观测/估计（滤波、IMU 解算、观测器）
3. **ControlUpdate** — 闭环控制计算（PID/ADRC 输出执行）

## 构建

- **IDE**：STM32CubeIDE 生成初始化代码 + Keil MDK-ARM 编译调试
- **工程文件**：`hero_down/MDK-ARM/Hero_Reeeee64.uvprojx`
- **编译命令**：通过 Keil MDK IDE 或命令行 `UV4.exe -b Hero_Reeeee64.uvprojx`
- **目标芯片**：STM32H723VGTx / STM32H743 系列
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

项目大量使用 `#ifdef` 条件编译隔离配置：
```c
#define MATCH_MODE     // 比赛模式
#define SHOOT_OFF      // 关闭发射
#define CHASSIS_OFF    // 关闭底盘
#define GIMBAL_OFF     // 关闭云台
#define OLD_CAPACITY   // 老电容控制板
#define CENTRIFUGE_REVOLVE  // 离心旋转
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
4. 初始化函数在 `robot_control_task.c` 或相应的 task 中调用
5. 遵循标准分层依赖：`Task → Application → Driver → HAL`

## 注意事项

- **hero_down 和 hero_up 的 `PrivateApplications/` 代码大部分相同**，修改算法模块时应同步两个代码树的对应文件，或至少确认差异是有意的
- **FreeRTOS 任务优先级** 在 `FreeRTOSConfig.h` 和 `robot_control_task.c` 中定义，修改时注意实时性约束
- **CAN 总线** 是主要的外设通信方式，电机、IMU、裁判系统均通过 CAN 连接
- **ADRC 模块** 的 `hero_down` 版本有额外的安全检查（`isfinite`），比 `hero_up` 更成熟
- **调试信息** 通过 `peripheral_transmit_task.c` 向 DT7 上位机发送