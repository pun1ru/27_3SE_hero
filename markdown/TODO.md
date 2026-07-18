# QPC 接入 27_3SE_hero — 完整复刻 serialleg QM 模型

## serialleg 完整模型结构

```
model (framework: qpc, version: 6.1.1)
│
├── package "Demo" (stereotype=0x00)
│   └── class "Class1" : qpc::QActive
│       ├── attribute × 60+          成员变量（0x00=public, 0x02=private）
│       ├── operation × 6            类方法（State_update, State_Stable, is_Stable, LqrLock...）
│       └── statechart (SM)           状态图（Fall→Stand→Rotate→Action...）
│
├── package "Shared" (stereotype=0x00)
│   ├── attribute "StateMachine_Sig"   enum, 0x04   信号枚举
│   ├── attribute "ctrl_mode_enum"     enum, 0x04   控制模式枚举
│   ├── attribute "chassis_mode_enum"  enum, 0x04   底盘模式枚举
│   ├── operation "Class1_ctor"       void, 0x00    构造函数
│   ├── attribute "AO_Class1"          QActive*, 0x00    AO指针
│   └── attribute "pClass"             const Class1*, 0x00   只读指针
│
└── directory "directory1"
    ├── file "decision.c"              生成模板
    └── file "decision.h"              生成模板
```

## visibility 对照表（serialleg 实际用的）

| 值 | QM界面 | 用在 |
|----|--------|------|
| 0x00 | public | AO成员变量、operation、AO/const指针 |
| 0x02 | private | QTimeEvt、内部标志位 |
| 0x04 | （QM自动选的） | 包级自由枚举 |

---

## 第1步：新建模型
- File → New Model → **None**, Framework: **qpc**
- Save As → `hero_down\qp\hero_ao.qm`

## 第2步：建 Package 和目录

| 操作 | Name |
|------|------|
| 右键模型 → Add Package | `Hero_Demo` |
| 右键模型 → Add Package | `Hero_Shared` |
| 右键模型 → Add Directory | `directory1` |

目录 `directory1` 里不放东西，serialleg 用它指定生成路径。

## 第3步：Hero_Shared 包 — 加枚举 attribute

右键 `Hero_Shared` → Add Attribute：

### 3.1 StateMachineSig (type=enum, visibility 留默认让QM选)

代码编辑器填：
```
{
    SWITCH_PC_SIG = Q_USER_SIG,
    SWITCH_REMOTE_SIG,
    SWITCH_SNIPER_SIG,
    KEY_X_SIG,
    KEY_Q_SIG,
    KEY_F_SIG,
    KEY_G_SIG,
    KEY_C_SIG,
    KEY_V_SIG,
    KEY_B_SIG,
    KEY_E_SIG,
    KEY_Z_SIG,
    CH4_POS_SIG,
    CH4_NEG_SIG,
    L1_UP_SIG,
    L1_DOWN_SIG,
    MOUSE_LEFT_SIG,
    RC_LOST_SIG,
    MAX_PUB_SIG,
    TIMEOUT_SIG,
    MAX_SIG
};
```

### 3.2 CtrlTerminalEnum (type=enum)

```
{ CTRL_STOP = 0, CTRL_REMOTE, CTRL_PC };
```

### 3.3 ChassisModeEnum (type=enum)

```
{ CHASSIS_FOLLOW = 0, CHASSIS_FOLLOW_BACK, CHASSIS_REVOLVE, CHASSIS_SEPARATE };
```

## 第4步：Hero_Shared 包 — 加 operation（构造函数）

右键 `Hero_Shared` → Add Operation：

| Name | Type | Visibility |
|------|------|-----------|
| `DecisionAO_ctor` | `void` | public |

双击 → 代码编辑器：
```
DecisionAO * const me = &DecisionAO_inst;
QActive_ctor(&me->super, Q_STATE_CAST(&DecisionAO_initial));
QTimeEvt_ctorX(&me->timeEvt_test, &me->super, TIMEOUT_SIG, 0U);
me->ctrl_terminal = CTRL_STOP;
me->chassis_mode = CHASSIS_FOLLOW;
me->sniper = 0;
me->fric_mode = 0;
me->stir_mode = 0;
me->joint_mode = 0;
me->stand_mode = 0;
me->mouse_fix = 0;
me->cam_target = 2;
me->world_enable = 0;
me->stable = 1;
me->rc_lost_flag = 0;
```

## 第5步：Hero_Shared 包 — 加 AO 指针 attribute

右键 `Hero_Shared` → Add Attribute：

| Name | Type | Visibility | 代码 |
|------|------|-----------|------|
| `AO_DecisionAO` | `QActive * const` | public | ` = &DecisionAO_inst.super;` |
| `pDecisionAO` | `const DecisionAO *` | public | `= &DecisionAO_inst;` |

## 第6步：Hero_Demo 包 — 建 AO 类

右键 `Hero_Demo` → Add Class：
- Name: `DecisionAO`
- Superclass: `qpc::QActive`

## 第7步：加 AO attribute

右键 `DecisionAO` → Add Attribute：

### public (0x00)

| Name | Type |
|------|------|
| `ctrl_terminal` | `uint8_t` |
| `chassis_mode` | `uint8_t` |
| `sniper` | `uint8_t` |
| `fric_mode` | `uint8_t` |
| `stir_mode` | `uint8_t` |
| `joint_mode` | `uint8_t` |
| `stand_mode` | `uint8_t` |
| `mouse_fix` | `uint8_t` |
| `cam_target` | `uint8_t` |
| `world_enable` | `uint8_t` |
| `stable` | `uint8_t` |

### private (0x02)

| Name | Type |
|------|------|
| `timeEvt_test` | `QTimeEvt` |
| `rc_lost_flag` | `uint8_t` |

## 第8步：加类 operation（右键 DecisionAO → Add Operation）

| Name | Type | Visibility | 代码 |
|------|------|-----------|------|
| `State_Stable` | `void` | public | `me->stable = 1;` |
| `State_Unstable` | `void` | public | `me->stable = 0;` |
| `is_Stable` | `uint8_t` | public | `return me->stable;` |

## 第9步：加 State Machine + 画状态图

右键 `DecisionAO` → Add State Machine

### 4个状态（比之前多一个 StopMode）

| 状态 | 含义 | entry 代码 |
|------|------|-----------|
| `StopMode` | 保护/停止（默认） | `me->ctrl_terminal = CTRL_STOP;` |
| `PcMode` | PC键盘鼠标 | `me->ctrl_terminal = CTRL_PC;` |
| `RemoteMode` | 遥控器 | `me->ctrl_terminal = CTRL_REMOTE; me->sniper = 0;` |
| `SniperMode` | 遥控吊射 | `me->ctrl_terminal = CTRL_REMOTE; me->sniper = 1;` |

### 迁移（箭头 + trigger）

```
黑点 → StopMode     （开机默认保护）

StopMode → PcMode         trigger: SWITCH_PC_SIG
StopMode → RemoteMode     trigger: SWITCH_REMOTE_SIG
StopMode → SniperMode     trigger: SWITCH_SNIPER_SIG

PcMode → RemoteMode       trigger: SWITCH_REMOTE_SIG
PcMode → SniperMode       trigger: SWITCH_SNIPER_SIG
PcMode → StopMode         trigger: RC_LOST_SIG

RemoteMode → PcMode       trigger: SWITCH_PC_SIG
RemoteMode → SniperMode   trigger: SWITCH_SNIPER_SIG
RemoteMode → StopMode     trigger: RC_LOST_SIG

SniperMode → PcMode       trigger: SWITCH_PC_SIG
SniperMode → RemoteMode   trigger: SWITCH_REMOTE_SIG
SniperMode → StopMode     trigger: RC_LOST_SIG
```

### 逻辑解释

```
遥控器 R1拨杆 → task_decision_state 检测到位置变化
               → QACTIVE_POST(AO_DecisionAO, SWITCH_PC_SIG)
               → AO 收到信号，触发状态迁移
               → 新的状态 entry 写 me->ctrl_terminal
               → 控制任务读到 _decisionAO->ctrl_terminal → 改变行为
```

serialleg 也是这个模式：`Fall` 是初始保护态 → 收到 `SwitchR_mid/down` → 解锁进入 `Stand`。

### 12个 trigger — 跟 enum 里定义的信号逐个对应

你在 enum 里定义了什么信号，画箭头时 Trigger 就填什么名字。QM 会自动匹配。

Ctrl+G → Output dir: `hero_down\PrivateApplications\MainControl\`

生成 `decision_ao.h` + `decision_ao.c`

## 第11步：qpc_init.c 启动 AO（手写）

```c
#include "decision_ao.h"

// QpInit() 里加:
DecisionAO_ctor();
static QEvt const *decisionQueue[64];
static StackType_t decisionStack[1024];
QActive_setAttr(AO_DecisionAO, TASK_NAME_ATTR, "DecisionAO");
QACTIVE_START(AO_DecisionAO, 5U, decisionQueue, Q_DIM(decisionQueue),
              decisionStack, sizeof(decisionStack), (void *)0);
```

## 第12步：发事件 + 读状态（手写）

```c
// 发事件（如 task_decision_state.c）:
QACTIVE_POST(AO_DecisionAO, Q_NEW(QEvt, SWITCH_PC_SIG), NULL);

// 读状态（如 chassisControl.c）:
#include "decision_ao.h"
uint8_t mode = pDecisionAO->chassis_mode;
```
