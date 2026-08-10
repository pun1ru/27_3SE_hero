# QP 目录与 QM 文件说明

本文说明项目中 `qp` 目录的结构、各类文件的职责，以及 QM 模型生成后如何接入固件。当前上下板使用的是 QP/C 8.1.3 和 FreeRTOS 端口。

## `.qm` 和 `.qms` 的区别

### `.qm`：状态机模型源文件

`.qm` 是 QM 图形化建模工具保存的模型文件，包含状态、层级关系、转移、事件、动作代码、主动对象和代码生成配置等内容。对本项目而言，它是状态机逻辑的事实来源。

例如：

```text
hero_down/qp/qm/decision_ao.qm
```

QM 根据该模型生成：

```text
hero_down/qp/decision_ao.h
hero_down/qp/decision_ao.c
```

状态、转移、entry/exit 或信号定义发生变化时，应先修改 `.qm`，再从 QM 重新生成 `.c/.h`。不要只手工修改生成文件，否则下一次生成会覆盖修改，并造成模型与代码不一致。

### `.qms`：QM 编辑器会话文件

`.qms` 是 QM 的 session（会话）文件，不是状态机模型，也不是生成的 C 代码。它主要保存编辑器工作状态，例如：

- 已打开的图、代码文件和标签页；
- 窗口位置、大小及布局；
- 网格、备份等编辑器设置；
- 搜索选项、外部工具和会话变量。

项目中的 `hero_down/qp/qm/decision_ao.qms` 就包含这些 XML 格式的会话信息，并记录了 `decision_ao.h/.c` 的编辑窗口。

### 创建 `.qm` 会自动生成 `.qms` 吗

需要区分“创建模型”和“保存编辑器会话”：

- 在 QM 软件中新建或打开 `.qm`，随后保存会话或正常退出时，QM 通常会在模型旁创建或更新同名 `.qms`。
- 仅在文件系统中手工创建一个 `.qm` 文件，不会自动出现 `.qms`。
- 运行代码生成也不依赖 `.qms`，生成 `.c/.h` 的输入是 `.qm`。
- 删除 `.qms` 一般不会损坏模型或影响固件编译；QM 可以重新建立会话文件，但原有窗口布局等会话信息会丢失。

因此，`.qm` 必须重点维护；`.qms` 可以随 QM 会话自然生成，不需要手工编写。

## 目录总览

完整接入一个 `DecisionAO` 后，单板的 QP 目录结构如下：

```text
qp/
|-- qm/
|   |-- decision_ao.qm       # QM 模型源文件
|   `-- decision_ao.qms      # QM 编辑器会话文件
|-- qpc/
|   |-- include/             # QP/C 框架头文件
|   |-- ports/               # FreeRTOS/编译器适配层
|   `-- src/                 # QP/C 框架实现
|-- decision_ao.c            # QM 生成的主动对象实现
|-- decision_ao.h            # QM 生成的信号、类型和接口
|-- qp_config.h              # 本工程的 QP/C 编译期配置
|-- qpc_init.c               # QP 初始化及 AO 启动
`-- qpc_init.h               # 初始化接口和 QP 回调声明
```

`qm` 保存用户维护的模型和编辑器会话；`qpc` 保存第三方 QP/C 框架；`qp` 根目录保存本项目的配置、初始化代码和模型生成结果。

## 根目录文件职责

### `qp_config.h`

QP/C 的工程级编译配置文件。它控制框架所需类型宽度、活动对象数量、事件池数量、时间事件 tick rate 等编译期选项。该文件应按目标板资源和实际 AO 数量统一配置，不属于 QM 的生成结果。

### `qpc_init.h`

对外声明 `QpInit()`，并声明 QP/C 要求应用提供的回调：

- `QF_onStartup()`：框架启动回调；
- `QF_onCleanup()`：框架停止后的清理回调；
- `Q_onError()`：QP 断言失败处理入口。

业务任务只需要通过该头文件调用初始化接口，不应直接依赖 `qpc/src` 的内部实现。

### `qpc_init.c`

负责把通用 QP/C 框架接入当前单板，典型工作顺序为：

1. 调用 `QF_init()` 初始化框架；
2. 用 `QF_poolInit()` 注册动态事件内存池；
3. 用 `QActive_psInit()` 初始化发布/订阅表；
4. 调用生成的主动对象构造函数，例如 `DecisionAO_ctor()`；
5. 准备 AO 的事件队列和 FreeRTOS 栈；
6. 设置任务属性并调用 `QACTIVE_START()` 启动 AO。

本项目的 FreeRTOS 调度器在 `QpInit()` 执行时已经运行，因此这里不再调用 `QF_run()`。`Q_onError()` 当前通过关闭中断并停留在死循环中保留断言现场。

### `decision_ao.h`

由 QM 根据 `.qm` 生成，通常包含：

- 状态机使用的信号枚举；
- 主动对象类型和业务状态字段；
- AO 实例的公开只读入口，例如 `AO_DecisionAO`；
- 构造函数和模型需要公开的接口。

其他任务或中断在需要驱动主干状态迁移时，通过这里定义的信号构造/投递事件。

### `decision_ao.c`

由 QM 生成主动对象实现，包括状态处理函数、状态层级、转移表、entry/exit 动作、构造逻辑和 AO 实例。它在 QP/C 框架上运行，不应脱离 `.qm` 单独维护状态转移逻辑。

## `qpc/include` 头文件职责

| 文件 | 职责 |
|---|---|
| `qpc.h` | 应用侧统一入口；选择 FreeRTOS port，并组合 QP/C、功能安全和 QS 接口。 |
| `qp.h` | QP/C 核心公开 API 和类型，包括事件、状态机、主动对象、时间事件、发布订阅等。 |
| `qequeue.h` | 原生 QP 事件队列类型与操作。FreeRTOS port 会把 AO 队列映射到 RTOS 队列。 |
| `qmpool.h` | 固定块内存池 `QMPool`，供动态事件分配和回收使用。 |
| `qp_pkg.h` | QP/C 框架实现内部共享声明；应用业务代码不应直接包含。 |
| `qsafe.h` | 断言、契约检查、关键区及功能安全相关宏。 |
| `qs_dummy.h` | 未启用 `Q_SPY` 时使用的空 QS 跟踪接口，使跟踪调用不产生运行负担。 |
| `qstamp.h` | 声明 QP/C 构建日期和构建时间标识。 |

这些文件属于 QP/C 框架版本的一部分。除非是在有依据地升级或移植框架，否则不应按业务需求随意修改。

## `qpc/ports` 适配层职责

| 文件 | 职责 |
|---|---|
| `qp_port.h` | 将 `QActive` 的线程、队列、临界区、ISR 投递和调度操作映射到 FreeRTOS。 |
| `qf_port.c` | 实现 FreeRTOS 下 AO 任务入口、启动/停止、事件投递、ISR API 和端口相关事件池操作。 |
| `qs_port.h` | QS 软件跟踪在当前端口下的适配配置；未定义 `Q_SPY` 时实际使用 `qs_dummy.h`。 |
| `syscalls.c` | 为特定标准库/链接环境提供系统调用桩。是否加入 Keil 编译应由目标工具链决定，不是 QP 运行的必选文件。 |

端口层决定 QP/C 如何使用 FreeRTOS。任务优先级、事件队列容量、AO 栈空间以及 ISR 中使用 `FromISR` API 的规则都必须与该层匹配。

## `qpc/src` 框架实现职责

| 文件 | 职责 |
|---|---|
| `qep_hsm.c` | 函数式层次状态机 `QHsm` 的初始化、事件分派和状态转移。 |
| `qep_msm.c` | QM 生成代码使用的表驱动状态机 `QMsm` 实现。 |
| `qf_act.c` | QF 框架初始化、活动对象注册表及通用 AO 管理。 |
| `qf_defer.c` | 事件 defer/recall（延后处理和召回）机制。 |
| `qf_dyn.c` | 动态事件分配、引用计数和垃圾回收。 |
| `qf_mem.c` | 固定块内存池的初始化、申请、释放和使用率统计。 |
| `qf_ps.c` | 事件发布/订阅及订阅表管理。 |
| `qf_qact.c` | `QActive` 构造和状态机分派相关实现。 |
| `qf_qeq.c` | QP 原生事件队列实现。 |
| `qf_qmact.c` | 基于 `QMsm` 的主动对象构造与运行支持。 |
| `qf_time.c` | 时间事件的装载、解除、重装和 tick 处理。 |

`qpc/src` 是通用框架代码，不承载机器人业务决策。业务状态机应保留在 `.qm` 和生成的 `decision_ao.c/.h` 中。

## 运行时调用链

初始化链路为：

```text
InitTask
  -> QpInit()
      -> QF_init()
      -> 初始化事件池
      -> 初始化发布/订阅表
      -> DecisionAO_ctor()
      -> QACTIVE_START()
          -> 创建 DecisionAO 对应的 FreeRTOS 任务
```

运行期间，普通任务使用 `QACTIVE_POST()` 等接口向 AO 投递事件；中断上下文必须使用端口提供的 `FromISR` 接口。事件进入 AO 自己的队列后，由其 FreeRTOS 任务串行取出并分派给状态机，因此状态转移和相应 entry/exit 动作在 AO 上下文中执行。

动态事件来自 `QF_poolInit()` 注册的事件池，处理完成后由 QP/C 的引用计数和垃圾回收机制返还。发布/订阅功能则依赖 `QActive_psInit()` 创建的订阅表。

## 上下板当前状态

### `hero_down`

下板已经完整接入 `DecisionAO`：存在 `.qm/.qms`、生成的 `decision_ao.c/.h`，并且 `qpc_init.c` 已配置事件池、发布订阅表、队列、栈和 AO 启动流程。它是上板接入时的主要结构参考。

### `hero_up`

上板目前已经具备：

- QP/C 8.1.3 框架源码；
- FreeRTOS port；
- `qp_config.h`；
- `qpc_init.c/.h`；
- `InitTask -> QpInit() -> QF_init()` 的基础调用链；
- Keil 和 clangd 所需的基础 QP include/source 环境。

上板暂时没有 `qp/qm`、`decision_ao.qm/.qms` 和生成的 `decision_ao.c/.h`。因此当前 `QpInit()` 只初始化框架，尚未建立事件池、发布订阅表或启动 `DecisionAO`。这些内容要等上板 QM 模型由人工完成并生成代码后再接入。

## 上板模型生成后的接入步骤

1. 在 `hero_up/qp/qm/` 保存 `decision_ao.qm`；同目录的 `.qms` 交给 QM 会话自动维护。
2. 在 QM 中将生成目标设置为 `hero_up/qp/decision_ao.c` 和 `hero_up/qp/decision_ao.h`。
3. 核对生成文件中的 include、信号枚举、AO 构造函数和公开实例名称。
4. 参考下板扩展 `hero_up/qp/qpc_init.c`，加入事件池、发布订阅表、事件队列、任务栈、`DecisionAO_ctor()` 和 `QACTIVE_START()`。
5. 将生成的 `decision_ao.c/.h` 加入 Keil 工程分组和 include 路径。
6. 运行项目的 `add_missing.py`，更新上板 `compile_commands.json`。
7. 重启 clangd，使其重新读取编译数据库和新增头文件。
8. 编译上板工程，并逐项核对每个 SIG 的定义、投递上下文、接收状态、队列容量和状态动作。

## 维护注意事项

- `.qm` 是状态机逻辑的事实来源；生成代码必须与模型同步。
- `.qms` 只是编辑器会话数据，不等于 QM 模型，也不等于生成代码。
- 不要在未更新 `.qm` 的情况下长期手改 `decision_ao.c/.h` 中的状态转移逻辑。
- 不要把 ISR 中的投递写成普通任务上下文 API；必须使用 FreeRTOS port 对应的 ISR 接口。
- 修改 AO 优先级、栈、队列或事件池大小时，要结合实时性、最坏事件突发量和内存占用验证。
- `qpc` 中的文件带有 Quantum Leaps 的许可证头。复制、分发或修改时必须保留许可证声明，并确认项目使用方式符合对应许可。
