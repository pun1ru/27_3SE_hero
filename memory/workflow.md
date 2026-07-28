# 开发工作流

## 任务开始

1. 完整读取 `AGENTS.md`、`MEMORY.md` 及其引用的全部文件。
2. 运行 `git status --short`，识别并保护用户已有修改。
3. 确认任务属于 `hero_down`、`hero_up`，还是需要双板同步。
4. 搜索定义、声明、调用点、配置宏和工程文件引用，明确改动范围。
5. 逐行查看目标代码周围风格后再编辑。

## 一般修改流程

1. 明确数据所有者、调用 Task 和控制链阶段。
2. 在原有模块边界内完成最小必要修改。
3. 公共状态维持 `const` 只读指针封装。
4. 共用算法或驱动应对比上下板实现，确认同步范围和板级差异。
5. 板间协议改动必须同时修改并核对两端。
6. 检查公开声明、公共 include、初始化入口和 Keil 工程源文件列表。
7. 运行与风险匹配的验证，并说明无法执行的硬件工具链验证。

## 新增模块

1. 在 `PrivateApplications/ModuleName` 或 `PrivateDrivers/ModuleName` 创建目录。
2. 创建公开头文件和实现文件，沿用相邻模块风格。
3. 高频模块头文件加入对应板的 `Tasks/Inc/general_task_include.h`。
4. 共享状态通过 `extern const ModuleName* _moduleName` 只读暴露。
5. 在 `initial_task.c` 或实际拥有者 Task 中初始化。
6. 检查另一板是否需要同步，并保留硬件和功能差异。
7. 将新增 `.c` 文件加入 Keil 工程并补全 clangd 编译数据库。

头文件骨架：

```c
#ifndef _MODULE_NAME_H_
#define _MODULE_NAME_H_

#include "general_task_include.h"

typedef struct
{
    /* 状态字段 */
} ModuleName;

/**
 * @brief   初始化
 * @param   module 模块结构体
 * @retval  void
 */
void ModuleNameInitialize(ModuleName* module);

#endif
```

## 编译数据库维护

新增 `PrivateApplications` 或 `PrivateDrivers` 下的 `.c` 文件后，在对应目录运行：

```text
hero_down/MDK-ARM/out/Hero_Reeeee64/Hero_Reeeee64/add_missing.py
hero_up/MDK-ARM/out/Hero_Reeeee64/Hero_Reeeee64/add_missing.py
```

解释器命令：

```text
"C:/Program Files/Python314/python.exe" add_missing.py
```

随后清理具体工程输出目录的 `.cache/clangd`，并在 VS Code 执行 `clangd: Restart language server`。禁止对仓库根目录执行递归删除。

## 验证流程

- C 代码：检查声明与定义、条件编译、单位、边界值、空指针和非有限值。
- 控制链：检查 Decision、Estimate、Control 阶段和任务通知关系。
- 双板通信：核对两端结构、ID、长度、缩放、字节序和周期。
- 新模块：检查 Keil 工程引用、`compile_commands.json` 和 clangd 解析。
- 可用 Keil 时通过对应 `.uvprojx` 执行完整构建。
- 无法运行 Keil 或硬件验证时必须明确说明，不能把静态检查表述为完整编译通过。

## 文档维护

- 架构或接口变化更新 `project.md`。
- 强制规范变化更新 `rules.md`。
- 可重复流程变化更新 `workflow.md`。
- 未完成事项和验证状态更新 `todo.md`。
