# 强制规则

## 修改边界

- 修改前运行 `git status --short`；已有修改属于用户，不得擅自覆盖、回退或清理。
- 禁止擅自使用 `git reset --hard`、`git clean` 等破坏性命令。
- `git checkout --` 前必须说明会覆盖哪些文件并取得用户明确同意。
- 修改紧贴请求范围，不重构无关模块，不刷新无关生成文件。
- 代码和构建配置是事实来源；与记忆冲突时先核实代码，再更新文档。

## 架构与同步

- 遵循 `Task -> Application -> Driver -> HAL` 依赖方向。
- 修改共用算法或驱动时，检查 `hero_down` 与 `hero_up` 是否需要同步，并保留板级差异。
- 修改板间通信时同步检查两端、CAN ID、长度、缩放、字节序和频率。
- CAN ID 不得与现有电机 ID（如 `0x01-0x08`、`0x141`）冲突。
- 修改任务优先级、通知链或共享数据时评估实时性和并发一致性。
- 共享状态使用“模块内可写实例 + 外部 `const` 指针”的所有权模式。

## 编码规范

- 公开函数使用带模块前缀的 CamelCase；静态函数使用小写蛇形。
- 变量和字段使用 snake_case；物理量必须带明确单位后缀。
- 类型使用 CamelCase 且不加 `_t`；宏和枚举值使用 UPPER_SNAKE_CASE。
- 使用 4 空格、Allman 大括号和 `if(condition)` 风格。
- API 使用 Doxygen `@brief`、`@param`、`@retval`、`@note` 注释。
- 单位后缀包括 `_deg`/`_d`、`_rad`、`_dps`、`_radps`、`_mps`、`_rps`、`_rpm`、`_nm`。

## 文件与工具

- 新增代码前逐行检查相邻代码的缩进、注释、大括号和命名风格。
- 含中文的 UTF-8 源文件禁止使用 PowerShell 5.1 默认编码读写。
- 根 `.clangd` 禁止添加 `CompilationDatabase` 和 `CompileFlags.Add`。
- 不修改或提交无关的 Keil 临时文件、clangd 缓存和构建产物。

规则优先级：`AGENTS.md` > `rules.md` > `workflow.md` > `project.md` > `apply.md` > `todo.md`。
