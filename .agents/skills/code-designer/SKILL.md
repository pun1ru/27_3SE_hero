---
name: code-designer
description: Safely modify and validate embedded C/C++ code in the dual-board firmware project. Use for changes involving hero_up, hero_down, Keil projects, FreeRTOS tasks, CAN communication, drivers, control modules, or generated QP state machines. Do not use for unrelated repositories or general documentation edits.
---

# Code Designer

## 工作流

1. 将本文件所在目录记为 `<skill-dir>`，将当前项目根目录记为 `<project-root>`。
2. 完整读取项目 `AGENTS.md`、根 `MEMORY.md`，并按其中顺序读取全部记忆文件。
3. 修改源码或工程配置前，读取[嵌入式代码修改规则](references/editing-rules.md)。涉及通信、任务、共用模块或新增文件时，再读取[验证清单](references/validation.md)。
4. 运行：

   ```powershell
   powershell -ExecutionPolicy Bypass -File "<skill-dir>/scripts/preflight.ps1" -ProjectRoot "<project-root>"
   ```

   该脚本只检查仓库、必需记忆文件和现有改动，不写文件。
5. 搜索定义、声明、调用点、配置和工程引用；查看相邻代码后做最小必要修改。保护用户已有改动，实际代码和构建配置优先于记忆文档。
6. 对重复且完全确定的文本替换，可使用 `scripts/apply_replacements.py` 提速。先准备位于 Skill 目录之外的 JSON manifest，执行 `--check` 查看 unified diff；确认后再执行 `--apply`。该脚本只接受精确旧文本和预期匹配次数，不能替代控制逻辑分析。

   ```powershell
   python "<skill-dir>/scripts/apply_replacements.py" --project-root "<project-root>" --manifest "<manifest.json>" --check
   python "<skill-dir>/scripts/apply_replacements.py" --project-root "<project-root>" --manifest "<manifest.json>" --apply
   ```

   manifest 格式：

   ```json
   {"files":[{"path":"hero_down/path/file.c","replacements":[{"old":"old text","new":"new text","count":1}]}]}
   ```

   不要用它批量改控制算法、协议帧、生成代码或 `.qm` 模型，除非已完成对应的语义核对和同步。
7. 按风险运行针对性测试，并将本次修改的相对路径明确传入验证脚本：

   ```powershell
   & "<skill-dir>/scripts/validate_changes.ps1" `
       -ProjectRoot "<project-root>" `
       -Paths @("<changed-path-1>", "<changed-path-2>")
   ```

   该脚本只检查传入路径，输出范围内变更、双板同步提示和新增 C 文件提示，并在对应路径的 `git diff --check` 失败时返回非零状态。
8. 仅在稳定事实、规则、流程或待办状态变化时更新对应记忆文件。最终说明修改内容、验证结果和未完成的硬件或工具链验证。

不要臆造硬件状态、构建结果或缺失数据，不要用静态检查代替完整编译结论。
