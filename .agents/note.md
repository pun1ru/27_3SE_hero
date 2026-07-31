## SKILL
# 放在哪里
```
项目根目录/

└── .agents/

    └── skills/

        └── motor-log-analyzer/

        ├── SKILL.md                 # 必需，固定文件名

        ├── agents/

        │   └── openai.yaml          # 可选，界面名称、图标、依赖等

        ├── references/              # 可选，供 Codex 按需阅读的资料

        │   ├── csv-format.md

        │   └── error-codes.md

        ├── scripts/                 # 可选，可执行脚本

        │   ├── analyze_log.py

        │   └── validate_input.ps1

        └── assets/                  # 可选，模板、图片、字体等输出素材

            └── report-template.md

```
# 格式
```
---
name: motor-log-analyzer
description: Analyze motor-control CSV logs and identify current, speed, and temperature anomalies. Use when the user asks to inspect, summarize, validate, or troubleshoot motor log files.
---


# Motor Log Analyzer

## Workflow

1. Confirm the input file is CSV.
2. Read [CSV format](references/csv-format.md) when column meanings are unclear.
3. Run `python scripts/analyze_log.py <input.csv>`.
4. Report the detected anomalies and the evidence for each conclusion.

Do not invent missing sensor values.
```
# 最稳妥的 Codex 格式要求：
* 文件名必须是大写的 SKILL.md。
* YAML 必须被两行 --- 包围。
* name 和 description 必须存在。
* name 建议只用小写字母、数字和连字符，例如 motor-log-analyzer。
* 目录名应与 name 相同。
* name 建议不超过 64 个字符。
* description 必须同时写清楚“做什么”和“什么时候触发”。
* 详细流程写在正文，不要只写在 description。
* 正文推荐使用命令式，例如“Read…、Run…、Check…”。
* 中文正文没有问题，文件建议统一使用 UTF-8。
* 一个常见错误是把“什么时候使用这个 Skill”只写在正文。Codex 在决定是否启用 Skill 时，首先只能看到 name 和 description，正文此时还没有加载。

# markdown怎么放
* 详细资料放在 references/，然后从 SKILL.md 直接引用
* references/ 中可以放多个 .md 文件。
* 明确说明什么情况下读取哪个文件。
* 不要期望 Codex 自动读取所有 Markdown。
* 尽量让 SKILL.md 直接链接引用文件，避免引用文件再引用很多层。
* 超过约 100 行的参考文档，建议在顶部加目录。
* SKILL.md 最好保持精简，建议低于 500 行。
* 不建议额外添加 README.md、CHANGELOG.md、安装指南等无关文件。

# Script 文件怎么放
* 重复、容易出错、需要确定性结果的操作，放进 scripts/：
* 在 SKILL.md 中明确写运行方式、输入和输出
* Python、PowerShell、JavaScript、Bash 都可以。
* Windows 环境优先考虑 Python 或 PowerShell。
* 不要依赖“当前工作目录刚好是 Skill 目录”，应指示 Codex相对于 SKILL.md 所在目录解析路径。
* 第三方依赖要写清楚，例如需要 pandas。
* 脚本必须实际运行测试。
* 如果普通指令就能可靠完成，不必为了形式加入脚本。
脚本产生的临时文件不要写进 Skill 目录，除非它就是预期输出。

