# Codex 命令卡顿处理说明

本文档用于提醒 Codex 在本仓库执行 PowerShell 命令时，遇到命令长时间无输出、超时或明显变慢时如何处理。

## 典型现象

- `Get-Content -Raw`、`Get-ChildItem -Recurse`、`git status --untracked-files=all` 等命令长时间停在 Running 状态。
- 命令本身很简单，但 PowerShell 启动、编码设置或递归扫描导致响应慢。
- 上一次命令超时后，后续状态检查也变慢，可能是因为文件系统、Git 或终端会话还在处理未完成任务。

## 优先处理方式

1. 先确认命令通道是否恢复：

```powershell
Write-Output 'ok'
```

2. 避免继续运行大范围递归命令。不要优先使用：

```powershell
Get-ChildItem -Recurse
git status --untracked-files=all
Get-Content -Raw <large-file>
```

3. 改用更窄的命令：

```powershell
Test-Path -LiteralPath '<path>'
rg -n '<pattern>' '<specific-file-or-dir>'
Get-Content -LiteralPath '<file>' -Encoding UTF8 -TotalCount 120
Select-String -LiteralPath '<file>' -Pattern '<pattern>' -Context 2,4
```

4. 如果只是需要修改文件，优先使用 `apply_patch`，不要为了创建目录或写文件先跑额外的 shell 命令。

5. 对中文文件读取，只在确实需要完整内容时使用编码包装；一般优先读取片段或用 `rg` 定位：

```powershell
$OutputEncoding=[System.Text.UTF8Encoding]::new($false)
[Console]::OutputEncoding=[System.Text.UTF8Encoding]::new($false)
rg -n '<pattern>' '<file>'
```

6. 如果命令已经超时，不要假设它成功。先用 `Test-Path` 或针对性 `rg` 检查结果，再继续。

## 本仓库建议

- 查文档内容：优先 `rg -n` 或 `Select-String`，少用整文件 `Get-Content -Raw`。
- 查文件是否存在：用 `Test-Path` 或 `rg --files | rg '<name>'`。
- 查 Git 状态：先用普通 `git status --short`；只有确实需要时才加 `--untracked-files=all`。
- 新增代码或文档：直接用 `apply_patch` 创建文件，减少 PowerShell 文件写入和目录创建命令。
- 遇到慢命令后，先降级为“小范围、短输出、明确路径”的命令，避免连续递归扫描。

## 不应做的事

- 不要因为命令卡住而扩大任务范围。
- 不要用递归扫描替代明确路径检查。
- 不要在同一个问题上连续运行多个大输出命令。
- 不要把超时命令的结果当作已成功执行。
