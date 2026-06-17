# CORMC

一个面向匝道合流研究的 Python 仿真与 rolling 合流算法工作台，支持统一场景接口、批量执行、结果分析与 SUMO GUI 回放。

## 这是什么

CORMC 把一个完整的多车合流实验链路收在同一个仓库里：

- 上游是场景定义与统一场景接口。
- 中间是 one-step / rolling 合流算法执行。
- 下游是轨迹导出、结果图、诊断报告、SUMO replay 产物。

如果你关心的是 21 个多 MV 场景批量跑通，这个仓库已经有现成入口；如果你只想看 rolling 算法本体，也可以只聚焦 `cormc/onestep/rolling` 和 `cormc/onestep/kernel`。

## 快速开始

下面这套步骤面向“第一次在新机器上把项目跑起来”。

### 1. 准备 Python 环境

项目要求：

- Python 3.12+
- 推荐使用 `uv`

安装依赖：

```powershell
uv sync --dev
```

如果你不用 `uv`，至少需要安装：

- `numpy>=1.26`
- `pyyaml>=6.0`
- `pytest>=8.0`（跑测试时需要）

### 2. 准备 SUMO

完整链路中的 replay / GUI 回放需要安装 SUMO，并满足：

- 能使用 `sumo`、`sumo-gui`、`netconvert`
- Python 能导入 `traci` 和 `sumolib`
- `SUMO_HOME` 已正确配置，或 SUMO 的 Python tools 已加入 Python 搜索路径

说明：

- rolling 算法核心本身不依赖 SUMO GUI。
- 但如果你要生成 replay 文件、做 GUI smoke/play，或者使用 `cormc.sumo` 相关模块，就需要 SUMO 环境。

### 3. 跑一个最小可用示例

下面的命令会跑通一个多 MV 场景，并自动生成 stage2 分析结果和 stage8 observation / replay 产物：

```powershell
uv run python scripts/run_multimv_rolling.py `
  --scenario-id RM-M2-S01 `
  --run-id demo_rm_m2_s01 `
  --output artifacts/demo_multimv
```

运行完成后，你会得到两层产物：

1. 批次级汇总，位于 `artifacts/demo_multimv/`
2. 场景级详细产物，位于 `artifacts/demo_multimv/RM-M2-S01/demo_rm_m2_s01/`

## 使用示例

### 示例 1：跑一个多 MV 场景全流程

```powershell
uv run python scripts/run_multimv_rolling.py `
  --scenario-id RM-M2-S01 `
  --run-id demo_rm_m2_s01 `
  --output artifacts/demo_multimv
```

这条命令会经历以下阶段：

1. 从统一场景接口读取 `RM-M2-S01`
2. 编译成静态仿真输入
3. 执行 rolling 历史仿真
4. 导出 stage2 摘要、轨迹、gap 记录与图像
5. 基于 stage2 输出继续构建 stage8 observation plots 与 SUMO replay 目录

会产生这些关键文件：

```text
artifacts/demo_multimv/
├─ multimv_run_manifest.json
├─ multimv_summary.csv
├─ multimv_report.md
└─ RM-M2-S01/
   └─ demo_rm_m2_s01/
      ├─ stage2_summary.json
      ├─ stage2_report.md
      ├─ trajectory.csv
      ├─ gap_rows.json
      ├─ process_x_t_local.png
      ├─ process_v_t.png
      ├─ process_y_t.png
      ├─ lifecycle_timeline.png
      ├─ stage8_artifact_manifest.json
      ├─ stage8_report.md
      ├─ stage8_plots/
      └─ stage8_sumo_replay/
```

其中：

- `multimv_run_manifest.json`：整个批次的总清单
- `multimv_summary.csv`：适合快速筛选 completed / incomplete / exception
- `stage2_summary.json`：单场景 rolling 执行摘要
- `trajectory.csv`：轨迹明细
- `gap_rows.json`：每轮 gap 评估明细
- `stage8_plots/`：统一 observation 图片
- `stage8_sumo_replay/`：SUMO replay 所需文件和回放脚本

### 示例 2：只构建 observation 产物

如果某个场景目录里已经有 `stage2_summary.json`、`trajectory.csv`、`gap_rows.json`，可以单独补建 observation 产物：

```powershell
uv run python -m cormc.observation.cli `
  --source-dir "artifacts/demo_multimv/RM-M2-S01/demo_rm_m2_s01" `
  --build
```

这会在源目录下生成：

- `stage8_plots/`
- `stage8_sumo_replay/`
- `stage8_artifact_manifest.json`
- `stage8_report.md`

### 示例 3：做一次 GUI smoke replay

```powershell
$env:PYTHONIOENCODING='utf-8'
uv run python -m cormc.observation.cli `
  --source-dir "artifacts/demo_multimv/RM-M2-S01/demo_rm_m2_s01" `
  --smoke
```

如果你已经构建好了 replay 目录，也可以直接执行生成出来的脚本：

```powershell
& "artifacts/demo_multimv/RM-M2-S01/demo_rm_m2_s01/stage8_sumo_replay/play_gui_replay.ps1"
```

## 项目架构

项目可以粗分为 5 层：

1. `cormc.scenes`
   - 统一场景入口、场景规格、场景编译
2. `cormc.simulation_core`
   - 仿真底座，负责状态推进、历史记录、提交输出
3. `cormc.onestep.kernel` + `cormc.onestep.rolling`
   - 合流算法核心
4. `cormc.observation` + `cormc.sumo`
   - 结果整理、绘图、SUMO replay 构建与 GUI 回放
5. `scripts` + `tests`
   - 批量入口、回归测试、边界验证

从代码阅读顺序看，建议这样进入：

1. [`scripts/run_multimv_rolling.py`](scripts/run_multimv_rolling.py)
2. [`cormc/onestep/rolling/stage2_multimv_runner.py`](cormc/onestep/rolling/stage2_multimv_runner.py)
3. [`cormc/onestep/rolling/stage2_runner.py`](cormc/onestep/rolling/stage2_runner.py)
4. [`cormc/onestep/rolling/__init__.py`](cormc/onestep/rolling/__init__.py)
5. [`docs/CORMC_ARCHITECTURE.md`](docs/CORMC_ARCHITECTURE.md)

## 环境要求

这里写的是“别人参考也成立”的通用要求，不包含任何本机私有路径。

### 必需环境

- Python 3.12 或更高版本
- `numpy>=1.26`
- `pyyaml>=6.0`

### 开发与测试建议

- `pytest>=8.0`
- 推荐使用 `uv` 管理环境和锁文件

### 如果要跑完整链路

除了 Python 依赖，还需要：

- 安装 SUMO
- `sumo` / `sumo-gui` / `netconvert` 可用
- Python 可导入 `traci`、`sumolib`

### 平台说明

- rolling 核心模块主要是纯 Python
- 现成的 GUI replay 脚本以 PowerShell 为主，完整工作流更偏向 Windows + SUMO 的日常使用方式
- 如果在其他平台运行，核心算法模块通常可以复用，但 replay 命令和环境变量写法需要按平台调整

## 如果你只想用 rolling 算法

最短路径不是从 `scripts/` 或 `SUMO` 开始，而是：

1. 先看 [`cormc/onestep/rolling/__init__.py`](cormc/onestep/rolling/__init__.py)
2. 再看 [`cormc/onestep/rolling/stage2_runner.py`](cormc/onestep/rolling/stage2_runner.py)
3. 然后看：
   - [`cormc/onestep/rolling/engine.py`](cormc/onestep/rolling/engine.py)
   - [`cormc/onestep/rolling/planner.py`](cormc/onestep/rolling/planner.py)
   - [`cormc/onestep/rolling/motion.py`](cormc/onestep/rolling/motion.py)
   - [`cormc/onestep/rolling/safety.py`](cormc/onestep/rolling/safety.py)
   - [`cormc/onestep/rolling/gaps.py`](cormc/onestep/rolling/gaps.py)
   - [`cormc/onestep/rolling/state.py`](cormc/onestep/rolling/state.py)
4. 最后再下钻 [`cormc/onestep/kernel`](cormc/onestep/kernel)

详细说明见 [`docs/CORMC_ARCHITECTURE.md`](docs/CORMC_ARCHITECTURE.md) 中的“只使用 Rolling 算法时的最短路径”。

## 参考文档

- [`docs/CORMC_ARCHITECTURE.md`](docs/CORMC_ARCHITECTURE.md)
- [`docs/paper_reproduction_environment_setup_zh.md`](docs/paper_reproduction_environment_setup_zh.md)
- [`docs/one_step_algorithm/当前项目实现导读.md`](docs/one_step_algorithm/当前项目实现导读.md)
