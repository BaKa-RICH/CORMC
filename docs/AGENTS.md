# Agent 快速对齐

这份文档只在用户明确要求时给 agent 阅读。它不放在根目录，避免未来内容没及时更新时被自动读取并误导执行。

# 项目主线

当前正式主线是：

```text
统一场景 -> One-Step 连续滚动 -> 五层 Stage2 summary -> observation -> plot -> SUMO replay
```

默认从 `cormc.scenes` 进入场景，从 `cormc.onestep.rolling` 运行正式连续滚动，从 `cormc.observation` 生成数据、图和 replay bundle。

# 先读这些文件

- `docs/one_step_algorithm/当前项目实现导读.md`
- `docs/one_step_algorithm/公式整理与分块.md`
- `scripts/phase9_golden.py`
- `tests/test_phase9_public_boundary.py`

# 正式入口

- `cormc.scenes`：统一场景入口，负责固定场景、随机流场景、场景 ID 和配置加载。
- `cormc.onestep.rolling`：One-Step 连续滚动正式主线。
- `cormc.observation`：Stage2 输出读取、标准数据集、plot、artifact bundle 和 replay fidelity validation。
- `cormc.sumo`：中性 SUMO 基础设施、网络、坐标映射、轨迹权威和 replay。

# 内部地基

- `cormc.onestep.kernel`：One-Step 单帧算法核心。
- `cormc.onestep.lab`：算法实验室，不是正式产品入口。
- `cormc.scenario_schema`：场景 schema、加载、匹配和报告。
- `cormc.simulation_core`：车辆状态、时间推进、事件、轨迹记录等底层地基。
- `cormc.traffic_flow`：可复用交通流和边界流生成。

# 不要做

- 不要恢复旧 public import wrapper。
- 不要把 `cormc.legacy` 当正式入口。
- 不要把 `temp/` 或可再生 artifact 当长期资料。
- 不要迁移到 WSL；默认使用 Windows 原生 SUMO。
- 不要直接倾倒大段中文文档、大 JSONL 或 artifact 内容。

# 验收命令

在 Windows 原生 SUMO 环境下，用非登录 PowerShell 和 UTF-8：

```powershell
$env:PYTHONIOENCODING='utf-8'
python scripts/phase9_golden.py --compare tests/golden/phase9_expected_fingerprints.json --output artifacts/phase9_regression/check
```

本机环境事实：

- `SUMO_HOME=D:\software\Eclipse\Sumo\`
- `sumo.exe` / `netconvert.exe` 是 SUMO 1.22.0
- 当前 Python 可 import `$SUMO_HOME/tools` 下的 `traci` 和 `sumolib`

# 中文路径和文档读取

读取中文文档时，PowerShell 输出乱码、问号、截断或超时通常是编码/缓冲问题，不是文件缺失。建议：

- 命令使用 `login:false`。
- 运行 Python/pytest 前设置 `$env:PYTHONIOENCODING='utf-8'`。
- 中文路径不要直接写进 PowerShell 源码；用 Python 枚举目录，再用 Unicode escape 文件名匹配。
- 大文档只小块读取，不要一次性倾倒。
