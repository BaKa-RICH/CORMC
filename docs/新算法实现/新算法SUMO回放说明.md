# 新算法 SUMO 回放说明

## 阶段 8 范围

阶段 8 的 SUMO replay 是离线事实轨迹播放。算法先在 Python stage2 链路中生成并验收五层 summary、`trajectory.csv` 和 `gap_rows.json`，随后统一由 `ObservationDataset` 消费这些标准数据包，生成 plot、artifact report、manifest 和 SUMO-GUI replay。

SUMO 端只负责按 `trajectory.csv` 对应的事实轨迹播放画面，不参与决策、不修正轨迹、不承担闭环控制。GUI 中车辆位置来自 `stage8_sumo_replay/replay_trajectory.jsonl`，该 JSONL 由 stage2 `trajectory.csv` 一对一转换而来。

## 统一入口

唯一推荐的构建和播放入口是：

```powershell
$env:PYTHONIOENCODING='utf-8'; python -m cormc.observation.cli --source-dir "<stage7 scenario dir>" --build
```

```powershell
$env:PYTHONIOENCODING='utf-8'; python -m cormc.observation.cli --source-dir "<stage7 scenario dir>" --play
```

```powershell
$env:PYTHONIOENCODING='utf-8'; python -m cormc.observation.cli --source-dir "<stage7 scenario dir>" --smoke
```

`--play` 和 `--smoke` 都会先刷新 artifact。`--play` 打开 SUMO-GUI 并保留窗口用于人工观察；`--smoke` 使用快速播放参数，适合本机确认播放器能读 JSONL 并执行 TraCI `moveToXY`。

构建后也可以使用场景旁边生成的脚本：

```powershell
& "<stage7 scenario dir>\stage8_sumo_replay\play_gui_replay.ps1"
```

```powershell
& "<stage7 scenario dir>\stage8_sumo_replay\play_gui_replay.ps1" -Smoke
```

## 输入数据包

`--source-dir` 必须指向一个已经存在的阶段 7 场景 artifact 目录，目录中必须有：

- `stage2_summary.json`
- `trajectory.csv`
- `gap_rows.json`

`stage2_summary.json` 顶层结构固定为：

- `scenario_summary`
- `round_summaries`
- `mv_summaries`
- `cross_mv_summary`
- `artifact_paths`

`gap_rows` 是正式中间产物，来自 `mv_summaries.*.gap_rows`，并导出为 `gap_rows.json`。阶段 8 不重新运行 stage2 runner，也不从旧 replay 数据推断新算法目标。

## 输出产物

每个场景旁边固定生成：

- `stage8_plots/trajectory.csv`
- `stage8_plots/process_x_t_local.png`
- `stage8_plots/process_v_t.png`
- `stage8_plots/process_y_t.png`
- `stage8_plots/lifecycle_timeline.png`
- `stage8_sumo_replay/replay_trajectory.jsonl`
- `stage8_sumo_replay/sumo/p17.sumocfg`
- `stage8_sumo_replay/play_gui_replay.ps1`
- `stage8_artifact_manifest.json`
- `stage8_report.md`

`stage8_artifact_manifest.json` 使用 `observation_artifact.v1` schema，并记录原始输入文件、plot 路径、SUMO replay 路径、人工播放命令、smoke 命令和 replay fidelity 结果。

## 分层边界

- `cormc.observation.stage2_artifacts` 只负责把阶段 7 标准 artifact 转成 `ObservationDataset`。
- `cormc.observation.plotting` 只消费 `ObservationDataset` 生成 x-t、v-t、y-t 和 lifecycle timeline。
- `cormc.observation.sumo_replay` 只消费 `ObservationDataset` 生成 replay JSONL、P17 SUMO 网络文件和 GUI 播放脚本。
- `cormc.sumo.trajectory_gui_replay` 只播放 JSONL 轨迹，不参与算法判断。
- `cormc.sumo.env / network / mapping / executor / loop / authority` 是底层 SUMO 基础设施，继续服务非 replay public 基础测试和后续闭环能力。

这种分层让 replay 可视化不反向影响算法，也避免为了播放而接回旧链路。

## 不进入的旧链路

旧 BASIC、rolling BASIC、random 6450、旧 ramp merge replay、P17 canned replay、P17.1/MVS replay public artifact 入口已退役，不再维护，也不作为阶段 8 验收数据来源。旧数据未来如需观察，应该先通过一个薄 adapter 转成标准 `ObservationDataset` 所需的五层 summary、`trajectory.csv` 和 `gap_rows.json`。

以下旧入口不再作为可运行命令或 public import 使用：

- `cormc.sumo.basic_replay_artifacts`
- `cormc.sumo.rolling_basic_replay_artifacts`
- `cormc.sumo.random_6450_replay_artifacts`
- `cormc.sumo.ramp_merge_replay_artifacts`
- `cormc.sumo.mvs_replay_artifacts`
- `cormc.sumo.mvs_replay_specs`

## 人工验收重点

2-MV S07 rolling 场景使用阶段 7 根目录数据：

```powershell
$env:PYTHONIOENCODING='utf-8'; python -m cormc.observation.cli --source-dir "D:\PycharmProjects\CORMC\artifacts\scene_interface_phase7_validation" --play
```

人工观察时应看到 `S07_MV` 和 `S07_MV_REAR` 都从 on-ramp `y=-3.5` 进入主线 lane_2 `y=0.0`。播放不应通过 SUMO 原生决策改变轨迹；所有车辆位置都应来自 `replay_trajectory.jsonl`。


## ????? Stage2 ??

????? One-Step Stage2 ?????? `run_onestep_stage2_random_analysis(...)` ?????? 7/8 ????????? `RM-ONESTEP-RANDOM-S07-LANE2-RAMP-100S`??? seed ? `645001`??? profile ? `medium`??? horizon ? `100.0s`??????? `stage2_summary.json`?`stage2_report.md`?`trajectory.csv`?`gap_rows.json` ??? 8 plot/SUMO replay ?????

?????????? horizon ????? `open_mv_ids_at_horizon`?? completed MV ????? Stage2 ?????????? open MV ??? summary ????? final status?? `random_6450_runner.py` ????????????? Stage2 ??? SUMO replay ??????

