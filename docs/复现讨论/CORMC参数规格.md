# CORMC 参数规格

本文档是 CORMC 第一版复现的统一参数来源文档。它只管理参数名称、论文符号、数值或分布、单位、来源状态、使用模块和待审阅点，避免同一参数在公式映射、道路几何、车辆模型、状态接口、日志验证、代码结构和最小验证场景中重复定义。

本文档不解释公式推导，不设计 Python 数据结构，不决定模块接口，也不设定最小验证场景中每辆车的初始 `x / y / v / type / compliance`。车辆模型规格负责说明如何消费这些参数；最小验证场景规格可以把随机参数覆盖为显式值，但不能改变本文档记录的论文参数来源。

本文档的上游依据是：

```text
docs/复现讨论/CORMC论文公式与实现映射.md
docs/复现讨论/CORMC道路几何与区域规格.md
docs/复现讨论/CORMC复现讨论对齐记录.md
docs/papers/P002-2023-Hou-Cooperative-On-Ramp-Merging-Control.md
docs/papers/THE VALUES OF VARIABLES AND PARAMETERS.png
docs/papers/Fig. 10. Illustration of a simple merging network in simulation.png
```

## 1. 文档定位

第一版参数规格回答以下问题：

```text
1. 论文 Table I / 参数表图片中有哪些可直接采用的参数。
2. 道路几何中哪些参数已固定，哪些是由论文参数推导得到。
3. 哪些参数是第一版为了跑通主链路而采用的默认值。
4. 哪些参数进入代码或论文级实验前需要继续审阅。
5. 后续车辆模型、状态接口、日志验证、代码结构和最小验证场景应从哪里取参数。
```

本文档不负责：

```text
1. 不解释 Eq.57-Eq.60、IDM、CPID、CUC、CMC 等公式如何计算。
2. 不定义 dataclass、enum、config schema 或命令缓冲结构。
3. 不决定 smoke scenario 中车辆初始坐标、速度、类型或 compliance。
4. 不把 assignment invalid、越界保守策略等工程补丁写成模型参数。
```

## 2. 来源状态与表格规范

本文档使用四类来源状态：

```text
paper:
    论文正文、Fig.10、Table I 或参数表图片明确给出。

paper-derived:
    由论文给出的参数或几何量直接计算得到。

first-version-default:
    论文未明确给出，但第一版为了跑通主链路需要配置化默认。

to-review:
    进入代码、论文级实验或数值复现前需要回查 PDF、前作或后续讨论确认。
```

参数表统一使用以下列：

```text
参数名 / 实现建议名
论文符号
数值或分布
单位
来源状态
使用模块
备注 / 待审阅点
```

单位统一使用 `m`、`s`、`m/s`、`m/s^2`、`veh/h/lane`、`%` 等文本格式。无量纲参数单位写 `-`。

## 3. 全局、道路与仿真时间参数

这些参数为仿真时间推进、道路几何、轨迹图坐标和论文级观测区提供统一数值来源。

| 参数名 / 实现建议名 | 论文符号 | 数值或分布 | 单位 | 来源状态 | 使用模块 | 备注 / 待审阅点 |
| --- | --- | --- | --- | --- | --- | --- |
| 仿真时间步长 `dt` | `Delta` | `0.1` | s | paper | 时间步主循环、车辆模型 | Table I 给出 time interval |
| 仿真时长 `simulation_time` | `All_time` | `1200` | s | paper | 论文实验、日志输出 | 第一版 smoke scenario 可缩短，但不改变论文参数来源 |
| 随机重复次数 `num_random_runs` | - | `20` | 次 | paper | 论文级实验 | Section V-C：20 个随机种子取平均 |
| 主线起点 `mainline_start_global` | - | `0` | m | paper | 道路几何、边界生成 | 主线 starting boundary |
| 主线总长 `mainline_length` | - | `10000` | m | paper | 道路几何、驶离判定 | Section V-C：10 km |
| 主线终点 `mainline_end_global` | - | `10000` | m | paper-derived | 道路几何、驶离判定 | `mainline_start_global + mainline_length` |
| warm-up 长度 `warmup_length` | - | `4000` | m | paper | 道路几何、轨迹图、输出指标 | Fig.10 标注 |
| 绘图坐标偏移 `plot_x_offset` | - | `4000` | m | paper-derived | 轨迹图、Fig.13 对齐 | `x_plot = x_global - plot_x_offset` |
| merging zone 起点 `x0_m_global` | `x_0^m` | `6950` | m | paper | APS、CMC、道路几何 | 按道路几何规格解释为 merging zone 起点 |
| merging zone 长度 `L_merging` | - | `300` | m | paper | 道路几何、CMC | Section V-C / Fig.10 |
| on-ramp 下游边界 `x_ramp_end_global` | `x_ramp^end` | `7250` | m | paper-derived | CMC、边界防撞、日志验证 | `x0_m_global + L_merging` |
| merging zone 后主线长度 `post_merging_mainline_length` | - | `2750` | m | paper-derived | 道路几何、轨迹图 | Fig.10 标注，等于 `10000 - 7250` |
| fixed cooperative zone 长度 `L_coop_fixed` | - | `300` | m | paper | 道路几何、绘图标注 | Fig.10 图示；不用于 APS 候选集合 |
| communication range `L_cr` | `L^cr` | `300` | m | paper | APS、道路几何 | APS 候选窗口唯一使用该参数 |
| 论文级观测区起点 `obs_start_global` | - | `4000` | m | paper | 输出指标 | Section VI：4 km 到 10 km |
| 论文级观测区终点 `obs_end_global` | - | `10000` | m | paper | 输出指标 | Section VI |
| 观测区长度 `obs_segment_length` | - | `500` | m | paper | 输出指标 | 12 个 500 m 区间 |
| 观测区数量 `obs_segment_count` | - | `12` | 个 | paper-derived | 输出指标 | `(10000 - 4000) / 500` |

约束：`L_cr = 300 m` 是 APS candidate window 的参数。第一版实现不得使用 fixed cooperative zone 或 dynamic cooperative window 替代 `L_cr` 搜索窗口。

## 4. 车辆基础与交通组成参数

这些参数描述车辆基本属性、车辆类型组成和论文实验中的交通需求设置。

| 参数名 / 实现建议名 | 论文符号 | 数值或分布 | 单位 | 来源状态 | 使用模块 | 备注 / 待审阅点 |
| --- | --- | --- | --- | --- | --- | --- |
| 车辆长度 `vehicle_length` | `L` | `4` | m | paper | 纵向模型、APS、CMC、碰撞检测 | Table I 给出 vehicle length |
| 停止间距 `standstill_spacing` | `d_0` | `2` | m | paper | CAV、IDM、CMC 边界防撞 | Table I 给出 stopping spacing |
| 车辆类型集合 `vehicle_types` | - | `CAV`, `CHV` | - | paper | 车辆生成、状态接口 | 所有 HV 均视为 connected HV，即 CHV |
| CHV compliance 状态 `chv_compliance_state` | - | compliant / non-compliant | - | paper | CUC、车辆生成 | compliance rate 决定 CHV 是否接受 CUC 建议 |
| CHV compliance rate 实验值 `C_values` | `C` | `0, 25, 50, 75, 100` | % | paper | 论文实验矩阵 | Table II 使用；主实验常用 `75%` |
| CAV penetration 实验值 `P1_values` | `P_1` | `0, 20, 40, 60, 80, 100` | % | paper | 论文实验矩阵、车辆生成 | Section V-C |
| mainline demand ratio `eta_main` | `eta_main` | `70` | % | paper | 车辆生成、论文实验 | Section V-C：mainline demand 为 theoretical capacity 的 70% |
| on-ramp flow ratio 实验值 `eta_ramp_values` | `eta_ramp` | `10, 20, 30, 40` | % | paper | 车辆生成、论文实验 | Section V-C |
| compliance sensitivity 场景 `compliance_scenario` | - | `P1 = 60%, eta_ramp = 20%` | - | paper | 论文实验 | Table II 对应场景 |
| 子模型对比场景 `submodel_scenario` | - | `eta_ramp = 30%, C = 75%` | - | paper | 论文实验 | Fig.11 对应场景 |
| 轨迹示例场景 `trajectory_scenario` | - | `P1 = 80%, eta_ramp = 20%, C = 75%` | - | paper | 论文轨迹图 | Fig.13 对应 CORMC 场景 |

说明：第一版 smoke scenario 可以显式指定车辆类型和 compliance，绕开随机生成；这属于最小验证场景配置，不改变本文档的论文参数。

## 5. 纵向模型参数

### 5.1 CAV Cruising 与通用约束

| 参数名 / 实现建议名 | 论文符号 | 数值或分布 | 单位 | 来源状态 | 使用模块 | 备注 / 待审阅点 |
| --- | --- | --- | --- | --- | --- | --- |
| cruising 控制增益 `cav_cruise_gain` | `k_1` | `0.4` | s^-1 | paper | CAV cruising | Table I |
| CAV 惯性滞后 `cav_inertial_lag` | `tau_i` | `U(0.4, 0.7)` | s | paper | CAV CPID、车辆生成 | 每辆 CAV 独立抽样 |
| CAV 期望时距 `cav_desired_time_gap` | `h_CAV` | `1.2` | s | paper | CAV desired spacing、capacity | Table I |
| 最小控制量 `u_min` | `u_min` | `-6` | m/s^2 | paper | CAV CPID | Table I |
| 最大控制量 `u_max` | `u_max` | `4` | m/s^2 | paper | CAV CPID | Table I |
| 最小加速度 `a_min` | `a_min` | `-6` | m/s^2 | paper | CAV、碰撞避免 | Table I |
| 最大加速度 `a_max` | `a_max` | `4` | m/s^2 | paper | CAV | Table I |

### 5.2 CPID 默认增益

CORMC 论文本篇给出 CAV gap-regulating 的 CPID 公式，但 Table I / 参数表图片没有给出完整 PID 增益。第一版不让 CPID 成为复现障碍：参数可配置，默认采用 `CORMC复现讨论对齐记录.md` 中记录的前作 DCPID Group 2 参数。

| 参数名 / 实现建议名 | 论文符号 | 数值或分布 | 单位 | 来源状态 | 使用模块 | 备注 / 待审阅点 |
| --- | --- | --- | --- | --- | --- | --- |
| 外环比例增益 `cpid_Kpx` | `K_px` | `8` | - | first-version-default / to-review | CAV CPID | 来自复现讨论对齐记录引用前作 DCPID Group 2 |
| 外环积分增益 `cpid_Kix` | `K_ix` | `0` | - | first-version-default / to-review | CAV CPID | 同上 |
| 外环微分增益 `cpid_Kdx` | `K_dx` | `10` | - | first-version-default / to-review | CAV CPID | 同上 |
| 内环比例增益 `cpid_Kpv` | `K_pv` | `5` | - | first-version-default / to-review | CAV CPID | 同上 |
| 内环积分增益 `cpid_Kiv` | `K_iv` | `0` | - | first-version-default / to-review | CAV CPID | 同上 |
| 内环微分增益 `cpid_Kdv` | `K_dv` | `0` | - | first-version-default / to-review | CAV CPID | 同上 |

这些增益只作为第一版可运行默认值，不代表论文数值完全复现。后续实验阶段可以根据前作文献、补充材料或标定结果修正。

### 5.3 IDM / CHV 参数

| 参数名 / 实现建议名 | 论文符号 | 数值或分布 | 单位 | 来源状态 | 使用模块 | 备注 / 待审阅点 |
| --- | --- | --- | --- | --- | --- | --- |
| CHV 期望时距 `chv_desired_time_gap` | `h_CHV` | `2` | s | paper | IDM、capacity | Table I |
| CHV 期望速度 `chv_desired_speed` | `v_i^f` | `N(30, 1.5)` | m/s | paper | IDM、车辆生成 | Table I；按正态分布抽样 |
| IDM 最大加速度 `idm_max_acceleration` | `A_i` | `1.25` | m/s^2 | paper | IDM | Table I |
| IDM 舒适减速度 `idm_comfort_deceleration` | `b_i` | `2.09` | m/s^2 | paper | IDM | Table I |

IDM 同时使用车辆长度 `L = 4 m` 和停止间距 `d_0 = 2 m`，这两个参数已在车辆基础参数中统一定义。

## 6. APS / CUC / CMC 参数

### 6.1 APS 参数

| 参数名 / 实现建议名 | 论文符号 | 数值或分布 | 单位 | 来源状态 | 使用模块 | 备注 / 待审阅点 |
| --- | --- | --- | --- | --- | --- | --- |
| 协同合流预期最小可接受时距 `aps_min_merge_time_gap` | `g_min^CM` | `1.2` | s | paper | APS case 判断 | Table I / Algorithm 1 |
| APS 决策时间间隔 `aps_decision_interval` | `T_APS` | `5` | s | paper | APS assignment cache | Table I |
| 通信范围 `communication_range` | `L_cr` | `300` | m | paper | APS 候选集合 | Table I；唯一用于 APS candidate window |

硬约束：

```text
APS 候选集合唯一按 lane 2 中 [x_MV_global - L_cr, x_MV_global + L_cr] 计算。
不得使用 fixed_cooperative_zone 或 dynamic_coop_window_for(MV) 替代。
```

### 6.2 CUC 参数

| 参数名 / 实现建议名 | 论文符号 | 数值或分布 | 单位 | 来源状态 | 使用模块 | 备注 / 待审阅点 |
| --- | --- | --- | --- | --- | --- | --- |
| CUC 安全项权重 `cuc_alpha` | `alpha` | `-1` | - | paper | CUC utility | Table I |
| CUC 后车影响项权重 `cuc_beta` | `beta` | `1.5` | - | paper | CUC utility | Table I |
| CUC CV 加速度收益权重 `cuc_gamma` | `gamma` | `0.5` | - | paper | CUC utility | Table I |
| CUC 舒适性权重 `cuc_zeta` | `zeta` | `-0.5` | - | paper | CUC utility | Table I |
| TT 最小安全阈值 `tt_min` | `TT_min` | `1.5` | s | paper | CUC 目标车道安全检查 | 正文说明：TT below 1.5 s unsafe |

### 6.3 CMC 参数

| 参数名 / 实现建议名 | 论文符号 | 数值或分布 | 单位 | 来源状态 | 使用模块 | 备注 / 待审阅点 |
| --- | --- | --- | --- | --- | --- | --- |
| 合流可接受时距上界 `cmc_upper_merge_time_gap` | `h_upper^CM` | `1.2` | s | paper | CMC dynamic gap acceptance | Table I |
| 动态 gap 线性系数 `cmc_dynamic_gap_xi` | `xi` | `2/3` | - | paper | CMC dynamic gap acceptance | Table I |

CMC 还使用 `x0_m_global`、`x_ramp_end_global`、`vehicle_length`、`standstill_spacing`、`lane_width`、`lane_change_planned_acceleration` 等参数。几何边界在第 3 节统一定义，`L_w` 和 `a_p` 在第 7 节统一定义。

## 7. Lane-changing 与横向轨迹参数

| 参数名 / 实现建议名 | 论文符号 | 数值或分布 | 单位 | 来源状态 | 使用模块 | 备注 / 待审阅点 |
| --- | --- | --- | --- | --- | --- | --- |
| lane-changing 最小可接受时距 `lc_min_time_gap` | `h_min^LC` | `1.2` | s | paper | 普通 lane-changing 判断 | 第一版关闭普通主线主动换道，但保留参数来源 |
| 换道速度差阈值 `lc_speed_diff_threshold` | `Delta v_threshold` | `4` | m/s | paper | 普通 lane-changing 判断 | 第一版关闭普通主线主动换道 |
| 轨迹规划加速度 `lane_change_planned_acceleration` | `a_p` | `0.1` | m/s^2 | paper | 正弦轨迹、front/boundary collision avoidance | Table I |
| 完成换道后 centerline 跟踪距离 `lane_change_centerline_length` | `L_centerline` | `100` | m | paper | 正弦轨迹规划 | 正文说明设置为 100 m |
| 车道宽度 `lane_width` | `L_w` | `3.5` | m | first-version-default / to-review | 横向 centerline、CMC boundary avoidance | Table I 未给出；道路几何文档建议默认候选 |

`L_w = 3.5 m` 只是第一版默认候选，不是论文 Table I 明确参数。若后续 PDF、原图、前作或代码材料确认其他值，应在本文档中更新并同步道路几何和车辆模型规格。

## 8. Vehicle Generation 与论文实验矩阵参数

### 8.1 Vehicle Generation 参数

| 参数名 / 实现建议名 | 论文符号 | 数值或分布 | 单位 | 来源状态 | 使用模块 | 备注 / 待审阅点 |
| --- | --- | --- | --- | --- | --- | --- |
| shifted headway lane 1 `h_shifted_lane1` | `h_shifted` | `1.2` | s | paper | vehicle generation | Table I |
| shifted headway lane 2 `h_shifted_lane2` | `h_shifted` | `1.2` | s | paper | vehicle generation | Table I |
| shifted headway on-ramp `h_shifted_ramp` | `h_shifted` | `3.5` | s | paper | vehicle generation | Table I |
| equilibrium speed `equilibrium_speed` | `v_e` | `30` | m/s | paper | CAV cruising、capacity、vehicle generation | Table I |
| lane 1 maximum speed `vmax_lane1` | `v_max` | `33` | m/s | paper | vehicle generation、speed constraints | Table I |
| lane 2 maximum speed `vmax_lane2` | `v_max` | `33` | m/s | paper | vehicle generation、speed constraints | Table I |
| on-ramp maximum speed `vmax_ramp` | `v_max` | `30` | m/s | paper | vehicle generation、speed constraints | Table I |
| lane 1 initial speed `initial_speed_lane1` | `v` | `30` | m/s | paper | vehicle generation | Table I |
| lane 2 initial speed `initial_speed_lane2` | `v` | `30` | m/s | paper | vehicle generation | Table I |
| on-ramp initial speed `initial_speed_ramp` | `v` | `16` | m/s | paper | vehicle generation | Table I |

Vehicle generation 还使用 Eq.57-Eq.60 中的 mixed capacity、on-ramp flow 和 shifted negative exponential arrival headway。本文档只收录所需参数；公式本身由车辆生成或车辆模型相关规格展开。

车辆生成规格需要明确 `eta_main` 如何换算并分配到 `Q_lane1` / `Q_lane2`。本文档不决定主线总需求是先按 lane 平分，还是每条 lane 各自设置 flow。

车辆生成规格还需要检查给定流量下 shifted negative exponential arrival headway 是否可行，尤其是 Eq.59-Eq.60 中 `1 / lambda - h_shifted` 应保持为正。

### 8.2 论文实验矩阵参数

| 参数名 / 实现建议名 | 论文符号 | 数值或分布 | 单位 | 来源状态 | 使用模块 | 备注 / 待审阅点 |
| --- | --- | --- | --- | --- | --- | --- |
| CAV penetration rate grid `P1_grid` | `P_1` | `0, 20, 40, 60, 80, 100` | % | paper | 论文实验矩阵 | Section V-C |
| on-ramp flow ratio grid `eta_ramp_grid` | `eta_ramp` | `10, 20, 30, 40` | % | paper | 论文实验矩阵 | Section V-C |
| mainline demand ratio `eta_main` | `eta_main` | `70` | % | paper | 论文实验矩阵 | Section V-C |
| default compliance for penetration experiments `default_C_for_penetration` | `C` | `75` | % | paper | 论文实验矩阵 | Section VI-C |
| compliance rate grid `C_grid` | `C` | `0, 25, 50, 75, 100` | % | paper | compliance 实验 | Section VI-B |
| random run count `num_random_runs` | - | `20` | 次 | paper | 论文实验矩阵 | Section V-C |
| full experiment scenario count `num_scenarios` | - | `24` | 个 | paper-derived | 论文实验矩阵 | `6` 个 P1 水平 x `4` 个 eta_ramp 水平 |

第一版代码结构可以预留论文实验矩阵配置，但第一版 smoke scenario 不需要完整跑 24 个场景或 20 个随机种子。

所有百分数网格参数只用于展示、实验命名和人工阅读。进入 Eq.57-Eq.60 或其他公式计算前，必须统一转换为 `0-1` 比例：

```text
60% -> 0.6
20% -> 0.2
75% -> 0.75
```

该规则适用于 `P1_grid`、`eta_main`、`eta_ramp_grid`、`C_grid` 以及后续新增的百分数参数。

## 9. 第一版默认与待审阅清单

以下参数不是 Table I / 论文正文明确给出的完整可直接复现参数。实现时必须保留来源标记，不能写成论文原参数。

| 参数名 | 当前值 | 来源状态 | 为什么需要审阅 |
| --- | ---: | --- | --- |
| `lane_width` / `L_w` | `3.5 m` | first-version-default / to-review | 论文公式使用 `L_w`，但 Table I、参数表图片和 Fig.10 未给出典型值 |
| `cpid_Kpx` | `8` | first-version-default / to-review | CORMC 论文本篇未完整给出 CPID 增益 |
| `cpid_Kix` | `0` | first-version-default / to-review | 同上 |
| `cpid_Kdx` | `10` | first-version-default / to-review | 同上 |
| `cpid_Kpv` | `5` | first-version-default / to-review | 同上 |
| `cpid_Kiv` | `0` | first-version-default / to-review | 同上 |
| `cpid_Kdv` | `0` | first-version-default / to-review | 同上 |

以下内容不进入参数规格，除非后续车辆模型或日志验证规格明确需要阈值：

```text
1. APS 候选不足时如何兜底。
2. assignment invalid 后如何减速或等待。
3. 多 MV 共享 CV 仲裁的优先级权重。
4. MV 越过 x_ramp_end_global 后是否终止仿真。
5. smoke scenario 中每辆车的初始 x / y / v / type / compliance。
```

这些是工程策略、状态机或场景配置问题，不是本文档当前阶段的参数表职责。

## 10. 后续 Spec 使用关系

后续文档应按以下方式引用本文档：

| 后续文档 | 如何使用本文档 |
| --- | --- |
| `CORMC车辆模型规格.md` | 引用 CAV、IDM、lane-changing、CMC、CUC 参数，说明如何消费这些数值 |
| `CORMC状态与模块接口规格.md` | 引用参数名作为配置输入，但不重复定义数值 |
| `CORMC输出指标与日志验证规格.md` | 引用仿真时长、观测区、随机种子轮数、轨迹图坐标口径 |
| `CORMC代码数据结构设计.md` | 将本文档中的参数组织为 config 结构，但不重新决定参数来源 |
| `CORMC最小验证场景规格.md` | 可以覆盖随机参数为显式场景值，但必须说明覆盖目的和不改变论文参数来源 |

实施约束：

```text
1. 后续文档若需要参数数值，应引用本文档，不重新抄 Table I。
2. 若发现本文档参数有误，应先修订本文档，再同步修订消费它的规格。
3. 若后续新增 first-version-default 参数，必须补充到第 9 节待审阅清单。
4. 参数规格不应吸收工程补丁策略；工程补丁应留在状态接口、车辆模型或日志验证规格中定义。
```

## 11. 验收检查

本文档应满足以下检查：

```text
1. 后续写车辆模型规格时，不需要再从论文 Table I 重新抄参数。
2. 后续实现代码时，能看出哪些参数可直接采用，哪些只能作为第一版默认。
3. 文档明确 L_w 不是论文 Table I 明确参数。
4. 文档明确 CPID 增益不是 CORMC 论文本篇 Table I 明确参数。
5. 文档明确 L_cr = 300 m 是 APS 候选窗口参数。
6. 文档没有把 cooperative zone 写成 APS 候选窗口参数。
7. 文档没有提前决定 smoke scenario 中每辆车的初始状态。
8. 文档没有把 assignment invalid、越界保守处理等工程补丁策略写成参数。
```
