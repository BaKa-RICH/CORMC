# CORMC 论文公式与实现映射

本文档是第一版 CORMC 复现的流程驱动公式映射文档。它不写成“论文公式大全”，也不替代参数规格、车辆模型规格、状态接口规格或代码数据结构设计。

本文档的主要输入是：

```text
docs/复现讨论/CORMC时间步执行顺序梳理.md
```

辅助输入是：

```text
docs/复现讨论/CORMC复现spec体系梳理.md
docs/复现讨论/CORMC复现讨论对齐记录.md
docs/papers/P002-2023-Hou-Cooperative-On-Ramp-Merging-Control.md
docs/papers/Cooperative_On-Ramp_Merging_Control_Model_for_Mixed_Traffic_on_Multi-Lane_Freeways.pdf
```

本文档的使用方式是：沿一次 `S(t) -> S(t+dt)` 时间步推演，识别流程中需要定义、计算、判断或约束的量，再映射到论文公式、Algorithm 1、Fig. 9 或正文说明。

统一映射口径：

```text
流程位置 -> 需要定义/计算/判断的量 -> 论文依据 -> 第一版处理状态 -> 后续展开 spec
```

## 1. 映射状态分类

本文档使用四类状态标注论文内容和第一版实现之间的关系。

```text
论文原公式：
    论文明确给出，第一版应尽量按原公式实现。

第一版简化：
    论文有完整机制，但第一版有意简化，例如不做 MPC tracking。

第一版关闭：
    论文有机制，但第一版暂不做，例如普通主线主动换道、CMC platoon。

工程补丁：
    论文未完整定义，但第一版实现必须补齐，例如 first_APS(MV)、assignment invalid、多 MV 共享 CV 仲裁。
```

工程补丁不能写成论文原公式。第一版简化和第一版关闭也不能被描述成论文缺失，而应明确为复现边界。

## 2. 按时间步流程推演的公式需求

### Step 1：边界车辆生成

| 流程位置 | 需要定义/计算/判断的量 | 论文依据 | 第一版处理状态 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| 边界车辆生成 | initialization / boundary generation 的区别 | Section V-A Vehicle Generation Model | 论文原公式/正文机制 | 道路几何与区域规格、参数规格 |
| 边界车辆生成 | 等待车辆相对前车的实际 headway `HW` 是否不小于 assigned arrival headway `HA` | Section V-A：若 `HW >= HA` 则生成车辆 | 论文原机制 | 状态与模块接口规格、代码数据结构设计 |
| 边界车辆生成 | mixed traffic 理论容量 `C_mix(P1)` | Eq.57 | 论文原公式 | 参数规格、输出指标与日志验证规格 |
| 边界车辆生成 | on-ramp flow `Q_ramp` | Eq.58 | 论文原公式 | 参数规格 |
| 边界车辆生成 | shifted negative exponential arrival headway | Eq.59-Eq.60 | 论文原公式；最小 smoke 场景可用确定性输入简化 | 参数规格、最小验证场景规格 |
| 边界车辆生成 | 车辆类型、CHV desired speed、CHV compliance、CAV inertial lag 等随机属性 | Section V-A 正文、Table I | 论文原机制；随机实验阶段实现，smoke 场景可显式指定 | 参数规格、代码数据结构设计、最小验证场景规格 |

说明：边界车辆生成发生在冻结 `S(t)` 之前。公式映射只记录论文依据；入口队列、临时车辆和可加载配置形态交给后续 spec。

### Step 3：车辆关系刷新

| 流程位置 | 需要定义/计算/判断的量 | 论文依据 | 第一版处理状态 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| 车辆关系刷新 | TLV / TFV / LV / FV 的语义 | CUC section 中 Fig. 4 后正文；lane-changing section Fig. 6 附近正文 | 论文原语义 | 状态与模块接口规格 |
| 车辆关系刷新 | CUC choice 1 时 CV、TLV、TFV 构成 lane-changing demand system | CUC choice 1 正文 | 论文原语义 | 车辆模型规格、状态与模块接口规格 |
| 车辆关系刷新 | CUC choice 2 时 LV、CV、FV 构成 no lane-changing demand system | CUC choice 2 正文 | 论文原语义 | 车辆模型规格、状态与模块接口规格 |
| 车辆关系刷新 | 换道中 SV/CV 与 TLV、TFV、FV 的纵向关系 | Lane-changing decision 正文：SV 与 TLV，TFV 将 SV 视为 leader，FV 仍将 SV 视为 leader | 论文原语义 | 状态与模块接口规格、代码数据结构设计 |
| 车辆关系刷新 | 不按 physical y 连续切换 leader/follower | 由论文 lane-changing subsystem 语义和第一版同步提交原则得到 | 第一版实现约束 | 状态与模块接口规格 |

说明：车辆关系刷新主要是语义映射，不是公式计算表。具体 relations 表达方式放到状态与模块接口规格和代码数据结构设计。

### Step 4：处理 on-ramp MV

| 流程位置 | 需要定义/计算/判断的量 | 论文依据 | 第一版处理状态 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| MV 不在 merging zone | 执行 APS，而不是 CMC | Fig. 9 simulation process；Algorithm 1 APS | 论文原调度语义 | 状态与模块接口规格 |
| MV 已在 merging zone | 执行 CMC | Fig. 9 simulation process；Section IV-B CMC | 论文原调度语义 | 车辆模型规格、状态与模块接口规格 |
| APS 周期判断 | 每个 MV 按 `T_APS` 更新 pair of CVs 和 case | APS 正文、Algorithm 1 | 论文原机制；`first_APS(MV)` 为工程补丁 | 状态与模块接口规格、代码数据结构设计 |
| APS 候选车辆集合 | lane 2 中 `Lcr` 范围内的 `Gap_car_MV(t)` | Eq.1-Eq.2 | 论文原公式 | 道路几何与区域规格、状态与模块接口规格 |
| APS 到达预测 | `T*_MV(t)`、候选车辆预测位置、anticipatory spacing | Eq.3-Eq.6 | 论文原公式 | 车辆模型规格、最小验证场景规格 |
| APS 选择 CLV / CFV | `D*_j > 0` 且 `D*_{j+1} < 0` | APS 正文、Algorithm 1 | 论文原机制 | 状态与模块接口规格 |
| APS case 判断 | CLV/CFV 预测间隙约束与最小间距 | Eq.7-Eq.9 | 论文原公式 | 最小验证场景规格 |
| APS case 2 / 4 CFV desired spacing | CFV 使用 Eq.10 为 MV 创建后向 gap | Eq.10；case 2/4 正文 | 论文原公式；只明确用于 CFV | 车辆模型规格 |
| APS case 3 CLV 处理 | MV 可将 CLV 作为 leading vehicle 调整间距，不对 CLV 套 Eq.10 | APS case 3 后正文 | 论文原语义 | 车辆模型规格、状态与模块接口规格 |
| CMC 动态可接受时间间隙 | `h~_MV^CM(t)` | Eq.52 | 论文原公式 | 车辆模型规格、参数规格 |
| CMC 合流 gap 判断 | MV 与 assigned CLV / CFV 的实际 gap 是否满足 | Eq.53 | 论文原公式；执行前做 assignment valid 工程验证 | 车辆模型规格、输出指标与日志验证规格 |
| CMC boundary speed cap | on-ramp downstream boundary 速度约束 | Eq.54-Eq.56 | 论文原公式；职责为 CMC 计算 speed cap | 车辆模型规格 |

APS 边界情况不属于论文主公式，但代码必须处理。至少包括：lane 2 候选不足、没有满足 `D*_j > 0` 且 `D*_{j+1} < 0` 的插入对、全部 `D*` 为正、全部 `D*` 为负、`v_MV` 接近 0 导致 `T*_MV` 除零、MV 已越过 `x0^m` 但状态尚未进入 CMC。这些应标为工程实现边界，不写成论文原公式。

### Step 5：汇总 APS 产生的 CV 协同请求

| 流程位置 | 需要定义/计算/判断的量 | 论文依据 | 第一版处理状态 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| 多 MV 请求同一 CV | 冲突消解优先级 | 论文未完整定义 | 工程补丁 | 状态与模块接口规格、输出指标与日志验证规格、代码数据结构设计 |
| 未获得 CV 的 MV | 标记等待或冲突，后续通过 APS 更新处理 | 论文未完整定义 | 工程补丁 | 状态与模块接口规格 |

说明：多 MV 共享 CV 仲裁不是论文原生多 MV 分配算法，不能写成论文公式。默认优先级沿用时间步总纲：已在 merging zone 的 MV 优先，其次 `T*_MV` 更小，其次距离 `x0^m` 更近。

### Step 6：处理 mainline 车辆 / CUC

| 流程位置 | 需要定义/计算/判断的量 | 论文依据 | 第一版处理状态 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| CUC 作用对象 | APS 指定且 `col = 1` 的 CLV / CFV | APS Eq.9、case 正文、CUC 开头正文 | 论文原语义 | 状态与模块接口规格 |
| CUC choice | choice 1 换到 lane 1；choice 2 留在 lane 2 | CUC choice 1/2 正文 | 论文原语义 | 车辆模型规格 |
| CHV compliance | compliant CHV 接受 CUC；non-compliant CHV 忽略建议 | CUC section 中 CHV compliance 正文 | 论文原语义 | 参数规格、车辆模型规格 |
| CUC utility | choice 1 / choice 2 效用 | Eq.11-Eq.12 | 论文原公式 | 车辆模型规格、参数规格 |
| CUC safety measure | `c_f^l(t)` | Eq.13 | 论文原公式 | 车辆模型规格 |
| 目标车道 TT 安全约束 | `TT_CV^TLV`、`TT_TFV^CV` 与 `TT_min` | Eq.14-Eq.15 | 论文原公式；第一版用于 CUC lane 1 安全检查 | 车辆模型规格 |
| CUC final choice | `M_CV` | Eq.16 | 论文原公式 | 车辆模型规格 |
| active lane change 不重新 CUC | `lane_change_state == executing` 时不重选 maneuver | 论文 Fig. 9 每步 CUC + 第一版状态机约束 | 第一版实现约束 | 状态与模块接口规格、代码数据结构设计 |

注意：论文 Markdown 在 Eq.14 前文字出现“choice 2”表述，但 Eq.14 的 TLV/TFV 语义对应目标车道安全检查。第一版按时间步总纲：CUC 选择 lane 1 时检查目标车道安全。

Eq.16 的第一版解释必须写硬：`if U1 > U2 and Eq.14 satisfied -> choice 1; otherwise -> choice 2`。也就是说，即使 `U1 > U2`，只要目标车道 TT 安全约束不满足，CV 就不能启动 lane 2 -> lane 1 换道，应回退为 choice 2。

### Step 7：计算纵向动力学

| 流程位置 | 需要定义/计算/判断的量 | 论文依据 | 第一版处理状态 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| CAV actual spacing | `d_i(t)` | Eq.17 | 论文原公式 | 车辆模型规格 |
| CAV desired spacing | `S_i(t)` | Eq.18 | 论文原公式 | 车辆模型规格 |
| CAV collision avoidance spacing | `C_i(t)` | Eq.19 | 论文原公式 | 车辆模型规格 |
| CAV cruising | 无 leader 或 actual time gap 足够大时趋向 equilibrium speed | CAV cruising 正文、Eq.20 | 论文原公式 | 车辆模型规格 |
| CAV gap-regulating / CPID | spacing error、speed error、outer/inner PID、inertial lag | Eq.21-Eq.27 | 论文原公式；CPID 参数可能需后续审阅 | 车辆模型规格、参数规格 |
| CHV / IDM | stochastic IDM acceleration 与 desired dynamic spacing | Eq.28-Eq.29 | 论文原公式 | 车辆模型规格、参数规格 |
| CFV 留 lane 2 协同期望间距 | case 2 / 4 中 CFV 使用 Eq.10 | Eq.10，APS case 2/4 正文 | 论文原公式；不套给 CLV | 车辆模型规格 |
| MV waiting / executing 纵向速度约束 | 使用 CMC 产生的 boundary speed cap | Eq.56；时间步总纲职责拆分 | 论文原公式 + 第一版职责约束 | 车辆模型规格 |

当前文档只确认“case 2 / 4 中 CFV 使用 Eq.10”的论文语义。若 CFV 是 CAV、compliant CHV 或 non-compliant CHV，Eq.10 如何进入纵向模型，必须由 `CORMC车辆模型规格.md` 明确；本文档不直接决定它是改 IDM 的期望间距、作为 virtual MV / leader 目标，还是只对可控车辆生效。

### Step 8：计算横向运动与安全修正

| 流程位置 | 需要定义/计算/判断的量 | 论文依据 | 第一版处理状态 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| 普通主线 lane-changing decision | 主线主动换道的速度和安全约束 | Eq.30-Eq.32 | 第一版关闭 | 车辆模型规格 |
| CUC lane 2 -> lane 1 正弦换道轨迹 | improved sine trajectory | Eq.33-Eq.36 | 论文原公式；第一版直接用参考轨迹，不做 MPC tracking | 车辆模型规格 |
| trajectory derivative / curvature / yaw / steering | 轨迹跟踪相关量 | Eq.37-Eq.41 | 第一版简化；不做 MPC tracking 时不作为核心实现 | 车辆模型规格 |
| front-collision-avoidance | 中点防撞时间、距离和约束 | Eq.42-Eq.46 | 论文原公式；第一版保留速度约束/回退语义 | 车辆模型规格 |
| MPC lateral tracking | 轨迹跟踪控制 | Eq.47-Eq.51 | 第一版简化：不做严格 MPC tracking | 车辆模型规格 |
| CMC on-ramp -> lane 2 合流轨迹 | 合流采用 lane-changing model | Section IV-B merging behavior 正文，引用 Eq.33-Eq.36 | 论文原语义；第一版直接按正弦参考轨迹 | 车辆模型规格 |
| boundary-collision-avoidance | 下游边界防撞与速度上限 | Eq.54-Eq.56 | 论文原公式；第一版由 CMC 产出 speed cap，纵向阶段施加 | 车辆模型规格 |

后续车辆模型规格必须固化 front-collision-avoidance、boundary speed cap 和正弦轨迹 planning speed 的合成口径。若正弦轨迹 planning speed、front-collision-avoidance 回退速度、boundary speed cap 同时存在，第一版应采用更保守速度，并记录触发原因。本文档不展开具体速度裁剪公式或 command 字段。

### Step 9-10：同步提交与信息集成

| 流程位置 | 需要定义/计算/判断的量 | 论文依据 | 第一版处理状态 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| 状态反馈到下一时间步 | 下层更新 vehicle status，上层下一步再决策 | Section II framework 正文、Fig. 2、Fig. 9 | 论文原语义 | 状态与模块接口规格 |
| 每车每步只提交一次 | 只读 `S(t)`、只写 command / next-state、最后提交 `S(t+dt)` | 论文未以此实现术语定义 | 第一版实现约束 | 状态与模块接口规格、代码数据结构设计 |
| 轨迹、事件、碰撞、越界记录 | 仿真平台输出和论文实验结果 | Section V/VI、Fig. 11-Fig. 14 | 第一版输出验证机制 | 输出指标与日志验证规格 |

## 3. 按模块归档的公式索引

| 模块 | 公式/依据 | 用途 | 第一版处理状态 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| APS | Eq.1-Eq.2 | lane 2 通信范围候选车辆集合 `Gap_car_MV(t)` | 论文原公式 | 状态与模块接口规格 |
| APS | Eq.3-Eq.6 | `T*_MV`、候选车辆预测位置、anticipatory spacing | 论文原公式 | 车辆模型规格、最小验证场景规格 |
| APS | Eq.7-Eq.9 | CLV/CFV 最小间距约束、`col`、case 判断 | 论文原公式 | 最小验证场景规格 |
| APS | Eq.10 | case 2 / 4 中 CFV desired car-following spacing | 论文原公式；仅明确用于 CFV | 车辆模型规格 |
| APS | Algorithm 1、`T_APS` 正文 | APS 周期更新 pair of CVs 和 case | 论文原机制；`first_APS(MV)` 为工程补丁 | 状态与模块接口规格 |
| CUC | CUC choice 1/2 正文 | CV 换到 lane 1 或留在 lane 2 | 论文原语义 | 车辆模型规格 |
| CUC | Eq.11-Eq.12 | 两个 choice 的 utility | 论文原公式 | 车辆模型规格 |
| CUC | Eq.13 | safety measure `c_f^l(t)` | 论文原公式 | 车辆模型规格 |
| CUC | Eq.14-Eq.15 | 目标车道 TT 安全约束 | 论文原公式 | 车辆模型规格 |
| CUC | Eq.16 | cooperative maneuver choice `M_CV` | 论文原公式 | 车辆模型规格 |
| CUC | CHV compliance 正文 | compliant / non-compliant CHV 行为差异 | 论文原语义 | 参数规格、车辆模型规格 |
| CAV longitudinal | Eq.17-Eq.19 | actual spacing、desired spacing、collision avoidance spacing | 论文原公式 | 车辆模型规格 |
| CAV longitudinal | Eq.20 | cruising acceleration | 论文原公式 | 车辆模型规格 |
| CAV longitudinal | Eq.21-Eq.27 | CPID gap-regulating、控制量/加速度/速度约束 | 论文原公式；参数需后续审阅 | 车辆模型规格、参数规格 |
| CHV / IDM | Eq.28-Eq.29 | stochastic IDM acceleration 和 desired dynamic spacing | 论文原公式 | 车辆模型规格 |
| lane-changing | Eq.30-Eq.32 | 普通主线主动换道判断 | 第一版关闭 | 车辆模型规格 |
| lane-changing | Eq.33-Eq.36 | 正弦参考轨迹、轨迹长度、规划速度、横向距离 | 论文原公式；第一版直接用参考轨迹 | 车辆模型规格 |
| lane-changing | Eq.37-Eq.41 | 轨迹导数、曲率、yaw、steering | 第一版简化 | 车辆模型规格 |
| lane-changing | Eq.42-Eq.46 | front-collision-avoidance | 论文原公式；第一版保留速度约束/回退语义 | 车辆模型规格 |
| lane-changing | Eq.47-Eq.51 | MPC tracking | 第一版简化 | 车辆模型规格 |
| CMC | Eq.52 | dynamic time gap acceptance | 论文原公式 | 车辆模型规格 |
| CMC | Eq.53 | merging gap constraints | 论文原公式；执行前做 assignment valid 验证 | 车辆模型规格、输出指标与日志验证规格 |
| CMC | Eq.54-Eq.56 | boundary-collision-avoidance 与 speed cap | 论文原公式 | 车辆模型规格 |
| CMC | platoon merging rules 正文 | 多 MV platoon 首尾规则 | 第一版关闭 | 后续增强再展开 |
| vehicle generation | Eq.57-Eq.58 | mixed capacity 与 on-ramp flow | 论文原公式 | 参数规格 |
| vehicle generation | Eq.59-Eq.60 | shifted negative exponential arrival headway | 论文原公式；smoke 场景可显式指定 | 参数规格 |
| vehicle generation | Section V-A 正文 | 初始化、边界生成、随机属性 | 论文原机制 | 参数规格、代码数据结构设计 |
| scenario / output metrics | Table I | 参数典型值 | 论文原参数来源 | 参数规格 |
| scenario / output metrics | Section V-C / VI | 论文实验矩阵、平均速度、流量、延误等 | 后续论文级实验使用 | 输出指标与日志验证规格 |

## 4. 工程补丁与论文公式边界

### 4.1 `first_APS(MV)`

`first_APS(MV)` 是第一版工程调度规则，不是论文公式。

论文依据是 Algorithm 1 中为每个 MV 定义 APS initial decision time，并按 `T_APS` 周期更新 APS assignment。第一版加入 `first_APS(MV)` 的目的，是避免 MV 首次进入 APS 适用阶段时没有 assignment cache 可沿用。

### 4.2 `assignment invalid`

CMC 执行 Eq.53 时，以 APS assignment 中的 CLV / CFV 作为目标协同对象。执行 Eq.53 前验证 assigned CLV / CFV 是否仍有效，是第一版工程安全验证。

该处理不等价于：

```text
每步实时重查 lane 2 actual leader/follower 并替代 APS assignment
```

若 assigned CV 已换道离开 lane 2、已驶离或不再形成安全边界，则本步 MV 暂不开始合流，等待下一次 APS 或执行保守安全处理。

### 4.3 多 MV 共享 CV 仲裁

多个 MV 同时选中同一 CV 时的仲裁是第一版工程安全补丁，不是论文原生多 MV 协同分配机制。

默认优先级沿用时间步总纲：

```text
已在 merging zone 的 MV > T*_MV 更小的 MV > 距离 x0^m 更近的 MV
```

未获得该 CV 的 MV 标记为等待或冲突状态，后续通过 APS 更新处理。

### 4.4 同步提交 `S(t+dt)`

每车每步只读 `S(t)`、只写 command / next-state、最终同步提交 `S(t+dt)` 是第一版仿真实现约束。

论文框架说明下层每步更新 vehicle status，并反馈给下一时间步上层决策；但论文未以 `S(t)` / command buffer / next-state buffer 的工程术语定义同步提交机制。

## 5. 后续 spec 使用关系

| 内容 | 后续展开文档 |
| --- | --- |
| 道路几何、lane centerline、cooperative zone、merging zone、`x0^m`、`x_ramp_end` | `CORMC道路几何与区域规格.md` |
| 参数值、典型值、随机分布、参数来源和待审阅参数 | `CORMC参数规格.md` |
| CAV / IDM / 正弦轨迹 / front-collision-avoidance / boundary-collision-avoidance 的落地细节 | `CORMC车辆模型规格.md` |
| `S(t)`、assignment cache、command buffer、next-state buffer、relations、state machine | `CORMC状态与模块接口规格.md` |
| 日志点、轨迹图、sanity check、平均速度、流量、延误等指标 | `CORMC输出指标与日志验证规格.md` |
| dataclass、enum、config、event record、trajectory record | `CORMC代码数据结构设计.md` |
| Scenario A/B/C 的车辆初始状态数值反推和预期事件 | `CORMC最小验证场景规格.md` |

## 6. 后续车辆模型规格必须固化的实现口径

以下内容已经超出本文档的公式索引职责，但会直接影响代码正确性。后续 `CORMC车辆模型规格.md` 必须明确这些口径。

1. **CUC Eq.16 的回退规则**
   - 固化为：`if U1 > U2 and Eq.14 satisfied -> choice 1; otherwise -> choice 2`。
   - `U1 > U2` 但目标车道 TT 安全约束不满足时，不能启动换道。

2. **Eq.53 actual spacing 口径**
   - 必须明确 `d_CFV^MV`、`d_MV^CLV` 的 x 方向、车辆前后顺序、扣减哪辆车长度、使用车辆中心点还是前后端。
   - 当前映射文档不直接拍板该实现公式，避免在道路坐标和 OCR 复核前过早定死。

3. **APS 边界情况**
   - 必须定义候选不足、无有效插入对、全部 `D*` 为正、全部 `D*` 为负、`v_MV` 接近 0、MV 已越过 `x0^m` 但状态尚未进入 CMC 时的处理。
   - 这些属于工程实现边界，不属于论文原公式。

4. **Eq.10 对不同车辆类型的消费方式**
   - 必须明确 CFV 为 CAV、compliant CHV、non-compliant CHV 时，Eq.10 如何影响纵向模型。
   - 当前文档只锁定 Eq.10 的论文语义：case 2 / 4 中用于 CFV，不用于 case 3 的 CLV。

5. **front-collision-avoidance 与 boundary speed cap 合成**
   - 必须明确正弦轨迹 planning speed、front-collision-avoidance 回退速度、boundary speed cap 同时存在时的优先级。
   - 第一版建议采用更保守速度，并记录触发原因；具体落地由车辆模型规格和日志验证规格展开。

6. **OCR 公式校验**
   - Eq.14、Eq.26-Eq.27、Eq.47-Eq.51 等 Markdown OCR/排版不稳定的公式，进入车辆模型规格和代码实现前必须回查 PDF 或截图。
   - 不允许只依赖 Markdown OCR 作为最终实现依据。

## 7. 需审阅问题

以下问题不阻止本文档作为公式映射索引使用，但需要后续 spec 或实现计划中确认。

1. **论文 Markdown 中部分公式 OCR 排版不稳定**
   - Eq.14 前文字、Eq.26-Eq.27、Eq.47-Eq.51 等位置存在排版粘连。
   - 后续写车辆模型规格和代码实现前，应回查 PDF 或截图确认公式。

2. **CPID 参数来源**
   - 论文给出 CPID 结构，但外环/内环 PID 参数可能来自前作或未完整列入 Table I。
   - 参数规格中应单独列为待审阅或第一版默认。

3. **MPC tracking 的处理边界**
   - 论文给出 MPC tracking 相关公式。
   - 第一版已确定不做严格 MPC tracking，只按正弦参考轨迹更新横向位置；车辆模型规格需写清简化边界。

4. **普通主线主动换道关闭后的 Eq.30-Eq.32 位置**
   - Eq.30-Eq.32 属于论文 lane-changing decision，但第一版关闭普通主线主动换道。
   - CUC 触发的协同换道仍需要目标车道安全和正弦轨迹，不应误用普通主动换道触发条件。

5. **Eq.53 中 actual spacing 的符号和车辆顺序**
   - 后续最小验证场景反推时，需要明确 `d_CFV^MV`、`d_MV^CLV` 的实现方向、车辆长度扣减口径，以及中心点/前后端坐标口径。
   - 该问题应在车辆模型规格和状态接口规格中细化。

6. **APS 边界情况**
   - 论文主公式默认能找到可用 CLV / CFV 插入对。
   - 代码实现必须处理候选不足、无有效插入对、全部 `D*` 为正、全部 `D*` 为负、`v_MV` 接近 0、MV 已越过 `x0^m` 但状态尚未进入 CMC 等情况。

7. **Eq.10 与车辆类型**
   - Eq.10 用于 case 2 / 4 中 CFV 的期望跟驰间距。
   - 若 CFV 是 CAV、compliant CHV、non-compliant CHV，Eq.10 如何被其纵向模型消费，需要车辆模型规格明确。
