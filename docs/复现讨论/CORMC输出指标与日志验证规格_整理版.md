# CORMC 输出指标与日志验证规格（整理版）

本文档是第一版 CORMC 复现的输出指标与日志验证规格。它承接时间步总纲、公式映射、道路几何、参数规格、车辆模型规格和状态与模块接口规格，定义第一版仿真应记录哪些轨迹、事件、中间结果、状态转移、异常和验收检查。

本文档的目标不是复刻论文全部统计实验，而是让第一版 Python 微观仿真可调试、可验收、可定位错误。第一版优先服务 smoke scenario、关键链路检查和轨迹图 PNG；论文级平均速度、流量、延误、吞吐等宏观指标只预留入口，后续论文级实验再展开。

本文档不设计 Python dataclass，不定义 CSV / JSON schema，不重复车辆模型公式，不决定道路或参数数值，不设定 smoke scenario 中每辆车的初始 `x / y / v / type / compliance`，也不改变算法主循环。

边界说明：本文只定义日志、sanity check、PNG 与 smoke 验收的通用语义；字段全集、枚举、payload 结构、ScenarioConfig 与 expected_* 的结构化 schema 由 `CORMC代码数据结构设计_整理版.md` 维护。具体 MVS 场景初始状态和 expected_* 值由 `CORMC最小验证场景执行规格.md` 维护。

本文档的主要输入是：

```text
docs/复现讨论/CORMC时间步执行顺序梳理.md
```

辅助输入是：

```text
docs/复现讨论/CORMC论文公式与实现映射.md
docs/复现讨论/CORMC道路几何与区域规格.md
docs/复现讨论/CORMC参数规格.md
docs/复现讨论/CORMC车辆模型规格.md
docs/复现讨论/CORMC状态与模块接口规格.md
docs/复现讨论/CORMC复现spec体系梳理.md
```

本文档的使用方式是：沿一次 `S(t) -> S(t+dt)` 时间步推演，识别每个流程步骤需要记录哪些输入、决策、中间量、command / candidate / result、状态转移和异常，再将这些记录归档为 trajectory history、event history、sanity check、PNG 输出和后续论文级指标入口。

统一映射口径：

```text
流程步骤 -> 需要验证的问题 -> 需要记录的事件 / 中间量 -> 用途 -> 后续展开 spec
```

## 1. 文档定位

输出指标与日志验证规格回答以下问题：

```text
1. 运行第一版 smoke scenario 后，如何判断 APS / CUC / CMC / 纵向 / 横向 / commit 链路是否跑通。
2. 当车辆没有按预期协同、换道或合流时，日志应能定位到哪个流程步骤。
3. 轨迹图 PNG 至少需要展示哪些道路、车辆轨迹、区域标线和事件点。
4. 哪些 sanity check 是第一版必须记录的调试结果。
5. 哪些论文级指标只预留接口，后续实验阶段再展开。
```

本文档与相邻 spec 的职责边界如下：

```text
状态与模块接口规格：
    定义模块之间读什么、写什么、何时提交。

输出指标与日志验证规格：
    定义为了调试和验收需要记录什么、检查什么、输出什么，以及通用 smoke 验收语义。

代码数据结构设计：
    将本文档的记录需求落成 enum、record、buffer、config、ScenarioConfig 和字段名；它是字段与 schema 权威。

最小验证场景执行规格：
    根据本文档的日志和验收语义，给出每个 MVS 场景的具体车辆初始状态、preloaded state 与 expected_*。
```

## 2. 日志与验证总原则

第一版日志与验证遵守以下硬约束：

```text
1. 日志、轨迹历史和 sanity check 不反向改变车辆运动。
2. Step 10 information integration 发生在 commit 之后。
3. 日志可以读取 `S(t)`、本步 command / candidate / result、commit 后的 `S(t+dt)`。
4. 日志不能绕过 command / next-state / commit 写真实 `x / y / v / a`。
5. `x_global` 是算法与原始轨迹记录坐标；`x_plot` 只作为绘图派生值。
6. `CUC choice` 默认作为本步 command / event / history，不作为下一步控制状态。
```

工程补丁事件必须醒目标明来源。以下事件或状态一律记录为第一版工程补丁或第一版实现约束，不能写成论文原生机制：

```text
first_APS(MV)
assignment valid / invalid check
assignment invalid 后 immediate_APS_refresh
多 MV 共享 CV 仲裁
same_step_maneuver_relation_overlay
boundary speed cap 不可行时的保守处理入口
```

第一版日志优先级如下：

```text
第一优先级：
    能定位 APS / CUC / CMC / 纵向 / 横向 / commit 的链路错误。

第二优先级：
    能支撑 smoke scenario 的人工验收和轨迹图检查。

第三优先级：
    为后续论文级指标保留可扩展入口。
```

## 3. 按时间步流程推演的日志需求

本节按时间步总纲 Step 0-11 推演日志需求。表中的“需要记录的事件 / 中间量”是概念需求，不是代码字段全集。

### Step 0：清理与准备

| 流程步骤 | 需要验证的问题 | 需要记录的事件 / 中间量 | 用途 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| 清理驶出车辆 | 是否误删 active MV / CV | removed vehicle ids、removed reason、last lane / role、last `x_global` | 排查车辆突然消失 | 代码数据结构设计 |
| 清空本步 buffer | command / next-state 是否从空状态开始 | command buffer cleared、next-state buffer cleared | 排查跨步 command 污染 | 代码数据结构设计 |
| 保留持久 cache | APS cache 和 active maneuver trajectory state 是否被误清 | retained APS cache ids、retained maneuver states | 排查 APS 沿用失败、换道/合流中断 | 代码数据结构设计 |

验收要点：

```text
Step 0 可以清理每步临时 buffer，但不得清理跨步持久的 APS assignment cache 和 active maneuver trajectory state。
```

### Step 1：边界车辆生成

| 流程步骤 | 需要验证的问题 | 需要记录的事件 / 中间量 | 用途 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| 查看入口队列 | 本步是否检查了各入口 waiting vehicle | lane id、waiting vehicle id、queue state | 排查车辆生成缺失 | 代码数据结构设计 |
| 判断到达时距 | arrival headway 是否满足 | assigned arrival headway、actual waiting headway、pass / fail | 排查车辆生成节奏 | 车辆生成规格、代码数据结构设计 |
| 判断入口安全间隙 | 新车是否能安全进入 | entrance gap concept、pass / fail、fail reason | 排查入口重叠或缺车 | 车辆生成规格 |
| 生成车辆 | 新车是否进入本步冻结前 active set | new vehicle id、type、compliance、lane、initial `x_global / y / v / a` | 还原初始状态 | 代码数据结构设计 |

验收要点：

```text
Boundary generation 是 pre-freeze population update。
冻结 `S(t)` 后，不允许再插入新车影响本步 APS / CUC / CMC / 纵向 / 横向计算。
```

### Step 2：冻结 `S(t)`

| 流程步骤 | 需要验证的问题 | 需要记录的事件 / 中间量 | 用途 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| 冻结当前状态 | 后续模块是否基于同一状态快照 | `t`、`step`、`dt`、active vehicle count、vehicle ids | 排查时间步不一致 | 代码数据结构设计 |
| 记录配置引用 | 本步使用哪个道路 / 参数配置 | road config reference、parameter config reference | 排查参数或几何不一致 | 代码数据结构设计 |
| 状态快照边界 | 冻结后是否禁止直接改真实状态 | snapshot created marker | 排查中途状态写入 | 状态接口规格 |

验收要点：

```text
APS / CUC / CMC / 纵向 / 横向都只读本步冻结的 `S(t)`。
```

### Step 3：刷新车辆关系

| 流程步骤 | 需要验证的问题 | 需要记录的事件 / 中间量 | 用途 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| lane ordering | 是否使用 `x_global` 排序 | lane id、ordered vehicle ids、ordered `x_global` | 排查 leader / follower 错误 | 代码数据结构设计 |
| leader / follower | 普通纵向关系是否正确 | vehicle id、leader id、follower id、spacing concept | 排查纵向模型输入 | 车辆模型规格、代码数据结构设计 |
| TLV / TFV / LV / FV | CUC 所需邻接关系是否正确 | CV id、TLV / TFV / LV / FV ids | 排查 CUC safety / utility | 代码数据结构设计 |
| logical role | active lane-change / merge 车辆关系是否正确 | vehicle id、logical longitudinal role、active maneuver relation | 排查 active maneuver 中关系跳变 | 状态接口规格 |
| overlay relation basis | 是否为后续 same-step overlay 保留可引用关系基础 | TLV / TFV / FV basis ids、relation snapshot source | 支撑 Step 6 新启动换道时创建 overlay | 状态接口规格 |

验收要点：

```text
relations 使用 `x_global`，不使用 `x_plot`。
正在换道或合流车辆不按 `physical y` 最近 lane centerline 连续切换 leader / follower。
Step 3 只记录可供后续 overlay 引用的 TLV / TFV / FV relation basis；`same_step_maneuver_relation_overlay` 由 Step 6 的 lane-change command 创建。
```

### Step 4：处理 on-ramp MV

Step 4 同时包含 APS 分支和 CMC 分支。日志必须能看出 MV 当前走的是哪一个分支。

| 流程步骤 | 需要验证的问题 | 需要记录的事件 / 中间量 | 用途 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| MV 分支判断 | MV 本步走 APS、CMC waiting 还是继续 executing | MV id、`x_global`、zone state、`merge_state`、branch name | 排查调度分叉错误 | 状态接口规格 |
| APS 触发 | 是否 first_APS、APS_due 或 reused cache | trigger reason、last APS time、`T_APS` relation | 排查 APS 周期 | 代码数据结构设计 |
| APS 候选集合 | lane 2 候选是否正确 | candidate window、candidate ids、candidate `x_global / v` | 排查 CLV / CFV 缺失 | 道路几何规格、代码数据结构设计 |
| APS 到达预测 | 预测量是否可追踪 | `T*_MV`、candidate predicted position、anticipatory spacing concept | 排查 case 误判 | 最小验证场景规格 |
| APS assignment | 是否选出 CLV / CFV 和 case | CLV id、CFV id、APS case、`col_CLV / col_CFV`、Eq.10 desired spacing 语义 | 排查 CUC / CMC 输入 | 车辆模型规格、代码数据结构设计 |
| APS failure | 边界情况是否被显式记录 | failure reason：候选不足、无插入对、全部为正/负、`v_MV` 接近 0 等 | 排查工程兜底 | 代码数据结构设计 |
| effective assignment | Step 5 消费的是本步更新还是旧 cache | `effective_assignment_this_step` source | 排查同一步 APS 输出消费 | 状态接口规格 |
| CMC assignment validation | assigned CLV / CFV 是否仍有效 | valid / invalid、invalid reason | 排查 assignment invalid | 输出指标与日志验证规格、代码数据结构设计 |
| CMC gap decision | Eq.53 是否满足 | gap decision result、CLV / CFV ids、MV id、pass / fail | 排查 waiting / executing 转移 | 车辆模型规格 |
| boundary speed cap | CMC 是否产生速度上限 | boundary speed cap、cap source、cap feasibility | 排查边界防撞 | 车辆模型规格 |
| merge transition | `merge_state` 是否正确转移 | old merge_state、new merge_state、transition reason | 排查合流状态机 | 状态接口规格 |

验收要点：

```text
MV 未进 merging zone 时走 APS，不走 CMC 横向合流。
MV 已进 merging zone 且不是 executing 时走 CMC。
`merge_state == executing` 后继续既有合流轨迹，不重新判断是否开始合流。
assignment invalid 是第一版工程安全验证，不等价于每步 actual leader / follower 替换。
```

### Step 5：汇总 APS 产生的 CV 协同请求

| 流程步骤 | 需要验证的问题 | 需要记录的事件 / 中间量 | 用途 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| 收集请求 | 哪些 `effective_assignment_this_step` 产生 CV 请求 | MV id、CV id、role CLV / CFV、`col` | 排查 CUC 未触发 | 代码数据结构设计 |
| 冲突检测 | 同一 CV 是否被多个 MV 请求 | conflict group、requested CV id、requesting MV ids | 排查多 MV 冲突 | 代码数据结构设计 |
| 仲裁结果 | 谁赢、谁输、为什么 | winner MV id、loser MV ids、priority values concept | 排查工程仲裁 | 输出指标与日志验证规格 |
| loser 状态 | 未获胜 MV 如何处理 | loser result：waiting / conflict | 排查非法多 command | 状态接口规格 |

验收要点：

```text
多 MV 共享 CV 仲裁必须标注为第一版工程补丁。
同一 CV 在同一时间步不能接收多个互相冲突的协同目标。
```

### Step 6：处理 mainline 车辆 / CUC

| 流程步骤 | 需要验证的问题 | 需要记录的事件 / 中间量 | 用途 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| active lane-change 跳过 CUC | executing 车辆是否不重选 CUC | vehicle id、lane_change_state、skip reason | 排查重复决策 | 状态接口规格 |
| active CV 判断 | 本车是否是本步 active CV | vehicle id、active request id、source MV id、role CLV / CFV | 排查 CUC 漏执行 | 代码数据结构设计 |
| compliance 判断 | CHV 是否执行建议 | vehicle type、compliance、execute / ignore | 排查 CHV 行为 | 参数规格、车辆模型规格 |
| CUC utility / safety | choice 是否有依据 | utility comparison concept、target-lane safety pass / fail | 排查 choice 错误 | 车辆模型规格 |
| final choice | 最终 choice 是 lane 1 还是留 lane 2 | CUC choice、fallback reason、event/history marker | 排查 maneuver command | 状态接口规格 |
| lane-change command | 是否启动 lane 2 -> lane 1 | command created、target lane、same-step overlay created、overlay source basis | 排查换道启动 | 状态接口规格 |
| desired spacing override | 留 lane 2 时是否设置协同期望间距 | override created、applies to CFV case 2 / 4 | 排查 Eq.10 消费 | 车辆模型规格 |

验收要点：

```text
CUC choice 是本步 command / event，不作为下一时间步控制状态。
choice 1 才触发 `lane 2 -> lane 1`。
non-compliant CHV 不执行 CUC 建议。
本步新启动 lane change 时，`same_step_maneuver_relation_overlay` 在 Step 6 随 lane-change command 创建，并标注为第一版工程实现约束。
```

### Step 7：计算纵向动力学

| 流程步骤 | 需要验证的问题 | 需要记录的事件 / 中间量 | 用途 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| 纵向模式选择 | 每车使用哪种纵向语义 | vehicle id、mode：CAV cruising / CAV gap-regulating / CHV IDM / CUC cooperation / CMC waiting / CMC executing | 排查模型分支 | 车辆模型规格 |
| leader 输入 | 纵向 leader 是否正确 | vehicle id、leader id、relation source、overlay consumed yes / no、overlay source | 排查加速度异常 | 状态接口规格 |
| desired spacing | 是否使用普通或协同期望间距 | desired spacing source、override yes / no | 排查 Eq.10 误用 | 车辆模型规格 |
| candidate acceleration | 加速度候选是否可追踪 | candidate acceleration、constraint source concept | 排查速度跳变 | 车辆模型规格 |
| candidate speed | 速度候选是否可追踪 | candidate speed、pre-cap speed | 排查运动异常 | 车辆模型规格 |
| speed cap 合成 | 是否触发 boundary / front fallback | applicable speed caps、final planning speed、most conservative source | 排查安全约束 | 车辆模型规格 |

验收要点：

```text
case 2 / 4 中留在 lane 2 的 CFV 才消费 Eq.10。
boundary speed cap 从 CMC 流向纵向模型。
同车多速度约束取最保守 planning speed。
若本步新启动 lane change，纵向模型应记录是否消费 Step 6 创建的 same-step overlay source。
```

### Step 8：计算横向运动与安全修正

| 流程步骤 | 需要验证的问题 | 需要记录的事件 / 中间量 | 用途 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| active trajectory | 车辆是否处于换道或合流轨迹 | maneuver type、start state、target centerline、progress | 排查横向轨迹中断 | 车辆模型规格 |
| candidate y | 横向候选是否合理 | candidate y、target_y、progress | 排查横向越界 | 车辆模型规格 |
| front-collision fallback | 是否触发前向防撞回退 | fallback yes / no、fallback reason、speed used | 排查防撞约束 | 车辆模型规格 |
| boundary speed cap consumption | 合流轨迹是否消费已约束速度 | planning speed source、cap used yes / no | 排查边界防撞 | 车辆模型规格 |
| closed feature guard | 普通主线主动换道是否关闭 | unexpected ordinary lane-change attempt | 排查第一版关闭项误触发 | 时间步总纲 |

验收要点：

```text
第一版直接按正弦参考轨迹更新横向位置，不做 MPC tracking。
普通主线主动换道关闭。
front-collision-avoidance 是事前速度约束；simple collision check 是事后 sanity check。
```

### Step 9：同步提交 `S(t+dt)`

| 流程步骤 | 需要验证的问题 | 需要记录的事件 / 中间量 | 用途 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| 单车提交 | 每车是否只提交一次 | vehicle id、commit count、final `x_global / y / v / a` | 排查重复写状态 | 代码数据结构设计 |
| lane 更新 | lane 归属是否只在 commit 更新 | old lane、new lane、update reason | 排查 lane 提前更新 | 状态接口规格 |
| state transition | 状态机转移是否正确 | old / new `lane_change_state`、old / new `merge_state`、transition reason | 排查状态机 | 状态接口规格 |
| cache update / cleanup | APS cache 是否正确更新或清理 | cache update reason、cache cleanup reason | 排查 merged 后 cache 残留 | 状态接口规格 |
| CUC history | CUC choice 是否仅作为历史事件 | CUC choice history marker、source command | 排查历史 choice 误用 | 状态接口规格 |
| vehicle exit | 驶出车辆是否记录 | vehicle id、exit lane、exit `x_global`、exit reason | 排查流量和轨迹断点 | 代码数据结构设计 |

验收要点：

```text
每车每步只提交一次。
MV merged 后 `merge_state = merged`，下一步可压缩为 `none`。
CV 完成换道后 lane 变为 lane 1，`lane_change_state` 回到 normal。
`CUC choice` 只作为 event/history 记录。
```

### Step 10：Vehicle States Information Integration

| 流程步骤 | 需要验证的问题 | 需要记录的事件 / 中间量 | 用途 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| trajectory history | 是否形成可绘图轨迹 | per-step vehicle trajectory concept | 输出 PNG、调试轨迹 | 代码数据结构设计 |
| event history | 是否保留关键决策链 | event records concept | 定位错误 | 代码数据结构设计 |
| sanity check | 是否记录碰撞 / 越界 / 状态异常 | collision、near-collision、boundary violation、state inconsistency | smoke 验收 | 代码数据结构设计 |
| metrics input | 是否保留后续论文级指标入口 | observation region trajectory data concept | 后续平均速度 / 流量 | 论文级实验规格 |

验收要点：

```text
information integration 不反向改写 `S(t+dt)`。
记录结果只用于调试、验收和指标统计。
```

### Step 11：推进时间

| 流程步骤 | 需要验证的问题 | 需要记录的事件 / 中间量 | 用途 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| 推进真实时间 | 时间是否按 `dt` 推进 | old `t`、new `t`、old step、new step、`dt` | 排查 APS_due 周期错误 | 代码数据结构设计 |
| APS 时间口径 | 是否误用 `time + 1` | APS_due time basis concept | 排查 APS 周期 | 状态接口规格 |

验收要点：

```text
真实时间使用 `t += dt` 推进。
APS_due 使用真实时间或等价 step-time 映射。
```

## 4. 按记录类型归档的输出索引

本节把流程推演得到的日志需求归档为后续代码数据结构设计可消费的输出类别。这里仍只定义概念、验收用途和排错语义，不定义字段名、字段全集、文件格式或 schema；字段权威以 `CORMC代码数据结构设计_整理版.md` 为准。

### 4.1 Trajectory History

trajectory history 用于绘图、回放、smoke scenario 验收和后续论文级指标。第一版至少需要能表达：

```text
t / step
vehicle id
vehicle type
CHV compliance
x_global
y
v
a
physical lane
mainline / on-ramp role
logical longitudinal role
lane_change_state
merge_state
```

约束：

```text
x_global 是原始轨迹坐标。
x_plot 只作为绘图派生值，不进入算法状态。
```

### 4.2 Event History

event history 用于定位 APS / CUC / CMC / 纵向 / 横向 / commit 的链路错误。第一版至少需要覆盖以下事件类型：

```text
boundary generation event
relation refresh event
APS event
assignment invalid event
cooperative request event
multi-MV conflict event
CUC event
CMC event
longitudinal model event
lateral trajectory event
commit event
sanity check event
engineering patch event
```

每个事件至少要能追溯到以下概念：

```text
step
t
module
event type
related vehicle id
reason
result
source state / command / candidate 概念
```

具体字段名、嵌套结构、CSV / JSON / Parquet 等格式由 `CORMC代码数据结构设计.md` 决定。

### 4.3 Sanity Check Result

sanity check result 用于记录仿真是否明显违反基本安全或状态机约束。第一版至少覆盖：

```text
collision
near-collision
boundary violation
assignment invalid
state machine inconsistency
lane / geometry inconsistency
unexpected ordinary lane-change attempt
multiple commit for one vehicle in one step
```

sanity check 的结果只记录、告警和用于验收，不反向改变本步车辆状态。

### 4.4 PNG Output

第一版至少输出一张轨迹图 PNG。PNG 输出依赖 trajectory history 和 event history，不反向影响算法。

最低内容：

```text
lane 1 / lane 2 / on-ramp 的轨迹
fixed cooperative zone 标线
merging zone 标线
lane-change start event
merge start event
merge completion event
assignment invalid / conflict / boundary violation 等异常点
```

绘图坐标：

```text
第一版主 PNG 推荐为 time-space trajectory。
横轴使用 x_plot = x_global - warmup_length 展示论文 Fig.13 对齐坐标。
纵轴使用 t。
lane 1 / lane 2 / on-ramp 使用分面、纵向偏移或颜色区分。
内部日志仍保留 x_global。
```

可选输出：

```text
x-y 平面轨迹图可用于检查横向合流 / 换道形态，但不作为第一版最低要求。
```

第一版不要求复刻论文所有 Fig.11-Fig.14，也不要求 SUMO 对比图。

## 5. Sanity Check 与安全验证

### 5.1 Collision Check

collision check 用于发现车辆在 commit 后是否出现明显重叠。本文档不定义具体矩形碰撞算法或阈值，只要求后续实现能记录：

```text
related vehicle ids
time / step
lane / role context
collision type concept
check result
```

### 5.2 Near-collision Check

near-collision check 用于记录接近碰撞风险。阈值和具体判断口径由代码数据结构设计或后续验证实现细化。本文档只要求记录风险事件，不用它改变本步运动。

### 5.3 Boundary Violation

boundary violation 至少需要记录：

```text
MV 是否越过 x_ramp_end_global 仍未 merged
vehicle id
x_global
merge_state
boundary id
violation reason
```

越界后的具体失败处理、停车策略或终止策略不在本文档决定。

### 5.4 Assignment Validity

assignment validity check 至少需要记录：

```text
assigned CLV / CFV 是否仍在 lane 2
assigned CLV / CFV 是否已驶离
assigned CLV / CFV 是否仍能形成目标协同 gap
assignment invalid reason
assignment_invalid_policy
```

`assignment_invalid_policy` 若采用 `immediate_APS_refresh`，必须标明为第一版工程补丁。

### 5.5 State Machine Consistency

状态机一致性检查至少覆盖：

```text
lane_change_state == executing 时不重新执行 CUC
merge_state == executing 后不撤销、不重新判断开始合流
merged 后 APS assignment cache 被清理
lane 归属只在 commit 阶段正式更新
CUC choice 不作为下一步控制状态
```

### 5.6 Geometry Consistency

几何一致性检查至少覆盖：

```text
算法状态不使用 x_plot
relations 不按 x_plot 排序
换道 / 合流目标 centerline 与道路几何规格一致
普通主线主动换道未被误触发
```

## 6. 轨迹图 PNG 输出规格

第一版轨迹图 PNG 用于人工检查车辆运动、协同、换道、合流和异常。它不是论文图完全复刻。

最低绘图内容：

```text
1. lane 1、lane 2、on-ramp 三条车道或对应轨迹分区。
2. 每辆车随时间的轨迹线。
3. fixed cooperative zone 与 merging zone 的纵向标线。
4. CUC lane-change start 和 lane-change completion 事件点。
5. MV merge start 和 merge completion 事件点。
6. assignment invalid、multi-MV conflict、boundary violation、collision / near-collision 等异常点。
```

坐标口径：

```text
第一版主 PNG 推荐为 time-space trajectory。
横轴使用 x_plot。
纵轴使用 t。
lane 1 / lane 2 / on-ramp 使用分面、纵向偏移或颜色区分。
日志和算法内部仍使用 x_global。
```

可选图型：

```text
x-y 平面轨迹图可用于观察横向换道 / 合流轨迹，但不作为第一版最低验收输出。
```

第一版 PNG 验收标准：

```text
能看出 lane 1 / lane 2 / on-ramp 中车辆随时间运动。
能看出协同换道和 MV 合流是否发生。
能看出 MV 是否在 on-ramp downstream boundary 前完成合流。
能定位关键异常事件的大致位置和时间。
```

## 7. Smoke Scenario 验收口径

本文档不设定具体 smoke scenario 初始车辆状态，只定义后续最小验证场景应能通过日志和 PNG 验收什么。

### 7.1 APS Smoke

日志应能还原：

```text
MV 是否执行 first_APS 或 APS_due
候选车辆集合
CLV / CFV
APS case
col_CLV / col_CFV
effective_assignment_this_step 来源
```

### 7.2 CUC Smoke

日志应能还原：

```text
active CV
source MV
vehicle type / compliance
CUC choice
安全回退原因
lane-change command 或留 lane 2 协同语义
```

### 7.3 CMC Smoke

日志应能还原：

```text
assignment valid / invalid
Eq.53 gap 判断结果
boundary speed cap
waiting / executing / merged state transition
merged 后 cache cleanup
```

### 7.4 Conflict Smoke

日志应能还原：

```text
多个 MV 请求同一 CV
winner / loser
仲裁依据
loser 的 waiting / conflict 结果
工程补丁标记
```

### 7.5 Safety Smoke

日志应能记录：

```text
collision
near-collision
boundary violation
front-collision fallback
boundary speed cap 生效
state machine inconsistency
```

### 7.6 PNG Smoke

轨迹图应能看出：

```text
三条车道轨迹
区域边界
协同换道
MV 合流
merge completion
异常事件点
```

## 8. 论文级指标预留

论文级指标不是第一版 smoke scenario 的必须验收项，但需要预留入口，避免后续重做记录结构。

后续论文级实验可能需要：

```text
Edie average speed
Edie average flow
Edie average density
time spent in observation region
distance traveled in observation region
mainline / merging bottleneck throughput
delay
merge success rate
lane-change count
CHV compliance sensitivity
CAV penetration sensitivity
on-ramp flow ratio sensitivity
```

这些指标需要的基础数据主要来自：

```text
trajectory history
vehicle entry / exit event
observation region definition
scenario config
random seed / run id
```

`scenario config` 至少需要能表达以下论文级实验维度的概念：

```text
CAV penetration rate
CHV compliance rate
on-ramp flow ratio
mainline demand ratio
strategy / baseline label
```

第一版只要求记录足够支撑后续扩展的轨迹和事件概念，不在本文档展开 Edie 指标公式或论文表格复现流程。

## 9. 后续规格承接

后续文档职责如下：

```text
CORMC代码数据结构设计_整理版.md:
    将本文档中的 trajectory history、event history、sanity check result、PNG 输出需求落成 record、buffer、enum、config 和字段名。

CORMC最小验证场景执行规格.md:
    根据本文档的 APS / CUC / CMC / conflict / safety smoke 验收口径，维护具体车辆初始状态、preloaded state、expected_events、expected_sanity_checks 与 expected_png_features。

执行计划文档:
    将日志、PNG、sanity check 和 smoke 验收拆成可执行实现任务。

论文级实验规格:
    后续若需要复刻论文表格和 SUMO 对比，再展开 Edie 指标、随机种子、观测区和批量实验流程。
```

承接约束：

```text
1. 后续数据结构设计不得把日志字段反向提升为算法必要状态，除非先修订状态接口规格。
2. 后续最小验证场景必须引用本文档的通用日志语义和 PNG 特征语义；具体 expected_* 由执行规格维护。
3. 后续执行计划不能首次决定核心日志概念，只能实现或小修本文档已定义的需求。
```

## 10. 验收检查

本文档应满足以下检查：

```text
1. 能沿 Step 0-11 回答“这一步为什么要记、记什么、用于验证什么、后续谁消费”。
2. 能定位 APS 未选出 CLV / CFV、CUC choice 安全回退、CMC Eq.53 不满足、boundary speed cap 生效、multi-MV conflict、assignment invalid 等关键问题。
3. 明确日志和 sanity check 不反向改变车辆运动。
4. 明确 `CUC choice` 只作为本步 command / event / history，不作为下一步控制状态。
5. 明确工程补丁事件必须保留工程补丁来源标记。
6. 明确 `x_plot` 只作为绘图派生值。
7. 定义了 trajectory history、event history、sanity check result 和 PNG 输出的概念需求。
8. 定义了 APS / CUC / CMC / conflict / safety / PNG smoke 验收口径。
9. 预留论文级指标入口，但没有把论文级表格复刻作为第一版验收要求。
10. 文档没有设计 Python dataclass。
11. 文档没有定义 CSV / JSON schema、日志字段全集或 ScenarioConfig schema。
12. 文档没有重复车辆模型公式。
13. 文档没有设定 smoke scenario 初始车辆。
14. 文档没有改变已有时间步主循环。
15. 明确字段全集由代码数据结构设计维护，具体场景输入由最小验证场景执行规格维护。
```
