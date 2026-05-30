# CORMC 道路几何与区域规格

本文档定义 CORMC 第一版复现使用的道路几何、空间坐标、区域判定和边界口径。它为时间步主循环提供空间坐标底座，服务于边界车辆生成、APS 候选车辆搜索、merging zone / cooperative zone 判断、CMC 边界防撞、横向轨迹目标 centerline 和轨迹图输出。

本文档不是车辆模型规格、不是参数全集、不是数据结构设计，也不是最小验证场景设计。具体车辆如何计算加速度、如何沿正弦轨迹移动、如何记录事件、如何配置 smoke scenario，分别由后续车辆模型、状态接口、日志验证和最小验证场景规格展开。

本文档的上游依据是：

```text
docs/复现讨论/CORMC时间步执行顺序梳理.md
docs/复现讨论/CORMC复现spec体系梳理.md
docs/复现讨论/CORMC论文公式与实现映射.md
docs/papers/P002-2023-Hou-Cooperative-On-Ramp-Merging-Control.md
docs/papers/Fig. 10. Illustration of a simple merging network in simulation.png
docs/papers/THE VALUES OF VARIABLES AND PARAMETERS.png
```

## 1. 文档定位

第一版道路几何规格回答以下问题：

```text
1. 仿真内部使用哪个纵向坐标。
2. 论文轨迹图中的局部坐标如何映射到仿真内部坐标。
3. lane 1、lane 2、on-ramp 的横向 centerline 如何表达。
4. merging zone、cooperative zone、warm-up section 和主线边界如何定义。
5. MV 何时处于 APS 阶段，何时进入 CMC 阶段。
6. APS 搜索 lane 2 候选车时使用什么空间窗口。
7. 车辆驶离路段和 MV 越过 on-ramp downstream boundary 如何判定。
```

本文档不负责：

```text
1. 不写 CAV / CHV / IDM / CPID / CUC / CMC 的运动公式。
2. 不决定所有模型参数的最终数值。
3. 不设计 Python dataclass、enum、config 或 buffer。
4. 不设定最小验证场景中的车辆初始 x / y / v。
5. 不决定 MV 超过 x_ramp_end_global 后的保守处理策略。
```

## 2. 坐标系统

第一版使用两套纵向坐标口径，但仿真内部只使用全局坐标。

| 名称 | 定义 | 用途 | 状态 |
| --- | --- | --- | --- |
| `x_global` | 以主线 starting boundary 为 0，沿车辆行驶方向递增 | APS / CUC / CMC / 纵向运动 / 横向轨迹 / 边界判断的内部唯一纵向坐标 | 第一版实现约束 |
| `x_plot` | `x_plot = x_global - warmup_length` | 仅用于轨迹图展示、论文 Fig.13 坐标对齐和观测区展示 | 第一版绘图口径 |
| `y` | 横向坐标，用于表示车道 centerline、换道和合流的横向位置 | 横向轨迹、lane centerline、轨迹图 | 第一版实现约束 |

所有算法内部计算均使用 `x_global`。不得在 APS、CUC、CMC、车辆关系刷新或状态提交中使用 `x_plot` 替代 `x_global`。

论文 Fig.10 标出 warm-up section 长度为 `4000 m`。因此：

```text
warmup_length = 4000 m
x_plot = x_global - 4000 m
x_global = x_plot + 4000 m
```

论文 Fig.13 文本中提到 MVs 的 merging start positions 分布在 `2950 m` 到 `3250 m`。该区间应解释为局部绘图坐标：

```text
2950 m <= x_plot <= 3250 m

对应全局坐标：

6950 m <= x_global <= 7250 m
```

也就是说，Fig.13 的 `2950-3250 m` 与 Fig.10 / Section V-C 的 `6950-7250 m` 不冲突；前者是减去 `4000 m` warm-up section 后的绘图坐标，后者是仿真内部全局坐标。

## 3. 主线与匝道纵向几何

第一版采用论文 Fig.10 和 Section V-C 中的主线几何设置。

| 几何量 | 符号 / 字段名 | 数值 | 来源状态 | 说明 |
| --- | --- | ---: | --- | --- |
| 主线起点 | `mainline_start_global` | `0 m` | paper | 主线 starting boundary，也是 `x_global = 0` |
| warm-up section 长度 | `warmup_length` | `4000 m` | paper | Fig.10 标注，用于解释局部绘图坐标和观测展示 |
| 主线总长度 | `mainline_length` | `10000 m` | paper | Section V-C：mainline section total length 为 10 km |
| 主线终点 | `mainline_end_global` | `10000 m` | paper-derived | `mainline_start_global + mainline_length` |
| merging zone 起点 | `x0_m_global` | `6950 m` | paper | Section V-C / Fig.10：on-ramp located at 6950 m，下文按 merging zone 起点使用 |
| merging zone 长度 | `L_merging` | `300 m` | paper | Section V-C / Fig.10 |
| on-ramp 下游边界 | `x_ramp_end_global` | `7250 m` | paper-derived | `x0_m_global + L_merging` |
| merging zone 后主线长度 | `post_merging_mainline_length` | `2750 m` | paper-derived | `mainline_end_global - x_ramp_end_global`，Fig.10 也标注 2750 m |
| merging zone 起点绘图坐标 | `x0_m_plot` | `2950 m` | paper-derived | `x0_m_global - warmup_length` |
| on-ramp 下游边界绘图坐标 | `x_ramp_end_plot` | `3250 m` | paper-derived | `x_ramp_end_global - warmup_length` |

其中 `x0_m_global` 是论文公式中的 `x_0^m`，表示 merging zone 的起点纵向坐标。`x_ramp_end_global` 是论文公式中的 `x_ramp^end`，表示 on-ramp downstream boundary 的纵向坐标。

第一版内部几何关系固定为：

```text
mainline_start_global = 0 m
warmup_length = 4000 m
x0_m_global = 6950 m
L_merging = 300 m
x_ramp_end_global = x0_m_global + L_merging = 7250 m
mainline_end_global = 10000 m
post_merging_mainline_length = mainline_end_global - x_ramp_end_global = 2750 m
```

### Fig.10 中的 on-ramp 100 m

Fig.10 在 on-ramp 下方标注 `100 m`，并标出 on-ramp 的 `Start point`。第一版将该 `100 m` 记录为 on-ramp upstream preparation / insertion segment 的几何提示，用于后续车辆生成、绘图或最小验证场景布置时参考。

该 `100 m` 不改变 APS / CMC 的核心纵向边界：

```text
合流相关核心边界仍为：
    x0_m_global = 6950 m
    x_ramp_end_global = 7250 m
```

on-ramp 入口车辆的具体初始坐标、入口安全间隙和预生成队列位置，由车辆生成规格或最小验证场景规格决定。本文档不提前设定 on-ramp 入口车辆初始 `x`。

## 4. 车道与横向 Centerline

第一版保留论文中的道路结构：

```text
lane 1:
    主线左侧 / 外侧目标车道。
    CUC 中 lane 2 的 CV 可从 lane 2 换到 lane 1。

lane 2:
    主线右侧 / 邻近 on-ramp 的车道。
    MV 从 on-ramp 合流进入 lane 2。
    APS 在 lane 2 中搜索 CLV / CFV。

on-ramp:
    匝道车道。
    MV 的初始所在车道。
```

CUC 与 CMC 在横向几何上的目标关系为：

```text
CUC triggered lane change:
    lane 2 -> lane 1

CMC triggered merging:
    on-ramp -> lane 2
```

论文公式中使用 `L_w` 表示 lane width，但当前 Markdown、Table I 图片和 Fig.10 图片没有给出 `L_w` 的典型值。第一版道路几何文档只使用符号 `L_w`，不把具体数值写成论文参数。

为保证第一版代码和图形口径一致，建议采用以下横向默认布局：

| 车道 | centerline | 来源状态 | 说明 |
| --- | ---: | --- | --- |
| `lane 1` | `y = +L_w` | first-version-default | CUC 中 lane 2 CV 的换道目标 |
| `lane 2` | `y = 0` | first-version-default | MV 的合流目标车道 |
| `on-ramp` | `y = -L_w` | first-version-default | MV 的初始车道 |

该布局只定义横向方向和相对位置。`L_w` 的具体数值交给 `CORMC参数规格.md` 统一确认。若第一版实现需要默认值，可在参数规格中将常用车道宽度 `3.5 m` 标为 `first-version-default / to-review`，而不是在本文档中作为论文给定值固定。

正在换道或合流的车辆可能处于两条车道 centerline 之间。它的 `physical y` 用于横向轨迹和绘图；它的 `logical longitudinal role` 仍按时间步总纲和状态接口规格定义，不应仅根据 `y` 更接近哪条 centerline 来连续切换 leader / follower。

## 5. 区域定义

CORMC 论文将道路划分为 mainline zone、cooperative zone 和 merging zone。第一版几何规格定义如下。

### 5.1 Mainline Zone

mainline zone 是 merging zone 上游、没有进入协同合流核心区的主线运行区域。第一版关闭普通主线主动换道，因此 mainline zone 主要服务以下流程：

```text
1. lane 1 / lane 2 主线车辆纵向行驶。
2. 边界车辆生成和 warm-up 后交通状态形成。
3. 车辆关系刷新中的普通 lane ordering 和 leader / follower。
```

mainline zone 的精确边界不作为第一版 APS / CMC 的触发条件。MV 的关键触发边界是 `x0_m_global` 和 `x_ramp_end_global`。

### 5.2 Merging Zone

merging zone 是 MV 执行 CMC 合流决策和合流轨迹的区域。

```text
merging_zone_global = [x0_m_global, x_ramp_end_global]
                    = [6950 m, 7250 m]

merging_zone_plot = [x0_m_plot, x_ramp_end_plot]
                  = [2950 m, 3250 m]
```

MV 的调度分叉使用全局坐标：

```text
if x_MV_global < x0_m_global:
    MV 尚未进入 merging zone
    本步按时间步总纲执行 APS 或沿用 APS assignment

if x0_m_global <= x_MV_global <= x_ramp_end_global:
    MV 已在 merging zone
    本步进入 CMC

if x_MV_global > x_ramp_end_global and MV 尚未 merged:
    MV 已越过 on-ramp downstream boundary
    属于边界防撞失败 / 保守处理问题
```

最后一种情况只在本文档中定义为几何越界。具体是减速、停车、失败记录、强制安全处理还是终止场景，由车辆模型规格和日志验证规格决定。

### 5.3 Cooperative Zone

论文中 cooperative zone 有两个容易混淆的口径。第一版文档同时记录二者，并要求后续实现不要混用。

第一种是 Fig.10 图示的固定 cooperative zone。它位于 merging zone 起点上游，长度为 `300 m`：

```text
fixed_cooperative_zone_global = [x0_m_global - 300 m, x0_m_global]
                              = [6650 m, 6950 m]

fixed_cooperative_zone_plot = [2650 m, 2950 m]
```

第二种是论文正文描述的 MV 动态协作语义：cooperative zone 随 on-ramp MV 的位置动态移动，并位于该 MV 上游 `300 m`。第一版将其称为动态协作窗口：

```text
dynamic_coop_window_for(MV):
    长度 = 300 m
    随 MV 的 x_MV_global 更新
    具体端点口径由后续状态接口 / 车辆模型规格确认
```

二者的使用边界：

```text
fixed_cooperative_zone:
    主要用于解释 Fig.10、绘图展示、全局区域标线和论文坐标对齐。

dynamic_coop_window_for(MV):
    主要用于表达论文正文中“cooperative zone moves dynamically according to the position of MVs”的语义。
```

后续状态接口和代码数据结构规格需要决定是否同时保留 `fixed_coop_zone` 与 `dynamic_coop_window` 两个概念。本文档只明确它们的几何含义和来源，不提前决定字段形态。

第一版实现必须明确约束：APS 候选车搜索不得使用 `fixed_cooperative_zone` 或 `dynamic_coop_window_for(MV)` 替代。APS 候选集合唯一按 `lane 2` 中 `[x_MV_global - L_cr, x_MV_global + L_cr]` 计算。cooperative zone 仅用于论文区域语义、绘图标注或后续指标解释。

### 5.4 APS 候选窗口

APS 的候选车辆搜索范围不是 cooperative zone 的同义词。它来自论文 Eq.1-Eq.2 和 Table I 中的 communication range。

```text
L_cr = 300 m

APS candidate window for MV:
    [x_MV_global - L_cr, x_MV_global + L_cr]
```

APS 只在 `lane 2` 中搜索该窗口内的候选车辆：

```text
x_MV_global - L_cr <= x_i_global <= x_MV_global + L_cr
```

这里的 `x_i_global` 是 lane 2 候选车辆的全局纵向坐标。该判断必须使用 `x_global`，不能使用 `x_plot`。

## 6. 边界与驶离判定

### 6.1 主线边界

主线入口边界：

```text
mainline_start_global = 0 m
```

主线出口边界：

```text
mainline_end_global = 10000 m
```

第一版主线车辆驶离判定：

```text
if x_vehicle_global > mainline_end_global:
    车辆驶出主线仿真路段
    下一时间步清理
```

车辆长度是否参与驶离判定由后续状态接口或车辆模型规格细化。本文档只定义主线几何边界。

### 6.2 Warm-up Section

warm-up section 为：

```text
warmup_global = [0 m, 4000 m]
```

它的主要作用是：

```text
1. 解释 Fig.10 和 Fig.13 的坐标差异。
2. 支持后续输出指标或轨迹图只展示 warm-up 后的局部观测区。
3. 支持车辆在进入核心观测区前形成更自然的交通状态。
```

warm-up section 不改变仿真内部坐标，也不改变 APS / CMC 的触发边界。算法内部仍使用 `x_global`。

### 6.3 On-ramp Downstream Boundary

on-ramp downstream boundary 为：

```text
x_ramp_end_global = 7250 m
```

CMC 的 boundary-collision-avoidance 使用该几何边界。本文档只定义边界位置。边界防撞如何计算 speed cap、speed cap 如何和 longitudinal model / front-collision-avoidance 合成，由车辆模型规格展开。

第一版越界判定：

```text
if MV 未 merged and x_MV_global > x_ramp_end_global:
    MV 越过 on-ramp downstream boundary
    记录为 boundary violation 或进入后续保守处理
```

具体记录字段、严重程度、是否终止仿真、是否强制停车，不在本文档决定。

## 7. 与时间步流程的对应关系

本文档只提供几何判断和坐标口径，不改变 `CORMC时间步执行顺序梳理.md` 中的主循环。

| 时间步位置 | 使用本文档的内容 | 说明 |
| --- | --- | --- |
| Step 1 边界车辆生成 | `mainline_start_global`、`mainline_end_global`、on-ramp 几何提示 | 判断入口和出口边界；on-ramp 入口初值后续规格细化 |
| Step 3 车辆关系刷新 | lane centerline、physical lane、`x_global` 排序 | lane ordering 和 leader / follower 基于冻结的 `S(t)` |
| Step 4 处理 on-ramp MV | `x0_m_global`、`x_ramp_end_global`、merging zone 判定 | `x < x0_m_global` 走 APS；进入 `[x0_m_global, x_ramp_end_global]` 走 CMC |
| Step 4 APS 内部 | `L_cr`、APS candidate window | 在 lane 2 中搜索 `[x_MV - L_cr, x_MV + L_cr]` 内候选车辆 |
| Step 6 CUC | lane 2 -> lane 1 横向目标 | CUC 输出 choice 后，横向目标由 lane centerline 提供 |
| Step 8 横向运动 | `lane 1`、`lane 2`、`on-ramp` centerline | 正弦换道 / 合流使用目标 centerline |
| Step 8 CMC 边界防撞 | `x_ramp_end_global` | boundary speed cap 的几何边界输入 |
| Step 9 同步提交 | `x_global`、`y`、physical lane | 更新车辆位置和车道归属 |
| Step 10 信息集成 | `x_plot`、zone 标线、warm-up | 轨迹图可用局部坐标展示，但不回写算法状态 |

特别需要保持以下边界：

```text
1. `x_plot` 只用于输出，不进入算法状态。
2. APS candidate window 不是 cooperative zone。
3. fixed cooperative zone 主要用于 Fig.10 / 绘图对齐。
4. dynamic cooperative window 表达 MV 动态协作语义，具体字段后续再定。
5. MV 越过 x_ramp_end_global 后的处理不是道路几何文档职责。
```

## 8. 留给后续 Spec 的问题

以下问题不阻止道路几何规格成立，但需要后续文档继续细化。

1. **`L_w` 的最终数值**
   - 当前论文 Markdown、Table I 图片和 Fig.10 图片未给出 lane width 的典型值。
   - 本文档使用符号 `L_w`。
   - 若第一版需要数值默认，可在 `CORMC参数规格.md` 中设为 `3.5 m` 并标注 `first-version-default / to-review`。

2. **on-ramp 入口具体初始坐标**
   - Fig.10 标出 on-ramp `Start point` 和 `100 m`。
   - 车辆生成规格或最小验证场景规格需要决定 on-ramp 入口车辆如何布置、入口队列从何处进入。
   - 本文档不提前设定 smoke scenario 中 MV 的初始 `x`。

3. **fixed cooperative zone 与 dynamic cooperative window 的代码形态**
   - 本文档已经区分两者的几何含义。
   - 状态接口规格和代码数据结构设计需要决定是否显式保留两个字段或用函数派生。

4. **MV 越过 `x_ramp_end_global` 后的保守处理**
   - 本文档只定义几何越界。
   - 车辆模型规格需要决定 boundary speed cap 不可行或越界时的车辆行为。
   - 日志验证规格需要决定 boundary violation 如何记录。

5. **输出指标观测区**
   - 论文在 Section VI 中使用主线起点下游 `4 km` 到 `10 km` 的 12 个 500 m 区间计算平均流量和速度。
   - 第一版轨迹图和 smoke scenario 验证可先用 `x_plot` 观测区展示。
   - 论文级指标的观测区切分由输出指标与日志验证规格展开。

## 9. 验收检查

本文档应满足以下检查：

```text
1. 后续实现者能明确知道算法内部使用 x_global。
2. 后续实现者能明确知道绘图对齐使用 x_plot = x_global - 4000 m。
3. 后续实现者能明确知道 x0_m_global = 6950 m。
4. 后续实现者能明确知道 x_ramp_end_global = 7250 m。
5. 后续实现者能解释 Fig.13 的 2950-3250 m 来自减去 4000 m warm-up。
6. 后续实现者不会把 APS candidate window 误写成 cooperative zone。
7. 后续实现者能明确知道 MV 何时走 APS、何时走 CMC。
8. 后续实现者能明确知道 lane 2 是合流目标，lane 1 是 CUC 换道目标。
9. 文档没有提前决定 smoke scenario 车辆初始位置。
10. 文档没有提前决定 Python 数据结构。
11. 文档没有把 L_w = 3.5 m 写成论文给定参数。
12. 文档没有决定 CMC 越界失败后的车辆行为。
```

## 10. 来源状态汇总

本文档使用以下来源状态标记：

```text
paper:
    论文正文、Fig.10 或 Table I 图片明确给出。

paper-derived:
    由论文给出的几何量直接计算得到。

first-version-default:
    论文未明确，但第一版为了统一实现和绘图口径采用的默认约定。

to-review:
    后续需要在参数规格、PDF 原图或其他材料中复核。
```

当前关键几何量的来源状态：

| 内容 | 状态 |
| --- | --- |
| 主线两车道 + 一条 on-ramp | paper |
| 主线总长 `10000 m` | paper |
| warm-up section `4000 m` | paper |
| merging zone 起点 `6950 m` | paper |
| merging zone 长度 `300 m` | paper |
| cooperative zone 长度 `300 m` | paper |
| communication range `L_cr = 300 m` | paper |
| `x_ramp_end_global = 7250 m` | paper-derived |
| Fig.13 局部坐标 `2950-3250 m` 对应全局 `6950-7250 m` | paper-derived |
| `lane 2: y = 0`、`lane 1: y = +L_w`、`on-ramp: y = -L_w` | first-version-default |
| `L_w` 具体数值 | to-review |
