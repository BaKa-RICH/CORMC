# CORMC 状态与模块接口规格

本文档是第一版 CORMC 复现的概念级接口契约。它承接时间步总纲、公式映射、道路几何、参数规格和车辆模型规格，定义 `S(t)`、`S(t+dt)`、command / next-state、relations、APS assignment cache、CUC choice、`lane_change_state`、`merge_state`，以及各模块的输入输出边界。

本文档不写 Python dataclass，不定日志字段全集，不重复车辆模型公式，不重新定义时间步主循环。它回答“模块之间读什么、写什么、何时提交、哪些状态必须存在”；后续 `CORMC代码数据结构设计.md` 再把这些概念落成代码级类型和字段。

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
docs/复现讨论/CORMC复现spec体系梳理.md
docs/复现讨论/CORMC复现讨论对齐记录.md
```

本文档的使用方式是：沿一次 `S(t) -> S(t+dt)` 的状态推进过程，识别哪些内容必须作为持久状态存在，哪些内容只应作为本步派生结果存在，哪些模块可以写 command，哪些模块只能消费 next-state，哪些内容必须等到同步提交阶段才能成为真实状态。

## 0. 工程补丁边界

本节是本文档的硬边界。以下内容可以存在于状态接口文档和后续实现中，但必须继续标注为第一版工程补丁，不能写成论文已经明确给出的原生机制：

```text
1. first_APS(MV)
2. assignment valid / invalid check
3. assignment invalid 后 immediate_APS_refresh
4. 多 MV 共享 CV 仲裁
5. same_step_maneuver_relation_overlay
6. boundary speed cap 不可行时的保守处理入口
```

这些内容的作用是让第一版同步仿真稳定运行，并明确异常或冲突状态的入口。它们不改变 CORMC 论文中 APS / CUC / CMC 的主语义；后续文档引用这些概念时，也必须保留“工程补丁 / 第一版实现约束”的来源标记。

统一约束口径：

```text
流程位置 -> 需要读取的状态 -> 允许写入的 command / next-state / cache -> 持久化规则 -> 后续展开 spec
```

## 1. 文档定位

本文档是第一版 CORMC 复现的概念级接口 spec，不是代码数据结构设计，不是日志规格，也不是车辆模型计算说明。它直接服务于时间步总纲中 Step 2 冻结 `S(t)`、Step 9 同步提交 `S(t+dt)` 的实现语义。

本文档和其他上游 spec 的职责区分如下：

```text
时间步总纲：
    规定每个时间步什么时候做什么。

公式映射：
    规定每个流程点对应论文哪些公式、正文或图示依据。

道路几何与参数规格：
    提供 `x_global`、lane centerline、区域边界和统一参数来源。

车辆模型规格：
    规定给定输入后如何计算纵向 / 横向 candidate。

状态与模块接口规格：
    规定模块之间至少要共享哪些概念、读什么、写什么、何时提交。

代码数据结构设计：
    再把这些概念落成 dataclass、enum、buffer、config 和字段名。
```

本文档明确不做以下事情：

```text
1. 不设计 Python dataclass、enum 或最终字段名。
2. 不重复 Eq.10、Eq.17-Eq.56、IDM、CPID、正弦轨迹等车辆模型公式。
3. 不定义完整日志字段、事件表和碰撞检查输出格式。
4. 不设定 smoke scenario 中每辆车的初始 `x / y / v / type / compliance`。
5. 不改变 `CORMC时间步执行顺序梳理.md` 已确定的主循环顺序。
```

## 2. 状态读写总原则

第一版状态接口遵守以下硬约束：

```text
1. 每个时间步先冻结 `S(t)`。
2. APS / CUC / CMC / 纵向 / 横向模块全部只读 `S(t)`。
3. 所有模块只写本步 command / next-state 候选 / 本步派生结果。
4. `S(t+dt)` 只能由同步提交阶段生成。
5. `S(t+dt)` 只供下一时间步使用。
6. 任一车辆每个时间步最多提交一次最终状态。
```

由此得到的接口约束是：

```text
command:
    表示本步意图、约束或 maneuver 选择。

next-state:
    表示基于 `S(t)` 计算出的候选下一状态。

real state:
    只存在于 `S(t)` 与 commit 后的 `S(t+dt)`。
```

任何模块都不得在本步中途直接修改车辆真实 `x / y / v / a` 后再让其他模块读取该已更新值。否则会破坏冻结输入语义，导致前后车更新顺序依赖。

`relations` 也必须按冻结 `S(t)` 计算得到。它是本步派生快照，不是中途可被模块反复重写的真实状态。若某车本步被决定换道或开始合流，这一决策只写入 command / next-state，本步其他模块仍消费同一份冻结关系快照，除非时间步总纲明确要求消费的是“active maneuver relation”这类从冻结状态派生出的关系语义。

## 3. `S(t)` 概念组成

`S(t)` 是本步所有模块的共同只读输入。它至少要包含以下概念状态。

### 3.1 仿真时间状态

```text
t:
    当前真实仿真时间。

step:
    当前离散步编号。

dt:
    时间步长。
```

`APS_due` 必须基于真实时间或等价 step-time 映射判断，不能把 `time + 1` 当作真实时间推进语义。

### 3.2 车辆当前状态

对每辆 active vehicle，`S(t)` 至少要能提供：

```text
vehicle id
vehicle type
CHV compliance state
vehicle size / length
current x_global
current y
current v
current a
physical lane
mainline / on-ramp role
是否已驶离
```

其中：

```text
x_global / y:
    是算法内部状态坐标。

x_plot:
    只用于绘图和论文图坐标对齐，不进入 `S(t)` 算法状态。
```

### 3.3 车辆属性与随机属性

`S(t)` 中还需要包含不会每步重采样、但会被车辆模型和策略模块反复读取的车辆属性，例如：

```text
CHV desired speed
CAV inertial lag
vehicle desired time gap class
arrival headway assignment
其他来自车辆生成阶段的持久随机属性
```

这些内容属于跨步持久状态，不属于本步临时派生量。

### 3.4 车道、区域与身份状态

`S(t)` 至少需要让后续模块判断：

```text
车辆当前 physical lane
车辆当前是否位于 on-ramp / lane 2 / lane 1
车辆是 on-ramp MV 还是 mainline vehicle
车辆是否已进入 merging zone
车辆是否已驶出仿真路段
```

区域判断使用道路几何规格中的 `x_global`、`x0_m_global`、`x_ramp_end_global` 等定义，不在本文档重复给数值。

### 3.5 状态机状态

`S(t)` 至少需要持久化以下状态机概念：

```text
lane_change_state
merge_state
```

其中：

```text
lane_change_state:
    用于表达主线 CV 是否处于 active lane change。第一版持久语义只需要区分 normal / executing；完成换道作为 commit transition result，不作为长期持久态。

merge_state:
    用于表达车辆是否需要 CMC 管理，以及 MV 是否处于 not_started / waiting / executing / merged 等阶段。非 MV 或已压缩为普通 mainline vehicle 的车辆使用 none 语义。
```

它们是跨步持久状态，不是本步瞬时结果。

### 3.6 持久缓存

`S(t)` 至少要持有以下跨时间步持久缓存：

```text
APS assignment cache
active maneuver trajectory state
车辆历史必要状态
```

`active maneuver trajectory state` 是 active lane change / active merge 能跨步继续执行的关键状态。它至少需要表达以下概念：

```text
maneuver_type
start_time 或 start_step
start_x
start_y
target_y
target_lane
planned / current progress
source command
```

其中 `source command` 只用于说明该 maneuver 来自 CUC lane-change 还是 CMC merge，不应被下一步用来重新决策。下一步继续执行 maneuver 的依据是 `lane_change_state` / `merge_state` 与 trajectory state，而不是历史 CUC choice 或历史 CMC decision。

“车辆历史必要状态”在概念层只表示那些会被下一步模型继续消费的最小历史信息，例如上一时刻速度、上一时刻加速度、active maneuver 的起点信息等。具体字段名和组织方式留给代码数据结构设计。

### 3.7 配置引用

`S(t)` 的读取环境中还需要有只读配置引用：

```text
参数规格引用
道路几何规格引用
功能开关引用
```

这些内容可以不并入车辆对象本身，但必须作为模块的只读输入可达。

### 3.8 持久状态与本步派生状态的边界

本节给出第一版硬边界：

```text
持久状态：
    vehicle current state
    vehicle attributes
    lane / role state
    lane_change_state
    merge_state
    APS assignment cache
    active maneuver trajectory state
    历史必要状态
    配置引用

本步派生状态：
    relations snapshot
    same_step_maneuver_relation_overlay
    effective_assignment_this_step
    active cooperative request
    conflict request
    CUC choice
    CMC decision
    lane_change_completed / merge_completed transition result
    candidate speed / acceleration / y
    front-collision fallback
    boundary speed cap 的本步计算结果
```

其中某些本步派生结果在 commit 后可能被“状态化”并进入 `S(t+dt)`，例如更新后的 assignment cache、state transition、生效中的 maneuver progress。是否状态化取决于它是否需要被下一时间步继续消费。

`CUC choice` 默认是本步 command / event，不作为下一步重新驱动车辆行为的真实状态。若车辆已进入 active lane change，下一步继续换道的依据是 `lane_change_state == executing` 与 active maneuver trajectory state，而不是上一轮 `CUC choice`。

## 4. Command 与 Next-state 边界

第一版必须在概念上区分 command buffer 和 next-state buffer。

### 4.1 Command 的概念

command 表示“本步模块意图”或“对后续计算的约束”。它至少包括以下概念类型：

```text
longitudinal command
cooperation command
lane-change command
merge command
speed cap
desired spacing override
state transition request
```

其语义如下：

```text
longitudinal command:
    表示车辆本步应采用哪类纵向控制语义或控制目标。

cooperation command:
    表示 APS / CUC / CMC 派生出的协同意图，不直接改位置。

lane-change command:
    表示车辆进入或继续 lane 2 -> lane 1 换道。

merge command:
    表示 MV 进入或继续 on-ramp -> lane 2 合流。

speed cap:
    表示对 planning speed 的上界约束，例如 boundary speed cap。

desired spacing override:
    表示用协同期望间距覆盖普通 desired spacing 语义，例如 case 2 / 4 中 CFV 的 Eq.10。

state transition request:
    表示模块建议状态机发生变化，但不在模块内直接修改真实状态。
```

### 4.2 Next-state 的概念

next-state buffer 表示本步基于 `S(t)` 计算出的候选下一状态，例如：

```text
candidate x_global
candidate y
candidate v
candidate a
candidate physical lane
candidate role update
candidate lane_change_state
candidate merge_state
candidate cache update
```

这些都是本步候选结果。它们在 commit 前不能反向成为其他车辆的 `S(t)` 输入。

### 4.3 Command 与 Next-state 的职责边界

第一版接口必须明确以下边界：

```text
command 不直接改车辆真实状态。
next-state 不直接成为本步其他模块的输入状态。
real state 只在 commit 后更新。
```

例如：

```text
CUC choice:
    是 command / maneuver choice，不是位置更新。

APS assignment:
    是 cache 更新，不是车辆位置更新。

boundary speed cap:
    是速度约束 command，本步由纵向与横向模块消费。

lane update:
    应在 commit 阶段正式写入 `S(t+dt)`。
```

### 4.4 多 command 的合成原则

若多个模块对同一车辆写入 command，状态接口需要先定义概念级优先级，再交给代码设计实现。第一版口径如下：

```text
1. active maneuver 相关 command 优先于普通纵向 command。
2. CMC / merge executing 相关约束优先于普通 on-ramp 纵向意图。
3. speed cap 合成取最保守 planning speed。
4. 同一属性在 next-state 中只允许保留一个最终候选值。
```

这里的“优先”是概念层语义，不代表已经规定具体字段覆盖顺序。最终代码如何表达覆盖或合成，由代码数据结构设计文档定义。

### 4.5 Boundary Generation 的 Pre-freeze 边界

Boundary generation 发生在 Step 2 冻结 `S(t)` 之前，因此它是本步 active vehicle set 的构造步骤，而不是冻结后普通模块写状态。

第一版口径如下：

```text
Boundary generation 是 pre-freeze population update。

它可以把满足入口条件的新车加入本步 active vehicle set，使其进入随后冻结的 S(t)。

冻结 S(t) 之后，不允许再插入新车影响本步 APS / CUC / CMC / 纵向 / 横向计算。

Boundary generation 不得修改已存在车辆的 x / y / v / a，也不得绕过 command / next-state / commit 机制更新既有车辆。
```

这一口径用于同时保留两条原则：边界生成必须先于冻结发生；冻结之后所有核心模块只读 `S(t)`。

## 5. Relations Snapshot

`relations snapshot` 是每步冻结 `S(t)` 后生成的派生快照，供 APS、CUC、CMC、纵向模型和横向模型读取。

### 5.1 Relations 必须表达的概念

```text
lane ordering
leader / follower
TLV / TFV / LV / FV
logical longitudinal role
active lane-change multi-relation
```

其中：

```text
lane ordering:
    每条 lane 内按 `x_global` 排序的车辆序列。

leader / follower:
    普通纵向跟驰链条中的前后车关系。

TLV / TFV / LV / FV:
    CUC 和 lane-changing subsystem 需要的目标车道 / 原车道邻接关系。

logical longitudinal role:
    正在换道或合流车辆在本步纵向关系中的角色。

active lane-change multi-relation:
    表达一个正在换道的 CV 同时与目标 lane 和原 lane 发生纵向关系。
```

### 5.2 排序与坐标口径

relations 的排序坐标必须写硬：

```text
使用 x_global 排序
不得使用 x_plot 排序
不得按 physical y 最近车道中心线切换排序口径
```

### 5.3 Active Lane-change 的关系语义

当 CV 处于 `lane_change_state == executing` 时，第一版 relations 至少要能表达：

```text
CV -> TLV:
    CV 以目标车道前车 TLV 为主 leader。

TFV -> CV:
    目标车道后车 TFV 以正在换道的 CV 为 leader。

FV -> CV:
    原车道后车 FV 在 CV 完成换道前仍以 CV 为 leader。
```

因此：

```text
CV 只有一个主 leader，即 TLV。
CV 可以同时作为 TFV 和 FV 的 leader。
lane 归属在 commit 完成前不正式更新。
```

### 5.4 不按 `physical y` 连续切换关系

这是第一版状态接口的硬约束：

```text
正在换道或合流的车辆可能物理上位于两条车道之间。
它的 physical y 只用于横向轨迹、绘图和碰撞检查。
它的 longitudinal relation 由状态机和 relations snapshot 决定。
不得仅根据 y 更接近哪条 lane centerline 来连续切换 leader / follower。
```

### 5.5 Relations 的生命周期

`relations snapshot` 的生命周期是“每步生成、每步消费、下一步重建”：

```text
Step 2:
    冻结 `S(t)`

Step 3:
    基于 `S(t)` 生成 relations snapshot

Step 4-8:
    后续模块只读这份 relations snapshot

Step 9:
    commit 生成 `S(t+dt)`

next step:
    基于新的 `S(t+dt)` 重新生成 relations snapshot
```

relations 不是跨步持久缓存；它是每步派生快照。

### 5.6 Same-step Maneuver Relation Overlay

Step 3 的 relations snapshot 基于冻结 `S(t)` 生成。若某辆 CV 在 Step 6 通过 CUC 本步新启动 `lane 2 -> lane 1` 换道，那么 Step 7 纵向模型需要使用换道语义中的 TLV / TFV / FV，而不能把该车继续当作完全普通的 lane 2 车辆处理。

因此，状态接口需要允许一个本步派生的关系覆盖概念：

```text
same_step_maneuver_relation_overlay:
    当 CUC 本步新启动 lane 2 -> lane 1 换道时，
    lane-change command 必须携带或引用本步 relations snapshot 中的 TLV / TFV / FV。
```

使用规则：

```text
1. overlay 是本步派生关系，不改变 physical lane。
2. 纵向模型可以基于 overlay 计算本步 candidate acceleration / speed。
3. commit 后若换道仍在执行，下一步 relations refresh 根据新的 state machine 与 trajectory state 重新派生关系。
```

该 overlay 是第一版工程实现约束，用于让同一步新启动的 maneuver 能被纵向模型正确消费；它不是论文额外定义的换道决策算法。

## 6. APS Assignment Cache 与协同请求

第一版要求每个 MV 持有一份跨步持久的 APS assignment cache。

### 6.1 Cache 生命周期

`APS assignment cache` 的生命周期如下：

```text
MV 首次进入 APS 适用阶段：
    必须执行 first_APS(MV)，生成 assignment 或显式标记失败。

APS 周期到达：
    更新 assignment cache。

非 APS 周期：
    沿用当前 assignment cache。

assignment invalid：
    标记 invalid，并按第一版工程策略处理。

MV merged：
    清理该 MV 的 APS assignment cache。
```

`first_APS(MV)` 是第一版工程调度规则，不是论文原公式。它的存在目的是避免 MV 首次进入 APS 适用阶段时没有 cache 可沿用。

### 6.2 Cache 至少要表达的概念

每个 MV 的 assignment cache 至少应表达：

```text
CLV
CFV
APS case
col_CLV / col_CFV
Eq.10 desired spacing 语义
T*_MV
assignment valid / invalid 状态
last update time
```

其中：

```text
Eq.10 desired spacing:
    这里只要求 cache 能表达“case 2 / 4 中 CFV 需要的协同期望间距语义”。
    具体如何落为代码字段，由后续数据结构设计决定。
```

### 6.3 Valid / Invalid 的接口语义

`assignment valid / invalid` 是第一版工程安全状态，不是论文定义的“实时替换 actual leader/follower”机制。

接口上至少需要支持以下判断：

```text
assigned CLV / CFV 是否仍在 lane 2
assigned CLV / CFV 是否已驶离
assigned CLV / CFV 是否仍能形成目标协同 gap
```

若 invalid：

```text
MV 本步暂不开始合流
MV 可进入 waiting 或保守安全处理
等待下一次 APS 更新
```

本文档只定义接口语义，不定义 invalid 后的具体减速策略。

第一版允许保留以下工程策略入口：

```text
assignment_invalid_policy:
    wait_until_next_APS
    immediate_APS_refresh
    conservative_wait
```

语义如下：

```text
wait_until_next_APS:
    标记 invalid，等待下一次 APS 周期刷新。

immediate_APS_refresh:
    允许本步或下一步立即触发 APS 刷新，以重新寻找 CLV / CFV。
    该策略是第一版工程补丁，不是论文原生算法。

conservative_wait:
    MV 不开始合流，沿 on-ramp 纵向行驶，并受 boundary speed cap 或保守速度约束。
```

`immediate_APS_refresh` 不等于每步用 actual leader / follower 替代 APS assignment。它仍必须通过 APS 语义重新生成 assignment。

### 6.4 Effective Assignment This Step

APS 本步可能更新 assignment，也可能沿用上一轮 cache。为了让 Step 5 汇总协同请求时既能消费本步 APS 输出，又不破坏真实状态只在 commit 后更新的原则，第一版引入以下概念：

```text
effective_assignment_this_step:
    if APS 本步更新成功:
        使用 APS 本步输出的 assignment update
    else:
        使用 S(t) 中已有 assignment cache
```

使用规则：

```text
1. Step 4 处理 MV 时生成 effective_assignment_this_step。
2. Step 5 汇总协同请求时读取 effective_assignment_this_step。
3. commit 阶段再把需要跨步保留的 assignment update 写入 S(t+dt)。
```

`effective_assignment_this_step` 是本步派生结果，不是跨步持久 cache。跨步持久化仍由 APS assignment cache 承担。

### 6.5 Active Cooperative Request 与 Conflict Request

Step 5 需要从所有有效 `effective_assignment_this_step` 中汇总出本步协同请求。状态接口需要能表达：

```text
active cooperative request
conflict request
winner / loser 概念结果
```

建议的概念边界是：

```text
active cooperative request:
    本步经仲裁后，真正交给 CUC 消费的协同请求。

conflict request:
    因多 MV 共享 CV 冲突而未被激活的请求。
```

### 6.6 多 MV 共享 CV 仲裁

多 MV 共享 CV 的处理是第一版工程补丁，不是论文原生多 MV 分配算法。状态接口至少要允许仲裁模块表达以下结果：

```text
同一 CV 被哪些 MV 请求
本步哪一个 MV 获得该 CV
未获得该 CV 的 MV 被标为 waiting / conflict
```

默认优先级沿用时间步总纲：

```text
已在 merging zone 的 MV 优先
其后 T*_MV 更小的 MV 优先
其后离 x0^m 更近的 MV 优先
```

## 7. CUC Choice 与 Lane-change State

### 7.1 CUC Choice 的接口语义

CUC 只对“经仲裁后 active cooperative request 中 `col = 1` 的 CV”执行。状态接口至少要能表达：

```text
该车本步是否是 active CV
该车本步是否执行 CUC
该车本步 CUC choice 是 lane 1 还是留 lane 2
该 choice 是否已被目标车道安全约束回退
```

CUC choice 的本质是 maneuver choice：

```text
它不直接更新车辆位置
它必须能被后续纵向模型和横向轨迹模块读取
它可能触发 lane-change command
它也可能只生成 cooperation / spacing override 语义
```

CUC choice 的持久化边界如下：

```text
CUC choice 默认是本步 command / event。
它不作为下一步重新决策的必要真实状态。
```

只有以下情况需要保留历史记录：

```text
1. 为日志、调试、轨迹图标注记录历史 CUC choice。
2. 为 active lane-change trajectory 记录 maneuver source。
3. 为输出指标统计协同事件。
```

下一步若车辆已经处于 `lane_change_state == executing`，应由 `lane_change_state` 和 active maneuver trajectory state 决定继续换道，而不是由历史 CUC choice 决定。

### 7.2 `lane_change_state` 的概念状态

`lane_change_state` 作为跨步持久状态，第一版只需要表达以下概念：

```text
normal
executing
```

完成换道不作为长期持久态，而作为 commit 阶段产生的 transition result：

```text
lane_change_completed:
    本步 commit event / transition result，不是下一步继续持有的 lane_change_state。
```

本文档不强制最终代码枚举名，但要求代码层不要让 `completed` 与 `normal` 长期并存造成状态职责重叠。

### 7.3 Active Lane-change 时不重选

当：

```text
lane_change_state == executing
```

则本步接口语义必须保证：

```text
不重新执行 CUC
不重新选择 maneuver target
继续既有 lane-change trajectory
仍参与本步纵向动力学计算
仍参与本步横向轨迹更新
```

这条规则是第一版状态机硬约束。

### 7.4 Lane-change Command 与提交语义

若本步 CUC 选择 `lane 2 -> lane 1` 且目标车道安全约束满足，则：

```text
写入 lane-change command
写入 lane_change_state transition request
初始化或继续 maneuver trajectory state
```

但：

```text
physical lane 不在 CUC 模块内直接修改
lane 归属只在 commit 阶段正式更新
```

完成换道后的提交语义是：

```text
physical lane 变为 lane 1
lane_change_state 回到 normal
lane_change_completed 可作为本步 transition result / event 被信息集成阶段记录
原 lane 2 的 FV 在下一次关系刷新中连接到新 leader
```

## 8. CMC State 与 Merge State

### 8.1 `merge_state` 的概念状态

`merge_state` 至少要能表达以下概念：

```text
none
not_started
waiting
executing
merged
```

### 8.2 各状态的接口语义

```text
none:
    车辆不是 on-ramp MV，或已经被压缩为普通 mainline vehicle 后不再需要 merge_state 管理。

not_started:
    MV 已进入需要 CMC 管理的阶段，但尚未开始横向合流。

waiting:
    MV 本步不进行横向合流，但继续沿 on-ramp 纵向行驶。

executing:
    MV 已开始横向合流，继续执行既有 merge trajectory。

merged:
    MV 已到达 lane 2 centerline，commit 后转为 mainline vehicle。
```

其中 waiting 必须明确：

```text
waiting 不是停车
waiting == no lateral merge this step + longitudinal movement continues
```

### 8.3 Executing 后不撤销

一旦：

```text
merge_state == executing
```

则状态接口必须保证：

```text
本步继续已有合流轨迹
不重新判断“是否开始合流”
不因短时 gap 变化退回 waiting
assignment valid 检查不再用于撤销本次合流
除非发生实现层面的硬异常，否则继续既有 trajectory
```

### 8.4 Boundary Speed Cap、Assignment Valid 与 Merge Command

CMC 模块至少需要向后续模块提供：

```text
assignment valid / invalid 结果
merge decision
boundary speed cap
merge command
```

这些概念的接口关系是：

```text
assignment valid / invalid:
    影响本步能否从 waiting / not_started 进入 executing。

merge command:
    表示本步开始或继续合流。

boundary speed cap:
    作为 speed cap command 流向纵向模型。
    经纵向模块消费后形成最终 planning speed。
    横向轨迹模块消费该已约束后的 planning speed。
```

### 8.5 Merged 后的提交语义

当 MV 到达 lane 2 centerline 后，commit 阶段至少要能状态化以下结果：

```text
physical lane 变为 lane 2
身份转为 mainline vehicle
merge_state 变为 merged
清理该 MV 的 APS assignment cache
```

若代码层不希望长期保留 `merged`，可以采用以下接口语义：

```text
merged 是 commit transition result；
下一步状态中该车 merge_state 可转为 none。
```

本文档不再使用 `merged / normal` 混合表达。`normal` 属于 `lane_change_state` 语义，不属于 `merge_state`。

## 9. 模块输入输出边界

本节按模块给出概念级接口边界。除 commit 外，任何模块都不得直接写真实 `x / y / v / a`。

| 模块 | 读取内容 | 写入内容 | 禁止行为 | 后续展开 spec |
| --- | --- | --- | --- | --- |
| Boundary generation | boundary queues、入口安全间隙、arrival headway、道路边界 | pre-freeze new vehicle candidates、active vehicles 增量概念 | 冻结 `S(t)` 后不得再插入新车；不得修改已存在车辆 `x / y / v / a` | 代码数据结构设计、最小验证场景规格 |
| Relation refresh | `S(t)`、`x_global`、physical lane、状态机、maneuver state | relations snapshot | 不得按 `x_plot` 排序；不得按 `y` 最近 centerline 连续切换关系 | 代码数据结构设计 |
| APS | `S(t)`、relations、道路几何、参数、MV 当前 cache | effective_assignment_this_step、assignment cache update request、APS case、`col`、Eq.10 语义、APS failure / invalid 概念 | 不得写车辆位置；不得把 actual leader/follower 实时替换成 assignment invalid 兜底 | 代码数据结构设计、最小验证场景规格 |
| Request arbitration | effective_assignment_this_step、MV priority 依据 | active cooperative request、conflict request、winner / loser 结果 | 不得写车辆位置；不得把工程仲裁写成论文原生多 MV 分配算法 | 输出指标与日志验证规格、代码数据结构设计 |
| CUC | active request、relations、车辆状态、compliance、参数 | CUC choice、lane-change command、same_step_maneuver_relation_overlay、cooperation command、state transition request | 不得直接改位置；active lane change 时不得重选 maneuver | 车辆模型规格、代码数据结构设计 |
| CMC | MV state、assignment cache、道路几何、参数、relations | merge decision、boundary speed cap、merge command、merge_state transition request | 不得直接改位置；不得在模块内部正式改 lane | 车辆模型规格、代码数据结构设计 |
| Longitudinal model | `S(t)`、relations、vehicle type、commands、desired spacing override、speed cap | candidate acceleration、candidate speed、speed-constrained planning result | 不得直接提交真实 `v / a / x`；不得把 candidate 反写回 `S(t)` | 车辆模型规格、代码数据结构设计 |
| Lateral trajectory | active maneuver state、planning speed、target centerline、front-collision fallback | candidate y、maneuver progress、continue / delay maneuver result | 不得正式改 lane；不得单独提交真实状态 | 车辆模型规格、代码数据结构设计 |
| Commit | `S(t)`、all next-state candidates、state transition requests、cache updates | 唯一真实 `S(t+dt)` | 不得遗漏单车冲突合成；不得允许一车多次提交 | 代码数据结构设计、执行计划 |
| Information integration | `S(t+dt)`、本步 command / candidate / result | trajectory history、event concept、sanity check result | 不得重新改写已提交的车辆运动状态 | 输出指标与日志验证规格 |

### 9.1 Commit 是唯一真实写入点

状态接口必须把这一点写硬：

```text
只有 commit 阶段能生成 `S(t+dt)`。
只有 commit 阶段能正式写入车辆下一状态。
只有 commit 阶段能正式更新 lane 归属和主线 / 匝道身份。
```

### 9.2 信息集成不是状态回写

Step 10 的 information integration 只负责：

```text
记录轨迹
记录事件概念
记录 sanity check 结果
```

它不应反向改变本步已提交的真实车辆运动状态。

## 10. 后续规格承接

本文档之后的职责分工如下：

```text
CORMC代码数据结构设计.md：
    负责把本文档中的概念状态、buffer、cache、state machine 和模块接口落成 dataclass、enum、config 与字段名。

CORMC输出指标与日志验证规格.md：
    负责 event record、assignment invalid reason、collision / boundary violation、sanity check 和调试日志字段。

CORMC最小验证场景规格.md：
    负责具体车辆初始 `x / y / v / type / compliance`，以及预期状态流转。

执行计划文档：
    负责实现顺序、阶段验收和测试任务拆分。
```

本节同时给出承接约束：

```text
1. 后续数据结构设计不得反向改变本文档的状态读写原则。
2. 后续日志规格不得把日志字段反向提升为算法必要状态，除非先修订本文档。
3. 后续最小场景规格只能消费本文档已定义的状态机与接口语义，不能私自改主循环。
4. 若后续实现发现缺少必要概念，应先修订本文档，再进入代码实现。
```

## 11. 验收检查

本文档应满足以下检查：

```text
1. 后续实现者能明确哪些状态必须在 `S(t)` 中可读。
2. 后续实现者能明确哪些结果只能写入 command / next-state。
3. 后续实现者能明确 commit 是唯一生成 `S(t+dt)` 的阶段。
4. 后续实现者能明确 APS assignment cache 何时更新、沿用、invalid、清理。
5. 后续实现者能明确 CUC choice 如何进入 lane-change command。
6. 后续实现者能明确 CMC boundary speed cap 如何流向纵向和横向模块。
7. 后续实现者能明确 active lane-change 车辆如何参与 TLV / TFV / FV 关系。
8. 后续实现者能明确 `merge_state == executing` 后不重新判断开始合流。
9. 后续实现者能明确 `x_plot` 不进入算法状态，只用于输出。
10. 文档没有设计 Python dataclass。
11. 文档没有重复车辆模型公式。
12. 文档没有定义日志字段全集。
13. 文档没有设定 smoke scenario 初始车辆。
14. 文档没有改变已有时间步主循环。
15. `merge_state` 枚举没有 `merged / normal` 混用。
16. `lane_change_state` 没有把 completed 作为长期持久状态。
17. CUC choice 默认是本步 command / event，不作为下一步控制状态。
18. Boundary generation 被明确为 pre-freeze population update。
19. APS 本步输出通过 effective_assignment_this_step 被 Step 5 消费。
20. assignment invalid 后是否允许 immediate_APS_refresh 有明确接口口径。
21. 新启动 lane change 时，纵向模型能通过 same_step_maneuver_relation_overlay 读取 TLV / TFV / FV。
22. active maneuver trajectory state 的最小语义已定义。
23. 工程补丁与论文原机制保持区分。
```
