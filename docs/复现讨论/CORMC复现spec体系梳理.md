# CORMC 复现 spec 体系梳理

本文档用于整理 CORMC 第一版复现后续需要产出的 spec 文档体系。它不是具体算法 spec、不是公式详解、不是参数表、不是数据结构字段全集，也不是实现计划。

当前上游总纲是：

```text
docs/复现讨论/CORMC时间步执行顺序梳理.md
```

后续所有 spec 都应服务于该算法流程总纲。算法总纲回答“每个时间步什么时候做什么”；下游 spec 的任务是补齐该流程实现所需的依据和边界。并记住"D:\PycharmProjects\CORMC\docs\复现讨论\CORMC复现讨论对齐记录.md"的思想。

本文档采用的拆分方法是：

```text
从 CORMC 时间步主循环 Step 0-13 出发，
沿着一次仿真时间步模拟跑一遍算法，
逐步识别每一步需要什么定义、公式、参数、道路几何、车辆模型、状态接口、日志、代码结构或验证场景，
再把这些需求分配到对应 spec 文档。
```

因此，后续 spec 不是独立发散的资料堆叠，而是围绕算法流程逐层补齐实现依据。

## 1. 拆分原则

第一版 spec 拆分遵守以下原则：

```text
1. 流程驱动：
   所有 spec 都从 CORMC 时间步流程出发，服务于 APS、CUC、CMC、纵向动力学、横向轨迹和状态提交的实现。

2. 公式映射不是公式清单：
   公式映射应记录“流程中的计算/判断点”与“论文公式、Algorithm 1、Fig. 9、正文说明”的对应关系。

3. 道路几何与最小验证场景分开：
   道路几何中论文已明确的内容可以先按论文落地；
   最小验证场景必须等公式、参数、模型、日志和代码配置形态确定后，再用数值反推。

4. 参数规格只管理数值和来源：
   参数规格不解释公式推导，不设计模块接口，也不决定小场景车辆初始状态。

5. 车辆模型规格回答“怎么算”：
   它引用公式和参数，说明纵向动力学、横向轨迹和避碰约束如何落地，但不重复参数表。

6. 状态与模块接口规格先做概念契约：
   它定义 S(t)、command、next-state、relations、assignment cache、state machine 和模块输入输出。

7. 代码数据结构设计再做代码形态：
   它把概念契约和日志需求落成 dataclass / enum / buffer / config / event record 等候选结构。

8. 输出指标与日志验证优先服务调试：
   第一版先保证能定位 APS/CUC/CMC/状态转移哪里算错，再扩展到论文级平均速度、流量、延误等指标。

9. 执行计划不做成一个巨型文档：
   等 spec 基本稳定后，按阶段拆成多个可验收 plan；执行计划不应首次决定核心数据结构。
```

## 2. 建议 spec 文档

### 2.1 论文公式与实现映射

建议文件：

```text
docs/复现讨论/CORMC论文公式与实现映射.md
```

定位：

该文档不是简单罗列论文公式。它应从 `CORMC时间步执行顺序梳理.md` 的主循环出发，模拟跑一遍算法，识别每一步需要定义、计算、判断或约束的量，再回到论文查找对应公式、Algorithm 1、Fig. 9 或正文说明。

建议输出形态：

```text
流程步骤 -> 需要定义/计算/判断的量 -> 论文依据 -> 公式状态 -> 后续展开 spec
```

职责：

- 建立时间步流程中的计算/判断点与论文依据之间的映射。
- 说明 Algorithm 1 主要服务 APS 内部计算，Fig. 9 主要服务主循环调度语义核对。
- 将公式槽位按 APS、CUC、CAV 纵向模型、CHV/IDM、lane-changing、CMC、vehicle generation、输出指标等模块归类。
- 标明公式状态：

```text
论文原公式：
    论文明确给出，第一版应尽量按原公式实现。

第一版简化：
    论文有完整机制，但第一版有意简化，例如不做 MPC tracking。

第一版关闭：
    论文有机制，但第一版暂不做，例如普通主线主动换道、CMC platoon。

工程补丁：
    论文未完整定义，但第一版实现必须补齐，例如 assignment invalid、多 MV 共享 CV 仲裁。
```

- 标明每个公式或规则后续由哪个 spec 详细展开。

不负责：

- 不设计数据结构。
- 不写 Python 接口。
- 不填参数表。
- 不选择最小验证场景的具体数值。

需要特别标注：

```text
first_APS(MV) 不是论文公式，而是第一版工程调度规则。
它对应论文中每个 MV 的 APS initial decision time 与 T_APS 周期更新语义，
用于避免首次进入 APS 适用阶段时没有 assignment cache。
```

### 2.2 道路几何与区域规格

建议文件：

```text
docs/复现讨论/CORMC道路几何与区域规格.md
```

定位：

该文档给时间步流程提供空间坐标底座。它应服务于边界车辆生成、APS 候选范围、merging zone / cooperative zone 判断、CMC 边界防撞、横向轨迹目标 centerline 等流程需求。

职责：

- 定义 x / y 坐标方向。
- 定义 lane 1、lane 2、on-ramp 的 centerline。
- 定义主线长度、on-ramp 位置、merging zone、cooperative zone、`x0^m`、`x_ramp_end`。
- 定义入口边界、出口边界和车辆是否驶离仿真路段的判定口径。
- 明确道路几何中哪些内容来自论文，哪些属于第一版工程默认。

不负责：

- 不设计 Scenario A/B/C 的车辆初始坐标。
- 不展开车辆模型公式。
- 不管理 IDM、CUC、CMC 等模型参数表。
- 不定义 Python 数据结构。

说明：

道路几何中论文已经明确的部分可以先按论文落地；最小验证场景需要等公式、参数、模型、日志和代码配置形态确认后再单独推算。

### 2.3 参数规格

建议文件：

```text
docs/复现讨论/CORMC参数规格.md
```

定位：

该文档给公式、车辆模型、道路几何、场景生成、代码配置和验证提供统一数值来源，避免同一参数在不同文档或代码中重复定义。

职责：

- 汇总全局参数、道路参数、车辆参数、APS 参数、CUC 参数、CMC 参数、CAV 参数、IDM 参数、lane-changing 参数、vehicle generation 参数。
- 为每个参数标注来源：

```text
paper:
    论文明确给出，第一版直接采用。

paper-derived:
    论文给出关系式，需要由其他参数计算得到。

first-version-default:
    论文未明确，但第一版为了运行必须设定。

to-review:
    暂时不能确认，后续需要审阅。
```

- 明确哪些参数供公式映射使用，哪些参数供车辆模型使用，哪些参数供道路几何、代码数据结构和最小验证场景引用。

不负责：

- 不解释公式推导。
- 不设计小场景车辆初始状态。
- 不定义模块数据结构。
- 不决定日志字段。

### 2.4 车辆模型规格

建议文件：

```text
docs/复现讨论/CORMC车辆模型规格.md
```

定位：

该文档是车辆运动与控制量计算规格，重点回答时间步中的纵向动力学和横向运动怎么算。它从公式映射取论文依据，从参数规格取数值，从时间步总纲取执行位置和先后关系。

职责：

- 说明 CAV cruising 纵向模型。
- 说明 CAV gap-regulating / CPID 纵向模型。
- 说明 CHV / IDM 模型。
- 说明 compliance 如何影响 CHV 是否接受 CUC 建议。
- 说明 on-ramp MV 在 not_started、waiting、executing、merged 状态下的纵向行为。
- 说明 CUC lane 2 -> lane 1 正弦换道轨迹。
- 说明 CMC on-ramp -> lane 2 正弦合流轨迹。
- 说明 front-collision-avoidance 的第一版落地口径。
- 说明 boundary-collision-avoidance 如何在 CMC 中产出 boundary speed cap。
- 说明纵向计算、speed cap、planning speed、横向轨迹之间的先后关系。

不负责：

- 不决定道路长度或 lane centerline。
- 不决定 Scenario A/B/C 的车辆初始布置。
- 不定义 Python dataclass、enum 或 buffer。
- 不定义日志格式。
- 不重复参数表。

依赖：

```text
CORMC论文公式与实现映射.md
CORMC参数规格.md
CORMC时间步执行顺序梳理.md
```

说明：

车辆模型规格可以说明模型如何消费参数，但具体数值仍以 `CORMC参数规格.md` 为准。

### 2.5 状态与模块接口规格

建议文件：

```text
docs/复现讨论/CORMC状态与模块接口规格.md
```

定位：

该文档是最直接受时间步总纲约束的概念级接口 spec。它从 Step 2 冻结 `S(t)`、Step 9 同步提交 `S(t+dt)`，以及 APS / CUC / CMC / 纵向 / 横向模块读写关系出发，定义模块之间需要什么概念。

职责：

- 定义 `S(t)` 至少包含哪些概念状态。
- 定义 `S(t+dt)` 同步提交哪些内容。
- 定义 command buffer 与 next-state buffer 的职责边界。
- 定义 relations：lane ordering、leader/follower、TLV/TFV/LV/FV、logical longitudinal role。
- 定义 APS assignment cache、CUC choice、lane_change_state、merge_state 的模块级语义。
- 定义 APS、CUC、CMC、纵向动力学、横向轨迹、状态提交、信息集成的输入输出边界。
- 明确每个模块只读 `S(t)`、只写本步 command / next-state 的约束。

不负责：

- 不写具体 Python dataclass。
- 不决定文件存储格式。
- 不重写算法时间步顺序。
- 不决定日志展示方式。

说明：

该文档负责“模块之间需要什么概念”；`CORMC代码数据结构设计.md` 负责“这些概念在代码中如何落成类型和字段”。

### 2.6 输出指标与日志验证规格

建议文件：

```text
docs/复现讨论/CORMC输出指标与日志验证规格.md
```

定位：

该文档用于让实现可调试、可验收，而不是只生成最终论文指标。第一版应优先服务 smoke scenario 的链路检查，论文级宏观指标后续扩展。

职责：

- 定义轨迹图 PNG 的最低输出内容。
- 定义 trajectory history、event log、APS event、CUC event、CMC event、merge event、lane-change event。
- 定义时间步流程中关键决策点的日志需求，例如：

```text
APS case
CLV / CFV
col_CLV / col_CFV
CUC choice
Eq.53 gap 判断结果
Eq.56 boundary speed cap
lane_change_state transition
merge_state transition
assignment invalid reason
multi-MV conflict winner / loser
```

- 定义 collision、near-collision、boundary violation、越界等 sanity check。
- 定义第一版 smoke scenario 的验证口径。
- 预留论文级平均速度、流量、延误、合流成功率等指标。

不负责：

- 不定义车辆模型公式。
- 不决定道路或参数数值。
- 不改变算法主循环。
- 不设计完整 Python 数据结构。

说明：

日志验证规格决定需要记录哪些事件和中间量；`CORMC代码数据结构设计.md` 负责给这些日志和记录定义代码级承载形态。

### 2.7 代码数据结构设计

建议文件：

```text
docs/复现讨论/CORMC代码数据结构设计.md
```

定位：

该文档把 `CORMC状态与模块接口规格.md` 的概念契约、`CORMC输出指标与日志验证规格.md` 的记录需求、`CORMC参数规格.md` 的配置需求，转换成第一版代码可实现的数据结构设计。

它不应早于状态接口和日志验证，因为它需要知道模块读写契约和调试验收需要记录什么；也不应晚到执行计划阶段才首次出现，否则执行计划会被迫决定核心数据形态。

职责：

- 定义代码级候选结构，例如：

```text
VehicleState
VehicleSpec
SimulationState
Relations
APSAssignment
CommandBuffer
NextStateBuffer
EventRecord
TrajectoryRecord
ScenarioConfig
ParameterConfig
```

- 定义必要 enum / state machine 名称，例如：

```text
vehicle type
compliance
physical lane
logical role
lane_change_state
merge_state
event type
```

- 明确哪些字段属于只读 `S(t)`，哪些属于本步 command，哪些属于 next-state，哪些属于日志和历史记录。
- 明确配置数据结构如何支撑后续最小验证场景可加载，而不是只写自然语言场景。

不负责：

- 不重新定义算法流程。
- 不重新推导公式。
- 不填具体参数值。
- 不实现 Python 代码。
- 不提前给 Scenario A/B/C 的具体初始车辆数值。

依赖：

```text
CORMC状态与模块接口规格.md
CORMC输出指标与日志验证规格.md
CORMC参数规格.md
CORMC道路几何与区域规格.md
CORMC车辆模型规格.md
```

### 2.8 最小验证场景规格

建议文件：

```text
docs/复现讨论/CORMC最小验证场景规格.md
```

定位：

该文档用于在公式、参数、道路、模型、状态接口、输出日志和代码数据结构设计基本确定后，反推可控小场景的车辆初始状态。

职责：

- 为每个场景定义车辆初始 x / y / v / type / compliance。
- 明确这些初始状态为什么会触发目标 APS case、CUC choice 或 CMC state transition。
- 明确预期 CLV / CFV、日志事件、轨迹图特征和 sanity check 结果。
- 明确最小场景最终应能落到 `ScenarioConfig` 或等价配置结构中，而不是只停留在自然语言描述。

建议保留三类最小场景目标：

```text
Scenario A:
    单 MV + lane 2 前后车
    验证 APS 找 CLV / CFV
    验证 CMC gap 满足后合流
    不要求 CUC 换道

Scenario B:
    单 MV + CFV 需要协同
    构造 APS case 2
    验证 CFV 留 lane 2 时 Eq.10 期望间距语义
    或验证 CUC 选择 lane 1 后换道

Scenario C:
    两个 MV 竞争同一 CV
    验证多 MV 共享 CV 工程仲裁
    验证未获得 CV 的 MV 进入 waiting/conflict 并等待 APS 更新
```

不负责：

- 不提前拍脑袋设定车辆初始坐标。
- 不替代完整论文随机实验矩阵。
- 不重新定义代码数据结构。

说明：

具体车辆初始 x / y / v / type / compliance 应通过 Eq.7、Eq.10、Eq.52、Eq.53 等公式和参数反推。该文档应后置，不能早于公式映射、参数规格、车辆模型规格、日志验证规格和代码数据结构设计。

## 3. 建议依赖顺序

推荐按以下顺序推进：

```text
1. CORMC论文公式与实现映射.md
2. CORMC道路几何与区域规格.md
3. CORMC参数规格.md
4. CORMC车辆模型规格.md
5. CORMC状态与模块接口规格.md
6. CORMC输出指标与日志验证规格.md
7. CORMC代码数据结构设计.md
8. CORMC最小验证场景规格.md
9. 多个执行计划文档
10. 代码实现
```

对应关系：

```text
算法总纲:
    什么时候做什么。

公式映射:
    沿时间步流程建立流程步骤到论文公式/依据的索引，避免自创公式或规则。

道路几何:
    给仿真提供空间坐标底座。

参数规格:
    给公式、模型、配置和验证提供统一数值来源。

车辆模型:
    说明车辆运动、控制量、轨迹和避碰约束怎么算。

状态接口:
    定义模块之间传什么、读什么、写什么、何时提交。

输出验证:
    定义记录什么、检查什么、如何定位错误。

代码数据结构:
    把接口、参数和日志需求落成代码级类型、字段、buffer 和 config 形态。

最小场景:
    用公式、参数、模型和配置结构反推可控验证场景。

执行计划:
    把 spec 转换成可验收的代码任务。
```

## 4. 执行计划的组织方式

不建议写一个巨型 `CORMC实现计划与验证点.md`。更建议在 spec 基本稳定后按阶段拆分，例如：

```text
docs/执行计划/P01-项目骨架与配置加载.md
docs/执行计划/P02-道路几何与车辆生成.md
docs/执行计划/P03-状态推进与同步提交.md
docs/执行计划/P04-纵向模型实现.md
docs/执行计划/P05-APS实现与验证.md
docs/执行计划/P06-CUC实现与验证.md
docs/执行计划/P07-CMC实现与验证.md
docs/执行计划/P08-横向轨迹与避碰.md
docs/执行计划/P09-日志输出与轨迹图.md
docs/执行计划/P10-最小验证场景跑通.md
```

每个执行计划都应引用相关 spec，并包含明确验证点。执行计划只在前置 spec 足够清楚后再写。

执行计划阶段不应首次决定核心数据结构。核心数据结构应先在 `CORMC代码数据结构设计.md` 中形成第一版设计，执行计划只能按照该设计实现，或在发现明确问题时做小范围修订。

## 5. 当前下一步

下一步建议先写：

```text
docs/复现讨论/CORMC论文公式与实现映射.md
```

原因：

```text
1. 它沿时间步流程建立“流程步骤到论文公式/依据”的索引。
2. 它能明确哪些内容是论文公式，哪些是第一版简化、第一版关闭或工程补丁。
3. 它会支撑道路几何、参数、车辆模型、状态接口、日志验证、代码数据结构和最小验证场景。
4. 它能降低后续凭直觉补公式、自创规则或临时拼数据结构的风险。
```
