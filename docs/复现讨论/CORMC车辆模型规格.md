# CORMC 车辆模型规格

本文档是 CORMC 第一版复现中“车辆运动与控制量怎么算”的规格文档。它承接时间步总纲、论文公式映射、道路几何和参数规格，重点固化纵向动力学、协同换道、MV 合流、避碰速度约束和第一版简化边界。

本文档不重复参数表，不设计 Python 数据结构，不写最小验证场景车辆初值，不实现完整 MPC tracking，也不重新定义时间步主循环。它只规定各类车辆在给定冻结状态 `S(t)`、relations、APS assignment、CUC choice、CMC state 和参数配置后，如何生成本步纵向 / 横向运动 command 或 next-state 候选。

本文档的上游依据是：

```text
docs/复现讨论/CORMC时间步执行顺序梳理.md
docs/复现讨论/CORMC论文公式与实现映射.md
docs/复现讨论/CORMC道路几何与区域规格.md
docs/复现讨论/CORMC参数规格.md
docs/papers/P002-2023-Hou-Cooperative-On-Ramp-Merging-Control.md
```

## 1. 文档定位

第一版车辆模型规格回答以下问题：

```text
1. CAV 什么时候 cruising，什么时候 gap-regulating。
2. CHV / IDM 如何更新纵向加速度。
3. CHV compliance 如何影响 CUC 建议的接受，而不是改变 CHV 纵向模型本质。
4. APS case 2 / 4 中 Eq.10 如何被 CFV 消费。
5. CUC choice 如何转化为 lane 2 -> lane 1 换道或 lane 2 留车道纵向协同。
6. MV 在 not_started、waiting、executing、merged 状态下如何纵向行驶和横向合流。
7. Eq.53 actual spacing 如何按中心点坐标和车辆长度计算。
8. front-collision-avoidance、boundary speed cap 和 planning speed 如何合成。
9. 第一版明确关闭或简化哪些论文机制。
```

本文档不负责：

```text
1. 不定义 dataclass、enum、command buffer 或 next-state buffer 字段。
2. 不重复 `CORMC参数规格.md` 中的参数数值。
3. 不设定 smoke scenario 中车辆初始 x / y / v / type / compliance。
4. 不定义日志字段和图表输出格式。
5. 不改变 `CORMC时间步执行顺序梳理.md` 中的主循环顺序。
```

## 2. 统一计算原则

每个时间步内，车辆模型遵守以下原则：

```text
1. 所有车辆模型只读冻结的 S(t)。
2. 车辆模型只生成本步 command / next-state 候选，不直接提交 S(t+dt)。
3. 纵向动力学先于横向轨迹计算。
4. speed cap 先作为纵向速度约束施加，再由横向轨迹消费最终 planning speed。
5. 每辆车每步只提交一次状态，提交动作由时间步总纲负责。
```

本步计算顺序应保持为：

```text
1. 根据 S(t)、relations、APS assignment、CUC choice、CMC state 确定：
   leader / follower
   desired spacing
   speed cap
   lane-change / merge target centerline

2. 计算 longitudinal candidate：
   candidate acceleration
   candidate speed
   optional speed cap constrained speed

3. 计算 lateral candidate：
   if lane_change_state == executing:
       沿既有 lane-changing trajectory 更新横向位置
   if merge_state == executing:
       沿既有 merging trajectory 更新横向位置

4. 输出 command / next-state 候选，等待同步提交。
```

正在换道或合流的车辆可能物理上处于两条 lane centerline 之间。它的 `physical y` 只用于横向轨迹、绘图和碰撞检查；它的 `logical longitudinal role` 由关系刷新和状态机决定，不得仅根据 `y` 更接近哪条 centerline 来连续切换 leader / follower。

## 3. CAV 纵向模型

CAV 纵向模型保留论文中的 cruising + gap-regulating 思路。

### 3.1 基础间距量

CAV 纵向模型使用论文 Eq.17-Eq.19：

```text
actual spacing:
    d_i(t) = x_{i-1}(t) - x_i(t) - L_{i-1}

desired spacing:
    S_i(t) = v_i(t) * h_i + d_0 + C_i(t)

collision avoidance spacing:
    if v_i(t) > v_{i-1}(t):
        C_i(t) = (v_i(t) - v_{i-1}(t))^2 / (2 * |a_min|)
    else:
        C_i(t) = 0
```

其中 `x` 使用道路几何规格中的 `x_global`。车辆长度、停止间距、期望时距和加速度上下界来自 `CORMC参数规格.md`。

### 3.2 Cruising 模式

CAV 在以下情况下使用 cruising 模式：

```text
1. 没有 leader。
2. 或实际 spacing 足够大：
       d_i(t) >= 2 * S_i(t)
```

cruising acceleration 使用论文 Eq.20：

```text
a_i(t) = k_1 * (v_e - v_i(t))
```

计算后需要按参数规格中的加速度和速度上下界裁剪。裁剪规则的具体字段由后续状态接口和代码数据结构规格定义。

### 3.3 Gap-regulating / CPID 模式

CAV 在有 leader 且需要跟驰时使用 gap-regulating 模式。第一版按论文 Eq.21-Eq.27 的 CPID 语义实现：

```text
spacing error:
    ex_i(t) = d_i(t) - S_i(t)

speed error:
    ev_i(t) = v_{i-1}(t) - v_i(t)

outer loop PID:
    partial_i(t) = K_px * ex_i(t)
                 + K_ix * integral(ex_i)
                 + K_dx * derivative(ex_i)

inner loop error:
    e_i(t) = partial_i(t) - ev_i(t)

inner loop PID:
    u_i(t) = K_pv * e_i(t)
           + K_iv * integral(e_i)
           + K_dv * derivative(e_i)

inertial lag update:
    a_i(t + dt) = (1 - dt / tau_i) * a_i(t)
                + (dt / tau_i) * u_i(t)
```

第一版 CPID 增益采用 `CORMC参数规格.md` 中的默认值，并保留 `first-version-default / to-review` 状态。不得把这些增益写成 CORMC 论文本篇 Table I 明确参数。

CPID 输出应进行以下约束：

```text
u_min <= u_i(t) <= u_max
a_min <= a_i(t) <= a_max
v_min <= v_i(t) <= v_max
```

其中 `v_min` 若未在参数规格中显式给出，第一版可由实现层采用非负速度下界，但应在代码数据结构或车辆模型实现计划中标明来源，不在本文档新增参数。

### 3.4 CAV 与协同目标

当 CAV 是 case 2 / 4 中留在 lane 2 的 CFV，并且需要消费 Eq.10 时，`S_i(t)` 或等价 desired spacing 目标可以被 Eq.10 的协同期望间距覆盖。该覆盖只影响本步跟驰目标，不改变 CAV 仍使用 CAV gap-regulating 模型的事实。

## 4. CHV / IDM 纵向模型

CHV 纵向模型使用论文 Eq.28-Eq.29 的 stochastic IDM 语义。CHV 的 desired speed `v_i^f` 在车辆生成阶段按参数规格中的分布抽样，之后作为该车辆属性参与 IDM。

第一版规则：

```text
1. compliant CHV 可以接受 CUC 的换道或留车道建议。
2. compliant CHV 接受建议后，纵向加速度仍按 IDM 计算。
3. non-compliant CHV 不接受 CUC 建议，继续按普通 IDM 行驶。
4. CHV 不因为 compliance 而变成 CAV 控制器。
```

IDM 使用以下参数：

```text
h_CHV
d_0
L
v_i^f
A_i
b_i
```

具体数值来自 `CORMC参数规格.md`。IDM 公式中的 desired dynamic spacing、速度差和加速度计算口径按论文 Eq.28-Eq.29 展开。若 Markdown OCR 对公式排版不稳定，进入代码实现前必须回查 PDF 或公式截图。

当 compliant CHV 是 case 2 / 4 中留在 lane 2 的 CFV，并且需要消费 Eq.10 时，第一版将 Eq.10 作为该 CHV 的协同期望间距目标交给 IDM 的 desired spacing 语义使用。该处理表达“compliant CHV 接受协同建议”，但不改变其纵向模型仍是 IDM。

当 non-compliant CHV 是 CFV 时，不消费 Eq.10，按普通 IDM 行驶。未执行协同建议这一事实由后续日志验证规格记录。

## 5. CUC 协同车辆运动

CUC 是 cooperative vehicle 的 maneuver choice 模块，不直接更新车辆位置。它只对 APS 指定且 `col = 1` 的 CLV / CFV 执行。

### 5.1 CUC final choice

第一版固化 Eq.16 的回退规则：

```text
if U1 > U2 and Eq.14 target-lane TT safety constraint is satisfied:
    choice 1
else:
    choice 2
```

也就是说，即使 `U1 > U2`，只要目标车道 TT 安全约束不满足，CV 就不能启动 lane 2 -> lane 1 换道，应回退为 choice 2。

### 5.2 Choice 1：lane 2 -> lane 1

choice 1 表示 CV 从 lane 2 换到 lane 1。

```text
1. CV 的 lane_change_state 进入 executing。
2. 初始化 lane 2 -> lane 1 正弦换道轨迹。
3. CV 纵向上以目标车道 TLV 为主 leader。
4. TFV 将正在换道的 CV 视为 leader。
5. 原 lane 2 的 FV 在 CV 完成换道前仍将 CV 视为 leader。
```

choice 1 启动后，该车处于 active lane change。active lane change 中不重新执行 CUC，不重新选择目标，只继续既有横向轨迹，同时仍参与本步纵向动力学计算。

若 case 2 / 4 中的 CFV 选择 choice 1，则 Eq.10 不再作为其原 lane 2 留车道跟驰目标；该车辆进入 lane-changing model。

### 5.3 Choice 2：留在 lane 2

choice 2 表示 CV 继续留在 lane 2。

```text
1. CV 不发生横向换道。
2. CV 继续按车辆类型对应的纵向模型行驶。
3. 若 CV 是 case 2 / 4 中的 CFV，且该车可协同，则使用 Eq.10 对应的期望跟驰间距语义。
4. 若 CV 是 case 3 / 4 中的 CLV，按 APS case 语义和正常纵向模型处理，不套用 CFV 的 Eq.10。
```

### 5.4 Eq.10 消费边界

Eq.10 只用于 APS case 2 / 4 中需要协同的 CFV，不用于 case 3 的 CLV。

第一版按车辆类型和 CUC choice 使用：

| CFV 类型 / 状态 | 是否消费 Eq.10 | 第一版口径 |
| --- | --- | --- |
| CAV，CUC choice 2 留 lane 2 | 是 | Eq.10 作为协同期望跟驰间距 / virtual target spacing，交给 CAV gap-regulating 使用 |
| compliant CHV，CUC choice 2 留 lane 2 | 是 | Eq.10 作为协同期望间距目标，交给 IDM desired spacing 语义使用 |
| non-compliant CHV | 否 | 不接受协同建议，按普通 IDM 行驶 |
| 任意类型，CUC choice 1 换到 lane 1 | 否 | 进入换道模型，不再作为原 lane 2 留车道 CFV 消费 Eq.10 |

该规则是第一版落地口径，用于把论文 Eq.10 的语义接入实际纵向模型。后续如需更严格复现 virtual MV' 的具体构造，应在车辆模型规格修订后再进入代码。

### 5.5 Case 3 的 CLV 协同边界

APS case 3 中，active cooperative vehicle 是 CLV，但这不等于 CLV 消费 Eq.10。CLV 的 active cooperative 身份只表示它参与 CUC choice、assignment lifecycle 和 lane_2 gap 边界；若 CLV 最终 stay lane_2，它仍按自身 CAV / CHV 纵向模型行驶，不套用 CFV 的 Eq.10 desired spacing。

case 3 的纵向协同由 MV 侧表达：MV 将 assigned CLV 作为 logical leading vehicle，继续使用自身车辆类型对应的纵向模型。若 MV 是 CAV，则以 assigned CLV 作为 leader 计算 Eq.17-Eq.19 / CPID 或 cruising；若 MV 是 CHV，则以 assigned CLV 作为 leader 使用 IDM。该 leader 关系只在 assigned CLV 仍 active、位于 lane_2、未执行换道且位于 MV 前方时有效。

`d_star_clv` 和 `d_star_cfv` 在 case 3 中只属于 APS assignment evidence，用于说明为什么得到 case 3；它们不是 MV 速度公式、不是 boundary speed cap，也不是 `d_star_clv / tau` 形式的纵向控制量。

## 6. MV 的 CMC 纵向与合流

CMC 只对已经进入 merging zone 的 MV 执行。MV 状态沿用时间步总纲：

```text
not_started:
    MV 已在 merging zone，但尚未开始横向合流。

waiting:
    MV 在 CMC 内等待可接受 gap，同时仍沿 on-ramp 纵向行驶。

executing:
    MV 已开始横向合流，继续执行既有正弦合流轨迹。

merged:
    MV 已到达 lane 2 centerline，转为 mainline vehicle。
```

### 6.1 On-ramp MV 纵向模型

MV 未合流前仍按自身车辆类型使用纵向模型，不另行发明一个脱离车辆类型的 on-ramp-following 规则。

第一版口径：

```text
if MV is CAV:
    未合流前使用 CAV cruising / gap-regulating。
    leader 来自 on-ramp logical longitudinal relation。
    waiting / executing 时额外叠加 boundary speed cap。

if MV is CHV:
    未合流前使用 IDM。
    compliant 只影响是否接受协同 / 合流建议，不改变 IDM 本质。
    leader 来自 on-ramp logical longitudinal relation。
    waiting / executing 时额外叠加 boundary speed cap。
```

也就是说，`not_started`、`waiting` 和 `executing` 只改变 MV 是否进入 CMC、是否开始横向合流、是否施加 boundary speed cap；它们不改变 MV 的基础纵向模型类型。

### 6.2 Waiting 不是停车

当 MV 已在 merging zone 但 Eq.53 gap 不满足，或者 assignment invalid 时，MV 本步不开始横向合流，但仍沿 on-ramp 纵向行驶。

```text
waiting != stop
waiting == no lateral merge this step + longitudinal movement continues
```

waiting / executing 状态下，MV 纵向速度需要受到 CMC 产生的 boundary speed cap 约束。

### 6.3 Dynamic time gap acceptance

CMC 使用论文 Eq.52 计算 MV 的动态可接受合流时间间隙：

```text
h_tilde_MV_CM(t)
```

其参数包括：

```text
h_upper^CM
xi
x0_m_global
x_ramp_end_global
x_MV_global(t)
```

具体数值来自参数规格和道路几何规格。Eq.52 的 OCR 排版在 Markdown 中存在粘连，进入代码实现前应回查 PDF 或公式截图。

### 6.4 Eq.53 gap 判断

Eq.53 使用 APS assignment 中的 CLV / CFV 作为目标协同对象。第一版不得每步实时重查 lane 2 actual leader / follower 并替代 APS assignment。

执行 Eq.53 前必须进行 assignment valid 工程验证：

```text
1. assigned CLV / CFV 仍存在。
2. assigned CLV / CFV 仍能形成目标协同 gap。
3. assigned CLV / CFV 未驶离仿真路段。
4. assigned CLV / CFV 的 lane / maneuver 状态没有使其失去目标边界意义。
```

assignment invalid 时：

```text
1. MV 本步不开始横向合流。
2. MV 继续 on-ramp 纵向行驶或保守等待。
3. 具体日志字段由输出指标与日志验证规格定义。
```

第一版 Eq.53 actual spacing 使用中心点纵向坐标，并扣减前车长度：

```text
gap_follow_to_lead = x_lead - x_follow - L_lead
```

具体到 MV 与 assigned CLV / CFV：

```text
d_MV_to_CLV = x_CLV - x_MV - L_CLV
d_CFV_to_MV = x_MV - x_CFV - L_MV
```

该口径要求：

```text
x_CLV > x_MV > x_CFV
```

如果该顺序不成立，则 assignment 不能直接用于 Eq.53，应标为 invalid 或等待下一次 APS 更新。是否立即触发保守停车不在本文档决定。

### 6.5 Executing 后不撤销

一旦 MV 进入 `merge_state == executing`：

```text
1. 继续已有 on-ramp -> lane 2 正弦合流轨迹。
2. 不重新判断“是否开始合流”。
3. 不因短时 gap 变化退回 waiting。
4. 除非发生实现层面的硬异常，否则不撤销本次合流。
```

executing 状态仍需要使用 boundary speed cap 或其他速度约束得到最终 planning speed。

## 7. 正弦横向轨迹

第一版不做 MPC tracking。论文 Eq.33-Eq.36 的 improved sine trajectory 作为直接位置更新依据。

两类横向轨迹：

```text
CUC triggered lane change:
    lane 2 -> lane 1

CMC triggered merging:
    on-ramp -> lane 2
```

横向目标 centerline 来自道路几何规格：

```text
lane 1: y = +L_w
lane 2: y = 0
on-ramp: y = -L_w
```

`L_w = 3.5 m` 只是第一版默认候选，仍保留 `to-review` 状态。车辆模型规格使用 `L_w` 符号和参数规格中的当前值，不把它写成论文已给参数。

### 7.1 Maneuver 初始化

换道或合流开始时固定以下语义：

```text
start_x = vehicle.x_global at maneuver start
start_y = vehicle.y at maneuver start
target_y = target lane centerline
lateral_displacement = target_y - start_y
maneuver_type = lane_change or merge
```

执行过程中不因每步 relations 变化重置起点或目标 centerline。若车辆已经处于 active lane change / merge executing，只继续既有轨迹。

### 7.2 位置更新与完成条件

第一版直接按正弦参考轨迹更新车辆横向位置。纵向位置仍由纵向动力学和 planning speed 推进。

完成条件：

```text
primary:
    vehicle.y 到达 target_y 的容差范围内

protective:
    轨迹长度或预计 maneuver 进度已完成
```

具体容差数值不在本文档定义，由后续参数修订或实现计划决定。完成后由状态提交阶段正式更新 physical lane 和 state。

### 7.3 不做 MPC tracking

第一版不实现论文 Eq.47-Eq.51 的 MPC tracking。Eq.37-Eq.41 中 yaw、curvature、steering 等轨迹跟踪相关量不作为第一版核心实现要求；如后续需要更高保真横向动力学，再单独扩展。

## 8. 避碰速度约束

第一版区分事前避碰速度约束和事后 sanity check。

```text
front-collision-avoidance:
    事前约束，影响是否采用 candidate speed 或是否延迟横向 maneuver。

boundary-collision-avoidance:
    事前约束，针对 MV 与 on-ramp downstream boundary，产出 speed cap。

simple collision check:
    事后 sanity check，用于日志、调试和验收，不反向改变本步运动。
```

### 8.1 Front-collision-avoidance

front-collision-avoidance 对应论文 Eq.42-Eq.46。第一版保留其速度约束 / 回退语义：

```text
1. 先按纵向模型得到 candidate_speed。
2. 用 candidate_speed 计算或更新正弦轨迹相关速度语义。
3. 检查轨迹中点防撞约束。
4. 若满足：使用 candidate_speed。
5. 若不满足：使用上一时刻速度或延迟本次横向 maneuver。
```

第一版不在本文档定义“上一时刻速度”和“延迟 maneuver”的具体字段。该部分由状态接口规格和日志验证规格承接。

MV 合流是 on-ramp -> lane 2 的特殊 lane-changing，也应消费 lane-changing model 中可适用的 front-collision-avoidance 语义。boundary speed cap 是额外针对 on-ramp downstream boundary 的约束，不是 front-collision-avoidance 的替代品。若第一版代码暂时只对 CUC 触发的 lane 2 -> lane 1 换道启用 front-collision-avoidance，而不对 MV 合流启用，必须显式标为 first-version simplification，并在日志或验收中保留该风险说明。

### 8.2 Boundary-collision-avoidance

boundary-collision-avoidance 对应论文 Eq.54-Eq.56，只针对 MV 与 on-ramp downstream boundary。

第一版职责拆分：

```text
CMC:
    根据 Eq.54-Eq.56 计算或刷新 boundary speed cap。

纵向动力学阶段:
    将 boundary speed cap 作为 MV 的速度上限施加。

横向轨迹阶段:
    使用已经被 speed cap 约束后的 planning speed。
```

若 boundary speed cap 不可行、为负、过低或导致车辆无法安全运行，本文档不决定失败策略。该问题由车辆模型后续修订、状态接口和日志验证规格共同处理。

### 8.3 多重速度约束合成

当 candidate speed、front-collision 回退速度、boundary speed cap 同时存在时，第一版采用最保守的 planning speed：

```text
planning_speed = min(applicable_safe_speeds)
```

这里的 `applicable_safe_speeds` 是概念集合，不是代码字段名。后续日志验证规格应记录触发了哪些速度约束，以及最终采用哪个 planning speed。

## 9. 第一版关闭 / 简化内容

第一版明确关闭或简化以下论文机制：

| 论文机制 | 论文依据 | 第一版状态 | 说明 |
| --- | --- | --- | --- |
| 普通主线主动换道判断 | Eq.30-Eq.32 | 第一版关闭 | 只做 CUC 触发的 lane 2 -> lane 1 协同换道 |
| yaw / curvature / steering 轨迹跟踪量 | Eq.37-Eq.41 | 第一版简化 | 不作为核心状态更新依据 |
| MPC lateral tracking | Eq.47-Eq.51 | 第一版简化 | 不做严格 MPC tracking，直接使用正弦参考轨迹 |
| CMC platoon 首尾规则 | Section IV-B platoon merging rules | 第一版关闭 | 每辆 MV 独立处理 APS 和 CMC |

关闭或简化不代表论文缺失，而是第一版复现边界。后续增强时应先修订本文档，再进入代码实现。

## 10. 待后续规格承接的问题

以下问题不阻止车辆模型规格作为第一版实现依据，但需要后续文档继续细化。

1. **状态接口字段**
   - command / next-state buffer 的具体字段由 `CORMC状态与模块接口规格.md` 定义。
   - 本文档只定义需要表达的计算语义。

2. **日志字段**
   - speed cap、front-collision fallback、assignment invalid、boundary violation、Eq.53 gap 判断结果等日志字段由 `CORMC输出指标与日志验证规格.md` 定义。

3. **最小验证场景**
   - Scenario A/B/C 的车辆初始状态应在 `CORMC最小验证场景规格.md` 中通过 Eq.7、Eq.10、Eq.52、Eq.53 等反推。
   - 本文档不设定具体初始车辆。

4. **OCR 公式复核**
   - Eq.14、Eq.26-Eq.27、Eq.47-Eq.51、Eq.52、Eq.53、Eq.56 等在 Markdown 中可能存在 OCR 或排版不稳定。
   - 进入代码实现前，应回查 PDF、公式截图或原图。

5. **`L_w` 和 CPID 增益**
   - `L_w = 3.5 m` 仍只是第一版默认候选，保留 `to-review`。
   - CPID 增益来自参数规格中的第一版默认，保留 `to-review`。

6. **boundary speed cap 不可行时的策略**
   - 本文档只定义 speed cap 的消费顺序。
   - 若 speed cap 过低、为负或导致 MV 无法继续安全运行，具体保守策略需要后续确定。

## 11. 验收检查

本文档应满足以下检查：

```text
1. 后续实现者能明确 CAV 什么时候 cruising，什么时候 gap-regulating。
2. 后续实现者能明确 CHV compliance 影响 maneuver 接受，不改变 IDM 纵向模型本质。
3. 后续实现者能明确 Eq.10 只影响 case 2 / 4 中可协同且留 lane 2 的 CFV。
4. 后续实现者能明确 CUC choice 1 必须满足 U1 > U2 且目标车道 TT 安全。
5. 后续实现者能明确 CMC Eq.53 使用 APS assignment 中的 CLV / CFV。
6. 后续实现者能明确 Eq.53 gap 按中心点坐标并扣减前车长度计算。
7. 后续实现者能明确 boundary speed cap 在 CMC 中产生，纵向阶段施加，横向轨迹消费最终 planning speed。
8. 后续实现者能明确 front-collision-avoidance 与 simple collision check 不是同一件事。
9. 文档没有重复参数表。
10. 文档没有设计 dataclass。
11. 文档没有设定 smoke scenario 车辆初始状态。
12. 文档保留 L_w、CPID 增益和 OCR 不稳定公式的待审阅边界。
```
