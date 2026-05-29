# CORMC 时间步执行顺序梳理

本文档用于讨论第一版 CORMC 复现中，一个仿真时间步 `t -> t + dt` 内 APS、CUC、CMC、纵向更新和横向轨迹更新的执行顺序。

这是一份时间步调度讨论稿，不是完整复现计划，也不是小场景设计。它的目标是先把论文中的分层逻辑转换成一个清楚、稳定、可实现的主循环语义，避免后续实现时把上层决策、下层运动、横向轨迹和纵向加速度混在一起。

本文档延续 `CORMC复现讨论对齐记录.md` 中已经确定的第一版边界：

- 不做 SUMO 对比。
- 不做严格 MPC 横向轨迹跟踪。
- 不做 MV platoon 首尾规则。
- 不做普通主线主动换道，只做 CUC 触发的协同换道。
- 横向换道和合流直接按正弦参考轨迹更新位置。
- 纵向运动按实时加速度规则推进。

## 1. 论文中的基本调度思想

论文的 CORMC 是分层框架：

```text
上层：APS + CUC
下层：纵向模型 + 换道模型 + CMC
```

上层根据当前交通状态做决策：

- APS 给每个 MV 寻找 anticipatory merging position，并指定 CLV / CFV。
- APS 根据预测间隙判断 cooperative case，并标记 CLV / CFV 是否需要协同。
- CUC 对需要协同的 lane 2 中 CV 做 maneuver choice：换到 lane 1，或继续留在 lane 2。

下层根据上层输出执行运动：

- 主线车辆用纵向模型和换道模型更新微观轨迹。
- MV 用 CMC 判断是否开始合流，并执行合流运动。
- 每一步更新车辆的 position、speed、acceleration 等状态。
- 更新后的状态再反馈给下一时间步的上层决策。

因此，第一版主循环应该遵守一个基本原则：

```text
用 t 时刻状态做上层决策和本步控制量计算；
再同步写入 t + dt 状态；
t + dt 状态只供下一步使用。
```

这样可以避免“前面的车先更新、后面的车用已经更新后的状态决策”的顺序偏差。

## 2. 推荐的 `t -> t + dt` 主循环顺序

### Step 0：冻结当前状态 `S(t)`

每个时间步开始时，先把当前所有车辆状态视为一个快照：

```text
S(t) = {
  vehicle id,
  lane / logical lane,
  x, y,
  v, a,
  vehicle type,
  compliance state,
  maneuver state,
  APS assignment,
  CUC choice,
  merge state
}
```

本步中的 APS、CUC、CMC、leader/follower 查找和加速度计算，都应基于这个快照。不要在本步中途一边更新某辆车的位置，一边让另一辆车使用更新后的结果。

输出：

```text
只读快照 S(t)
```

可能问题：

- 如果直接操作车辆对象，很容易在本步中途写入新位置，造成异步更新。
- 建议实现时区分 `state_t` 和 `state_next`，或至少先收集所有本步更新量，再统一提交。

第一版决定和待实现细节：

- 第一版具体使用浅拷贝、不可变快照，还是用“先算 delta 再统一 apply”的结构，需要到代码设计阶段确定。

### Step 1：刷新派生交通关系

根据 `S(t)` 计算每条车道上的车辆排序和前后车关系。

需要得到：

```text
lane 1 中每辆车的 leader / follower
lane 2 中每辆车的 leader / follower
on-ramp 中每辆车的 leader / follower
CUC 计算需要的 TLV / TFV / LV / FV
CMC 判断需要的 CLV / CFV 实际位置
```

这里要区分两个概念：

```text
physical y：车辆当前横向位置
logical lane：本步纵向关系使用的车道归属
```

正在换道的车辆可能 `y` 还在两条车道之间，但纵向上已经需要和目标车道车辆形成子系统。论文中 CV 换到 lane 1 后，CV 与 TLV、TFV 形成新的 lane-changing demand system；在下层 lane-changing model 中，论文又用 `SV` 表示 subject vehicle，也就是“正在执行换道的那辆车”。放到 CUC 触发的协同换道语境里，这个 `SV` 就是正在从 lane 2 换到 lane 1 的 `CV`。因此论文所说的 `SV` 与 TLV 形成新 subsystem、TFV 将 SV 视为新 leader、FV 仍将 SV 视为 leader，可以在第一版中理解为：CV 与 TLV 形成新 subsystem，TFV 将 CV 视为新 leader，同时 FV 仍将 CV 视为 leader 并进行协同运动。

第一版建议：

- 未开始换道的车按当前 lane 建立纵向关系。
- CUC 决定换到 lane 1 后，该 CV 自身的纵向 leader 切换为目标 lane 1 的 TLV。
- 目标 lane 1 的 TFV 将正在换道的 CV 视为 leader。
- 原 lane 2 的 FV 在 CV 换道完成前仍将 CV 视为 leader。
- CV 完成换道后，FV 再重新连接到原 lane 2 中 CV 前方的下一辆车。

换句话说，换道中的 CV 不应该“只属于一个普通 lane 排序”。它应拥有物理横向位置和纵向关系角色两个概念：

```text
physical y：用于画图、横向位置、碰撞检测
logical longitudinal role：用于决定谁跟谁
```

### 换道中 CV 的三类纵向关系

对于从 lane 2 换到 lane 1 的 CV，第一版应按论文语义保留三类关系：

```text
CV -> TLV：CV 以目标车道前车 TLV 为 leader
TFV -> CV：目标车道后车 TFV 以 CV 为 leader
FV  -> CV：原车道后车 FV 在 CV 换道完成前仍以 CV 为 leader
```

这里的箭头表示“后车跟随前车”。因此，CV 自己只有一个主 leader，即 TLV；但 CV 可以同时作为 TFV 和 FV 的 leader。这不是重复约束，而是换道过程中对两个后车的影响同时存在。

不建议第一版根据 `y` 距离哪条车道中心线更近来动态切换纵向关系。那样会导致换道中途 leader/follower 突然跳变。更稳妥的语义是：换道开始时确定 TLV/TFV/FV 关系，换道完成后再更新 lane 归属和原车道 FV 的 leader。

这里与“FV 在 CV 的 `y` 到达目标 lane 中心线时重新连接”并不矛盾。前者禁止的是换道过程中按 `y` 最近车道连续改 leader/follower；后者定义的是一个一次性的 maneuver 完成事件。换道开始到完成之前，关系保持为 `CV -> TLV`、`TFV -> CV`、`FV -> CV`；当 `CV.y` 到达目标 lane 中心线后，才提交 lane 归属变化，并让原 lane 的 FV 重新连接到新 leader。

输出：

```text
lane orderings
leader/follower map
TLV/TFV/LV/FV query results
```

可能问题：

- 正在换道车辆同时影响目标 lane 和原 lane，但这不应理解为 CV 自己同时跟随两个 leader；CV 自己跟随 TLV，TFV 和 FV 分别跟随 CV。
- 如果完全只按 `y` 最近车道归属，换道中途的纵向关系会跳变。
- 关系图需要支持“一个 vehicle 作为多个 follower 的 leader”，普通 lane 排序结果本身可能不够表达这种临时关系。

第一版决定和待实现细节：

- FV 重新连接到原 lane 2 新 leader 的精确时刻：第一版按 CV 的 `y` 到达目标 lane 中心线判定；实现时只需设置一个很小的 `abs(y - target_y)` 容差。
- 第一版碰撞检测只作为仿真指标和实现 sanity check，采用简单物理 `x/y` 阈值或简化矩形检查即可，不作为论文算法的一部分，也不引入复杂碰撞检测算法。

### 更远上游车辆如何受影响

论文在 CUC 和 lane-changing model 中显式建模的是紧邻车辆：

```text
目标 lane：TLV, TFV
当前 lane：LV, FV
```

CUC 的效用函数也只显式包含 CV、TLV、TFV、LV、FV 之间的安全项和新加速度项，例如 `a_TFV^CV`、`a_FV^CV`。论文没有给出“直接控制 FV 后面第二辆、第三辆上游车”的规则。

因此第一版建议：

```text
CV 换道时，显式影响 TFV 和 FV；
FV 后方更远的上游车不被 CUC 直接控制；
它们通过正常纵向跟驰链条间接受影响。
```

也就是说，CV 影响 FV，FV 的速度/加速度变化再影响 FV 后面的车。这既符合论文的局部 subsystem 表达，也避免把 CUC 扩展成论文没有定义的多车全局控制器。

### Step 2：APS 更新

APS 只对尚未完成合流的 MV 执行。论文强调 APS 不应该每个 `dt` 都强制更换 CLV / CFV，而是通过 `T_APS` 间隔更新，以避免频繁切换 cooperative case 导致交通流不稳定。

建议第一版 APS 更新条件：

```text
MV 在 on-ramp 上，尚未进入 active merge
MV 已进入需要进行 APS 的区域或状态
当前时间满足该 MV 的 APS 更新间隔 T_APS
```

APS 本步做的事情：

```text
1. 从 lane 2 中筛选 MV 通信范围内的候选车辆。
2. 计算 MV 到达 merging zone 起点的预测时间 T*_MV(t)。
3. 预测候选 lane 2 车辆在 T*_MV(t) 后的位置。
4. 找到预测位置一前一后的 CLV / CFV。
5. 根据预测间隙判断 case 1/2/3/4。
6. 设置 col_CLV / col_CFV。
7. 在 case 2/4 中，为 CFV 计算用于协同的期望 spacing，并保留 virtual MV' 语义。
```

输出：

```text
MV.assigned_clv
MV.assigned_cfv
MV.aps_case
MV.col_clv
MV.col_cfv
CFV.desired_spacing_for_merge_gap（case 2/4）
```

可能问题：

- 如果同一个 CV 被多个 MV 同时选中，会出现协同冲突。
- 如果 MV 已经开始合流，继续 APS 可能导致 CLV/CFV 在合流过程中跳变。
- 如果 lane 2 通信范围内没有足够车辆，APS 需要边界处理，例如只有前车、只有后车或无车。

待确定：

- 第一版是否只对“尚未 active merge 的 MV”运行 APS。建议默认是：MV 一旦进入 active merge，就固定最近一次 CLV/CFV，直到合流完成。
- 多 MV 同时请求同一 CV 时如何仲裁。建议待确认默认：更靠近 merging zone 起点或更早到达合流区的 MV 优先。
- APS 更新时机是严格按 `T_APS`，还是首次进入 cooperative zone 时立即执行一次，然后再按 `T_APS` 更新。建议第一版采用“首次立即执行 + 后续按间隔更新”。

### Step 3：CUC 决策

CUC 对 APS 标记为需要协同的 lane 2 中 CV 执行。

需要协同的 CV 来自：

```text
col_CLV = 1
col_CFV = 1
```

CUC 的输出不是位置，也不是加速度，而是 maneuver choice：

```text
choice 1：CV 从 lane 2 换到 lane 1
choice 2：CV 继续留在 lane 2
```

执行规则建议：

- CAV 接受 CUC 决策。
- compliant CHV 接受 CUC 决策。
- non-compliant CHV 忽略 CUC 决策，继续按自身模型行驶。
- 已经处于 active lane change 的 CV，不应在每个时间步被 CUC 反复改决策。
- CUC 可在 APS 更新后触发；如果 APS 未更新且 CV 已有未完成 maneuver，则沿用原决策。

CUC 决策时需要查询：

```text
当前 lane 2 的 LV / FV
目标 lane 1 的 TLV / TFV
换道 choice 的效用 U1
留在 lane 2 choice 的效用 U2
目标车道安全约束 TT
```

论文中有一个需要注意的文字问题：原文写到 choice 2 时提及 TT 安全约束，但 Eq. 14 的 TLV/TFV 语义对应的是目标车道安全检查。因此第一版应按下面语义处理：

```text
只有当 CV 准备换到 lane 1 时，才检查目标 lane 1 中 TLV/TFV 的 TT 安全约束。
```

输出：

```text
CV.cuc_choice
CV.target_lane（若选择换道）
CV.maneuver_state（若开始换道）
CV.cooperative_longitudinal_target（若留在 lane 2 且需要纵向协同）
```

可能问题：

- 同一 CV 被多个 MV 赋予不同协同目标。
- CV 换到 lane 1 对 lane 1 的 TFV 产生影响，TFV 的纵向 leader 需要更新。
- CV 留在 lane 2 时，如何把“继续纵向调整”落实为具体 desired spacing 或 leader 关系，需要结合 APS case 处理。

待确定与已定默认：

- CUC 是否每个 `dt` 重新计算效用。建议第一版不这么做，而是在 APS 更新或 CV 当前无 active maneuver 时更新。
- 同一 CV 同时服务多个 MV 的冲突仲裁策略。
- 如果 CUC 选择换道但目标 lane 1 安全约束不满足，是强制留 lane 2，还是延迟决策等待下一步。建议第一版强制选择留 lane 2，并记录原因。

### Step 4：CMC 合流判断

CMC 负责 MV 的单车合流。

CMC 本步使用最近一次 APS 给出的：

```text
CLV
CFV
aps_case
```

CMC 每个时间步都可以判断是否满足合流条件，因为即使 APS 每 `T_APS` 才更新，实际 gap 会随着车辆运动每步变化。

CMC 判断包括：

```text
1. 动态可接受时间间隙 h~_MV^CM(t)。
2. MV 与 CLV 的实际前向间隙。
3. CFV 与 MV 的实际后向间隙。
4. 边界防撞速度上限。
```

如果 Eq. 53 对应的实际间隙条件满足，并且边界防撞允许，则 MV 进入 active merge：

```text
MV.merge_state = active
MV.target_lane = lane 2
MV.merge_trajectory = sine reference trajectory
```

如果不满足，则 MV 继续在 on-ramp 上纵向运动，并继续等待 gap。

输出：

```text
MV.merge_state
MV.target_lane
MV.boundary_speed_cap
MV.merge_trajectory_state
```

可能问题：

- MV 太接近 ramp end 时，边界防撞公式可能给出非常低甚至不可行的速度上限。
- 如果一直不满足 gap，MV 可能被迫急减速或失败。
- MV active merge 后，如果继续更换 CLV/CFV，轨迹和安全判断会变得不稳定。

待确定：

- 边界防撞速度上限为负或过低时，第一版如何处理。建议记录为 failed/unsafe case，或将速度目标裁剪到 0 并保留告警。
- active merge 后是否继续 APS。建议第一版不继续，直到合流完成。
- MV 合流过程中的纵向 leader 默认是否固定为 CLV。建议第一版固定为最近一次 APS 的 CLV。

### Step 5：确定本步纵向子系统

上层决策完成后，需要为所有车辆确定本步的纵向更新关系。

每辆车本步至少需要知道：

```text
longitudinal leader
desired time gap
desired spacing
speed limit / speed cap
是否有特殊协同目标
```

建议第一版规则：

- 普通 mainline 车辆：跟随当前 logical lane 的 leader。
- 普通 on-ramp 车辆：跟随 on-ramp leader。
- 选择换到 lane 1 的 CV：纵向主 leader 使用 lane 1 的 TLV。
- lane 1 的 TFV：在目标车道中将换道 CV 视为 leader。
- 原 lane 2 的 FV：在 CV 换道完成前仍将 CV 视为 leader，换道完成后再重新连接到 lane 2 中新的 leader。
- 留在 lane 2 的 CFV：在 APS case 2/4 中使用 Eq. 10 对应的期望 spacing 语义，为 MV 创建后向 gap。
- active merge 的 MV：以 CLV 作为主要 leader，同时检查 CFV 后向安全。

输出：

```text
vehicle.longitudinal_context
vehicle.leader_id
vehicle.desired_spacing_override
vehicle.speed_cap
```

可能问题：

- 论文对 case 3 中 CLV 如果不换道时如何纵向协同，表达不如 CFV 明确。
- 正在换道车辆会同时影响当前 lane 和目标 lane：这属于论文语义，而不是可忽略的异常情况。
- MV 既在物理上处于 on-ramp/lane 2 之间，又需要和 lane 2 的 CLV/CFV 形成合流关系。

待确定与已定默认：

- Case 3 中，CLV 选择留 lane 2 时是否只按普通纵向模型行驶，还是额外给加速目标。此处需要后续结合论文公式和实验表现再定。
- 换道中车辆的 logical longitudinal role 在 maneuver 开始时切到 target lane 关系；正式 lane 归属在 `y` 到达目标 lane 中心线后提交。在绘图和碰撞检测中始终使用物理 `y`。
- 原 lane 2 的 FV 重新连接到新 leader 的时刻：第一版按 CV 的 `y` 到达目标 lane 中心线判定。轨迹长度完成可作为辅助检查，但不作为主要重连条件。

### Step 6：计算纵向加速度

确定纵向子系统后，计算每辆车本步加速度。

CAV：

```text
无 leader 或实际 time gap 足够大：cruising
有 leader 且需要跟驰：gap-regulating
```

CHV：

```text
IDM
```

MV：

```text
按其车辆类型使用纵向模型；
如果受到边界防撞约束，则额外应用 speed cap 或 deceleration cap。
```

注意：

- compliant CHV 可以接受 CUC 的换道/留车道建议，但纵向加速度仍按 CHV/IDM 逻辑计算。
- non-compliant CHV 不接受 CUC 建议。
- 所有加速度都应基于 `S(t)` 和本步已经确定的纵向 context 计算，不能使用其他车辆已经更新到 `t+dt` 的位置。

输出：

```text
a_i(t)
v_i target/capped
```

可能问题：

- CAV gap-regulating 的 CPID 参数来自前作，CORMC 本篇没有完整列出，需要配置化。
- IDM 在极小间距下可能产生很大减速度，需要裁剪。
- 如果 MV 同时需要跟驰和边界防撞，应明确谁优先。建议第一版边界防撞速度上限优先。

第一版决定和待实现细节：

- 纵向积分时使用的加速度是 `a(t)` 还是 CPID 公式中的 `a(t+dt)`。建议实现时把模型输出视为本步将应用的加速度，并统一记录。
- 加速度和速度上下界的裁剪顺序。

### Step 7：同步更新纵向状态

所有车辆的加速度计算完后，再统一更新纵向状态。

建议第一版采用统一、简单、可解释的积分规则，例如：

```text
v(t+dt) = clip(v(t) + a(t) * dt)
x(t+dt) = x(t) + v(t) * dt + 0.5 * a(t) * dt^2
```

更新后再做：

```text
速度上下界裁剪
加速度上下界裁剪
道路边界检查
```

输出：

```text
x(t+dt)
v(t+dt)
a(t+dt) 或 a_applied(t)
```

可能问题：

- 如果先裁剪速度再更新位置，和先按加速度更新再裁剪，会产生不同结果。
- 边界防撞速度上限如果在 Step 6 已经生效，Step 7 不应再次产生越界速度。

待确定：

- 最终积分公式在实现文档中固定后，所有模型都必须共用同一套规则。

### Step 8：更新横向轨迹状态

纵向状态更新后，对正在执行 lane change 或 merge 的车辆更新横向位置。

第一版原则：

```text
不做 MPC tracking；
直接按正弦参考轨迹计算 y(t+dt)。
```

正在换道的 CV：

```text
lane 2 -> lane 1
```

正在合流的 MV：

```text
on-ramp -> lane 2
```

横向轨迹需要记录：

```text
trajectory_start_x
trajectory_start_y
target_y
trajectory_length M
start_time
```

完成条件可以初步理解为：

```text
车辆 y 足够接近 target_y
或 x - trajectory_start_x 达到轨迹规划长度
```

完成后：

```text
lane = target_lane
logical lane = target_lane
maneuver_state = none/completed
```

可能问题：

- 论文称 dynamic lane-changing trajectory planning，`M(t)` 与速度有关，可能每步更新。
- 第一版若每步重算 `M(t)`，轨迹可能抖动；若固定 `M`，则和论文 dynamic 语义略有偏差。
- MV 合流过程中如果纵向速度变化很大，固定轨迹长度可能导致横向完成时机不自然。

待确定：

- 第一版建议固定一次 maneuver 的起点和目标，按正弦参考轨迹直接更新；是否每步重算 `M(t)` 作为后续增强。
- 换道/合流完成以 `y` 到达目标 lane 中心线为主；实现阶段只需确定 `abs(y - target_y)` 的数值容差。轨迹长度完成可作为保护性检查。

### Step 9：记录、检测、反馈

本步最后统一记录：

```text
车辆轨迹
APS assignment
CUC choice
merge start/completion event
lane change start/completion event
collision/near-collision event
boundary violation event
```

然后执行安全检查：

```text
同 lane/logical lane 前后车纵向间距
物理 x/y 距离是否重叠
MV 是否越过 ramp end 仍未完成合流
车辆是否驶出仿真道路边界
```

最后形成：

```text
S(t+dt)
```

作为下一时间步输入。

可能问题：

- 只按 lane 检查碰撞可能漏掉换道中车辆的侧向冲突。
- 只按物理距离检查又可能需要更复杂的车辆矩形模型。

待确定：

- 第一版碰撞检测只用于记录指标和发现实现错误，可以先用简化矩形或纵横向阈值。它不参与 APS/CUC/CMC 决策，也不作为论文算法增益。

### 碰撞检测与 front-collision-avoidance 的区别

这里需要区分两个概念：

```text
简单碰撞检测：仿真平台的事后检查/指标记录。
front-collision-avoidance：论文 lane-changing model 的事前安全约束。
```

简单碰撞检测发生在状态更新后，用来回答：

```text
这一步仿真结果里有没有车辆重叠、过近或越界？
```

它主要服务于调试、指标记录和 sanity check。第一版不需要做复杂的全局碰撞检测算法，简单的 `x/y` 距离阈值或简化矩形检查就够。

front-collision-avoidance 则发生在换道/合流运动执行过程中，用来避免 SV 在换道到车道分界线附近时追上当前 lane 的 LV。论文给出的思路是检查 SV 到轨迹中点时，与 LV 的纵向间距是否仍满足安全条件；如果不满足，则本步轨迹规划不使用按最新纵向加速度更新后的速度，而是回退使用上一时刻速度。

更准确地说，论文这里不是另写一个连续优化控制器，也不是做全局碰撞检测，而是在正弦参考轨迹规划前加入一个局部安全检查。它会影响本步用于计算轨迹长度 `M(t)` 的速度，从而影响横向换道参考轨迹。

因此，二者区别是：

```text
简单碰撞检测：被动记录，不改变车辆运动。
front-collision-avoidance：主动约束，会影响换道/合流时可采用的速度或是否继续按当前速度规划。
```

第一版建议保留 front-collision-avoidance 的速度回退/速度约束语义，因为它属于论文换道模型的一部分；但不扩展成复杂控制器，也不和全局碰撞检测混为一谈。这里的“简化”主要指第一版不实现论文后续 MPC 横向轨迹跟踪，也不额外求解一个论文没有给出的最优安全速度。

### front-collision-avoidance 的第一版落地口径

第一版建议按下面口径实现，既保留论文语义，又避免扩展成论文没有定义的复杂控制器：

```text
先按纵向模型得到 candidate_speed = v(t - dt) + a(t) * dt
用 candidate_speed 计算正弦轨迹长度 M(t)
检查 front-collision-avoidance 的轨迹中点安全约束
如果满足：本步用 candidate_speed 规划横向参考轨迹
如果不满足：本步轨迹规划速度回退到上一时刻速度
```

这里“回退到上一时刻速度”是论文明确给出的处理方式。它不是在求一个新的最优安全速度，也不是让车辆进入额外的横向控制器。

如果车辆尚未正式进入 active lane change / active merge，而中点安全约束已经不满足，第一版可以先延迟启动本次横向 maneuver。这属于工程上的保守处理，需要在实现中记录为第一版取舍，而不是论文明确给出的新增算法。

对已经处于 active lane change / active merge 的车辆，第一版不建议每步随意改换道对象或重新触发 CUC/APS；只在当前 maneuver 内根据 front-collision-avoidance 选择本步用于正弦参考轨迹规划的速度。

需要特别区分：

```text
front-collision-avoidance：原 lane 前方 LV 的中点防撞检查。
boundary-collision-avoidance：MV 与匝道末端边界的速度上限检查。
```

前者主要影响 SV/CV/MV 在换道或合流横向轨迹中的规划速度；后者只针对 MV，且论文 Eq. 56 给出了更直接的 ramp-end 速度上限。第一版实现时，如果 MV 同时受到二者影响，应先按纵向模型得到 candidate speed，再同时应用 front-collision 的速度回退语义和 boundary-collision 的速度上限，最终使用更保守的规划速度。

## 3. 推荐主循环伪代码

下面伪代码用于表达顺序，不代表最终代码结构：

```text
for each time step t:
    state_t = freeze_current_state()

    relations = build_lane_relations(state_t)

    for each MV not merged and not active_merge:
        if should_run_aps(MV, t):
            aps_result = run_aps(MV, state_t, relations)
            store_aps_assignment(MV, aps_result)

    for each CV marked by APS as col = 1:
        if can_accept_cuc(CV) and not active_lane_change(CV):
            cuc_choice = run_cuc(CV, state_t, relations)
            store_cuc_choice(CV, cuc_choice)
        else:
            keep_or_ignore_cuc_choice(CV)

    for each MV not merged:
        cmc_result = evaluate_cmc(MV, state_t, latest_aps_assignment)
        if cmc_result.can_merge:
            start_or_continue_merge(MV, cmc_result)

    longitudinal_context = build_longitudinal_context(
        state_t,
        relations,
        aps_assignments,
        cuc_choices,
        merge_states
    )

    for each vehicle:
        acceleration[vehicle] = compute_longitudinal_acceleration(
            vehicle,
            state_t,
            longitudinal_context
        )

    state_next = integrate_longitudinal_state(state_t, acceleration, dt)

    for each vehicle with active lane_change or active merge:
        planning_speed = choose_lateral_planning_speed(
            vehicle,
            state_t,
            state_next,
            front_collision_avoidance=True,
            boundary_collision_avoidance_if_mv=True
        )
        update_lateral_position_by_sine_reference(vehicle, state_next, planning_speed)
        complete_maneuver_if_reached_target(vehicle, state_next)

    record_history_and_events(state_next)
    check_collisions_and_boundaries(state_next)
    commit(state_next)
```

## 4. 当前最需要后续确认的问题

以下问题不会阻止继续推进，但需要在实现前或实现中逐步锁定：

1. **多 MV 冲突**  
   多个 MV 同时选择同一 CLV/CFV 或同一 CV 时，第一版采用什么优先级。

2. **Case 3 的 CLV 留 lane 2 行为**  
   论文对 CFV 的期望 spacing 讲得更明确，对 CLV 留在 lane 2 时如何纵向协同不够直接。需要实现时谨慎处理。

3. **换道中车辆的双车道关系落地方式**  
   论文语义上，CV 换道时目标 lane 的 TFV 和原 lane 的 FV 都受影响。更远上游车辆通过 FV 的纵向跟驰链条间接受影响，不做 CUC 的显式多车控制。后续需要确认的是数据结构，而不是是否保留 FV 影响。

4. **横向轨迹是否动态重规划**  
   论文是 dynamic trajectory planning；第一版倾向固定一次 maneuver 的参考轨迹，避免抖动。后续可以增强为每步重规划。

5. **边界防撞失败状态**  
   如果 MV 接近 ramp end 且仍无法合流，需要定义失败、急减速或强制等待的处理方式。

6. **碰撞检测粒度**  
   第一版碰撞检测只做简单指标记录和实现检查，不引入复杂全局碰撞检测算法。需要确定的只是采用纵横向阈值还是简化矩形。

7. **APS / CUC 开关下的行为**  
   后续为了消融实验，需要明确 `enable_aps=False` 或 `enable_cuc=False` 时，MV 和 CV 的默认行为。

## 5. 第一版默认建议

为了让第一版先跑通，建议先采用以下默认：

- 上层决策和加速度计算全部基于 `S(t)` 快照。
- APS 首次进入协同范围立即执行，之后按 `T_APS` 更新。
- MV 进入 active merge 后固定 CLV/CFV，不再继续 APS。
- CUC 不对 active lane change 的 CV 反复重决策。
- CUC 选择换道但目标 lane 不安全时，CV 留在 lane 2。
- 正在换道车辆的纵向主关系以 target lane 为主：CV 跟随 TLV，TFV 跟随 CV，同时原 lane 的 FV 在换道完成前仍跟随 CV。
- FV 在 CV 的 `y` 到达目标 lane 中心线时重新连接到原 lane 的新 leader。
- CV 换道对更远上游车只通过 FV 间接传播，不显式控制 FV 后方更多车辆。
- 横向轨迹在 maneuver 开始时固定起点、目标和轨迹长度，后续按参考轨迹直接更新。
- 所有车辆同步更新纵向状态，再更新横向位置。
- 简单碰撞检测只用于指标记录；front-collision-avoidance 作为论文中的换道安全约束，以简化速度约束语义保留。
- 对论文表达不清的地方，在代码中保留注释和配置开关，不假装已经完全确定。
