# CORMC 时间步执行顺序梳理

本文档是第一版 CORMC 复现的算法流程总纲。它用于约束后续场景、参数、车辆模型、数据结构、输出指标等 spec 文档的写作方向，也用于后续代码实现时确认每个仿真时间步内 APS、CUC、CMC、纵向模型、横向轨迹和状态提交的调度关系。

本文档不是论文笔记、不是小场景设计、不是数据结构设计，也不是继续讨论问题的草稿。已经确认的默认语义直接写入主流程；确实无法从论文和现有讨论中确认的问题，单独放在文末供审阅。

本文档吸收了前序算法主循环评估稿中的有效结论，并延续 `docs/复现讨论/CORMC复现讨论对齐记录.md` 中的第一版边界。

## 1. 第一版边界

第一版目标是先做一个能运行、能看到效果的 Python 微观仿真版本。运行后应能输出车辆轨迹图 PNG，并能观察 `lane 1`、`lane 2` 和 `on-ramp` 中车辆随时间运动、协同、换道与合流的过程。

第一版不追求和论文结果完全数值一致。优先目标是跑通 CORMC 主链路，保证算法语义、模块边界和后续扩展方向清楚。

第一版保留：

- 两条主线车道 `lane 1`、`lane 2`，以及一条 `on-ramp`。
- 混合交通：`CAV` 和 `CHV`；`CHV` 按 compliance rate 分为 compliant 和 non-compliant。
- APS：每个 MV 独立寻找 lane 2 中的 anticipatory merge position，并选出 CLV / CFV。
- CUC：lane 2 中需要协同的 CV 在“换到 lane 1”和“继续留在 lane 2”之间做决策。
- CMC 单车合流：动态可接受间隙、边界防撞、正弦轨迹合流。
- 主线纵向模型：CAV 使用 cruising + gap-regulating 思路，CHV 使用 IDM。
- front-collision-avoidance 的简化速度约束语义。

第一版暂不做：

- SUMO 对比。
- 严格 MPC 横向轨迹跟踪。
- CMC platoon 部分，即多个 MV 被视为 platoon 时“只考虑首尾 MV”的规则。
- 普通主线主动换道；第一版只做 CUC 触发的协同换道。
- 全局多 MV gap 优化或全局合流顺序优化。

## 2. 三类语义边界

后续实现和文档都必须区分三类内容：

```text
论文原算法：
    论文明确给出的 APS、CUC、CMC、纵向模型、换道模型、车辆生成和避碰逻辑。

第一版简化：
    为了先跑通主链路而主动关闭或简化的内容，例如不做 MPC、不做 platoon、不做普通主线主动换道。

工程安全补丁：
    为了让第一版代码稳定、避免冲突或非法状态而补充的处理，例如 assignment 失效兜底、多 MV 共享 CV 仲裁、每车每步只提交一次。
```

工程安全补丁不能写成论文已经定义的算法。尤其是 CMC 中 assignment 失效处理、多 MV 共享 CV 的冲突仲裁，必须明确标注为第一版工程补充。

## 3. 时间步核心原则

每个仿真时间步都遵守以下原则：

```text
用 S(t) 做本步所有决策和控制量计算；
所有模块只写本步 command / next-state；
最后同步提交为 S(t+dt)；
S(t+dt) 只供下一时间步使用。
```

不得在本步中途直接修改车辆的 `x`、`y`、`v`、`a` 后再让其他车辆使用这些已更新状态。否则会出现前后车更新顺序偏差。

真实时间使用 `t += dt` 推进，不用 `time = time + 1` 表示真实时间。论文参数中 `dt = 0.1 s`，`T_APS = 5 s`，因此 APS 周期判断必须基于真实时间或等价的 step-time 映射。

`CV` 在本文档中指 cooperative vehicle，即 APS 指定的 CLV / CFV，不是 CAV。CAV 是自动驾驶车辆类型，CV 是协同角色。

车辆有两个需要区分的概念：

```text
physical y：
    车辆当前横向物理位置，用于画图、横向轨迹和碰撞检查。

logical longitudinal role：
    本步纵向关系角色，用于决定谁跟随谁。
```

正在换道的 CV 可能物理上位于两条车道之间，但纵向关系上已经与目标车道 TLV / TFV 和原车道 FV 同时发生作用。不能简单按 `y` 距离哪条车道中心线更近来连续切换 leader/follower。

## 4. 第一版主循环

下面伪代码规定第一版算法流程顺序。它表达调度语义，不设计具体数据结构。

```text
初始化：
    预生成 lane 1 / lane 2 / on-ramp 的 boundary vehicle queues
    生成初始车辆
    初始化 APS assignment cache
    初始化车辆 maneuver / merge 状态
    t = 0
    step = 0
    dt = 0.1

while t <= T_end:

    0. 清理与准备
       移除驶出仿真路段的车辆
       清空本步 command / next-state 缓冲
       不清空 APS assignment cache

    1. 边界车辆生成
       对 lane 1 / lane 2 / on-ramp 的入口队列：
           若等待车辆的到达时距条件和入口安全间隙满足：
               生成该车辆
           否则：
               本步不生成该车

    2. 冻结当前状态 S(t)
       本步 APS / CUC / CMC / 纵向 / 横向决策都只读 S(t)
       所有模块只写 command / next-state

    3. 刷新车辆关系
       基于 S(t) 更新：
           lane 1、lane 2、on-ramp 排序
           leader / follower
           TLV / TFV / LV / FV
           正在换道或合流车辆的 logical longitudinal role

    4. 处理 on-ramp MV

       for each MV:

           if merge_state == executing:
               继续 CMC 合流轨迹
               使用 CMC 已计算或刷新的 boundary speed cap
               写入 MV 的 merge command
               continue

           if MV 不在 merging zone:
               if first_APS(MV) or APS_due(MV, t):
                   执行 APS
                   更新该 MV 的 APS assignment cache
               else:
                   沿用该 MV 的上一轮 APS assignment

               本步 MV 不执行横向合流
               只生成 on-ramp 纵向 command
               continue

           if MV 已在 merging zone and merge_state != executing:
               进入 CMC
               计算动态可接受合流时间间隙
               验证 APS assignment 中 CLV / CFV 是否仍有效
               计算或刷新 boundary speed cap

               if assignment invalid:
                   本步暂不开始合流
                   标记等待下一次 APS 或保守安全处理
                   写入 on-ramp 纵向 command
               else if Eq.53 gap 满足:
                   merge_state = executing
                   初始化或继续正弦合流轨迹
                   写入 merge command
               else:
                   不横向合流
                   继续 on-ramp 纵向行驶 / 调整速度 / 受边界约束
                   写入 on-ramp 纵向 command

    5. 汇总 APS 产生的 CV 协同请求
       对所有有效 APS assignment：
           收集 col = 1 的 CLV / CFV 请求
       如果同一 CV 被多个 MV 请求：
           按第一版工程安全仲裁消解冲突
           优先级：已在 merging zone 的 MV > T*_MV 更小 > 离 x0^m 更近

    6. 处理 mainline 车辆

       for each mainline vehicle:

           if lane_change_state == executing:
               写入 continue lane-change command
               本步不重新执行 CUC
               仍进入 Step 7 计算纵向动力学，Step 8 更新横向轨迹
               continue to next mainline vehicle for CUC processing

           if vehicle 是被选中的 active CV 且 col = 1:

               if vehicle 是 CAV 或 compliant CHV:
                   执行 CUC
                   if CUC choice == lane 1 且目标车道安全:
                       lane_change_state = executing
                       初始化 lane 2 -> lane 1 正弦换道轨迹
                   else:
                       留在 lane 2
                       设置纵向协同目标

               else if vehicle 是 non-compliant CHV:
                   不执行 CUC 建议
                   按普通 CHV / IDM 行驶

           else:
               生成普通主线纵向 command

    7. 计算纵向动力学

       CAV:
           cruising 或 gap-regulating
       CHV:
           IDM
       CUC 留在 lane 2 的 CV:
           若为 case 2 / 4 中的 CFV，使用 Eq.10 对应的期望跟驰间距语义
           若为 case 3 / 4 中的 CLV，按 APS case 语义和正常纵向模型处理，不套用 CFV 的 Eq.10
       不在 merging zone 的 MV:
           按 on-ramp 纵向模型行驶
       CMC waiting / executing MV:
           在 CMC 语义内计算纵向运动，并使用 boundary speed cap 约束纵向速度

    8. 计算横向运动与安全修正

       对 CUC 触发的 lane 2 -> lane 1 换道:
           按正弦参考轨迹更新横向位置
           应用 front-collision-avoidance

       对 MV 合流:
           按正弦参考轨迹更新横向位置
           使用已受 speed cap 约束后的 planning speed

       第一版不做普通主线主动换道

    9. 同步提交 S(t+dt)

       每辆车本步只提交一次：
           x, y, v, a
           physical lane / logical longitudinal role
           APS assignment state
           CUC choice 的 event/history 记录
           lane_change_state
           merge_state

       如果 CV 完成换道：
           lane = lane 1
           lane_change_state = normal
           原 lane 2 的 FV 重新连接到原 lane 中新的 leader

       如果 MV 到达 lane 2 centerline：
           lane = lane 2
           MV 转为 mainline vehicle
           merge_state = merged
           下一时间步可压缩为 merge_state = none
           清理该 MV 的 APS assignment

    10. Vehicle States Information Integration
        记录轨迹、协同事件、CUC choice history、换道事件、合流事件、越界和碰撞检查结果
        形成下一时间步使用的 S(t+dt)

    11. t += dt
        step += 1
```

## 5. 边界车辆生成

车辆生成分为初始化和边界生成两部分。初始化阶段在仿真开始前生成初始车辆；边界生成阶段在每个时间步检查是否有新车从 lane 1、lane 2 或 on-ramp 上游边界进入。

第一版保留论文车辆生成的基本语义：

```text
对每个入口车道：
    查看等待队列中的下一辆车
    判断该车分配到的 arrival headway HA 是否满足
    判断等待车辆相对前车的实际 headway HW 是否满足
    判断入口处与前车的安全间隙是否满足
    满足则生成，否则等待下一时间步
```

车辆类型、CHV compliance、CHV desired speed、CAV inertial lag、arrival headway 等随机属性在车辆生成相关文档中细化。本文档只规定车辆生成在主循环中的位置：它发生在冻结 `S(t)` 之前。

## 6. 车辆关系刷新

冻结 `S(t)` 后，需要基于当前状态刷新车辆关系，包括每条车道的排序、前后车关系，以及 CUC 需要的 TLV / TFV / LV / FV。

第一版保留换道中的三类纵向关系：

```text
CV -> TLV：
    CV 以目标车道前车 TLV 为 leader。

TFV -> CV：
    目标车道后车 TFV 以正在换道的 CV 为 leader。

FV -> CV：
    原车道后车 FV 在 CV 换道完成前仍以 CV 为 leader。
```

箭头表示“后车跟随前车”。CV 自己只有一个主 leader，即 TLV；但 CV 可以同时作为 TFV 和 FV 的 leader。换道开始到完成之前，不根据 `y` 最近车道连续改变纵向关系。CV 到达目标 lane centerline 后，才提交 lane 归属变化，并让原 lane 的 FV 重新连接到新 leader。

更远上游车辆不被 CUC 直接控制。CV 换道显式影响 TFV 和 FV；FV 后方车辆只通过正常跟驰链条间接受影响。

## 7. APS 调度

APS 只作用于尚未进入 merging zone 的 on-ramp MV。这个触发位置来自 Fig. 9；APS 内部预测和 case 判断则遵循 Algorithm 1。Fig. 9 的核心分叉是：

```text
MV 不在 merging zone:
    APS

MV 已在 merging zone:
    CMC
```

因此，第一版不再采用“所有未合流 MV 每步先 APS，再由 CMC 判断能否合流”的调度。

APS 按每辆 MV 的 `T_APS` 周期更新。MV 首次进入 APS 适用阶段时应立即执行 APS，避免没有 assignment cache 可沿用。之后若本步未到 APS 周期，则沿用上一轮 APS assignment。非 APS 周期不代表该 MV 没有 CLV / CFV / case，而是继续使用当前生效的协同关系。

APS 每次更新时执行：

```text
1. 搜索 lane 2 中 MV 前后 Lcr 范围内的候选车辆。
2. 计算 MV 到达 merging zone 起点 x0^m 的预测时间 T*_MV。
3. 预测候选 lane 2 车辆在 T*_MV 后的位置。
4. 找出预测插入间隙对应的 CLV / CFV。
5. 根据预测间隙判断 case 1 / 2 / 3 / 4。
6. 标记 col_CLV / col_CFV。
7. case 2 / 4 设置 virtual MV'，并为 CFV 设置 Eq.10 的期望跟驰间距语义。
8. case 3 标记 MV 可将 CLV 作为协同前车来调整间距。
```

APS 产出的 assignment 是后续 CUC 和 CMC 的依据。第一版不在本文档中设计 assignment 的具体字段。

## 8. CV 协同请求仲裁

多个 MV 独立运行 APS 时，可能选中同一辆 lane 2 车辆作为 CLV 或 CFV。论文没有完整定义这种冲突消解规则，但第一版实现必须避免同一辆 CV 在同一时间步接收多个互相冲突的协同目标。

第一版采用工程安全仲裁：

```text
如果同一 CV 被多个 MV 请求：
    优先服务已经在 merging zone 的 MV；
    其次服务 T*_MV 更小的 MV；
    再其次服务距离 merging zone 起点 x0^m 更近的 MV；
    未获得该 CV 的 MV 标记为等待或冲突状态，后续通过 APS 更新处理。
```

该仲裁属于第一版工程补充，不是论文原生算法。正式实现和后续文档中都不能把它描述成论文已经给出的多 MV 协同分配机制。

## 9. CUC 调度

CUC 只对 APS 指定且 `col = 1` 的 cooperative vehicle 执行。这里的 CV 是 CLV / CFV 角色，不是 CAV 类型。

CUC 的输出是 maneuver choice，不直接更新车辆位置：

```text
choice 1:
    CV 从 lane 2 换到 lane 1。

choice 2:
    CV 留在 lane 2，继续通过纵向模型调整速度和间距。
```

执行规则：

- CAV 接受 CUC 决策。
- compliant CHV 接受 CUC 决策，但纵向行为仍按 CHV/IDM 更新。
- non-compliant CHV 不执行 CUC 建议，继续按自身模型行驶。
- 若 CV 已处于 `lane_change_state == executing`，本步不重新执行 CUC，不重新选择目标，只继续既有换道轨迹；该车仍参与本步纵向动力学计算和横向轨迹更新。
- 若 CV 尚未开始换道，则可以按 Fig. 9 的时间步语义检查 CUC。

CUC 选择 lane 1 时，必须检查目标车道 TLV / TFV 的 TT 安全约束。若换道收益不足或目标车道不安全，则 CV 留在 lane 2。

APS case 与 CUC 后续运动的关系：

- case 1：CLV / CFV 都不需要协同。
- case 2：CFV 需要协同；若 CFV 留在 lane 2，则使用 Eq.10 对应的期望跟驰间距语义为 MV 创建后向 gap。
- case 3：CLV 需要协同；MV 可将 CLV 作为前车来调整间距。
- case 4：CLV 和 CFV 都需要协同；CFV 使用 virtual MV' / Eq.10 语义，CLV 按 CUC choice 执行协同。

## 10. CMC 调度

CMC 只对已经进入 merging zone 的 MV 执行。CMC 包括动态可接受时间间隙、合流决策、合流轨迹和 boundary-collision-avoidance。

第一版 MV 合流状态使用以下语义：

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

若 `merge_state == executing`，本步继续已有合流轨迹，不重新判断“是否开始合流”，也不因短时 gap 变化退回 waiting。进入 executing 后，assignment valid 检查不再用于撤销本次合流；除非发生实现层面的硬异常，否则 MV 继续既有合流轨迹。

若 MV 已在 merging zone 且尚未开始合流，CMC 执行：

```text
1. 读取当前有效 APS assignment。
2. 计算动态可接受合流时间间隙 h~_MV^CM(t)。
3. 验证 assigned CLV / CFV 是否仍有效。
4. 根据 Eq.56 计算或刷新 boundary speed cap。
5. 若 assignment 有效且 Eq.53 gap 满足，则开始合流。
6. 若 assignment 无效或 Eq.53 不满足，则不横向合流，继续 on-ramp 纵向行驶。
```

Eq.53 使用 APS assignment 中的 CLV / CFV 作为目标协同对象。执行 Eq.53 前必须验证 assigned CLV / CFV 是否仍有效：例如是否仍在 lane 2、是否已驶离、是否仍能形成目标协同 gap。

若 assigned CV 已换道离开 lane 2、已驶离、或不再形成安全边界，则：

```text
assignment invalid；
MV 本步暂不开始合流；
等待下一次 APS 或执行保守安全处理。
```

第一版不把这种兜底写成“每步实时重查 lane 2 actual leader/follower 并替代 APS assignment”的论文算法。它只是工程安全补丁。

“等待”不是停车。gap 不满足时，MV 仍沿 on-ramp 纵向行驶，并在纵向动力学阶段使用 boundary speed cap 约束速度。

## 11. 纵向动力学

纵向模型在本步车辆关系、APS assignment、CUC choice 和 CMC state 确定后计算。

CAV：

```text
无 leader 或实际 time gap 足够大：
    cruising

有 leader 且需要跟驰：
    gap-regulating
```

CHV：

```text
IDM
```

compliant CHV 可以接受 CUC 的换道或留车道建议，但纵向加速度仍按 CHV/IDM 逻辑计算。non-compliant CHV 不接受 CUC 建议。

MV：

```text
不在 merging zone：
    按 on-ramp 纵向模型行驶。

在 merging zone 且 waiting：
    在 CMC 内按 on-ramp 纵向行驶，并使用 CMC 产生的 boundary speed cap 约束速度。

在 merging zone 且 executing：
    在 CMC 内继续合流轨迹，并使用 CMC 产生的 boundary speed cap 约束速度。
```

所有纵向计算都只读 `S(t)`。纵向模型输出本步将应用的加速度、速度约束或纵向 command，最后统一提交。

## 12. 横向轨迹与避碰

第一版不做 MPC tracking。论文中给出正弦参考轨迹的地方，第一版直接按参考轨迹更新横向位置。

CUC 触发的 CV 换道：

```text
lane 2 -> lane 1
```

CMC 触发的 MV 合流：

```text
on-ramp -> lane 2
```

换道或合流开始时确定起点、目标 lane centerline 和正弦参考轨迹语义。完成条件以车辆横向位置到达目标 lane centerline 为主；轨迹长度完成可作为保护性检查。具体容差由后续参数或实现文档给出。

front-collision-avoidance 与简单碰撞检测必须区分：

```text
front-collision-avoidance:
    论文 lane-changing model 的事前安全约束，会影响换道/合流时可采用的规划速度。

简单碰撞检测:
    仿真平台的事后检查和 sanity check，不改变车辆运动。
```

front-collision-avoidance 第一版落地口径：

```text
先按纵向模型得到 candidate_speed
用 candidate_speed 计算或更新正弦轨迹相关速度语义
检查轨迹中点防撞约束
若满足：使用 candidate_speed
若不满足：使用上一时刻速度或延迟本次横向 maneuver
```

boundary-collision-avoidance 只针对 MV 与 on-ramp downstream boundary 的安全，核心是 Eq.56 给出的速度上限。第一版中，CMC 负责根据 Eq.56 计算或刷新 boundary speed cap；纵向动力学阶段负责施加该速度上限；横向合流轨迹只消费已经约束后的 planning speed。若 MV 同时受到 front-collision-avoidance 和 boundary speed cap 影响，使用更保守的 planning speed。

## 13. 状态提交与信息集成

每辆车每个时间步只能提交一次状态更新。提交内容包括：

```text
x, y
v, a
physical lane
logical longitudinal role
APS assignment state
CUC choice 的 event/history 记录
lane_change_state
merge_state
事件记录
```

提交规则：

- CV 完成换道后，正式归属 `lane 1`，`lane_change_state` 回到 normal。
- CV 完成换道后，原 lane 2 的 FV 重新连接到原 lane 中新的 leader。
- MV 到达 lane 2 centerline 后，正式归属 `lane 2`，转为 mainline vehicle，`merge_state` 变为 merged，并清理该 MV 的 APS assignment；若后续实现不长期保留 merged，则下一时间步可将该车压缩为 `merge_state = none`。
- `CUC choice` 默认是本步 command / event，不作为下一时间步继续控制车辆的真实状态；若车辆已处于 `lane_change_state == executing`，后续继续换道由 `lane_change_state` 和 maneuver trajectory state 决定。
- 简单碰撞检测、越界检查、near-collision 记录发生在状态提交后，用于调试、指标和 sanity check，不参与 APS/CUC/CMC 决策。

状态集成后形成 `S(t+dt)`，作为下一时间步的输入。

## 14. 前序评估结论的吸收情况

本文档已吸收前序算法主循环评估稿中的关键修正：

- 使用 `t += dt`，不把 `time = time + 1` 当真实时间。
- APS 使用 assignment cache，首次进入 APS 适用阶段立即执行，非 APS 周期沿用上一轮 assignment。
- on-ramp MV 按是否进入 merging zone 分叉：未进入走 APS，进入后走 CMC。
- CMC 使用 `merge_state`，开始合流后不每步重新“开始/等待”。
- CMC 执行 Eq.53 前验证 APS assignment 有效性。
- boundary-collision-avoidance 的职责收敛为：CMC 计算 speed cap，纵向阶段施加，横向阶段消费最终 planning speed。
- CUC 只对 APS 指定且 `col = 1` 的 cooperative vehicle 执行。
- active lane change 中的 CV 不重新执行 CUC。
- Eq.10 的协同期望跟驰间距语义只明确套用于 case 2 / 4 中的 CFV，不误套 CLV。
- 每辆车每步只写 command / next-state，最后同步提交一次。
- 保留 front-collision-avoidance 的简化速度约束语义。
- 多 MV 共享 CV 的处理标注为第一版工程安全仲裁。
- 普通主线主动换道明确为第一版关闭。

本文档与前序评估稿的必要差异：

- 前序评估稿是评估/建议稿，包含评分、诊断和对话语气；本文档是正式算法流程指导，不保留评分和聊天式表述。
- 前序评估稿为了指出问题保留了“必须修改或补充”的评审结构；本文档将已经确认的修正直接写入主流程。
- 前序评估稿中涉及数据结构的示意只保留概念，不在本文档中设计具体字段或类。

## 15. 需后续审阅或细化的问题

以下内容不阻止第一版算法流程成立，但需要后续在参数、车辆模型、数据结构或异常处理文档中细化。这里不直接猜测实现细节。

1. **assignment invalid 后的保守安全处理**
   - 当前流程规定 MV 暂不开始合流，等待下一次 APS 或保守处理。
   - 具体是减速、限速、失败记录还是强制等待，需要后续实现策略文档确认。

2. **边界防撞速度上限过低或不可行**
   - Eq.56 给出 MV 的边界速度约束。
   - 若速度上限过低、为负或导致车辆无法继续安全运行，后续需要定义失败/告警/保守停车策略。

3. **正弦轨迹是否每步动态重规划**
   - 论文称 dynamic lane-changing trajectory planning。
   - 第一版按正弦参考轨迹直接更新横向位置；是否固定一次 maneuver 的起点和目标，或每步更新轨迹长度，由后续车辆模型/轨迹文档细化。

4. **APS / CUC 开关下的消融行为**
   - 第一版代码结构应保留 APS 和 CUC 开关。
   - `enable_aps=False` 或 `enable_cuc=False` 时的默认车辆行为，后续实验/消融文档单独定义。

5. **碰撞检测粒度**
   - 本文档只规定简单碰撞检测用于记录和 sanity check。
   - 具体采用纵横向阈值还是简化矩形检查，由后续指标或仿真实现文档确定。
