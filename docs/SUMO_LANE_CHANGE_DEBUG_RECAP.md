# SUMO 换道瞬移与中心线收敛调试复盘

## 1. 文档目的

这份文档记录了本轮为了修复 SUMO GUI 中“换道瞬移”问题而引出的一整条 debug 链，包括：

- 最初看到的现象
- 每一步排查时发现的真实根因
- 实际落地的代码修改
- 为什么后来又出现了“贴边走”“没有回到车道中心线”“甚至堵住”的现象
- 我们最终采用的自动验收方案
- 当前已经达到的效果，以及仍然保留的边界问题

这份文档面向两类读者：

- 人类开发者：快速理解这次到底改了什么、为什么这样改
- 后续 agent：接手时不用重新读完整段聊天记录，也能继续复现、验证、迭代

注意：当前仓库 `git diff` 里还有一些与 `Algorithm_python/` 相关的并行修改；本文只覆盖本轮与 `ramp/` 下 SUMO 换道瞬移、中心线收敛、观测与验收相关的改动。

---

## 2. 先建立概念框架

这一节不直接讲 bug，而是先把理解这次问题所需的几个概念讲清楚。  
如果先没有这些概念，后面看到 `sublane`、`laneChangeMode`、`latAlignment`、`center_hold_ok` 这些词，会很容易把“换道决策”和“横向执行”混成一件事。

### 2.1 把“换道问题”拆成三层

在 SUMO 里，一次换道可以拆成三个层次：

1. 决策层：谁来决定“要不要换道、什么时候换、换到哪条车道”
2. 执行层：车辆如何从当前车道横向移动到目标车道，是瞬移还是连续移动
3. 验收层：我们如何判断这次换道是否真的完成，尤其是车是否回到了目标车道中心线

这次项目里的职责分工是：

- 你的算法主要负责决策层
- SUMO 负责底层执行、碰撞/安全约束和横向状态演化
- 我这轮新增的观测与报告，主要属于验收层

这也是本次 debug 最重要的认识之一：

> “算法已经决定换道成功” 不等于 “SUMO 已经把车稳定地带到目标车道中心线”。

### 2.2 `lane_id` 和 “横向是否居中” 不是一回事

很多初看 SUMO 的人会自然地以为：

- 车在 `main_h3_1`
- 就说明它已经位于这条车道的中心

这在默认离散车道模型里看起来像是成立的，但在开启 sublane 模型后就不成立了。

开启 sublane 后，车辆同时有两类状态：

1. 车道归属：例如 `lane_id = main_h3_1`
2. 横向位置：例如 `getLateralLanePosition(veh_id) = y`

其中：

- `lane_id` 表示车辆当前被归类在哪条 lane 上
- `y = 0` 才表示车辆位于当前 lane 的中心线
- `|y|` 越大，说明离中心线越远

所以会出现一种很关键的现象：

- 车辆的 `lane_id` 已经切到了目标 lane
- 但它的横向位置还没有回到目标 lane 中心线

这正是这次“看起来换过去了，但还是贴边走”的技术本质。

### 2.3 什么是 sublane 模型

SUMO 默认的车道模型，可以粗略理解为“按整条 lane 离散处理”：

- 车辆在某一时刻属于某一条 lane
- 换道时可以直接从一条 lane 跳到另一条 lane
- GUI 上会表现成明显的横向突变

`sublane model` 则是在 lane 内部再引入横向连续坐标。  
开启后，车辆不只是“属于哪条 lane”，还会有“在这条 lane 里偏左还是偏右”的状态。

直观地说：

- 不开 sublane：像在方格纸上跳格子
- 开了 sublane：像在一条带宽度的走廊里连续侧移

它带来的直接变化有三类：

1. 换道轨迹可以连续，不再一定是瞬移
2. 车辆可以在 lane 内处于非中心位置
3. 可以定义和观测“是否真的回到中心线”

### 2.4 `--lateral-resolution` 到底做了什么

开启 sublane 模型的关键参数是：

```bash
--lateral-resolution 0.25
```

它的作用不是“每次横向只移动 0.25 米”，而是：

- 告诉 SUMO 启用横向细分
- 横向计算、碰撞处理和 lane 内行为不再只按整条 lane 处理
- 横向判断的细粒度达到 `0.25m`

它对项目的影响主要有四个：

1. GUI 里的换道不再是瞬移
2. `getLateralLanePosition()` 这类横向数据开始变得有意义
3. 车辆可能长期停留在非中心位置，因此“已换道”和“已居中”必须分开判断
4. 仿真会比完全离散 lane 模型稍重一些

这次项目里，sublane 模型现在已经明确打开了，而且是两层保障：

- 场景配置里默认打开  
  `ramp/scenarios/ramp__mlane_v2_mixed/ramp__mlane_v2_mixed.sumocfg`
- 运行入口 `ramp/experiments/run.py` 里也默认传入 `--lateral-resolution 0.25`

### 2.5 `latAlignment` 是什么，它和“回中心线”有什么关系

开启 sublane 模型后，车辆在 lane 内可以有不同的横向偏好。  
`latAlignment` 就是在描述这种偏好。

如果没有显式设定 `latAlignment=center`，那么车辆虽然在 lane 内合法行驶，但：

- 未必会主动把自己放在 lane 中心线附近
- 也可能长期保持一定横向偏移

因此，这次我在 vType 上显式加入了：

```xml
latAlignment="center"
```

这不是在替代你的换道算法，而是在告诉 SUMO：

> 当车辆已经处于某条 lane 内时，它的默认横向偏好应该是 lane 中心，而不是边缘或某个任意偏置位置。

### 2.6 `laneChangeMode` 是什么

`laneChangeMode` 不是一个简单的“自动换道开/关”开关，而是一组 bit 位控制。

它至少同时控制三类东西：

1. SUMO 是否允许车辆出于战略、协作、提速、靠右等动机自主换道
2. TraCI 发出的换道请求是否仍要遵守安全间距
3. 是否允许 sublane 级别的横向变化

因此，`laneChangeMode = 0` 的含义其实很强：

- 关闭了 SUMO 自主换道的主要动机
- 同时也把 sublane 级横向变化一并压掉了

这就是为什么它能“彻底接管换道决策”，却也会有副作用。

### 2.7 为什么 `laneChangeMode = 0` 会有副作用

最初我们确实希望：

- 不让 SUMO 自己乱换道
- 所有受控 CAV 的 lane 选择都由你的算法决定

所以用 `laneChangeMode = 0` 看起来很直接。  
但它的问题在于，它不只关掉了“自动决定换到哪条 lane”，还过度限制了“已经确定目标 lane 后，车辆怎样在横向上自然归中”。

于是就会出现下面这条链：

1. 你的算法已经决定要换到目标 lane
2. TraCI 命令也发出去了
3. 车道归属已经切换
4. 但 lane 内的横向居中能力被一起冻结
5. 车辆可能长期贴边，不自然回到中心线

所以问题不是“算法没控制换道”，而是：

> 控制器把决策权接得很稳，但把执行层也锁得太死了。

### 2.8 当前项目里，谁在控制什么

这次修复之后，当前项目的职责边界可以概括为：

- 受控 CAV 的换道决策：仍然主要由你的算法控制
- SUMO 自动的战略/协作/提速/靠右换道：对受控 CAV 继续关闭
- SUMO 的横向执行与安全约束：保留
- lane 内部的回中心线能力：保留

再说得更直白一点：

- “换不换、换到哪条 lane” 还是你的算法说了算
- “换过去之后怎么连续移动、怎么在安全条件下归中” 交给 SUMO 底层执行

这就是本轮修改后最重要的控制语义。

### 2.9 用一句话对应到本次代码修改

如果把这次改动和上面的概念一一对应，可以归纳成下面这张“概念 -> 实际修改”的对照表：

1. 为了让换道不再瞬移  
   我打开了 sublane 模型，也就是启用了 `--lateral-resolution 0.25`

2. 为了让车辆在 lane 内有“回到中心线”的偏好  
   我给 vType 显式加上了 `latAlignment=center`

3. 为了继续保留“受控 CAV 不让 SUMO 自己乱换道”  
   我没有放开 SUMO 的战略/协作/提速型自动换道

4. 为了避免把“lane 内回中心线”的能力也一起冻死  
   我把 `laneChangeMode` 从完全冻结改成“冻结自动动机，但保留 sublane 居中和安全约束”

5. 为了让“是否真的居中”可以被证明，而不是靠肉眼截图  
   我把横向位置、换道生命周期和 merge 车道健康都记到了输出文件，并做了自动验收脚本

如果读到后面一时忘了“为什么会改这里”，回到这一小节通常就能重新对上。

---

## 3. 问题是怎么一步步暴露出来的

### 阶段 A：最初现象是“瞬间平移换道”

最开始在 SUMO GUI 里看到的现象是：

- 车辆从一个车道中心直接跳到另一个车道中心
- 横向没有连续过渡
- 看起来像“瞬移”或者“平移”

这类现象最常见的直接原因是 SUMO 没开 sublane 横向细分，也就是 `lateral-resolution = 0`。

---

### 阶段 B：加了横向细分以后，不再瞬移，但又出现“贴边走”

在打开 `--lateral-resolution 0.25` 之后，GUI 里的确不再是瞬移了，但又暴露出第二层问题：

- 有些车并没有回到目标车道中心线
- 有些车会贴着分道线跑
- 有些车在目标车道的下半区域“直线滑行”
- 某些 seed 下还会出现很难看的拥堵，甚至看起来像把主线堵住

也就是说：

- 第一层问题是“有没有横向连续轨迹”
- 第二层问题是“换到目标车道之后，是否真的回到了该车道的中心线”

这两个不是同一个问题。

---

### 阶段 C：单靠 GUI 肉眼看不够，必须改成量化验收

继续只靠 GUI 看，会有两个严重问题：

1. `lane_id` 已经切过去了，不代表车身已经回到中心线
2. 有些车在过渡过程中偏移很大，但过 1 秒后又会回到中心；如果只截图，很容易误判

所以这轮工作一个关键转向是：

> 从“看起来像修好了”切换为“用横向偏移公式和自动报告来验收是否真的修好了”。

---

## 4. 这轮定位出来的根因

### 4.1 根因 1：`lateral-resolution = 0` 时，SUMO 本来就会瞬移

这是最早、也最直接的根因。

当 SUMO 的横向分辨率为 0 时，车辆没有 sublane 级别的横向轨迹，只会从一个车道中心直接跳到另一个车道中心。

因此：

- `changeLane()` 仍然可以成功
- 但 GUI 表现会是“突变”

这部分的修复很明确：必须保证 `lateral-resolution > 0`。

---

### 4.2 根因 2：`target_lane_reached` 不等于“已经回到中心线”

这是本轮最重要的认知修正之一。

车已经到达目标车道，只能说明：

- 当前 `(edge_id, lane_index)` 落在目标走廊里

但它并不能说明：

- 横向位置已经回到该车道中心线

因此，必须把“到达目标车道”和“中心线收敛”分成两个独立指标。

---

### 4.3 根因 3：vType 没有显式 `latAlignment=center` 时，可能保留横向偏置

打开 sublane 模型以后，车辆的横向位置不再只有“车道中心”这一种可能。

如果车辆类型没有明确声明：

```xml
latAlignment="center"
```

那么不同车辆、不同 vType、不同 route 文件来源，可能会表现出不同的横向偏置。结果就是：

- 车虽然在目标车道里
- 但它未必主动往中心线收敛
- GUI 上会看到贴边、偏半车道、长期不归中

---

### 4.4 根因 4：之前的 `laneChangeMode = 0` 把横向行为“冻得太死”

之前控制器里大量使用的是：

- `LC_MODE_PROHIBIT_ALL = 0`

这会把 SUMO 自主换道相关行为全部关掉，但副作用是：

- 对 sublane 居中也过于激进
- 换道完成后，车可能继续保持一个偏移过的横向状态
- 即使 lane id 已经切到目标车道，也未必会自然回到中心线

所以这一层不是“换道命令没发出去”，而是：

> 发出命令后，横向状态被锁得太死，缺少后续的居中恢复。

---

### 4.5 根因 5：换道指令持续时间与重试节奏不合适，会干扰稳定收敛

另一个更细的控制层问题是：

- 换道命令持续时间太短或重发节奏不合适
- 车辆刚切入目标车道，后续又收到重复命令或重试
- 会干扰横向稳定下来

因此后面我们把换道命令持续时间和 fallback/retry 的计时逻辑也一起收紧了。

---

### 4.6 根因 6：最初的观测链有盲区，导致“其实居中了，但证据没记下来”

这是本轮排查里非常关键的一次“误判纠正”。

最初 `control_zone_trace.csv` 只记录控制区里的车辆状态，也就是大致等价于：

- 还在 `control_zone_state` 里
- 还没彻底离开控制区

问题在于，车辆完成合流后，最重要的“中心线是否收敛”往往发生在：

- `main_h3_1`
- `:n_merge_0_0`
- `main_h4_0`

这些位置上，车可能已经不在原本的控制区状态里了。如果这时 trace 停止记录，就会出现一种假象：

- GUI 上看起来它可能后面已经归中
- 但 `control_zone_trace.csv` 没有后半段样本
- 自动分析就只能得到“不确定”甚至“未收敛”

因此这轮实际还修了一个观测层 bug：

> 对目标走廊里的车辆做补充追踪，哪怕它已经离开原始控制区，只要还在目标走廊，就继续写入 trace。

这一步修完以后，之前一些“像是控制没修好”的 false negative 被消掉了。

---

## 5. 本轮实际落地的修复

### 5.1 让 sublane 模型默认开启，先消除“瞬移”

已落地两层保障：

1. 场景配置文件里写入默认值  
   文件：`ramp/scenarios/ramp__mlane_v2_mixed/ramp__mlane_v2_mixed.sumocfg`

```xml
<processing>
    <lateral-resolution value="0.25"/>
</processing>
```

2. 运行入口也暴露并默认传入 `--lateral-resolution 0.25`  
   文件：`ramp/experiments/run.py`

这样无论是 GUI 运行还是非 GUI 运行，只要不显式覆盖为 0，默认都不会再回到“瞬移换道”的模式。

---

### 5.2 在 vType 上显式设定 `latAlignment=center`

修改文件：

- `ramp/common/vehicle_defs.py`

对以下类型显式加入：

- `VTYPE_CAV["latAlignment"] = "center"`
- `_HDV_BASE["latAlignment"] = "center"`
- `VTYPE_HDV["latAlignment"] = "center"`

这一步的作用是：

- 当 sublane 模型开启后，车辆的默认横向目标是车道中心
- 减少不同 route/vType 来源带来的横向偏置不一致

另外，`run.py` 还会对 route 文件里的 vType 与代码里的 SSOT 做一致性检查，避免“代码改了，但实际跑的 `.rou.xml` 还是旧定义”的情况悄悄发生。

---

### 5.3 把 lane-change mode 从“完全冻结”改成“冻结自主换道，但保留 sublane 居中”

修改文件：

- `ramp/runtime/controller.py`

关键改动：

- 原来：`LC_MODE_PROHIBIT_ALL = 0`
- 现在：`LC_MODE_FREEZE_AUTONOMY_KEEP_SUBLANE = 2560`

语义上现在更接近：

- 禁止 SUMO 自己乱做战略/协作/收益型换道
- 但保留安全检查
- 同时保留 in-lane sublane alignment 的能力

这一步的价值很大，因为它直接针对了“已经并道，但横向一直贴边”的症状。

---

### 5.4 对 post-merge corridor 加一个最小、定向的居中修复

修改文件：

- `ramp/runtime/controller.py`

新增了 `_apply_post_merge_centering(...)`，逻辑是：

- 只针对 ramp CAV
- 只在 post-merge corridor 里触发
- 当前定义主要覆盖：
  - `main_h3` 的目标车道
  - `:n_merge_0_0`
- 读取当前横向偏移 `getLateralLanePosition(veh_id)`
- 如果 `|lat| > 0.25m`，则用 `changeSublane()` 做一个受限幅值的小修正
- 单步最大修正幅值限制为 `0.4m`

这一步不是大改控制策略，而是：

> 在换道已经完成、只剩“回到中心线”这个尾部动作时，给一个小而可解释的横向归中 nudge。

---

### 5.5 调整换道命令持续时间和 retry 语义

修改文件：

- `ramp/policies/hierarchical/merge_point.py`

改动包括：

- 新增 `lc_command_duration_s = 3.0`
- 用它替代原先直接复用的 `t_lc_s`
- fallback/retry 时，保持 `MERGING` 状态但重置计时器，避免逐步重复发令

这一步的目标不是解决“瞬移”，而是减轻这类副作用：

- 指令风暴
- 过晚完成
- 刚切过去就被下一轮干扰

它属于“让车辆在切到目标车道后更容易稳定下来”的辅助修复。

---

### 5.6 把“只看 GUI”升级为“有证据链的自动验收”

这一部分是本轮最重要的工程化改动之一。

### 新增/增强的输出文件

文件：`ramp/experiments/run.py`

运行后现在会额外输出：

1. `control_zone_trace.csv`

追加横向观测列：

- `edge_id`
- `lane_id`
- `lane_index`
- `lane_width_m`
- `lat_lane_pos_m`
- `lat_abs_m`
- `lat_norm`
- `is_centered_0p25`

2. `lane_change_lifecycle.csv`

记录关键事件的横向快照，例如：

- `zone_c_lc_command`
- `zone_c_lc_complete`
- `lane_id_switch`

3. `merge_lane_health.csv`

记录 merge 相关车道健康信息，例如：

- `vehicle_count`
- `halting_count`
- `mean_speed`
- `starting_teleport_count`

4. `centering_report.json`
5. `centering_report.csv`

6. `metrics.json`

里面会自动注入中心线验收相关字段。

---

### 5.7 修复 trace 盲区：目标走廊里的车辆继续记录

修改文件：

- `ramp/experiments/run.py`

这一步专门修复了“证据链不完整”的问题。

现在 trace 的写法不再只盯 `control_zone_state`，而是：

- 先记录控制区车辆
- 再补一层扫描当前 `active_vehicle_ids`
- 只要车辆还在目标走廊里，即使已离开原始控制区，也继续写入横向样本

目标走廊当前定义为：

```text
(main_h3, 1), (:n_merge_0, 0), (main_h4, 0)
```

这能保证“lane 已切换，但中心线还在收敛”的尾部阶段不会丢样本。

---

## 6. 这轮采用的验收公式

下面这些公式已经落实在：

- `ramp/experiments/merge_centering.py`
- `ramp/tools/verify_merge_centering.py`

### 6.1 基础横向量

对每个时刻 `k`、每台车 `v`：

- `y_k(v) = getLateralLanePosition(v)`
- `w_k(v) = lane.getWidth(lane_id)`
- `d_k(v) = |y_k(v)|`
- `d_k_norm(v) = 2 * |y_k(v)| / w_k(v)`

并定义：

- `is_centered_0p25 = 1[d_k <= 0.25]`

这里最核心的量其实就是 `|y_k|`：

- `y = 0` 表示正在该车道中心线上
- `|y|` 越大，说明离中心线越远

---

### 6.2 目标走廊

定义：

```text
L_target = {
    (main_h3, 1),
    (:n_merge_0, 0),
    (main_h4, 0)
}
```

直观含义：

- ramp 车成功并入主线目标车道后，应该落在这条走廊里

---

### 6.3 到达目标走廊

对车辆 `v`：

```text
target_lane_reached(v) = 1[存在某个时刻 k，使得 (edge_k, lane_k) 属于 L_target]
```

这个指标回答的问题是：

- 它有没有真正走到我们定义的目标并道走廊里

注意：

- 这个指标只说明“到了”
- 不说明“居中了”

---

### 6.4 中心线收敛

绝对阈值：

```text
C_k(v) = 1[|y_k(v)| <= 0.25m]
```

持续保持窗口：

```text
center_hold_ok(v) = 1[
    存在连续窗口 W，且 |W| >= N，
    对所有 k 属于 W，都有 |y_k(v)| <= 0.25m
]
```

其中：

```text
N = ceil(1.0 / step_length)
```

如果 `step_length = 0.1s`，那么：

- `N = 10`

也就是说：

> 车辆必须连续 1.0 秒都保持在距离当前车道中心线 0.25m 以内，才算“真正收敛到中心线”。

这正是“第一辆匝道车到底有没有开到中心线”最核心的自动判据。

---

### 6.5 贴边异常

定义：

```text
edge_stick(v) = 1[
    存在连续 >= 0.5s 的样本，
    使得 |y_k| > w_k / 2 - 0.3
]
```

直观理解：

- 车已经接近车道边缘 0.3m 内，并且持续了至少 0.5 秒

当前实现只在目标走廊内计算 `edge_stick`，避免把正常换道过渡阶段误算成失败。

---

### 6.6 堵塞判定

当前自动验收里把以下任一情况视为 `blockage_flag = true`：

1. `main_h3_0` 的 `halting_count >= 2` 持续超过 `8s`
2. `main_h3_0` 的 `mean_speed < 3m/s` 持续超过 `10s`
3. `starting_teleport_count > 0`

实现上还做了一个修正：

- 空车道不应该因为 `mean_speed = 0` 被误判为拥堵
- 因此低速堵塞判定要求 `vehicle_count > 0`

---

### 6.7 首辆匝道车的定义

为了把验收重点固定下来，这轮定义了“first probe vehicle”：

主规则：

- 第一个出现 `zone_c_lc_command` 或 `zone_c_lc_issued` 的 ramp 车

回退规则：

- 如果没有记录到上述事件，则回退到第一个进入 `main_h3` 的 ramp 车

这一步是为了避免每次都人工指定“看哪辆车”。

---

## 7. 代码改动落点

本轮与这个问题直接相关的主要文件如下：

- `ramp/scenarios/ramp__mlane_v2_mixed/ramp__mlane_v2_mixed.sumocfg`
  - 默认开启 `lateral-resolution = 0.25`

- `ramp/experiments/run.py`
  - CLI 新增/默认 `--lateral-resolution 0.25`
  - 输出横向 trace、lane change lifecycle、merge health
  - 补充记录目标走廊中的车辆
  - 自动生成 `centering_report.*`
  - 把中心线验收指标写回 `metrics.json`

- `ramp/experiments/merge_centering.py`
  - 实现目标走廊、中心线收敛、贴边、堵塞、first probe 等验收逻辑

- `ramp/tools/verify_merge_centering.py`
  - 提供独立复验入口

- `ramp/runtime/controller.py`
  - lane change mode 从 `0` 改为 `2560`
  - 新增 post-merge 居中 nudge

- `ramp/common/vehicle_defs.py`
  - 显式加入 `latAlignment=center`

- `ramp/policies/hierarchical/merge_point.py`
  - 调整 lane-change command duration
  - 改善 retry 计时语义

- `ramp/tests/test_verify_merge_centering.py`
  - 覆盖正常收敛、贴边失败、无事件时的回退规则

- `ramp/tests/test_evidence_chain.py`
  - 覆盖中心线报告向 `metrics.json` 的投影等相关逻辑

---

## 8. 如何复现与验证

### 8.1 GUI 手动运行

下面这条是本轮常用的 GUI 运行方式之一：

```bash
cd /home/liangyunxuan/src/Sumo-Carla-simulation-for-Vehicle-Road-Cloud-Integeration

SUMO_GUI=1 uv run python -m ramp.experiments.run \
  --scenario ramp__mlane_v2_mixed \
  --policy hierarchical \
  --policy-variant strong_a_v1 \
  --duration-s 120 \
  --step-length 0.1 \
  --seed 42 \
  --generate-rou \
  --cav-ratio 0.5 \
  --main-vph 1500 \
  --ramp-vph 500 \
  --delta-1-s 1.5 \
  --delta-2-s 2.0 \
  --dp-replan-interval-s 0.5 \
  --lateral-resolution 0.25 \
  --out-dir output/stronga_accept/gui_seed42_manual \
  --gui
```

虽然现在 `.sumocfg` 和 `run.py` 默认值都已经带了 `0.25`，但在手动调试时，显式写出来更不容易混淆。

---

### 8.2 自动验收脚本

运行结束后，可以手动再次验证：

```bash
cd /home/liangyunxuan/src/Sumo-Carla-simulation-for-Vehicle-Road-Cloud-Integeration

uv run python -m ramp.tools.verify_merge_centering \
  --out-dir output/stronga_accept/gui_seed42_manual
```

这会重新生成：

- `centering_report.json`
- `centering_report.csv`

---

### 8.3 推荐看的输出文件

如果要排查“为什么 GUI 看着不对”，建议按这个顺序看：

1. `centering_report.json`
   - 先看首辆匝道车是否通过
   - 再看 `blockage_flag`

2. `metrics.json`
   - 看自动回写的几个核心验收字段

3. `lane_change_lifecycle.csv`
   - 看 `zone_c_lc_command / lane_id_switch / zone_c_lc_complete` 时刻的横向状态

4. `control_zone_trace.csv`
   - 看某台车进入目标走廊后，`lat_lane_pos_m` 是否在 1 秒窗口内收敛到 `[-0.25, 0.25]`

5. `merge_lane_health.csv`
   - 看是否伴随长时间低速、停车、teleport

---

### 8.4 测试

本轮相关测试当前通过情况：

- `ramp/tests/test_verify_merge_centering.py`
- `ramp/tests/test_evidence_chain.py`

合计：

- `33 passed`

---

## 9. 本轮最终达到了什么效果

根据当前已有输出目录中的报告，`seed = 1, 42, 99` 的核心结果如下：

### seed 1

- `first_probe_vehicle_id = ramp_R1_1`
- `first_probe_target_lane_reached = true`
- `first_probe_center_hold_ok = true`
- `global_center_hold_pass_rate = 1.0`
- `blockage_flag = false`
- `edge_stick_rate = 0.3`

### seed 42

- `first_probe_vehicle_id = ramp_R1_1`
- `first_probe_target_lane_reached = true`
- `first_probe_center_hold_ok = true`
- `global_center_hold_pass_rate = 1.0`
- `blockage_flag = false`
- `edge_stick_rate = 0.4166666666666667`

### seed 99

- `first_probe_vehicle_id = ramp_R1_1`
- `first_probe_target_lane_reached = true`
- `first_probe_center_hold_ok = true`
- `global_center_hold_pass_rate = 1.0`
- `blockage_flag = false`
- `edge_stick_rate = 0.5833333333333334`

可以把这轮的结果总结为：

1. “瞬移换道”这个最初问题已经被解决  
   因为现在 sublane 横向分辨率已经启用。

2. “首辆匝道车是否真正收敛到目标车道中心线”这个核心问题，当前在 `seed 1/42/99` 上都通过了  
   这是本轮最重要的验收结果。

3. “是否出现明显堵塞/teleport”在这三组结果里是通过的  
   `blockage_flag` 均为 `false`。

4. `edge_stick_rate` 仍然偏高  
   这说明“完全消除所有贴边样本”还没有彻底做完，或者这个指标的语义仍然偏严格。  
   换句话说：本轮已经把“瞬移”和“首辆匝道车是否回到中心线”解决到了一个可验证的状态，但如果把 `edge_stick_rate <= 0.02` 当成硬门槛，当前实现还没有完全达到。

---

## 10. 为什么 `max_abs_offset_m` 还能很大，但 `center_hold_ok` 仍然通过

这也是后续读报告时很容易困惑的一点。

例如首辆匝道车的 `max_abs_offset_m` 可能仍在 `1.55m` 左右，但 `center_hold_ok = true`。

这并不矛盾，因为：

- `max_abs_offset_m` 记录的是整个目标走廊阶段里出现过的最大绝对偏移
- 它会把“换道过渡阶段贴近边线”的瞬时峰值也算进去
- `center_hold_ok` 关心的是：之后是否出现了连续 1 秒、`|y| <= 0.25m` 的稳定窗口

所以：

- `max_abs_offset_m` 大，说明过程里曾经偏得很厉害
- `center_hold_ok = true`，说明它后来确实收敛回中心线了

这也是为什么本轮验收必须同时保留：

- “过程峰值”
- “最终是否稳定归中”

两个视角，而不能只看其中一个。

---

## 11. 后续如果还要继续优化，建议怎么做

如果未来要继续迭代，建议遵循下面的顺序，而不是直接盲改控制器：

1. 先看 `centering_report.json`
   - 首辆匝道车是否通过
   - 是否有堵塞

2. 再看 `lane_change_lifecycle.csv`
   - `zone_c_lc_command`
   - `lane_id_switch`
   - `zone_c_lc_complete`
   这几个时刻横向位置分别是什么

3. 再看 `control_zone_trace.csv`
   - 目标走廊阶段是否有完整样本
   - 是否真的缺少中心线收敛窗口

4. 如果 first probe 已通过，但 `edge_stick_rate` 仍偏高
   - 优先判断这是“控制还不够稳”还是“指标定义仍偏严”

5. 如果再次出现“看着像没居中”
   - 不要只看 `lane_id`
   - 一定要看 `lat_lane_pos_m`
   - 一定要用持续窗口判断，而不是单帧截图

---

## 12. 一句话总结本轮 debug

这轮并不是只加了一个 `--lateral-resolution 0.25` 就结束了，而是沿着下面这条链路把问题逐层拆开并工程化了：

```text
瞬移换道
-> 开启 sublane 横向细分
-> 暴露出“已并道但未归中”的第二层问题
-> 把 GUI 肉眼判断升级为中心线公式验收
-> 修补观测盲区，避免 false negative
-> 最后再做最小控制修复，让 post-merge 真正回到中心线
```

如果后续还要继续完善，这份文档可以作为下一轮 debug 的起点，而不是从“为什么 GUI 看起来怪怪的”重新开始。
