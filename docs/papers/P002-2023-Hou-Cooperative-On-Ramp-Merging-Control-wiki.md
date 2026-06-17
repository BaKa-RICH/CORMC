---
type: paper
paper_id: P002
title: "Cooperative On-Ramp Merging Control Model for Mixed Traffic on Multi-Lane Freeways"
authors: "Kangning Hou, Fangfang Zheng, Xiaobo Liu, Ge Guo"
year: 2023
venue: "IEEE Transactions on Intelligent Transportation Systems"
status: read
confidence: medium
source_path: raw/papers/P002-2023-Hou-Cooperative-On-Ramp-Merging-Control.md
zotero_key:
doi: 10.1109/TITS.2023.3274586
tags: [CAV, CHV, mixed-traffic, multilane-merging, cooperative-merging-control, APS, CUC]
last_updated: 2026-04-26
---

# P002: Mixed-Traffic Multilane Cooperative On-Ramp Merging

## 1. 一句话定位

这篇论文提出 CORMC：面向多车道 freeway on-ramp 的混合交通合流控制框架，把 CAV 与 connected human-driven vehicles 的异质性、主线车辆换道协作、驾驶人 compliance 和匝道车安全合流统一到一个分层分布式模型中。

## 2. 核心贡献

- `EXTRACTED` 提出 hierarchical distributed CORMC 框架，上层由 APS algorithm 和 CUC model 负责决策，下层由纵向、换道和 cooperative merging control 模型执行。
- `EXTRACTED` 在多车道主线中显式考虑主线协作车辆可以保持纵向运动或换道到相邻车道以给匝道车创造 gap。
- `EXTRACTED` 引入 CHV compliance rate，区分 compliant CHVs 与 non-compliant CHVs，从而评估驾驶人是否接受协作建议对系统性能的影响。
- `EXTRACTED` 设计 boundary-collision-avoidance 和 front-collision-avoidance 算法，避免匝道边界碰撞和换道过程前向碰撞。

## 3. 方法抓手

- APS：根据匝道车到达合流区起点的 anticipatory time，预测 lane 2 中候选车辆位置，为每个 merging vehicle 指定 anticipatory merging position、CLV 和 CFV。
- CUC：让协作车辆在“换到 lane 1”和“继续留在 lane 2”之间做 utility choice，效用同时考虑安全、对后车影响、加速度收益和舒适性。
- Lower layer：CAV 使用 stochastic CPID longitudinal model，CHV 使用 stochastic IDM；换道轨迹用 improved sine function，并由 MPC tracking controller 执行。
- CMC：用 dynamic time gap acceptance model 描述匝道车越接近匝道末端越降低可接受 gap 的强制合流行为。

## 4. 关键实验结论

- `EXTRACTED` 仿真平台为 MATLAB micro-simulation，场景是双主线车道 + 单匝道，主线 10 km，合流区 300 m，协作区随匝道车动态移动且通信范围为 300 m。
- `EXTRACTED` CAV penetration 设置为 0%-100%，on-ramp flow ratio 为 10%-40%，每个场景 1200s，并用 20 个随机种子取平均。
- `EXTRACTED` 当 CHV compliance rate 小于 50% 时，CUC 带来的性能收益较小；高于 50% 后改善更明显。
- `EXTRACTED` CAV penetration 低于 60% 时系统改善有限，高于 60% 时速度和流量提升更明显；高匝道流量、拥堵条件下收益更大。
- `EXTRACTED` 三种策略对比显示 APS 比 CUC 更关键：指定具体 cooperative vehicles 能显著提升 merging speed。
- `EXTRACTED` 相比 SUMO 默认 LC2013 行为，CORMC 能显著缓解 congestion wave propagation；在高 CAV penetration 和高 on-ramp ratio 下，吞吐与速度提升可超过 80%。

## 5. 局限与隐含假设

- 论文自述局限：
  - 未来需要扩展到超过两条主线车道，并考虑不同主线车道的流量不平衡和平衡速度差异。
  - 未来计划把框架扩展到多车道 off-ramp bottleneck。
- 你识别到的隐含假设：
  - 所有车辆都是 connected vehicles，可通过 V2V 获取实时状态，且不考虑通信延迟。
  - CHV compliance 被简化为遵从/不遵从建议，未充分建模人类驾驶人反应延迟、误解建议或部分执行。
  - 仿真由自建 MATLAB micro-simulation 完成，真实交通数据校准和外部验证仍需回 raw 或后续论文核查。

## 6. 关系线索

- extends: P001 的纯 CAV 单车道合流控制，扩展到 mixed traffic + multilane + mainline lane-changing cooperation。
- contrasts: SUMO LC2013 default lane-changing；只靠主线纵向减速创造 gap 的单车道合流方法。
- uses: [[wiki/concepts/mixed-traffic-multilane-cormc]], APS, CUC, CPID, IDM, MPC tracking, dynamic time gap acceptance。
- suggests_gap: CHV compliance 低时协同收益有限；多车道扩展、通信延迟、人类行为不确定性仍需进一步建模。
- asset_todo: 原始 MinerU 文件含多处外链图片，后续如需写作审计，应本地化到 `raw/assets/P002/`。

## 7. 对我研究的可能用途

- baseline: 可作为 mixed traffic + multilane on-ramp merging 的强规则/模型型 baseline。
- idea_source: 可启发把“提前指定协作车辆 + 主线换道协作 + 动态 gap acceptance”作为机制组合。
- counterexample: 反驳只研究纯 CAV 或只研究单车道纵向控制足以覆盖合流问题的设定。
- dataset_or_metric: CAV penetration、CHV compliance、on-ramp flow ratio、average flow/speed、congestion wave propagation。
- assumption: 全连接车辆、无通信延迟、双主线车道、CHV compliance 二值化、自建仿真平台。

## 8. 原文锚点

- raw: `raw/papers/P002-2023-Hou-Cooperative-On-Ramp-Merging-Control.md`
- zotero:
- doi: 10.1109/TITS.2023.3274586
- keywords: Connected automated vehicles; cooperative on-ramp merging; mixed traffic flow; multi-lane traffic; hierarchical framework.

## 9. 必要摘录

> `EXTRACTED` The upper-layer of the CORMC model employs APS to determine anticipatory positions and cooperative vehicles, while CUC determines whether cooperative vehicles should change lanes or keep longitudinal movement.

> `EXTRACTED` The performance benefits of CUC are marginal when CHV compliance is relatively low, but become significant at higher compliance rates.

> `EXTRACTED` The APS algorithm plays a more critical role than the CUC model because assigning a specific CV to the MV can significantly improve merging speed.

## 10. 回查触发点

- proof：需要论证“提前协作车辆分配为何能缓解合流冲击波”时，回查 `III. UPPER LAYER OF THE CORMC MODEL`、`A. Anticipatory Position Searching (APS) Algorithm`、`VI. RESULTS`。
- 实验设计：需要设置 CAV penetration、CHV compliance、on-ramp flow ratio、双主线车道仿真场景和观测指标时，回查 `V. NUMERICAL EXPERIMENTS`、`C. Scenario Settings and Parameters`、`A. Assessment Indicators of Traffic Efficiency`。
- baseline 复现：需要复现 APS、CUC、CMC、CPID/IDM、MPC tracking 或 SUMO 对比时，回查 `III. UPPER LAYER OF THE CORMC MODEL`、`IV. LOWER LAYER OF CORMC MODEL`、`E. Comparison With SUMO`。
- 写作：需要描述 mixed traffic + multilane gap、驾驶人 compliance 或主线换道协作贡献时，回查 `I. INTRODUCTION`、`VII. CONCLUSION AND FUTURE WORK`。
- citation audit：需要核对 DOI、IEEE TITS 元数据、LC2013、CPID、IDM、MPC tracking 引用链时，回查 `REFERENCES`；具体页码需回 raw 或 Zotero 核查。

## 11. 关键原文位置

- 论文题名、作者、DOI、摘要、关键词：开头 metadata、`Abstract`、`Index Terms`。
- 研究动机与贡献：`I. INTRODUCTION`。
- CORMC 总体框架和核心假设：`II. COOPERATIVE ON-RAMP MERGING CONTROL (CORMC) MODEL FRAMEWORK`。
- APS anticipatory position 与 cooperative vehicle assignment：`III. UPPER LAYER OF THE CORMC MODEL`、`A. Anticipatory Position Searching (APS) Algorithm`。
- CUC utility choice 和 compliance：`B. Collaborative Utility Choice (CUC) Model`。
- 纵向、换道和 MPC tracking：`IV. LOWER LAYER OF CORMC MODEL`、`A. Longitudinal and Lane Changing Models`。
- 匝道车 CMC、动态 gap acceptance 和边界防碰撞：`B. Cooperative Merging Control (CMC) Model`。
- 仿真设置、参数和车辆生成：`V. NUMERICAL EXPERIMENTS`。
- compliance、CAV penetration、子模型消融、SUMO 对比：`VI. RESULTS`。
- 局限与 future work：`VII. CONCLUSION AND FUTURE WORK`。
