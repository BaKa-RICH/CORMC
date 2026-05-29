# Cooperative On-Ramp Merging Control Model for Mixed Traffic on Multi-Lane Freeways

Kangning Hou Graduate Student Member, IEEE, Fangfang Zheng , Member, IEEE, Xiaobo Liu , and Ge Guo , Senior Member, IEEE 

Abstract— This paper proposes a hierarchical model for cooperative on-ramp merging control (CORMC) in mixed traffic with both connected automated vehicles (CAVs) and connected human-driven vehicles (CHVs). The upper-layer of the CORMC model employs an anticipatory position searching (APS) algorithm to determine the anticipatory positions at which merging vehicles (MVs) should merge from the on-ramp lane to the adjacent mainline lane, and to assign cooperative vehicles (CVs) for each MV. A collaborative utility choice (CUC) model is presented to determine the optimal maneuver of CVs to create proper gaps for MVs. The driver compliance rate is introduced to account for CHVs’ unwillingness to follow the instructions given by the CUC model. The lower-layer comprises a cooperative merging control (CMC) model that ensures safe and smooth merging execution for MVs. Longitudinal and lane changing models are developed for mainline vehicles to facilitate an efficient and safe merging process. Simulation results show that the performance benefits of the CUC model are marginal when the CHV compliance rate is relatively low. However, the performance improvement is significant at higher compliance rates $( > 5 0 \% )$ . Furthermore, the CORMC model has the potential to increase merging and mainline throughput by over $75 \%$ at sufficiently high CAV penetration rates. Comparison of three control strategies shows that the APS algorithm plays an important role in the CORMC model. A comparison with the Simulation of Urban Mobility (SUMO) indicates that the CORMC model significantly mitigates the propagation of congestion waves across varying levels of CAV penetration and on-ramp flow rates. 

Index Terms— Connected automated vehicles (CAVs), cooperative on-ramp merging, mixed traffic flow, multi-lane traffic, hierarchical framework. 

# I. INTRODUCTION

FREEWAY on-ramp merging areas are widely recognizedas major bottlenecks that significantly impact the safety, efficiency and reliability of freeway traffic operations. At the 

Manuscript received 23 June 2022; revised 21 December 2022, 17 March 2023, and 1 May 2023; accepted 1 May 2023. Date of publication 16 May 2023; date of current version 4 October 2023. This work was supported in part by the National Natural Science Foundation of China (NSFC) under Grant 52072315, Grant 61673321, and Grant 62173079. The Associate Editor for this article was X. Di. (Corresponding author: Fangfang Zheng.) 

Kangning Hou, Fangfang Zheng, and Xiaobo Liu are with the School of Transportation and Logistics, National Engineering Laboratory of Integrated Transportation Big Data Application Technology, and the National United Engineering Laboratory of Integrated and Intelligent Transportation, Southwest Jiaotong University, Chengdu, Sichuan 611756, China (e-mail: kangning.hou@my.swjtu.edu.cn; fzheng@swjtu.cn; xiaobo.liu@swjtu.cn). 

Ge Guo is with the State Key Laboratory of Synthetical Automation for Process Industries, Northeastern University, Shenyang 110819, China, and also with the School of Control Engineering, Northeastern University at Qinhuangdao, Qinhuangdao 066004, China (e-mail: geguo@yeah.net). 

Digital Object Identifier 10.1109/TITS.2023.3274586 

upstream of the freeway merges, the slow moving vehicles caused by merging vehicles (MVs) can propagate upstream to the mainline, resulting in congestion and an increased risk of crashes and conflicts [1], [2]. However, with the development of communication, sensing and control technologies, connected automated vehicles (CAVs) have the potential to alleviate congestion at freeway merging areas [3], reduce energy consumption and emissions [4], and enhance merging efficiency and safety [5]. Nonetheless, in the foreseeable future, freeway traffic will inevitably experience a mixed traffic state where both traditional human-driven vehicles (HVs) and CAVs coexist. Thus, it is of great significance to investigate efficient merging strategies for the mixed traffic flow on freeways. 

In recent years, considerable research has been devoted to on-ramp merging control algorithms for pure CAV traffic with objectives such as minimizing fuel consumption, travel time, passenger discomfort [6], [7], [8], [9], [10], or improving traffic safety and efficiency [11], [12]. For instance, Ntousakis et al. [7] present a longitudinal trajectory planning method to assist the merging of vehicles on freeways, using a scenario of 5 vehicles with a single main lane to demonstrate the validity of their method. Scholte et al. [13] propose a cooperative strategy for a single AV merging into a vehicle platoon. However, their proposed method does not explicitly consider the merging sequence of multiple vehicles and therefore cannot adequately address the situation of multiple AVs merging into the platoon. Unlike the above studies, which mainly focus on merging trajectory planning and control, Jing et al. [9] propose a cooperative multi-player game-based optimization framework to globally optimize the merging sequence and trajectory for each vehicle. The simulation results demonstrate that their proposed model can reduce fuel consumption and travel time. Later, Chen et al. [11] develop a hierarchical control approach for efficient and safe merging operations of CAVs, where a tactical layer controller determines the optimal vehicle merging sequence, while an operational layer controller is designed based on model predictive control (MPC) to optimize the desired acceleration of CAVs. 

Previous studies have developed approaches and algorithms for the cases with a single main lane, which do not consider lane-changing decisions and maneuvers for the mainline traffic. However, when on-ramp CAVs merge into a multilane freeway section, vehicles in the main lanes have the option to keep longitudinal movement or change lanes. Algorithms that only allow vehicles to keep longitudinal 

movement in their original lane cannot fully utilize the capacity of the other lanes, thereby reducing merging efficiency. To address this limitation, researchers propose cooperative merging control algorithms for multilane merging areas to increase throughput, reduce fuel consumptions, delays or travel times [14], [15], [16], [17], [18], [19]. 

Notably, most existing studies focus on pure CAV traffic where all vehicles are fully controllable. Limited research has investigated merging control in mixed traffic with CAVs and HVs. Wei et al. [20] develop a merging decision model that considers the interaction between the merging vehicle and the following vehicle in the main lane. Though the heterogeneity of AVs in terms of different driving styles is considered, the heterogeneity of HVs and their interactions are not explicitly taken into account. Pueboobpaphan et al. [21] consider mixed traffic with manual vehicles and cooperative adaptive cruise control (CACC) vehicles in the mainline and propose a merging assistant algorithm to control the mainline vehicles to create proper gaps for the on-ramp manual vehicles. However, their strategy cannot align the on-ramp manual vehicle with the assigned gap, resulting in only marginal improvement of efficiency (e.g., vehicle km-traveled and average travel time) for a given percentage of CACC vehicles. Karimi et al. [22] propose cooperative merging control algorithms for triplets of vehicles under different combinations of CAVs and HVs, with an optimal trajectory determined for each vehicle in the triplet considering safety and comfort. Similarly, Sun et al. [23] develop a bi-level cooperative control model that considers interactions between HVs and CAVs for a triplet of vehicles in a single main lane and one on-ramp lane, and validate their model’s effectiveness with microscopic simulation. Mu et al. [24] present an event-triggered rolling horizon-based trajectory planning approach for the merging process of two platoons, aiming to improve traffic efficiency and smoothness, instead of independently addresses each individual merging maneuver. 

Most state-of-the-art studies focus on pure CAV environment or mixed traffic situations where the merging area comprises only a single main lane and an on-ramp lane. Consequently, lane-changing decisions and maneuvers are not considered for mainline traffic, and the heterogeneity of HVs and CAVs in the mixed traffic flow is not fully addressed. Although some studies [14], [15], [16], [17], [18], [19] have dealt with the merging scenarios involving multilane situations, they primarily focus on optimizing the longitudinal trajectories of mainline vehicles using global optimization methods that require centralized controllers or road units to regulate the speed of each vehicle, leading to high computational and communication costs. Moreover, while lane-changing advice strategies for mainline vehicles are used to facilitate the merging process in [14], [15], [16], [17], and [18], the lane-changing maneuvers are determined without considering their impact on the vehicles upstream of the target lane, potentially affecting the safety of the vehicles upstream of the target lane. 

To address the aforementioned issues, this paper proposes a hierarchical distributed framework for the cooperative on-ramp merging control (CORMC) model for mixed traffic 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/4bc2e3f74c1cd888c4ba2c811479769f11f94ce4af5525af818a847ee6114255.jpg)



Fig. 1. The scenario of a freeway section with two main lanes in one direction and an on-ramp.


on multi-lane freeway merging sections, based on our previous works [25], [26], [27]. The main contributions of this study are as follows: (1) The proposed hierarchical distributed control approach ensures efficient and safe merging of vehicles in the multi-lane merging zone under mixed traffic environments, explicitly considering the lane changing maneuvers in the main lanes. (2) Two collision-avoidance algorithms, the boundarycollision-avoidance and front-collision-avoidance algorithms, are developed to ensure collision-free merging and lane changing processes. (3) A vehicle generation model is designed to describe the stochastic properties of the mixed traffic flow, taking into account the heterogeneity of HVs in terms of the desired speed and the heterogeneity of CAVs in terms of inertial lag. (4) The driver compliance to the suggestions provided by the proposed model is explicitly considered, and thus the impact of driver behavior on the performance of the proposed control approach can be effectively evaluated. 

The remainder of the paper is organized as follows. Section II presents the proposed CORMC model framework, which is formulated as a hierarchical control framework comprising the upper decision-making layer and the lower control layer. Sections III and IV present the upper and lower layers of the CORMC model, respectively. In section V, a vehicle generation model is presented, and scenario settings and model parameters are given. In section VI, the simulation results are analyzed and discussed. Section VII concludes the paper, and provides some prospects for future work. 

# II. COOPERATIVE ON-RAMP MERGING CONTROL (CORMC) MODEL FRAMEWORK

This study considers a freeway section consisting of two main lanes in one direction and an on-ramp lane, as shown in Fig. 1. The section is divided into three areas: the mainline, cooperative and merging zones. The red vehicles represent CAVs, while the blue vehicles represent HVs. In the cooperative zone, the vehicle in the on-ramp lane searches for an anticipated merging position in lane 2. The on-ramp vehicle then cooperates with the vehicles in front or behind the anticipated merging position to reserve a safe merging spacing. In the merging zone, the on-ramp vehicle performs the merging maneuver until it reaches the center of lane 2. 

Before presenting the framework of the entire CORMC model, some assumptions are made as follows: 

1) All vehicles on all lanes are connected vehicles, and can communicate with each other and receive real-time state information via Vehicle-to-Vehicle (V2V) communication without considering communication delays. Throughout this paper, all HVs refer to Connected Human-Driven Vehicles (CHVs). 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/55354ca842373e789abcaadf70a1f7b171e9c05dd7c123861dfa87491a811db2.jpg)



Fig. 2. Hierarchical framework of the CORMC model.


2) The behavior of CAVs is assumed to be fully controllable and deterministic, whereas CHVs are uncontrollable. In this study, CHVs are classified into two categories: compliant and non-compliant CHVs. The former adopts favorable suggestions given by the proposed model, while the latter does not. Thus, the mixed traffic flow consists of CAVs, compliant CHVs and noncompliant CHVs. It is noteworthy that non-compliant CHVs can be regarded as HVs lacking V2V communication. 

Fig. 2 illustrates the proposed hierarchical framework of the CORMC model. The framework is distributed, meaning that each vehicle is controlled by its own controller, and multi-vehicle cooperative control is achieved through the information exchange between the vehicles. The upper layer of the framework consists of two parts: the anticipatory position searching (APS) algorithm and the collaborative utility choice (CUC) model. The APS algorithm determines the anticipatory positions at which the merging vehicles (MVs) merge from the on-ramp lane to lane 2 (as shown in fig. 1), and also determines the cooperative vehicles (CVs) in lane 2 for each MV. The CUC model allows the CVs in lane 2 determined by the APS algorithm to make the best choice between maintaining longitudinal movement and changing lanes. At the lower layer, the vehicle status, including position, speed, and acceleration, is updated for each time step using the longitudinal and lane-changing models based on the output decisions from the upper layer. The updated vehicle status information is then feedback to the upper layer to make decisions for the next time step. For the mainline vehicles, the microscopic trajectories are updated using the longitudinal and lane-changing models. For the MVs, a cooperative merging control (CMC) model is developed to coordinate the movements of vehicles in the on-ramp with vehicles in the adjacent lane to achieve a safe and efficient merging process. 

# III. UPPER LAYER OF THE CORMC MODEL

# A. Anticipatory Position Searching (APS) Algorithm

In order to ensure safe and efficient merging of MVs onto the main road (lane 2), it is crucial to determine the 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/73a9088ff478893b3f56319ed21d6bfab4b0e98dd67d9c179249872c01d4ea9f.jpg)



Fig. 3. Schematic illustration of the merging area including the on-ramp and lane 2.


anticipatory position of the MV in lane 2 in advance. The two vehicles in front and behind the anticipated merging position, called cooperative vehicles (CVs), then cooperate to reserve the safe merging spacing for the MV. To achieve this, we have developed an APS algorithm which judges the MV’s suitable position and assigns CVs for each MV. 

Fig. 3 illustrates the merging zone, including the on-ramp and lane 2. We define the set of vehicles which may create a merging gap for the MV at time t as: 

$$
G a p \_ c a r _ {M V} (t) = \left[ G _ {1}, G _ {2}, \dots , G _ {n (t)} \right] \tag {1}
$$

with constraints: 

$$
\left\{ \begin{array}{l} x _ {j} (t) \in \left[ x _ {M V} (t) - L ^ {c r}, x _ {M V} (t) + L ^ {c r} \right], j \in G a p \_ c a r _ {M V} (t) \\ x _ {G _ {1}} (t) > x _ {G _ {2}} (t) > \dots > x _ {G _ {n (t)}} (t) \end{array} \right. \tag {2}
$$

where $G _ { j }$ represents vehicle $j$ within vehicle set $G a p \_ c a r _ { M V }$ . $n ( t )$ represents the total number of vehicles that may produce a merging gap for the MV at time t . $x _ { j } ( t )$ and $x _ { M V } ( t )$ represent the longitudinal positions of vehicles $j$ and the MV at time $t$ , respectively. $L ^ { c r }$ represents the length of the communication range, and all vehicles in $G a p \_ c a r _ { M V }$ are within the communication range $L ^ { c r }$ of the MV. In this study, $L ^ { c r }$ is set to $3 0 0 \mathrm { m }$ . 

The core idea of the APS algorithm is that the MV calculates the anticipatory arrival positions of all $G a p \_ c a r _ { M V }$ vehicles traveling at their current speed for time $T _ { M V } ^ { * } \left( t \right)$ . $T _ { M V } ^ { * }$ (t ) represents the anticipatory time required for the MV to travel at a constant speed of $v _ { M V } ( t )$ from the current position to the start point of the merging zone at time t . The anticipatory longitudinal spacing $\mathcal { D } _ { j } ^ { * } ( t )$ between the anticipatory arrival position of vehicle $j$ and the start point of the merging zone at time $t$ is then calculated as: 

$$
\mathcal {D} _ {j} ^ {*} (t) = x _ {j} ^ {*} (t) - x _ {0} ^ {m} - L _ {j} \tag {3}
$$

$$
x _ {j} ^ {*} (t) = v _ {j} (t) \cdot T _ {M V} ^ {*} (t) + x _ {j} (t), j \in G a p \_ c a r _ {M V} (t) \tag {4}
$$

$$
T _ {M V} ^ {*} (t) = \frac {x _ {0} ^ {m} - x _ {M V} (t)}{v _ {M V} (t)} \tag {5}
$$

$$
\mathcal {D} _ {M V} (t) = \left[ \mathcal {D} _ {1} ^ {*} (t), \mathcal {D} _ {2} ^ {*} (t), \dots , \mathcal {D} _ {j} ^ {*} (t), \dots , \mathcal {D} _ {n (t)} ^ {*} (t) \right] \tag {6}
$$

where $x _ { 0 } ^ { m }$ represents the longitudinal coordinate of the start point of the merging zone. $x _ { j } ^ { * } \left( t \right)$ represents the anticipatory position of vehicle $j$ after traveling at a constant speed of $v _ { j } ( t )$ for a time period of $T _ { M V } ^ { * } ( t ) . \mathcal { D } _ { j } ^ { * } \left( t \right) > 0$ indicates that vehicle $j$ travels past the start point of the merging zone after time $T _ { M V } ^ { * } ( t )$ while the MV is at the start point of the merging zone. $L _ { j }$ is the length of vehicle $j . \mathcal { D } _ { M V } ( t )$ is the set of anticipatory longitudinal spacings of all vehicles in $G a p _ { c a r M V } ( t )$ at time $t$ . 

If the conditions that $\mathcal { D } _ { j } ^ { * } \left( t \right) > 0$ and $\mathcal { D } _ { j + 1 } ^ { * } \left( t \right) < 0$ are satisfied, it indicates that the vehicles preceding vehicle $j$ have 

passed the starting point of the merging zone by the time the MV reaches it, while the vehicles behind vehicle $j$ are all located upstream of the starting point of the merging zone. Then, vehicle $j$ is designated as the anticipatory cooperative leading vehicle (CLV), and the vehicle that follows vehicle $j$ is designated as the anticipatory cooperative following vehicle (CFV). Correspondingly, the position between the CLV and CFV is considered as the anticipatory position of the MV in lane 2. 

Let ${ \mathcal { D } } _ { C L V } ^ { * } ( t )$ and ${ \mathcal { D } } _ { C F V } ^ { * } ( t )$ denote the anticipatory longitudinal spacings of the CLV and CFV at time t, respectively. Let $D _ { m i n } ^ { \bar { C } L V } ( t )$ and $D _ { m i n } ^ { C F V } ( t )$ denote the minimum acceptable spacing between the MV and CLV, the minimum acceptable spacing between the MV and CFV at time $t$ , respectively. The constraints for the anticipatory longitudinal spacings of the CLV and CFV are given by: 

$$
\left\{ \begin{array}{l} \mathcal {D} _ {C L V} ^ {*} (t) \geq \mathcal {D} _ {\text {m i n}} ^ {C L V} (t) \\ | \mathcal {D} _ {C F V} ^ {*} (t) | \geq \mathcal {D} _ {\text {m i n}} ^ {C F V} (t) \end{array} \right. \tag {7}
$$

$$
\left\{ \begin{array}{l} \mathcal {D} _ {\text {m i n}} ^ {C L V} (t) = v _ {M V} (t) \cdot g _ {\text {m i n}} ^ {C M} \\ \mathcal {D} _ {\text {m i n}} ^ {C F V} (t) = v _ {C F V} (t) \cdot g _ {\text {m i n}} ^ {C M} \end{array} \right. \tag {8}
$$

where $v _ { M V } \left( t \right)$ and $v _ { C F V } ( t )$ represent the speeds of the MV and CFV at time $t$ , respectively. $g _ { m i n } ^ { C M }$ represents the anticipatory minimum acceptable time gap of cooperative merging. 

The collaborative attribute of CVs is given by: 

$$
c o l = \left\{ \begin{array}{l l} 0, & \text {n o c o l l a b o r a t i o n i s r e q u i r e d} \\ 1, & \text {c o l l a b o r a t i o n i s r e q u i r e d} \end{array} \right. \tag {9}
$$

where 0 and 1 indicate that the coordinated maneuver is not required and coordinated maneuver is required, respectively. According to the constraints in Eq. (7), we can divide the cooperative situation into four cases. 

Case 1: Both constraints in Eq. (7) are satisfied. 

In this case, when the MV travels to the start point of the merging zone, the distance between the CFV and CLV ensures that the MV can merge into lane 2 smoothly and safely if all three vehicles drive at the current speed. Therefore, no collaborative maneuvers are needed and the collaborative attributes $c o l _ { C L V } = 0$ and $c o l _ { C F V } = 0$ . 

Case 2: Only the first constraint in Eq. (7) is satisfied. 

In this case, the anticipatory longitudinal spacing between the MV (at the start point of the merging zone) and CFV is smaller than the minimum acceptable spacing. Therefore, the CFV needs to perform collaborative maneuvers based on the CUC model, which will be introduced in detail in section III-B. The collaborative attributes $c o l _ { C L V } = 0$ and $c o l _ { C F V } = 1$ . 

Case 3: Only the second constraint in Eq. (7) is satisfied. 

In this case, the anticipatory longitudinal spacing between the CLV and MV (at the start point of the merging zone) is smaller than the minimum acceptable spacing. Therefore, the CLV needs to perform collaborative maneuvers based on the CUC model. The collaborative attributes $c o l _ { C L V } = 1$ and $c o l _ { C F V } = 0$ . 

Case 4: Both constraints in Eq. (7) are unsatisfied. 

In this case, both the CLV and CFV need to perform collaborative maneuvers based on the CUC model. The collaborative attributes $c o l _ { C L V } = 1$ and $c o l _ { C F V } = 1$ . 

In cases 2 and 4, to ensure that the CFV can adjust the required merging gap for the MV to satisfy both constraints in Eq. (7), the desired car-following spacing of the CFV is calculated according to: 

$$
S _ {C F V} = \mathcal {D} _ {\text {m i n}} ^ {C F V} (t) + L _ {M V} + \max  \left(\mathcal {D} _ {\text {m i n}} ^ {C L V} (t), \mathcal {D} _ {C L V} ^ {*} (t)\right) \tag {10}
$$

In case 3, although the spacing between the MV and CLV does not satisfy the first constraint in Eq. (7), the MV can adjust the spacing by considering the CLV as its leading vehicle such that the constraint can be satisfied. In this case, it is unnecessary to calculate the desired spacing for the CLV using Eq. (10). Likewise, in case 1, there is no need to calculate the desired spacing for the CLV using Eq. (10) as both constraints of Eq. (7) are met. It is worth noting that the APS algorithm may yield a different pair of CVs for an MV at every time step, resulting in a different cooperative merging case. Such frequent updates of CVs and switching between different cases may cause traffic flow instability in the mainline. Fixing the pair of CVs and the merging case once determined for the first time may be inappropriate in the future since the traffic status changes over time. To address this, we introduce a decision time interval, denoted as $T ^ { A P S }$ , which determines how often the APS algorithm updates the pair of CVs and the merging case for each MV based on real-time traffic information every $T ^ { A P S }$ time. The pseudocode of the APS algorithm is summarized in Algorithm 1. 

# B. Collaborative Utility Choice (CUC) Model

In the proposed APS algorithm, a pair of CVs (i.e., a CLV and a CFV) and a cooperative merging case are assigned specifically for each MV every $T ^ { A P \bar { S } }$ time. Once these are determined, it is necessary to control or provide suggestions to the CLV and CFV based on their collaborative attribute col to maintain a safe merging spacing for the MV. 

Existing on-ramp merging models in the literature [28], [29], [30] are only applicable to scenarios where the main road has a single lane. In such cases, the CV can only accelerate or decelerate on the current lane to provide sufficient merging space for the MV. However, the impact of MV deceleration on upstream vehicles can take a long time to dissipate, which reduces the road’s traffic capacity and increases the risk of rear-end collision between the CV and the vehicle behind it [31]. When the main road has more than one lane, such as two lanes, the CV has two choices to cooperate with the MV: 1) Change from lane 2 to lane 1; 2) Continue driving in lane 2. In this case, the methods mentioned in the literature [28], [29], [30] are no longer applicable. Therefore, we propose a CUC model for cooperative merging control in multilane freeway merging areas. The CUC model is a discrete choice model that enables the CV to make the best choice between maintaining longitudinal movement and changing lanes based on the utility. This way, the vehicles on the mainline are encouraged 


Algorithm 1 Anticipatory Position Searching (APS) Algorithm


DEFINE $t = 0$ $g_{min}^{CM} = 1.2s$ FOR each merging vehicle (MV) within the on-ramp DEFINE $t_{MV,APS}^{0}$ as the initial decision time of the APS algorithm of MV IF $\left(t - t_{MV,APS}^{0}\right)\mathrm{MODT}^{APS}\equiv 1$ FOR each vehicle (i) within lane 2 IF $x_{i}(t)\leq x_{MV}(t) + L^{cr}$ AND $x_{i}(t)\geq x_{MV}(t) - L^{cr}$ $i\in Gap_{carMV}(t)$ END IF $Gap_{carMV}(t) = [G_1,G_2\dots ,G_n]$ END FOR $T_{MV}^{*}(t) = \frac{x_{0}^{m} - x_{MV}(t)}{v_{MV}(t)};$ FOR each vehicle $j\in Gap_{carMV}(t)$ $x_j^* (t) = v_j(t)\cdot T_M^* (t) + x_j(t);$ $\mathcal{D}_j^* (t) = x_j^* (t) - x_0^m -l;$ END IF $\mathcal{D}_{MV}(t) = [\mathcal{D}_1^* (t),\mathcal{D}_2^* (t),\dots ,\mathcal{D}_j^* (t)];$ IF $\mathcal{D}_j^* (t) > 0$ AND $\mathcal{D}_{j + 1}^{*}(t) <   0$ $j$ is the CLV AND the vehicle that follows vehicle $j$ is the CFV; END IF $\mathcal{D}_{min}^{CLV}(t) = v_{MV}(t)\cdot g_{min}^{CM};$ $\mathcal{D}_{min}^{CFV}(t) = v_{CFV}(t)\cdot g_{min}^{CM};$ IF $\mathcal{D}_{CLV}^{*}(t)\geq \mathcal{D}_{min}^{CLV}(t)$ AND $|\mathcal{D}_{CFV}^{*}(t)|\geq \mathcal{D}_{min}^{CFV}(t)$ CASE 1: colCLV $= 0$ AND colCFV $= 0$ ELSE IF $\mathcal{D}_{CLV}^{*}(t)\geq \mathcal{D}_{min}^{CLV}(t)$ AND $|\mathcal{D}_{CFV}^{*}(t)|$ $\langle \mathcal{D}_{min}^{CFV}(t)$ CASE 2: colCLV $= 0$ AND colCFV $= 1$ SET the virtual vehicle MV'; ELSE IF $\mathcal{D}_{CLV}^{*}(t) <   \mathcal{D}_{min}^{CLV}(t)$ AND $|\mathcal{D}_{CFV}^{*}(t)|\geq$ $\mathcal{D}_{min}^{CFV}(t)$ CASE 3: colCLV $= 1$ AND colCFV $= 0$ ELSE IF $\mathcal{D}_{CLV}^{*}(t) <   \mathcal{D}_{min}^{CLV}(t)$ AND $|\mathcal{D}_{CFV}^{*}(t)|$ $\langle \mathcal{D}_{min}^{CFV}(t)$ CASE 4: colCLV $= 1$ AND colCFV $= 1$ SET the virtual vehicle MV'; END IF END IF END FOR $t = t + time\_step;$ 

to leave more space upstream of the on-ramp to reduce merging conflicts, improving the efficiency of the entire merging process. To highlight the different characteristics of CHVs and CAVs, we make the following statements. 

1) Although CHVs can receive instructions determined by the CUC model, not all of them are willing to follow these instructions. Hence, the driver compliance rate is introduced to account for this phenomenon. Two types of CHVs are classified according to the compliance, namely the compliant and non-compliant CHVs. 

2) The compliance rate $C$ of CHVs is calculated as the percentage of compliant CHVs among all CHVs. When 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/c0d3b452e72c020586534b05e282a19da7a9fafa4dc6798516ab288b108502df.jpg)



Fig. 4. Illustration of on-ramp cooperative system.


$C$ equals to 0, all CHVs are non-compliant CHVs, which can also be referred to HVs. When $C$ equals to 1, all CHVs are assumed to fully comply with the suggestions given by the CUC model. 

3) The non-compliant CHVs do not consider the suggestions given by the CUC model and continue to drive in their original states, while the CAVs and compliant CHVs fully comply with the CUC model within the cooperative and merging zones. Although both CAVs and compliant CHVs adhere to the guidelines of the CUC model, they exhibit different longitudinal behaviors. As such, this study considers the distinction between CAVs and CHVs in mixed traffic flow from both the longitudinal and lateral behavior perspectives. 

As shown in Fig. 4, the TLV and TFV represent the leading and following vehicles of the CV in the target lane (lane 1), respectively. The CV can be the CLV or CFV defined in section III-A. The FV and LV represent the leading and following vehicles of the CV in the current lane, respectively. 

During the cooperative process, the CV faces two choices, described as follows: 

Choice 1: The CV changes from lane 2 to lane 1, where the TFV and CV form a subsystem, and the CV and TLV form another subsystem. The new accelerations of the TFV and CV are denoted by $\tilde { a } _ { T F V } ^ { C V }$ and $\tilde { a } _ { C V } ^ { T L V }$ , respectively. The TLV, CV, and TFV form a lane-changing demand system. 

Choice 2: The CV continues to drive in lane 2, where the FV and CV form a subsystem, and the CV and LV form another subsystem. The new accelerations of the FV and CV are denoted by $\tilde { a } _ { F V } ^ { C V }$ and $\tilde { a } _ { C V } ^ { L V }$ , respectively. The LV, CV and FV form a no lane-changing demand system. 

The CUC model assumes that the CV considers not only its own safety but also the impact of each choice on the upstream vehicles, as well as the acceleration and comfort benefits that can be obtained through the choice during the process of collaborative choice making. Therefore, the CV makes the choice with higher overall system benefits. 

The utility functions of the two choices are given by: 

$$
\begin{array}{l} U _ {1} (t) = \alpha \cdot \left(c _ {C V} ^ {T L V} (t) + c _ {T F V} ^ {C V} (t)\right) + \beta \cdot \tilde {a} _ {T F V} ^ {C V} (t) \\ + \gamma \cdot \tilde {a} _ {C V} ^ {T L V} (t) + \zeta \cdot \left| \tilde {a} _ {C V} ^ {T L V} (t) - a _ {C V} ^ {L V} (t) \right| \tag {11} \\ \end{array}
$$

$$
\begin{array}{l} U _ {2} (t) = \alpha \cdot (c _ {C V} ^ {L V} (t) + c _ {F V} ^ {C V} (t)) + \beta \cdot \tilde {a} _ {F V} ^ {C V} (t) \\ + \gamma \cdot \tilde {a} _ {C V} ^ {L V} (t) + \zeta \cdot \left| \tilde {a} _ {C V} ^ {L V} (t) - a _ {C V} ^ {L V} (t) \right| \tag {12} \\ \end{array}
$$

$$
c _ {f} ^ {l} (t) = \frac {\left(v _ {f} (t) - v _ {l} (t)\right) ^ {2}}{x _ {l} - x _ {f} - L _ {l}} \tag {13}
$$

where $U _ { 1 } ( t )$ and $U _ { 2 } ( t )$ represent the utilities of the two choices available to the CV at time $t$ , respectively. The acceleration of 

the CV at time $t$ is denoted by $a _ { C V } ^ { L V } ( t ) . c _ { f } ^ { l } ( t )$ is the deceleration rate that vehicle $f$ needs to be taken to avoid crash relative to vehicle $l$ at time $t$ according to [32], [33], and [34], which serves as a safety measure index. Here, $l \in \{ C V , L V , T L V \}$ and $f { \in \{ } C V , T F V , F V \}$ . The smaller the value of $c _ { f } ^ { l } ( t )$ , the safer it is. α, β , γ , and $\zeta$ are weight coefficients, where $\alpha \ \textless \ 0$ , $\beta ~ > ~ 0$ , $\gamma ~ > ~ 0$ and $\zeta ~ < ~ 0$ . The first terms of Eqs. (11) and (12) denote the safety considerations of the CV and the following vehicle, with a larger value indicating greater safety. The second terms reflect the degree of influence on the following vehicle, with a smaller value indicating a higher impact. The third terms denote the acceleration benefits that the CV can obtain from its choices, with a higher value indicating a higher potential speed. The fourth terms represent the difference between the current and new accelerations of the CV, characterizing the driving comfort. The smaller the value of the fourth term, the more comfortable it is. 

When the CV chooses choice 2, the following safety constraints need to be considered: 

$$
\left\{ \begin{array}{l} T T _ {C V} ^ {T L V} (t) \geq T T _ {\min } \\ T T _ {T F V} ^ {C V} (t) \geq T T _ {\min } \end{array} \right. \tag {14}
$$

with 

$$
\begin{array}{l} T T _ {f} ^ {l} (t) = \frac {- \Delta V _ {f} (t) \mp \sqrt {\Delta V _ {f} (t) ^ {2} + 2 \Delta a _ {f} (t) d _ {f} (t)}}{\Delta a _ {f} (t)}, \\ l \in \{T L V, C V \}, f \in \{C V, T F V \} \tag {15} \\ \end{array}
$$

where $T T$ is the time that a following vehicle would take to collide with a leading vehicle if both vehicles maintain their current speed and acceleration [32], [33]. Generally, the value of TT below 1.5s is unsafe. Therefore, the value of $T T _ { m i n }$ is set to be 1.5s [33]. The final $T T$ value is obtained as follows: 1) If both $T T$ values are positive, the smaller one is chosen as the final value; 2) If one $T T$ value is positive and the other is negative, the positive one is chosen as the final value. 

Let $\mathbf { M } _ { C V }$ denote the choice of the CV, the cooperative decision of the CV can be obtained as: 

$$
\mathrm {M} _ {C V} = \left\{ \begin{array}{l} \text {C h o i c e 1 , U} _ {1} (t) > U _ {2} (t) \text {a n d E q . (1 4) i s s a t i f i e d} \\ \text {C h o i c e 2 , U} _ {1} (t) \leq U _ {2} (t) \end{array} \right. \tag {16}
$$

# IV. LOWER LAYER OF CORMC MODEL

A. Longitudinal and Lane Changing Models 

1) Longitudinal Models: 

a) Longitudinal model of $C A V s$ : We consider a subsystem consisting of two adjacent vehicles, in which the following vehicle i performs longitudinal movement and follows the preceding vehicle $i - 1$ safely according to the car following model. Let xi (t ), vi (t ), $S _ { i }$ (t), di (t) and $C _ { i } \left( t \right)$ denote the longitudinal position, speed, desired spacing, actual spacing and collision avoidance spacing of vehicle i at time t, respectively. The kinematic relationship between the two vehicles in the subsystem is shown in Fig. 5, we can obtain: 

$$
d _ {i} (t) = x _ {i - 1} (t) - x _ {i} (t) - L _ {i - 1} \tag {17}
$$

$$
S _ {i} (t) = v _ {i} (t) h _ {i} + d _ {0} + C _ {i} (t) \tag {18}
$$

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/5d47b1295fd81b4c69f74bc7f5a26e04162b7d454eea66f9eaee8f7aa555a10f.jpg)



Fig. 5. Illustration of the kinematic relationship between vehicles in the subsystem.


where $x _ { i - 1 } \left( t \right)$ represents the longitudinal position of vehicle i-1 at time t . $L _ { i - 1 }$ and $L _ { i }$ represent the length of vehicle $i .$ - 1 and vehicle $i$ , respectively. $h _ { i }$ represents the time gap of vehicle i . $d _ { 0 }$ is the stopping spacing. 

As for the CAV’s car-following model, the collision avoidance spacing $C _ { i } \left( t \right)$ is given by: 

$$
C _ {i} (t) = \left\{ \begin{array}{l l} \frac {\left(v _ {i} (t) - v _ {i - 1} (t)\right) ^ {2}}{2 \left| a _ {\min } \right|}, & v _ {i} (t) > v _ {i - 1} (t) \\ 0, & o t h e r s \end{array} \right. \tag {19}
$$

where $a _ { m i n }$ represents the minimum acceleration of vehicle $i$ . To ensure safety, the speed of vehicle i must be less than or equal to the speed of vehicle i-1. If the speed of vehicle i is higher than the speed of vehicle i-1, a collision may occur. Therefore, we introduce a collision avoidance spacing for the desired spacing, which is essentially the distance traveled by vehicle i when it decelerates at the maximum deceleration until its speed equals that of vehicle i-1. The larger the speed difference between the two vehicles, the larger the collision avoidance spacing is required. 

To perform the longitudinal control of CAVs, we consider two possible driving modes. 

1) The cruising mode, under which the CAVcan maintain the desired speed when the preceding vehicle is absent or the actual time gap is more than two times of the desired time gap [3]. 

2) The gap-regulating mode, which is used to maintain a constant time gap of the CAV with its preceding vehicle under the car-following condition. 

In the cruising mode, we can obtain the vehicle acceleration as: 

$$
a _ {i} (t) = k _ {1} \left(v _ {e} - v _ {i} (t)\right) \tag {20}
$$

where $k _ { 1 }$ represents the control gain parameter, and the value of $k _ { 1 }$ is set to be $0 . 4 s ^ { - 1 }$ according to [35]. $v _ { e }$ represents the equilibrium speed. $a _ { i } \left( t \right)$ and $v _ { i } \left( t \right)$ represent the acceleration and the velocity of vehicle $i$ at time t ,respectively. 

In the gap-regulating mode, we use the Cascade Proportional Integral Derivate (CPID) model, which is an acceleration model, to control the longitudinal maneuver of CAVs. The CPID model combines an inner loop controlling the velocity error and an outer loop controlling the spacing error with objectives of reducing both errors to zero. In our previous work, we demonstrated the strong stability and good performance of the CPID model [27]. To account for the heterogeneity of CAVs, we introduce a random variable $\tau ( \omega )$ to represent the inertial lag. We define the random parameter $\phi ( \omega ) \equiv \tau ( \omega )$ with distribution function $F _ { \phi }$ . The inertial lag $\tau _ { i }$ for each CAV i is drawn independently from this common 

distribution: $\phi _ { i } \sim F _ { \phi }$ .The mathematical formulations of the stochastic CPID model are given by: 

$$
e x _ {i} (t) = d _ {i} (t) - S _ {i} (t) \tag {21}
$$

$$
e v _ {i} (t) = v _ {i - 1} (t) - v _ {i} (t) \tag {22}
$$

$$
\begin{array}{l} \partial_ {i} (t) = K _ {p x} e x _ {i} (t) + K _ {i x} \int_ {0} ^ {t} e x _ {i} (\xi) d \xi \\ + K _ {d x} \frac {d}{d t} e x _ {i} (t) \tag {23} \\ \end{array}
$$

$$
e _ {i} (t) = \partial_ {i} (t) - e v _ {i} (t) \tag {24}
$$

$$
u _ {i} (t) = K _ {p v} e _ {i} (t) + K _ {i v} \int_ {0} ^ {t} e _ {i} (\xi) d \xi + K _ {d v} \frac {d}{d t} e _ {i} (t) \tag {25}
$$

$$
\begin{array}{l} a _ {i} (t + \Delta t, \omega) = \left(1 - \frac {\Delta t}{\tau_ {i} (\omega)}\right) a _ {i} (t) + \frac {\Delta t}{\tau_ {i} (\omega)} u _ {i} (t) (26) \\ u _ {\min } \leq u _ {i} (t) \leq u _ {\max } \\ a _ {\min } \leq a _ {i} (t) \leq a _ {\max } \\ v _ {\text {m i n}} \leq v _ {i} (t) \leq v _ {\text {m a x}} (27) \\ \end{array}
$$

where, $\omega$ is used to distinguish between the stochastic relations and the deterministic ones, e.g., $\tau _ { i } ( \omega )$ and $\tau _ { i } ( \cdot ) . \ e x _ { i } ( t )$ is the spacing error of vehicle i compared with the desired spacing at time t . $e v _ { i } ( t )$ is the speed error of vehicle i relative to vehicle i -1 at time $t , ~ \partial _ { i } ( t )$ and $u _ { i } ( t )$ represent the output of the outer and inner loop PIDs of the cascade PID model at time $t$ , respectively. $e _ { i }$ represents the input error of the inner loop PID. $\xi$ is the integral variable. $K _ { p x }$ , $K _ { i x }$ and $K _ { d x }$ are the parameters of the outer loop PID. $K _ { p v }$ , $K _ { i v }$ and $K _ { d v }$ are the parameters of the inner loop PID. $\Delta t$ represents the time interval. $u _ { m i n }$ and $u _ { m a x }$ are the maximum and minimum constraints on $u _ { i }$ , respectively. $a _ { i } ( t )$ represents the acceleration of vehicle $i$ at time t . $a _ { m i n }$ and $a _ { m a x }$ are the maximum and minimum constraints on $a _ { i } . ~ v _ { m i n }$ and $v _ { m a x }$ are the maximum and minimum constraints on $v _ { i }$ . 

b) Longitudinal model of $H V s$ : We utilize the Intelligent Driver Model (IDM) developed by Treiber et al. [36] to describe the longitudinal behavior of CHVs. The IDM model provides greater realism in capturing congested dynamics [37], and produce collision-free and smooth traffic flow [5]. To account for the heterogeneity of CHVs, we assume that $v ^ { f }$ is a random variable following a normal distribution. We define the random parameter $\mathcal { \vartheta } ( \omega ) \overset { \mathbf { \tilde { \mathbf { \phi } } } } { \equiv } v ^ { f } ( \omega )$ with distribution function $F _ { \vartheta }$ , and the desired speed $v _ { i } ^ { f }$ for each HV i is drawn independently from this common distribution: $\vartheta _ { i } \sim F _ { \vartheta }$ .The mathematical formulation of the stochastic IDM is given by: 

$$
\begin{array}{l} a _ {i} (t, \omega) = A _ {i} \left[ 1 - \left(\frac {v _ {i} (t)}{v _ {i} ^ {f} (\omega)}\right) ^ {4} - \left(\frac {S _ {i} ^ {*} (t)}{d _ {i} (t)}\right) ^ {2} \right] (28) \\ S _ {i} ^ {*} (t) = d _ {0} + \max  \left(0, v _ {i} (t) h _ {i} - \frac {v _ {i} (t) \left[ v _ {i - 1} (t) - v _ {i} (t) \right]}{2 \sqrt {A _ {i} b _ {i}}}\right) (29) \\ \end{array}
$$

where, $v _ { i } ^ { f } ( \omega )$ is the random parameter describing the desired speed of CHV i. $A _ { i }$ , $b _ { i }$ represent the maximum and comfort decelerations of vehicle $i$ , respectively. $d _ { 0 }$ represents the stopping distance. ai (t), vi (t), $S _ { i } ^ { * } ( t )$ , $d _ { i } ( t )$ represent the 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/8a3ed00fb6399dbdefa5b6cf6fbb8cbb06375c3a01142e0e0108255f5aa9382e.jpg)



Fig. 6. Illustration of lane-change spacing for vehicles.


acceleration, speed, desired car-following distance, and actual car-following distance of vehicle $i$ at time t , respectively. $v _ { i - 1 } \left( t \right)$ is the speed of vehicle i-1 at time t . $h _ { i }$ represents the desired time gap of vehicle i . 

2) Lane-Changing Model: In this section, we provide detailed modeling of the lateral movement process, including the lane-changing process of vehicles in the mainline and the merging process of vehicles in the on-ramp, which can be considered as a specific type of lane-changing. The whole process is composed of three parts: 1) lane-changing (or merging) decision-making, 2) dynamic lane-changing trajectory planning, and 3) trajectory tracking control. It is worth noting that the lane-changing model in this section describes the lateral movement characteristics of both CAVs and CHVs. 

a) Lane-changing decision: It is worth noting that the lane-changing decision of vehicles in the main lanes has different constraints within different zones defined in Fig. 1. In this subsection, we discuss only the lane-changing decision within the mainline zone. The lane-changing decision within the cooperative and merging zones is discussed in sections III-B and IV-B. As shown in Fig. 6, $d _ { S V }$ , $d _ { S V } ^ { T L V }$ and $d _ { T F V } ^ { S V }$ denote the longitudinal spacing between the subject vehicle (SV) and LV, the longitudinal spacing between the SV and TLV, and the longitudinal spacing between the TFV and SV, respectively. To simplify the lane-changing decision-making behavior of vehicles within the mainline, we consider rule-based lanechanging criteria, which include the speed constraint and two safety spacing constraints given by: 

$$
v _ {T L V} - v _ {S V} \geq \Delta v _ {\text {t h r e s h o l d}} \tag {30}
$$

$$
d _ {T F V} ^ {S V} (t) \geq v _ {T F V} (t) \cdot h _ {\min } ^ {L C} \tag {31}
$$

$$
d _ {S V} ^ {T L V} (t) > v _ {S V} (t) \cdot h _ {\text {m i n}} ^ {L C} \tag {32}
$$

where hLCmin $h _ { m i n } ^ { L C }$ represents the minimum acceptable time gap for lane-changing. Eq. (30) indicates that if the SV’s speed is constrained by the LV such that the SV can’t achieve a faster velocity, then the SV chooses to change lanes. 

When the above lane-changing criteria are met, the following changes occur: 1) The SV and TLV form a new subsystem, with the TLV being considered as the new leader; 2) The TFV treats the SV as its new leader and they constitute a new subsystem. The TFV performs cooperative maneuvers based on the state of the SV, ensuring the safety of the lane-changing process; 3) At the meantime, the FV still regards the SV as its leader, and they maneuver cooperatively as a subsystem. 

b) Dynamic lane-changing trajectory planning: We use an improved sine function for the lane-changing trajectory planning. The good performance of this method has been demonstrated in our previous work [27]. The mathematical description of the lane-changing trajectory planning model is 

given by: 

$$
\begin{array}{l} y _ {S V} ^ {r} \left(x _ {S V} ^ {r} (t)\right) = y _ {S V} ^ {0} + \frac {y _ {d} ^ {0}}{2 \pi} \left\{\frac {2 \pi}{M (t)} \left(x _ {S V} ^ {r} (t) - x _ {S V} ^ {0}\right) \right\} \\ - \sin \left[ \frac {2 \pi}{M (t)} \left(x _ {S V} ^ {r} (t) - x _ {S V} ^ {0}\right) \right] \\ x _ {S V} ^ {r} (t) \in \left[ x _ {S V} ^ {0}, x _ {S V} ^ {0} + M (t) \right] (33) \\ M (t) = v _ {S V} (t) \sqrt {\frac {2 \left| y _ {d} ^ {0} \right|}{a _ {p}}} + L _ {\text {c e n t e r l i n e}} (34) \\ v _ {S V} (t) = v _ {S V} (t - \Delta t) + a _ {S V} (t) \cdot \Delta t (35) \\ y _ {d} ^ {0} = y _ {T L V} ^ {0} - y _ {S V} ^ {0} (36) \\ \end{array}
$$

$$
\left. y _ {S V} ^ {r} \left(x _ {S V} ^ {r} (t)\right) = \frac {y _ {d} ^ {0}}{M (t)} \left\{1 - \cos \left[ \frac {2 \pi}{M (t)} \left(x _ {S V} ^ {r} (t) - x _ {S V} ^ {0}\right) \right] \right\} \right. \tag {37}
$$

$$
y _ {S V} ^ {r \prime \prime} \left(x _ {S V} ^ {r} (t)\right) = \frac {\pi a _ {p}}{v _ {S V} ^ {2} (t)} \sin \left[ \frac {2 \pi}{M (t)} \left(x _ {S V} ^ {r} (t) - x _ {S V} ^ {0}\right) \right] \tag {38}
$$

$$
K \left(x _ {S V} ^ {r} (t)\right) = \frac {y _ {S V} ^ {r} {} ^ {\prime \prime} \left(x _ {S V} ^ {r} (t)\right)}{\left[ 1 + \left(y _ {S V} ^ {r} \left(x _ {S V} ^ {r} (t)\right)\right) ^ {2} \right] ^ {\frac {2}{3}}} \tag {39}
$$

$$
\varphi_ {S V} ^ {r} \left(x _ {S V} ^ {r} (t)\right) = \tan^ {- 1} \left(y _ {S V} ^ {r} \left(x _ {S V} ^ {r} (t)\right)\right) \tag {40}
$$

$$
\delta_ {S V} ^ {r} \left(x _ {S V} ^ {r} (t)\right) = \tan^ {- 1} \left(l * K \left(x _ {S V} ^ {r} (t)\right)\right) \tag {41}
$$

where $x _ { S V } ^ { r } ( t )$ and $y _ { S V } ^ { r } ( t )$ represent the planned longitudinal and lateral locations of the $S V$ at time $t$ , respectively. $y _ { d } ^ { 0 }$ is the lateral distance between the TLV and SV at the beginning of the lane-changing maneuver. $x _ { S V } ^ { 0 }$ , $y _ { S V } ^ { 0 }$ and $y _ { T L V } ^ { 0 }$ represent the longitudinal position, lateral position of the SV and lateral position of the TLV at the beginning of the lane-changing process, respectively. $M ( t )$ represents the longitudinal length of the planned trajectory at time t ; Lcenterline is the distance that the SV will continue to track the centerline of the lane after completing a lane change. The parameter is designed to prevent errors during the lane-changing process that could cause the SV to have the non-zero yaw and front wheel angles when it reaches the centerline of the target lane. In this study, we set $L _ { c e n t e r l i n e }$ to $1 0 0 \mathrm { m }$ . Notably, the lane change time can be approximated using $\sqrt { \frac { 2 \big | y _ { d } ^ { 0 } \big | } { a _ { p } } } . \ v _ { S V } ( t )$ represents the velocity of the $S V$ at time $t$ , which is the safe lane-changing speed derived based on the longitudinal model. $a _ { p }$ is the planned acceleration considering the comfort of lane changing. $y _ { S V ^ { \prime } } ^ { r }$ and $y _ { S V } ^ { r } { ^ { \prime \prime } }$ represent the first derivative and second derivative of $y _ { S V } ^ { r }$ , respectively; $K$ is the curvature of $y _ { S V } ^ { r } ; \varphi _ { S V } ^ { r }$ and $\delta _ { S V } ^ { r }$ represent the reference yaw angle and reference front wheel angle of the $S V$ , respectively;l is the distance between the front and rear axles of the SV. 

c) Front-collision-avoidance algorithm: During a lanechanging process, it is crucial that the SV avoids potential collisions with the TLV, TFV and LV. The first two collision-avoidance are determined by longitudinal models. However, the SV may also be at risk of collision with the LV prior to changing lanes. To address this issue, a front-collisionavoidance algorithm is developed for lane changing process, aiming at preventing collisions between the SV and LV. Let $T _ { m i d } \left( t \right)$ denote the time required for the SV to travel from the 

current position to the midpoint of the planned trajectory, given its current speed vSV (t). Let $d _ { m i d } ( t )$ denote the difference between the longitudinal displacement of the SV and that of the LV after they have each traveled for time $T _ { m i d } \left( t \right)$ at their respective current speeds. We can obtain: 

$$
x _ {S V} ^ {\text {m i d}} (t) = x _ {S V} ^ {0} + \frac {M (t)}{2} \tag {42}
$$

$$
T _ {m i d} (t) = \frac {x _ {S V} ^ {m i d} (t) - x _ {S V} (t)}{v _ {S V} (t)} \tag {43}
$$

$$
d _ {m i d} (t) = T _ {m i d} (t) \cdot \left(v _ {S V} (t) - v _ {L V} (t)\right) \tag {44}
$$

where $x _ { S V } ^ { m i d } ( t )$ represents the longitudinal position of the midof collision-avoidance is to ensure that the SV does not collide with the LV when the SV travels to the midpoint of its planned trajectory (lane dividing line) at each time step, during which the speeds of the SV and LV are assumed to be constant. That is, at each time step, the longitudinal gap spacing between the SV and LV at time t is larger than $d _ { m i d } \left( t \right)$ . Therefore, the collision-avoidance constraint can be obtained as: 

$$
d _ {m i d} (t) <   x _ {L V} (t) - x _ {S V} (t) - L _ {L V} \tag {45}
$$

By substituting Eqs. (34) and (42)-(44) into Eq. (45), we can obtain: 

$$
\begin{array}{l} \frac {x _ {S V} ^ {0} + \frac {1}{2} \cdot v _ {S V} (t) \sqrt {\frac {2 L _ {w}}{a _ {p}}} - x _ {S V} (t)}{v _ {S V} (t)} \cdot \left(v _ {S V} (t) - v _ {L V} (t)\right) \\ <   x _ {L V (t)} - x _ {S V} (t) - L _ {L V} \tag {46} \\ \end{array}
$$

If the above constraint is unsatisfied, $v _ { S V } \left( t \right)$ used in the trajectory planning is equal to vSV (t − 1), which is the speed of the SV at the previous time. 

d) Trajectory tracking control: In this study, we use the MPC controller to perform the real-time lateral tracking control of the planned trajectory during the lane changing and merging processes. We denote $\boldsymbol { \chi } = \left[ x , y , \varphi \right]$ as the system state vector, which includes the longitudinal location $x$ , lateral position $y$ , yaw angle $\varphi$ . The discrete state-space equationis given by: 

$$
\bar {\chi} (k + 1) = A (k) \bar {\chi} (k) + B (k) \bar {u} (k) \tag {47}
$$

with 

$$
A (k) = \left[ \begin{array}{c c c} 1 & 0 & - v \sin \varphi_ {r} \Delta t \\ 0 & 1 & v \cos \varphi_ {r} \Delta t \\ 0 & 0 & 1 \end{array} \right] B (k) = \left[ \begin{array}{c} 0 \\ 0 \\ \frac {v \Delta t}{l \cos^ {2} \delta_ {f r}} \end{array} \right] \tag {48}
$$

$$
\bar {\chi} (k) = \left[ \begin{array}{l} x (k) - x _ {r} (k) \\ y (k) - y _ {r} (k) \\ \varphi (k) - \varphi_ {r} (k) \end{array} \right] \bar {u} (k) = \left(u (k) - u _ {r} (k)\right) \tag {49}
$$

where, the system control variable $u$ is equal to $\delta _ { f }$ , which is the front wheel angle. $l$ is the distance between the front and the rear axles, and $\Delta t$ is the time step. 

To ensure that the vehicle can track the reference trajectory quickly and smoothly, the objective function is formulated as: 

$$
\begin{array}{l} \min  J (k) = \sum_ {i = 1} ^ {n _ {p}} \left\| \eta (k + i \mid k) - \eta_ {r e f} (k + i \mid k) \right\| _ {Q} \\ + \sum_ {i - 1} ^ {N _ {c} - 1} \| \Delta U (k + j \mid k) \| _ {n} ^ {2} + \rho \varepsilon^ {2} \tag {50} \\ \end{array}
$$

with constraints: 

$$
\left\{ \begin{array}{l} \Delta U _ {\min } \leq \Delta U (k) \leq \Delta U _ {\max } \\ U _ {\min } \leq A \Delta U (k) + U (k - 1) \leq U _ {\max } \end{array} \right. \tag {51}
$$

where $N _ { p }$ and $N _ { c }$ represent the prediction and control horizons, respectively. $\eta \left( k + i \mid k \right)$ represents the vehicle state at prediction step $i$ when $k$ is treated as the current sampling point. $\eta _ { r e f }$ represents the reference state obtained from the planned trajectory. $Q$ and $R$ are the weight matrices. $\Delta U ( k )$ is the system control variable increment at time step k. $\rho$ is the weight coefficient. ε is the relaxation factor. $\Delta U _ { m i n }$ , $\Delta U _ { m a x }$ are the limits of the control variable increment. A is the coefficient matrix of the constraint equation. 

The trajectory tracking control is then converted into a Quadratic Programming (QP) problem under the MPC framework. The good performance of the MPC tracking controller has been proved in our previous work [27]. 

# B. Cooperative Merging Control (CMC) Model

The CMC model includes the dynamic time gap acceptance model, merging behavior model and boundary-collisionavoidance algorithm. During the merging process, the vehicle’s merging intention is higher as it approaches the downstream boundary of the on-ramp, leading to a smaller acceptable merging time gap. To characterize this merging decision behavior, we introduce the dynamic time gap acceptance model that determines the minimum acceptable time gap for cooperative merging. The MV assesses the merging gap in the target lane and determines the feasibility of a merging maneuver using Eq. (53). The merging behavior model, which includes merging decision-making, merging trajectory planning, and merging trajectory tracking control, iteratively controls the MV for each time step until it reaches the centerline of the target lane. The boundary-collision-avoidance algorithm ensures collision-free between MVs and the downstream boundary of the on-ramp by deriving the speed constraints of the MV. 

1) Dynamic Time Gap Acceptance Model: Because merging is a mandatory lane change due to the strong restriction of the downstream boundary of the ramp, we assume that MV’s minimum acceptable time gap for cooperative merging gradually decreases as the distance between the MV and downstream boundary of the ramp becomes smaller. This is consistent with actual merging decision in traffic. Therefore, the dynamic time gap acceptance model is formulated as: 

$$
\tilde {h} _ {M V} ^ {C M} (t) = h _ {u p p e r} ^ {C M} \left(1 - \zeta \frac {x _ {M V} (t) - x _ {0} ^ {m}}{\left(x _ {r a m p} ^ {e n d} - x _ {0} ^ {m}\right)}\right) \tag {52}
$$

where $\tilde { h } _ { M V } ^ { C M } ( t )$ represents the MV’s minimum acceptable time gap of cooperative merging at time t . hC Mupper $h _ { u p p e r } ^ { C M }$ represents the upper bound of the acceptable time gap of cooperative merging. $x _ { 0 } ^ { m }$ represents the longitudinal position of the start point of the merging zone. $x _ { r a m p } ^ { e n d }$ represents the longitudinal position of the downstream boundary of the on-ramp. $\zeta$ represents the linear coefficient of the dynamic time gap acceptance model. Eq. (52) indicates that as the distance betwestart point of the merging zone increases, $\tilde { h } _ { M V } ^ { C M } ( t )$ V and thedecreases 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/9331a53948be6fadd63e52d49d7ae7bd5990f1c3b796cb402dc9772e24a029ae.jpg)



Fig. 7. Illustration of the scenario where multiple MVs merge to the adjacent main lane (lane 2).


linearly. If the MV is at the start point of the merging zone, $\tilde { h } _ { M V } ^ { C M } \dot { = } h _ { u p p e r } ^ { C M }$ = h C M d if the Mequals to wnstream boundary. $\tilde { h } _ { M V } ^ { C M }$ $( 1 - \xi ) h _ { u p p e r } ^ { C M }$ 

2) Merging Behavior Model: The merging behavior model is to describe the merging process of the MV, which is essentially a process of lane-changing from the on-ramp to the adjacent lane (lane 2). Therefore, we can adopt the lane-changing model proposed in section IV-A.2.) to describe the MV’s merging behavior. It is noteworthy that the merging decision making is different from the lane-changing decision. The MV starts to make merging maneuver when the follow constraints are satisfied: 

$$
\left\{ \begin{array}{l} d _ {C F V} ^ {M V} (t) \geq v _ {C F V} (t) \cdot \tilde {h} _ {M V} ^ {C M} (t) \\ d _ {M V} ^ {C L V} (t) \geq v _ {M V} (t) \cdot \tilde {h} _ {M V} ^ {C M} (t) \end{array} \right. \tag {53}
$$

where $d _ { C F V } ^ { M V } ( t )$ represents the actual spacing between the CFV C F V and MV at time t . $d _ { M V } ^ { C L V } ( t )$ represents the actual spacing between the MV and CLV at time t. $v _ { C F V } ( t )$ and $v _ { M V } ( t )$ represent the speeds of the CFV and MV at time $t$ , respectively. 

The merging trajectory planning and tracking control methods of MVs are the same as those in the lane changing model, which is provided in section IV-A.2). 

Collaborative merging involving multiple vehicles is a common in real-world traffic, as depicted in Fig. 7. In this situation, the MVs can be viewed as a vehicle platoon. In [38], two vehicles are simply regarded as one vehicle by adding their lengths together, which is impractical. To address this limitation, we propose the following rules for the merging process: 

1) Only the first and last MVs in the platoon need to be considered in the merging process. Specifically, the first MV and the CLV form a new subsystem, while the CFV and the last MV constitute another subsystem. 

2) The remaining MVs in the platoon continue to maintain car-following in the current subsystem. In other words, the speed of these MVs required for merging (lanechanging) trajectory planning in Eqs. (33) and (34) is updated within the current subsystem. 

Under these rules, the CFV only needs to maintain a safe spacing with the last vehicle in the platoon, which is consistent with the real-world multi-vehicle merging scenarios. The leading vehicle of the platoon ensures the safety of the entire platoon merging process. 

3) Boundary-Collision-Avoidance Algorithm: Boundarycollision-avoidance is crucial to ensure collision-free merging between MVs and the downstream boundary of the on-ramp. The mechanism of collision-avoidance is shown in Fig. 8. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/eb1dffe5eba41a812d1b04f37ea3fa83527abc7aafe03bdf1511e382fc9ab151.jpg)



Fig. 8. Illustration of the boundary-collision-avoidance for downstream ramp of MVs.


$x _ { M V } ^ { m i d } ( t )$ represents the longitudinal position of the midpoint of MV the MV’s planned trajectory at time t. $x _ { r a m p } ^ { e n d }$ is the longitudinal position of the downstream boundary of the on-ramp. Merging is a specific type of lane-changing, so the collision-avoidance spacing requirement must be satisfied when the center of the MV travels to the midpoint of its planned trajectory at each time step. Here, the collision-avoidance spacing is the stopping distance $d _ { 0 }$ . The boundary-collision-avoidance condition can be expressed as: 

$$
\begin{array}{l} x _ {\text {r a m p}} ^ {\text {e n d}} - x _ {M V} ^ {\text {m i d}} (t) - \frac {L _ {M V}}{2} > d _ {0} (54) \\ x _ {M V} ^ {\text {m i d}} (t) = x _ {M V} (t) + \frac {1}{2} v _ {M V} (t) \sqrt {\frac {2 L _ {w}}{a _ {p}}} (55) \\ \end{array}
$$

where $x _ { M V } \left( t \right)$ represents the longitudinal position of the MV at time t . $L _ { M V }$ is the length of the $\mathbf { M V } . L _ { w }$ is the lane width. $a _ { p }$ represents the planned acceleration in trajectory planning. By substituting Eq. (55) into Eq. (54), we can obtain: 

$$
v _ {M V} (t) <   \left(x _ {\text {r a m p}} ^ {\text {e n d}} - x _ {M V} (t) - \frac {L _ {M V}}{2} - d _ {0}\right) \cdot \sqrt {\frac {2 a _ {p}}{L _ {w}}} \tag {56}
$$

Therefore, to avoid a boundary-collision, the speed of the MV at time t should satisfy the condition defined in Eq. (56). 

# V. NUMERICAL EXPERIMENTS

# A. Vehicle Generation Model

In order to ensure continuous traffic flow on a simulated road section, a vehicle generation model is developed, which consists of two components: initialization and boundary generation. The initialization stage generates vehicles based on the density and road length prior to the start of the simulation, while the boundary generation stage produces upcoming vehicles at the upstream boundary of the road section during the simulation process, considering both traffic demand and the arrival headway distribution between two adjacent vehicles. 

The theoretical capacity of mixed traffic can be calculated: 

$$
C _ {m i x} \left(P _ {1}\right) = \frac {3 6 0 0}{P _ {1} \cdot h _ {C A V} + \left(1 - P _ {1}\right) \cdot h _ {C H V} + \frac {d _ {0} + L}{v _ {e}}} \tag {57}
$$

where $C _ { m i x }$ represents the theoretical capacity (veh/h/lane) of mixed traffic. $P _ { 1 }$ represents the penetration rate of CAVs. $h _ { C A V }$ and $h _ { C H V }$ represent the desired time gap of CAVs and CHVs, respectively. $v _ { e }$ represents the equilibrium speed. $d _ { 0 }$ and $L$ respectively represent the stopping spacing and length of vehicle. The traffic volume of the on-ramp lane $Q _ { r a m p }$ is calculated as: 

$$
Q _ {r a m p} = C _ {m i x} \left(P _ {1}\right) \cdot \eta_ {r a m p} \tag {58}
$$

where ηramp is a parameter representing the ratio of the on-ramp flow to the theoretical capacity of the mixed traffic. 

In the vehicle generation model, we consider randomness and uncertainty of traffic flow from four aspects: the vehicle’s arrival time headway, CHV’s desired speed, CHV’s compliance and CAV’s inertial lag. Before the simulation starts, a set of temporary vehicles is pre-generated on each lane based on the input flow and the CAV penetration rate. For each vehicle, the vehicle type (CAV or CHV), desired time gap, driver compliance, desired speed (for CHVs) or inertial lag (for CAVs), lane attributes, and arrival time headway are assigned. Once the simulation starts, the system checks at each time step whether the headway of the waiting vehicle relative to its preceding vehicle $( H _ { W } )$ is greater than or equal to its assigned arrival time headway $\left( H _ { A } \right)$ . If $H _ { W }$ is greater than $H _ { A }$ , the vehicle is generated. 

In the previous studies, randomly distributed inertial lag for CAVs and a power-law headway distribution for the initial conditions are utilized [39], [40]. However, the arrival time headway distribution described by the negative exponential distribution is inconsistent with actual traffic flow characteristics, as it allows for extremely small headways (e.g., 0.01s), which is not conducive to safe driving. Therefore, we use the shifted negative exponential distribution to describe the headway distribution. The probability density function is given by: 

$$
f \left(h _ {A T}\right) = \left\{ \begin{array}{l} \frac {1}{1 / \lambda - h _ {\text {s h i f t e d}}} e ^ {- \frac {1}{1 / \lambda - h _ {\text {s h i f t e d}}} \left(h _ {A T} - h _ {\text {s h i f t e d}}\right)}, h _ {A T} > 0 \\ 0, h _ {A T} \leq 0 \end{array} \right. \tag {59}
$$

$$
\lambda = Q / 3 6 0 0 \tag {60}
$$

where $Q$ is the traffic volume $\left[ v e h / h / l n \right]$ . λ is the vehicle arrival rate per unit time $\left[ v e h / s \right]$ . hshi f ted is the shifted time headway which is the minimum arrival time headway. $h _ { A T }$ is the arrival time headway. 

# B. Simulation Platform

To evaluate the effectiveness of the proposed algorithm and generate the required continuous traffic flow in a mixed traffic environment, we have developed a microscopic simulation platform using MATLAB. This platform incorporates the longitudinal and lane-changing models developed in sections IV-A, explicitly considering both longitudinal and lateral movements of vehicles. Additionally, the vehicle generation model proposed in section V-A is used to simulate vehicle arrival, taking into account of randomness and uncertainty. Fig. 9 shows the flow chart of the simulation process. 

# C. Scenario Settings and Parameters

To test the proposed CORMC model, a typical freeway segment with two main lanes in one direction and an on-ramp is created in the simulation platform, as shown in Fig 10. The mainline section has a total length of $1 0 \mathrm { k m }$ , with the on-ramp located at $6 9 5 0 \mathrm { m }$ downstream from the starting boundary of the mainline. The merging zone has a length of $3 0 0 \mathrm { m } .$ , 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/9d3950beca30e441e57ff6d4cf13ed9ba4cdb684e34052bff9f3a583f68cacb3.jpg)



Fig. 9. The flow chart of the simulation process.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/1a8a42ee52c5550e78a2ce701105fd4f67abc3cfbb49f647a6f4d477715409af.jpg)



Fig. 10. Illustration of a simple merging network in simulation.


while the cooperative zone has a length of $3 0 0 \mathrm { ~ m ~ }$ and moves dynamically according to the position of the MVs on the ramp, which is $3 0 0 \mathrm { m }$ upstream of each MV. Here, we assume that the communication range $L ^ { c r }$ of the vehicle is $3 0 0 \mathrm { m }$ [41]. 

We set the penetration rate of CAVs to be $0 \%$ , $20 \%$ , $40 \%$ , $60 \%$ , $80 \%$ and $100 \%$ to investigate the impact of the CAV on the efficiency of the mainline traffic flow. To better understand the impact of different levels of merging flow ratios on the mainline traffic efficiency, we set the on-ramp flow ratios $\eta _ { r a m p }$ between $10 \%$ and $40 \%$ with a $10 \%$ increment, while the mainline demand $\eta _ { m a i n }$ is set to be $70 \%$ of the theoretical capacity. In this way, we can observe different traffic flow conditions (e.g., uncongested and congested) in the merging area when the on-ramp demand increases for each CAV penetration rate. A total of 24 scenarios are designed, and for each scenario, the simulation time period is 1200s. We use different random number seeds to generate simulation results, and take the average value of 20 simulations to reduce the randomness of the simulation experiment. The random seed assigns the vehicle type (CAV or CHV), the inertial lag of the CAV, the compliance and desired speed of the CHV, the initial speed, and the initial arrival time headway between two 


TABLE I THE VALUES OF VARIABLES AND PARAMETERS


<table><tr><td></td><td>Parameter</td><td>Notation</td><td>Typical value</td></tr><tr><td rowspan="8">CAV&#x27;s longitudinal model</td><td>Control gain parameter of the cruising mode</td><td>k1</td><td>0.4s-1</td></tr><tr><td>Time interval</td><td>Δ</td><td>0.1 s</td></tr><tr><td>Time inertial lag</td><td>τi</td><td>U (0.4, 0.7)</td></tr><tr><td>Desired time gap of the CAV</td><td>hCAV</td><td>1.2 s</td></tr><tr><td>Minimum control value</td><td>umin</td><td>-6 m/s2</td></tr><tr><td>Maximum control value</td><td>umax</td><td>4 m/s2</td></tr><tr><td>Minimum acceleration</td><td>amin</td><td>-6 m/s2</td></tr><tr><td>Maximum acceleration</td><td>amax</td><td>4 m/s2</td></tr><tr><td rowspan="6">IDM model</td><td>Desired time gap of the CHV</td><td>hCHV</td><td>2 s</td></tr><tr><td>Stopping spacing</td><td>d0</td><td>2 m</td></tr><tr><td>Vehicle length</td><td>L</td><td>4 m</td></tr><tr><td>Desired speed</td><td>vf</td><td>N (30, 1.5) m/s</td></tr><tr><td>Maximum acceleration</td><td>Ai</td><td>1.25 m/s2</td></tr><tr><td>Comfort deceleration</td><td>bi</td><td>2.09 m/s2</td></tr><tr><td rowspan="3">Lane-changing model</td><td>Minimum acceptable time gap of lane-changing</td><td>hLCmin</td><td>1.2 s</td></tr><tr><td>Speed difference threshold for lane-changing</td><td>Δvthreshold</td><td>4 m/s</td></tr><tr><td>Planned acceleration in trajectory planning process</td><td>ap</td><td>0.1 m/s2</td></tr><tr><td rowspan="3">APS algorithm</td><td>Anticipatory minimum acceptable time gap of merging</td><td>gCMmin</td><td>1.2 s</td></tr><tr><td>Decision time interval of APS algorithm</td><td>TAPS</td><td>5 s</td></tr><tr><td>Length of the communication range</td><td>Lcr</td><td>300 m</td></tr><tr><td rowspan="6">CMC and CUC models</td><td>upper bound of the acceptable time gap of merging</td><td>hupper</td><td>1.2 s</td></tr><tr><td>Linear coefficient of dynamic time gap acceptance model</td><td>ξ</td><td>2/3</td></tr><tr><td>CUC model&#x27;s weight coefficient</td><td>α</td><td>-1</td></tr><tr><td>CUC model&#x27;s weight coefficient</td><td>β</td><td>1.5</td></tr><tr><td>CUC model&#x27;s weight coefficient</td><td>γ</td><td>0.5</td></tr><tr><td>CUC model&#x27;s weight coefficient</td><td>ζ</td><td>-0.5</td></tr><tr><td rowspan="5">Vehicle generation model</td><td>Shifted time headway of lane 1, lane 2 and on-ramp</td><td>hshifted</td><td>1.2 s / 1.2 s / 3.5 s</td></tr><tr><td>Equilibrium speed</td><td>ve</td><td>30 m/s</td></tr><tr><td>Maximum speed of lane 1, lane 2 and on-ramp</td><td>vmax</td><td>33 m/s / 33 m/s / 30 m/s</td></tr><tr><td>Initial speeds of vehicles in lane 1, lane 2 and on-ramp</td><td>v</td><td>30 m/s / 30 m/s / 16 m/s</td></tr><tr><td>Simulation time period</td><td>All_time</td><td>1200 s</td></tr></table>

adjacent vehicles in the vehicle generation model. The values of the model variables and parameters are listed in Table I. 

# VI. RESULTS

# A. Assessment Indicators of Traffic Efficiency

To evaluate the impact of the proposed CORMC model on traffic efficiency, we used the average flow and average speed defined by Edie based on the time-space trajectories of vehicles. Edie’s generalized average flow and average speed are given by: 

$$
k (A) = \frac {t (A)}{| A |} \tag {61}
$$

$$
q (A) = \frac {d (A)}{| A |} \tag {62}
$$

$$
v (A) = \frac {d (A)}{t (A)} = \frac {q (A)}{k (A)} \tag {63}
$$

where A represents the closed region observed on the time-space trajectory diagram. The area of the region is denoted by|A|, and q (A), k (A)and $v \left( A \right)$ represent Eddie’s 


TABLE II MAINLINE TRAFFIC EFFICIENCY WITH DIFFERENT COMPLIANCE RATES


<table><tr><td></td><td></td><td colspan="5">Compliance rates C (%)</td></tr><tr><td></td><td></td><td>0%</td><td>25%</td><td>50%</td><td>75%</td><td>100%</td></tr><tr><td rowspan="4">Flow (veh/h/ lane)</td><td>Merging flow</td><td>1389</td><td>1373</td><td>1430</td><td>1469</td><td>1521</td></tr><tr><td>Diff (%)</td><td>-</td><td>-1.2</td><td>3.0</td><td>5.8</td><td>9.5</td></tr><tr><td>Mainline flow</td><td>1553</td><td>1556</td><td>1567</td><td>1616</td><td>1699</td></tr><tr><td>Diff (%)</td><td>-</td><td>0.2</td><td>0.9</td><td>4.1</td><td>9.4</td></tr><tr><td rowspan="4">Speed (m/s)</td><td>Merging speed</td><td>20.8</td><td>21.1</td><td>20.5</td><td>21.4</td><td>22.1</td></tr><tr><td>Diff (%)</td><td>-</td><td>1.4</td><td>-1.4</td><td>2.9</td><td>6.3</td></tr><tr><td>Mainline speed</td><td>24.3</td><td>24.8</td><td>25.2</td><td>26.1</td><td>26.4</td></tr><tr><td>Diff (%)</td><td>-</td><td>2.1</td><td>3.7</td><td>7.4</td><td>8.6</td></tr></table>

generalized flow, average density and average speed in the observed time-space region $A$ , respectively. The total time spent by all vehicles in region $A$ is denoted by $t ( A )$ , while the total distance travelled by all vehicles in region $A$ is denoted by $d \left( A \right)$ . 

In order to investigate the impact of ramp merging on mainline traffic, we divided the section between $4 ~ \mathrm { k m }$ and $1 0 \ \mathrm { k m }$ downstream of the starting boundary of the mainline into 12 observation regions, with each region covering a distance of $5 0 0 \mathrm { m }$ . We then calculated the average flow and speed of each observation region during the total simulation time. 

# B. Impact of CHV Compliance Rate on CORMC Model

To better reflect the impact of the compliance rate on the effectiveness of the proposed CORMC model and its effects on mixed traffic performance, we analyzed a scenario under uncongested traffic conditions where CHVs could have greater freedom to choose between maintaining their longitudinal driving or changing lanes. To explore this scenario, we set the CAV penetration rate $P _ { 1 }$ to $60 \%$ and the on-ramp flow ratio $\eta _ { r a m p }$ to $20 \%$ . We considered five cases with different CHV compliance rates $0 \%$ , $2 5 \%$ , $50 \%$ , $7 5 \%$ and $100 \%$ ). Table II presents the average flow and speed of the mainline and merging bottleneck under these different CHV compliance rate, as well as the corresponding percentage difference compared to the reference case with $0 \%$ compliance rate. The results show that the mainline and merging flows, as well as the mainline and merging speed, increase with the increase of the compliance rate, except for a slight decrease in merging flow and merging speed at $2 5 \%$ and $50 \%$ compliance rates, respectively. The performance improvements in terms of flow and speed are marginal when the compliance rate is less than $50 \%$ , while the benefits of the CUC model become more significant when the CHV compliance rate is larger than $7 5 \%$ . 

# C. Impact of CAV Penetration Rate on Mainline Traffic Efficiency

In this section, we discuss the impact of CAV penetration rate on mainline traffic efficiency in terms of average flow and average speed. As shown in Table II, a CHV compliance rate higher than $50 \%$ could lead to a good performance 


TABLE III AVERAGE MERGING AND MAINLINE FLOWS WITH DIFFERENT CAV PENETRATION RATE AND ON-RAMP FLOW RATIO (COMPLIANCE RATE IS $7 5 \%$ )


<table><tr><td rowspan="2">ηramp</td><td rowspan="2">Flow (veh/h/lane)</td><td colspan="6">P1(%)</td></tr><tr><td>0%</td><td>20%</td><td>40%</td><td>60%</td><td>80%</td><td>100%</td></tr><tr><td rowspan="4">10%</td><td>Merging flow</td><td>1066</td><td>1201</td><td>1332</td><td>1497</td><td>1609</td><td>1893</td></tr><tr><td>Diff (%)</td><td>-</td><td>12.7</td><td>25.0</td><td>40.4</td><td>50.9</td><td>77.6</td></tr><tr><td>Mainline flow</td><td>1106</td><td>1192</td><td>1332</td><td>1531</td><td>1683</td><td>1962</td></tr><tr><td>Diff (%)</td><td>-</td><td>7.8</td><td>20.4</td><td>38.4</td><td>52.2</td><td>77.4</td></tr><tr><td rowspan="4">20%</td><td>Merging flow</td><td>1076</td><td>1228</td><td>1372</td><td>1469</td><td>1585</td><td>1886</td></tr><tr><td>Diff (%)</td><td>-</td><td>14.1</td><td>27.5</td><td>36.5</td><td>47.3</td><td>75.3</td></tr><tr><td>Mainline flow</td><td>1148</td><td>1309</td><td>1417</td><td>1616</td><td>1717</td><td>1997</td></tr><tr><td>Diff (%)</td><td>-</td><td>14.0</td><td>23.4</td><td>40.8</td><td>49.6</td><td>74.0</td></tr><tr><td rowspan="4">30%</td><td>Merging flow</td><td>1076</td><td>1158</td><td>1264</td><td>1409</td><td>1655</td><td>1877</td></tr><tr><td>Diff (%)</td><td>-</td><td>7.6</td><td>17.5</td><td>30.9</td><td>53.8</td><td>74.4</td></tr><tr><td>Mainline flow</td><td>1144</td><td>1218</td><td>1423</td><td>1525</td><td>1752</td><td>1955</td></tr><tr><td>Diff (%)</td><td>-</td><td>6.5</td><td>24.4</td><td>33.3</td><td>53.1</td><td>70.9</td></tr><tr><td rowspan="4">40%</td><td>Merging flow</td><td>979</td><td>1097</td><td>1137</td><td>1386</td><td>1645</td><td>1863</td></tr><tr><td>Diff (%)</td><td>-</td><td>12.1</td><td>16.1</td><td>41.6</td><td>68.0</td><td>90.3</td></tr><tr><td>Mainline flow</td><td>1031</td><td>1175</td><td>1254</td><td>1454</td><td>1657</td><td>1832</td></tr><tr><td>Diff (%)</td><td>-</td><td>14.0</td><td>21.6</td><td>41.0</td><td>60.7</td><td>77.7</td></tr></table>

improvement. Therefore, in this experiment, the compliance rate is set to be $7 5 \%$ . Table III presents the average flow of the mainline and merging bottleneck under different CAV penetration rates and on-ramp flow ratios, as well as the corresponding percentage differences compared to the reference case of $0 \%$ CAVs. It is clear that the mainline and merging flows increase significantly with the increase of the CAV penetration rate under different on-ramp flow ratios (from $10 \%$ to $40 \%$ ). For example, under the on-ramp flow ratio of $10 \%$ the percentage increase of the mainline flow and that of the merging flow range from $12 . 7 \%$ to $7 7 . 6 \%$ and from $7 . 8 \%$ t o $7 7 . 4 \%$ , respectively, when the penetration rate increases from $20 \%$ to $100 \%$ . Similar results can be found for the cases of $2 0 \% - 4 0 \%$ on-ramp flow ratio. Additionally, both the mainline and merging flows first increase, and then decrease as the on-ramp flow ratio increases for a given CAV penetration rate. This indicates that the traffic condition of the merging area changes from the uncongested to congested condition when $\eta _ { r a m p }$ increases from $10 \%$ to $40 \%$ . 

Table IV shows the average speeds of the mainline and merging bottleneck, percentage speed difference, and corresponding merging and mainline flow rate under different CAV penetration rates and on-ramp flow ratios. It is clear that the average speed increases with the increase of the penetration rate. For the CAV penetration rate smaller than $60 \%$ , the increase of CAV penetration rate has a marginal impact on the increases of the average mainline and merging speeds under different on-ramp flow ratios, whereas the impact is more significant for higher CAV penetration rates $( > 6 0 \% )$ . The possible reason is that when the CAV penetration rate is low, only a limited number of vehicles in the mainline adopt the proposed CORMC model to improve their driving conditions, resulting in a marginal influence on the overall speed 


TABLE IV AVERAGE SPEEDS OF THE MAINLINE BOTTLENECK AND MAINLINE WITH DIFFERENT CAV PENETRATION RATE AND ON-RAMP FLOW RATIO (COMPLIANCE RATE IS $7 5 \%$ )


<table><tr><td rowspan="2">ηramp</td><td rowspan="2">Speed (m/s) and flow(veh/h/lan e)</td><td colspan="6">P1 (%)</td></tr><tr><td>0%</td><td>20%</td><td>40%</td><td>60%</td><td>80%</td><td>100%</td></tr><tr><td rowspan="6">10%</td><td>Merging speed</td><td>25.4</td><td>25.6</td><td>26.3</td><td>26.3</td><td>26.5</td><td>29.2</td></tr><tr><td>Diff (%)</td><td>-</td><td>0.8</td><td>3.5</td><td>3.5</td><td>4.3</td><td>15.0</td></tr><tr><td>Mainline speed</td><td>24.8</td><td>25.5</td><td>26.2</td><td>26.4</td><td>27.2</td><td>29.8</td></tr><tr><td>Diff (%)</td><td>-</td><td>2.8</td><td>5.6</td><td>6.5</td><td>9.7</td><td>20.2</td></tr><tr><td>Merging flow</td><td>1066</td><td>1201</td><td>1332</td><td>1497</td><td>1609</td><td>1893</td></tr><tr><td>Mainline flow</td><td>1106</td><td>1192</td><td>1332</td><td>1531</td><td>1683</td><td>1962</td></tr><tr><td rowspan="6">20%</td><td>Merging speed</td><td>20.6</td><td>21.1</td><td>21.2</td><td>21.4</td><td>23.3</td><td>27.4</td></tr><tr><td>Diff (%)</td><td>-</td><td>2.4</td><td>2.9</td><td>3.9</td><td>13.1</td><td>33.0</td></tr><tr><td>Mainline speed</td><td>23.7</td><td>23.9</td><td>24.8</td><td>26.1</td><td>27.1</td><td>29.8</td></tr><tr><td>Diff (%)</td><td>-</td><td>0.8</td><td>4.6</td><td>10.1</td><td>14.3</td><td>25.7</td></tr><tr><td>Merging flow</td><td>1076</td><td>1228</td><td>1372</td><td>1469</td><td>1585</td><td>1886</td></tr><tr><td>Mainline flow</td><td>1148</td><td>1309</td><td>1417</td><td>1616</td><td>1717</td><td>1997</td></tr><tr><td rowspan="6">30%</td><td>Merging speed</td><td>15.2</td><td>15.8</td><td>16.0</td><td>15.8</td><td>17.8</td><td>23.4</td></tr><tr><td>Diff (%)</td><td>-</td><td>3.9</td><td>5.3</td><td>3.9</td><td>17.1</td><td>53.9</td></tr><tr><td>Mainline speed</td><td>22.1</td><td>22.4</td><td>24.6</td><td>24.9</td><td>28.1</td><td>29.8</td></tr><tr><td>Diff (%)</td><td>-</td><td>1.4</td><td>11.3</td><td>12.7</td><td>27.1</td><td>34.8</td></tr><tr><td>Merging flow</td><td>1076</td><td>1158</td><td>1264</td><td>1409</td><td>1655</td><td>1877</td></tr><tr><td>Mainline flow</td><td>1144</td><td>1218</td><td>1423</td><td>1525</td><td>1752</td><td>1955</td></tr><tr><td rowspan="6">40%</td><td>Merging speed</td><td>14.0</td><td>14.4</td><td>15.1</td><td>15.7</td><td>18.2</td><td>21.2</td></tr><tr><td>Diff (%)</td><td>-</td><td>2.9</td><td>7.9</td><td>12.1</td><td>30.0</td><td>51.4</td></tr><tr><td>Mainline speed</td><td>20.2</td><td>22.0</td><td>23.4</td><td>23.5</td><td>25.7</td><td>29.8</td></tr><tr><td>Diff (%)</td><td>-</td><td>8.9</td><td>15.8</td><td>16.3</td><td>27.2</td><td>47.5</td></tr><tr><td>Merging flow</td><td>979</td><td>1097</td><td>1137</td><td>1386</td><td>1645</td><td>1863</td></tr><tr><td>Mainline flow</td><td>1031</td><td>1175</td><td>1254</td><td>1454</td><td>1657</td><td>1832</td></tr></table>

improvement. In addition, the improvement of speeds for both the merging bottleneck and mainline is more significant when the on-ramp flow is relatively high (e.g., $\eta _ { r a m p } = 4 0 \%$ ). The percentage increase of the mainline speed and that of the merging speed range from $8 . 9 \%$ to $4 7 . 5 \%$ and from $2 . 9 \%$ to $5 1 . 4 \%$ , respectively, when the CAV penetration rate increases from $20 \%$ to $100 \%$ . This indicates that the proposed CORMC model works more effectively under the situation where the merging bottleneck becomes congested (e.g., when the on-ramp flow ratios are $30 \%$ and $40 \%$ ). 

# D. Performance Comparison Among Different Sub-Models

In this section, we discuss the performance of three control strategies: 

Strategy 1: our proposed CORMC model with both the CUC model and the APS algorithm 

Strategy 2: the CORMC model without the CUC model but with the APS algorithm 

Strategy 3: the CORMC model without the APS algorithm but with the CUC model 

Fig. 11 shows the average flow and speed at the merging bottleneck (Fig. 11 (a)(b)) and the mainline (Fig. 11 (c)(d)) for 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/6272088184ae427a004090e791c35840e39ddbeef453164e0d3d49b2e17e7a5c.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/46026da6d1ca1904a4531e279feaa4bf6f49b736bd74050a27c0ddac71726a2f.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/f8ee9b82de542012d7374a6c7d79e56bcbab07f64da7f77d3fe9553178069058.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/c4ddba2d9a19fae59109e02b3f376c74e2163b3a3c5737fa0d32efd7dfb493fa.jpg)



Fig. 11. Comparison of average flow and speed with different strategies when $\eta _ { r a m p } { = 3 0 \% }$ and $C = 7 5 \%$ : a) average merging flow; b) average merging speed; c) average mainline flow; d) average mainline speed.


the three strategies. The on-ramp flow ratio $\eta _ { r a m p }$ and CHV compliance rate $( C )$ are set to be $30 \%$ and $7 5 \%$ , respectively. It can be clearly observed that strategy 1outperforms the other two strategies in terms of both average flow (throughput) and average speed. Additionally, strategy 2 significantly outperforms strategy 3 across all CAV penetration rates. The average merging speed under strategy 3 drops sharply when the CAV penetration rate increases as can be seen from Fig. 11 (b). This indicates that, in the CORMC model, the APS algorithm plays a more critical role than the CUC model. Assigning a specific CV to the MV can significantly improve the merging speed. Comparing the throughput of strategy 2 and strategy 3 in Fig. 11 (a), we observe a marginal difference at low CAV penetration rates $( P _ { 1 } < 0 . 4 )$ , but strategy 2 performs much better than strategy 3 at higher CAV penetration rates. This is because more HVs with larger headway gaps exist at low CAV penetration rates, so that the advantages of the APS algorithm cannot be fully exploited. In Fig. 11 (d), we observe that the average mainline speed of strategy 3 is the highest among the three strategies when the CAV penetration rate is very low (e.g., ${ < } 2 0 \%$ ). However, strategies 1 and 2 outperforms strategy 3 when the CAV penetrate rate is larger than $40 \%$ . When the CAV penetration rate is $100 \%$ , all vehicles are controllable, and the average mainline speed is the same for all three strategies. One possible reason for the observed results is that, at low CAV penetrate rate, there are more CHVs with larger headway gaps (the desired time gaps of HVs and CAVs are 2.0s and 1.2s in the simulation experiments). As a result, merging can be successfully completed without the APS algorithm, and the presence of the APS algorithm could actually reduce the speed of mainline vehicles. 

# E. Comparison With SUMO

In this section, we verify the performance of the CORMC model by comparing it with the SUMO simulation model. The lane changing behavior in the SUMO strategy is described by the LC2013 model, which is also known as a strategic lane 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/53f701d4ae7f825d01b2dbb130e1d728f3e302ba261845863a952a8592529c06.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/133d7494e03623e814a45ee404b609fdd29fbbdba72c3ceef8234356c506f294.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/1a70d2ba1715428ada12b20bfbc608b837aa848855aad0046e8cf66861cd036d.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/f160815af79d817370d3e0abdfa8872a5c080660054ff688032c132dc566afd7.jpg)



Fig. 12. The increment rates of average flow and speed under different CAV penetration rates compared to SUMO: a) merging flow; b) merging speed; c) mainline flow; d) mainline speed.


changing model [42]. This model can describe lane-changing behavior at lane drops or ramp bottlenecks. In the LC2013 model, a vehicle first evaluates the nearby lane as it approaches the area and determines the urgency of lane changing based on factors such as the remaining distance to the end of the area, speed, and traffic occupancy in the target lane. Next, the feasibility of lane changing is assessed based on the traffic conditions in the surrounding environment. The vehicle then adjusts its speed to safely execute the lane changing maneuver. It is worth noting that the LC2013 model does not include a mechanism for early cooperation with mainline vehicles before entering the on-ramp area. 

In the scenario of SUMO simulation, the default driver behavior models, which are included in SUMO, are applied to all vehicles, while the scenario settings and traffic flow inputs are consistent with those in the proposed CORMC model. We define two indicators, the average speed increment rate $( I _ { v } )$ and the average flow increment rate $( I _ { q } )$ , to measure the improvement achieved by the CORMC model, as given by: 

$$
I _ {v} (A) = \frac {v _ {C O R M C} (A) - v _ {S U M O} (A)}{v _ {S U M O} (A)} \cdot 100 \%
$$

$$
I _ {q} (A) = \frac {q _ {\text {C O R M C}} (A) - q _ {\text {S U M O}} (A)}{q _ {\text {S U M O}} (A)} \cdot 100 \% \tag{64}
$$

where, vC O RMC (A) and qC O RMC (A) represent Eddie’s generalized average speed and flow of the CORMC model in the observed time-space region A, while vSU M O (A)and qSU M O (A) represent those of the SUMO model. 

Fig. 12 illustrates the increment rates of average flows and speeds of the mainline and merging bottleneck under different CAV penetration rates and on-ramp flow ratios compared to the SUMO strategy. As the CAV penetration rate increases, the flow and speed increment rates continue to rise under different on-ramp flow ratios, ranging from $10 \%$ to $40 \%$ . For instance, when $P _ { 1 } = 1 0 0 \%$ and $\eta _ { r a m p } = 4 0 \%$ , the CORMC model can potentially increase merging throughput and speed by over $80 \%$ . These results demonstrate that the proposed CORMC model outperforms SUMO in improving the traffic 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/6a5478fc475c1fb54a38325ed0248855aea0ba3255dce5abf65953f8e3d127f3.jpg)



(a) Vehicle trajectories on the on-ramp


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/9a90c797f0d20308f349398f216a3a1b121627b8c07348bca20ee3c6a0b0d2de.jpg)



(b) Vehicle trajectories in lane 2


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/d3f73e11d1a53620472376491dc852ce9e772d75a2dd154204cc5ddd356ae7e5.jpg)



(c) Vehicle trajectories in lane 1



Fig. 13. Trajectories of all vehicles with $P _ { 1 } = 8 0 \%$ , $\eta _ { r a m p } = 2 0 \%$ and $C = 7 5 \%$ (CORMC).


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/ba40a4ee667d678066508ff36d40e509396b86b3c7d75e7dc3cdc2877e321b2f.jpg)



(a)Vehicle trajectories on the on-ramp


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/1197e0e24166dcff097881c2c524f307ce4a275312ad6a99d7841427d01eeeae.jpg)



(b) Vehicle trajectories in lane 2


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/3b62520567124b6f747b756c1a5640e34d72fccdcda00fd9f29318370b2c2640.jpg)



(c) Vehicle trajectories in lane 1



Fig. 14. Trajectories of all vehicles with $P _ { 1 } = 8 0 \%$ , $\eta _ { r a m p } = 2 0 \%$ (SUMO).


efficiency in the on-ramp merging bottleneck area. Moreover, Fig. 12 (a)(c) shows that higher on-ramp ratios $( > 2 0 \% )$ and CAV penetration rates $( > 6 0 \% )$ lead to greater improvements in merging and mainline flows. This finding indicates that the CORMC model performs more effectively in situations where the merging bottleneck becomes congested. 

Figs. 13 and 14 illustrate the vehicle trajectories of the CORMC and SUMO strategies, respectively, for scenario $P _ { 1 } = 8 0 \%$ and $\eta _ { r a m p } = 2 0 \%$ . In Fig. 13, the pink dot represents the starting point of lane-changing on the corresponding lane, 

while the two areas formed by three black dashed lines denote the merging and cooperative zones from top to bottom, respectively, as shown in Fig. 13(b)(c). Notably, no collisions occur during the entire process, indicating that the proposed CORMC model ensures safety. Fig. 13(a) reveals that the merging start positions of all MVs are distributed within the merging zone in the area between $2 9 5 0 \mathrm { m }$ and $3 2 5 0 \mathrm { m }$ , and the average merging speed is nearly $2 5 \mathrm { m / s }$ . Furthermore, all MVs can merge into lane 2 upstream of the downstream boundary of the on-ramp, validating the effectiveness of the proposed boundary-collision-avoidance algorithm. In contrast, for the SUMO simulation scenario, almost all MVs merge into lane 2 at the end of on-ramp $( 3 2 5 0 \mathrm { m } )$ as shown in Fig. 14 (a), and the average merging speed is less than $1 8 \mathrm { m / s }$ . This demonstrates the superior efficiency of the proposed CMC model. 

Figs. 13(b) and 14(b) demonstrate the impact of MVs on traffic dynamics of lane 2, such as speed reduction. This impact is due to the cooperative maneuvers of mainline vehicles, which decelerate to create gaps for the MV to merge into the mainline. In Fig. 14 (b), as on-ramp vehicles merge into lane 2, vehicles on lane 2 continue to decelerate, with some even stopping at the end of the merging zone. The deceleration wave then propagates upstream, gradually expanding. However, when the CORMC model is adopted, the impact is mainly in the cooperative zone upstream of the onramp. The deceleration wave caused by the merging does not propagate upstream and dissipates in a short period, as shown in Fig. 13 (b). This highlights the effectiveness of the proposed CORMC model in mitigating the propagation of congestion waves. 

Furthermore, Fig. 14(c) depicts that when using the SUMO simulation, only a few vehicles changed lanes to lane 1 in the cooperative and merging zones, likely due to the absence of a proper lane change guidance model. While adopting the proposed CORMC model, there was a significant increase in the number of vehicles in lane 2 changing to lane 1 to create larger merging gaps for MVs, as shown in Fig. 13(c). Despite the lower speed on lane 1 in the CORMC model compared to the SUMO strategy, the average speed of both lanes was approximately twice that of the SUMO simulation, as shown in Fig. 12(b). This improvement in speed is due to the more balanced traffic flow between lanes in the cooperative and merging zones when using the CUC model compared to the SUMO simulation. 

# VII. CONCLUSION AND FUTURE WORK

In this paper, we present a CORMC model for multilane mixed traffic on freeways, which considers both longitudinal and lateral movements, where a hierarchical framework consisting of the APS algorithm, CUC model and CMC model is developed. The multi-lane freeway section is divided into three zones: mainline zone, cooperative zone and merging zone. In the mainline zone, longitudinal and lane-changing models are used to describe vehicles’ movements. In the cooperative zone, the APS algorithm is developed to calculate anticipatory positions at which MVs merge from the on-ramp lane to the adjacent main lane (lane 2) and assign CVs in 

lane 2 for MVs. Then, the CUC model is formulated to determine the best cooperative maneuver choices for CVs for facilitating the creation of merging gaps. In the merging zone, the CMC model coordinates the movements of vehicles on the on-ramp and adjacent lane 2 for safe and efficient merging. A boundary-collision-avoidance algorithm is proposed to guarantee collision-free between MVs and the downstream boundary of the on-ramp. 

To verify the effectiveness of the proposed models and algorithms, we develop a micro-simulation platform using MATLAB. A vehicle generation model is proposed to simulate vehicle arrivals, and the uncertainty and heterogeneity of traffic flow is explicitly considered. Based on numerical experiments, we make six main observations. First, the performance benefits brought by the CUC model are marginal when the CHV compliance rate is low $( < 5 0 \% )$ , while the performance improvement is significant with higher compliance rates $( > 5 0 \% )$ . Second, the impact of the low CAV penetration rates $( < 6 0 \% )$ on the performance improvement of the mixed traffic system is marginal under different on-ramp flow ratios, whereas the impact is more significant for higher CAV penetration rates $( > 6 0 \% )$ . Third, the improvement of speed for the merging bottleneck and mainline is more significant in congested conditions when the on-ramp flow is relatively high (e.g., $\eta _ { r a m p } = 4 0 \%$ ). Fourth, the proposed CORMC model (Strategy 1) outperforms the CORMC without the CUC model (Strategy 2) and the CORMC without the APS algorithm (Strategy 3) in terms of throughput and average speed. The comparison results indicate that the APS algorithm plays a more important role in the CORMC model than the CUC model. The APS algorithm assigns a specific CV to the MV, which could greatly improve the merging speed. Fifth, there are no collisions during the entire process, indicating that the safety of vehicles using our proposed CORMC model can be assured. Finally, the proposed CORMC model can effectively mitigate the propagation of congestion waves. 

In future, we aim to extend the proposed CORMC model to freeways with more than two mainline lanes, taking into account the imbalance of flow and the difference in equilibrium speed on different mainline lanes. Additionally, with the proposed hierarchical control framework, we plan to investigate control strategies in the multi-lane freeway off-ramp bottleneck area. 

# REFERENCES



[1] R. L. Bertini and S. Malik, “Observed dynamic traffic features on freeway section with merges and diverges,” Transp. Res. Rec., J. Transp. Res. Board, vol. 1867, no. 1, pp. 25–35, Jan. 2004. 





[2] J. Zhu and I. Tasic, “Safety analysis of freeway on-ramp merging with the presence of autonomous vehicles,” Accident Anal. Prevention, vol. 152, Mar. 2021, Art. no. 105966. 





[3] H. Liu, X. Kan, S. E. Shladover, X.-Y. Lu, and R. E. Ferlis, “Modeling impacts of cooperative adaptive cruise control on mixed traffic flow in multi-lane freeway facilities,” Transp. Res. C, Emerg. Technol., vol. 95, pp. 261–279, Oct. 2018. 





[4] J. Rios-Torres and A. A. Malikopoulos, “A survey on the coordination of connected and automated vehicles at intersections and merging at highway on-ramps,” IEEE Trans. Intell. Transp. Syst., vol. 18, no. 5, pp. 1066–1077, May 2017. 





[5] M. Zhou, X. Qu, and S. Jin, “On the impact of cooperative autonomous vehicles in improving freeway merging: A modified intelligent driver model-based approach,” IEEE Trans. Intell. Transp. Syst., vol. 18, no. 6, pp. 1422–1428, Jun. 2017. 





[6] R. Scarinci, A. Hegyi, and B. Heydecker, “Definition of a merging assistant strategy using intelligent vehicles,” Transp. Res. C, Emerg. Technol., vol. 82, pp. 161–179, Sep. 2017. 





[7] I. A. Ntousakis, I. K. Nikolos, and M. Papageorgiou, “Optimal vehicle trajectory planning in the context of cooperative merging on highways,” Transp. Res. C, Emerg. Technol., vol. 71, pp. 464–488, Oct. 2016. 





[8] J. Rios-Torres and A. A. Malikopoulos, “Automated and cooperative vehicle merging at highway on-ramps,” IEEE Trans. Intell. Transp. Syst., vol. 18, no. 4, pp. 780–789, Apr. 2017. 





[9] S. Jing, F. Hui, X. Zhao, J. Rios-Torres, and A. J. Khattak, “Cooperative game approach to optimal merging sequence and on-ramp merging control of connected and automated vehicles,” IEEE Trans. Intell. Transp. Syst., vol. 20, no. 11, pp. 4234–4244, Nov. 2019. 





[10] Z. Tang, H. Zhu, X. Zhang, M. Iryo-Asano, and H. Nakamura, “A novel hierarchical cooperative merging control model of connected and automated vehicles featuring flexible merging positions in system optimization,” Transp. Res. C, Emerg. Technol., vol. 138, May 2022, Art. no. 103650. 





[11] N. Chen, B. van Arem, T. Alkim, and M. Wang, “A hierarchical model-based optimization control approach for cooperative merging by connected automated vehicles,” IEEE Trans. Intell. Transp. Syst., vol. 22, no. 12, pp. 7712–7725, Dec. 2021. 





[12] Y. Xue, C. Ding, B. Yu, and W. Wang, “A platoon-based hierarchical merging control for on-ramp vehicles under connected environment,” IEEE Trans. Intell. Transp. Syst., vol. 23, no. 11, pp. 21821–21832, Nov. 2022. 





[13] W. J. Scholte, P. W. A. Zegelaar, and H. Nijmeijer, “A control strategy for merging a single vehicle into a platoon at highway on-ramps,” Transp. Res. C, Emerg. Technol., vol. 136, Mar. 2022, Art. no. 103511. 





[14] D. Marinescu, J. Curn, M. Bouroche, and V. Cahill, “On-ramp traffic merging using cooperative intelligent vehicles: A slot-based approach,” in Proc. 15th Int. IEEE Conf. Intell. Transp. Syst., Sep. 2012, pp. 900–906. 





[15] X. Hu and J. Sun, “Trajectory optimization of connected and autonomous vehicles at a multilane freeway merging area,” Transp. Res. C, Emerg. Technol., vol. 101, pp. 111–125, Apr. 2019. 





[16] S. Karbalaieali, O. A. Osman, and S. Ishak, “A dynamic adaptive algorithm for merging into platoons in connected automated environments,” IEEE Trans. Intell. Transp. Syst., vol. 21, no. 10, pp. 4111–4122, Oct. 2020. 





[17] J. Liu, W. Zhao, and C. Xu, “An efficient on-ramp merging strategy for connected and automated vehicles in multi-lane traffic,” IEEE Trans. Intell. Transp. Syst., vol. 23, no. 6, pp. 1–12, Jun. 2022. 





[18] Y. Xie, H. Zhang, N. H. Gartner, and T. Arsava, “Collaborative merging strategy for freeway ramp operations in a connected and autonomous vehicles environment,” J. Intell. Transp. Syst., vol. 21, no. 2, pp. 136–147, Mar. 2017. 





[19] N. Chen, B. van Arem, and M. Wang, “Hierarchical optimal maneuver planning and trajectory control at on-ramps with multiple mainstream lanes,” IEEE Trans. Intell. Transp. Syst., vol. 23, no. 10, pp. 18889–18902, Oct. 2022. 





[20] C. Wei, Y. He, H. Tian, and Y. Lv, “Game theoretic merging behavior control for autonomous vehicle at highway on-ramp,” IEEE Trans. Intell. Transp. Syst., vol. 23, no. 11, pp. 21127–21136, Nov. 2022. 





[21] R. Pueboobpaphan, F. Liu, and B. van Arem, “The impacts of a communication based merging assistant on traffic flows of manual and equipped vehicles at an on-ramp using traffic flow simulation,” in Proc. 13th Int. IEEE Conf. Intell. Transp. Syst., Sep. 2010, pp. 1468–1473. 





[22] M. Karimi, C. Roncoli, C. Alecsandru, and M. Papageorgiou, “Cooperative merging control via trajectory optimization in mixed vehicular traffic,” Transp. Res. C, Emerg. Technol., vol. 116, Jul. 2020, Art. no. 102663. 





[23] Z. Sun, T. Huang, and P. Zhang, “Cooperative decision-making for mixed traffic: A ramp merging example,” Transp. Res. C, Emerg. Technol., vol. 120, Nov. 2020, Art. no. 102764. 





[24] C. Mu, L. Du, and X. Zhao, “Event triggered rolling horizon based systematical trajectory planning for merging platoons at mainline-ramp intersection,” Transp. Res. C, Emerg. Technol., vol. 125, Apr. 2021, Art. no. 103006. 





[25] G. Guo, P. Li, and L.-Y. Hao, “Adaptive fault-tolerant control of platoons with guaranteed traffic flow stability,” IEEE Trans. Veh. Technol., vol. 69, no. 7, pp. 6916–6927, Jul. 2020. 





[26] S. Wen and G. Guo, “Distributed trajectory optimization and sliding mode control of heterogenous vehicular platoons,” IEEE Trans. Intell. Transp. Syst., vol. 23, no. 7, pp. 7096–7111, Jul. 2022. 





[27] K. Hou, F. Zheng, X. Liu, and Z. Fan, “Cooperative vehicle platoon control considering longitudinal and lane-changing dynamics,” Transportmetrica A, Transp. Sci., vol. 2023, pp. 1–29, Feb. 2023. 





[28] W. Cao, M. Mukai, T. Kawabe, H. Nishira, and N. Fujiki, “Cooperative vehicle path generation during merging using model predictive control with real-time optimization,” Control Eng. Pract., vol. 34, pp. 98–105, Jan. 2015. 





[29] A. Duret, M. Wang, and A. Ladino, “A hierarchical approach for splitting truck platoons near network discontinuities,” Transp. Res. B, Methodol., vol. 132, pp. 285–302, Feb. 2020. 





[30] J. Ding, L. Li, H. Peng, and Y. Zhang, “A rule-based cooperative merging strategy for connected and automated vehicles,” IEEE Trans. Intell. Transp. Syst., vol. 21, no. 8, pp. 3436–3446, Aug. 2020. 





[31] Y. Zhou, E. Chung, A. Bhaskar, and M. E. Cholette, “A state-constrained optimal control based trajectory planning strategy for cooperative freeway mainline facilitating and on-ramp merging maneuvers under congested traffic,” Transp. Res. C, Emerg. Technol., vol. 109, pp. 321–342, Dec. 2019. 





[32] K. Ozbay, H. Yang, B. Bartin, and S. Mudigonda, “Derivation and validation of new simulation-based surrogate safety measure,” Transp. Res. Rec., J. Transp. Res. Board, vol. 2083, no. 1, pp. 105–113, Jan. 2008. 





[33] A. Sharma, Z. Zheng, J. Kim, A. Bhaskar, and M. M. Haque, “Assessing traffic disturbance, efficiency, and safety of the mixed traffic flow of connected vehicles and traditional vehicles by considering human factors,” Transp. Res. C, Emerg. Technol., vol. 124, Mar. 2021, Art. no. 102934. 





[34] L. Zheng, T. Sayed, and M. Essa, “Validating the bivariate extreme value modeling approach for road safety estimation with different traffic conflict indicators,” Accident Anal. Prevention, vol. 123, pp. 314–323, Feb. 2019. 





[35] H. Liu, X. Kan, S. E. Shladover, X.-Y. Lu, and R. E. Ferlis, “Impact of cooperative adaptive cruise control on multilane freeway merge capacity,” J. Intell. Transp. Syst., vol. 22, no. 3, pp. 263–275, May 2018. 





[36] M. Treiber, A. Hennecke, and D. Helbing, “Congested traffic states in empirical observations and microscopic simulations,” Phys. Rev. E, Stat. Phys. Plasmas Fluids Relat. Interdiscip. Top., vol. 62, no. 2, pp. 1805–1824, Aug. 2000. 





[37] A. Talebpour and H. S. Mahmassani, “Influence of connected and autonomous vehicles on traffic flow stability and throughput,” Transp. Res. C, Emerg. Technol., vol. 71, pp. 143–163, Oct. 2016. 





[38] Y. Wang, W. Tang, D. Tian, G. Lu, and G. Yu, “Automated on-ramp merging control algorithm based on internet-connected vehicles,” IET Intell. Transp. Syst., vol. 7, no. 4, pp. 371–379, Dec. 2013. 





[39] L. C. Davis, “Effect of cooperative merging on the synchronous flow phase of traffic,” Phys. A, Stat. Mech. Appl., vol. 361, no. 2, pp. 606–618, Mar. 2006. 





[40] C. Letter and L. Elefteriadou, “Efficient control of fully automated connected vehicles at freeway merge segments,” Transp. Res. C, Emerg. Technol., vol. 80, pp. 190–205, Jul. 2017. 





[41] L. Xiao, M. Wang, W. Schakel, and B. van Arem, “Unravelling effects of cooperative adaptive cruise control deactivation on traffic flow characteristics at merging bottlenecks,” Transp. Res. C, Emerg. Technol., vol. 96, pp. 380–397, Nov. 2018. 





[42] J. Erdmann, “SUMO’s lane-changing model,” in Modeling Mobility With Open Data (Lecture Notes in Mobility), M. Behrisch and M. Weber, Eds. Cham, Switzerland: Springer, 2015, pp. 105–123. 



![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/439d3a83597337f29dda0f8b80eef1434466c85f8b19c7520c78168c9d5e9173.jpg)


Kangning Hou (Graduate Student Member, IEEE) received the B.S. degree in transportation from Southwest Jiaotong University, Chengdu, China, in 2020, where he is currently pursuing the master’s degree in transportation engineering. His main research interests include modeling of mixed traffic flow, connected and automated vehicles control, autonomous driving, and related fields. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/5e7231d7dba80834c1bd5064c4f177282a58b8dbc38910327102c1339fcc665d.jpg)


Fangfang Zheng (Member, IEEE) received the B.S. degree in communication engineering and the M.S. degree in transportation planning and management from Southwest Jiaotong University, Chengdu, China, in 2003 and 2006, respectively, and the Ph.D. degree in transport and planning from the Delft University of Technology, Delft, The Netherlands, in 2011. 

From 2013 to 2018, she was an Associate Professor with Southwest Jiaotong University. Since 2018, she has been a Professor with the School 

of Transportation and Logistics, Southwest Jiaotong University. She has published more than 50 papers in peer-reviewed journals and conference proceedings. Her research interests include urban traffic flow theory and modeling, intelligent transportation systems, and traffic control. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/8cb5cd7c51b2e01316656f45f45cef3438f6c93fed5bb9d6db6b0d85888f35e7.jpg)


Ge Guo (Senior Member, IEEE) received the B.S. and Ph.D. degrees from Northeastern University, Shenyang, China, in 1994 and 1998, respectively. 

From 2000 to 2005, he was the Director of the Institute of Intelligent Control, Lanzhou University of Technology, where he has been a Professor since July 2004. He joined the Department of Automation, Dalian Maritime University, China, as a Professor. He is currently a Professor with Northeastern University. He has published over 170 international journal articles within his areas of interest. His 

research interests include intelligent transportation systems, cyber-physical systems, and connected vehicular systems. He received the CAA Young Scientist Award in 2017 and the First Prize of Natural Science Award of Hebei Province in 2020. He is an Associate Editor of IEEE TRANSACTIONS ON INTELLIGENT TRANSPORTATION SYSTEMS, IEEE TRANSACTIONS ON INTELLIGENT VEHICLES, Information Sciences, IEEE Intelligent Transportation Systems Magazine, and Acta Automatica Sinica. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-26/cbfad2e1-6950-4ae8-b44a-e78bf82a1ba4/cd21858113c65011f711d8ca28d3c3e8dccf4f10a6b99e16f5c1187c8a2cb7a5.jpg)


Xiaobo Liu received the B.S. degree in railway transportation and the M.S. degree in transportation management from Southwest Jiaotong University, Chengdu, China, in 1996 and 1999, respectively, and the Ph.D. degree from the New Jersey Institute of Technology (NJIT), Newark, NJ, USA, in 2004. 

He is currently a Professor with the School of Transportation and Logistics, Southwest Jiaotong University. His research interests include the direction of transportation system analysis under connected vehicle/autonomous vehicle environment, and 

intelligent logistics analysis. He received the George Krambles Transportation Scholarship in 2003, the Most Outstanding Student Paper Award by the Institute of Transportation Engineers (ITE) Metropolitan Section of NY&NJ in 2004, and the Stella Dafermos Best Paper Award, TRB Transportation Network Modeling Committee, in 2018. 