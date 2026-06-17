from __future__ import annotations

from dataclasses import dataclass, replace

from cormc.onestep.kernel import (
    AlgorithmConfig,
    GapBoundaryControllability,
    OneStepEvaluationResult,
    ScenarioConfig,
    TrajectoryContract,
)
from cormc.onestep.lab.reference_case import (
    get_reference_algorithm_config,
    get_reference_expected,
)


@dataclass(frozen=True)
class OneStepFixedExpectation:
    expected_status: str
    expected_best_gap_interval: tuple[float, float] | None
    expected_best_gap_id: str | None
    expected_delta_f_star: float | None
    expected_delta_r_star: float | None
    expected_d_i: float | None
    expected_no_solution_reason: str | None
    must_exclude_gap_ids: tuple[str, ...]
    must_have_reachable_gap_ids: tuple[str, ...]
    must_have_infeasible_gap_ids: tuple[str, ...]


@dataclass(frozen=True)
class OneStepSweepDefinition:
    parameter_name: str
    values: tuple[float, ...]
    representative_values_for_plot: tuple[float, ...]
    expected_trend: str


@dataclass(frozen=True)
class OneStepExperimentScenario:
    scenario_id: str
    description: str
    purpose: str
    x_targets: tuple[float, ...]
    fixed_expectation: OneStepFixedExpectation | None
    sweep: OneStepSweepDefinition | None
    tags: tuple[str, ...]


ONE_STEP_FIXED_SCENARIO_IDS: tuple[str, ...] = (
    "S01",
    "S02",
    "S03",
    "S04",
    "S05",
    "S06",
    "S07",
    "S08",
    "S09",
    "S10",
    "S11",
    "S12",
)

ONE_STEP_SWEEP_SCENARIO_IDS: tuple[str, ...] = ("S11", "S13", "S14")
ONE_STEP_SCENARIO_IDS: tuple[str, ...] = tuple(
    dict.fromkeys((*ONE_STEP_FIXED_SCENARIO_IDS, *ONE_STEP_SWEEP_SCENARIO_IDS))
)

_REQUIRED_CSV_COLUMNS = get_reference_expected().trajectory_contract.required_csv_columns
_XT_PLOT_GROUPS = ("all_vehicles",)
_XT_COLOR_RULES = (
    "merge_vehicle:red",
    "selected_gap_vehicles:blue",
    "non_selected_vehicles:green",
)
_VT_PLOT_GROUPS = (
    "merge_vehicle",
    "selected_gap_rear_vehicle",
    "selected_gap_front_vehicle",
)
_VT_COLOR_RULES = (
    "merge_vehicle:red",
    "selected_gap_vehicles:blue",
)


def build_one_step_scenario_config(x_targets: tuple[float, ...]) -> ScenarioConfig:
    return ScenarioConfig(
        x_targets=x_targets,
        x_m0=0.0,
        v_ref=20.0,
        v_max=30.0,
        v_min=0.0,
        a_max=3.0,
        a_min=-4.0,
        T=20.0,
        gap_boundary_controllability=tuple(
            GapBoundaryControllability(index, True, True)
            for index in range(max(0, len(x_targets) - 1))
        ),
    )


def build_default_one_step_algorithm_config() -> AlgorithmConfig:
    return get_reference_algorithm_config()


def build_one_step_trajectory_contract(
    evaluation: OneStepEvaluationResult,
) -> TrajectoryContract:
    if evaluation.status != "solved" or evaluation.best_gap is None or evaluation.best_score is None:
        raise ValueError("trajectory contract requires solved evaluation")

    best_gap = evaluation.best_gap
    best_score = evaluation.best_score
    sampling_dt = 0.1
    return TrajectoryContract(
        selected_gap_id=best_gap.gap_id,
        selected_gap_interval=(best_gap.x_rear, best_gap.x_front),
        merge_time_s=best_score.t_m,
        merge_point_x=best_score.p_m,
        selected_gap_vehicle_ids=(
            f"target_lane_rear_{int(best_gap.x_rear)}m",
            f"target_lane_front_{int(best_gap.x_front)}m",
        ),
        non_selected_motion_rule=f"x=x0+{int(evaluation.scenario.v_ref)}t; v={int(evaluation.scenario.v_ref)}",
        sampling_dt=sampling_dt,
        required_csv_columns=_REQUIRED_CSV_COLUMNS,
        xt_plot_vehicle_groups=_XT_PLOT_GROUPS,
        xt_plot_color_rules=_XT_COLOR_RULES,
        vt_plot_vehicle_groups=_VT_PLOT_GROUPS,
        vt_plot_color_rules=_VT_COLOR_RULES,
        required_check_times=tuple(
            round(best_score.t_m * fraction, 4)
            for fraction in (0.0, 0.25, 0.5, 0.75, 1.0)
        ),
    )


def load_one_step_experiment_scenario(scenario_id: str) -> OneStepExperimentScenario:
    try:
        return _ONE_STEP_SCENARIOS[scenario_id]
    except KeyError as exc:
        raise ValueError(f"unknown one-step scenario_id: {scenario_id}") from exc


def load_one_step_experiment_scenarios() -> dict[str, OneStepExperimentScenario]:
    return dict(_ONE_STEP_SCENARIOS)


def apply_sweep_parameter(
    algorithm: AlgorithmConfig,
    parameter_name: str,
    parameter_value: float,
) -> AlgorithmConfig:
    return replace(algorithm, **{parameter_name: parameter_value})


def _fixed_expectation(
    *,
    status: str,
    best_gap_id: str | None,
    best_gap_interval: tuple[float, float] | None,
    delta_f_star: float | None,
    delta_r_star: float | None,
    d_i: float | None,
    no_solution_reason: str | None = None,
    must_exclude_gap_ids: tuple[str, ...] = (),
    must_have_reachable_gap_ids: tuple[str, ...] = (),
    must_have_infeasible_gap_ids: tuple[str, ...] = (),
) -> OneStepFixedExpectation:
    return OneStepFixedExpectation(
        expected_status=status,
        expected_best_gap_interval=best_gap_interval,
        expected_best_gap_id=best_gap_id,
        expected_delta_f_star=delta_f_star,
        expected_delta_r_star=delta_r_star,
        expected_d_i=d_i,
        expected_no_solution_reason=no_solution_reason,
        must_exclude_gap_ids=must_exclude_gap_ids,
        must_have_reachable_gap_ids=must_have_reachable_gap_ids,
        must_have_infeasible_gap_ids=must_have_infeasible_gap_ids,
    )


_ONE_STEP_SCENARIOS: dict[str, OneStepExperimentScenario] = {
    "S01": OneStepExperimentScenario(
        scenario_id="S01",
        description="原始基准场景",
        purpose="冻结 strict 黄金基线并验证整条主链路。",
        x_targets=(-180.0, -90.0, -25.0, 30.0, 110.0, 190.0, 250.0),
        fixed_expectation=_fixed_expectation(
            status="solved",
            best_gap_id="gap4",
            best_gap_interval=(30.0, 110.0),
            delta_f_star=5.0,
            delta_r_star=0.0,
            d_i=72.5,
            must_exclude_gap_ids=("gap6",),
            must_have_reachable_gap_ids=("gap1", "gap2", "gap3", "gap4", "gap5"),
        ),
        sweep=None,
        tags=("fixed", "baseline", "strict"),
    ),
    "S02": OneStepExperimentScenario(
        scenario_id="S02",
        description="前方正好存在无需协作的目标 gap",
        purpose="验证 G_i=G_req 时不会强行协作。",
        x_targets=(10.0, 95.0, 170.0, 245.0, 320.0),
        fixed_expectation=_fixed_expectation(
            status="solved",
            best_gap_id="gap1",
            best_gap_interval=(10.0, 95.0),
            delta_f_star=0.0,
            delta_r_star=0.0,
            d_i=52.5,
        ),
        sweep=None,
        tags=("fixed", "exact_gap"),
    ),
    "S03": OneStepExperimentScenario(
        scenario_id="S03",
        description="所有可用 gap 都在合流车后方",
        purpose="验证负位移、等待式合流与速度下界约束。",
        x_targets=(-260.0, -175.0, -90.0, -5.0),
        fixed_expectation=_fixed_expectation(
            status="solved",
            best_gap_id="gap3",
            best_gap_interval=(-90.0, -5.0),
            delta_f_star=0.0,
            delta_r_star=0.0,
            d_i=-47.5,
        ),
        sweep=None,
        tags=("fixed", "negative_displacement"),
    ),
    "S04": OneStepExperimentScenario(
        scenario_id="S04",
        description="所有 gap 都在前方且不可达",
        purpose="验证无 reachable gap 时的 no-solution 退出路径。",
        x_targets=(200.0, 260.0, 320.0, 380.0, 440.0, 500.0),
        fixed_expectation=_fixed_expectation(
            status="no_solution",
            best_gap_id=None,
            best_gap_interval=None,
            delta_f_star=None,
            delta_r_star=None,
            d_i=None,
            no_solution_reason="no_reachable_gap",
            must_exclude_gap_ids=("gap1", "gap2", "gap3", "gap4", "gap5"),
        ),
        sweep=None,
        tags=("fixed", "no_solution"),
    ),
    "S05": OneStepExperimentScenario(
        scenario_id="S05",
        description="密集交通、部分协作不可行",
        purpose="验证 coop_feasible 过滤与排除逻辑。",
        x_targets=(-100.0, -50.0, 0.0, 50.0, 100.0, 160.0, 240.0),
        fixed_expectation=_fixed_expectation(
            status="solved",
            best_gap_id="gap1",
            best_gap_interval=(-100.0, -50.0),
            delta_f_star=0.0,
            delta_r_star=35.0,
            d_i=-92.5,
            must_have_infeasible_gap_ids=("gap2", "gap3", "gap4"),
        ),
        sweep=None,
        tags=("fixed", "coop_filter"),
    ),
    "S06": OneStepExperimentScenario(
        scenario_id="S06",
        description="对称协作分配",
        purpose="验证前后可调整空间相等时的近似平均分配。",
        x_targets=(-100.0, -20.0, 45.0, 125.0, 210.0, 300.0),
        fixed_expectation=_fixed_expectation(
            status="solved",
            best_gap_id="gap2",
            best_gap_interval=(-20.0, 45.0),
            delta_f_star=10.0,
            delta_r_star=10.0,
            d_i=12.5,
        ),
        sweep=None,
        tags=("fixed", "symmetric"),
    ),
    "S07": OneStepExperimentScenario(
        scenario_id="S07",
        description="前车优先调整",
        purpose="验证 q=6 时更偏向可调整空间更大的一侧。",
        x_targets=(-180.0, -90.0, -25.0, 30.0, 110.0, 190.0, 250.0),
        fixed_expectation=_fixed_expectation(
            status="solved",
            best_gap_id="gap4",
            best_gap_interval=(30.0, 110.0),
            delta_f_star=5.0,
            delta_r_star=0.0,
            d_i=72.5,
            must_exclude_gap_ids=("gap6",),
        ),
        sweep=None,
        tags=("fixed", "front_preference"),
    ),
    "S08": OneStepExperimentScenario(
        scenario_id="S08",
        description="后车优先调整",
        purpose="验证后方空间更大时会优先让后车后移。",
        x_targets=(-100.0, -20.0, 60.0, 115.0, 195.0, 275.0),
        fixed_expectation=_fixed_expectation(
            status="solved",
            best_gap_id="gap2",
            best_gap_interval=(-20.0, 60.0),
            delta_f_star=0.0,
            delta_r_star=5.0,
            d_i=17.5,
        ),
        sweep=None,
        tags=("fixed", "rear_preference"),
    ),
    "S09": OneStepExperimentScenario(
        scenario_id="S09",
        description="远方大 gap 不可参与评分",
        purpose="验证 reachable=False 的远方 gap 会被前置移除。",
        x_targets=(-120.0, -40.0, 20.0, 100.0, 250.0, 360.0, 470.0),
        fixed_expectation=_fixed_expectation(
            status="solved",
            best_gap_id="gap2",
            best_gap_interval=(-40.0, 20.0),
            delta_f_star=12.5,
            delta_r_star=12.5,
            d_i=-10.0,
            must_exclude_gap_ids=("gap4", "gap5", "gap6"),
        ),
        sweep=None,
        tags=("fixed", "reachability_filter"),
    ),
    "S10": OneStepExperimentScenario(
        scenario_id="S10",
        description="边界 gap 测试",
        purpose="验证首个 gap 可借助边界空间完成协作且不会报错。",
        x_targets=(-70.0, 10.0, 65.0, 130.0, 200.0, 280.0),
        fixed_expectation=_fixed_expectation(
            status="solved",
            best_gap_id="gap1",
            best_gap_interval=(-70.0, 10.0),
            delta_f_star=0.0,
            delta_r_star=5.0,
            d_i=-32.5,
        ),
        sweep=None,
        tags=("fixed", "boundary"),
    ),
    "S11": OneStepExperimentScenario(
        scenario_id="S11",
        description="w_c 敏感性母场景",
        purpose="保留固定基线，同时支持扫描协作代价权重。",
        x_targets=(-120.0, -40.0, 20.0, 90.0, 180.0, 270.0),
        fixed_expectation=_fixed_expectation(
            status="solved",
            best_gap_id="gap3",
            best_gap_interval=(20.0, 90.0),
            delta_f_star=15.0,
            delta_r_star=0.0,
            d_i=62.5,
        ),
        sweep=OneStepSweepDefinition(
            parameter_name="w_c",
            values=(0.0, 0.2, 1.0),
            representative_values_for_plot=(0.0, 1.0),
            expected_trend="w_c 越大，越倾向低协作代价 gap。",
        ),
        tags=("fixed", "sweep_source", "wc"),
    ),
    "S12": OneStepExperimentScenario(
        scenario_id="S12",
        description="完全对称 tie-breaking 测试",
        purpose="验证分数相等或接近时优先选择更靠前的 gap。",
        x_targets=(-85.0, 0.0, 85.0),
        fixed_expectation=_fixed_expectation(
            status="solved",
            best_gap_id="gap2",
            best_gap_interval=(0.0, 85.0),
            delta_f_star=0.0,
            delta_r_star=0.0,
            d_i=42.5,
        ),
        sweep=None,
        tags=("fixed", "tie_breaking"),
    ),
    "S13": OneStepExperimentScenario(
        scenario_id="S13",
        description="q 参数敏感性测试",
        purpose="验证空间敏感权重指数对协作分配偏向的影响。",
        x_targets=(-180.0, -90.0, -25.0, 30.0, 110.0, 190.0, 250.0),
        fixed_expectation=None,
        sweep=OneStepSweepDefinition(
            parameter_name="q",
            values=(0.5, 1.0, 2.0, 4.0, 6.0, 10.0),
            representative_values_for_plot=(0.5, 6.0, 10.0),
            expected_trend="q 越大，越偏向可调整空间更大的一侧。",
        ),
        tags=("sweep", "q"),
    ),
    "S14": OneStepExperimentScenario(
        scenario_id="S14",
        description="w_t 参数敏感性测试",
        purpose="验证时间惩罚权重是否推动算法偏向更快完成的方案。",
        x_targets=(-100.0, -20.0, 45.0, 130.0, 260.0, 350.0),
        fixed_expectation=None,
        sweep=OneStepSweepDefinition(
            parameter_name="w_t",
            values=(1.0, 5.0, 10.0, 30.0, 100.0),
            representative_values_for_plot=(1.0, 10.0, 100.0),
            expected_trend="w_t 越大，越偏向较近、较快完成的 gap。",
        ),
        tags=("sweep", "wt"),
    ),
}
