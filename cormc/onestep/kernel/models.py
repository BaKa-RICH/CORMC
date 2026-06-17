from __future__ import annotations

from dataclasses import dataclass

from cormc.onestep.kernel.config import AlgorithmConfig, ScenarioConfig


CONTROLLABILITY_BRANCH_A = "A_both_controllable"
CONTROLLABILITY_BRANCH_B = "B_front_controllable_rear_uncontrollable"
CONTROLLABILITY_BRANCH_C = "C_rear_controllable_front_uncontrollable"
CONTROLLABILITY_BRANCH_D = "D_none_controllable"


@dataclass(frozen=True)
class DerivedParams:
    b: float
    G_req: float
    G_adj: float


@dataclass(frozen=True)
class Gap:
    gap_id: str
    index: int
    x_rear: float
    x_front: float
    G_i: float
    c_i: float
    D_i: float
    front_controllable: bool = True
    rear_controllable: bool = True
    front_vehicle_id: str | None = None
    rear_vehicle_id: str | None = None


@dataclass(frozen=True)
class DirectionalReachKinematics:
    direction: str
    v_peak: float
    t_acc: float
    t_dec: float
    s_acc: float
    s_dec: float
    s_cruise: float
    t_reach: float


@dataclass(frozen=True)
class ReachabilityResult:
    gap_id: str
    D_i: float
    direction: str
    v_peak: float
    t_acc: float
    t_dec: float
    s_acc: float
    s_dec: float
    s_cruise: float
    t_reach: float
    p_pre: float
    reachable: bool


@dataclass(frozen=True)
class CooperationResult:
    gap_id: str
    controllability_branch: str
    front_controllable: bool
    rear_controllable: bool
    Delta: float
    G_prev: float
    G_next: float
    delta_f_bar: float
    delta_r_bar: float
    coop_feasible: bool
    gamma_f: float
    gamma_r: float
    L: float
    U: float
    delta_f_raw: float
    delta_f_star: float
    delta_r_star: float
    C_coop: float
    d_i: float
    failure_reason: str | None = None


@dataclass(frozen=True)
class TimingResult:
    gap_id: str
    d_i: float
    t_0: float
    a_lim: float
    t_a: float
    t_v: float
    t_m: float
    p_m: float


@dataclass(frozen=True)
class ScoreResult:
    gap_id: str
    gap_index: int
    x_rear: float
    x_front: float
    d_i: float
    delta_f_star: float
    delta_r_star: float
    C_coop: float
    t_0: float
    a_lim: float
    t_a: float
    t_v: float
    t_m: float
    p_m: float
    C_ego: float
    J: float
    included_in_best_selection: bool = True
    failure_reason: str | None = None


@dataclass(frozen=True)
class GapEvaluationRow:
    gap_id: str
    gap_index: int
    front_vehicle_id: str | None
    rear_vehicle_id: str | None
    front_controllable: bool
    rear_controllable: bool
    controllability_branch: str
    x_rear: float
    x_front: float
    G_i: float
    c_i: float
    D_i: float
    direction: str
    t_reach: float
    p_pre: float
    reachable: bool
    Delta: float | None
    G_prev: float | None
    G_next: float | None
    delta_f_bar: float | None
    delta_r_bar: float | None
    coop_feasible: bool | None
    gamma_f: float | None
    gamma_r: float | None
    L: float | None
    U: float | None
    delta_f_raw: float | None
    delta_f_star: float | None
    delta_r_star: float | None
    C_coop: float | None
    d_i: float | None
    t_0: float | None
    a_lim: float | None
    t_a: float | None
    t_v: float | None
    t_m: float | None
    p_m: float | None
    C_ego: float | None
    J: float | None
    included_in_scoring: bool
    failure_reason: str | None
    is_selected: bool = False


@dataclass(frozen=True)
class OneStepEvaluationResult:
    scenario: ScenarioConfig
    algorithm: AlgorithmConfig
    derived: DerivedParams
    gaps: tuple[Gap, ...]
    reachability: tuple[ReachabilityResult, ...]
    cooperation: tuple[CooperationResult, ...]
    timing: tuple[TimingResult, ...]
    scores: tuple[ScoreResult, ...]
    gap_rows: tuple[GapEvaluationRow, ...]
    best_score: ScoreResult | None
    best_gap: Gap | None
    status: str
    no_solution_reason: str | None


@dataclass(frozen=True)
class TrajectorySample:
    t: float
    vehicle_id: str
    role: str
    x: float
    v: float
    a: float
    is_selected_gap_vehicle: bool
    is_merge_vehicle: bool


@dataclass(frozen=True)
class TrajectoryBundle:
    selected_gap_id: str
    selected_gap_interval: tuple[float, float]
    merge_time_s: float
    merge_point_x: float
    samples: tuple[TrajectorySample, ...]
    check_times: tuple[float, ...]


@dataclass(frozen=True)
class TrajectoryArtifacts:
    trajectory_csv_path: str
    xt_plot_path: str
    vt_plot_path: str
    bundle: TrajectoryBundle


@dataclass(frozen=True)
class OneStepScenarioArtifacts:
    evaluation: OneStepEvaluationResult
    bundle: TrajectoryBundle | None
    trajectory_csv_path: str | None
    xt_plot_path: str | None
    vt_plot_path: str | None


@dataclass(frozen=True)
class GapReferenceRow:
    gap_id: str
    x_rear: float
    x_front: float
    G_i: float
    c_i: float
    D_i: float


@dataclass(frozen=True)
class ReachabilityReferenceRow:
    gap_id: str
    G_i: float
    c_i: float
    D_i: float
    t_reach: float
    p_pre: float
    reachable: bool


@dataclass(frozen=True)
class CooperationReferenceRow:
    gap_id: str
    Delta: float
    delta_f_bar: float
    delta_r_bar: float
    gamma_f: float
    gamma_r: float
    L: float
    U: float
    delta_f_star: float
    delta_r_star: float
    C_coop: float
    d_i: float


@dataclass(frozen=True)
class StrictScoreReferenceRow:
    gap_id: str
    d_i: float
    t_v: float
    t_m: float
    p_m: float
    C_coop: float
    C_ego: float
    J: float


@dataclass(frozen=True)
class TrajectoryContract:
    selected_gap_id: str
    selected_gap_interval: tuple[float, float]
    merge_time_s: float
    merge_point_x: float
    selected_gap_vehicle_ids: tuple[str, str]
    non_selected_motion_rule: str
    sampling_dt: float
    required_csv_columns: tuple[str, ...]
    xt_plot_vehicle_groups: tuple[str, ...]
    xt_plot_color_rules: tuple[str, ...]
    vt_plot_vehicle_groups: tuple[str, ...]
    vt_plot_color_rules: tuple[str, ...]
    required_check_times: tuple[float, ...]


@dataclass(frozen=True)
class ReferenceExpected:
    scenario: ScenarioConfig
    algorithm: AlgorithmConfig
    gap_rows: tuple[GapReferenceRow, ...]
    reachability_rows: tuple[ReachabilityReferenceRow, ...]
    cooperation_rows: tuple[CooperationReferenceRow, ...]
    strict_score_rows: tuple[StrictScoreReferenceRow, ...]
    best_gap_id: str
    best_gap_interval: tuple[float, float]
    best_delta_f_star: float
    best_delta_r_star: float
    best_d_i: float
    best_t_m: float
    best_p_m: float
    trajectory_contract: TrajectoryContract
