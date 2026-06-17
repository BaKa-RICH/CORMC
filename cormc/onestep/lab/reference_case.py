from __future__ import annotations

from cormc.onestep.kernel.config import (
    AlgorithmConfig,
    GapBoundaryControllability,
    ScenarioConfig,
)
from cormc.onestep.kernel.models import (
    CooperationReferenceRow,
    GapReferenceRow,
    ReachabilityReferenceRow,
    ReferenceExpected,
    StrictScoreReferenceRow,
    TrajectoryContract,
)


_REFERENCE_SCENARIO = ScenarioConfig(
    x_targets=(-180.0, -90.0, -25.0, 30.0, 110.0, 190.0, 250.0),
    x_m0=0.0,
    v_ref=20.0,
    v_max=30.0,
    v_min=0.0,
    a_max=3.0,
    a_min=-4.0,
    T=20.0,
    gap_boundary_controllability=tuple(
        GapBoundaryControllability(index, True, True)
        for index in range(6)
    ),
)

_REFERENCE_ALGORITHM = AlgorithmConfig(
    D_h=40.0,
    l_m=5.0,
    w_c=0.2,
    w_e=1.0,
    w_t=10.0,
    delta_ref=35.0,
    q=6.0,
    epsilon_delta=0.05,
    K=120.0 / 7.0,
    boundary_adjustment=100.0,
)

_REFERENCE_GAP_ROWS = (
    GapReferenceRow("gap1", -180.0, -90.0, 90.0, -135.0, -135.0),
    GapReferenceRow("gap2", -90.0, -25.0, 65.0, -57.5, -57.5),
    GapReferenceRow("gap3", -25.0, 30.0, 55.0, 2.5, 2.5),
    GapReferenceRow("gap4", 30.0, 110.0, 80.0, 70.0, 70.0),
    GapReferenceRow("gap5", 110.0, 190.0, 80.0, 150.0, 150.0),
    GapReferenceRow("gap6", 190.0, 250.0, 60.0, 220.0, 220.0),
)

_REFERENCE_REACHABILITY_ROWS = (
    ReachabilityReferenceRow("gap1", 90.0, -135.0, -135.0, 12.583, 116.667, True),
    ReachabilityReferenceRow("gap2", 65.0, -57.5, -57.5, 8.190, 106.309, True),
    ReachabilityReferenceRow("gap3", 55.0, 2.5, 2.5, 1.708, 36.657, True),
    ReachabilityReferenceRow("gap4", 80.0, 70.0, 70.0, 9.917, 268.333, True),
    ReachabilityReferenceRow("gap5", 80.0, 150.0, 150.0, 17.917, 508.333, True),
    ReachabilityReferenceRow("gap6", 60.0, 220.0, 220.0, 24.917, 718.333, False),
)

_REFERENCE_COOPERATION_ROWS = (
    CooperationReferenceRow("gap1", 0.0, 20.0, 100.0, 28.723, 0.001838, 0.0, 0.0, 0.000, 0.000, 0.000, -135.000),
    CooperationReferenceRow("gap2", 20.0, 10.0, 45.0, 1838.266, 0.221, 0.0, 10.0, 0.000, 20.000, 88.551, -67.500),
    CooperationReferenceRow("gap3", 30.0, 35.0, 20.0, 1.000, 28.723, 10.0, 30.0, 28.991, 1.009, 869.720, 16.491),
    CooperationReferenceRow("gap4", 5.0, 35.0, 10.0, 1.000, 1838.266, 0.0, 5.0, 5.000, 0.000, 25.000, 72.500),
    CooperationReferenceRow("gap5", 5.0, 15.0, 35.0, 161.384, 1.000, 0.0, 5.0, 0.000, 5.000, 25.000, 147.500),
)

_REFERENCE_STRICT_SCORE_ROWS = (
    StrictScoreReferenceRow("gap1", -135.000, 12.656, 17.497, 214.944, 0.000, 233.296, 233.296),
    StrictScoreReferenceRow("gap2", -67.500, 6.328, 12.372, 179.947, 88.551, 164.965, 182.675),
    StrictScoreReferenceRow("gap3", 16.491, 3.092, 6.115, 138.797, 869.720, 81.538, 255.482),
    StrictScoreReferenceRow("gap4", 72.500, 13.594, 13.594, 344.375, 25.000, 171.808, 176.808),
    StrictScoreReferenceRow("gap5", 147.500, 27.656, 27.656, 700.625, 25.000, 294.194, 299.194),
)

_REFERENCE_TRAJECTORY_CONTRACT = TrajectoryContract(
    selected_gap_id="gap4",
    selected_gap_interval=(30.0, 110.0),
    merge_time_s=13.594,
    merge_point_x=344.375,
    selected_gap_vehicle_ids=("target_lane_rear_30m", "target_lane_front_110m"),
    non_selected_motion_rule="x=x0+20t; v=20",
    sampling_dt=0.1,
    required_csv_columns=(
        "t",
        "vehicle_id",
        "role",
        "x",
        "v",
        "is_selected_gap_vehicle",
        "is_merge_vehicle",
    ),
    xt_plot_vehicle_groups=("all_vehicles",),
    xt_plot_color_rules=(
        "merge_vehicle:red",
        "selected_gap_vehicles:blue",
        "non_selected_vehicles:green",
    ),
    vt_plot_vehicle_groups=(
        "merge_vehicle",
        "selected_gap_rear_vehicle",
        "selected_gap_front_vehicle",
    ),
    vt_plot_color_rules=(
        "merge_vehicle:red",
        "selected_gap_vehicles:blue",
    ),
    required_check_times=(0.0, 3.3985, 6.797, 10.1955, 13.594),
)

_REFERENCE_EXPECTED = ReferenceExpected(
    scenario=_REFERENCE_SCENARIO,
    algorithm=_REFERENCE_ALGORITHM,
    gap_rows=_REFERENCE_GAP_ROWS,
    reachability_rows=_REFERENCE_REACHABILITY_ROWS,
    cooperation_rows=_REFERENCE_COOPERATION_ROWS,
    strict_score_rows=_REFERENCE_STRICT_SCORE_ROWS,
    best_gap_id="gap4",
    best_gap_interval=(30.0, 110.0),
    best_delta_f_star=5.0,
    best_delta_r_star=0.0,
    best_d_i=72.5,
    best_t_m=13.594,
    best_p_m=344.375,
    trajectory_contract=_REFERENCE_TRAJECTORY_CONTRACT,
)


def get_reference_scenario() -> ScenarioConfig:
    return _REFERENCE_SCENARIO


def get_reference_algorithm_config() -> AlgorithmConfig:
    return _REFERENCE_ALGORITHM


def get_reference_expected() -> ReferenceExpected:
    return _REFERENCE_EXPECTED
