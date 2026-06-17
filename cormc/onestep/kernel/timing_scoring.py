from __future__ import annotations

from math import sqrt

from cormc.onestep.kernel.config import AlgorithmConfig, ScenarioConfig
from cormc.onestep.kernel.models import CooperationResult, Gap, ScoreResult, TimingResult


FAILED_GAP_SCORE = 10000.0


def S(tau: float) -> float:
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def S_prime(tau: float) -> float:
    return 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4


def S_double_prime(tau: float) -> float:
    return 60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3


def compute_acc_limit(scenario: ScenarioConfig) -> float:
    return min(scenario.a_max, abs(scenario.a_min))


def compute_unconstrained_time(d: float, algorithm: AlgorithmConfig) -> float:
    return ((3.0 * algorithm.K * (d**2)) / algorithm.w_t) ** 0.25


def compute_acc_time_lower_bound(d: float, a_lim: float) -> float:
    return sqrt(((10.0 / sqrt(3.0)) * abs(d)) / a_lim)


def compute_speed_time_lower_bound(d: float, scenario: ScenarioConfig) -> float:
    peak_speed_coefficient = 15.0 / 8.0
    if d >= 0.0:
        return (peak_speed_coefficient * abs(d)) / (scenario.v_max - scenario.v_ref)
    return (peak_speed_coefficient * abs(d)) / (scenario.v_ref - scenario.v_min)


def compute_timing_for_gap(
    gap: Gap,
    coop: CooperationResult,
    scenario: ScenarioConfig,
    algorithm: AlgorithmConfig,
) -> TimingResult:
    d_i = coop.d_i
    t_0 = compute_unconstrained_time(d_i, algorithm)
    a_lim = compute_acc_limit(scenario)
    t_a = compute_acc_time_lower_bound(d_i, a_lim)
    t_v = compute_speed_time_lower_bound(d_i, scenario)
    t_m = max(t_0, t_a, t_v)
    p_m = scenario.x_m0 + scenario.v_ref * t_m + d_i
    return TimingResult(
        gap_id=gap.gap_id,
        d_i=d_i,
        t_0=t_0,
        a_lim=a_lim,
        t_a=t_a,
        t_v=t_v,
        t_m=t_m,
        p_m=p_m,
    )


def compute_ego_cost(d: float, t_m: float, algorithm: AlgorithmConfig) -> float:
    if abs(d) <= 1e-12 and abs(t_m) <= 1e-12:
        return 0.0
    return algorithm.K * (d**2) / (t_m**3) + algorithm.w_t * t_m


def compute_total_score(C_coop: float, C_ego: float, algorithm: AlgorithmConfig) -> float:
    return algorithm.w_c * C_coop + algorithm.w_e * C_ego


def score_gap(
    gap: Gap,
    coop: CooperationResult,
    timing: TimingResult,
    algorithm: AlgorithmConfig,
) -> ScoreResult:
    C_ego = compute_ego_cost(timing.d_i, timing.t_m, algorithm)
    J = compute_total_score(coop.C_coop, C_ego, algorithm)
    return ScoreResult(
        gap_id=gap.gap_id,
        gap_index=gap.index,
        x_rear=gap.x_rear,
        x_front=gap.x_front,
        d_i=timing.d_i,
        delta_f_star=coop.delta_f_star,
        delta_r_star=coop.delta_r_star,
        C_coop=coop.C_coop,
        t_0=timing.t_0,
        a_lim=timing.a_lim,
        t_a=timing.t_a,
        t_v=timing.t_v,
        t_m=timing.t_m,
        p_m=timing.p_m,
        C_ego=C_ego,
        J=J,
    )


def select_best_gap(score_results: tuple[ScoreResult, ...]) -> ScoreResult:
    eligible_results = tuple(
        result
        for result in score_results
        if result.included_in_best_selection
    )
    return min(eligible_results, key=lambda result: (result.J, -result.gap_index))
