from __future__ import annotations

from math import inf

from cormc.onestep.kernel.config import AlgorithmConfig, ScenarioConfig
from cormc.onestep.kernel.models import (
    CONTROLLABILITY_BRANCH_A,
    CONTROLLABILITY_BRANCH_B,
    CONTROLLABILITY_BRANCH_C,
    CONTROLLABILITY_BRANCH_D,
    CooperationResult,
    DerivedParams,
    Gap,
)


NONE_CONTROLLABLE_FALLBACK_GAP_M = 95.0


def compute_gap_deficit(gap: Gap, derived: DerivedParams) -> float:
    return max(0.0, derived.G_req - gap.G_i)


def compute_boundary_gap(derived: DerivedParams, algorithm: AlgorithmConfig) -> float:
    return derived.G_adj + algorithm.boundary_adjustment


def compute_adjacent_gap_lengths(
    gaps: tuple[Gap, ...],
    gap_index: int,
    boundary_gap: float,
) -> tuple[float, float]:
    G_prev = boundary_gap if gap_index == 0 else gaps[gap_index - 1].G_i
    G_next = boundary_gap if gap_index == len(gaps) - 1 else gaps[gap_index + 1].G_i
    return (G_prev, G_next)


def compute_adjustment_capacity(
    G_prev: float,
    G_next: float,
    derived: DerivedParams,
) -> tuple[float, float]:
    delta_f_bar = max(0.0, G_next - derived.G_adj)
    delta_r_bar = max(0.0, G_prev - derived.G_adj)
    return (delta_f_bar, delta_r_bar)


def controllability_branch_for_gap(gap: Gap) -> str:
    if gap.front_controllable and gap.rear_controllable:
        return CONTROLLABILITY_BRANCH_A
    if gap.front_controllable and not gap.rear_controllable:
        return CONTROLLABILITY_BRANCH_B
    if gap.rear_controllable and not gap.front_controllable:
        return CONTROLLABILITY_BRANCH_C
    return CONTROLLABILITY_BRANCH_D


def apply_controllability_to_capacity(
    gap: Gap,
    delta_f_bar: float,
    delta_r_bar: float,
) -> tuple[float, float, str]:
    branch = controllability_branch_for_gap(gap)
    if branch == CONTROLLABILITY_BRANCH_B:
        return (delta_f_bar, 0.0, branch)
    if branch == CONTROLLABILITY_BRANCH_C:
        return (0.0, delta_r_bar, branch)
    if branch == CONTROLLABILITY_BRANCH_D:
        return (0.0, 0.0, branch)
    return (delta_f_bar, delta_r_bar, branch)


def check_coop_feasibility(Delta: float, delta_f_bar: float, delta_r_bar: float) -> bool:
    return delta_f_bar + delta_r_bar >= Delta


def compute_gamma(delta_bar: float, algorithm: AlgorithmConfig) -> float:
    if delta_bar <= 0.0:
        return inf
    return (algorithm.delta_ref / delta_bar) ** algorithm.q


def compute_projection_bounds(
    Delta: float,
    delta_f_bar: float,
    delta_r_bar: float,
) -> tuple[float, float]:
    return (max(0.0, Delta - delta_r_bar), min(delta_f_bar, Delta))


def _clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def solve_projected_adjustment(
    Delta: float,
    delta_f_bar: float,
    delta_r_bar: float,
    gamma_f: float,
    gamma_r: float,
) -> tuple[float, float, float, float]:
    L, U = compute_projection_bounds(Delta, delta_f_bar, delta_r_bar)
    if Delta <= 0.0:
        delta_f_raw = 0.0
    elif gamma_f == inf and gamma_r == inf:
        delta_f_raw = 0.0
    elif gamma_f == inf:
        delta_f_raw = 0.0
    elif gamma_r == inf:
        delta_f_raw = Delta
    else:
        delta_f_raw = (gamma_r / (gamma_f + gamma_r)) * Delta
    delta_f_proj = _clip(delta_f_raw, L, U)
    return (L, U, delta_f_raw, delta_f_proj)


def apply_deadzone(
    Delta: float,
    delta_f_bar: float,
    delta_r_bar: float,
    delta_f_proj: float,
    algorithm: AlgorithmConfig,
) -> tuple[float, float]:
    delta_r_proj = Delta - delta_f_proj
    if 0.0 < delta_f_proj < algorithm.epsilon_delta and Delta <= delta_r_bar:
        return (0.0, Delta)
    if 0.0 < delta_r_proj < algorithm.epsilon_delta and Delta <= delta_f_bar:
        return (Delta, 0.0)
    return (delta_f_proj, delta_r_proj)


def _cost_term(gamma: float, delta: float) -> float:
    if delta == 0.0:
        return 0.0
    return gamma * (delta**2)


def compute_coop_cost(
    gamma_f: float,
    gamma_r: float,
    delta_f_star: float,
    delta_r_star: float,
) -> float:
    return _cost_term(gamma_f, delta_f_star) + _cost_term(gamma_r, delta_r_star)


def compute_adjusted_displacement(
    gap: Gap,
    delta_f_star: float,
    delta_r_star: float,
    scenario: ScenarioConfig,
) -> float:
    return gap.c_i + (delta_f_star - delta_r_star) / 2.0 - scenario.x_m0


def compute_cooperation_for_gap(
    gap: Gap,
    gaps: tuple[Gap, ...],
    scenario: ScenarioConfig,
    algorithm: AlgorithmConfig,
    derived: DerivedParams,
) -> CooperationResult:
    Delta = compute_gap_deficit(gap, derived)
    boundary_gap = compute_boundary_gap(derived, algorithm)
    G_prev, G_next = compute_adjacent_gap_lengths(gaps, gap.index, boundary_gap)
    delta_f_bar, delta_r_bar = compute_adjustment_capacity(G_prev, G_next, derived)
    delta_f_bar, delta_r_bar, branch = apply_controllability_to_capacity(
        gap,
        delta_f_bar,
        delta_r_bar,
    )
    coop_feasible = check_coop_feasibility(Delta, delta_f_bar, delta_r_bar)
    failure_reason = None

    if branch == CONTROLLABILITY_BRANCH_D:
        coop_feasible = gap.G_i > NONE_CONTROLLABLE_FALLBACK_GAP_M
        if not coop_feasible:
            failure_reason = "none_controllable_gap_not_over_95m"
    elif not coop_feasible and branch in {
        CONTROLLABILITY_BRANCH_B,
        CONTROLLABILITY_BRANCH_C,
    }:
        failure_reason = "insufficient_single_side_capacity"
    elif not coop_feasible:
        failure_reason = "insufficient_total_capacity"

    gamma_f = compute_gamma(delta_f_bar, algorithm)
    gamma_r = compute_gamma(delta_r_bar, algorithm)
    L, U, delta_f_raw, delta_f_proj = solve_projected_adjustment(
        Delta,
        delta_f_bar,
        delta_r_bar,
        gamma_f,
        gamma_r,
    )

    if coop_feasible and branch == CONTROLLABILITY_BRANCH_D:
        delta_f_star = 0.0
        delta_r_star = 0.0
        C_coop = 0.0
        d_i = gap.c_i - scenario.x_m0
    elif coop_feasible:
        delta_f_star, delta_r_star = apply_deadzone(
            Delta,
            delta_f_bar,
            delta_r_bar,
            delta_f_proj,
            algorithm,
        )
        C_coop = compute_coop_cost(gamma_f, gamma_r, delta_f_star, delta_r_star)
        d_i = compute_adjusted_displacement(gap, delta_f_star, delta_r_star, scenario)
    else:
        delta_f_star = 0.0
        delta_r_star = 0.0
        C_coop = 0.0
        d_i = gap.c_i - scenario.x_m0

    return CooperationResult(
        gap_id=gap.gap_id,
        controllability_branch=branch,
        front_controllable=gap.front_controllable,
        rear_controllable=gap.rear_controllable,
        Delta=Delta,
        G_prev=G_prev,
        G_next=G_next,
        delta_f_bar=delta_f_bar,
        delta_r_bar=delta_r_bar,
        coop_feasible=coop_feasible,
        gamma_f=gamma_f,
        gamma_r=gamma_r,
        L=L,
        U=U,
        delta_f_raw=delta_f_raw,
        delta_f_star=delta_f_star,
        delta_r_star=delta_r_star,
        C_coop=C_coop,
        d_i=d_i,
        failure_reason=failure_reason,
    )
