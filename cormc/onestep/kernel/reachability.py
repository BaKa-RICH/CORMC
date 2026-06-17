from __future__ import annotations

from math import sqrt

from cormc.onestep.kernel.config import ScenarioConfig
from cormc.onestep.kernel.models import (
    DerivedParams,
    DirectionalReachKinematics,
    Gap,
    ReachabilityResult,
)


def _compute_directional_reach_kinematics(
    *,
    distance: float,
    v_rel_limit: float,
    a_acc: float,
    a_dec: float,
    direction: str,
) -> DirectionalReachKinematics:
    v_peak = min(v_rel_limit, sqrt(2.0 * distance * a_acc * a_dec / (a_acc + a_dec)))
    t_acc = v_peak / a_acc
    t_dec = v_peak / a_dec
    s_acc = v_peak**2 / (2.0 * a_acc)
    s_dec = v_peak**2 / (2.0 * a_dec)
    s_cruise = max(0.0, distance - s_acc - s_dec)
    cruise_time = 0.0 if s_cruise == 0.0 else s_cruise / v_rel_limit
    return DirectionalReachKinematics(
        direction=direction,
        v_peak=v_peak,
        t_acc=t_acc,
        t_dec=t_dec,
        s_acc=s_acc,
        s_dec=s_dec,
        s_cruise=s_cruise,
        t_reach=t_acc + cruise_time + t_dec,
    )


def compute_front_reach_kinematics(
    D_i: float,
    scenario: ScenarioConfig,
    derived: DerivedParams,
) -> DirectionalReachKinematics:
    if D_i < 0.0:
        raise ValueError("front reachability requires D_i >= 0")
    return _compute_directional_reach_kinematics(
        distance=D_i,
        v_rel_limit=scenario.v_max - scenario.v_ref,
        a_acc=scenario.a_max,
        a_dec=derived.b,
        direction="front",
    )


def compute_rear_reach_kinematics(
    D_i: float,
    scenario: ScenarioConfig,
    derived: DerivedParams,
) -> DirectionalReachKinematics:
    if D_i >= 0.0:
        raise ValueError("rear reachability requires D_i < 0")
    return _compute_directional_reach_kinematics(
        distance=abs(D_i),
        v_rel_limit=scenario.v_ref - scenario.v_min,
        a_acc=derived.b,
        a_dec=scenario.a_max,
        direction="rear",
    )


def compute_reachability_for_gap(
    gap: Gap,
    scenario: ScenarioConfig,
    derived: DerivedParams,
) -> ReachabilityResult:
    kinematics = (
        compute_front_reach_kinematics(gap.D_i, scenario, derived)
        if gap.D_i >= 0.0
        else compute_rear_reach_kinematics(gap.D_i, scenario, derived)
    )
    p_pre = gap.c_i + scenario.v_ref * kinematics.t_reach
    return ReachabilityResult(
        gap_id=gap.gap_id,
        D_i=gap.D_i,
        direction=kinematics.direction,
        v_peak=kinematics.v_peak,
        t_acc=kinematics.t_acc,
        t_dec=kinematics.t_dec,
        s_acc=kinematics.s_acc,
        s_dec=kinematics.s_dec,
        s_cruise=kinematics.s_cruise,
        t_reach=kinematics.t_reach,
        p_pre=p_pre,
        reachable=kinematics.t_reach <= scenario.T,
    )


def filter_reachable_gaps(
    gaps: tuple[Gap, ...],
    reachability: tuple[ReachabilityResult, ...],
) -> tuple[Gap, ...]:
    reachable_gap_ids = {
        result.gap_id
        for result in reachability
        if result.reachable
    }
    return tuple(gap for gap in gaps if gap.gap_id in reachable_gap_ids)
