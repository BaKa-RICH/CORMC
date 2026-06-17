from __future__ import annotations

from math import floor, isclose

from cormc.onestep.kernel.config import ScenarioConfig
from cormc.onestep.kernel.models import Gap, ScoreResult, TrajectoryBundle, TrajectorySample, TrajectoryContract
from cormc.onestep.kernel.timing_scoring import S, S_double_prime, S_prime

MERGE_VEHICLE_ID = "merge_vehicle"
MERGE_VEHICLE_ROLE = "merge_vehicle"
SELECTED_REAR_ROLE = "selected_gap_rear_vehicle"
SELECTED_FRONT_ROLE = "selected_gap_front_vehicle"
NON_SELECTED_ROLE = "non_selected_vehicle"


def build_time_grid(t_m: float, dt: float) -> tuple[float, ...]:
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    if t_m < 0.0:
        raise ValueError("t_m must be non-negative")
    if t_m == 0.0:
        return (0.0,)

    step_count = int(floor(t_m / dt))
    times = [round(index * dt, 10) for index in range(step_count + 1)]
    if times[-1] > t_m:
        times.pop()
    if not isclose(times[-1], t_m, rel_tol=0.0, abs_tol=1e-9):
        times.append(t_m)
    else:
        times[-1] = t_m
    return tuple(times)


def compute_tau(t: float, t_m: float) -> float:
    if t_m <= 0.0:
        raise ValueError("t_m must be positive")
    if t < 0.0:
        raise ValueError("t must be non-negative")
    if t > t_m:
        raise ValueError("tau is only defined for t <= t_m")
    return t / t_m


def _sample_quintic_phase(
    *,
    t: float,
    t_m: float,
    x_base: float,
    v_ref: float,
    delta_x: float,
) -> tuple[float, float, float]:
    tau = compute_tau(t, t_m)
    x = x_base + v_ref * t + delta_x * S(tau)
    v = v_ref + (delta_x / t_m) * S_prime(tau)
    a = (delta_x / (t_m**2)) * S_double_prime(tau)
    return (x, v, a)


def sample_merge_vehicle_state(
    t: float,
    scenario: ScenarioConfig,
    score: ScoreResult,
) -> TrajectorySample:
    if t <= score.t_m:
        x, v, a = _sample_quintic_phase(
            t=t,
            t_m=score.t_m,
            x_base=scenario.x_m0,
            v_ref=scenario.v_ref,
            delta_x=score.d_i,
        )
    else:
        x = score.p_m + scenario.v_ref * (t - score.t_m)
        v = scenario.v_ref
        a = 0.0
    return TrajectorySample(
        t=t,
        vehicle_id=MERGE_VEHICLE_ID,
        role=MERGE_VEHICLE_ROLE,
        x=x,
        v=v,
        a=a,
        is_selected_gap_vehicle=False,
        is_merge_vehicle=True,
    )


def sample_selected_rear_vehicle_state(
    t: float,
    scenario: ScenarioConfig,
    best_gap: Gap,
    score: ScoreResult,
) -> TrajectorySample:
    if t <= score.t_m:
        x, v, a = _sample_quintic_phase(
            t=t,
            t_m=score.t_m,
            x_base=best_gap.x_rear,
            v_ref=scenario.v_ref,
            delta_x=-score.delta_r_star,
        )
    else:
        end_x = best_gap.x_rear + scenario.v_ref * score.t_m - score.delta_r_star
        x = end_x + scenario.v_ref * (t - score.t_m)
        v = scenario.v_ref
        a = 0.0
    return TrajectorySample(
        t=t,
        vehicle_id=f"target_lane_rear_{int(best_gap.x_rear)}m",
        role=SELECTED_REAR_ROLE,
        x=x,
        v=v,
        a=a,
        is_selected_gap_vehicle=True,
        is_merge_vehicle=False,
    )


def sample_selected_front_vehicle_state(
    t: float,
    scenario: ScenarioConfig,
    best_gap: Gap,
    score: ScoreResult,
) -> TrajectorySample:
    if t <= score.t_m:
        x, v, a = _sample_quintic_phase(
            t=t,
            t_m=score.t_m,
            x_base=best_gap.x_front,
            v_ref=scenario.v_ref,
            delta_x=score.delta_f_star,
        )
    else:
        end_x = best_gap.x_front + scenario.v_ref * score.t_m + score.delta_f_star
        x = end_x + scenario.v_ref * (t - score.t_m)
        v = scenario.v_ref
        a = 0.0
    return TrajectorySample(
        t=t,
        vehicle_id=f"target_lane_front_{int(best_gap.x_front)}m",
        role=SELECTED_FRONT_ROLE,
        x=x,
        v=v,
        a=a,
        is_selected_gap_vehicle=True,
        is_merge_vehicle=False,
    )


def sample_constant_speed_vehicle_state(
    t: float,
    vehicle_id: str,
    x0: float,
    v_ref: float,
    role: str,
    is_selected_gap_vehicle: bool,
    is_merge_vehicle: bool,
) -> TrajectorySample:
    return TrajectorySample(
        t=t,
        vehicle_id=vehicle_id,
        role=role,
        x=x0 + v_ref * t,
        v=v_ref,
        a=0.0,
        is_selected_gap_vehicle=is_selected_gap_vehicle,
        is_merge_vehicle=is_merge_vehicle,
    )


def compute_dynamic_gap_length(x_front: float, x_rear: float) -> float:
    return x_front - x_rear


def compute_dynamic_gap_center(x_front: float, x_rear: float) -> float:
    return (x_front + x_rear) / 2.0


def _format_target_lane_vehicle_id(x0: float) -> str:
    return f"target_lane_{int(x0)}m"


def build_best_gap_trajectory_bundle(
    scenario: ScenarioConfig,
    gaps: tuple[Gap, ...],
    best_score: ScoreResult,
    contract: TrajectoryContract,
) -> TrajectoryBundle:
    best_gap = next(gap for gap in gaps if gap.gap_id == best_score.gap_id)
    times = build_time_grid(best_score.t_m, contract.sampling_dt)
    samples: list[TrajectorySample] = []

    selected_rear_id, selected_front_id = contract.selected_gap_vehicle_ids

    for t in times:
        merge_sample = sample_merge_vehicle_state(t, scenario, best_score)
        samples.append(merge_sample)

        rear_sample = sample_selected_rear_vehicle_state(t, scenario, best_gap, best_score)
        rear_sample = TrajectorySample(
            t=rear_sample.t,
            vehicle_id=selected_rear_id,
            role=rear_sample.role,
            x=rear_sample.x,
            v=rear_sample.v,
            a=rear_sample.a,
            is_selected_gap_vehicle=rear_sample.is_selected_gap_vehicle,
            is_merge_vehicle=rear_sample.is_merge_vehicle,
        )
        samples.append(rear_sample)

        front_sample = sample_selected_front_vehicle_state(t, scenario, best_gap, best_score)
        front_sample = TrajectorySample(
            t=front_sample.t,
            vehicle_id=selected_front_id,
            role=front_sample.role,
            x=front_sample.x,
            v=front_sample.v,
            a=front_sample.a,
            is_selected_gap_vehicle=front_sample.is_selected_gap_vehicle,
            is_merge_vehicle=front_sample.is_merge_vehicle,
        )
        samples.append(front_sample)

        for x0 in scenario.x_targets:
            if isclose(x0, best_gap.x_rear, rel_tol=0.0, abs_tol=1e-9) or isclose(
                x0,
                best_gap.x_front,
                rel_tol=0.0,
                abs_tol=1e-9,
            ):
                continue
            samples.append(
                sample_constant_speed_vehicle_state(
                    t=t,
                    vehicle_id=_format_target_lane_vehicle_id(x0),
                    x0=x0,
                    v_ref=scenario.v_ref,
                    role=NON_SELECTED_ROLE,
                    is_selected_gap_vehicle=False,
                    is_merge_vehicle=False,
                )
            )

    return TrajectoryBundle(
        selected_gap_id=best_gap.gap_id,
        selected_gap_interval=(best_gap.x_rear, best_gap.x_front),
        merge_time_s=best_score.t_m,
        merge_point_x=best_score.p_m,
        samples=tuple(samples),
        check_times=contract.required_check_times,
    )
