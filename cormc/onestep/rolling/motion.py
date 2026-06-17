from __future__ import annotations

from dataclasses import dataclass, replace
from math import isclose
from types import MappingProxyType
from typing import Any, Mapping

from cormc.simulation_core.pre_freeze import (
    DEFAULT_ROAD_GEOMETRY,
    LANE_2,
    MAINLINE,
    RoadGeometryConfig,
    SimulationState,
)
from cormc.simulation_core.commit import (
    CandidateKinematics,
    CandidateLaneState,
    CandidateStateTransition,
    TEST_HARNESS_CANDIDATE_SOURCE,
)
from cormc.onestep.kernel.models import TrajectorySample
from cormc.onestep.kernel.trajectory import (
    MERGE_VEHICLE_ROLE,
    SELECTED_FRONT_ROLE,
    SELECTED_REAR_ROLE,
    compute_dynamic_gap_center,
    sample_merge_vehicle_state,
    sample_selected_front_vehicle_state,
    sample_selected_rear_vehicle_state,
)
from cormc.onestep.kernel.timing_scoring import S_double_prime, S_prime

from cormc.onestep.rolling.planner import (
    MERGE_STATE_MERGE_COMPLETED,
    MERGE_STATE_NORMAL,
    MERGE_STATE_MERGING,
    TRAJECTORY_APPROACHING,
    TRAJECTORY_MERGE_EXECUTION,
    derive_zone_state,
)
from cormc.onestep.rolling.state import (
    GapPlan,
    GapRef,
    LateralTrajectoryRef,
    MVPlanState,
    OneStepBoundaryState,
    OneStepPlanBundle,
    PlannedTrajectory,
    RampMergeRuntimeState,
    ZONE_MERGE,
)


STAGE2_LATERAL_DURATION_STEPS = 10


@dataclass(frozen=True)
class MotionBuildResult:
    candidate_kinematics: Mapping[str, tuple[CandidateKinematics, ...]]
    candidate_lane_state: Mapping[str, CandidateLaneState]
    candidate_state_transitions: Mapping[str, tuple[CandidateStateTransition, ...]]
    runtime: RampMergeRuntimeState
    motion_events: tuple[Mapping[str, Any], ...] = ()
    bundle_lifecycle_records: tuple[Mapping[str, Any], ...] = ()


@dataclass(frozen=True)
class TrajectoryAdvanceResult:
    candidate: CandidateKinematics
    trajectory: PlannedTrajectory | None
    completed: bool
    event_record: Mapping[str, Any]


def build_default_motion_candidates(
    state: SimulationState,
) -> MappingProxyType[str, tuple[CandidateKinematics, ...]]:
    candidates: dict[str, tuple[CandidateKinematics, ...]] = {}
    for vehicle_id in state.active_vehicle_ids:
        current = state.vehicle_states[vehicle_id]
        candidates[vehicle_id] = (
            _default_candidate(state, vehicle_id),
        )
    return MappingProxyType(candidates)


def build_motion_outputs(
    state: SimulationState,
    runtime: RampMergeRuntimeState,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    algorithm_variant: str = "legacy_batch_c",
) -> MotionBuildResult:
    if algorithm_variant == "onestep_stage2":
        return _build_onestep_stage2_motion_outputs(state, runtime)
    candidates: dict[str, tuple[CandidateKinematics, ...]] = {}
    lane_states: dict[str, CandidateLaneState] = {}
    state_transitions: dict[str, tuple[CandidateStateTransition, ...]] = {}
    mv_plan_states = dict(runtime.mv_plan_states)
    planned_trajectories = dict(runtime.planned_trajectories)
    motion_records: list[Mapping[str, Any]] = []

    for vehicle_id in state.active_vehicle_ids:
        mv_state = mv_plan_states.get(vehicle_id)
        trajectory = (
            planned_trajectories.get(mv_state.planned_trajectory_id)
            if mv_state is not None and mv_state.planned_trajectory_id is not None
            else None
        )
        if trajectory is None:
            candidate = _default_candidate(state, vehicle_id)
            candidates[vehicle_id] = (candidate,)
            continue

        advance = advance_planned_trajectory(
            state,
            trajectory,
            geometry=geometry,
        )
        candidates[vehicle_id] = (advance.candidate,)
        if not advance.completed:
            motion_records.append(
                _motion_event_record_with_mv_state(
                    advance.event_record,
                    mv_state,
                )
            )
        if advance.completed:
            lane_state = CandidateLaneState(
                candidate_id=advance.candidate.candidate_id,
                vehicle_id=vehicle_id,
                physical_lane=LANE_2,
                road_role=MAINLINE,
                reason="batch_c_merge_completed",
            )
            transition = CandidateStateTransition(
                candidate_id=advance.candidate.candidate_id,
                vehicle_id=vehicle_id,
                state_name="merge_state",
                old_state=state.vehicle_states[vehicle_id].merge_state,
                new_state=MERGE_STATE_NORMAL,
                reason="batch_c_merge_completed",
            )
            lane_states[vehicle_id] = lane_state
            state_transitions[vehicle_id] = (transition,)
            planned_trajectories.pop(trajectory.trajectory_id, None)
            if mv_state is not None:
                mv_plan_states[vehicle_id] = replace(
                    mv_state,
                    merge_state=MERGE_STATE_NORMAL,
                    current_plan_gap=None,
                    locked_gap=None,
                    planned_trajectory_id=None,
                )
            motion_records.append(
                {
                    "mv_id": vehicle_id,
                    "zone_state": mv_state.zone_state if mv_state is not None else None,
                    "current_plan_gap": None,
                    "locked_gap": None,
                    "selected_gap": trajectory.target_gap,
                    "trajectory_id": trajectory.trajectory_id,
                    "trajectory_kind": trajectory.kind,
                    "progress_step": (
                        advance.trajectory.progress_step
                        if advance.trajectory is not None
                        else trajectory.duration_steps
                    ),
                    "duration_steps": trajectory.duration_steps,
                    "merge_completed": True,
                    "reason": "batch_c_merge_completed",
                    "step": state.step,
                    "t": state.t,
                }
            )
        elif advance.trajectory is not None:
            planned_trajectories[advance.trajectory.trajectory_id] = advance.trajectory

    return MotionBuildResult(
        candidate_kinematics=MappingProxyType(candidates),
        candidate_lane_state=MappingProxyType(lane_states),
        candidate_state_transitions=MappingProxyType(state_transitions),
        runtime=replace(
            runtime,
            mv_plan_states=MappingProxyType(mv_plan_states),
            planned_trajectories=MappingProxyType(planned_trajectories),
            version="batch_c_v1",
        ),
        motion_events=tuple(motion_records),
    )


def advance_planned_trajectory(
    state: SimulationState,
    trajectory: PlannedTrajectory,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> TrajectoryAdvanceResult:
    current = state.vehicle_states[trajectory.mv_id]
    x_next = current.x_global + current.v * state.dt
    completed = False
    next_trajectory: PlannedTrajectory | None = trajectory

    if trajectory.kind == TRAJECTORY_APPROACHING:
        y_next = current.y
        progress_step = trajectory.progress_step
    elif trajectory.kind == TRAJECTORY_MERGE_EXECUTION:
        progress_step = trajectory.progress_step + 1
        duration_steps = max(trajectory.duration_steps, 1)
        ratio = min(progress_step / duration_steps, 1.0)
        y_next = trajectory.start_y + (trajectory.target_y - trajectory.start_y) * ratio
        completed = (
            progress_step >= trajectory.duration_steps
            or abs(y_next - float(geometry.lane_centerlines[LANE_2])) <= 1e-6
        )
        next_trajectory = replace(trajectory, progress_step=progress_step)
    else:
        y_next = current.y
        progress_step = trajectory.progress_step

    candidate = CandidateKinematics(
        candidate_id=f"ramp_merge_trajectory:{state.step}:{trajectory.mv_id}",
        vehicle_id=trajectory.mv_id,
        x_global=x_next,
        y=y_next,
        v=current.v,
        a=0.0,
        source=TEST_HARNESS_CANDIDATE_SOURCE,
    )
    return TrajectoryAdvanceResult(
        candidate=candidate,
        trajectory=next_trajectory,
        completed=completed,
        event_record={
            "mv_id": trajectory.mv_id,
            "zone_state": None,
            "current_plan_gap": None,
            "locked_gap": trajectory.target_gap,
            "selected_gap": trajectory.target_gap,
            "trajectory_id": trajectory.trajectory_id,
            "trajectory_kind": trajectory.kind,
            "progress_step": progress_step,
            "duration_steps": trajectory.duration_steps,
            "merge_completed": completed,
            "reason": "advance_planned_trajectory",
            "step": state.step,
            "t": state.t,
        },
    )


def _default_candidate(state: SimulationState, vehicle_id: str) -> CandidateKinematics:
    current = state.vehicle_states[vehicle_id]
    return CandidateKinematics(
        candidate_id=f"ramp_merge_default_motion:{state.step}:{vehicle_id}",
        vehicle_id=vehicle_id,
        x_global=current.x_global + current.v * state.dt,
        y=current.y,
        v=current.v,
        a=0.0,
        source=TEST_HARNESS_CANDIDATE_SOURCE,
    )


def _motion_event_record_with_mv_state(
    record: Mapping[str, Any],
    mv_state: MVPlanState | None,
) -> Mapping[str, Any]:
    if mv_state is None:
        return record
    return {
        **dict(record),
        "zone_state": mv_state.zone_state,
        "current_plan_gap": mv_state.current_plan_gap,
        "locked_gap": mv_state.locked_gap,
    }


def _build_onestep_stage2_motion_outputs(
    state: SimulationState,
    runtime: RampMergeRuntimeState,
) -> MotionBuildResult:
    candidates: dict[str, tuple[CandidateKinematics, ...]] = {}
    lane_states: dict[str, CandidateLaneState] = {}
    state_transitions: dict[str, tuple[CandidateStateTransition, ...]] = {}
    mv_states = dict(runtime.mv_plan_states)
    bundles = dict(runtime.onestep_plan_bundles)
    controlled = dict(runtime.controlled_vehicle_states)
    gap_plans = dict(runtime.gap_plans)
    lateral_trajectories = dict(runtime.lateral_trajectories)
    motion_records: list[Mapping[str, Any]] = []
    bundle_lifecycle_records: list[Mapping[str, Any]] = []

    lock_records = _lock_stage2_merge_zone_gaps(state, mv_states, gap_plans)
    motion_records.extend(lock_records)
    lateral_start_records, release_records = _start_lateral_if_longitudinal_ready(
        state,
        mv_states,
        bundles,
        controlled,
        gap_plans,
        lateral_trajectories,
    )
    motion_records.extend(lateral_start_records)
    bundle_lifecycle_records.extend(release_records)

    for vehicle_id in state.active_vehicle_ids:
        mv_state = mv_states.get(vehicle_id)
        lateral_ref = (
            lateral_trajectories.get(mv_state.active_lateral_trajectory_id)
            if mv_state is not None
            and mv_state.active_lateral_trajectory_id is not None
            else None
        )
        if (
            mv_state is not None
            and mv_state.merge_state == MERGE_STATE_MERGING
            and lateral_ref is not None
        ):
            advance = _advance_stage2_lateral_trajectory(state, lateral_ref)
            candidates[vehicle_id] = (advance["candidate"],)
            motion_records.append(advance["record"])
            if advance["completed"]:
                lane_states[vehicle_id] = CandidateLaneState(
                    candidate_id=advance["candidate"].candidate_id,
                    vehicle_id=vehicle_id,
                    physical_lane=state.vehicle_states[vehicle_id].physical_lane,
                    road_role=state.vehicle_states[vehicle_id].road_role,
                    reason="onestep_stage2_merge_completed_wait_step0",
                )
                state_transitions[vehicle_id] = (
                    CandidateStateTransition(
                        candidate_id=advance["candidate"].candidate_id,
                        vehicle_id=vehicle_id,
                        state_name="merge_state",
                        old_state=state.vehicle_states[vehicle_id].merge_state,
                        new_state=MERGE_STATE_MERGE_COMPLETED,
                        reason="onestep_stage2_merge_completed",
                    ),
                )
                lateral_trajectories.pop(lateral_ref.trajectory_id, None)
                mv_states[vehicle_id] = replace(
                    mv_state,
                    merge_state=MERGE_STATE_MERGE_COMPLETED,
                    active_lateral_trajectory_id=None,
                )
            continue

        controlled_state = controlled.get(vehicle_id)
        bundle = (
            bundles.get(controlled_state.bundle_id)
            if controlled_state is not None
            else None
        )
        if controlled_state is None or bundle is None:
            candidate = _constant_20_candidate(state, vehicle_id)
            candidates[vehicle_id] = (candidate,)
            continue

        sample_time = (state.t + state.dt) - bundle.start_t
        sample = _sample_bundle_vehicle_state(sample_time, bundle, controlled_state.role)
        candidate = CandidateKinematics(
            candidate_id=f"ramp_merge_onestep_stage2:{state.step}:{vehicle_id}",
            vehicle_id=vehicle_id,
            x_global=sample.x,
            y=state.vehicle_states[vehicle_id].y,
            v=sample.v,
            a=sample.a,
            source=TEST_HARNESS_CANDIDATE_SOURCE,
        )
        candidates[vehicle_id] = (candidate,)

    return MotionBuildResult(
        candidate_kinematics=MappingProxyType(candidates),
        candidate_lane_state=MappingProxyType(lane_states),
        candidate_state_transitions=MappingProxyType(state_transitions),
        runtime=replace(
            runtime,
            mv_plan_states=MappingProxyType(mv_states),
            onestep_plan_bundles=MappingProxyType(bundles),
            controlled_vehicle_states=MappingProxyType(controlled),
            gap_plans=MappingProxyType(gap_plans),
            lateral_trajectories=MappingProxyType(lateral_trajectories),
            version="onestep_stage2_v1",
        ),
        motion_events=tuple(motion_records),
        bundle_lifecycle_records=tuple(bundle_lifecycle_records),
    )


def _constant_20_candidate(state: SimulationState, vehicle_id: str) -> CandidateKinematics:
    current = state.vehicle_states[vehicle_id]
    default_v = 20.0
    return CandidateKinematics(
        candidate_id=f"ramp_merge_onestep_stage2_default:{state.step}:{vehicle_id}",
        vehicle_id=vehicle_id,
        x_global=current.x_global + default_v * state.dt,
        y=current.y,
        v=default_v,
        a=0.0,
        source=TEST_HARNESS_CANDIDATE_SOURCE,
    )


def _sample_bundle_vehicle_state(
    sample_time: float,
    bundle: OneStepPlanBundle,
    role: str,
):
    boundary = _bundle_boundary_state(bundle, role)
    target_x_global = _bundle_target_x_global(bundle, role)
    target_v = _bundle_target_v(bundle, role)
    target_a = 0.0
    duration = max(float(bundle.best_score.t_m), 1e-6)
    effective_time = min(max(float(sample_time), 0.0), duration)
    x_global, v, a = _sample_rolling_quintic_boundary(
        t=effective_time,
        duration=duration,
        x0=boundary.x_global,
        v0=boundary.v,
        a0=boundary.a,
        x1=target_x_global,
        v1=target_v,
        a1=target_a,
    )
    if sample_time > duration and not isclose(sample_time, duration, rel_tol=0.0, abs_tol=1e-9):
        coast_dt = sample_time - duration
        x_global = target_x_global + target_v * coast_dt
        v = target_v
        a = 0.0
    return _trajectory_sample_for_role(
        role=role,
        sample_time=sample_time,
        x_global=x_global,
        v=v,
        a=a,
        bundle=bundle,
    )
 
 
def _bundle_boundary_state(bundle: OneStepPlanBundle, role: str) -> OneStepBoundaryState:
    vehicle_id = _bundle_vehicle_id_for_role(bundle, role)
    return bundle.boundary_state_by_vehicle_id[vehicle_id]


def _bundle_vehicle_id_for_role(bundle: OneStepPlanBundle, role: str) -> str:
    if role == "mv":
        return bundle.mv_id
    if role == "rear":
        return bundle.selected_rear_vehicle_id
    if role == "front":
        return bundle.selected_front_vehicle_id
    raise ValueError(f"unknown stage2 controlled role: {role}")


def _bundle_target_x_global(bundle: OneStepPlanBundle, role: str) -> float:
    sample_time = float(bundle.best_score.t_m)
    if sample_time <= 1e-12:
        if role == "mv":
            return float(bundle.merge_point_x_global)
        if role == "rear":
            return float(bundle.origin_x_global) + float(bundle.best_gap.x_rear) - float(bundle.best_score.delta_r_star)
        if role == "front":
            return float(bundle.origin_x_global) + float(bundle.best_gap.x_front) + float(bundle.best_score.delta_f_star)
        raise ValueError(f"unknown stage2 controlled role: {role}")
    if role == "mv":
        sample = sample_merge_vehicle_state(
            sample_time,
            bundle.local_scenario,
            bundle.best_score,
        )
    elif role == "rear":
        sample = sample_selected_rear_vehicle_state(
            sample_time,
            bundle.local_scenario,
            bundle.best_gap,
            bundle.best_score,
        )
    elif role == "front":
        sample = sample_selected_front_vehicle_state(
            sample_time,
            bundle.local_scenario,
            bundle.best_gap,
            bundle.best_score,
        )
    else:
        raise ValueError(f"unknown stage2 controlled role: {role}")
    return float(bundle.origin_x_global) + float(sample.x)


def _bundle_target_v(bundle: OneStepPlanBundle, role: str) -> float:
    if role == "mv":
        return float(bundle.local_scenario.v_ref)
    return float(bundle.local_scenario.v_ref)


def _sample_rolling_quintic_boundary(
    *,
    t: float,
    duration: float,
    x0: float,
    v0: float,
    a0: float,
    x1: float,
    v1: float,
    a1: float,
) -> tuple[float, float, float]:
    if duration <= 0.0:
        return (x1, v1, a1)
    tau = t / duration
    h00 = 1.0 - 10.0 * tau**3 + 15.0 * tau**4 - 6.0 * tau**5
    h01 = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    h10 = tau - 6.0 * tau**3 + 8.0 * tau**4 - 3.0 * tau**5
    h11 = -4.0 * tau**3 + 7.0 * tau**4 - 3.0 * tau**5
    h20 = 0.5 * tau**2 - 1.5 * tau**3 + 1.5 * tau**4 - 0.5 * tau**5
    h21 = 0.5 * tau**3 - tau**4 + 0.5 * tau**5

    x = (
        h00 * x0
        + h01 * x1
        + duration * h10 * v0
        + duration * h11 * v1
        + (duration**2) * h20 * a0
        + (duration**2) * h21 * a1
    )

    h00_p = -S_prime(tau)
    h01_p = S_prime(tau)
    h10_p = 1.0 - 18.0 * tau**2 + 32.0 * tau**3 - 15.0 * tau**4
    h11_p = -12.0 * tau**2 + 28.0 * tau**3 - 15.0 * tau**4
    h20_p = tau - 4.5 * tau**2 + 6.0 * tau**3 - 2.5 * tau**4
    h21_p = 1.5 * tau**2 - 4.0 * tau**3 + 2.5 * tau**4
    v = (
        (h00_p * x0 + h01_p * x1) / duration
        + h10_p * v0
        + h11_p * v1
        + duration * h20_p * a0
        + duration * h21_p * a1
    )

    h00_pp = -S_double_prime(tau)
    h01_pp = S_double_prime(tau)
    h10_pp = -36.0 * tau + 96.0 * tau**2 - 60.0 * tau**3
    h11_pp = -24.0 * tau + 84.0 * tau**2 - 60.0 * tau**3
    h20_pp = 1.0 - 9.0 * tau + 18.0 * tau**2 - 10.0 * tau**3
    h21_pp = 3.0 * tau - 12.0 * tau**2 + 10.0 * tau**3
    a = (
        (h00_pp * x0 + h01_pp * x1) / (duration**2)
        + (h10_pp * v0 + h11_pp * v1) / duration
        + h20_pp * a0
        + h21_pp * a1
    )
    return (x, v, a)


def _trajectory_sample_for_role(
    *,
    role: str,
    sample_time: float,
    x_global: float,
    v: float,
    a: float,
    bundle: OneStepPlanBundle,
) -> TrajectorySample:
    if role == "mv":
        return TrajectorySample(
            t=sample_time,
            vehicle_id=bundle.mv_id,
            role=MERGE_VEHICLE_ROLE,
            x=x_global,
            v=v,
            a=a,
            is_selected_gap_vehicle=False,
            is_merge_vehicle=True,
        )
    if role == "rear":
        return TrajectorySample(
            t=sample_time,
            vehicle_id=bundle.selected_rear_vehicle_id,
            role=SELECTED_REAR_ROLE,
            x=x_global,
            v=v,
            a=a,
            is_selected_gap_vehicle=True,
            is_merge_vehicle=False,
        )
    if role == "front":
        return TrajectorySample(
            t=sample_time,
            vehicle_id=bundle.selected_front_vehicle_id,
            role=SELECTED_FRONT_ROLE,
            x=x_global,
            v=v,
            a=a,
            is_selected_gap_vehicle=True,
            is_merge_vehicle=False,
        )
    raise ValueError(f"unknown stage2 controlled role: {role}")


def _lock_stage2_merge_zone_gaps(
    state: SimulationState,
    mv_states: dict[str, MVPlanState],
    gap_plans: dict[str, GapPlan],
) -> tuple[Mapping[str, Any], ...]:
    records: list[Mapping[str, Any]] = []
    for mv_id in _ordered_mv_ids_by_x_desc(state, mv_states):
        mv_state = mv_states[mv_id]
        zone_state = derive_zone_state(state.vehicle_states[mv_id])
        if mv_state.zone_state != zone_state:
            mv_state = replace(mv_state, zone_state=zone_state)
            mv_states[mv_id] = mv_state
        if zone_state != ZONE_MERGE or mv_state.locked_gap is not None:
            continue
        if mv_state.current_plan_gap is None:
            records.append(
                _stage2_gap_lock_record(
                    state,
                    mv_id,
                    mv_state,
                    reason="onestep_stage2_merge_zone_no_locked_gap",
                )
            )
            continue
        locked_plan_id = mv_state.current_plan_id
        if locked_plan_id is not None and locked_plan_id not in gap_plans:
            locked_plan_id = None
        mv_state = replace(
            mv_state,
            locked_gap=mv_state.current_plan_gap,
            locked_plan_id=locked_plan_id,
        )
        mv_states[mv_id] = mv_state
        records.append(
            _stage2_gap_lock_record(
                state,
                mv_id,
                mv_state,
                reason="onestep_stage2_merge_zone_gap_locked",
            )
        )
    return tuple(records)


def _start_lateral_if_longitudinal_ready(
    state: SimulationState,
    mv_states: dict[str, MVPlanState],
    bundles: dict[str, OneStepPlanBundle],
    controlled: dict[str, Any],
    gap_plans: dict[str, GapPlan],
    lateral_trajectories: dict[str, LateralTrajectoryRef],
) -> tuple[tuple[Mapping[str, Any], ...], tuple[Mapping[str, Any], ...]]:
    lateral_start_records: list[Mapping[str, Any]] = []
    release_records: list[Mapping[str, Any]] = []
    start_candidates = _candidate_snapshot_for_longitudinal_ready(state, bundles, controlled)
    for mv_id in _ordered_mv_ids_by_x_desc(state, mv_states):
        mv_state = mv_states[mv_id]
        if mv_state.zone_state != ZONE_MERGE:
            continue
        if mv_state.locked_gap is None:
            continue
        if mv_state.merge_state != MERGE_STATE_NORMAL:
            continue
        if mv_state.active_lateral_trajectory_id is not None:
            continue
        if mv_state.active_bundle_id is None:
            continue
        bundle = bundles.get(mv_state.active_bundle_id)
        if bundle is None:
            continue
        mv_candidate = start_candidates.get(mv_id, ())
        if not mv_candidate:
            continue
        completion_eval = _evaluate_stage2_longitudinal_ready(bundle, start_candidates)
        if not completion_eval["longitudinal_ready"]:
            continue
        lateral_ref = _build_stage2_lateral_ref(state, mv_id, mv_state)
        lateral_trajectories[lateral_ref.trajectory_id] = lateral_ref
        release_records.append(
            _release_bundle_in_motion(
                mv_states,
                bundles,
                controlled,
                gap_plans,
                bundle,
                reason="onestep_stage2_lateral_start_release",
                step=state.step,
                t=state.t,
            )
        )
        mv_states[mv_id] = replace(
            mv_states[mv_id],
            merge_state=MERGE_STATE_MERGING,
            active_lateral_trajectory_id=lateral_ref.trajectory_id,
        )
        lateral_start_records.append({
            "mv_id": mv_id,
            "bundle_id": bundle.bundle_id,
            "merge_point_x_global": bundle.merge_point_x_global,
            "x_global_next": completion_eval["mv_x_global_next"],
            "front_gap_m": completion_eval["front_gap_m"],
            "rear_gap_m": completion_eval["rear_gap_m"],
            "gap_center_x_global": completion_eval["gap_center_x_global"],
            "required_longitudinal_gap_m": completion_eval["required_longitudinal_gap_m"],
            "merge_point_reached": completion_eval["merge_point_reached"],
            "longitudinal_ready_rule": completion_eval["rule"],
            "selected_vehicle_ids": list(bundle.selected_vehicle_ids),
            "longitudinal_completed": True,
            "lateral_started": True,
            "trajectory_id": lateral_ref.trajectory_id,
            "active_lateral_trajectory_id": lateral_ref.trajectory_id,
            "trajectory_kind": TRAJECTORY_MERGE_EXECUTION,
            "current_plan_gap": mv_state.current_plan_gap,
            "locked_gap": mv_state.locked_gap,
            "selected_gap": mv_state.locked_gap,
            "locked_plan_id": mv_state.locked_plan_id,
            "source_plan_id": lateral_ref.source_plan_id,
            "start_y": lateral_ref.start_y,
            "target_y": lateral_ref.target_y,
            "duration_steps": lateral_ref.duration_steps,
            "merge_state": MERGE_STATE_MERGING,
            "reason": "onestep_stage2_lateral_started",
            "step": state.step,
            "t": state.t,
        })
    return (tuple(lateral_start_records), tuple(release_records))


def _candidate_snapshot_for_longitudinal_ready(
    state: SimulationState,
    bundles: Mapping[str, OneStepPlanBundle],
    controlled: Mapping[str, Any],
) -> dict[str, tuple[CandidateKinematics, ...]]:
    candidates: dict[str, tuple[CandidateKinematics, ...]] = {}
    for vehicle_id in state.active_vehicle_ids:
        controlled_state = controlled.get(vehicle_id)
        bundle = (
            bundles.get(controlled_state.bundle_id)
            if controlled_state is not None
            else None
        )
        if controlled_state is None or bundle is None:
            candidate = _constant_20_candidate(state, vehicle_id)
        else:
            sample_time = (state.t + state.dt) - bundle.start_t
            sample = _sample_bundle_vehicle_state(sample_time, bundle, controlled_state.role)
            candidate = CandidateKinematics(
                candidate_id=f"ramp_merge_onestep_stage2:{state.step}:{vehicle_id}",
                vehicle_id=vehicle_id,
                x_global=sample.x,
                y=state.vehicle_states[vehicle_id].y,
                v=sample.v,
                a=sample.a,
                source=TEST_HARNESS_CANDIDATE_SOURCE,
            )
        candidates[vehicle_id] = (candidate,)
    return candidates


def _release_bundle_in_motion(
    mv_states: dict[str, MVPlanState],
    bundles: dict[str, OneStepPlanBundle],
    controlled: dict[str, Any],
    gap_plans: dict[str, GapPlan],
    bundle: OneStepPlanBundle,
    *,
    reason: str,
    step: int,
    t: float,
) -> Mapping[str, Any]:
    bundles.pop(bundle.bundle_id, None)
    for vehicle_id, controlled_state in tuple(controlled.items()):
        if controlled_state.bundle_id == bundle.bundle_id:
            controlled.pop(vehicle_id, None)
    mv_state = mv_states[bundle.mv_id]
    mv_states[bundle.mv_id] = replace(
        mv_state,
        active_bundle_id=None,
    )
    for plan_id, plan in tuple(gap_plans.items()):
        if plan.bundle_id == bundle.bundle_id:
            gap_plans[plan_id] = replace(plan, bundle_id=None)
    return {
        "bundle_action": "bundle_released",
        "bundle_id": bundle.bundle_id,
        "replaced_bundle_id": None,
        "mv_id": bundle.mv_id,
        "gap_plan_id": mv_state.current_plan_id,
        "bundle_shape": bundle.bundle_shape,
        "controlled_vehicle_ids": list(bundle.controlled_vehicle_ids),
        "controlled_roles_by_vehicle_id": dict(bundle.controlled_roles_by_vehicle_id),
        "gap_boundary_vehicle_ids": [
            bundle.selected_rear_vehicle_id,
            bundle.selected_front_vehicle_id,
        ],
        "selected_front_vehicle_id": bundle.selected_front_vehicle_id,
        "selected_rear_vehicle_id": bundle.selected_rear_vehicle_id,
        "selected_gap": bundle.selected_gap,
        "reason": reason,
        "round_id": None,
        "round_order": None,
        "step": step,
        "t": t,
    }


def _build_stage2_lateral_ref(
    state: SimulationState,
    mv_id: str,
    mv_state: MVPlanState,
) -> LateralTrajectoryRef:
    current = state.vehicle_states[mv_id]
    source_plan_id = (
        mv_state.locked_plan_id
        or mv_state.current_plan_id
        or f"gap_plan:unknown:{mv_id}"
    )
    return LateralTrajectoryRef(
        trajectory_id=f"onestep_stage2_lateral:{state.step}:{mv_id}",
        owner_mv_id=mv_id,
        source_plan_id=source_plan_id,
        start_step=state.step,
        start_t=state.t,
        start_y=float(current.y),
        target_y=float(DEFAULT_ROAD_GEOMETRY.lane_centerlines[LANE_2]),
        duration_steps=STAGE2_LATERAL_DURATION_STEPS,
    )


def _advance_stage2_lateral_trajectory(
    state: SimulationState,
    lateral_ref: LateralTrajectoryRef,
) -> dict[str, Any]:
    current = state.vehicle_states[lateral_ref.owner_mv_id]
    progress_step = max(state.step - lateral_ref.start_step + 1, 0)
    duration_steps = max(int(lateral_ref.duration_steps), 1)
    ratio = min(progress_step / duration_steps, 1.0)
    y_next = lateral_ref.start_y + (lateral_ref.target_y - lateral_ref.start_y) * ratio
    completed = (
        progress_step >= duration_steps
        or abs(y_next - lateral_ref.target_y) <= 1e-6
    )
    candidate = CandidateKinematics(
        candidate_id=f"onestep_stage2_lateral:{state.step}:{lateral_ref.owner_mv_id}",
        vehicle_id=lateral_ref.owner_mv_id,
        x_global=current.x_global + current.v * state.dt,
        y=y_next,
        v=current.v,
        a=0.0,
        source=TEST_HARNESS_CANDIDATE_SOURCE,
    )
    return {
        "candidate": candidate,
        "completed": completed,
        "record": {
            "mv_id": lateral_ref.owner_mv_id,
            "trajectory_id": lateral_ref.trajectory_id,
            "active_lateral_trajectory_id": lateral_ref.trajectory_id,
            "trajectory_kind": TRAJECTORY_MERGE_EXECUTION,
            "source_plan_id": lateral_ref.source_plan_id,
            "progress_step": progress_step,
            "duration_steps": duration_steps,
            "start_y": lateral_ref.start_y,
            "target_y": lateral_ref.target_y,
            "y_next": y_next,
            "x_global_next": candidate.x_global,
            "merge_completed": completed,
            "reason": (
                "onestep_stage2_merge_completed"
                if completed
                else "onestep_stage2_lateral_progress"
            ),
            "step": state.step,
            "t": state.t,
        },
    }


def _stage2_gap_lock_record(
    state: SimulationState,
    mv_id: str,
    mv_state: MVPlanState,
    *,
    reason: str,
) -> Mapping[str, Any]:
    return {
        "mv_id": mv_id,
        "zone_state": mv_state.zone_state,
        "current_plan_gap": mv_state.current_plan_gap,
        "locked_gap": mv_state.locked_gap,
        "selected_gap": mv_state.locked_gap,
        "locked_plan_id": mv_state.locked_plan_id,
        "active_bundle_id": mv_state.active_bundle_id,
        "reason": reason,
        "step": state.step,
        "t": state.t,
    }


def _ordered_mv_ids_by_x_desc(
    state: SimulationState,
    mv_states: Mapping[str, MVPlanState],
) -> tuple[str, ...]:
    return tuple(
        sorted(
            (
                mv_id
                for mv_id in mv_states
                if mv_id in state.active_vehicle_ids
                and state.vehicle_states[mv_id].is_active
            ),
            key=lambda mv_id: (
                -state.vehicle_states[mv_id].x_global,
                mv_id,
            ),
        )
    )


def _evaluate_stage2_longitudinal_ready(
    bundle: OneStepPlanBundle,
    candidates: Mapping[str, tuple[CandidateKinematics, ...]],
) -> dict[str, float | bool | str]:
    mv_candidate = candidates[bundle.mv_id][0]
    rear_candidate = candidates[bundle.selected_rear_vehicle_id][0]
    front_candidate = candidates[bundle.selected_front_vehicle_id][0]
    front_gap_m = float(front_candidate.x_global) - float(mv_candidate.x_global)
    rear_gap_m = float(mv_candidate.x_global) - float(rear_candidate.x_global)
    required_gap_m = float(bundle.required_longitudinal_gap_m)
    merge_point_reached = float(mv_candidate.x_global) + 1e-9 >= float(bundle.merge_point_x_global)
    longitudinal_ready = (
        merge_point_reached
        and front_gap_m + 1e-9 >= required_gap_m
        and rear_gap_m + 1e-9 >= required_gap_m
    )
    return {
        "mv_x_global_next": float(mv_candidate.x_global),
        "front_gap_m": front_gap_m,
        "rear_gap_m": rear_gap_m,
        "gap_center_x_global": compute_dynamic_gap_center(
            float(front_candidate.x_global),
            float(rear_candidate.x_global),
        ),
        "required_longitudinal_gap_m": required_gap_m,
        "merge_point_reached": merge_point_reached,
        "longitudinal_ready": longitudinal_ready,
        "rule": "merge_point_reached_and_front_rear_gap_ge_Greq_over_2",
    }
