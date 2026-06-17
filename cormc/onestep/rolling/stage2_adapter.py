from __future__ import annotations

from dataclasses import dataclass

from cormc.onestep.lab.reference_case import get_reference_algorithm_config
from cormc.onestep.kernel.config import (
    AlgorithmConfig,
    GapBoundaryControllability,
    ScenarioConfig,
)
from cormc.onestep.kernel.evaluation import evaluate_one_step_scenario
from cormc.onestep.kernel.models import OneStepEvaluationResult
from cormc.onestep.rolling.gaps import identify_and_number_gaps
from cormc.onestep.rolling.safety import effective_controllable
from cormc.onestep.rolling.state import GapSnapshot
from cormc.simulation_core.pre_freeze import SimulationState


@dataclass(frozen=True)
class Stage2OneStepEvaluation:
    origin_x_global: float
    local_frame: dict[str, object]
    local_scenario: ScenarioConfig
    algorithm: AlgorithmConfig
    evaluation: OneStepEvaluationResult


def build_stage2_local_frame(state: SimulationState, mv_id: str) -> dict[str, object]:
    runtime = state.ramp_merge_runtime
    danger_vehicle_ids = (
        tuple(runtime.danger_vehicle_ids)
        if runtime is not None and hasattr(runtime, "danger_vehicle_ids")
        else ()
    )
    gap_snapshot = identify_and_number_gaps(state, danger_vehicle_ids)
    return build_stage2_local_frame_from_snapshot(state, mv_id, gap_snapshot)


def build_stage2_local_frame_from_snapshot(
    state: SimulationState,
    mv_id: str,
    gap_snapshot: GapSnapshot,
) -> dict[str, object]:
    return _build_stage2_local_frame_for_runtime_gap_indices(
        state,
        mv_id,
        gap_snapshot,
        tuple(gap.index for gap in gap_snapshot.gaps),
    )


def _build_stage2_local_frame_for_runtime_gap_indices(
    state: SimulationState,
    mv_id: str,
    gap_snapshot: GapSnapshot,
    runtime_gap_indices: tuple[int, ...],
) -> dict[str, object]:
    origin_x_global = float(state.vehicle_states[mv_id].x_global)
    snapshot_lane_2_vehicle_order = _lane_2_vehicle_order_from_snapshot(gap_snapshot)
    lane_2_vehicle_order = _lane_2_vehicle_order_for_runtime_gap_indices(
        gap_snapshot,
        runtime_gap_indices,
    )
    snapshot_x_local_by_id = {
        vehicle_id: float(state.vehicle_states[vehicle_id].x_global) - origin_x_global
        for vehicle_id in snapshot_lane_2_vehicle_order
    }
    lane_2_vehicle_x_local_by_id = {
        vehicle_id: snapshot_x_local_by_id[vehicle_id]
        for vehicle_id in lane_2_vehicle_order
    }
    included_gaps = [
        gap for gap in gap_snapshot.gaps if gap.index in set(runtime_gap_indices)
    ]
    kernel_ordered_gaps = tuple(reversed(included_gaps))
    gap_intervals_local = tuple(
        (
            lane_2_vehicle_x_local_by_id[gap.rear_vehicle_id],
            lane_2_vehicle_x_local_by_id[gap.front_vehicle_id],
        )
        for gap in kernel_ordered_gaps
    )
    gap_centers_local = tuple(
        (interval[0] + interval[1]) / 2.0 for interval in gap_intervals_local
    )
    gap_vehicle_ids_by_index = {
        index: (gap.rear_vehicle_id, gap.front_vehicle_id)
        for index, gap in enumerate(kernel_ordered_gaps)
    }
    runtime_gap_index_by_kernel_index = {
        index: gap.index for index, gap in enumerate(kernel_ordered_gaps)
    }
    kernel_index_by_runtime_gap_index = {
        gap.index: index for index, gap in enumerate(kernel_ordered_gaps)
    }
    snapshot_gap_vehicle_ids_by_index = {
        index: (gap.rear_vehicle_id, gap.front_vehicle_id)
        for index, gap in enumerate(gap_snapshot.gaps)
    }
    snapshot_runtime_gap_index_by_kernel_index = {
        index: gap.index for index, gap in enumerate(gap_snapshot.gaps)
    }
    snapshot_kernel_index_by_runtime_gap_index = {
        gap.index: index for index, gap in enumerate(gap_snapshot.gaps)
    }
    return {
        "mv_id": mv_id,
        "origin_x_global": origin_x_global,
        "lane_2_vehicle_order": lane_2_vehicle_order,
        "lane_2_vehicle_x_local_by_id": lane_2_vehicle_x_local_by_id,
        "snapshot_lane_2_vehicle_order": snapshot_lane_2_vehicle_order,
        "snapshot_lane_2_vehicle_x_local_by_id": snapshot_x_local_by_id,
        "gap_intervals_local": gap_intervals_local,
        "gap_centers_local": gap_centers_local,
        "gap_vehicle_ids_by_index": gap_vehicle_ids_by_index,
        "runtime_gap_index_by_kernel_index": runtime_gap_index_by_kernel_index,
        "kernel_index_by_runtime_gap_index": kernel_index_by_runtime_gap_index,
        "snapshot_gap_vehicle_ids_by_index": snapshot_gap_vehicle_ids_by_index,
        "snapshot_runtime_gap_index_by_kernel_index": snapshot_runtime_gap_index_by_kernel_index,
        "snapshot_kernel_index_by_runtime_gap_index": snapshot_kernel_index_by_runtime_gap_index,
    }


def build_stage2_local_scenario(state: SimulationState, mv_id: str) -> ScenarioConfig:
    runtime = state.ramp_merge_runtime
    danger_vehicle_ids = (
        tuple(runtime.danger_vehicle_ids)
        if runtime is not None and hasattr(runtime, "danger_vehicle_ids")
        else ()
    )
    gap_snapshot = identify_and_number_gaps(state, danger_vehicle_ids)
    return build_stage2_local_scenario_from_snapshot(
        state,
        mv_id,
        gap_snapshot,
        uncontrollable_vehicle_ids=(),
        allowed_gap_indices=None,
    )


def build_stage2_local_scenario_from_snapshot(
    state: SimulationState,
    mv_id: str,
    gap_snapshot: GapSnapshot,
    uncontrollable_vehicle_ids: tuple[str, ...] | set[str] | frozenset[str] = (),
    allowed_gap_indices: tuple[int, ...] | set[int] | frozenset[int] | None = None,
) -> ScenarioConfig:
    included_runtime_indices = _included_runtime_gap_indices(
        gap_snapshot,
        allowed_gap_indices,
    )
    local_frame = _build_stage2_local_frame_for_runtime_gap_indices(
        state,
        mv_id,
        gap_snapshot,
        included_runtime_indices,
    )
    x_targets = tuple(
        float(local_frame["lane_2_vehicle_x_local_by_id"][vehicle_id])
        for vehicle_id in local_frame["lane_2_vehicle_order"]
    )
    included_gaps = [
        gap for gap in gap_snapshot.gaps if gap.index in set(included_runtime_indices)
    ]
    kernel_ordered_gaps = tuple(reversed(included_gaps))
    runtime = state.ramp_merge_runtime
    danger_vehicle_ids = (
        tuple(runtime.danger_vehicle_ids)
        if runtime is not None and hasattr(runtime, "danger_vehicle_ids")
        else ()
    )
    blocked_vehicle_ids = set(danger_vehicle_ids) | set(uncontrollable_vehicle_ids)
    gap_boundary_controllability = tuple(
        GapBoundaryControllability(
            gap_index=index,
            rear_vehicle_id=str(rear_id),
            front_vehicle_id=str(front_id),
            rear_controllable=effective_controllable(
                str(rear_id),
                state,
                blocked_vehicle_ids,
            ),
            front_controllable=effective_controllable(
                str(front_id),
                state,
                blocked_vehicle_ids,
            ),
        )
        for index, gap in enumerate(kernel_ordered_gaps)
        for rear_id, front_id in ((gap.rear_vehicle_id, gap.front_vehicle_id),)
    )
    return ScenarioConfig(
        x_targets=x_targets,
        x_m0=0.0,
        v_ref=20.0,
        v_max=30.0,
        v_min=0.0,
        a_max=3.0,
        a_min=-4.0,
        T=20.0,
        gap_boundary_controllability=gap_boundary_controllability,
    )


def build_stage2_algorithm_config() -> AlgorithmConfig:
    reference = get_reference_algorithm_config()
    return AlgorithmConfig(
        D_h=reference.D_h,
        l_m=reference.l_m,
        w_c=reference.w_c,
        w_e=reference.w_e,
        w_t=reference.w_t,
        delta_ref=reference.delta_ref,
        q=reference.q,
        epsilon_delta=reference.epsilon_delta,
        K=reference.K,
        boundary_adjustment=reference.boundary_adjustment,
    )


def evaluate_stage2_one_step(state: SimulationState, mv_id: str) -> Stage2OneStepEvaluation:
    runtime = state.ramp_merge_runtime
    danger_vehicle_ids = (
        tuple(runtime.danger_vehicle_ids)
        if runtime is not None and hasattr(runtime, "danger_vehicle_ids")
        else ()
    )
    gap_snapshot = identify_and_number_gaps(state, danger_vehicle_ids)
    return evaluate_stage2_one_step_from_snapshot(state, mv_id, gap_snapshot)


def evaluate_stage2_one_step_from_snapshot(
    state: SimulationState,
    mv_id: str,
    gap_snapshot: GapSnapshot,
    uncontrollable_vehicle_ids: tuple[str, ...] | set[str] | frozenset[str] = (),
    allowed_gap_indices: tuple[int, ...] | set[int] | frozenset[int] | None = None,
) -> Stage2OneStepEvaluation:
    included_runtime_indices = _included_runtime_gap_indices(
        gap_snapshot,
        allowed_gap_indices,
    )
    local_frame = _build_stage2_local_frame_for_runtime_gap_indices(
        state,
        mv_id,
        gap_snapshot,
        included_runtime_indices,
    )
    local_scenario = build_stage2_local_scenario_from_snapshot(
        state,
        mv_id,
        gap_snapshot,
        uncontrollable_vehicle_ids=uncontrollable_vehicle_ids,
        allowed_gap_indices=allowed_gap_indices,
    )
    algorithm = build_stage2_algorithm_config()
    evaluation = evaluate_one_step_scenario(local_scenario, algorithm)
    return Stage2OneStepEvaluation(
        origin_x_global=float(local_frame["origin_x_global"]),
        local_frame=local_frame,
        local_scenario=local_scenario,
        algorithm=algorithm,
        evaluation=evaluation,
    )


def resolve_best_gap_vehicle_ids(
    local_frame: dict[str, object],
    evaluation: OneStepEvaluationResult,
) -> tuple[str, str]:
    if evaluation.best_gap is None:
        raise ValueError("best gap is required to resolve vehicle ids")
    gap_vehicle_ids_by_index = dict(local_frame["gap_vehicle_ids_by_index"])
    try:
        rear_vehicle_id, front_vehicle_id = gap_vehicle_ids_by_index[
            evaluation.best_gap.index
        ]
    except KeyError as exc:
        raise ValueError(
            f"failed to resolve lane-2 vehicle ids for gap index={evaluation.best_gap.index}"
        ) from exc
    return str(rear_vehicle_id), str(front_vehicle_id)


def to_global_merge_point(origin_x_global: float, p_m_local: float) -> float:
    return float(origin_x_global) + float(p_m_local)


def _lane_2_vehicle_order_from_snapshot(gap_snapshot: GapSnapshot) -> tuple[str, ...]:
    if not gap_snapshot.gaps:
        return ()
    vehicle_ids = [gap_snapshot.gaps[0].front_vehicle_id]
    vehicle_ids.extend(gap.rear_vehicle_id for gap in gap_snapshot.gaps)
    return tuple(vehicle_ids)


def _lane_2_vehicle_order_for_runtime_gap_indices(
    gap_snapshot: GapSnapshot,
    runtime_gap_indices: tuple[int, ...],
) -> tuple[str, ...]:
    runtime_set = set(runtime_gap_indices)
    included_gaps = [gap for gap in gap_snapshot.gaps if gap.index in runtime_set]
    if not included_gaps:
        return ()
    vehicle_ids: list[str] = []
    for gap in included_gaps:
        if not vehicle_ids:
            vehicle_ids.append(gap.front_vehicle_id)
        elif vehicle_ids[-1] != gap.front_vehicle_id:
            vehicle_ids.append(gap.front_vehicle_id)
        vehicle_ids.append(gap.rear_vehicle_id)
    return tuple(reversed(vehicle_ids))


def _included_runtime_gap_indices(
    gap_snapshot: GapSnapshot,
    allowed_gap_indices: tuple[int, ...] | set[int] | frozenset[int] | None,
) -> tuple[int, ...]:
    if allowed_gap_indices is None:
        return tuple(gap.index for gap in gap_snapshot.gaps)
    allowed = set(allowed_gap_indices)
    return tuple(gap.index for gap in gap_snapshot.gaps if gap.index in allowed)
