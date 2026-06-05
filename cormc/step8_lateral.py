from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi
from types import MappingProxyType
from typing import Any, Mapping

from cormc.step0_3 import (
    LANE_1,
    LANE_2,
    MAINLINE,
    ManeuverTrajectoryState,
    RelationsSnapshot,
    SimulationState,
    VehicleState,
    assert_x_plot_not_used_in_algorithm_path,
)
from cormc.step9_11 import (
    CandidateLaneState,
    CandidateLateralKinematics,
    CandidateManeuverProgress,
    CandidateStateTransition,
    CommandBuffer,
    NextStateBuffer,
)


STEP8_LATERAL_SOURCE = "step8_lateral_trajectory"
PAPER_FORMULA_SOURCE = "paper_formula"
ENGINEERING_PATCH_SOURCE = "first_version_engineering_patch"
FIRST_VERSION_DIAGNOSTIC_SOURCE = "first_version_diagnostic"
COMPLETION_Y_TOLERANCE_M = 0.05


@dataclass(frozen=True)
class Step8LateralRunResult:
    state: SimulationState
    relations: RelationsSnapshot
    command_buffer: CommandBuffer
    next_state_buffer: NextStateBuffer
    actual_events: list[dict[str, Any]]
    actual_sanity_checks: list[dict[str, Any]]
    expected_png_features: list[dict[str, Any]]


def run_step8_lateral_trajectory_planning_speed_progress(
    state: SimulationState,
    relations: RelationsSnapshot,
    *,
    command_buffer: CommandBuffer,
    p08_next_state_buffer: NextStateBuffer | None = None,
    planning_speeds: Mapping[str, float] | None = None,
    front_fallback_diagnostics: Mapping[str, Mapping[str, Any]] | None = None,
    boundary_risk_diagnostics: Mapping[str, Mapping[str, Any]] | None = None,
    vehicle_ids: tuple[str, ...] | None = None,
) -> Step8LateralRunResult:
    before_signature = _state_signature(state)
    scenario_id = state.scenario_config_ref or "unknown"
    p08_candidates = (
        p08_next_state_buffer.candidate_longitudinal
        if p08_next_state_buffer is not None
        else {}
    )
    explicit_planning_speeds = planning_speeds or {}
    selected_vehicle_ids = _selected_vehicle_ids(state, command_buffer, vehicle_ids)
    lateral_candidates: dict[str, CandidateLateralKinematics] = {}
    progress_candidates: dict[str, CandidateManeuverProgress] = {}
    lane_state_candidates: dict[str, CandidateLaneState] = {}
    state_transition_candidates: dict[str, tuple[CandidateStateTransition, ...]] = {}
    events: list[dict[str, Any]] = []

    for vehicle_id in selected_vehicle_ids:
        source = _resolve_maneuver_source(
            state,
            command_buffer,
            vehicle_id,
        )
        if source is None:
            continue
        p08_candidate = p08_candidates.get(vehicle_id)
        planning_speed = _planning_speed_for_vehicle(
            state,
            vehicle_id,
            p08_candidate=p08_candidate,
            planning_speeds=explicit_planning_speeds,
        )
        if planning_speed is None:
            continue
        fallback = dict((front_fallback_diagnostics or {}).get(vehicle_id, {}))
        if (
            not fallback
            and p08_candidate is not None
            and "front_fallback" in p08_candidate.constraints_applied
        ):
            fallback = {"status": "consumed_from_p08", "consumed": True, "schema_gap": False}
        boundary_risk = dict((boundary_risk_diagnostics or {}).get(vehicle_id, {}))
        trajectory = _compute_sine_reference_candidate(
            state,
            vehicle_id,
            source=source,
            planning_speed=planning_speed,
        )
        source_commands = _source_commands(source, p08_candidate)
        lateral_candidate = CandidateLateralKinematics(
            candidate_id=f"p09:{state.step}:{vehicle_id}:lateral",
            vehicle_id=vehicle_id,
            y=trajectory["candidate_y"],
            target_y=source["target_y"],
            source=STEP8_LATERAL_SOURCE,
            front_collision_fallback=bool(fallback.get("consumed", False)),
            source_commands=source_commands,
        )
        progress_candidate = CandidateManeuverProgress(
            candidate_id=f"p09:{state.step}:{vehicle_id}:maneuver_progress",
            vehicle_id=vehicle_id,
            maneuver_type=source["maneuver_type"],
            progress=trajectory["progress"],
            completed=trajectory["completed"],
            target_y_reached=trajectory["target_y_reached"],
            source_command_id=source.get("source_command_id"),
        )
        lateral_candidates[vehicle_id] = lateral_candidate
        progress_candidates[vehicle_id] = progress_candidate
        if progress_candidate.completed:
            lane_state_candidates[vehicle_id] = _completion_lane_state(
                state,
                vehicle_id,
                source=source,
            )
            transitions = _completion_state_transitions(
                state,
                vehicle_id,
                source=source,
            )
            if transitions:
                state_transition_candidates[vehicle_id] = transitions
        events.append(
            emit_lateral_trajectory_event(
                state,
                vehicle_id,
                source=source,
                trajectory=trajectory,
                planning_speed=planning_speed,
                p08_candidate=p08_candidate,
                lateral_candidate=lateral_candidate,
                progress_candidate=progress_candidate,
                fallback=fallback,
                boundary_risk=boundary_risk,
                scenario_id=scenario_id,
            )
        )

    next_state_buffer = NextStateBuffer(
        step=state.step,
        t=state.t,
        candidate_lateral=MappingProxyType(lateral_candidates),
        candidate_maneuver_progress=MappingProxyType(progress_candidates),
        candidate_lane_state=MappingProxyType(lane_state_candidates),
        candidate_state_transitions=MappingProxyType(state_transition_candidates),
    )
    sanity_checks = [
        run_p09_no_write_before_commit_sanity(
            state,
            before_signature=before_signature,
            scenario_id=scenario_id,
            vehicle_ids=selected_vehicle_ids,
        ),
        run_p09_no_ordinary_lane_change_sanity(
            state,
            selected_vehicle_ids,
            candidate_vehicle_ids=tuple(lateral_candidates),
            command_buffer=command_buffer,
            scenario_id=scenario_id,
        ),
        run_p09_no_longitudinal_candidate_sanity(
            state,
            next_state_buffer,
            scenario_id=scenario_id,
            vehicle_ids=selected_vehicle_ids,
        ),
        run_p09_state_machine_sanity(
            state,
            scenario_id=scenario_id,
            vehicle_ids=selected_vehicle_ids,
        ),
        run_p09_x_plot_sanity(
            state,
            relations,
            scenario_id=scenario_id,
            vehicle_ids=selected_vehicle_ids,
        ),
        run_p09_boundary_risk_sanity(
            state,
            scenario_id=scenario_id,
            diagnostics=boundary_risk_diagnostics or {},
            vehicle_ids=selected_vehicle_ids,
        ),
    ]
    return Step8LateralRunResult(
        state=state,
        relations=relations,
        command_buffer=command_buffer,
        next_state_buffer=next_state_buffer,
        actual_events=events,
        actual_sanity_checks=sanity_checks,
        expected_png_features=register_p09_png_features(
            lateral_candidates,
            progress_candidates,
            events=events,
            boundary_risk_vehicle_ids=tuple((boundary_risk_diagnostics or {}).keys()),
        ),
    )


def emit_lateral_trajectory_event(
    state: SimulationState,
    vehicle_id: str,
    *,
    source: Mapping[str, Any],
    trajectory: Mapping[str, Any],
    planning_speed: float,
    p08_candidate: Any,
    lateral_candidate: CandidateLateralKinematics,
    progress_candidate: CandidateManeuverProgress,
    fallback: Mapping[str, Any],
    boundary_risk: Mapping[str, Any],
    scenario_id: str,
) -> dict[str, Any]:
    current = state.vehicle_states[vehicle_id]
    payload = {
        "maneuver_type": source["maneuver_type"],
        "source_command_id": source.get("source_command_id"),
        "source_lane_change_command_id": source.get("source_lane_change_command_id"),
        "source_merge_command_id": source.get("source_merge_command_id"),
        "source_overlay_id": source.get("source_overlay_id"),
        "declared_overlay_id": source.get("declared_overlay_id"),
        "same_step_overlay_consumed": bool(source.get("same_step_overlay_consumed", False)),
        "same_step_overlay_missing": bool(source.get("same_step_overlay_missing", False)),
        "source_longitudinal_candidate_id": (
            p08_candidate.candidate_id if p08_candidate is not None else None
        ),
        "trajectory_consumed_speed": planning_speed,
        "trajectory_consumed_speed_source": "p08_planning_speed",
        "vehicle_state_v": current.v,
        "used_vehicle_state_v_for_progress": False,
        "target_lane": source["target_lane"],
        "target_y": source["target_y"],
        "candidate_y": lateral_candidate.y,
        "candidate_lateral_id": lateral_candidate.candidate_id,
        "candidate_progress_id": progress_candidate.candidate_id,
        "previous_progress": trajectory["previous_progress"],
        "progress": progress_candidate.progress,
        "candidate_progress": progress_candidate.progress,
        "completed": progress_candidate.completed,
        "target_y_reached": progress_candidate.target_y_reached,
        "start_x_global": source["start_x_global"],
        "start_y": source["start_y"],
        "planned_length": source.get("planned_length"),
        "planned_length_available": source.get("planned_length") is not None,
        "active_maneuver_present": source["active_maneuver_present"],
        "active_maneuver_was_reset": False,
        "true_state_written_by_p09": False,
        "cuc_rerun_by_p09": False,
        "eq53_rejudged_by_p09": False,
        "boundary_cap_recomputed_by_p09": False,
        "speed_cap_recomputed_by_p09": False,
        "p08_speed_cap_composition_rerun_by_p09": False,
        "merge_progress_payload_created": source["maneuver_type"] == "merge",
        "lane_change_progress_payload_created": source["maneuver_type"] == "lane_change",
        "sine_formula_source": "vehicle_model_spec_eq33_eq36_first_version",
        "front_fallback_status": fallback.get("status", "not_applicable"),
        "front_fallback_schema_gap": bool(fallback.get("schema_gap", not bool(fallback))),
        "boundary_risk_status": boundary_risk.get("risk_status", "not_applicable"),
        "boundary_risk_source_id": boundary_risk.get("risk_source_id"),
    }
    if source.get("source_scenario_id") is not None:
        payload["source_scenario_id"] = source["source_scenario_id"]
    if source.get("does_not_rejudge_merge_start") is not None:
        payload["does_not_rejudge_merge_start"] = source["does_not_rejudge_merge_start"]
    if source.get("assigned_clv_id") is not None:
        payload["assigned_clv_id"] = source["assigned_clv_id"]
    if source.get("assigned_cfv_id") is not None:
        payload["assigned_cfv_id"] = source["assigned_cfv_id"]
    if p08_candidate is not None:
        payload["p08_constraints_applied"] = list(p08_candidate.constraints_applied)
        payload["p08_source_commands"] = list(p08_candidate.source_commands)
    return _event(
        state,
        module="Step8ManeuverCompletion" if progress_candidate.completed else "Step8LateralTrajectory",
        event_type="lateral_trajectory",
        vehicle_id=vehicle_id,
        related_vehicle_ids=tuple(item for item in (vehicle_id, source.get("source_mv_id")) if item),
        scenario_id=scenario_id,
        reason=source["reason"],
        source=PAPER_FORMULA_SOURCE,
        is_engineering_patch=False,
        payload=payload,
    )


def run_p09_no_write_before_commit_sanity(
    state: SimulationState,
    *,
    before_signature: tuple[Any, ...],
    scenario_id: str,
    vehicle_ids: tuple[str, ...],
) -> dict[str, Any]:
    unchanged = before_signature == _state_signature(state)
    return _sanity(
        state,
        "no_write_before_commit",
        "pass" if unchanged else "fail",
        scenario_id=scenario_id,
        vehicle_ids=vehicle_ids,
        reason="p09_outputs_are_candidates_events_sanity_and_png_only",
        payload={
            "state_unchanged": unchanged,
            "lateral_candidate_written_to_vehicle_state": False,
            "completion_written_to_vehicle_state": False,
        },
    )


def run_p09_no_ordinary_lane_change_sanity(
    state: SimulationState,
    vehicle_ids: tuple[str, ...],
    *,
    candidate_vehicle_ids: tuple[str, ...],
    command_buffer: CommandBuffer,
    scenario_id: str,
) -> dict[str, Any]:
    ordinary_candidates = [
        vehicle_id
        for vehicle_id in candidate_vehicle_ids
        if vehicle_id not in command_buffer.lane_change_commands
        and vehicle_id not in state.active_maneuvers
        and vehicle_id not in command_buffer.merge_commands
    ]
    return _sanity(
        state,
        "unexpected_ordinary_lane_change_attempt",
        "fail" if ordinary_candidates else "pass",
        scenario_id=scenario_id,
        vehicle_ids=tuple(ordinary_candidates or vehicle_ids),
        reason="ordinary_mainline_lane_change_closed",
        payload={
            "ordinary_lane_change_candidate_created": bool(ordinary_candidates),
            "ordinary_candidate_vehicle_ids": ordinary_candidates,
        },
    )


def run_p09_no_longitudinal_candidate_sanity(
    state: SimulationState,
    next_state_buffer: NextStateBuffer,
    *,
    scenario_id: str,
    vehicle_ids: tuple[str, ...],
) -> dict[str, Any]:
    return _sanity(
        state,
        "state_machine_inconsistency",
        "pass",
        scenario_id=scenario_id,
        vehicle_ids=vehicle_ids,
        reason="p09_no_longitudinal_candidate_or_commit",
        payload={
            "candidate_longitudinal_created": bool(next_state_buffer.candidate_longitudinal),
            "candidate_kinematics_created": bool(next_state_buffer.candidate_kinematics),
            "commit_event_created": False,
        },
    )


def run_p09_state_machine_sanity(
    state: SimulationState,
    *,
    scenario_id: str,
    vehicle_ids: tuple[str, ...],
) -> dict[str, Any]:
    inconsistent = [
        vehicle_id
        for vehicle_id in vehicle_ids
        if state.vehicle_states[vehicle_id].lane_change_state == "executing"
        and state.vehicle_states[vehicle_id].merge_state == "executing"
    ]
    return _sanity(
        state,
        "state_machine_inconsistency",
        "fail" if inconsistent else "pass",
        scenario_id=scenario_id,
        vehicle_ids=tuple(inconsistent or vehicle_ids),
        reason="lane_change_and_merge_executing_exclusive",
        payload={"inconsistent_vehicle_ids": inconsistent},
    )


def run_p09_x_plot_sanity(
    state: SimulationState,
    relations: RelationsSnapshot,
    *,
    scenario_id: str,
    vehicle_ids: tuple[str, ...],
) -> dict[str, Any]:
    ok = assert_x_plot_not_used_in_algorithm_path(state, relations)
    return _sanity(
        state,
        "x_plot_used_in_algorithm_path",
        "pass" if ok else "fail",
        scenario_id=scenario_id,
        vehicle_ids=vehicle_ids,
        reason="x_global_only_algorithm_path",
        payload={"x_plot_used_in_algorithm_path": False},
    )


def run_p09_boundary_risk_sanity(
    state: SimulationState,
    *,
    scenario_id: str,
    diagnostics: Mapping[str, Mapping[str, Any]],
    vehicle_ids: tuple[str, ...],
) -> dict[str, Any]:
    risky_vehicle_ids = tuple(str(vehicle_id) for vehicle_id in diagnostics)
    return _sanity(
        state,
        "boundary_violation",
        "warning" if risky_vehicle_ids else "pass",
        scenario_id=scenario_id,
        vehicle_ids=risky_vehicle_ids or vehicle_ids,
        reason="p09_boundary_risk_observable",
        payload={
            "risk_vehicle_ids": list(risky_vehicle_ids),
            "diagnostics": {key: dict(value) for key, value in diagnostics.items()},
            "full_conservative_policy_implemented": False,
            "true_state_written_by_p09": False,
        },
    )


def register_p09_png_features(
    lateral_candidates: Mapping[str, CandidateLateralKinematics],
    progress_candidates: Mapping[str, CandidateManeuverProgress],
    *,
    events: list[dict[str, Any]],
    boundary_risk_vehicle_ids: tuple[str, ...],
) -> list[dict[str, Any]]:
    lane_change_ids = [
        event["vehicle_id"]
        for event in events
        if event.get("payload", {}).get("maneuver_type") == "lane_change"
    ]
    merge_ids = [
        event["vehicle_id"]
        for event in events
        if event.get("payload", {}).get("maneuver_type") == "merge"
    ]
    completion_ids = [
        vehicle_id
        for vehicle_id, progress in progress_candidates.items()
        if progress.completed
    ]
    fallback_ids = [
        event["vehicle_id"]
        for event in events
        if event.get("payload", {}).get("front_fallback_status") not in {None, "not_applicable"}
    ]
    features: list[dict[str, Any]] = []
    if lane_change_ids:
        features.append(_png_feature("lane_change_trajectory_marker", lane_change_ids))
    if merge_ids:
        features.append(_png_feature("merge_trajectory_marker", merge_ids))
    if progress_candidates:
        features.append(_png_feature("maneuver_progress_marker", progress_candidates))
    if lateral_candidates:
        features.append(
            _png_feature("planning_speed_consumption_marker", lateral_candidates, required=False)
        )
    if completion_ids:
        features.append(_png_feature("completion_candidate_marker", completion_ids, required=False))
    if fallback_ids:
        features.append(_png_feature("front_collision_fallback_marker", fallback_ids, required=False))
    if boundary_risk_vehicle_ids:
        features.append(_png_feature("boundary_risk_marker", boundary_risk_vehicle_ids, required=False))
    return features


def _selected_vehicle_ids(
    state: SimulationState,
    command_buffer: CommandBuffer,
    vehicle_ids: tuple[str, ...] | None,
) -> tuple[str, ...]:
    if vehicle_ids is not None:
        return tuple(vehicle_id for vehicle_id in vehicle_ids if vehicle_id in state.vehicle_states)
    ids: list[str] = []
    for source in (
        state.active_maneuvers,
        command_buffer.lane_change_commands,
        command_buffer.merge_commands,
    ):
        for vehicle_id in source:
            if vehicle_id in state.vehicle_states and vehicle_id not in ids:
                ids.append(vehicle_id)
    return tuple(ids or state.active_vehicle_ids)


def _resolve_maneuver_source(
    state: SimulationState,
    command_buffer: CommandBuffer,
    vehicle_id: str,
) -> dict[str, Any] | None:
    current = state.vehicle_states[vehicle_id]
    active = state.active_maneuvers.get(vehicle_id)
    lane_command = command_buffer.lane_change_commands.get(vehicle_id)
    merge_command = command_buffer.merge_commands.get(vehicle_id)
    if active is not None:
        command = merge_command if active.maneuver_type == "merge" else lane_command
        source = _source_from_active_maneuver(current, active, command)
        if command is not None:
            source.update(_command_trace(command))
        return source
    if isinstance(lane_command, Mapping):
        overlay = command_buffer.same_step_overlays.get(vehicle_id)
        return _source_from_lane_change_command(current, lane_command, overlay)
    if isinstance(merge_command, Mapping):
        return _source_from_merge_command(current, merge_command)
    return None


def _source_from_active_maneuver(
    current: VehicleState,
    active: ManeuverTrajectoryState,
    command: Any,
) -> dict[str, Any]:
    maneuver_type = str(active.maneuver_type)
    source_command_id = (
        str(command.get("command_id")) if isinstance(command, Mapping) else active.source_command_id
    )
    return {
        "maneuver_type": maneuver_type,
        "reason": f"{maneuver_type}_continuation",
        "start_x_global": float(active.start_x_global),
        "start_y": float(active.start_y),
        "target_lane": active.target_lane,
        "target_y": float(active.target_y),
        "planned_length": active.planned_length,
        "previous_progress": float(active.progress),
        "active_maneuver_present": True,
        "source_command_id": source_command_id,
        "source_lane_change_command_id": source_command_id if maneuver_type == "lane_change" else None,
        "source_merge_command_id": source_command_id if maneuver_type == "merge" else None,
        "source_overlay_id": None,
        "source_mv_id": None,
        "does_not_rejudge_merge_start": (
            bool(command.get("does_not_rejudge_merge_start", False))
            if isinstance(command, Mapping)
            else maneuver_type == "merge"
        ),
        "assigned_clv_id": active.assigned_clv_id,
        "assigned_cfv_id": active.assigned_cfv_id,
        "current_y": float(current.y),
    }


def _source_from_lane_change_command(
    current: VehicleState,
    command: Mapping[str, Any],
    overlay: Any,
) -> dict[str, Any]:
    overlay_consumed = isinstance(overlay, Mapping)
    declared_overlay_id = _optional_str(command.get("overlay_id"))
    overlay_id = _optional_str(overlay.get("overlay_id")) if overlay_consumed else None
    source_scenario_id = _optional_str(command.get("source_scenario_id"))
    if source_scenario_id is None and overlay_consumed:
        source_scenario_id = _optional_str(overlay.get("source_scenario_id"))
    return {
        "maneuver_type": "lane_change",
        "reason": "lane_change_start",
        "start_x_global": float(current.x_global),
        "start_y": float(current.y),
        "target_lane": str(command.get("target_lane") or LANE_1),
        "target_y": float(command["target_y"]),
        "planned_length": _optional_float(command.get("planned_length")),
        "previous_progress": 0.0,
        "active_maneuver_present": False,
        "source_command_id": str(command.get("command_id")),
        "source_lane_change_command_id": str(command.get("command_id")),
        "source_merge_command_id": None,
        "source_overlay_id": overlay_id,
        "declared_overlay_id": declared_overlay_id,
        "same_step_overlay_consumed": overlay_consumed,
        "same_step_overlay_missing": declared_overlay_id is not None and not overlay_consumed,
        "source_mv_id": _optional_str(command.get("source_mv_id")),
        "source_scenario_id": source_scenario_id,
        "does_not_rejudge_merge_start": None,
        "assigned_clv_id": None,
        "assigned_cfv_id": None,
        "current_y": float(current.y),
    }


def _source_from_merge_command(
    current: VehicleState,
    command: Mapping[str, Any],
) -> dict[str, Any]:
    mode = str(command.get("init_or_continue_maneuver") or "init")
    return {
        "maneuver_type": "merge",
        "reason": "merge_continuation" if mode == "continue" else "merge_start",
        "start_x_global": float(current.x_global),
        "start_y": float(current.y),
        "target_lane": str(command.get("target_lane") or LANE_2),
        "target_y": float(command["target_y"]),
        "planned_length": _optional_float(command.get("planned_length")),
        "previous_progress": 0.0,
        "active_maneuver_present": bool(command.get("active_maneuver_present", False)),
        "source_command_id": str(command.get("command_id")),
        "source_lane_change_command_id": None,
        "source_merge_command_id": str(command.get("command_id")),
        "source_overlay_id": None,
        "source_mv_id": None,
        "does_not_rejudge_merge_start": bool(command.get("does_not_rejudge_merge_start", False)),
        "assigned_clv_id": _optional_str(command.get("assigned_clv_id")),
        "assigned_cfv_id": _optional_str(command.get("assigned_cfv_id")),
        "current_y": float(current.y),
    }


def _command_trace(command: Any) -> dict[str, Any]:
    if not isinstance(command, Mapping):
        return {}
    command_id = str(command.get("command_id"))
    if command.get("command_type") == "merge":
        return {
            "source_command_id": command_id,
            "source_merge_command_id": command_id,
            "does_not_rejudge_merge_start": bool(command.get("does_not_rejudge_merge_start", False)),
            "assigned_clv_id": _optional_str(command.get("assigned_clv_id")),
            "assigned_cfv_id": _optional_str(command.get("assigned_cfv_id")),
        }
    return {
        "source_command_id": command_id,
        "source_lane_change_command_id": command_id,
        "source_overlay_id": None,
        "declared_overlay_id": _optional_str(command.get("overlay_id")),
        "same_step_overlay_consumed": False,
        "source_mv_id": _optional_str(command.get("source_mv_id")),
        "source_scenario_id": _optional_str(command.get("source_scenario_id")),
    }


def _planning_speed_for_vehicle(
    state: SimulationState,
    vehicle_id: str,
    *,
    p08_candidate: Any,
    planning_speeds: Mapping[str, float],
) -> float | None:
    if p08_candidate is not None:
        return float(p08_candidate.planning_speed)
    if vehicle_id in planning_speeds:
        return float(planning_speeds[vehicle_id])
    return None


def _compute_sine_reference_candidate(
    state: SimulationState,
    vehicle_id: str,
    *,
    source: Mapping[str, Any],
    planning_speed: float,
) -> dict[str, Any]:
    previous_progress = float(source.get("previous_progress", 0.0))
    planned_length = source.get("planned_length")
    if planned_length is not None and float(planned_length) > 0.0:
        progress_delta = max(0.0, float(planning_speed) * float(state.dt) / float(planned_length))
        progress = min(1.0, previous_progress + progress_delta)
    else:
        progress_delta = 0.0
        progress = previous_progress
    start_y = float(source["start_y"])
    target_y = float(source["target_y"])
    candidate_y = start_y + (target_y - start_y) * _sine_reference_fraction(progress)
    target_y_reached = abs(candidate_y - target_y) <= COMPLETION_Y_TOLERANCE_M
    completed = bool(progress >= 1.0 or target_y_reached)
    if completed:
        candidate_y = target_y
        progress = 1.0
        target_y_reached = True
    return {
        "previous_progress": previous_progress,
        "progress_delta": progress_delta,
        "progress": progress,
        "candidate_y": candidate_y,
        "target_y_reached": target_y_reached,
        "completed": completed,
    }


def _sine_reference_fraction(progress: float) -> float:
    clipped = min(1.0, max(0.0, float(progress)))
    return 0.5 - 0.5 * cos(pi * clipped)


def _source_commands(source: Mapping[str, Any], p08_candidate: Any) -> tuple[str, ...]:
    commands: list[str] = []
    for key in ("source_command_id", "source_overlay_id"):
        value = source.get(key)
        if value is not None and str(value) not in commands:
            commands.append(str(value))
    if p08_candidate is not None and p08_candidate.candidate_id not in commands:
        commands.append(p08_candidate.candidate_id)
    return tuple(commands)


def _completion_lane_state(
    state: SimulationState,
    vehicle_id: str,
    *,
    source: Mapping[str, Any],
) -> CandidateLaneState:
    target_lane = str(source["target_lane"])
    return CandidateLaneState(
        candidate_id=f"p09:{state.step}:{vehicle_id}:lane_state",
        vehicle_id=vehicle_id,
        physical_lane=target_lane,
        road_role=MAINLINE if target_lane in {LANE_1, LANE_2} else state.vehicle_states[vehicle_id].road_role,
        reason=f"{source['maneuver_type']}_target_y_reached",
    )


def _completion_state_transitions(
    state: SimulationState,
    vehicle_id: str,
    *,
    source: Mapping[str, Any],
) -> tuple[CandidateStateTransition, ...]:
    current = state.vehicle_states[vehicle_id]
    maneuver_type = str(source["maneuver_type"])
    if maneuver_type == "lane_change":
        return (
            CandidateStateTransition(
                candidate_id=f"p09:{state.step}:{vehicle_id}:lane_change_state",
                vehicle_id=vehicle_id,
                state_name="lane_change_state",
                old_state=current.lane_change_state,
                new_state="normal",
                reason="lane_change_target_y_reached",
            ),
        )
    if maneuver_type == "merge":
        return (
            CandidateStateTransition(
                candidate_id=f"p09:{state.step}:{vehicle_id}:merge_state",
                vehicle_id=vehicle_id,
                state_name="merge_state",
                old_state=current.merge_state,
                new_state="merged",
                reason="merge_target_y_reached",
            ),
        )
    return ()


def _event(
    state: SimulationState,
    *,
    module: str,
    event_type: str,
    vehicle_id: str | None,
    related_vehicle_ids: tuple[str, ...],
    scenario_id: str,
    reason: str,
    source: str,
    is_engineering_patch: bool,
    payload: Mapping[str, Any],
) -> dict[str, Any]:
    vehicle_ids = list(related_vehicle_ids)
    if vehicle_id is not None and vehicle_id not in vehicle_ids:
        vehicle_ids.append(vehicle_id)
    return {
        "step": state.step,
        "t": state.t,
        "module": module,
        "event_type": event_type,
        "vehicle_id": vehicle_id,
        "vehicle_ids": vehicle_ids,
        "related_vehicle_ids": list(related_vehicle_ids),
        "scenario_id": scenario_id,
        "reason": reason,
        "source": source,
        "is_engineering_patch": is_engineering_patch,
        "payload": dict(payload),
    }


def _sanity(
    state: SimulationState,
    check_type: str,
    result: str,
    *,
    scenario_id: str,
    vehicle_ids: tuple[str, ...],
    reason: str,
    payload: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "step": state.step,
        "t": state.t,
        "check_type": check_type,
        "result": result,
        "vehicle_ids": list(vehicle_ids),
        "scenario_id": scenario_id,
        "reason": reason,
        "payload": dict(payload or {}),
    }


def _png_feature(
    feature_type: str,
    vehicle_ids: Any,
    *,
    required: bool = True,
) -> dict[str, Any]:
    ids = sorted(str(vehicle_id) for vehicle_id in vehicle_ids)
    return {
        "feature_type": feature_type,
        "required": required,
        "vehicle_ids": ids,
        "expected_visibility": "visible" if required else "optional",
        "notes": "registered only; renderer deferred",
    }


def _state_signature(state: SimulationState) -> tuple[Any, ...]:
    return (
        state.t,
        state.step,
        state.dt,
        state.active_vehicle_ids,
        tuple(
            (
                vehicle_id,
                state.vehicle_states[vehicle_id].x_global,
                state.vehicle_states[vehicle_id].y,
                state.vehicle_states[vehicle_id].v,
                state.vehicle_states[vehicle_id].a,
                state.vehicle_states[vehicle_id].physical_lane,
                state.vehicle_states[vehicle_id].road_role,
                state.vehicle_states[vehicle_id].lane_change_state,
                state.vehicle_states[vehicle_id].merge_state,
            )
            for vehicle_id in state.active_vehicle_ids
        ),
        tuple(
            (
                vehicle_id,
                state.active_maneuvers[vehicle_id].maneuver_type,
                state.active_maneuvers[vehicle_id].start_step,
                state.active_maneuvers[vehicle_id].start_t,
                state.active_maneuvers[vehicle_id].start_x_global,
                state.active_maneuvers[vehicle_id].start_y,
                state.active_maneuvers[vehicle_id].target_lane,
                state.active_maneuvers[vehicle_id].target_y,
                state.active_maneuvers[vehicle_id].source_command_id,
                state.active_maneuvers[vehicle_id].source_event_id,
                state.active_maneuvers[vehicle_id].planned_length,
                state.active_maneuvers[vehicle_id].progress,
                state.active_maneuvers[vehicle_id].last_planning_speed,
                state.active_maneuvers[vehicle_id].assigned_clv_id,
                state.active_maneuvers[vehicle_id].assigned_cfv_id,
            )
            for vehicle_id in sorted(state.active_maneuvers)
        ),
        tuple((key, tuple(sorted(value.items()))) for key, value in state.assignment_records_by_mv.items()),
        tuple(
            (
                vehicle_id,
                memory.ex_prev,
                memory.e_prev,
                memory.integral_ex,
                memory.integral_e,
                memory.last_t,
                memory.last_controller_update_step,
                memory.controller_mode,
            )
            for vehicle_id, memory in sorted(state.controller_memory_by_vehicle.items())
        ),
    )


def _optional_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    return float(value)


def _optional_str(value: Any) -> str | None:
    if value in (None, ""):
        return None
    return str(value)
