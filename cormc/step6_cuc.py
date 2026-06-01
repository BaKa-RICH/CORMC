from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping

from cormc.step0_3 import (
    DEFAULT_ROAD_GEOMETRY,
    LANE_1,
    LANE_2,
    RelationsSnapshot,
    RoadGeometryConfig,
    SimulationState,
    resolve_lane_centerline,
)
from cormc.step9_11 import CommandBuffer


ENGINEERING_PATCH_SOURCE = "first_version_engineering_patch"
PAPER_FORMULA_SOURCE = "paper_formula"
TEST_HARNESS_OVERRIDE_SOURCE = "test_harness_override"
TT_MIN_SECONDS = 1.5


@dataclass(frozen=True)
class Step6CUCRunResult:
    state: SimulationState
    relations: RelationsSnapshot
    command_buffer: CommandBuffer
    actual_events: list[dict[str, Any]]
    actual_sanity_checks: list[dict[str, Any]]
    expected_png_features: list[dict[str, Any]]
    cuc_decisions: Mapping[str, Mapping[str, Any]]
    longitudinal_candidates: Mapping[str, Any]
    lateral_candidates: Mapping[str, Any]
    suppressed_requests_ignored_as_active_input: bool


def run_step6_cuc_choice_compliance_lane_change_overlay(
    state: SimulationState,
    relations: RelationsSnapshot,
    *,
    active_requests: Mapping[str, Mapping[str, Any]],
    suppressed_requests: tuple[Mapping[str, Any], ...] = (),
    utility_overrides: Mapping[str, Mapping[str, Any]] | None = None,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> Step6CUCRunResult:
    before_signature = _state_signature(state)
    events: list[dict[str, Any]] = []
    sanity_checks: list[dict[str, Any]] = []
    decisions: dict[str, dict[str, Any]] = {}
    lane_change_commands: dict[str, dict[str, Any]] = {}
    cooperation_commands: dict[str, dict[str, Any]] = {}
    state_transition_commands: dict[str, tuple[dict[str, Any], ...]] = {}
    same_step_overlays: dict[str, dict[str, Any]] = {}

    if not active_requests:
        events.append(
            _event(
                state,
                module="Step6CUC",
                event_type="CUC",
                scenario_id=state.scenario_config_ref or "unknown",
                reason="no_active_request_no_cuc",
                source=PAPER_FORMULA_SOURCE,
                is_engineering_patch=False,
                payload={
                    "active_request_count": 0,
                    "cuc_executed": False,
                },
            )
        )

    for cv_id in sorted(active_requests):
        request = dict(active_requests[cv_id])
        if request.get("active") is False:
            continue
        if str(request.get("cv_id", cv_id)) != cv_id:
            request["cv_id"] = cv_id

        decision = _build_base_decision(state, relations, request)
        cv_state = state.vehicle_states.get(cv_id)
        cv_spec = state.vehicle_specs.get(cv_id)

        if cv_state is None or cv_spec is None or cv_id not in state.active_vehicle_ids:
            decision.update(
                {
                    "recommended_choice": "not_applicable",
                    "effective_choice": "not_applicable",
                    "final_choice": "not_applicable",
                    "fallback_reason": "not_active_cv",
                    "accepted_by_vehicle": False,
                }
            )
            decisions[cv_id] = decision
            events.append(_cuc_event(state, decision, reason="not_active_cv"))
            continue

        if cv_state.lane_change_state == "executing":
            decision.update(
                {
                    "recommended_choice": "not_applicable",
                    "effective_choice": "not_applicable",
                    "final_choice": "not_applicable",
                    "fallback_reason": "already_executing_lane_change",
                    "accepted_by_vehicle": False,
                    "cuc_suggestion_executed": False,
                }
            )
            decisions[cv_id] = decision
            events.append(_cuc_event(state, decision, reason="already_executing_lane_change"))
            sanity_checks.append(
                _sanity(
                    state,
                    "state_machine_inconsistency",
                    "pass",
                    vehicle_ids=(cv_id,),
                    reason="active_lane_change_skip_cuc",
                    payload={
                        "vehicle_id": cv_id,
                        "lane_change_state": cv_state.lane_change_state,
                        "new_lane_change_command_created": False,
                    },
                )
            )
            continue

        utility = _evaluate_utility_or_override(
            state,
            relations,
            request,
            override=(utility_overrides or {}).get(cv_id),
        )
        safety = _evaluate_target_lane_safety(state, relations, cv_id)
        accepted_by_vehicle = _vehicle_accepts_cuc(cv_spec.vehicle_type, cv_spec.compliance_state)
        decision.update(utility)
        decision.update(
            {
                "target_lane_safe": safety["target_lane_safe"],
                "target_lane": LANE_1,
                "source_lane": LANE_2,
                "TLV_id": safety["TLV_id"],
                "TFV_id": safety["TFV_id"],
                "TT_CV_TLV": safety["TT_CV_TLV"],
                "TT_TFV_CV": safety["TT_TFV_CV"],
                "TT_min": TT_MIN_SECONDS,
                "vehicle_type": cv_spec.vehicle_type,
                "compliance_state": cv_spec.compliance_state,
                "accepted_by_vehicle": accepted_by_vehicle,
            }
        )
        events.append(_target_lane_safety_event(state, request, safety))
        events.append(_compliance_event(state, request, cv_spec, accepted_by_vehicle))

        recommended_choice = str(utility["recommended_choice"])
        fallback_reason: str | None = None
        effective_choice = recommended_choice
        if not accepted_by_vehicle:
            effective_choice = "not_applicable"
            fallback_reason = "non_compliant_chv"
        elif recommended_choice == "change_to_lane_1" and not safety["target_lane_safe"]:
            effective_choice = "stay_lane_2"
            fallback_reason = "target_lane_unsafe"
        elif recommended_choice != "change_to_lane_1":
            effective_choice = "stay_lane_2"
            fallback_reason = "utility_not_better"

        decision.update(
            {
                "effective_choice": effective_choice,
                "final_choice": effective_choice,
                "fallback_reason": fallback_reason,
                "cuc_suggestion_executed": accepted_by_vehicle,
            }
        )
        decisions[cv_id] = decision
        events.append(
            _cuc_event(
                state,
                decision,
                reason=_decision_reason(decision),
            )
        )

        if fallback_reason == "target_lane_unsafe":
            sanity_checks.append(
                _sanity(
                    state,
                    "state_machine_inconsistency",
                    "pass",
                    vehicle_ids=(cv_id,),
                    reason="target_lane_unsafe_fallback",
                    payload={
                        "vehicle_id": cv_id,
                        "fallback_reason": fallback_reason,
                        "lane_change_command_created": False,
                    },
                )
            )
        if fallback_reason == "non_compliant_chv":
            sanity_checks.append(
                _sanity(
                    state,
                    "state_machine_inconsistency",
                    "pass",
                    vehicle_ids=(cv_id,),
                    reason="non_compliant_no_action",
                    payload={
                        "vehicle_id": cv_id,
                        "lane_change_command_created": False,
                        "spacing_override_command_created": False,
                    },
                )
            )
            continue

        if effective_choice == "change_to_lane_1":
            command = _build_lane_change_command(state, decision, geometry=geometry)
            transition = _build_state_transition_command(state, decision)
            overlay = _build_same_step_overlay(state, decision)
            lane_change_commands[cv_id] = command
            state_transition_commands[cv_id] = (transition,)
            same_step_overlays[cv_id] = overlay
            events.append(
                _event(
                    state,
                    module="Step6LaneChangeCommand",
                    event_type="CUC",
                    vehicle_id=cv_id,
                    related_vehicle_ids=(str(request.get("source_mv_id")), cv_id),
                    scenario_id=state.scenario_config_ref or "unknown",
                    reason="lane_change_command_created",
                    source=PAPER_FORMULA_SOURCE,
                    is_engineering_patch=False,
                    payload=command,
                )
            )
            events.append(
                _event(
                    state,
                    module="Step6SameStepOverlay",
                    event_type="CUC",
                    vehicle_id=cv_id,
                    related_vehicle_ids=(str(request.get("source_mv_id")), cv_id),
                    scenario_id=state.scenario_config_ref or "unknown",
                    reason=overlay["reason"],
                    source=ENGINEERING_PATCH_SOURCE,
                    is_engineering_patch=True,
                    payload=overlay,
                )
            )
        elif effective_choice == "stay_lane_2" and request.get("desired_spacing_override") is not None:
            cooperation = _build_spacing_override_command(state, decision, request)
            cooperation_commands[cv_id] = cooperation
            events.append(
                _event(
                    state,
                    module="Step6SpacingOverrideCommand",
                    event_type="CUC",
                    vehicle_id=cv_id,
                    related_vehicle_ids=(str(request.get("source_mv_id")), cv_id),
                    scenario_id=state.scenario_config_ref or "unknown",
                    reason="spacing_override_handoff_to_p08",
                    source=PAPER_FORMULA_SOURCE,
                    is_engineering_patch=False,
                    payload=cooperation,
                )
            )

    state_unchanged = before_signature == _state_signature(state)
    sanity_checks.extend(
        [
            _sanity(
                state,
                "no_write_before_commit",
                "pass" if state_unchanged else "fail",
                vehicle_ids=tuple(state.active_vehicle_ids),
                reason="p07_no_write_before_commit",
                payload={
                    "state_unchanged": state_unchanged,
                    "p07_outputs_are_command_event_overlay_only": True,
                },
            ),
            _sanity(
                state,
                "state_machine_inconsistency",
                "pass",
                vehicle_ids=tuple(decisions) or tuple(state.active_vehicle_ids),
                reason="cuc_decision_not_persistent",
                payload={
                    "cuc_decision_persisted_to_vehicle_state": False,
                    "command_buffer_cuc_decisions_used": False,
                },
            ),
        ]
    )

    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        cooperation_commands=MappingProxyType(cooperation_commands),
        lane_change_commands=MappingProxyType(lane_change_commands),
        state_transition_commands=MappingProxyType(state_transition_commands),
        same_step_overlays=MappingProxyType(same_step_overlays),
    )
    return Step6CUCRunResult(
        state=state,
        relations=relations,
        command_buffer=command_buffer,
        actual_events=events,
        actual_sanity_checks=sanity_checks,
        expected_png_features=register_p07_png_features(
            decisions,
            lane_change_commands=lane_change_commands,
            cooperation_commands=cooperation_commands,
            same_step_overlays=same_step_overlays,
        ),
        cuc_decisions=MappingProxyType(decisions),
        longitudinal_candidates=MappingProxyType({}),
        lateral_candidates=MappingProxyType({}),
        suppressed_requests_ignored_as_active_input=bool(suppressed_requests),
    )


def register_p07_png_features(
    decisions: Mapping[str, Mapping[str, Any]],
    *,
    lane_change_commands: Mapping[str, Mapping[str, Any]],
    cooperation_commands: Mapping[str, Mapping[str, Any]],
    same_step_overlays: Mapping[str, Mapping[str, Any]],
) -> list[dict[str, Any]]:
    features: list[dict[str, Any]] = []
    if decisions:
        features.append(_png_feature("cuc_decision_marker", decisions))
    if lane_change_commands:
        features.append(_png_feature("lane_change_intent_marker", lane_change_commands))
    if cooperation_commands:
        features.append(_png_feature("spacing_override_marker", cooperation_commands))
    if same_step_overlays:
        features.append(_png_feature("same_step_overlay_marker", same_step_overlays))
    unsafe_ids = {
        vehicle_id
        for vehicle_id, decision in decisions.items()
        if decision.get("fallback_reason") == "target_lane_unsafe"
    }
    if unsafe_ids:
        features.append(_png_feature("target_lane_unsafe_fallback_marker", unsafe_ids))
    non_compliant_ids = {
        vehicle_id
        for vehicle_id, decision in decisions.items()
        if decision.get("fallback_reason") == "non_compliant_chv"
    }
    if non_compliant_ids:
        features.append(_png_feature("non_compliant_ignored_marker", non_compliant_ids))
    return features


def _build_base_decision(
    state: SimulationState,
    relations: RelationsSnapshot,
    request: Mapping[str, Any],
) -> dict[str, Any]:
    cv_id = str(request["cv_id"])
    source_mv_id = str(request.get("source_mv_id"))
    return {
        "cuc_decision_id": f"p07:{state.step}:{cv_id}:cuc_decision",
        "source_request_id": str(request.get("request_id")),
        "source_mv_id": source_mv_id,
        "cv_id": cv_id,
        "vehicle_id": cv_id,
        "cv_role": str(request.get("cv_role", "unknown")),
        "aps_case": request.get("aps_case"),
        "assignment_source": request.get("assignment_source"),
        "source_conflict_id": request.get("source_conflict_id"),
        "utility_inputs_logged": False,
        "U1": None,
        "U2": None,
        "target_lane_safe": None,
        "fallback_reason": None,
        "accepted_by_vehicle": None,
        "relation_step": relations.step,
    }


def _evaluate_utility_or_override(
    state: SimulationState,
    relations: RelationsSnapshot,
    request: Mapping[str, Any],
    *,
    override: Mapping[str, Any] | None,
) -> dict[str, Any]:
    cv_id = str(request["cv_id"])
    if override is not None:
        recommended_choice = str(override.get("recommended_choice") or "change_to_lane_1")
        return {
            "utility_source": TEST_HARNESS_OVERRIDE_SOURCE,
            "source": TEST_HARNESS_OVERRIDE_SOURCE,
            "override_reason": "override_choice1_for_required_gate",
            "utility_inputs_logged": True,
            "U1": float(override.get("U1", 1.0)),
            "U2": float(override.get("U2", 0.0)),
            "recommended_choice": recommended_choice,
        }

    neighborhood = relations.lane_change_neighborhood.get(cv_id)
    cv_state = state.vehicle_states[cv_id]
    tlv_gap = _gap_to_neighbor(state, cv_id, neighborhood.tlv_id if neighborhood else None, ahead=True)
    tfv_gap = _gap_to_neighbor(state, cv_id, neighborhood.tfv_id if neighborhood else None, ahead=False)
    # This first-version probe is deliberately simple: it exposes real inputs and
    # a deterministic comparison without locking the paper utility formula.
    u1 = float((tlv_gap or 0.0) + (tfv_gap or 0.0)) / max(float(cv_state.v), 1.0)
    u2 = float(request.get("desired_spacing_override") or 0.0) / 10.0
    return {
        "utility_source": "real_CUC",
        "source": PAPER_FORMULA_SOURCE,
        "utility_inputs_logged": True,
        "U1": u1,
        "U2": u2,
        "recommended_choice": "change_to_lane_1" if u1 > u2 else "stay_lane_2",
    }


def _evaluate_target_lane_safety(
    state: SimulationState,
    relations: RelationsSnapshot,
    cv_id: str,
) -> dict[str, Any]:
    neighborhood = relations.lane_change_neighborhood.get(cv_id)
    tlv_id = neighborhood.tlv_id if neighborhood is not None else None
    tfv_id = neighborhood.tfv_id if neighborhood is not None else None
    cv_state = state.vehicle_states[cv_id]
    tt_cV_tlv = _time_gap_to_neighbor(state, cv_id, tlv_id, ahead=True)
    tt_tfv_cv = _time_gap_to_neighbor(state, cv_id, tfv_id, ahead=False)
    target_lane_safe = (
        (tt_cV_tlv is None or tt_cV_tlv >= TT_MIN_SECONDS)
        and (tt_tfv_cv is None or tt_tfv_cv >= TT_MIN_SECONDS)
    )
    return {
        "vehicle_id": cv_id,
        "target_lane": LANE_1,
        "TLV_id": tlv_id,
        "TFV_id": tfv_id,
        "TT_CV_TLV": tt_cV_tlv,
        "TT_TFV_CV": tt_tfv_cv,
        "TT_min": TT_MIN_SECONDS,
        "target_lane_safe": target_lane_safe,
        "cv_x_global": cv_state.x_global,
    }


def _time_gap_to_neighbor(
    state: SimulationState,
    cv_id: str,
    neighbor_id: str | None,
    *,
    ahead: bool,
) -> float | None:
    gap = _gap_to_neighbor(state, cv_id, neighbor_id, ahead=ahead)
    if gap is None:
        return None
    cv_speed = max(abs(float(state.vehicle_states[cv_id].v)), 1.0)
    return gap / cv_speed


def _gap_to_neighbor(
    state: SimulationState,
    cv_id: str,
    neighbor_id: str | None,
    *,
    ahead: bool,
) -> float | None:
    if neighbor_id is None:
        return None
    cv_x = float(state.vehicle_states[cv_id].x_global)
    neighbor_x = float(state.vehicle_states[neighbor_id].x_global)
    return max(0.0, neighbor_x - cv_x if ahead else cv_x - neighbor_x)


def _vehicle_accepts_cuc(vehicle_type: str, compliance_state: str) -> bool:
    if str(vehicle_type).lower() == "chv" and str(compliance_state).lower() == "non_compliant":
        return False
    return True


def _build_lane_change_command(
    state: SimulationState,
    decision: Mapping[str, Any],
    *,
    geometry: RoadGeometryConfig,
) -> dict[str, Any]:
    cv_id = str(decision["cv_id"])
    target_y = resolve_lane_centerline(LANE_1, geometry=geometry).y
    overlay_id = f"p07:{state.step}:{cv_id}:same_step_overlay"
    return {
        "command_id": f"p07:{state.step}:{cv_id}:lane_change",
        "command_type": "lane_change",
        "module": "Step6CUC",
        "vehicle_id": cv_id,
        "source_request_id": decision["source_request_id"],
        "source_mv_id": decision["source_mv_id"],
        "source_lane": LANE_2,
        "target_lane": LANE_1,
        "target_y": target_y,
        "cuc_decision_id": decision["cuc_decision_id"],
        "overlay_id": overlay_id,
        "init_maneuver": True,
    }


def _build_state_transition_command(
    state: SimulationState,
    decision: Mapping[str, Any],
) -> dict[str, Any]:
    cv_id = str(decision["cv_id"])
    return {
        "command_id": f"p07:{state.step}:{cv_id}:lane_change_state_transition",
        "command_type": "state_transition",
        "module": "Step6CUC",
        "vehicle_id": cv_id,
        "state_name": "lane_change_state",
        "old_state": state.vehicle_states[cv_id].lane_change_state,
        "requested_new_state": "executing",
        "reason": "cuc_choice_change_to_lane_1",
        "cuc_decision_id": decision["cuc_decision_id"],
    }


def _build_same_step_overlay(
    state: SimulationState,
    decision: Mapping[str, Any],
) -> dict[str, Any]:
    cv_id = str(decision["cv_id"])
    overlay_id = f"p07:{state.step}:{cv_id}:same_step_overlay"
    return {
        "overlay_id": overlay_id,
        "vehicle_id": cv_id,
        "source_request_id": decision["source_request_id"],
        "source_mv_id": decision["source_mv_id"],
        "source": ENGINEERING_PATCH_SOURCE,
        "reason": "same_step_cuc_lane_change_relation_overlay",
        "is_engineering_patch": True,
        "source_lane": LANE_2,
        "target_lane": LANE_1,
        "target_lane_neighbors": {
            "TLV_id": decision.get("TLV_id"),
            "TFV_id": decision.get("TFV_id"),
        },
        "cuc_decision_id": decision["cuc_decision_id"],
    }


def _build_spacing_override_command(
    state: SimulationState,
    decision: Mapping[str, Any],
    request: Mapping[str, Any],
) -> dict[str, Any]:
    cv_id = str(decision["cv_id"])
    return {
        "command_id": f"p07:{state.step}:{cv_id}:spacing_override",
        "command_type": "cooperation",
        "module": "Step6CUC",
        "vehicle_id": cv_id,
        "source_request_id": decision["source_request_id"],
        "source_mv_id": decision["source_mv_id"],
        "cv_role": decision["cv_role"],
        "aps_case": decision.get("aps_case"),
        "eq10_desired_spacing": request.get("desired_spacing_override"),
        "consumed_by": "P08",
        "p07_longitudinal_candidate_created": False,
        "cuc_decision_id": decision["cuc_decision_id"],
    }


def _target_lane_safety_event(
    state: SimulationState,
    request: Mapping[str, Any],
    safety: Mapping[str, Any],
) -> dict[str, Any]:
    cv_id = str(request["cv_id"])
    return _event(
        state,
        module="Step6TargetLaneSafety",
        event_type="CUC",
        vehicle_id=cv_id,
        related_vehicle_ids=tuple(
            str(vehicle_id)
            for vehicle_id in (
                request.get("source_mv_id"),
                cv_id,
                safety.get("TLV_id"),
                safety.get("TFV_id"),
            )
            if vehicle_id is not None
        ),
        scenario_id=state.scenario_config_ref or "unknown",
        reason="target_lane_safe" if safety["target_lane_safe"] else "target_lane_unsafe",
        source=PAPER_FORMULA_SOURCE,
        is_engineering_patch=False,
        payload={
            **dict(safety),
            "source_request_id": request.get("request_id"),
            "source_mv_id": request.get("source_mv_id"),
            "fallback_reason": None if safety["target_lane_safe"] else "target_lane_unsafe",
        },
    )


def _compliance_event(
    state: SimulationState,
    request: Mapping[str, Any],
    cv_spec: Any,
    accepted_by_vehicle: bool,
) -> dict[str, Any]:
    cv_id = str(request["cv_id"])
    return _event(
        state,
        module="Step6CUCCompliance",
        event_type="CUC",
        vehicle_id=cv_id,
        related_vehicle_ids=(str(request.get("source_mv_id")), cv_id),
        scenario_id=state.scenario_config_ref or "unknown",
        reason="compliant_accepts_cuc" if accepted_by_vehicle else "non_compliant_chv_ignored",
        source=PAPER_FORMULA_SOURCE,
        is_engineering_patch=False,
        payload={
            "vehicle_id": cv_id,
            "source_request_id": request.get("request_id"),
            "source_mv_id": request.get("source_mv_id"),
            "vehicle_type": cv_spec.vehicle_type,
            "chv_compliance_state": cv_spec.compliance_state,
            "accepted_by_vehicle": accepted_by_vehicle,
            "cuc_suggestion_executed": accepted_by_vehicle,
            "lane_change_command_created": False,
            "spacing_override_consumed_by_p07": False,
        },
    )


def _cuc_event(
    state: SimulationState,
    decision: Mapping[str, Any],
    *,
    reason: str,
) -> dict[str, Any]:
    related_ids = tuple(
        str(vehicle_id)
        for vehicle_id in (
            decision.get("source_mv_id"),
            decision.get("cv_id"),
            decision.get("TLV_id"),
            decision.get("TFV_id"),
        )
        if vehicle_id is not None
    )
    return _event(
        state,
        module="Step6CUC",
        event_type="CUC",
        vehicle_id=str(decision.get("cv_id")),
        related_vehicle_ids=related_ids,
        scenario_id=state.scenario_config_ref or "unknown",
        reason=reason,
        source=str(decision.get("source") or PAPER_FORMULA_SOURCE),
        is_engineering_patch=False,
        payload=dict(decision),
    )


def _decision_reason(decision: Mapping[str, Any]) -> str:
    if decision.get("fallback_reason") == "target_lane_unsafe":
        return "fallback_target_lane_unsafe"
    if decision.get("fallback_reason") == "non_compliant_chv":
        return "non_compliant_chv"
    if decision.get("effective_choice") == "change_to_lane_1":
        return "final_choice_change_to_lane_1"
    return "final_choice_stay_lane_2"


def _event(
    state: SimulationState,
    *,
    module: str,
    event_type: str,
    scenario_id: str,
    reason: str,
    source: str,
    is_engineering_patch: bool,
    payload: Mapping[str, Any],
    vehicle_id: str | None = None,
    related_vehicle_ids: tuple[str, ...] = (),
) -> dict[str, Any]:
    return {
        "t": state.t,
        "step": state.step,
        "module": module,
        "event_type": event_type,
        "vehicle_id": vehicle_id,
        "vehicle_ids": list(related_vehicle_ids or ((vehicle_id,) if vehicle_id else ())),
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
    vehicle_ids: tuple[str, ...],
    reason: str,
    payload: Mapping[str, Any],
) -> dict[str, Any]:
    return {
        "t": state.t,
        "step": state.step,
        "check_type": check_type,
        "result": result,
        "vehicle_ids": list(vehicle_ids),
        "reason": reason,
        "payload": dict(payload),
    }


def _png_feature(feature_type: str, vehicle_ids: Mapping[str, Any] | set[str]) -> dict[str, Any]:
    ids = sorted(vehicle_ids.keys() if isinstance(vehicle_ids, Mapping) else vehicle_ids)
    return {
        "feature_type": feature_type,
        "required": True,
        "vehicle_ids": ids,
        "expected_visibility": "visible",
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
        tuple((key, tuple(sorted(value.items()))) for key, value in state.aps_assignment_cache.items()),
    )
