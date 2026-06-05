from __future__ import annotations

from types import MappingProxyType
from typing import Any

from cormc import build_prefreeze_workspace_from_scenario, freeze_simulation_state, refresh_relations_snapshot
from cormc.step7_longitudinal import run_step7_longitudinal_model_spacing_speedcap
from cormc.step9_11 import CommandBuffer


def test_p08_mvs_cuc_2_consumes_eq10_after_p07_unsafe_fallback() -> None:
    state, relations = _state_and_relations(_p08_config())
    before_signature = _state_signature(state)
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        cooperation_commands=MappingProxyType(
            {
                "CFV_X": {
                    "command_id": "p07:0:CFV_X:spacing_override",
                    "command_type": "cooperation",
                    "module": "Step6CUC",
                    "vehicle_id": "CFV_X",
                    "source_request_id": "p06:0:CFV_X:MV_CUC:cfv",
                    "source_mv_id": "MV_CUC",
                    "cv_role": "cfv",
                    "aps_case": "case_2",
                    "eq10_desired_spacing": 58.0,
                    "consumed_by": "P08",
                    "p07_longitudinal_candidate_created": False,
                    "cuc_decision_id": "p07:0:CFV_X:cuc_decision",
                }
            }
        ),
    )

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=command_buffer,
        vehicle_ids=("CFV_X",),
    )

    assert "CFV_X" in result.next_state_buffer.candidate_longitudinal
    candidate = result.next_state_buffer.candidate_longitudinal["CFV_X"]
    assert candidate.vehicle_id == "CFV_X"
    assert "eq10_spacing_override" in candidate.constraints_applied
    assert "p07:0:CFV_X:spacing_override" in candidate.source_commands
    event = _event(result.actual_events, event_type="longitudinal_model", vehicle_id="CFV_X")
    assert event["payload"]["longitudinal_mode"] == "cav_gap_regulating"
    assert event["payload"]["desired_spacing_source"] == "Eq10"
    assert event["payload"]["desired_spacing_override"] == 58.0
    assert event["payload"]["source_spacing_command_id"] == "p07:0:CFV_X:spacing_override"
    spacing_event = _event(
        result.actual_events,
        event_type="spacing_override_consumption",
        vehicle_id="CFV_X",
    )
    assert spacing_event["payload"]["cv_role"] == "cfv"
    assert spacing_event["payload"]["source_mv_id"] == "MV_CUC"
    assert result.next_state_buffer.candidate_lateral == {}
    assert result.next_state_buffer.candidate_maneuver_progress == {}
    assert not _has_event_type(result.actual_events, "lateral_trajectory")
    assert _state_signature(state) == before_signature


def test_p08_mvs_cuc_3_non_compliant_chv_no_eq10_consumption() -> None:
    state, relations = _state_and_relations(
        _p08_config(cv_type="CHV", compliance_state="non_compliant")
    )
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        cooperation_commands=MappingProxyType(
            {
                "CFV_X": {
                    "command_id": "p07:0:CFV_X:illegal_spacing_override",
                    "command_type": "cooperation",
                    "vehicle_id": "CFV_X",
                    "source_mv_id": "MV_CUC",
                    "cv_role": "cfv",
                    "aps_case": "case_2",
                    "eq10_desired_spacing": 58.0,
                    "consumed_by": "P08",
                }
            }
        ),
    )

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=command_buffer,
        vehicle_ids=("CFV_X",),
    )

    event = _event(result.actual_events, event_type="longitudinal_model", vehicle_id="CFV_X")
    assert event["payload"]["longitudinal_mode"] == "chv_idm"
    assert event["payload"]["vehicle_type"] == "chv"
    assert event["payload"]["compliance_state"] == "non_compliant"
    assert event["payload"]["spacing_override_consumed"] is False
    assert event["payload"]["desired_spacing_source"] == "ordinary_idm"
    assert not _has_event_type(result.actual_events, "spacing_override_consumption")
    wrong_vehicle = _sanity(result.actual_sanity_checks, "Eq10_applied_to_wrong_vehicle")
    assert wrong_vehicle["result"] == "pass"
    assert wrong_vehicle["payload"]["wrong_vehicle_consumption_detected"] is False


def test_p08_mvs_safe_1a_waiting_cap_composes_planning_speed() -> None:
    state, relations = _state_and_relations(_p08_config(mv_x=7237.0))
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        longitudinal_commands=MappingProxyType(
            {
                "MV_CUC": {
                    "command_id": "p05:0:waiting:MV_CUC",
                    "vehicle_id": "MV_CUC",
                    "command_type": "longitudinal",
                    "longitudinal_mode": "cmc_waiting",
                    "source_speed_cap_command_id": "p05:0:speed_cap:MV_CUC",
                }
            }
        ),
        speed_cap_commands=MappingProxyType(
            {
                "MV_CUC": (
                    {
                        "command_id": "p05:0:speed_cap:MV_CUC",
                        "vehicle_id": "MV_CUC",
                        "command_type": "speed_cap",
                        "speed_cap": 2.63,
                        "cap_source": "boundary_collision_avoidance",
                        "cap_reason": "normal_cap",
                        "cap_feasible": True,
                        "cap_binding": True,
                        "source": "paper_formula",
                    },
                )
            }
        ),
    )

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=command_buffer,
        vehicle_ids=("MV_CUC",),
    )

    candidate = result.next_state_buffer.candidate_longitudinal["MV_CUC"]
    assert candidate.planning_speed <= 2.63 + 1e-9
    assert "boundary_speed_cap" in candidate.constraints_applied
    event = _event(result.actual_events, event_type="speed_cap", vehicle_id="MV_CUC")
    assert event["payload"]["base_candidate_speed"] > event["payload"]["boundary_speed_cap"]
    assert event["payload"]["boundary_speed_cap"] == 2.63
    assert event["payload"]["front_fallback_speed"] == "not_applicable"
    assert event["payload"]["planning_speed"] == candidate.planning_speed
    assert event["payload"]["most_conservative_source"] == "boundary_speed_cap"
    assert event["payload"]["source_speed_cap_command_id"] == "p05:0:speed_cap:MV_CUC"
    assert result.next_state_buffer.candidate_lateral == {}
    assert not _has_event_type(result.actual_events, "lateral_trajectory")


def test_p08_cav_cruising_without_leader_or_large_spacing() -> None:
    state, relations = _state_and_relations(_p08_config(cfv_x=6930.0, leader_x=7045.0))

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=CommandBuffer(step=state.step, t=state.t),
        vehicle_ids=("CFV_X",),
    )

    event = _event(result.actual_events, event_type="longitudinal_model", vehicle_id="CFV_X")
    assert event["payload"]["longitudinal_mode"] == "cav_cruising"
    assert "CFV_X" in result.next_state_buffer.candidate_longitudinal


def test_p08_cav_gap_regulating_with_leader_and_small_spacing() -> None:
    state, relations = _state_and_relations(_p08_config(cfv_x=6844.0, leader_x=6865.0))

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=CommandBuffer(step=state.step, t=state.t),
        vehicle_ids=("CFV_X",),
    )

    event = _event(result.actual_events, event_type="longitudinal_model", vehicle_id="CFV_X")
    assert event["payload"]["longitudinal_mode"] == "cav_gap_regulating"
    assert event["payload"]["longitudinal_formula_status"] == "locked_formula"
    assert event["payload"]["cpid_mode"] == "minimal_formula_mode"
    assert event["payload"]["eq21_eq27_locked"] is True
    assert event["payload"]["ev"] == event["payload"]["leader_speed"] - event["payload"]["current_speed"]
    assert event["payload"]["controller_cache_initialized"] is True
    assert result.next_state_buffer.candidate_cache_updates[0].cache_name == "longitudinal_controller_cache"
    assert "CFV_X" in result.next_state_buffer.candidate_longitudinal


def test_p08_cav_cruising_requires_two_times_desired_spacing_threshold() -> None:
    desired_spacing = 40.0
    cfv_x = 6844.0
    vehicle_length = 4.0
    one_point_five_s_leader_x = cfv_x + vehicle_length + 1.5 * desired_spacing
    two_point_one_s_leader_x = cfv_x + vehicle_length + 2.1 * desired_spacing

    gap_state, gap_relations = _state_and_relations(
        _p08_config(cfv_x=cfv_x, leader_x=one_point_five_s_leader_x)
    )
    cruise_state, cruise_relations = _state_and_relations(
        _p08_config(cfv_x=cfv_x, leader_x=two_point_one_s_leader_x)
    )

    gap_result = run_step7_longitudinal_model_spacing_speedcap(
        gap_state,
        gap_relations,
        command_buffer=_spacing_command_buffer(
            gap_state,
            aps_case="case_2",
            cv_role="cfv",
            desired_spacing=desired_spacing,
        ),
        vehicle_ids=("CFV_X",),
    )
    cruise_result = run_step7_longitudinal_model_spacing_speedcap(
        cruise_state,
        cruise_relations,
        command_buffer=_spacing_command_buffer(
            cruise_state,
            aps_case="case_2",
            cv_role="cfv",
            desired_spacing=desired_spacing,
        ),
        vehicle_ids=("CFV_X",),
    )

    assert _event(
        gap_result.actual_events,
        event_type="longitudinal_model",
        vehicle_id="CFV_X",
    )["payload"]["longitudinal_mode"] == "cav_gap_regulating"
    assert _event(
        cruise_result.actual_events,
        event_type="longitudinal_model",
        vehicle_id="CFV_X",
    )["payload"]["longitudinal_mode"] == "cav_cruising"


def test_p08_mv_on_ramp_longitudinal_candidate_without_boundary_cap() -> None:
    state, relations = _state_and_relations(_p08_config(mv_x=6840.0))

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=CommandBuffer(step=state.step, t=state.t),
        vehicle_ids=("MV_CUC",),
    )

    event = _event(result.actual_events, event_type="longitudinal_model", vehicle_id="MV_CUC")
    assert event["payload"]["longitudinal_mode"] == "mv_on_ramp"
    assert "MV_CUC" in result.next_state_buffer.candidate_longitudinal
    assert not _has_event_type(result.actual_events, "speed_cap")
    assert not _has_event_type(result.actual_events, "CMC")
    assert not _has_event_type(result.actual_events, "lateral_trajectory")


def test_p08_mv_control_zone_applies_aps_gap_protection_for_all_cases() -> None:
    for aps_case in ("case_1", "case_2", "case_3", "case_4"):
        state, relations = _state_and_relations(
            _p08_config(
                mv_x=6840.0,
                mv_initial_v=28.0,
                preloaded_assignments=[
                    _aps_assignment("MV_CUC", aps_case=aps_case, d_star_clv=30.0)
                ],
            )
        )

        result = run_step7_longitudinal_model_spacing_speedcap(
            state,
            relations,
            command_buffer=CommandBuffer(step=state.step, t=state.t),
            vehicle_ids=("MV_CUC",),
        )

        event = _event(result.actual_events, event_type="longitudinal_model", vehicle_id="MV_CUC")
        candidate = result.next_state_buffer.candidate_longitudinal["MV_CUC"]
        assert event["payload"]["aps_gap_protection_applied"] is True
        assert event["payload"]["aps_gap_protection_speed_cap"] == 25.0
        assert event["payload"]["aps_gap_protection_source"] == "d_star_clv_over_tau"
        assert event["payload"]["source_aps_case"] == aps_case
        assert event["payload"]["source_d_star_clv"] == 30.0
        assert event["payload"]["source_tau"] == 1.2
        assert event["payload"]["original_desired_speed"] == 30.0
        assert event["payload"]["effective_desired_speed"] == 25.0
        assert event["payload"]["aps_gap_protection_rejection_reason"] is None
        assert candidate.v < state.vehicle_states["MV_CUC"].v
        assert candidate.a < 0.0
        assert not _has_event_type(result.actual_events, "speed_cap")


def test_p08_mv_control_zone_valid_assignment_missing_d_star_clv_is_diagnostic() -> None:
    state, relations = _state_and_relations(
        _p08_config(
            mv_x=6840.0,
            mv_initial_v=28.0,
            preloaded_assignments=[
                _aps_assignment("MV_CUC", aps_case="case_2", include_d_star_clv=False)
            ],
        )
    )

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=CommandBuffer(step=state.step, t=state.t),
        vehicle_ids=("MV_CUC",),
    )

    event = _event(result.actual_events, event_type="longitudinal_model", vehicle_id="MV_CUC")
    candidate = result.next_state_buffer.candidate_longitudinal["MV_CUC"]
    assert event["payload"]["aps_gap_protection_applied"] is False
    assert event["payload"]["aps_gap_protection_rejection_reason"] == "missing_d_star_clv"
    assert event["payload"]["effective_desired_speed"] == 30.0
    assert candidate.v > state.vehicle_states["MV_CUC"].v


def test_p08_mv_control_zone_ignores_invalid_assignment_status() -> None:
    state, relations = _state_and_relations(
        _p08_config(
            mv_x=6840.0,
            mv_initial_v=28.0,
            preloaded_assignments=[
                _aps_assignment("MV_CUC", aps_case="case_2", d_star_clv=30.0, status="invalid")
            ],
        )
    )

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=CommandBuffer(step=state.step, t=state.t),
        vehicle_ids=("MV_CUC",),
    )

    event = _event(result.actual_events, event_type="longitudinal_model", vehicle_id="MV_CUC")
    assert event["payload"]["aps_gap_protection_applied"] is False
    assert (
        event["payload"]["aps_gap_protection_rejection_reason"]
        == "invalid_assignment_status:invalid"
    )
    assert event["payload"]["effective_desired_speed"] == 30.0


def test_p08_mv_merge_zone_keeps_cmc_boundary_speed_cap_independent() -> None:
    state, relations = _state_and_relations(
        _p08_config(
            mv_x=7237.0,
            preloaded_assignments=[
                _aps_assignment("MV_CUC", aps_case="case_2", d_star_clv=30.0)
            ],
        )
    )
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        speed_cap_commands=MappingProxyType(
            {
                "MV_CUC": (
                    {
                        "command_id": "p05:0:speed_cap:MV_CUC",
                        "vehicle_id": "MV_CUC",
                        "command_type": "speed_cap",
                        "speed_cap": 2.63,
                        "cap_feasible": True,
                        "cap_binding": True,
                    },
                )
            }
        ),
    )

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=command_buffer,
        vehicle_ids=("MV_CUC",),
    )

    longitudinal = _event(result.actual_events, event_type="longitudinal_model", vehicle_id="MV_CUC")
    speed_cap = _event(result.actual_events, event_type="speed_cap", vehicle_id="MV_CUC")
    assert longitudinal["payload"]["aps_gap_protection_applied"] is False
    assert longitudinal["payload"]["aps_gap_protection_rejection_reason"] == "not_control_zone:merge_zone"
    assert longitudinal["payload"]["aps_gap_protection_speed_cap"] is None
    assert speed_cap["payload"]["boundary_speed_cap"] == 2.63
    assert speed_cap["payload"]["most_conservative_source"] == "boundary_speed_cap"


def test_p08_chv_uses_idm_and_compliance_does_not_change_model() -> None:
    compliant_state, compliant_relations = _state_and_relations(
        _p08_config(cv_type="CHV", compliance_state="compliant")
    )
    non_state, non_relations = _state_and_relations(
        _p08_config(cv_type="CHV", compliance_state="non_compliant")
    )

    compliant = run_step7_longitudinal_model_spacing_speedcap(
        compliant_state,
        compliant_relations,
        command_buffer=CommandBuffer(step=compliant_state.step, t=compliant_state.t),
        vehicle_ids=("CFV_X",),
    )
    non_compliant = run_step7_longitudinal_model_spacing_speedcap(
        non_state,
        non_relations,
        command_buffer=CommandBuffer(step=non_state.step, t=non_state.t),
        vehicle_ids=("CFV_X",),
    )

    assert _event(compliant.actual_events, event_type="longitudinal_model", vehicle_id="CFV_X")[
        "payload"
    ]["longitudinal_mode"] == "chv_idm"
    assert _event(non_compliant.actual_events, event_type="longitudinal_model", vehicle_id="CFV_X")[
        "payload"
    ]["longitudinal_mode"] == "chv_idm"


def test_p08_case_2_and_case_4_cfv_only_eq10_consumption() -> None:
    for aps_case in ("case_2", "case_4"):
        state, relations = _state_and_relations(_p08_config())
        result = run_step7_longitudinal_model_spacing_speedcap(
            state,
            relations,
            command_buffer=_spacing_command_buffer(state, aps_case=aps_case, cv_role="cfv"),
            vehicle_ids=("CFV_X",),
        )
        assert _has_event_type(result.actual_events, "spacing_override_consumption")

    state, relations = _state_and_relations(_p08_config())
    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=_spacing_command_buffer(state, aps_case="case_2", cv_role="clv"),
        vehicle_ids=("CFV_X",),
    )
    assert not _has_event_type(result.actual_events, "spacing_override_consumption")


def test_p08_case_3_clv_no_eq10_consumption() -> None:
    state, relations = _state_and_relations(_p08_config())

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=_spacing_command_buffer(state, aps_case="case_3", cv_role="clv"),
        vehicle_ids=("CFV_X",),
    )

    assert not _has_event_type(result.actual_events, "spacing_override_consumption")
    sanity = _sanity(result.actual_sanity_checks, "Eq10_applied_to_wrong_vehicle")
    assert sanity["result"] == "pass"


def test_p08_speed_cap_front_fallback_base_speed_min_composition() -> None:
    state, relations = _state_and_relations(_p08_config(mv_x=7237.0))
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        speed_cap_commands=MappingProxyType(
            {
                "MV_CUC": (
                    {
                        "command_id": "p05:0:speed_cap:MV_CUC",
                        "vehicle_id": "MV_CUC",
                        "command_type": "speed_cap",
                        "speed_cap": 12.0,
                        "cap_feasible": True,
                        "cap_binding": True,
                    },
                )
            }
        ),
    )

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=command_buffer,
        vehicle_ids=("MV_CUC",),
        front_fallback_speeds={"MV_CUC": 8.0},
    )

    event = _event(result.actual_events, event_type="speed_cap", vehicle_id="MV_CUC")
    assert event["payload"]["base_candidate_speed"] > 12.0
    assert event["payload"]["boundary_speed_cap"] == 12.0
    assert event["payload"]["front_fallback_speed"] == 8.0
    assert event["payload"]["planning_speed"] == 8.0
    assert event["payload"]["most_conservative_source"] == "front_fallback"


def test_p08_does_not_rerun_aps_cmc_p06_or_p07() -> None:
    state, relations = _state_and_relations(_p08_config())

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=CommandBuffer(step=state.step, t=state.t),
    )

    forbidden_event_types = {
        "APS",
        "APS_candidate",
        "CMC",
        "assignment_validation",
        "cooperative_request",
        "conflict_resolution",
        "CUC",
    }
    assert forbidden_event_types.isdisjoint({event["event_type"] for event in result.actual_events})


def test_p08_ignores_suppressed_or_historical_request_without_p07_spacing_command() -> None:
    state, relations = _state_and_relations(_p08_config())

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=CommandBuffer(step=state.step, t=state.t),
        suppressed_requests=(
            {
                "request_id": "p06:0:CFV_X:MV_LOSER:cfv",
                "source_mv_id": "MV_LOSER",
                "cv_id": "CFV_X",
                "cv_role": "cfv",
                "aps_case": "case_2",
                "desired_spacing_override": 58.0,
                "active": False,
            },
        ),
        vehicle_ids=("CFV_X",),
    )

    assert not _has_event_type(result.actual_events, "spacing_override_consumption")
    event = _event(result.actual_events, event_type="longitudinal_model", vehicle_id="CFV_X")
    assert event["payload"]["spacing_override_consumed"] is False


def test_p08_does_not_create_lateral_candidates_or_maneuver_progress() -> None:
    state, relations = _state_and_relations(_p08_config())

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=CommandBuffer(step=state.step, t=state.t),
    )

    assert result.next_state_buffer.candidate_lateral == {}
    assert result.next_state_buffer.candidate_maneuver_progress == {}


def test_p08_no_write_before_commit() -> None:
    state, relations = _state_and_relations(_p08_config())
    before_signature = _state_signature(state)

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=CommandBuffer(step=state.step, t=state.t),
    )

    assert _state_signature(state) == before_signature
    sanity = _sanity(result.actual_sanity_checks, "no_write_before_commit")
    assert sanity["result"] == "pass"
    assert all(
        candidate.planning_speed != state.vehicle_states[vehicle_id].v
        or state.vehicle_states[vehicle_id].v == before_speed
        for vehicle_id, candidate in result.next_state_buffer.candidate_longitudinal.items()
        for before_speed in (state.vehicle_states[vehicle_id].v,)
    )


def test_p08_planning_speed_handoff_for_p09_without_lateral_progress() -> None:
    state, relations = _state_and_relations(_p08_config())

    result = run_step7_longitudinal_model_spacing_speedcap(
        state,
        relations,
        command_buffer=CommandBuffer(step=state.step, t=state.t),
        vehicle_ids=("CFV_X",),
    )

    candidate = result.next_state_buffer.candidate_longitudinal["CFV_X"]
    assert result.planning_speeds["CFV_X"] == candidate.planning_speed
    assert candidate.planning_speed >= 0.0
    assert result.next_state_buffer.candidate_lateral == {}
    assert result.next_state_buffer.candidate_maneuver_progress == {}


def _spacing_command_buffer(
    state: Any,
    *,
    aps_case: str,
    cv_role: str,
    desired_spacing: float = 58.0,
) -> CommandBuffer:
    return CommandBuffer(
        step=state.step,
        t=state.t,
        cooperation_commands=MappingProxyType(
            {
                "CFV_X": {
                    "command_id": f"p07:0:CFV_X:{aps_case}:{cv_role}:spacing_override",
                    "command_type": "cooperation",
                    "vehicle_id": "CFV_X",
                    "source_request_id": "p06:0:CFV_X:MV_CUC:cfv",
                    "source_mv_id": "MV_CUC",
                    "cv_role": cv_role,
                    "aps_case": aps_case,
                    "eq10_desired_spacing": desired_spacing,
                    "consumed_by": "P08",
                }
            }
        ),
    )


def _p08_config(
    *,
    cv_type: str = "CAV",
    compliance_state: str = "not_applicable",
    cfv_x: float = 6844.0,
    leader_x: float = 6865.0,
    mv_x: float = 6840.0,
    mv_initial_v: float = 16.0,
    preloaded_assignments: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    return {
        "scenario_id": "P08-STEP7-UNIT",
        "scenario_name": "P08 Step7 unit",
        "purpose": "Inline P08 Step7 tests",
        "test_level": "unit",
        "status": "required",
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            _vehicle(
                "MV_CUC",
                "on_ramp",
                mv_x,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
                initial_v=mv_initial_v,
            ),
            _vehicle(
                "CFV_X",
                "lane_2",
                cfv_x,
                0.0,
                vehicle_type=cv_type,
                compliance_state=compliance_state,
                initial_v=20.0,
            ),
            _vehicle("CLV_Y", "lane_2", leader_x, 0.0, initial_v=18.0),
            _vehicle("FV_X", "lane_2", 6780.0, 0.0, initial_v=20.0),
        ],
        "module_overrides": {
            "boundary_generation_enabled": False,
            "random_arrival_enabled": False,
            "random_vehicle_attributes_enabled": False,
        },
        "preloaded_assignments": preloaded_assignments or [],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": [],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [],
        "expected_png_features": [],
    }


def _aps_assignment(
    mv_id: str,
    *,
    aps_case: str,
    d_star_clv: float = 30.0,
    include_d_star_clv: bool = True,
    d_star_cfv: float = -24.0,
    status: str = "valid",
) -> dict[str, Any]:
    assignment = {
        "mv_id": mv_id,
        "clv_id": "CLV_Y",
        "cfv_id": "CFV_X",
        "aps_case": aps_case,
        "col_clv": aps_case in {"case_3", "case_4"},
        "col_cfv": aps_case in {"case_2", "case_4"},
        "desired_spacing_override": 58.0 if aps_case in {"case_2", "case_4"} else None,
        "d_star_cfv": d_star_cfv,
        "aps_min_merge_time_gap_s": 1.2,
        "t_star_mv": 5.5,
        "t_mv_star": 5.5,
        "status": status,
        "created_at_t": 0.0,
        "created_at_step": 0,
        "source": "aps_cache",
        "valid_until_next_aps": True,
        "staleness_policy": "valid_until_next_aps",
    }
    if include_d_star_clv:
        assignment["d_star_clv"] = d_star_clv
    return assignment


def _vehicle(
    vehicle_id: str,
    lane: str,
    x_global: float,
    y: float,
    *,
    vehicle_type: str = "CAV",
    compliance_state: str = "not_applicable",
    road_role: str = "mainline",
    lane_change_state: str = "normal",
    merge_state: str = "none",
    initial_v: float = 20.0,
) -> dict[str, Any]:
    return {
        "vehicle_id": vehicle_id,
        "vehicle_type": vehicle_type,
        "compliance_state": compliance_state,
        "initial_x_global": x_global,
        "initial_y": y,
        "initial_v": initial_v,
        "initial_a": 0.0,
        "physical_lane": lane,
        "road_role": road_role,
        "lane_change_state": lane_change_state,
        "merge_state": merge_state,
        "spec_overrides": {},
    }


def _state_and_relations(config: dict[str, Any]) -> tuple[Any, Any]:
    workspace, _ = build_prefreeze_workspace_from_scenario(config)
    state = freeze_simulation_state(workspace)
    relations = refresh_relations_snapshot(state)
    return state, relations


def _event(
    events: list[dict[str, Any]],
    *,
    event_type: str,
    vehicle_id: str,
) -> dict[str, Any]:
    for event in events:
        if event.get("event_type") == event_type and event.get("vehicle_id") == vehicle_id:
            return event
    raise AssertionError(f"missing event: {event_type} {vehicle_id}")


def _sanity(checks: list[dict[str, Any]], check_type: str) -> dict[str, Any]:
    for check in checks:
        if check.get("check_type") == check_type:
            return check
    raise AssertionError(f"missing sanity: {check_type}")


def _has_event_type(events: list[dict[str, Any]], event_type: str) -> bool:
    return any(event.get("event_type") == event_type for event in events)


def _state_signature(state: Any) -> tuple[Any, ...]:
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
