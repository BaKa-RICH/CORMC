from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from cormc.mvs import run_targeted_scenario
from cormc.p11_output import build_regression_report, run_full_required_mvs_smoke_suite
from cormc.simulation_loop import (
    SimulationLoopConfig,
    build_step_command_buffer,
    run_deterministic_simulation,
)
from cormc.step9_11 import CommandBuffer


def test_p12_mvs_e2e_1_official_runner_route_green() -> None:
    report = run_targeted_scenario("MVS-E2E-1")

    assert report.scenario_id == "MVS-E2E-1"
    assert report.status == "required"
    assert report.classification == "required_passed"
    assert report.passed is True
    assert report.blocks_required_suite is False
    assert report.failure_reasons == []


def test_p12_runs_multiple_steps_advances_state_and_completes_merge_before_boundary() -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id="MVS-E2E-1",
            run_id="p12-e2e-complete",
            max_steps=120,
            render_png=False,
        )
    )

    initial = result.initial_state.vehicle_states["MV_DEMO"]
    final = result.final_state.vehicle_states["MV_DEMO"]

    assert result.final_state.step == 120
    assert result.final_state.t > result.initial_state.t
    assert final.x_global > initial.x_global
    assert final.x_global < 7250.0
    assert final.y == 0.0
    assert final.merge_state == "merged"
    assert final.physical_lane == "lane_2"
    assert final.road_role == "mainline"
    assert "MV_DEMO" not in result.final_state.active_maneuvers
    assert {record.step for record in result.history.trajectory_records} >= set(range(0, 120))


def test_p12_step_order_event_chain() -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id="MVS-E2E-1",
            run_id="p12-e2e-chain",
            max_steps=70,
            render_png=False,
        )
    )
    events = result.history.event_dicts()

    assert _has_event(events, "APS", reason="first_aps", payload={"aps_case": "case_1"})
    assert _has_event(events, "CMC", reason="assignment_validation", payload={"assignment_valid": True})
    assert _has_event(events, "CMC", reason="eq53_gap", payload={"eq53_pass": True})
    assert _has_event(events, "CMC", reason="merge_start")
    assert _has_event(events, "longitudinal_model", vehicle_id="MV_DEMO")
    assert _has_event(events, "lateral_trajectory", vehicle_id="MV_DEMO", payload={"maneuver_type": "merge"})
    assert _has_event(events, "commit", vehicle_id="MV_DEMO")
    assert _has_event(events, "information_integration")
    assert _has_event(events, "time_advance")


def test_p12_canonical_command_buffer_does_not_overwrite_namespaces() -> None:
    p05_buffer = CommandBuffer(
        step=3,
        t=0.3,
        merge_commands={"MV_A": {"command_id": "p05:merge", "vehicle_id": "MV_A"}},
        speed_cap_commands={"MV_A": ({"command_id": "p05:cap", "vehicle_id": "MV_A"},)},
        state_transition_commands={
            "MV_A": ({"command_id": "p05:state", "vehicle_id": "MV_A", "state_name": "merge_state", "new_state": "executing"},)
        },
        cache_update_commands=({"command_id": "p05:cache", "owner_vehicle_id": "MV_A"},),
    )
    p07_buffer = CommandBuffer(
        step=3,
        t=0.3,
        cooperation_commands={"CFV_X": {"command_id": "p07:spacing", "vehicle_id": "CFV_X"}},
        lane_change_commands={"CFV_X": {"command_id": "p07:lane", "vehicle_id": "CFV_X"}},
        state_transition_commands={
            "CFV_X": ({"command_id": "p07:state", "vehicle_id": "CFV_X", "state_name": "lane_change_state", "requested_new_state": "executing"},)
        },
        same_step_overlays={"CFV_X": {"overlay_id": "p07:overlay"}},
    )

    combined = build_step_command_buffer(
        SimpleNamespace(state=SimpleNamespace(step=3, t=0.3), command_buffer=p05_buffer),
        SimpleNamespace(active_requests={"CFV_X": {"request_id": "p06:request"}}),
        SimpleNamespace(command_buffer=p07_buffer, cuc_decisions={"CFV_X": {"final_choice": "change_to_lane_1"}}),
    )

    assert combined.merge_commands["MV_A"]["command_id"] == "p05:merge"
    assert combined.speed_cap_commands["MV_A"][0]["command_id"] == "p05:cap"
    assert combined.lane_change_commands["CFV_X"]["command_id"] == "p07:lane"
    assert combined.cooperation_commands["CFV_X"]["command_id"] == "p07:spacing"
    assert "CFV_X" in combined.same_step_overlays
    assert "MV_A" in combined.state_transition_commands
    assert "CFV_X" in combined.state_transition_commands
    assert "CFV_X" not in combined.cooperation_commands.get("MV_A", {})


def test_p12_next_state_buffer_preserves_p08_p09_boundaries() -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id="MVS-E2E-1",
            run_id="p12-e2e-buffer",
            max_steps=61,
            render_png=False,
        )
    )
    trace = result.step_traces[60]
    buffer = trace.canonical_next_state_buffer

    assert "MV_DEMO" in buffer.candidate_longitudinal
    assert "MV_DEMO" in buffer.candidate_lateral
    assert "MV_DEMO" in buffer.candidate_maneuver_progress
    assert not buffer.candidate_kinematics
    assert buffer.candidate_longitudinal["MV_DEMO"].x_global > trace.step0_3_result.state.vehicle_states["MV_DEMO"].x_global
    assert buffer.candidate_lateral["MV_DEMO"].y > trace.step0_3_result.state.vehicle_states["MV_DEMO"].y


def test_p12_aps_cache_action_commits_to_next_state_and_reuses_cache() -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id="MVS-E2E-1",
            run_id="p12-cache",
            max_steps=2,
            render_png=False,
        )
    )

    assert "MV_DEMO" in result.final_state.aps_assignment_cache
    assert result.final_state.aps_assignment_cache["MV_DEMO"]["aps_case"] == "case_1"
    assert _has_event(result.history.event_dicts(), "assignment_cache", reason="reuse_cache_until_next_APS")


def test_p12_maneuver_planned_length_uses_parameter_spec() -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id="P12-BRANCH-CUC-LANECHANGE",
            run_id="p12-lane-length",
            max_steps=1,
            render_png=False,
        )
    )
    trace = result.step_traces[0]
    command = trace.canonical_command_buffer.lane_change_commands["CFV_X"]
    progress = trace.canonical_next_state_buffer.candidate_maneuver_progress["CFV_X"]

    assert command["planned_length"] == 100.0
    assert command["planned_length_source"] == "lane_change_centerline_length"
    assert progress.progress > 0.0


def test_p12_cuc_lanechange_branch_uses_labeled_override() -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id="P12-BRANCH-CUC-LANECHANGE",
            run_id="p12-cuc-lanechange",
            max_steps=1,
            render_png=False,
        )
    )
    state = result.final_state.vehicle_states["CFV_X"]
    events = result.history.event_dicts()

    assert state.lane_change_state == "executing"
    assert state.y > 0.0
    assert _has_event(
        events,
        "CUC",
        vehicle_id="CFV_X",
        payload={"utility_formula_status": "test_harness_override_not_formula"},
    )
    assert _has_event(
        events,
        "lateral_trajectory",
        vehicle_id="CFV_X",
        payload={"source_scenario_id": "P12-BRANCH-CUC-LANECHANGE"},
    )


def test_p12_cuc_fallback_and_non_compliant_branches() -> None:
    fallback = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id="P12-BRANCH-CUC-FALLBACK",
            run_id="p12-cuc-fallback",
            max_steps=1,
            render_png=False,
        )
    )
    non_compliant = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id="P12-BRANCH-CUC-NONCOMPLIANT",
            run_id="p12-cuc-noncompliant",
            max_steps=1,
            render_png=False,
        )
    )

    assert fallback.final_state.vehicle_states["CFV_X"].lane_change_state == "normal"
    assert fallback.final_state.vehicle_states["CFV_X"].physical_lane == "lane_2"
    assert _has_event(fallback.history.event_dicts(), "CUC", vehicle_id="CFV_X", reason="fallback_target_lane_unsafe")
    assert _has_event(fallback.history.event_dicts(), "spacing_override_consumption", vehicle_id="CFV_X")

    assert non_compliant.final_state.vehicle_states["CFV_X"].lane_change_state == "normal"
    assert _has_event(non_compliant.history.event_dicts(), "CUC", vehicle_id="CFV_X", reason="non_compliant_chv")
    assert not _has_event(non_compliant.history.event_dicts(), "spacing_override_consumption", vehicle_id="CFV_X")
    assert _has_event(non_compliant.history.event_dicts(), "longitudinal_model", vehicle_id="CFV_X", payload={"longitudinal_mode": "chv_idm"})


def test_p12_safe_cap_branches() -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id="P12-BRANCH-SAFE-CAP",
            run_id="p12-safe-cap",
            max_steps=1,
            render_png=False,
        )
    )
    events = result.history.event_dicts()

    assert _has_event(events, "speed_cap", vehicle_id="MV_SAFE_WAIT", payload={"most_conservative_source": "boundary_speed_cap"})
    assert not _has_event(events, "lateral_trajectory", vehicle_id="MV_SAFE_WAIT")
    assert _has_event(
        events,
        "lateral_trajectory",
        vehicle_id="MV_SAFE_EXEC",
        payload={"trajectory_consumed_speed_source": "p08_planning_speed"},
    )
    assert _has_event(events, "CMC", vehicle_id="MV_SAFE_RISK", reason="boundary_speed_cap", payload={"cap_feasible": False})
    assert _has_event(events, "lateral_trajectory", vehicle_id="MV_SAFE_RISK", payload={"boundary_risk_status": "cap_infeasible"})


def test_p12_active_continuation_does_not_rerun_decisions() -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id="P12-BRANCH-ACTIVE-CONTINUATION",
            run_id="p12-active-continuation",
            max_steps=1,
            render_png=False,
        )
    )
    events = result.history.event_dicts()

    assert _has_event(events, "APS", vehicle_id="MV_CACHE", reason="reuse_cache")
    assert _has_event(events, "CMC", vehicle_id="MV_ACTIVE", reason="cmc_executing_continuation")
    assert not _has_event(events, "CMC", vehicle_id="MV_ACTIVE", reason="eq53_gap")
    assert _has_event(events, "lateral_trajectory", vehicle_id="CFV_ACTIVE", reason="lane_change_continuation")
    assert not _has_event(events, "CUC", vehicle_id="CFV_ACTIVE", reason="final_choice_change_to_lane_1")
    assert result.final_state.active_maneuvers["CFV_ACTIVE"].progress > result.initial_state.active_maneuvers["CFV_ACTIVE"].progress
    assert result.final_state.active_maneuvers["MV_ACTIVE"].progress > result.initial_state.active_maneuvers["MV_ACTIVE"].progress


def test_p12_png_created_nonblank_and_contains_feature_evidence(tmp_path: Path) -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id="MVS-E2E-1",
            run_id="p12-png",
            max_steps=70,
            output_dir=tmp_path,
            render_png=True,
        )
    )

    png_path = Path(result.png_path or "")
    assert png_path.exists()
    data = png_path.read_bytes()
    assert data.startswith(b"\x89PNG\r\n\x1a\n")
    assert len(data) > 500
    features = {feature["feature_type"] for feature in result.expected_png_features}
    assert {
        "lane_centerline_quicklook",
        "merging_zone_boundary_quicklook",
        "merge_start_marker",
        "merge_trajectory_marker",
        "commit_marker",
    }.issubset(features)


def test_p12_does_not_claim_full_required_ready() -> None:
    suite = run_full_required_mvs_smoke_suite()
    report = build_regression_report(suite, run_id="p12-suite")
    by_id = {result.scenario_id: result for result in suite.scenario_results}

    assert by_id["MVS-E2E-1"].classification == "required_passed"
    assert "MVS-E2E-1" in report.required_green
    assert by_id["MVS-COMMIT-1-full"].classification == "required_blocked"
    assert any(item["scenario_id"] == "MVS-COMMIT-1-full" for item in report.required_blocked)
    assert report.suite_status == "failed_until_required_blockers_resolved"
    assert "MVS-COMMIT-1-full" in report.runner_gaps


def _has_event(
    events: list[dict],
    event_type: str,
    *,
    vehicle_id: str | None = None,
    reason: str | None = None,
    payload: dict | None = None,
) -> bool:
    for event in events:
        if event.get("event_type") != event_type and event.get("module") != event_type:
            continue
        if vehicle_id is not None and vehicle_id not in set(event.get("vehicle_ids") or []):
            continue
        if reason is not None and event.get("reason") != reason:
            continue
        event_payload = event.get("payload") or {}
        if payload and any(event_payload.get(key) != value for key, value in payload.items()):
            continue
        return True
    return False
