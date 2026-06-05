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


def test_p12_pre_control_mv_rolls_dt_without_control_modules() -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario=_basic_region_config("BASIC-PRE-CONTROL-GATING", mv_x=6640.0),
            run_id="p12-pre-control-gating",
            max_steps=5,
            render_png=False,
        )
    )
    events = result.history.event_dicts()

    assert result.final_state.step == 5
    assert result.final_state.vehicle_states["BASIC_MV"].x_global > 6640.0
    assert result.final_state.aps_assignment_cache == {}
    assert {trace.on_ramp_control_regions["BASIC_MV"].region for trace in result.step_traces} == {
        "pre_control",
    }
    assert not _has_event(events, "APS", vehicle_id="BASIC_MV")
    assert not _has_event(events, "assignment_cache", vehicle_id="BASIC_MV")
    assert not _has_event(events, "cooperative_request")
    assert not _has_event(events, "CUC")
    assert not _has_event(events, "CMC", vehicle_id="BASIC_MV")
    assert _has_event(events, "longitudinal_model", vehicle_id="BASIC_MV")
    assert _has_event(events, "commit", vehicle_id="BASIC_MV")


def test_p12_control_zone_mv_runs_aps_not_cmc() -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario=_basic_region_config("BASIC-CONTROL-ZONE-GATING", mv_x=6850.0),
            run_id="p12-control-zone-gating",
            max_steps=1,
            render_png=False,
        )
    )
    events = result.history.event_dicts()
    trace = result.step_traces[0]

    assert trace.on_ramp_control_regions["BASIC_MV"].region == "control_zone"
    assert _has_event(events, "APS", vehicle_id="BASIC_MV", reason="first_aps")
    assert _has_event(events, "assignment_cache", vehicle_id="BASIC_MV")
    assert _has_event(events, "cooperative_request", vehicle_id="BASIC_CFV")
    assert _has_event(events, "CUC", vehicle_id="BASIC_CFV")
    assert not _has_event(events, "CMC", vehicle_id="BASIC_MV")


def test_p12_merge_zone_mv_runs_cmc_without_new_aps_or_cuc() -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario=_basic_region_config(
                "BASIC-MERGE-ZONE-GATING",
                mv_x=6950.0,
                clv_x=6990.0,
                cfv_x=6920.0,
                preload_assignment=True,
            ),
            run_id="p12-merge-zone-gating",
            max_steps=1,
            render_png=False,
        )
    )
    events = result.history.event_dicts()
    trace = result.step_traces[0]

    assert trace.on_ramp_control_regions["BASIC_MV"].region == "merge_zone"
    assert not _has_event(events, "APS", vehicle_id="BASIC_MV")
    assert not _has_event(events, "assignment_cache", vehicle_id="BASIC_MV")
    assert not _has_event(events, "cooperative_request")
    assert not _has_event(events, "CUC")
    assert _has_event(events, "CMC", vehicle_id="BASIC_MV", reason="assignment_validation")
    assert _has_event(events, "CMC", vehicle_id="BASIC_MV", reason="boundary_speed_cap")


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


def test_p13_closes_full_required_mvs_suite() -> None:
    suite = run_full_required_mvs_smoke_suite()
    report = build_regression_report(suite, run_id="p12-suite")
    by_id = {result.scenario_id: result for result in suite.scenario_results}

    assert by_id["MVS-E2E-1"].classification == "required_passed"
    assert "MVS-E2E-1" in report.required_green
    assert by_id["MVS-COMMIT-1-full"].classification == "required_passed"
    assert by_id["MVS-SAFE-1A_waiting_cap"].classification == "required_passed"
    assert len(report.required_green) == 20
    assert report.required_blocked == ()
    assert report.required_failed == ()
    assert report.runner_gaps == ()
    assert report.classification_blockers == ()
    assert report.suite_status == "passed"


def test_p13_commit_full_advances_active_maneuvers_and_step11_once() -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id="MVS-COMMIT-1-full",
            run_id="p13-commit-full",
            max_steps=1,
            render_png=False,
        )
    )
    events = result.history.event_dicts()

    assert result.initial_state.step == 20
    assert result.final_state.step == 21
    assert result.final_state.t == 2.1
    assert (
        result.final_state.active_maneuvers["CV_ACTIVE_LC"].progress
        > result.initial_state.active_maneuvers["CV_ACTIVE_LC"].progress
    )
    assert (
        result.final_state.active_maneuvers["MV_ACTIVE_MERGE"].progress
        > result.initial_state.active_maneuvers["MV_ACTIVE_MERGE"].progress
    )
    assert _has_event(events, "APS", vehicle_id="MV_CACHE", reason="reuse_cache")
    assert _has_event(events, "CMC", vehicle_id="MV_ACTIVE_MERGE", reason="cmc_executing_continuation")
    assert not _has_event(events, "CMC", vehicle_id="MV_ACTIVE_MERGE", reason="eq53_gap")
    assert _has_event(events, "lateral_trajectory", vehicle_id="CV_ACTIVE_LC", reason="lane_change_continuation")
    assert _has_event(events, "lateral_trajectory", vehicle_id="MV_ACTIVE_MERGE", reason="merge_continuation")
    assert not _has_event(events, "CUC", vehicle_id="CV_ACTIVE_LC", reason="final_choice_change_to_lane_1")


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


def _basic_region_config(
    scenario_id: str,
    *,
    mv_x: float,
    clv_x: float | None = None,
    cfv_x: float | None = None,
    preload_assignment: bool = False,
) -> dict:
    clv_x = mv_x + 34.0 if clv_x is None else clv_x
    cfv_x = mv_x - 6.0 if cfv_x is None else cfv_x
    config = {
        "scenario_id": scenario_id,
        "scenario_name": scenario_id,
        "purpose": "P12 targeted on-ramp control-region gating scenario",
        "test_level": "integration",
        "status": "required",
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            _vehicle("BASIC_MV", "on_ramp", mv_x, -3.5, road_role="on_ramp_mv", merge_state="not_started"),
            _vehicle("BASIC_CLV", "lane_2", clv_x, 0.0),
            _vehicle("BASIC_CFV", "lane_2", cfv_x, 0.0),
            _vehicle("BASIC_TLV_CFV", "lane_1", cfv_x + 9.0, 3.5, v=15.0),
        ],
        "module_overrides": {
            "boundary_generation_enabled": False,
            "random_arrival_enabled": False,
            "random_vehicle_attributes_enabled": False,
            "ordinary_mainline_lane_change_enabled": False,
            "platoon_cmc_enabled": False,
            "mpc_lateral_tracking_enabled": False,
            "test_harness_overrides": {"source": "basic_region_gating"},
        },
        "preloaded_assignments": [],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": [],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [],
        "expected_png_features": [],
    }
    if preload_assignment:
        config["preloaded_assignments"] = [
            {
                "mv_id": "BASIC_MV",
                "clv_id": "BASIC_CLV",
                "cfv_id": "BASIC_CFV",
                "aps_case": "case_1",
                "col_clv": False,
                "col_cfv": False,
                "desired_spacing_override": None,
                "status": "valid",
                "created_at_t": -1.0,
                "created_at_step": -10,
                "source": "aps_cache",
                "valid_until_next_aps": True,
                "staleness_policy": "valid_until_next_aps",
            }
        ]
    return config


def _vehicle(
    vehicle_id: str,
    lane: str,
    x_global: float,
    y: float,
    *,
    road_role: str = "mainline",
    merge_state: str = "none",
    v: float = 20.0,
) -> dict:
    return {
        "vehicle_id": vehicle_id,
        "vehicle_type": "CAV",
        "compliance_state": "not_applicable",
        "initial_x_global": x_global,
        "initial_y": y,
        "initial_v": v,
        "initial_a": 0.0,
        "physical_lane": lane,
        "road_role": road_role,
        "lane_change_state": "normal",
        "merge_state": merge_state,
        "spec_overrides": {},
    }
