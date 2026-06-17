from __future__ import annotations

from pathlib import Path

from cormc.simulation_core.engine import build_initial_state_from_scenario_config
from cormc.legacy.rolling_basic_numeric import (
    _rolling_basic_complete_condition,
    run_rolling_basic_numeric_scenario,
)
from cormc.scenes import (
    ROLLING_BASIC_MAINLINE_VEHICLE_IDS,
    ROLLING_BASIC_MV_IDS,
    ROLLING_BASIC_SCENARIO_ID,
    get_rolling_basic_expectations,
    load_scene_config,
)
from cormc.simulation_core.pre_freeze import VehicleState


def test_rolling_basic_config_matches_document_vehicle_table() -> None:
    config = load_scene_config(ROLLING_BASIC_SCENARIO_ID)

    expected = [
        ("RB01_MV", "on_ramp", "on_ramp_mv", 6750.0, -3.5, 20.0),
        ("RB01_CLV", "lane_2", "mainline", 6784.0, 0.0, 20.0),
        ("RB01_CFV", "lane_2", "mainline", 6744.0, 0.0, 20.0),
        ("RB01_TLV_CFV", "lane_1", "mainline", 6753.0, 3.5, 15.0),
        ("RB02_MV", "on_ramp", "on_ramp_mv", 6640.0, -3.5, 20.0),
        ("RB02_CLV", "lane_2", "mainline", 6674.0, 0.0, 20.0),
        ("RB02_CFV", "lane_2", "mainline", 6634.0, 0.0, 20.0),
        ("RB02_TLV_CFV", "lane_1", "mainline", 6643.0, 3.5, 15.0),
        ("RB03_MV", "on_ramp", "on_ramp_mv", 6540.0, -3.5, 20.0),
        ("RB03_CLV", "lane_2", "mainline", 6554.0, 0.0, 20.0),
        ("RB03_CFV", "lane_2", "mainline", 6514.0, 0.0, 20.0),
        ("RB03_TLV_CLV", "lane_1", "mainline", 6563.0, 3.5, 15.0),
        ("RB04_MV", "on_ramp", "on_ramp_mv", 6440.0, -3.5, 20.0),
        ("RB04_CLV", "lane_2", "mainline", 6454.0, 0.0, 20.0),
        ("RB04_CFV", "lane_2", "mainline", 6434.0, 0.0, 20.0),
        ("RB04_TLV_CFV", "lane_1", "mainline", 6443.0, 3.5, 15.0),
        ("RB04_TLV_CLV", "lane_1", "mainline", 6463.0, 3.5, 15.0),
    ]
    actual = [
        (
            vehicle["vehicle_id"],
            vehicle["physical_lane"],
            vehicle["road_role"],
            vehicle["initial_x_global"],
            vehicle["initial_y"],
            vehicle["initial_v"],
        )
        for vehicle in config["initial_vehicles"]
    ]

    assert config["scenario_id"] == ROLLING_BASIC_SCENARIO_ID
    assert actual == expected
    assert len(config["initial_vehicles"]) == 17
    assert [item[0] for item in actual if item[2] == "on_ramp_mv"] == list(ROLLING_BASIC_MV_IDS)
    assert [item[2] for item in actual if item[0] not in ROLLING_BASIC_MV_IDS] == ["mainline"] * 13


def test_rolling_basic_config_pins_mainline_cuc_utility_as_diagnostic_stopgap() -> None:
    config = load_scene_config(ROLLING_BASIC_SCENARIO_ID)
    module_overrides = config["module_overrides"]
    harness = module_overrides["test_harness_overrides"]
    overrides = harness["cuc_utility_overrides"]

    assert module_overrides["boundary_generation_enabled"] is False
    assert module_overrides["random_arrival_enabled"] is False
    assert module_overrides["random_vehicle_attributes_enabled"] is False
    assert module_overrides["ordinary_mainline_lane_change_enabled"] is False
    assert module_overrides["platoon_cmc_enabled"] is False
    assert module_overrides["mpc_lateral_tracking_enabled"] is False
    assert set(overrides) == set(ROLLING_BASIC_MAINLINE_VEHICLE_IDS)
    for vehicle_id in ROLLING_BASIC_MAINLINE_VEHICLE_IDS:
        assert overrides[vehicle_id] == {
            "recommended_choice": "stay_lane_2",
            "U1": 0.0,
            "U2": 10000.0,
        }
    assert "freeze_first_aps_assignment_until_cmc" not in harness
    assert config["preloaded_assignments"] == []
    assert config["preloaded_state_machine_states"] == []
    assert config["preloaded_maneuver_trajectory_states"] == []


def test_rolling_basic_expectations_match_case_matrix() -> None:
    expectations = get_rolling_basic_expectations()

    assert expectations["RB01_MV"].expected_aps_case == "case_2"
    assert expectations["RB01_MV"].expected_active_cv_ids == ("RB01_CFV",)
    assert expectations["RB01_MV"].expected_eq10_consumer_ids == ("RB01_CFV",)
    assert expectations["RB02_MV"].expected_aps_case == "case_2"
    assert expectations["RB02_MV"].expected_active_cv_ids == ("RB02_CFV",)
    assert expectations["RB02_MV"].expected_eq10_consumer_ids == ("RB02_CFV",)
    assert expectations["RB03_MV"].expected_aps_case == "case_3"
    assert expectations["RB03_MV"].expected_active_cv_ids == ("RB03_CLV",)
    assert expectations["RB03_MV"].expected_eq10_consumer_ids == ()
    assert expectations["RB04_MV"].expected_aps_case == "case_4"
    assert expectations["RB04_MV"].expected_active_cv_ids == ("RB04_CLV", "RB04_CFV")
    assert expectations["RB04_MV"].expected_eq10_consumer_ids == ("RB04_CFV",)


def test_rolling_basic_smoke_run_writes_artifacts_and_summary(tmp_path: Path) -> None:
    result = run_rolling_basic_numeric_scenario(
        output_dir=tmp_path,
        run_id="rolling-smoke",
        max_steps=20,
        render_png=False,
    )
    summary = result.numeric_summary

    assert summary["scenario_id"] == ROLLING_BASIC_SCENARIO_ID
    assert summary["actual_steps"] == 20
    assert set(summary["mv_summaries"]) == set(ROLLING_BASIC_MV_IDS)
    assert summary["mv_summaries"]["RB01_MV"]["first_control_zone_step"] == 0
    rb01_first_aps = summary["mv_summaries"]["RB01_MV"]["first_aps"]
    assert rb01_first_aps is not None or any(
        finding["bug_id"] == "RB-APS-001" and "RB01_MV" in finding["affected_mv_ids"]
        for finding in summary["bug_findings"]
    )
    assert isinstance(summary["cross_mv_summary"]["shared_cv_timeline"], list)
    assert "per_step_active_assignments_by_mv" in summary["cross_mv_summary"]
    assert isinstance(summary["bug_findings"], list)
    assert all(finding["bug_id"] != "RB-CUC-001" for finding in summary["bug_findings"])
    for mv_summary in summary["mv_summaries"].values():
        assert all(
            item["final_choice"] in {"stay_lane_2", None}
            for item in mv_summary["cuc_choice_timeline"]
        )
    for finding in summary["bug_findings"]:
        assert finding["bug_id"]
        assert finding["category"]
        assert finding["status"] in {"observed", "inferred_from_data", "not_observed"}
        assert finding["evidence_refs"]

    artifact_paths = summary["artifact_paths"]
    assert Path(artifact_paths["trajectory"]).exists()
    assert Path(artifact_paths["events"]).exists()
    assert Path(artifact_paths["sanity"]).exists()
    assert Path(result.numeric_summary_path).exists()
    assert Path(result.scenario_report_path).exists()
    assert Path(result.artifact_manifest_path).exists()


def test_rolling_basic_independent_assignments_merge_without_step5_conflicts(tmp_path: Path) -> None:
    result = run_rolling_basic_numeric_scenario(
        output_dir=tmp_path,
        run_id="rolling-independent-assignment",
        max_steps=1200,
        render_png=False,
    )
    summary = result.numeric_summary
    expectations = get_rolling_basic_expectations()

    assert summary["status"] == "passed"
    assert summary["bug_findings"] == []
    cross = summary["cross_mv_summary"]
    assert cross["shared_cv_timeline"] == []
    assert cross["step5_conflict_resolution_timeline"] == []

    for mv_id, expectation in expectations.items():
        mv_summary = summary["mv_summaries"][mv_id]
        assert mv_summary["observed_clv_id"] == expectation.expected_clv_id
        assert mv_summary["observed_cfv_id"] == expectation.expected_cfv_id
        assert mv_summary["bounded_assignment_merge_success"] is True
        assert mv_summary["used_front_only_recovery_for_success"] is False
        assert mv_summary["merge_success_clv_id"] == expectation.expected_clv_id
        assert mv_summary["merge_success_cfv_id"] == expectation.expected_cfv_id
        assert mv_summary["merged_and_past_ramp"] is True


def test_rolling_basic_stop_condition_requires_all_mvs_past_ramp() -> None:
    config = load_scene_config(ROLLING_BASIC_SCENARIO_ID)
    state = build_initial_state_from_scenario_config(config)
    condition = _rolling_basic_complete_condition()

    assert condition(state) is False
    partial = _with_mv_positions(
        state,
        {
            "RB01_MV": (7260.0, "merged"),
            "RB02_MV": (7260.0, "merged"),
            "RB03_MV": (7260.0, "merged"),
            "RB04_MV": (7240.0, "executing"),
        },
    )
    assert condition(partial) is False
    complete = _with_mv_positions(
        state,
        {
            "RB01_MV": (7260.0, "merged"),
            "RB02_MV": (7260.0, "merged"),
            "RB03_MV": (7260.0, "merged"),
            "RB04_MV": (7260.0, "merged"),
        },
    )
    assert condition(complete) is True


def _with_mv_positions(state, updates: dict[str, tuple[float, str]]):
    vehicle_states = dict(state.vehicle_states)
    for mv_id, (x_global, merge_state) in updates.items():
        old = vehicle_states[mv_id]
        vehicle_states[mv_id] = VehicleState(
            vehicle_id=old.vehicle_id,
            x_global=x_global,
            y=old.y,
            v=old.v,
            a=old.a,
            physical_lane=old.physical_lane,
            road_role=old.road_role,
            lane_change_state=old.lane_change_state,
            merge_state=merge_state,
            is_active=old.is_active,
        )
    return type(state)(
        t=state.t,
        step=state.step,
        dt=state.dt,
        active_vehicle_ids=state.active_vehicle_ids,
        vehicle_states=vehicle_states,
        vehicle_specs=state.vehicle_specs,
        assignment_records_by_mv=state.assignment_records_by_mv,
        active_maneuvers=state.active_maneuvers,
        road_config_ref=state.road_config_ref,
        parameter_config_ref=state.parameter_config_ref,
        scenario_config_ref=state.scenario_config_ref,
        output_config_ref=state.output_config_ref,
        controller_memory_by_vehicle=state.controller_memory_by_vehicle,
    )
