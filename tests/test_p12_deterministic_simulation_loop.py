from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from cormc.scenes import BASIC_SCENARIO_IDS
from cormc.simulation_core.loop import (
    SimulationLoopConfig,
    build_step_command_buffer,
    run_deterministic_simulation,
)
from cormc.simulation_core.commit import CommandBuffer


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
    assert result.final_state.assignment_records_by_mv == {}
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


def test_p12_png_created_nonblank_and_contains_feature_evidence(tmp_path: Path) -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id=BASIC_SCENARIO_IDS[0],
            run_id="p12-png",
            max_steps=3,
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
        "commit_marker",
    }.issubset(features)


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
