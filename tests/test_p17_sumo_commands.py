from __future__ import annotations

import json
from dataclasses import asdict

from cormc.sumo.commands import (
    ControlledTrajectoryCommand,
    P17SumoArtifactResult,
    RealizationRecord,
    SpawnCommand,
    SumoSimulationResult,
    SumoTrajectoryAuthorityConfig,
)


def test_p17_controlled_trajectory_command_is_json_serializable() -> None:
    command = ControlledTrajectoryCommand(
        vehicle_id="mv",
        step=1,
        t=0.0,
        target_t=0.1,
        x_global=6950.0,
        y=-3.5,
        v=20.0,
        a=0.2,
        physical_lane="on_ramp",
        road_role="on_ramp",
        authority_reason="merge_executing",
    )

    payload = asdict(command)

    assert payload["authority_mode"] == "trajectory_authority"
    assert json.loads(json.dumps(payload))["vehicle_id"] == "mv"
    assert command.to_dict() == payload


def test_p17_spawn_command_is_neutral_json_serializable() -> None:
    command = SpawnCommand(
        vehicle_id="bg0",
        step=0,
        t=0.0,
        route_id="route_main",
        edge_id="main_pre",
        lane_index=0,
        depart_pos=10.0,
        x_global=10.0,
        y=0.0,
        v=22.0,
        vehicle_type="sumo_background",
        compliance_state="background",
        source_queue_seed=17,
        source_profile_id="p16_internal_demo_v1",
        source_spawn_reason="seeded_arrival",
    )

    assert json.loads(json.dumps(asdict(command)))["route_id"] == "route_main"


def test_p17_result_dataclasses_are_json_serializable() -> None:
    record = RealizationRecord(
        vehicle_id="mv",
        step=1,
        t=0.1,
        command_x_global=6951.0,
        command_y=-3.4,
        command_v=20.0,
        command_a=0.0,
        realized_x_global=6951.0,
        realized_y=-3.4,
        realized_v=20.0,
        realized_edge_id="merge_zone",
        realized_lane_id="merge_zone_0",
        realized_lane_position=1.0,
        dx_abs=0.0,
        dy_abs=0.0,
        dv_abs=0.0,
        result="matched",
    )
    result = SumoSimulationResult(
        run_id="p17",
        status="passed",
        sumo_version="SUMO test",
        net_file="p17.net.xml",
        route_file="p17.routes.xml",
        sumocfg_file="p17.sumocfg",
        steps=1,
        realization_records=(record,),
    )
    artifact = P17SumoArtifactResult(
        run_id="p17",
        output_dir="out",
        network_files={"net": "p17.net.xml"},
        simulation_result=result,
    )
    config = SumoTrajectoryAuthorityConfig()

    assert config.step_length == 0.1
    assert config.mismatch_x_tolerance_m == 0.75
    assert config.mismatch_y_tolerance_m == 0.35
    assert config.mismatch_v_tolerance_mps == 1.0
    assert json.loads(json.dumps(asdict(artifact)))["simulation_result"]["status"] == "passed"
