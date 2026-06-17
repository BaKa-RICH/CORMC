from __future__ import annotations

from cormc.simulation_core.pre_freeze import (
    LongitudinalControllerMemory,
    ManeuverTrajectoryState,
    PreFreezeWorkspace,
    VehicleSpec,
    VehicleState,
    freeze_simulation_state,
)
from cormc.sumo.adapter import SumoRealizedStateAdapter, adapt_sumo_realized_to_state


def test_adapter_maps_realized_edge_lane_position_to_cormc_state_and_preserves_caches() -> None:
    previous = _state_with_cache()
    realized = {
        "MV_A": {
            "edge_id": "merge_zone",
            "lane_id": "merge_zone_0",
            "lane_position": 25.0,
            "x_global": 123.0,
            "y": -3.25,
            "v": 18.0,
        },
        "BG_1": {
            "edge_id": "main_pre",
            "lane_id": "main_pre_1",
            "lane_position": 100.0,
            "y": 3.5,
            "v": 30.0,
        },
    }

    state = adapt_sumo_realized_to_state(realized, previous_state=previous, step=11, t=1.1)

    assert state.step == 11
    assert state.t == 1.1
    assert state.active_vehicle_ids == ("MV_A", "BG_1")
    assert state.vehicle_states["MV_A"].x_global == 6975.0
    assert state.vehicle_states["MV_A"].y == -3.25
    assert state.vehicle_states["MV_A"].physical_lane == "on_ramp"
    assert state.vehicle_states["MV_A"].road_role == "on_ramp_mv"
    assert state.vehicle_states["MV_A"].merge_state == "executing"
    assert state.assignment_records_by_mv["MV_A"]["mv_id"] == "MV_A"
    assert "MV_A" in state.active_maneuvers
    assert "MV_A" in state.controller_memory_by_vehicle
    assert state.vehicle_specs["MV_A"].vehicle_type == "cav"
    assert state.vehicle_specs["BG_1"].vehicle_type == "sumo_background"
    assert state.vehicle_specs["BG_1"].source_lane_at_generation == "lane_1"


def test_adapter_accepts_traci_module_and_reads_y_from_get_position() -> None:
    traci = _FakeTraci()

    state = SumoRealizedStateAdapter().adapt(traci, step=2, t=0.2)

    assert state.active_vehicle_ids == ("MAIN",)
    assert state.vehicle_states["MAIN"].x_global == 10.0
    assert state.vehicle_states["MAIN"].y == 0.25
    assert state.vehicle_states["MAIN"].physical_lane == "lane_2"
    assert state.vehicle_specs["MAIN"].vehicle_type == "sumo_background"


def _state_with_cache():
    workspace = PreFreezeWorkspace(
        t=1.0,
        step=10,
        dt=0.1,
        active_vehicle_ids=["MV_A"],
        vehicle_states={
            "MV_A": VehicleState(
                vehicle_id="MV_A",
                x_global=6960.0,
                y=-3.5,
                v=16.0,
                a=0.0,
                physical_lane="on_ramp",
                road_role="on_ramp_mv",
                merge_state="executing",
            )
        },
        vehicle_specs={"MV_A": VehicleSpec("MV_A", "cav", "not_applicable")},
        assignment_records_by_mv={"MV_A": {"mv_id": "MV_A", "assigned_cfv_id": "CFV_A"}},
        active_maneuvers={
            "MV_A": ManeuverTrajectoryState(
                vehicle_id="MV_A",
                maneuver_type="merge",
                start_step=9,
                start_t=0.9,
                start_x_global=6950.0,
                start_y=-3.5,
                target_lane="lane_2",
                target_y=0.0,
            )
        },
        command_buffer={},
        next_state_buffer={},
        controller_memory_by_vehicle={"MV_A": LongitudinalControllerMemory("MV_A")},
    )
    return freeze_simulation_state(workspace)


class _FakeVehicleApi:
    def getIDList(self):
        return ["MAIN"]

    def getPosition(self, vehicle_id):
        assert vehicle_id == "MAIN"
        return 10.2, 0.25

    def getSpeed(self, vehicle_id):
        return 29.0

    def getRoadID(self, vehicle_id):
        return "main_pre"

    def getLaneID(self, vehicle_id):
        return "main_pre_0"

    def getLanePosition(self, vehicle_id):
        return 10.0


class _FakeTraci:
    vehicle = _FakeVehicleApi()
