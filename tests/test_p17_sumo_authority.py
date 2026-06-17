from __future__ import annotations

from dataclasses import dataclass, field

from cormc.simulation_core.pre_freeze import (
    ManeuverTrajectoryState,
    PreFreezeWorkspace,
    VehicleSpec,
    VehicleState,
    freeze_simulation_state,
)
from cormc.sumo.authority import ActiveControlRegistry, resolve_active_authority


@dataclass(frozen=True)
class _CommandBuffer:
    cooperation_commands: dict = field(default_factory=dict)
    speed_cap_commands: dict = field(default_factory=dict)
    lane_change_commands: dict = field(default_factory=dict)
    merge_commands: dict = field(default_factory=dict)


def test_authority_marks_mv_waiting_executing_and_post_merge_hold_then_releases() -> None:
    registry = ActiveControlRegistry(post_merge_hold_steps=10)
    waiting = _state([_vehicle("MV", "cav", "not_applicable", "on_ramp", "on_ramp_mv", "waiting")])

    first = registry.update(waiting)

    assert first.active_vehicle_ids == ("MV",)
    assert "mv_on_ramp_until_merged" in first.reasons_by_vehicle["MV"]
    assert "merge_state_waiting" in first.reasons_by_vehicle["MV"]

    merged = _state([_vehicle("MV", "cav", "not_applicable", "on_ramp", "on_ramp_mv", "merged")])
    hold_steps = [registry.update(merged).active_vehicle_ids for _ in range(10)]
    released = registry.update(merged)

    assert hold_steps[0] == ("MV",)
    assert hold_steps[-1] == ("MV",)
    assert released.active_vehicle_ids == ()
    assert registry.post_hold_by_vehicle == {}


def test_authority_uses_commands_active_maneuvers_and_chv_compliance_rules() -> None:
    state = _state(
        [
            _vehicle("CV", "cav", "not_applicable", "lane_2", "mainline", "none"),
            _vehicle("CLV", "cav", "not_applicable", "lane_1", "mainline", "none"),
            _vehicle("CFV", "cav", "not_applicable", "lane_2", "mainline", "none"),
            _vehicle("CHV_OK", "chv", "compliant", "lane_2", "mainline", "none"),
            _vehicle("CHV_NO", "chv", "non_compliant", "lane_2", "mainline", "none"),
        ],
        active_maneuver_ids=("CFV",),
    )
    commands = _CommandBuffer(
        cooperation_commands={"CV": ({"command_id": "cooperate"},)},
        speed_cap_commands={"CLV": ({"command_id": "cap"},)},
        lane_change_commands={"CFV": {"command_id": "lc"}},
        merge_commands={},
    )

    snapshot = resolve_active_authority(state, commands)

    assert set(snapshot.active_vehicle_ids) == {"CV", "CLV", "CFV", "CHV_OK"}
    assert snapshot.reasons_by_vehicle["CV"] == ("cooperation_command",)
    assert "speed_cap_command" in snapshot.reasons_by_vehicle["CLV"]
    assert "lane_change_command" in snapshot.reasons_by_vehicle["CFV"]
    assert "active_maneuver" in snapshot.reasons_by_vehicle["CFV"]
    assert snapshot.reasons_by_vehicle["CHV_OK"] == ("compliant_chv",)
    assert "CHV_NO" not in snapshot.reasons_by_vehicle


def _state(vehicles, *, active_maneuver_ids=()):
    states = {state.vehicle_id: state for state, _ in vehicles}
    specs = {state.vehicle_id: spec for state, spec in vehicles}
    maneuvers = {
        vehicle_id: ManeuverTrajectoryState(
            vehicle_id=vehicle_id,
            maneuver_type="lane_change",
            start_step=0,
            start_t=0.0,
            start_x_global=100.0,
            start_y=0.0,
            target_lane="lane_1",
            target_y=3.5,
        )
        for vehicle_id in active_maneuver_ids
    }
    return freeze_simulation_state(
        PreFreezeWorkspace(
            t=0.0,
            step=0,
            dt=0.1,
            active_vehicle_ids=list(states),
            vehicle_states=states,
            vehicle_specs=specs,
            assignment_records_by_mv={},
            active_maneuvers=maneuvers,
            command_buffer={},
            next_state_buffer={},
        )
    )


def _vehicle(vehicle_id, vehicle_type, compliance, lane, road_role, merge_state):
    state = VehicleState(
        vehicle_id=vehicle_id,
        x_global=100.0,
        y=0.0 if lane != "lane_1" else 3.5,
        v=25.0,
        a=0.0,
        physical_lane=lane,
        road_role=road_role,
        lane_change_state="normal",
        merge_state=merge_state,
    )
    return state, VehicleSpec(vehicle_id, vehicle_type, compliance)
