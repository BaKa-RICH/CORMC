from __future__ import annotations

from cormc.simulation_core.pre_freeze import LANE_2, SimulationState

from cormc.onestep.rolling.safety import effective_controllable
from cormc.onestep.rolling.state import (
    EFFECTIVE_CONTROL_BOTH,
    EFFECTIVE_CONTROL_FRONT,
    EFFECTIVE_CONTROL_NONE,
    EFFECTIVE_CONTROL_REAR,
    GapCandidate,
    GapSnapshot,
)


def identify_and_number_gaps(
    state: SimulationState,
    danger_vehicle_ids: set[str] | frozenset[str] | tuple[str, ...],
) -> GapSnapshot:
    danger_set = set(danger_vehicle_ids)
    lane_2_vehicle_ids = sorted(
        (
            vehicle_id
            for vehicle_id in state.active_vehicle_ids
            if state.vehicle_states[vehicle_id].is_active
            and state.vehicle_states[vehicle_id].physical_lane == LANE_2
        ),
        key=lambda vehicle_id: (
            -state.vehicle_states[vehicle_id].x_global,
            vehicle_id,
        ),
    )
    gaps: list[GapCandidate] = []
    for index, (front_id, rear_id) in enumerate(
        zip(lane_2_vehicle_ids, lane_2_vehicle_ids[1:]),
        start=1,
    ):
        front_controllable = effective_controllable(front_id, state, danger_set)
        rear_controllable = effective_controllable(rear_id, state, danger_set)
        if front_controllable and rear_controllable:
            control_type = EFFECTIVE_CONTROL_BOTH
        elif front_controllable:
            control_type = EFFECTIVE_CONTROL_FRONT
        elif rear_controllable:
            control_type = EFFECTIVE_CONTROL_REAR
        else:
            control_type = EFFECTIVE_CONTROL_NONE
        front = state.vehicle_states[front_id]
        rear = state.vehicle_states[rear_id]
        rear_spec = state.vehicle_specs[rear_id]
        gaps.append(
            GapCandidate(
                gap_id=f"gap:{state.step}:{index}",
                index=index,
                front_vehicle_id=front_id,
                rear_vehicle_id=rear_id,
                front_x_global=front.x_global,
                rear_x_global=rear.x_global,
                bumper_gap_m=front.x_global - rear.x_global - rear_spec.length,
                effective_control_type=control_type,
            )
        )
    return GapSnapshot(
        step=state.step,
        t=state.t,
        lane_id=LANE_2,
        gaps=tuple(gaps),
        danger_vehicle_ids=tuple(sorted(danger_set)),
    )
