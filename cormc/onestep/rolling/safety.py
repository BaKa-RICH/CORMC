from __future__ import annotations

from cormc.simulation_core.pre_freeze import SimulationState, VehicleSpec

from cormc.onestep.rolling.state import SafetyCheckResult


def run_safety_check(
    state: SimulationState,
    *,
    ttc_threshold_s: float = 1.5,
    min_gap_m: float = 2.0,
) -> SafetyCheckResult:
    danger_ids: set[str] = set()
    danger_pairs: list[dict[str, object]] = []
    vehicles_by_lane: dict[str, list[str]] = {}
    for vehicle_id in state.active_vehicle_ids:
        vehicle_state = state.vehicle_states[vehicle_id]
        if not vehicle_state.is_active:
            continue
        vehicles_by_lane.setdefault(vehicle_state.physical_lane, []).append(vehicle_id)

    for lane_vehicle_ids in vehicles_by_lane.values():
        ordered = sorted(
            lane_vehicle_ids,
            key=lambda vehicle_id: (
                state.vehicle_states[vehicle_id].x_global,
                vehicle_id,
            ),
        )
        for rear_id, front_id in zip(ordered, ordered[1:]):
            rear = state.vehicle_states[rear_id]
            front = state.vehicle_states[front_id]
            rear_spec = state.vehicle_specs[rear_id]
            bumper_gap = front.x_global - rear.x_global - rear_spec.length
            closing_speed = rear.v - front.v
            ttc_s = (
                bumper_gap / closing_speed
                if closing_speed > 0.0
                else None
            )
            short_gap = bumper_gap <= min_gap_m
            unsafe_ttc = (
                ttc_s is not None
                and ttc_s < ttc_threshold_s
            )
            if unsafe_ttc:
                danger_ids.update((front_id, rear_id))
                danger_pairs.append(
                    {
                        "front_vehicle_id": front_id,
                        "rear_vehicle_id": rear_id,
                        "physical_lane": rear.physical_lane,
                        "bumper_gap_m": bumper_gap,
                        "closing_speed_mps": closing_speed,
                        "ttc_s": ttc_s,
                        "short_gap": short_gap,
                        "unsafe_ttc": unsafe_ttc,
                    }
                )

    return SafetyCheckResult(
        step=state.step,
        t=state.t,
        safety_alert=bool(danger_ids),
        danger_vehicle_ids=tuple(sorted(danger_ids)),
        danger_pairs=tuple(danger_pairs),
        ttc_threshold_s=ttc_threshold_s,
        min_gap_m=min_gap_m,
    )


def is_base_controllable(spec: VehicleSpec) -> bool:
    vehicle_type = spec.vehicle_type.lower()
    compliance_state = spec.compliance_state.lower()
    if vehicle_type == "cav":
        return True
    if vehicle_type == "chv" and compliance_state == "compliant":
        return True
    return False


def effective_controllable(
    vehicle_id: str,
    state: SimulationState,
    danger_vehicle_ids: set[str] | frozenset[str] | tuple[str, ...],
) -> bool:
    return (
        is_base_controllable(state.vehicle_specs[vehicle_id])
        and vehicle_id not in danger_vehicle_ids
    )
