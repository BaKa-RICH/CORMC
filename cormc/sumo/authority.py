from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Mapping

from cormc.step0_3 import SimulationState


POST_MERGE_HOLD_STEPS = 10


@dataclass(frozen=True)
class AuthorityDecision:
    vehicle_id: str
    active: bool
    reasons: tuple[str, ...]
    post_hold_remaining: int = 0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class AuthorityRegistrySnapshot:
    step: int
    active_vehicle_ids: tuple[str, ...]
    reasons_by_vehicle: Mapping[str, tuple[str, ...]]
    post_hold_by_vehicle: Mapping[str, int]
    decisions: tuple[AuthorityDecision, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class ActiveControlRegistry:
    post_merge_hold_steps: int = POST_MERGE_HOLD_STEPS
    post_hold_by_vehicle: dict[str, int] = field(default_factory=dict)
    released_merged_vehicle_ids: set[str] = field(default_factory=set)

    def update(
        self,
        state: SimulationState,
        command_buffer: Any | None = None,
    ) -> AuthorityRegistrySnapshot:
        decisions: list[AuthorityDecision] = []
        active_ids: list[str] = []
        reasons_by_vehicle: dict[str, tuple[str, ...]] = {}
        next_hold: dict[str, int] = {}

        command_vehicle_ids = _command_vehicle_ids(command_buffer)
        for vehicle_id in state.active_vehicle_ids:
            vehicle_state = state.vehicle_states[vehicle_id]
            spec = state.vehicle_specs.get(vehicle_id)
            reasons: list[str] = []

            road_role = str(vehicle_state.road_role).lower()
            merge_state = str(vehicle_state.merge_state).lower()
            lane_change_state = str(vehicle_state.lane_change_state).lower()
            vehicle_type = str(getattr(spec, "vehicle_type", "sumo_background")).lower()
            compliance_state = str(getattr(spec, "compliance_state", "not_applicable")).lower()

            if road_role in {"on_ramp", "on_ramp_mv"} and merge_state != "merged":
                reasons.append("mv_on_ramp_until_merged")
            if merge_state in {"waiting", "executing"}:
                reasons.append(f"merge_state_{merge_state}")
            if lane_change_state in {"waiting", "executing"}:
                reasons.append(f"lane_change_state_{lane_change_state}")

            if vehicle_id in command_vehicle_ids:
                reasons.extend(command_vehicle_ids[vehicle_id])
            if vehicle_id in state.active_maneuvers:
                reasons.append("active_maneuver")
            if vehicle_type == "chv" and compliance_state == "compliant":
                reasons.append("compliant_chv")
            if vehicle_type == "chv" and compliance_state == "non_compliant":
                reasons = [
                    reason
                    for reason in reasons
                    if reason
                    in {
                        "mv_on_ramp_until_merged",
                        "merge_state_waiting",
                        "merge_state_executing",
                        "lane_change_state_waiting",
                        "lane_change_state_executing",
                    }
                ]

            previous_hold = int(self.post_hold_by_vehicle.get(vehicle_id, 0))
            if merge_state != "merged":
                self.released_merged_vehicle_ids.discard(vehicle_id)
            if merge_state == "merged":
                if (
                    previous_hold <= 0
                    and road_role in {"on_ramp", "on_ramp_mv"}
                    and vehicle_id not in self.released_merged_vehicle_ids
                ):
                    previous_hold = self.post_merge_hold_steps
                if previous_hold > 0:
                    reasons.append("post_merge_hold")
                    remaining = previous_hold - 1
                    if remaining > 0:
                        next_hold[vehicle_id] = remaining
                    else:
                        self.released_merged_vehicle_ids.add(vehicle_id)

            unique_reasons = tuple(dict.fromkeys(reasons))
            active = bool(unique_reasons)
            if active:
                active_ids.append(vehicle_id)
                reasons_by_vehicle[vehicle_id] = unique_reasons
            decisions.append(
                AuthorityDecision(
                    vehicle_id=vehicle_id,
                    active=active,
                    reasons=unique_reasons,
                    post_hold_remaining=next_hold.get(vehicle_id, 0),
                )
            )

        self.post_hold_by_vehicle = {vehicle_id: hold for vehicle_id, hold in next_hold.items() if hold > 0}
        return AuthorityRegistrySnapshot(
            step=state.step,
            active_vehicle_ids=tuple(active_ids),
            reasons_by_vehicle=reasons_by_vehicle,
            post_hold_by_vehicle=dict(self.post_hold_by_vehicle),
            decisions=tuple(decisions),
        )


def resolve_active_authority(
    state: SimulationState,
    command_buffer: Any | None = None,
    *,
    registry: ActiveControlRegistry | None = None,
) -> AuthorityRegistrySnapshot:
    active_registry = registry or ActiveControlRegistry()
    return active_registry.update(state, command_buffer)


def _command_vehicle_ids(command_buffer: Any | None) -> dict[str, list[str]]:
    if command_buffer is None:
        return {}
    fields = {
        "cooperation_commands": "cooperation_command",
        "speed_cap_commands": "speed_cap_command",
        "lane_change_commands": "lane_change_command",
        "merge_commands": "merge_command",
    }
    result: dict[str, list[str]] = {}
    for field_name, reason in fields.items():
        value = getattr(command_buffer, field_name, None)
        if value is None and isinstance(command_buffer, Mapping):
            value = command_buffer.get(field_name)
        if not value:
            continue
        if isinstance(value, Mapping):
            vehicle_ids = value.keys()
        else:
            vehicle_ids = []
        for vehicle_id in vehicle_ids:
            result.setdefault(str(vehicle_id), []).append(reason)
    return result
