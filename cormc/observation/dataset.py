from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence


@dataclass(frozen=True)
class ObservationTrajectoryRecord:
    scenario_id: str
    run_id: str
    step: int
    t: float
    vehicle_id: str
    vehicle_type: str
    compliance_state: str
    x_global: float
    y: float
    v: float
    a: float
    physical_lane: str
    road_role: str
    primary_leader_id: str
    lane_change_state: str
    merge_state: str
    active_event_tags: tuple[str, ...]

    def to_csv_row(self) -> dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "run_id": self.run_id,
            "step": self.step,
            "t": self.t,
            "vehicle_id": self.vehicle_id,
            "vehicle_type": self.vehicle_type,
            "compliance_state": self.compliance_state,
            "x_global": self.x_global,
            "y": self.y,
            "v": self.v,
            "a": self.a,
            "physical_lane": self.physical_lane,
            "road_role": self.road_role,
            "primary_leader_id": self.primary_leader_id,
            "lane_change_state": self.lane_change_state,
            "merge_state": self.merge_state,
            "active_event_tags": "|".join(self.active_event_tags),
        }


@dataclass(frozen=True)
class ObservationLifecycle:
    mv_id: str
    first_trigger_step: int | None
    locked_gap_step: int | None
    lateral_start_step: int | None
    lateral_completed_step: int | None
    mainline_conversion_step: int | None
    final_physical_lane: str
    final_road_role: str
    final_merge_state: str


@dataclass(frozen=True)
class ObservationDataset:
    scenario_id: str
    run_id: str
    source_dir: str
    summary: Mapping[str, Any]
    trajectory_records: tuple[ObservationTrajectoryRecord, ...]
    gap_rows: tuple[Mapping[str, Any], ...]
    mv_ids: tuple[str, ...]
    lifecycles: Mapping[str, ObservationLifecycle]
    artifact_paths: Mapping[str, str]

    @property
    def source_path(self) -> Path:
        return Path(self.source_dir)

    def vehicle_ids(self) -> tuple[str, ...]:
        seen: set[str] = set()
        ordered: list[str] = []
        for record in self.trajectory_records:
            if record.vehicle_id in seen:
                continue
            seen.add(record.vehicle_id)
            ordered.append(record.vehicle_id)
        return tuple(ordered)

    def records_for(self, vehicle_id: str) -> tuple[ObservationTrajectoryRecord, ...]:
        return tuple(
            sorted(
                (record for record in self.trajectory_records if record.vehicle_id == vehicle_id),
                key=lambda record: (record.step, record.t),
            )
        )

    def step_range(self) -> tuple[int, int]:
        if not self.trajectory_records:
            raise ValueError("ObservationDataset has no trajectory records")
        steps = [record.step for record in self.trajectory_records]
        return min(steps), max(steps)

    def t_range(self) -> tuple[float, float]:
        if not self.trajectory_records:
            raise ValueError("ObservationDataset has no trajectory records")
        t_values = [record.t for record in self.trajectory_records]
        return min(t_values), max(t_values)


TRAJECTORY_FIELDNAMES: tuple[str, ...] = (
    "scenario_id",
    "run_id",
    "step",
    "t",
    "vehicle_id",
    "vehicle_type",
    "compliance_state",
    "x_global",
    "y",
    "v",
    "a",
    "physical_lane",
    "road_role",
    "primary_leader_id",
    "lane_change_state",
    "merge_state",
    "active_event_tags",
)


def ordered_records(records: Sequence[ObservationTrajectoryRecord]) -> tuple[ObservationTrajectoryRecord, ...]:
    return tuple(sorted(records, key=lambda record: (record.step, record.vehicle_id, record.t)))
