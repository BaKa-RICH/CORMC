from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Mapping

from cormc.observation.dataset import (
    TRAJECTORY_FIELDNAMES,
    ObservationDataset,
    ObservationLifecycle,
    ObservationTrajectoryRecord,
    ordered_records,
)


STAGE2_SUMMARY_TOP_LEVEL_KEYS: frozenset[str] = frozenset(
    {
        "scenario_summary",
        "round_summaries",
        "mv_summaries",
        "cross_mv_summary",
        "artifact_paths",
    }
)


def load_stage2_observation_dataset(source_dir: str | Path) -> ObservationDataset:
    source_path = Path(source_dir)
    summary_path = source_path / "stage2_summary.json"
    trajectory_path = source_path / "trajectory.csv"
    gap_rows_path = source_path / "gap_rows.json"

    missing = [
        str(path)
        for path in (summary_path, trajectory_path, gap_rows_path)
        if not path.exists()
    ]
    if missing:
        raise FileNotFoundError(f"stage2 observation source is missing required files: {missing}")

    summary = _read_json_object(summary_path)
    _validate_summary_top_level(summary, summary_path)
    scenario = _required_mapping(summary, "scenario_summary", summary_path)
    mv_summaries = _required_mapping(summary, "mv_summaries", summary_path)
    artifact_paths = _required_mapping(summary, "artifact_paths", summary_path)

    scenario_id = str(_required_value(scenario, "scenario_id", summary_path))
    run_id = str(_required_value(scenario, "run_id", summary_path))
    mv_ids = tuple(str(item) for item in _required_list(scenario, "mv_ids", summary_path))
    if set(mv_ids) != set(str(key) for key in mv_summaries):
        raise ValueError(
            f"{summary_path} scenario_summary.mv_ids must match mv_summaries keys; "
            f"mv_ids={mv_ids}, mv_summaries={sorted(mv_summaries)}"
        )

    records = _read_trajectory_records(
        trajectory_path,
        expected_scenario_id=scenario_id,
        expected_run_id=run_id,
    )
    if not records:
        raise ValueError(f"{trajectory_path} must contain at least one trajectory record")

    gap_rows = tuple(_read_json_list(gap_rows_path))
    expected_gap_count = sum(len(_required_list(dict(mv), "gap_rows", summary_path)) for mv in mv_summaries.values())
    if len(gap_rows) != expected_gap_count:
        raise ValueError(
            f"{gap_rows_path} row count {len(gap_rows)} does not match "
            f"summary mv_summaries gap row total {expected_gap_count}"
        )

    lifecycles = {
        mv_id: _read_lifecycle(mv_id, _required_mapping(dict(mv_summaries[mv_id]), "lifecycle", summary_path))
        for mv_id in mv_ids
    }

    return ObservationDataset(
        scenario_id=scenario_id,
        run_id=run_id,
        source_dir=str(source_path),
        summary=summary,
        trajectory_records=ordered_records(records),
        gap_rows=gap_rows,
        mv_ids=mv_ids,
        lifecycles=lifecycles,
        artifact_paths={str(key): str(value) for key, value in artifact_paths.items()},
    )


def _read_trajectory_records(
    path: Path,
    *,
    expected_scenario_id: str,
    expected_run_id: str,
) -> tuple[ObservationTrajectoryRecord, ...]:
    records: list[ObservationTrajectoryRecord] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != TRAJECTORY_FIELDNAMES:
            raise ValueError(
                f"{path} trajectory CSV fields must be exactly {list(TRAJECTORY_FIELDNAMES)}, "
                f"got {reader.fieldnames}"
            )
        for row_number, row in enumerate(reader, start=2):
            scenario_id = str(row["scenario_id"])
            run_id = str(row["run_id"])
            if scenario_id != expected_scenario_id or run_id != expected_run_id:
                raise ValueError(
                    f"{path}:{row_number} has scenario/run {(scenario_id, run_id)}, "
                    f"expected {(expected_scenario_id, expected_run_id)}"
                )
            records.append(
                ObservationTrajectoryRecord(
                    scenario_id=scenario_id,
                    run_id=run_id,
                    step=int(row["step"]),
                    t=float(row["t"]),
                    vehicle_id=str(row["vehicle_id"]),
                    vehicle_type=str(row["vehicle_type"]),
                    compliance_state=str(row["compliance_state"]),
                    x_global=float(row["x_global"]),
                    y=float(row["y"]),
                    v=float(row["v"]),
                    a=float(row["a"]),
                    physical_lane=str(row["physical_lane"]),
                    road_role=str(row["road_role"]),
                    primary_leader_id=str(row["primary_leader_id"]),
                    lane_change_state=str(row["lane_change_state"]),
                    merge_state=str(row["merge_state"]),
                    active_event_tags=tuple(
                        tag for tag in str(row["active_event_tags"]).split("|") if tag
                    ),
                )
            )
    return tuple(records)


def _read_lifecycle(mv_id: str, payload: Mapping[str, Any]) -> ObservationLifecycle:
    final_status = _required_mapping(payload, "final_status", Path("stage2_summary.json"))
    return ObservationLifecycle(
        mv_id=mv_id,
        first_trigger_step=_optional_int(payload.get("first_trigger_step")),
        locked_gap_step=_optional_int(payload.get("locked_gap_step")),
        lateral_start_step=_optional_int(payload.get("lateral_start_step")),
        lateral_completed_step=_optional_int(payload.get("lateral_completed_step")),
        mainline_conversion_step=_optional_int(payload.get("mainline_conversion_step")),
        final_physical_lane=str(_required_value(final_status, "physical_lane", Path("stage2_summary.json"))),
        final_road_role=str(_required_value(final_status, "road_role", Path("stage2_summary.json"))),
        final_merge_state=str(_required_value(final_status, "merge_state", Path("stage2_summary.json"))),
    )


def _validate_summary_top_level(summary: Mapping[str, Any], path: Path) -> None:
    keys = frozenset(summary)
    if keys != STAGE2_SUMMARY_TOP_LEVEL_KEYS:
        raise ValueError(
            f"{path} top-level keys must be exactly {sorted(STAGE2_SUMMARY_TOP_LEVEL_KEYS)}, "
            f"got {sorted(keys)}"
        )


def _read_json_object(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload


def _read_json_list(path: Path) -> list[Mapping[str, Any]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, list):
        raise ValueError(f"{path} must contain a JSON array")
    if not all(isinstance(item, dict) for item in payload):
        raise ValueError(f"{path} must contain an array of JSON objects")
    return payload


def _required_mapping(payload: Mapping[str, Any], key: str, path: Path) -> Mapping[str, Any]:
    value = payload.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"{path} field {key!r} must be a JSON object")
    return value


def _required_list(payload: Mapping[str, Any], key: str, path: Path) -> list[Any]:
    value = payload.get(key)
    if not isinstance(value, list):
        raise ValueError(f"{path} field {key!r} must be a JSON array")
    return value


def _required_value(payload: Mapping[str, Any], key: str, path: Path) -> Any:
    if key not in payload:
        raise ValueError(f"{path} missing required field {key!r}")
    return payload[key]


def _optional_int(value: Any) -> int | None:
    if value is None:
        return None
    return int(value)
