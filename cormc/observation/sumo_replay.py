from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping

from cormc.observation.dataset import ObservationDataset, ObservationTrajectoryRecord
from cormc.sumo.mapping import (
    EDGE_METADATA,
    MAINLINE_END_X,
    MERGE_END_X,
    RAMP_UPSTREAM_START_X,
    to_sumo_position,
)
from cormc.sumo.network import SumoNetworkConfig, build_sumo_network
from cormc.sumo.trajectory_gui_replay import run_trajectory_gui_replay


OBSERVATION_ROLE_COLORS: dict[str, tuple[int, int, int]] = {
    "mv": (210, 55, 35),
    "selected_front": (0, 120, 180),
    "selected_rear": (230, 140, 20),
    "mainline_context": (120, 130, 140),
    "context": (160, 160, 160),
}


@dataclass(frozen=True)
class ObservationSumoReplayArtifacts:
    replay_trajectory_path: str
    sumo_config_path: str
    gui_replay_script_path: str
    gui_smoke_status_path: str
    network_files: Mapping[str, str]
    replay_fidelity: Mapping[str, Any]
    visual_replay_hint_count: int
    role_map: Mapping[str, str]
    manual_replay_command: str
    smoke_replay_command: str
    track_vehicle_id: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def write_observation_replay_jsonl(
    dataset: ObservationDataset,
    path: str | Path,
) -> list[dict[str, Any]]:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    role_map = build_observation_vehicle_role_map(dataset)
    records: list[dict[str, Any]] = []
    with output_path.open("w", encoding="utf-8") as handle:
        for record in dataset.trajectory_records:
            role = role_map.get(record.vehicle_id, "context")
            payload = {
                "step": record.step,
                "t": record.t,
                "vehicle_id": record.vehicle_id,
                "vehicle_role": role,
                "source_scenario_id": dataset.scenario_id,
                "source_run_id": dataset.run_id,
                "algorithm_family": "ramp_merge_onestep_stage2",
                "x_global": record.x_global,
                "y": record.y,
                "v": record.v,
                "a": record.a,
                "physical_lane": record.physical_lane,
                "road_role": record.road_role,
                "merge_state": record.merge_state,
                "lane_change_state": record.lane_change_state,
                "color_rgb": list(OBSERVATION_ROLE_COLORS.get(role, OBSERVATION_ROLE_COLORS["context"])),
            }
            hint = _visual_replay_hint(record)
            if hint is not None:
                payload["visual_replay_hint"] = hint
            records.append(payload)
            handle.write(json.dumps(payload, ensure_ascii=False, sort_keys=True))
            handle.write("\n")
    return records


def verify_observation_replay_fidelity(
    dataset: ObservationDataset,
    replay_records: Iterable[Mapping[str, Any]],
) -> dict[str, Any]:
    source_by_key = {
        (record.step, record.vehicle_id): record
        for record in dataset.trajectory_records
    }
    replay_by_key = {
        (int(record["step"]), str(record["vehicle_id"])): record
        for record in replay_records
    }
    mismatches: list[dict[str, Any]] = []
    if set(source_by_key) != set(replay_by_key):
        missing = sorted(set(source_by_key) - set(replay_by_key))
        extra = sorted(set(replay_by_key) - set(source_by_key))
        mismatches.append(
            {
                "check": "key_set",
                "missing": [list(item) for item in missing[:10]],
                "extra": [list(item) for item in extra[:10]],
            }
        )
    for key, source in source_by_key.items():
        replay = replay_by_key.get(key)
        if replay is None:
            continue
        for field in (
            "x_global",
            "y",
            "v",
            "a",
            "physical_lane",
            "road_role",
            "merge_state",
            "lane_change_state",
        ):
            expected = getattr(source, field)
            actual = replay.get(field)
            if actual != expected:
                mismatches.append(
                    {
                        "check": "field_exact_match",
                        "key": list(key),
                        "field": field,
                        "expected": expected,
                        "actual": actual,
                    }
                )
                break
    return {
        "status": "passed" if not mismatches else "failed",
        "checked_records": len(source_by_key),
        "mismatches": mismatches,
    }


def build_observation_sumo_replay_artifacts(
    dataset: ObservationDataset,
    output_dir: str | Path,
    *,
    validate_gui_smoke: bool = False,
) -> ObservationSumoReplayArtifacts:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    replay_path = output_path / "replay_trajectory.jsonl"
    sumo_dir = output_path / "sumo"
    gui_status_path = output_path / "gui_smoke_status.json"
    gui_script_path = output_path / "play_gui_replay.ps1"

    replay_records = write_observation_replay_jsonl(dataset, replay_path)
    replay_fidelity = verify_observation_replay_fidelity(dataset, replay_records)
    network_files = build_sumo_network(
        sumo_dir,
        SumoNetworkConfig(end=dataset.t_range()[1] + 2.0),
    )
    track_vehicle_id = dataset.mv_ids[0]
    gui_smoke_status = _not_run_gui_smoke_status(
        sumocfg_file=Path(network_files.sumocfg_file),
        replay_path=replay_path,
        track_vehicle_id=track_vehicle_id,
    )
    if validate_gui_smoke:
        gui_smoke_status = _run_gui_smoke(
            sumocfg_file=Path(network_files.sumocfg_file),
            replay_path=replay_path,
            track_vehicle_id=track_vehicle_id,
        )
    _write_json(gui_status_path, gui_smoke_status)
    _write_gui_script(
        gui_script_path,
        sumocfg_file=Path(network_files.sumocfg_file),
        replay_path=replay_path,
        status_path=gui_status_path,
        track_vehicle_id=track_vehicle_id,
    )

    return ObservationSumoReplayArtifacts(
        replay_trajectory_path=str(replay_path),
        sumo_config_path=str(network_files.sumocfg_file),
        gui_replay_script_path=str(gui_script_path),
        gui_smoke_status_path=str(gui_status_path),
        network_files=network_files.to_dict(),
        replay_fidelity=replay_fidelity,
        visual_replay_hint_count=sum(1 for record in replay_records if "visual_replay_hint" in record),
        role_map=build_observation_vehicle_role_map(dataset),
        manual_replay_command=f'& "{gui_script_path.resolve()}"',
        smoke_replay_command=f'& "{gui_script_path.resolve()}" -Smoke',
        track_vehicle_id=track_vehicle_id,
    )


def build_observation_vehicle_role_map(dataset: ObservationDataset) -> dict[str, str]:
    mv_ids = set(dataset.mv_ids)
    role_map: dict[str, str] = {}
    for record in dataset.trajectory_records:
        if record.vehicle_id in role_map:
            continue
        if record.vehicle_id in mv_ids:
            role_map[record.vehicle_id] = "mv"
        elif record.physical_lane == "lane_2" and record.road_role == "mainline":
            role_map[record.vehicle_id] = "mainline_context"
        else:
            role_map[record.vehicle_id] = "context"

    for plan in _all_plans(dataset.summary):
        front = plan.get("selected_front_vehicle_id")
        rear = plan.get("selected_rear_vehicle_id")
        if front is not None and str(front) not in mv_ids:
            role_map[str(front)] = "selected_front"
        if rear is not None and str(rear) not in mv_ids:
            role_map[str(rear)] = "selected_rear"
    for mv_id in dataset.mv_ids:
        role_map[mv_id] = "mv"
    return role_map


def _visual_replay_hint(record: ObservationTrajectoryRecord) -> dict[str, Any] | None:
    try:
        to_sumo_position(record.x_global, record.physical_lane, record.road_role)
    except ValueError:
        if (
            record.physical_lane == "on_ramp"
            and record.road_role in {"on_ramp", "on_ramp_mv"}
            and record.x_global < RAMP_UPSTREAM_START_X
        ):
            return {
                "mode": "allow_pre_control_on_ramp",
                "edge_id": "ramp_upstream",
                "lane_index": 0,
                "reason": "stage2 on-ramp record is before the supported P17 ramp edge",
            }
        if (
            record.physical_lane == "on_ramp"
            and record.road_role in {"on_ramp", "on_ramp_mv"}
            and record.x_global >= MERGE_END_X
        ):
            edge = EDGE_METADATA["main_post"]
            clamped_x = min(float(record.x_global), MAINLINE_END_X)
            return {
                "mode": "allow_open_on_ramp_after_merge_end",
                "edge_id": "main_post",
                "lane_index": 0,
                "lane_position": clamped_x - edge.start_x,
                "reason": (
                    "open-horizon on-ramp MV remains classified on_ramp after "
                    "the supported merge-zone edge"
                ),
            }
        raise
    return None


def _all_plans(summary: Mapping[str, Any]) -> tuple[Mapping[str, Any], ...]:
    return tuple(
        plan
        for round_summary in summary.get("round_summaries") or []
        for plan in round_summary.get("plan_summaries") or []
    )


def _run_gui_smoke(
    *,
    sumocfg_file: Path,
    replay_path: Path,
    track_vehicle_id: str,
) -> dict[str, Any]:
    try:
        summary = run_trajectory_gui_replay(
            sumocfg_file,
            replay_path,
            track_vehicle_id=track_vehicle_id,
            delay_ms=1,
            hold_seconds=0.0,
            post_roll_steps=0,
            keep_open_after_replay=False,
        )
    except Exception as exc:
        return {
            "status": "failed",
            "sumocfg_file": str(sumocfg_file),
            "replay_jsonl": str(replay_path),
            "sumo_gui_started": False,
            "replayed_steps": 0,
            "replayed_vehicle_ids": [],
            "closed_on_finish": False,
            "track_vehicle_id": track_vehicle_id,
            "error": str(exc),
        }
    return summary.to_dict()


def _not_run_gui_smoke_status(
    *,
    sumocfg_file: Path,
    replay_path: Path,
    track_vehicle_id: str,
) -> dict[str, Any]:
    return {
        "status": "not_run",
        "sumocfg_file": str(sumocfg_file),
        "replay_jsonl": str(replay_path),
        "sumo_gui_started": False,
        "replayed_steps": 0,
        "replayed_vehicle_ids": [],
        "closed_on_finish": False,
        "track_vehicle_id": track_vehicle_id,
        "error": None,
    }


def _write_gui_script(
    path: Path,
    *,
    sumocfg_file: Path,
    replay_path: Path,
    status_path: Path,
    track_vehicle_id: str,
) -> None:
    repo_root = Path(__file__).resolve().parents[2]
    python_exe = repo_root / ".venv" / "Scripts" / "python.exe"
    python_command = str(python_exe if python_exe.exists() else "python")
    module = "cormc.sumo.trajectory_gui_replay"
    base = (
        f'& "{python_command}" -m {module} '
        f'--sumocfg "{sumocfg_file.resolve()}" '
        f'--replay "{replay_path.resolve()}" '
        f'--track-vehicle-id "{track_vehicle_id}" '
    )
    script = "\n".join(
        [
            "param([switch]$Smoke)",
            "$ErrorActionPreference = 'Stop'",
            "$env:PYTHONIOENCODING = 'utf-8'",
            f'Set-Location -LiteralPath "{repo_root}"',
            "",
            "if ($Smoke) {",
            f'  {base}--delay-ms 1 --hold-seconds 0 --post-roll-steps 0 --status-output "{status_path.resolve()}"',
            "  exit $LASTEXITCODE",
            "}",
            "",
            f'{base}--delay-ms 150 --hold-seconds 0 --post-roll-steps 5 --keep-open-after-replay --status-output "{status_path.resolve()}"',
            "exit $LASTEXITCODE",
            "",
        ]
    )
    path.write_text(script, encoding="utf-8")


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
