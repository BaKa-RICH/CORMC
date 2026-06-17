from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.observation.artifacts import build_observation_artifact_bundle

PHASE7_ROOT = Path("artifacts/scene_interface_phase7_validation")
PHASE7_SCENARIO_DIRS = (
    PHASE7_ROOT / "RM-ONESTEP-S05-PLAN-STEP0" / "scene-interface-phase7-validation",
    PHASE7_ROOT / "RM-ONESTEP-S05-ROLLING-ENTRY" / "scene-interface-phase7-validation",
    PHASE7_ROOT / "RM-ONESTEP-S07-PLAN-STEP0" / "scene-interface-phase7-validation",
    PHASE7_ROOT / "RM-ONESTEP-S07-ROLLING-ENTRY" / "scene-interface-phase7-validation",
    PHASE7_ROOT,
)


@pytest.mark.parametrize("source_dir", PHASE7_SCENARIO_DIRS)
def test_observation_artifact_bundle_builds_stage8_files(source_dir: Path) -> None:
    bundle = build_observation_artifact_bundle(source_dir, play=False)

    expected_paths = (
        source_dir / "stage8_plots" / "process_x_t_local.png",
        source_dir / "stage8_plots" / "process_v_t.png",
        source_dir / "stage8_plots" / "process_y_t.png",
        source_dir / "stage8_plots" / "lifecycle_timeline.png",
        source_dir / "stage8_sumo_replay" / "replay_trajectory.jsonl",
        source_dir / "stage8_sumo_replay" / "sumo" / "p17.sumocfg",
        source_dir / "stage8_sumo_replay" / "play_gui_replay.ps1",
        source_dir / "stage8_artifact_manifest.json",
        source_dir / "stage8_report.md",
    )
    for path in expected_paths:
        assert path.exists()

    manifest = json.loads((source_dir / "stage8_artifact_manifest.json").read_text(encoding="utf-8"))
    assert manifest["artifact_schema"] == "observation_artifact.v1"
    assert manifest["source_files"] == {
        "stage2_summary_json": str(source_dir / "stage2_summary.json"),
        "trajectory_csv": str(source_dir / "trajectory.csv"),
        "gap_rows_json": str(source_dir / "gap_rows.json"),
    }
    assert manifest["validation"]["status"] == "validation passed"
    report = (source_dir / "stage8_report.md").read_text(encoding="utf-8")
    assert "validation passed" in report
    assert "five-level summary" in report
    assert bundle.status == "passed"
