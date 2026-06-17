from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.observation.stage2_artifacts import load_stage2_observation_dataset

PHASE7_ROOT = Path("artifacts/scene_interface_phase7_validation")
PHASE7_SCENARIO_DIRS = (
    PHASE7_ROOT / "RM-ONESTEP-S05-PLAN-STEP0" / "scene-interface-phase7-validation",
    PHASE7_ROOT / "RM-ONESTEP-S05-ROLLING-ENTRY" / "scene-interface-phase7-validation",
    PHASE7_ROOT / "RM-ONESTEP-S07-PLAN-STEP0" / "scene-interface-phase7-validation",
    PHASE7_ROOT / "RM-ONESTEP-S07-ROLLING-ENTRY" / "scene-interface-phase7-validation",
    PHASE7_ROOT,
)
PHASE7_SCENARIO_IDS = (
    "RM-ONESTEP-S05-PLAN-STEP0",
    "RM-ONESTEP-S05-ROLLING-ENTRY",
    "RM-ONESTEP-S07-PLAN-STEP0",
    "RM-ONESTEP-S07-ROLLING-ENTRY",
    "RM-ONESTEP-S07-2MV-ROLLING-ENTRY",
)


def test_stage2_observation_dataset_loads_five_phase7_scenarios() -> None:
    datasets = [load_stage2_observation_dataset(path) for path in PHASE7_SCENARIO_DIRS]

    assert [dataset.scenario_id for dataset in datasets] == list(PHASE7_SCENARIO_IDS)
    for dataset in datasets:
        assert dataset.trajectory_records
        assert set(dataset.mv_ids) == set(dataset.summary["mv_summaries"])
        expected_gap_rows = sum(
            len(mv_summary["gap_rows"])
            for mv_summary in dataset.summary["mv_summaries"].values()
        )
        assert len(dataset.gap_rows) == expected_gap_rows


def test_stage2_observation_dataset_2mv_s07_counts_and_lifecycle() -> None:
    dataset = load_stage2_observation_dataset(PHASE7_ROOT)

    assert dataset.mv_ids == ("S07_MV", "S07_MV_REAR")
    assert dataset.step_range() == (0, 699)
    assert len(dataset.vehicle_ids()) == 9
    assert len(dataset.trajectory_records) == 6300
    for mv_id in dataset.mv_ids:
        lifecycle = dataset.lifecycles[mv_id]
        assert lifecycle.locked_gap_step is not None
        assert lifecycle.lateral_start_step is not None
        assert lifecycle.lateral_completed_step is not None
        assert lifecycle.mainline_conversion_step is not None
        assert lifecycle.final_physical_lane == "lane_2"
        assert lifecycle.final_road_role == "mainline"
        assert lifecycle.final_merge_state == "normal"


def test_stage2_observation_dataset_requires_formal_source_files() -> None:
    for source_dir in PHASE7_SCENARIO_DIRS:
        summary = json.loads((source_dir / "stage2_summary.json").read_text(encoding="utf-8"))
        assert set(summary) == {
            "scenario_summary",
            "round_summaries",
            "mv_summaries",
            "cross_mv_summary",
            "artifact_paths",
        }
