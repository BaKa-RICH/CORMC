from __future__ import annotations

from pathlib import Path

from cormc.sumo.env import ensure_sumo_available_or_skip
from cormc.sumo.monitoring import read_realization_jsonl
from cormc.sumo.replay import (
    REPLAY_CV_LANE_CHANGE,
    REPLAY_IDS,
    REPLAY_MAIN_STRAIGHT,
    REPLAY_MV_MERGE,
    build_replay_trajectory,
    run_replay,
)


def test_replay_trajectory_specs_are_fixed() -> None:
    main = build_replay_trajectory(REPLAY_MAIN_STRAIGHT)
    cv = build_replay_trajectory(REPLAY_CV_LANE_CHANGE)
    mv = build_replay_trajectory(REPLAY_MV_MERGE)

    assert main.vehicle_id == "MAIN_ACTIVE"
    assert main.commands[0].physical_lane == "lane_2"
    assert main.commands[0].x_global == 100
    assert main.commands[-1].x_global == 160
    assert main.commands[-1].y == 0
    assert main.commands[-1].v == 30

    assert cv.vehicle_id == "CV_ACTIVE"
    assert cv.commands[0].x_global == 6900
    assert cv.commands[-1].x_global == 7040
    assert cv.commands[0].y == 0
    assert cv.commands[-1].y == 3.5
    assert cv.commands[-1].physical_lane == "lane_1"

    assert mv.vehicle_id == "MV_ACTIVE"
    assert mv.route_id == "route_ramp"
    assert mv.commands[0].x_global == 6950
    assert mv.commands[-1].x_global == 7250
    assert mv.commands[0].y == -3.5
    assert mv.commands[-1].y == 0
    assert mv.commands[-1].physical_lane == "lane_2"


def test_three_p17_replays_run_real_sumo_and_emit_artifacts(tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()

    for run_id in REPLAY_IDS:
        result = run_replay(run_id, tmp_path / run_id)
        assert result.simulation_result is not None
        assert result.simulation_result.status == "passed"
        assert Path(result.extra_paths["realization_jsonl"]).exists()
        assert Path(result.report_path or "").exists()

        network_names = {Path(path).name for path in result.network_files.values() if path.endswith((".xml", ".sumocfg"))}
        assert {"p17.sumocfg", "p17.net.xml", "p17.rou.xml", "p17.nod.xml", "p17.edg.xml", "p17.con.xml"} <= network_names

        records = read_realization_jsonl(result.extra_paths["realization_jsonl"])
        assert records
        assert {record.result for record in records} == {"matched"}
        report = Path(result.report_path or "").read_text(encoding="utf-8")
        assert "sumo-gui" in report
        assert "move_to_xy_trajectory_authority" in report
