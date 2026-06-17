from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cormc.sumo.trajectory_gui_replay as trajectory_gui_replay
from cormc.observation.stage2_artifacts import load_stage2_observation_dataset
from cormc.observation.sumo_replay import (
    build_observation_sumo_replay_artifacts,
    verify_observation_replay_fidelity,
    write_observation_replay_jsonl,
)
from cormc.sumo.mapping import to_sumo_position

PHASE7_ROOT = Path("artifacts/scene_interface_phase7_validation")
PHASE7_SCENARIO_DIRS = (
    PHASE7_ROOT / "RM-ONESTEP-S05-PLAN-STEP0" / "scene-interface-phase7-validation",
    PHASE7_ROOT / "RM-ONESTEP-S05-ROLLING-ENTRY" / "scene-interface-phase7-validation",
    PHASE7_ROOT / "RM-ONESTEP-S07-PLAN-STEP0" / "scene-interface-phase7-validation",
    PHASE7_ROOT / "RM-ONESTEP-S07-ROLLING-ENTRY" / "scene-interface-phase7-validation",
    PHASE7_ROOT,
)


@pytest.mark.parametrize("source_dir", PHASE7_SCENARIO_DIRS)
def test_observation_sumo_replay_fidelity_for_phase7_scenarios(tmp_path: Path, source_dir: Path) -> None:
    dataset = load_stage2_observation_dataset(source_dir)
    replay_records = write_observation_replay_jsonl(dataset, tmp_path / dataset.scenario_id / "replay_trajectory.jsonl")

    assert verify_observation_replay_fidelity(dataset, replay_records)["status"] == "passed"
    for record in replay_records:
        to_sumo_position(record["x_global"], record["physical_lane"], record["road_role"])


def test_observation_sumo_replay_2mv_counts_and_final_states(tmp_path: Path) -> None:
    dataset = load_stage2_observation_dataset(PHASE7_ROOT)
    artifacts = build_observation_sumo_replay_artifacts(dataset, tmp_path / "replay")
    replay_records = [
        json.loads(line)
        for line in Path(artifacts.replay_trajectory_path).read_text(encoding="utf-8").splitlines()
    ]

    assert len(replay_records) == 6300
    assert {record["vehicle_id"] for record in replay_records} == set(dataset.vehicle_ids())
    assert artifacts.replay_fidelity["status"] == "passed"
    for mv_id in ("S07_MV", "S07_MV_REAR"):
        mv_records = [record for record in replay_records if record["vehicle_id"] == mv_id]
        assert min(record["y"] for record in mv_records) == pytest.approx(-3.5)
        assert max(record["y"] for record in mv_records) == pytest.approx(0.0)
        final = mv_records[-1]
        assert final["physical_lane"] == "lane_2"
        assert final["road_role"] == "mainline"
        assert final["merge_state"] == "normal"
    assert Path(artifacts.sumo_config_path).name == "p17.sumocfg"
    assert Path(artifacts.gui_replay_script_path).read_text(encoding="utf-8").count("cormc.sumo.trajectory_gui_replay") == 2


def test_trajectory_gui_replay_reads_jsonl_and_moves_all_vehicles(monkeypatch, tmp_path: Path) -> None:
    sumocfg = tmp_path / "p17.sumocfg"
    sumocfg.write_text("<configuration />", encoding="utf-8")
    replay = tmp_path / "replay_trajectory.jsonl"
    replay.write_text(
        "\n".join(
            [
                json.dumps(
                    {
                        "vehicle_id": "S07_MV",
                        "vehicle_role": "mv",
                        "step": 0,
                        "t": 0.0,
                        "x_global": 6602.0,
                        "y": -3.5,
                        "v": 20.0,
                        "a": 0.0,
                        "physical_lane": "on_ramp",
                        "road_role": "on_ramp_mv",
                        "merge_state": "not_started",
                        "lane_change_state": "normal",
                        "color_rgb": [210, 55, 35],
                    }
                ),
                json.dumps(
                    {
                        "vehicle_id": "S07_L2_04",
                        "vehicle_role": "selected_rear",
                        "step": 0,
                        "t": 0.0,
                        "x_global": 6820.0,
                        "y": 0.0,
                        "v": 20.0,
                        "a": 0.0,
                        "physical_lane": "lane_2",
                        "road_role": "mainline",
                        "merge_state": "normal",
                        "lane_change_state": "normal",
                        "color_rgb": [230, 140, 20],
                    }
                ),
            ]
        ),
        encoding="utf-8",
    )
    fake_traci = _FakeTraci()
    started: list[list[str]] = []
    monkeypatch.setattr(trajectory_gui_replay, "ensure_sumo_tools_on_path", lambda: _FakePaths())
    monkeypatch.setattr(trajectory_gui_replay, "import_traci", lambda: fake_traci)
    fake_traci.start_callback = lambda args: started.append(args)

    summary = trajectory_gui_replay.run_trajectory_gui_replay(
        sumocfg,
        replay,
        track_vehicle_id="S07_MV",
        delay_ms=1,
        hold_seconds=0,
        post_roll_steps=0,
    )

    assert summary.status == "ok"
    assert summary.replayed_steps == 1
    assert summary.replayed_vehicle_ids == ("S07_L2_04", "S07_MV")
    assert summary.closed_on_finish is True
    assert started[0][0] == "sumo-gui"
    assert {call["vehicle_id"] for call in fake_traci.vehicle.add_calls} == {"S07_MV", "S07_L2_04"}
    assert len(fake_traci.vehicle.move_calls) == 4
    assert fake_traci.gui.tracked == "S07_MV"


def test_trajectory_gui_replay_uses_open_on_ramp_after_merge_hint(monkeypatch, tmp_path: Path) -> None:
    sumocfg = tmp_path / "p17.sumocfg"
    sumocfg.write_text("<configuration />", encoding="utf-8")
    replay = tmp_path / "replay_trajectory.jsonl"
    replay.write_text(
        json.dumps(
            {
                "vehicle_id": "OPEN_MV",
                "vehicle_role": "mv",
                "step": 0,
                "t": 0.0,
                "x_global": 7250.7,
                "y": -3.5,
                "v": 20.0,
                "a": 0.0,
                "physical_lane": "on_ramp",
                "road_role": "on_ramp_mv",
                "merge_state": "not_started",
                "lane_change_state": "normal",
                "color_rgb": [210, 55, 35],
                "visual_replay_hint": {
                    "mode": "allow_open_on_ramp_after_merge_end",
                    "edge_id": "main_post",
                    "lane_index": 0,
                    "lane_position": 0.7,
                },
            }
        ),
        encoding="utf-8",
    )
    fake_traci = _FakeTraci()
    monkeypatch.setattr(trajectory_gui_replay, "ensure_sumo_tools_on_path", lambda: _FakePaths())
    monkeypatch.setattr(trajectory_gui_replay, "import_traci", lambda: fake_traci)

    summary = trajectory_gui_replay.run_trajectory_gui_replay(
        sumocfg,
        replay,
        track_vehicle_id="OPEN_MV",
        delay_ms=1,
        hold_seconds=0,
        post_roll_steps=0,
    )

    assert summary.status == "ok"
    assert fake_traci.vehicle.add_calls[0]["route_id"] == "route_ramp"
    move_args, _ = fake_traci.vehicle.move_calls[0]
    assert move_args[1] == "main_post"
    assert move_args[2] == 0


class _FakePaths:
    sumo_gui = "sumo-gui"
    sumo = "sumo"


class _FakeTraci:
    def __init__(self) -> None:
        self.vehicle = _FakeVehicle()
        self.gui = _FakeGui()
        self.step_count = 0
        self.closed = False
        self.start_callback = None

    def start(self, args):
        if self.start_callback is not None:
            self.start_callback(args)

    def simulationStep(self):
        self.step_count += 1

    def close(self, _wait=True):
        self.closed = True


class _FakeVehicle:
    def __init__(self) -> None:
        self.ids: set[str] = set()
        self.add_calls: list[dict[str, str]] = []
        self.move_calls: list[tuple] = []

    def getIDList(self):
        return list(self.ids)

    def add(self, vehicle_id, route_id, **kwargs):
        self.ids.add(vehicle_id)
        self.add_calls.append({"vehicle_id": vehicle_id, "route_id": route_id, **kwargs})

    def setSpeedMode(self, *_args):
        return None

    def setLaneChangeMode(self, *_args):
        return None

    def setPreviousSpeed(self, *_args):
        return None

    def setSpeed(self, *_args):
        return None

    def setColor(self, *_args):
        return None

    def moveToXY(self, *args, **kwargs):
        self.move_calls.append((args, kwargs))


class _FakeGui:
    DEFAULT_VIEW = "View #0"

    def __init__(self) -> None:
        self.tracked = None

    def trackVehicle(self, _view, vehicle_id):
        self.tracked = vehicle_id

    def setZoom(self, *_args):
        return None

    def setOffset(self, *_args):
        return None
