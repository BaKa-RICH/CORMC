from __future__ import annotations

import json
from pathlib import Path

from cormc.sumo import mvs_gui_replay


def test_p17_1_gui_replay_uses_lane_hints_colors_and_gui_focus(monkeypatch, tmp_path: Path) -> None:
    sumocfg = tmp_path / "p17.sumocfg"
    sumocfg.write_text("<configuration />", encoding="utf-8")
    replay = tmp_path / "replay_trajectory.jsonl"
    replay.write_text(
        "\n".join(
            [
                json.dumps(
                    {
                        "vehicle_id": "MV_DEMO",
                        "vehicle_role": "mv_on_ramp_active",
                        "step": 0,
                        "t": 0.0,
                        "x_global": 6832.0,
                        "y": -3.5,
                        "v": 20.0,
                        "a": 0.0,
                        "physical_lane": "on_ramp",
                        "road_role": "on_ramp_mv",
                        "merge_state": "not_started",
                        "lane_change_state": "normal",
                        "color_rgb": [0, 90, 220],
                    }
                ),
                json.dumps(
                    {
                        "vehicle_id": "MV_DEMO",
                        "vehicle_role": "mv_on_ramp_active",
                        "step": 1,
                        "t": 0.1,
                        "x_global": 6834.0,
                        "y": -3.5,
                        "v": 20.0,
                        "a": 0.0,
                        "physical_lane": "on_ramp",
                        "road_role": "on_ramp_mv",
                        "merge_state": "not_started",
                        "lane_change_state": "normal",
                        "color_rgb": [0, 90, 220],
                    }
                ),
            ]
        ),
        encoding="utf-8",
    )

    fake_traci = _FakeTraci()
    started: list[list[str]] = []
    monkeypatch.setattr(mvs_gui_replay, "ensure_sumo_tools_on_path", lambda: _FakePaths())
    monkeypatch.setattr(mvs_gui_replay, "import_traci", lambda: fake_traci)
    fake_traci.start_callback = lambda args: started.append(args)

    summary = mvs_gui_replay.run_mvs_gui_replay(
        sumocfg,
        replay,
        track_vehicle_id="MV_DEMO",
        delay_ms=1,
        hold_seconds=0,
        post_roll_steps=0,
    )

    assert summary.status == "ok"
    assert summary.sumo_gui_started is True
    assert summary.replayed_steps == 2
    assert summary.replayed_vehicle_ids == ("MV_DEMO",)
    assert summary.closed_on_finish is True
    assert started[0][0] == "sumo-gui"
    assert fake_traci.vehicle.colors["MV_DEMO"] == (0, 90, 220)
    assert fake_traci.vehicle.add_calls[0]["route_id"] == "route_ramp"
    assert fake_traci.vehicle.move_calls
    assert fake_traci.gui.tracked == "MV_DEMO"


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
        self.colors: dict[str, tuple[int, int, int]] = {}
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

    def setColor(self, vehicle_id, color):
        self.colors[vehicle_id] = tuple(color)

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
