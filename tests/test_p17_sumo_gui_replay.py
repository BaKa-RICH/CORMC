from __future__ import annotations

import json
from pathlib import Path

from cormc.sumo import gui_replay


def test_p17_gui_replay_reads_records_and_uses_traci_gui_entrypoint(monkeypatch, tmp_path: Path) -> None:
    sumocfg = tmp_path / "p17.sumocfg"
    sumocfg.write_text("<configuration />", encoding="utf-8")
    realization = tmp_path / "realization.jsonl"
    realization.write_text(
        "\n".join(
            [
                json.dumps(
                    {
                        "vehicle_id": "MV_ACTIVE",
                        "step": 0,
                        "t": 0.0,
                        "command_x_global": 6889.64,
                        "command_y": -3.5,
                        "command_v": 16.4,
                    }
                ),
                json.dumps(
                    {
                        "vehicle_id": "MV_ACTIVE",
                        "step": 1,
                        "t": 0.1,
                        "command_x_global": 6891.32,
                        "command_y": -3.5,
                        "command_v": 16.8,
                    }
                ),
            ]
        ),
        encoding="utf-8",
    )
    (tmp_path / "artifact_manifest.json").write_text(
        json.dumps({"seed": 16001, "background_vehicle_ids_sample": ["BG_0", "BG_1"]}),
        encoding="utf-8",
    )

    fake_traci = _FakeTraci()
    started: list[list[str]] = []

    monkeypatch.setattr(gui_replay, "ensure_sumo_tools_on_path", lambda: _FakePaths())
    monkeypatch.setattr(gui_replay, "import_traci", lambda: fake_traci)
    fake_traci.start_callback = lambda args: started.append(args)

    summary = gui_replay.run_p17_gui_replay(
        sumocfg,
        realization,
        delay_ms=1,
        hold_seconds=0,
        post_roll_steps=0,
    )

    assert summary.status == "ok"
    assert summary.replayed_steps == 2
    assert summary.replayed_vehicle_ids == ("MV_ACTIVE",)
    assert started
    assert started[0][0] == "sumo-gui"
    assert "--start" in started[0]
    assert "--delay" in started[0]
    assert "MV_ACTIVE" in fake_traci.vehicle.ids
    assert "BG_0" in fake_traci.vehicle.ids
    assert "BG_1" in fake_traci.vehicle.ids
    assert fake_traci.vehicle.move_count >= 4
    assert fake_traci.step_count == 2
    assert fake_traci.closed is True


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
        self.move_count = 0

    def getIDList(self):
        return list(self.ids)

    def add(self, vehicle_id, *_args, **_kwargs):
        self.ids.add(vehicle_id)

    def setSpeedMode(self, *_args):
        return None

    def setLaneChangeMode(self, *_args):
        return None

    def setPreviousSpeed(self, *_args):
        return None

    def setSpeed(self, *_args):
        return None

    def moveToXY(self, *_args, **_kwargs):
        self.move_count += 1


class _FakeGui:
    DEFAULT_VIEW = "View #0"

    def trackVehicle(self, *_args):
        return None

    def setZoom(self, *_args):
        return None

    def setOffset(self, *_args):
        return None
