from __future__ import annotations

from dataclasses import asdict
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.rolling import (
    EFFECTIVE_CONTROL_BOTH,
    EFFECTIVE_CONTROL_REAR,
    build_initial_onestep_stage2_state,
    identify_and_number_gaps,
)
from cormc.scenes import (
    RM_ONESTEP_S07_2MV_MV_IDS,
    RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
)
from cormc.simulation_core.pre_freeze import LANE_2


def test_gap_snapshot_uses_structured_lane2_boundary_vehicle_ids() -> None:
    state, _ = build_initial_onestep_stage2_state(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID
    )

    snapshot = identify_and_number_gaps(state, danger_vehicle_ids=())

    assert snapshot.step == state.step
    assert snapshot.t == pytest.approx(state.t)
    assert snapshot.lane_id == LANE_2
    assert len(snapshot.gaps) == 6
    first = snapshot.gaps[0]
    assert first.gap_id == "gap:0:1"
    assert first.index == 1
    assert first.rear_vehicle_id == "S07_L2_06"
    assert first.front_vehicle_id == "S07_L2_07"
    assert first.rear_x_global == pytest.approx(6790.0)
    assert first.front_x_global == pytest.approx(6850.0)
    assert first.bumper_gap_m == pytest.approx(56.0)
    assert first.effective_control_type == EFFECTIVE_CONTROL_BOTH

    for gap in snapshot.gaps:
        payload = asdict(gap)
        assert {
            "index",
            "front_vehicle_id",
            "rear_vehicle_id",
            "front_x_global",
            "rear_x_global",
            "bumper_gap_m",
            "effective_control_type",
        }.issubset(payload)


def test_gap_snapshot_index_increases_from_front_to_rear() -> None:
    state, _ = build_initial_onestep_stage2_state(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID
    )

    snapshot = identify_and_number_gaps(state, danger_vehicle_ids=())

    assert tuple(gap.index for gap in snapshot.gaps) == tuple(
        range(1, len(snapshot.gaps) + 1)
    )
    assert tuple(gap.rear_x_global for gap in snapshot.gaps) == tuple(
        sorted((gap.rear_x_global for gap in snapshot.gaps), reverse=True)
    )
    assert tuple(gap.front_x_global for gap in snapshot.gaps) == tuple(
        sorted((gap.front_x_global for gap in snapshot.gaps), reverse=True)
    )
    assert all(gap.rear_x_global < gap.front_x_global for gap in snapshot.gaps)


def test_gap_snapshot_ignores_two_on_ramp_mvs_as_gap_boundaries() -> None:
    state, _ = build_initial_onestep_stage2_state(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID
    )

    snapshot = identify_and_number_gaps(state, danger_vehicle_ids=())
    boundary_ids = {
        boundary_id
        for gap in snapshot.gaps
        for boundary_id in (gap.rear_vehicle_id, gap.front_vehicle_id)
    }

    assert not set(RM_ONESTEP_S07_2MV_MV_IDS) & boundary_ids
    assert all(
        state.vehicle_states[vehicle_id].physical_lane == LANE_2
        for vehicle_id in boundary_ids
    )


def test_gap_snapshot_danger_boundary_vehicle_degrades_adjacent_gap_control() -> None:
    state, _ = build_initial_onestep_stage2_state(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID
    )

    snapshot = identify_and_number_gaps(state, danger_vehicle_ids=("S07_L2_02",))
    last_two = snapshot.gaps[-2:]
    second_from_rear, rearmost = last_two

    assert rearmost.front_vehicle_id == "S07_L2_02"
    assert rearmost.effective_control_type == EFFECTIVE_CONTROL_REAR
    assert second_from_rear.rear_vehicle_id == "S07_L2_02"
    assert second_from_rear.effective_control_type != EFFECTIVE_CONTROL_BOTH
