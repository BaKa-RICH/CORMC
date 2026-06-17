from __future__ import annotations

from dataclasses import replace
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.legacy import build_initial_ramp_merge_state, run_safety_check
from cormc.simulation_core.pre_freeze import LANE_1, LANE_2


def test_safety_check_flags_adjacent_ttc_below_threshold() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-01")
    risky = _with_vehicle_state(
        _with_vehicle_state(state, "B01_CFV", x_global=100.0, v=30.0),
        "B01_CLV",
        x_global=118.9,
        v=20.0,
    )

    safety = run_safety_check(risky)

    assert set(safety.danger_vehicle_ids) == {"B01_CFV", "B01_CLV"}
    assert len(safety.danger_pairs) == 1
    pair = dict(safety.danger_pairs[0])
    assert pair["rear_vehicle_id"] == "B01_CFV"
    assert pair["front_vehicle_id"] == "B01_CLV"
    assert pair["physical_lane"] == LANE_2
    assert pair["bumper_gap_m"] == pytest.approx(14.9)
    assert pair["closing_speed_mps"] == pytest.approx(10.0)
    assert pair["ttc_s"] == pytest.approx(1.49)
    assert pair["unsafe_ttc"] is True
    assert pair["short_gap"] is False


def test_safety_check_ttc_equal_threshold_is_not_danger() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-01")
    threshold = _with_vehicle_state(
        _with_vehicle_state(state, "B01_CFV", x_global=100.0, v=30.0),
        "B01_CLV",
        x_global=119.0,
        v=20.0,
    )

    safety = run_safety_check(threshold)

    assert safety.safety_alert is False
    assert safety.danger_vehicle_ids == ()
    assert safety.danger_pairs == ()


def test_safety_check_short_gap_without_closing_speed_is_not_danger() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-01")
    short_gap = _with_vehicle_state(
        _with_vehicle_state(state, "B01_CFV", x_global=100.0, v=20.0),
        "B01_CLV",
        x_global=105.0,
        v=20.0,
    )

    safety = run_safety_check(short_gap)

    assert safety.safety_alert is False
    assert safety.danger_vehicle_ids == ()
    assert safety.danger_pairs == ()


def test_safety_check_ignores_close_vehicles_across_physical_lanes() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-01")
    cross_lane = _with_vehicle_state(
        _with_vehicle_state(
            state,
            "B01_CFV",
            x_global=100.0,
            v=30.0,
            physical_lane=LANE_2,
        ),
        "B01_CLV",
        x_global=105.0,
        v=20.0,
        physical_lane=LANE_1,
    )

    safety = run_safety_check(cross_lane)

    assert safety.safety_alert is False
    assert safety.danger_vehicle_ids == ()
    assert safety.danger_pairs == ()


def test_safety_check_only_evaluates_adjacent_pairs_in_lane_order() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-01")
    with_middle_vehicle = _with_vehicle_state(
        _with_vehicle_state(
            _with_vehicle_state(
                state,
                "B01_CFV",
                x_global=100.0,
                v=30.0,
                physical_lane=LANE_2,
            ),
            "B01_TLV_CFV",
            x_global=110.0,
            v=20.0,
            physical_lane=LANE_2,
        ),
        "B01_CLV",
        x_global=118.9,
        v=20.0,
        physical_lane=LANE_2,
    )

    safety = run_safety_check(with_middle_vehicle)

    assert len(safety.danger_pairs) == 1
    pair = dict(safety.danger_pairs[0])
    assert pair["rear_vehicle_id"] == "B01_CFV"
    assert pair["front_vehicle_id"] == "B01_TLV_CFV"
    assert not (
        pair["rear_vehicle_id"] == "B01_CFV"
        and pair["front_vehicle_id"] == "B01_CLV"
    )


def _with_vehicle_state(state, vehicle_id: str, **updates):
    vehicle_states = dict(state.vehicle_states)
    vehicle_states[vehicle_id] = replace(vehicle_states[vehicle_id], **updates)
    return replace(state, vehicle_states=vehicle_states)
