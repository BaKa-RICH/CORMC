from __future__ import annotations

import pytest

from cormc.random_generation import (
    ArrivalStream,
    SeededRandomProfile,
    default_p16_seeded_random_profile,
    generate_boundary_queue,
    profile_from_mapping,
    queue_fingerprint,
)
from cormc.step0_3 import DEFAULT_ROAD_GEOMETRY


def test_p16_same_seed_same_profile_reproducible_queue() -> None:
    profile = default_p16_seeded_random_profile(seed=16001)

    left = generate_boundary_queue(profile, max_t=10.0)
    right = generate_boundary_queue(profile, max_t=10.0)

    assert left
    assert queue_fingerprint(left) == queue_fingerprint(right)
    assert {item.lane_id for item in left} == {"lane_1", "lane_2", "on_ramp"}
    assert all(item.seed == 16001 for item in left)
    assert all(item.profile_id == "p16_internal_demo_v1" for item in left)


def test_p16_different_seed_changes_headway_or_vehicle_attributes() -> None:
    left = generate_boundary_queue(default_p16_seeded_random_profile(seed=16001), max_t=12.0)
    right = generate_boundary_queue(default_p16_seeded_random_profile(seed=16002), max_t=12.0)

    assert queue_fingerprint(left) != queue_fingerprint(right)


def test_p16_disabled_profile_returns_empty_queue() -> None:
    profile = default_p16_seeded_random_profile(seed=16001, enabled=False)

    assert generate_boundary_queue(profile, max_t=100.0) == ()


def test_p16_default_profile_uses_documented_first_version_spawn_defaults() -> None:
    profile = default_p16_seeded_random_profile(seed=16001)
    by_lane = {stream.lane_id: stream for stream in profile.arrival_streams}

    assert by_lane["lane_1"].spawn_x == DEFAULT_ROAD_GEOMETRY.mainline_start_global
    assert by_lane["lane_2"].spawn_x == DEFAULT_ROAD_GEOMETRY.mainline_start_global
    assert by_lane["on_ramp"].spawn_x == DEFAULT_ROAD_GEOMETRY.x0_m_global - 100.0
    assert by_lane["lane_1"].initial_speed == 30.0
    assert by_lane["lane_2"].initial_speed == 30.0
    assert by_lane["on_ramp"].initial_speed == 16.0
    assert by_lane["lane_1"].shifted_headway == 1.2
    assert by_lane["lane_2"].shifted_headway == 1.2
    assert by_lane["on_ramp"].shifted_headway == 3.5


@pytest.mark.parametrize(
    ("profile", "message"),
    [
        (
            SeededRandomProfile(
                arrival_streams=(
                    ArrivalStream(
                        lane_id="lane_3",
                        shifted_headway=1.2,
                        initial_speed=30.0,
                        spawn_x=0.0,
                        spawn_y=0.0,
                    ),
                )
            ),
            "unsupported lane_id",
        ),
        (
            SeededRandomProfile(
                cav_penetration_rate=1.2,
                arrival_streams=default_p16_seeded_random_profile().arrival_streams,
            ),
            "cav_penetration_rate",
        ),
        (
            SeededRandomProfile(
                chv_compliance_rate=-0.1,
                arrival_streams=default_p16_seeded_random_profile().arrival_streams,
            ),
            "chv_compliance_rate",
        ),
        (
            SeededRandomProfile(
                arrival_streams=(
                    ArrivalStream(
                        lane_id="lane_1",
                        shifted_headway=1.2,
                        initial_speed=30.0,
                        spawn_x=0.0,
                        spawn_y=3.5,
                        mean_headway=1.2,
                    ),
                )
            ),
            "mean_headway",
        ),
    ],
)
def test_p16_profile_validation_reports_clear_errors(
    profile: SeededRandomProfile,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        generate_boundary_queue(profile, max_t=5.0)


def test_p16_profile_mapping_round_trip_accepts_minimal_config() -> None:
    profile = profile_from_mapping(
        {
            "enabled": True,
            "seed": 42,
            "profile_id": "unit",
            "arrival_streams": [
                {
                    "lane_id": "lane_1",
                    "shifted_headway": 1.2,
                    "initial_speed": 30.0,
                    "spawn_x": 0.0,
                    "spawn_y": 3.5,
                    "mean_headway": 2.0,
                }
            ],
            "cav_penetration_rate": 0.5,
            "chv_compliance_rate": 0.5,
            "safe_spawn_gap_m": 20.0,
        }
    )

    queue = generate_boundary_queue(profile, max_t=5.0)

    assert profile.seed == 42
    assert profile.profile_id == "unit"
    assert queue
    assert all(item.lane_id == "lane_1" for item in queue)
