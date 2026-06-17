from cormc.traffic_flow.source import SeededRandomBoundaryFlowSource
from cormc.traffic_flow.generation import queue_fingerprint
from cormc.scenes import default_onestep_random_lane2_ramp_profile


def test_seeded_random_boundary_flow_source_is_reproducible() -> None:
    profile = default_onestep_random_lane2_ramp_profile(seed=645001)
    source = SeededRandomBoundaryFlowSource("test-source", profile)

    first = source.build_queue(10.0)
    second = source.build_queue(10.0)

    assert queue_fingerprint(first) == queue_fingerprint(second)


def test_seeded_random_boundary_flow_source_changes_with_seed() -> None:
    first = SeededRandomBoundaryFlowSource(
        "first",
        default_onestep_random_lane2_ramp_profile(seed=645001),
    ).build_queue(10.0)
    second = SeededRandomBoundaryFlowSource(
        "second",
        default_onestep_random_lane2_ramp_profile(seed=645002),
    ).build_queue(10.0)

    assert queue_fingerprint(first) != queue_fingerprint(second)


def test_seeded_random_boundary_flow_density_comes_from_profile() -> None:
    medium = SeededRandomBoundaryFlowSource(
        "medium",
        default_onestep_random_lane2_ramp_profile(
            seed=645001,
            lane2_mean_headway=2.2,
            ramp_mean_headway=6.0,
        ),
    ).build_queue(30.0)
    high = SeededRandomBoundaryFlowSource(
        "high",
        default_onestep_random_lane2_ramp_profile(
            seed=645001,
            lane2_mean_headway=1.5,
            ramp_mean_headway=4.2,
        ),
    ).build_queue(30.0)

    assert len(high) > len(medium)


def test_seeded_random_boundary_flow_summary_contains_profile_controls() -> None:
    profile = default_onestep_random_lane2_ramp_profile(seed=645001)
    source = SeededRandomBoundaryFlowSource("test-source", profile)

    summary = source.to_summary()

    assert summary["source_type"] == "seeded_random"
    assert summary["seed"] == 645001
    assert summary["profile_id"] == profile.profile_id
    assert summary["arrival_streams"]
    assert summary["safe_spawn_gap_m"] == 20.0
