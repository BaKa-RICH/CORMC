from __future__ import annotations


def test_sumo_public_api_exports_neutral_names_from_sumo_package() -> None:
    import cormc.sumo as sumo
    from cormc.sumo import (
        ControlledTrajectoryCommand,
        RealizationRecord,
        SpawnCommand,
        SumoArtifactResult,
        SumoClosedLoopRunnerConfig,
        SumoClosedLoopSimulationResult,
        SumoNetworkConfig,
        SumoNetworkFiles,
        SumoSimulationResult,
        SumoTrajectoryAuthorityConfig,
        build_sumo_network,
        run_sumo_trajectory_authority_simulation,
        run_trajectory_gui_replay,
    )

    assert ControlledTrajectoryCommand.__name__ == "ControlledTrajectoryCommand"
    assert SpawnCommand.__name__ == "SpawnCommand"
    assert RealizationRecord.__name__ == "RealizationRecord"
    assert SumoTrajectoryAuthorityConfig.__name__ == "SumoTrajectoryAuthorityConfig"
    assert SumoSimulationResult.__name__ == "SumoSimulationResult"
    assert SumoClosedLoopRunnerConfig.__name__ == "SumoClosedLoopRunnerConfig"
    assert SumoClosedLoopSimulationResult.__name__ == "SumoClosedLoopSimulationResult"
    assert SumoArtifactResult.__name__ == "SumoArtifactResult"
    assert SumoNetworkConfig.__name__ == "SumoNetworkConfig"
    assert SumoNetworkFiles.__name__ == "SumoNetworkFiles"
    assert callable(build_sumo_network)
    assert callable(run_sumo_trajectory_authority_simulation)
    assert callable(run_trajectory_gui_replay)
    assert not hasattr(sumo, "P17SumoArtifactResult")
    assert not hasattr(sumo, "P17SumoNetworkConfig")
    assert not hasattr(sumo, "P17SumoNetworkFiles")
    assert not hasattr(sumo, "build_p17_sumo_network")


def test_top_level_package_stays_thin() -> None:
    import cormc

    assert cormc.__version__ == "0.1.0"
    for name in (
        "ControlledTrajectoryCommand",
        "SpawnCommand",
        "RealizationRecord",
        "SumoArtifactResult",
        "P17SumoArtifactResult",
        "run_sumo_trajectory_authority_simulation",
        "run_trajectory_gui_replay",
        "run_basic_numeric_suite",
        "run_random_6450_numeric_scenario",
        "run_one_step_fixed_scenario",
    ):
        assert not hasattr(cormc, name)
