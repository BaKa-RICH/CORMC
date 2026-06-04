from __future__ import annotations


def test_p17_public_api_exports_from_sumo_package() -> None:
    from cormc.sumo import (
        ControlledTrajectoryCommand,
        P17SumoArtifactResult,
        RealizationRecord,
        SpawnCommand,
        SumoClosedLoopRunnerConfig,
        SumoClosedLoopSimulationResult,
        SumoSimulationResult,
        SumoTrajectoryAuthorityConfig,
        run_p17_sumo_artifact_bundle,
        run_sumo_trajectory_authority_simulation,
    )

    assert ControlledTrajectoryCommand.__name__ == "ControlledTrajectoryCommand"
    assert SpawnCommand.__name__ == "SpawnCommand"
    assert RealizationRecord.__name__ == "RealizationRecord"
    assert SumoTrajectoryAuthorityConfig.__name__ == "SumoTrajectoryAuthorityConfig"
    assert SumoSimulationResult.__name__ == "SumoSimulationResult"
    assert SumoClosedLoopRunnerConfig.__name__ == "SumoClosedLoopRunnerConfig"
    assert SumoClosedLoopSimulationResult.__name__ == "SumoClosedLoopSimulationResult"
    assert P17SumoArtifactResult.__name__ == "P17SumoArtifactResult"
    assert callable(run_sumo_trajectory_authority_simulation)
    assert callable(run_p17_sumo_artifact_bundle)


def test_p17_public_api_exports_from_top_level_package() -> None:
    import cormc

    assert cormc.ControlledTrajectoryCommand.__name__ == "ControlledTrajectoryCommand"
    assert cormc.SpawnCommand.__name__ == "SpawnCommand"
    assert cormc.RealizationRecord.__name__ == "RealizationRecord"
    assert cormc.SumoTrajectoryAuthorityConfig.__name__ == "SumoTrajectoryAuthorityConfig"
    assert cormc.SumoSimulationResult.__name__ == "SumoSimulationResult"
    assert cormc.SumoClosedLoopRunnerConfig.__name__ == "SumoClosedLoopRunnerConfig"
    assert cormc.SumoClosedLoopSimulationResult.__name__ == "SumoClosedLoopSimulationResult"
    assert cormc.P17SumoArtifactResult.__name__ == "P17SumoArtifactResult"
    assert callable(cormc.run_sumo_trajectory_authority_simulation)
    assert callable(cormc.run_p17_sumo_artifact_bundle)
