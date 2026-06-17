"""Unified observation artifact, plot, and SUMO replay interfaces."""

from __future__ import annotations

from typing import Any


__all__ = [
    "ObservationArtifactBundle",
    "ObservationDataset",
    "ObservationLifecycle",
    "ObservationPlotArtifacts",
    "ObservationSumoReplayArtifacts",
    "ObservationTrajectoryRecord",
    "build_observation_artifact_bundle",
    "build_observation_plot_artifacts",
    "build_observation_sumo_replay_artifacts",
    "load_stage2_observation_dataset",
    "verify_observation_replay_fidelity",
    "write_observation_replay_jsonl",
]


def __getattr__(name: str) -> Any:
    if name in {"ObservationDataset", "ObservationLifecycle", "ObservationTrajectoryRecord"}:
        from cormc.observation import dataset

        return getattr(dataset, name)
    if name == "load_stage2_observation_dataset":
        from cormc.observation.stage2_artifacts import load_stage2_observation_dataset

        return load_stage2_observation_dataset
    if name in {"ObservationPlotArtifacts", "build_observation_plot_artifacts"}:
        from cormc.observation import plotting

        return getattr(plotting, name)
    if name in {
        "ObservationSumoReplayArtifacts",
        "build_observation_sumo_replay_artifacts",
        "verify_observation_replay_fidelity",
        "write_observation_replay_jsonl",
    }:
        from cormc.observation import sumo_replay

        return getattr(sumo_replay, name)
    if name in {"ObservationArtifactBundle", "build_observation_artifact_bundle"}:
        from cormc.observation import artifacts

        return getattr(artifacts, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
