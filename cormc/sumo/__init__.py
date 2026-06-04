"""SUMO integration helpers for P17."""

from cormc.sumo.commands import (
    ControlledTrajectoryCommand,
    P17SumoArtifactResult,
    RealizationRecord,
    SpawnCommand,
    SumoSimulationResult,
    SumoTrajectoryAuthorityConfig,
)
from cormc.sumo.env import (
    SumoBinaryPaths,
    discover_sumo_binaries,
    ensure_sumo_available_or_skip,
    ensure_sumo_tools_on_path,
    get_sumo_version,
    import_sumolib,
    import_traci,
)
from cormc.sumo.mapping import (
    EDGE_METADATA,
    LANE_ROLE_MAP,
    from_sumo_position,
    lane_index_for_role,
    to_sumo_position,
    to_sumo_xy,
)
from cormc.sumo.loop import SumoClosedLoopSimulationResult, SumoClosedLoopRunnerConfig, run_sumo_trajectory_authority_simulation
from cormc.sumo.network import (
    P17SumoNetworkConfig,
    P17SumoNetworkFiles,
    build_p17_sumo_network,
)


def run_p17_sumo_artifact_bundle(*args, **kwargs):
    from cormc.sumo.artifacts import run_p17_sumo_artifact_bundle as _run_p17_sumo_artifact_bundle

    return _run_p17_sumo_artifact_bundle(*args, **kwargs)

__all__ = [
    "ControlledTrajectoryCommand",
    "EDGE_METADATA",
    "LANE_ROLE_MAP",
    "P17SumoArtifactResult",
    "P17SumoNetworkConfig",
    "P17SumoNetworkFiles",
    "RealizationRecord",
    "SpawnCommand",
    "SumoBinaryPaths",
    "SumoClosedLoopRunnerConfig",
    "SumoClosedLoopSimulationResult",
    "SumoSimulationResult",
    "SumoTrajectoryAuthorityConfig",
    "build_p17_sumo_network",
    "discover_sumo_binaries",
    "ensure_sumo_available_or_skip",
    "ensure_sumo_tools_on_path",
    "from_sumo_position",
    "get_sumo_version",
    "import_sumolib",
    "import_traci",
    "lane_index_for_role",
    "run_p17_sumo_artifact_bundle",
    "run_sumo_trajectory_authority_simulation",
    "to_sumo_position",
    "to_sumo_xy",
]
