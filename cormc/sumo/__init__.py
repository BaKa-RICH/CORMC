"""SUMO integration helpers for P17."""

from cormc.sumo.commands import (
    ControlledTrajectoryCommand,
    SumoArtifactResult,
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
    SumoNetworkConfig,
    SumoNetworkFiles,
    build_sumo_network,
)
from cormc.sumo.trajectory_gui_replay import (
    TrajectoryGuiReplaySummary,
    run_trajectory_gui_replay,
)

__all__ = [
    "ControlledTrajectoryCommand",
    "EDGE_METADATA",
    "LANE_ROLE_MAP",
    "SumoArtifactResult",
    "SumoNetworkConfig",
    "SumoNetworkFiles",
    "RealizationRecord",
    "SpawnCommand",
    "SumoBinaryPaths",
    "SumoClosedLoopRunnerConfig",
    "SumoClosedLoopSimulationResult",
    "SumoSimulationResult",
    "SumoTrajectoryAuthorityConfig",
    "TrajectoryGuiReplaySummary",
    "build_sumo_network",
    "discover_sumo_binaries",
    "ensure_sumo_available_or_skip",
    "ensure_sumo_tools_on_path",
    "from_sumo_position",
    "get_sumo_version",
    "import_sumolib",
    "import_traci",
    "lane_index_for_role",
    "run_sumo_trajectory_authority_simulation",
    "run_trajectory_gui_replay",
    "to_sumo_position",
    "to_sumo_xy",
]
