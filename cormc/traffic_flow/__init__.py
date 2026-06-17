"""Traffic-flow generation primitives."""

from cormc.traffic_flow.generation import (
    ArrivalStream,
    BoundaryQueueItem,
    SeededRandomProfile,
    SpawnDecision,
    compute_spawn_decisions,
    generate_boundary_queue,
    profile_from_mapping,
    profile_to_dict,
    queue_fingerprint,
)
from cormc.traffic_flow.source import BoundaryFlowSource, SeededRandomBoundaryFlowSource

__all__ = [
    "ArrivalStream",
    "BoundaryFlowSource",
    "BoundaryQueueItem",
    "SeededRandomBoundaryFlowSource",
    "SeededRandomProfile",
    "SpawnDecision",
    "compute_spawn_decisions",
    "generate_boundary_queue",
    "profile_from_mapping",
    "profile_to_dict",
    "queue_fingerprint",
]
