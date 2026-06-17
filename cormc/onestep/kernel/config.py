from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GapBoundaryControllability:
    gap_index: int
    front_controllable: bool
    rear_controllable: bool
    front_vehicle_id: str | None = None
    rear_vehicle_id: str | None = None


@dataclass(frozen=True)
class ScenarioConfig:
    x_targets: tuple[float, ...]
    x_m0: float
    v_ref: float
    v_max: float
    v_min: float
    a_max: float
    a_min: float
    T: float
    gap_boundary_controllability: tuple[GapBoundaryControllability, ...] = ()


@dataclass(frozen=True)
class AlgorithmConfig:
    D_h: float
    l_m: float
    w_c: float
    w_e: float
    w_t: float
    delta_ref: float
    q: float
    epsilon_delta: float
    K: float
    boundary_adjustment: float
