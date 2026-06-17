from __future__ import annotations

from cormc.onestep.kernel.config import AlgorithmConfig, ScenarioConfig
from cormc.onestep.kernel.models import DerivedParams


def compute_derived_params(
    scenario: ScenarioConfig,
    algorithm: AlgorithmConfig,
) -> DerivedParams:
    return DerivedParams(
        b=-scenario.a_min,
        G_req=2.0 * algorithm.D_h + algorithm.l_m,
        G_adj=algorithm.D_h + algorithm.l_m,
    )
