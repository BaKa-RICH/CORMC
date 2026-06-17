from __future__ import annotations

from cormc.onestep.kernel.config import ScenarioConfig
from cormc.onestep.kernel.models import Gap


def build_gaps(scenario: ScenarioConfig) -> tuple[Gap, ...]:
    gaps: list[Gap] = []
    controllability_by_index = {
        item.gap_index: item
        for item in getattr(scenario, "gap_boundary_controllability", ())
    }
    for index, (x_a, x_b) in enumerate(zip(scenario.x_targets, scenario.x_targets[1:])):
        controllability = controllability_by_index.get(index)
        x_rear = min(x_a, x_b)
        x_front = max(x_a, x_b)
        G_i = x_front - x_rear
        c_i = (x_front + x_rear) / 2.0
        gaps.append(
            Gap(
                gap_id=f"gap{index + 1}",
                index=index,
                x_rear=x_rear,
                x_front=x_front,
                G_i=G_i,
                c_i=c_i,
                D_i=c_i - scenario.x_m0,
                front_controllable=(
                    controllability.front_controllable
                    if controllability is not None
                    else True
                ),
                rear_controllable=(
                    controllability.rear_controllable
                    if controllability is not None
                    else True
                ),
                front_vehicle_id=(
                    controllability.front_vehicle_id
                    if controllability is not None
                    else None
                ),
                rear_vehicle_id=(
                    controllability.rear_vehicle_id
                    if controllability is not None
                    else None
                ),
            )
        )
    return tuple(gaps)
