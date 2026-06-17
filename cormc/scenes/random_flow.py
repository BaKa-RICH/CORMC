from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Literal

from cormc.scenes.model import (
    FlowStopConditionSpec,
    FlowValidationSpec,
    TrafficFlowSceneSpec,
)
from cormc.scenes.onestep import (
    RM_ONESTEP_CASE_SPECS,
    RM_ONESTEP_S07_2MV_REAR_MV_ID,
    RM_ONESTEP_S07_MV_ID,
    _scene,
)


RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID = (
    "RM-ONESTEP-RANDOM-S07-LANE2-RAMP-100S"
)
DEFAULT_ONESTEP_RANDOM_SEED = 645001
DEFAULT_ONESTEP_RANDOM_MAX_STEPS = 1000
DEFAULT_ONESTEP_RANDOM_HORIZON_S = 100.0

DensityName = Literal["medium", "high"]


@dataclass(frozen=True)
class OnestepRandomBoundaryFlowSource:
    source_id: str
    seed: int = DEFAULT_ONESTEP_RANDOM_SEED
    density: DensityName = "medium"
    lane2_mean_headway: float | None = None
    ramp_mean_headway: float | None = None
    lane2_initial_speed: float = 20.0
    ramp_initial_speed: float = 20.0
    desired_speed_mean: float = 30.0
    desired_speed_std: float = 1.5
    safe_spawn_gap_m: float = 20.0
    source_type: str = "seeded_random"

    def build_queue(self, horizon_s: float) -> tuple[Any, ...]:
        from cormc.traffic_flow.generation import generate_boundary_queue

        return generate_boundary_queue(
            self._profile(),
            max_t=float(horizon_s),
            start_t=0.0,
            start_step=0,
        )

    def to_summary(self) -> dict[str, Any]:
        from cormc.traffic_flow.generation import profile_to_dict

        profile = self._profile()
        return {
            "source_id": self.source_id,
            "source_type": self.source_type,
            "seed": profile.seed,
            "profile_id": profile.profile_id,
            "arrival_streams": [
                {
                    "lane_id": stream.lane_id,
                    "road_role": stream.road_role,
                    "merge_state": stream.merge_state,
                    "spawn_x": stream.spawn_x,
                    "spawn_y": stream.spawn_y,
                    "initial_speed": stream.initial_speed,
                    "shifted_headway": stream.shifted_headway,
                    "mean_headway": stream.mean_headway,
                    "flow_policy": stream.flow_policy,
                    "vehicle_id_prefix": stream.vehicle_id_prefix,
                    "vehicle_id_lane_label": stream.vehicle_id_lane_label,
                }
                for stream in profile.arrival_streams
            ],
            "safe_spawn_gap_m": profile.safe_spawn_gap_m,
            "cav_penetration_rate": profile.cav_penetration_rate,
            "chv_compliance_rate": profile.chv_compliance_rate,
            "desired_speed_mean": profile.desired_speed_mean,
            "desired_speed_std": profile.desired_speed_std,
            "profile": profile_to_dict(profile),
        }

    def _profile(self) -> Any:
        return default_onestep_random_lane2_ramp_profile(
            seed=self.seed,
            density=self.density,
            lane2_mean_headway=self.lane2_mean_headway,
            ramp_mean_headway=self.ramp_mean_headway,
            lane2_initial_speed=self.lane2_initial_speed,
            ramp_initial_speed=self.ramp_initial_speed,
            desired_speed_mean=self.desired_speed_mean,
            desired_speed_std=self.desired_speed_std,
            safe_spawn_gap_m=self.safe_spawn_gap_m,
        )


def default_onestep_random_lane2_ramp_profile(
    *,
    seed: int = DEFAULT_ONESTEP_RANDOM_SEED,
    density: DensityName = "medium",
    lane2_mean_headway: float | None = None,
    ramp_mean_headway: float | None = None,
    lane2_initial_speed: float = 20.0,
    ramp_initial_speed: float = 20.0,
    desired_speed_mean: float = 30.0,
    desired_speed_std: float = 1.5,
    safe_spawn_gap_m: float = 20.0,
    enabled: bool = True,
) -> Any:
    from cormc.traffic_flow.generation import ArrivalStream, SeededRandomProfile

    preset = _density_preset(density)
    lane2_headway = float(
        lane2_mean_headway
        if lane2_mean_headway is not None
        else preset["lane2_mean_headway"]
    )
    ramp_headway = float(
        ramp_mean_headway
        if ramp_mean_headway is not None
        else preset["ramp_mean_headway"]
    )
    return SeededRandomProfile(
        enabled=enabled,
        seed=int(seed),
        profile_id=f"onestep_random_lane2_ramp_{density}_v1",
        arrival_streams=(
            ArrivalStream(
                lane_id="lane_2",
                road_role="mainline",
                merge_state="none",
                spawn_x=6450.0,
                spawn_y=0.0,
                initial_speed=float(lane2_initial_speed),
                shifted_headway=1.2,
                mean_headway=lane2_headway,
                vehicle_id_prefix=f"onestep_random_{seed}",
                vehicle_id_lane_label="lane_2",
            ),
            ArrivalStream(
                lane_id="on_ramp",
                road_role="on_ramp_mv",
                merge_state="not_started",
                spawn_x=6450.0,
                spawn_y=-3.5,
                initial_speed=float(ramp_initial_speed),
                shifted_headway=3.5,
                mean_headway=ramp_headway,
                vehicle_id_prefix=f"onestep_random_{seed}",
                vehicle_id_lane_label="on_ramp",
            ),
        ),
        safe_spawn_gap_m=float(safe_spawn_gap_m),
        cav_penetration_rate=0.60,
        chv_compliance_rate=0.75,
        desired_speed_mean=float(desired_speed_mean),
        desired_speed_std=float(desired_speed_std),
        max_queue_items_per_lane=512,
    )


def build_onestep_random_lane2_ramp_scene(
    *,
    seed: int = DEFAULT_ONESTEP_RANDOM_SEED,
    density: DensityName = "medium",
    max_steps: int = DEFAULT_ONESTEP_RANDOM_MAX_STEPS,
    horizon_s: float = DEFAULT_ONESTEP_RANDOM_HORIZON_S,
    lane2_mean_headway: float | None = None,
    ramp_mean_headway: float | None = None,
    lane2_initial_speed: float = 20.0,
    ramp_initial_speed: float = 20.0,
    desired_speed_mean: float = 30.0,
    desired_speed_std: float = 1.5,
    safe_spawn_gap_m: float = 20.0,
) -> TrafficFlowSceneSpec:
    static = _scene(
        RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID,
        RM_ONESTEP_CASE_SPECS["S07"],
        description="OneStep Stage2 seeded random S07 lane_2 + on-ramp 100s",
        mv_x_global=RM_ONESTEP_CASE_SPECS["S07"].rolling_entry_mv_x_global,
        mainline_origin_x_global=RM_ONESTEP_CASE_SPECS[
            "S07"
        ].rolling_entry_mainline_origin_x_global,
        extra_mvs=((RM_ONESTEP_S07_2MV_REAR_MV_ID, 6540.0),),
    )
    return TrafficFlowSceneSpec(
        scenario_id=RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID,
        scenario_name="OneStep Stage2 seeded random S07 lane_2 + on-ramp 100s",
        purpose=(
            "Formal One-Step Stage2 random high-flow validation with continuous "
            "lane_2 and on-ramp boundary generation."
        ),
        initial_vehicles=static.vehicles,
        boundary_flow_source=OnestepRandomBoundaryFlowSource(
            source_id=f"onestep-random-s07-{density}-seed{seed}",
            seed=seed,
            density=density,
            lane2_mean_headway=lane2_mean_headway,
            ramp_mean_headway=ramp_mean_headway,
            lane2_initial_speed=lane2_initial_speed,
            ramp_initial_speed=ramp_initial_speed,
            desired_speed_mean=desired_speed_mean,
            desired_speed_std=desired_speed_std,
            safe_spawn_gap_m=safe_spawn_gap_m,
        ),
        safe_spawn_gap_m=float(safe_spawn_gap_m),
        stop_condition=FlowStopConditionSpec(
            mode="max_steps",
            max_steps=int(max_steps),
            horizon_s=float(horizon_s),
        ),
        validation=FlowValidationSpec(
            min_generated_lane2_count=1,
            min_generated_on_ramp_mv_count=1,
            min_completed_mv_count=1,
            allow_open_mvs_at_horizon=True,
        ),
        derivation_ref=(
            "RM-ONESTEP-S07-2MV-ROLLING-ENTRY warm start",
            "seeded boundary flow source",
            "one-step stage2 random formal validation",
        ),
        notes=(
            f"Warm start includes {RM_ONESTEP_S07_MV_ID} and "
            f"{RM_ONESTEP_S07_2MV_REAR_MV_ID}.",
            "Boundary queue is generated once for the full horizon.",
        ),
    )


def get_onestep_random_flow_scene_spec(
    scenario_id: str = RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID,
) -> TrafficFlowSceneSpec:
    try:
        return TRAFFIC_FLOW_SCENE_SPECS[scenario_id]
    except KeyError as exc:
        raise ValueError(f"unknown traffic flow scene_id: {scenario_id}") from exc


def _density_preset(density: DensityName) -> dict[str, float]:
    if density == "medium":
        return {"lane2_mean_headway": 2.2, "ramp_mean_headway": 6.0}
    if density == "high":
        return {"lane2_mean_headway": 1.5, "ramp_mean_headway": 4.2}
    raise ValueError(f"unsupported onestep random density: {density}")


TRAFFIC_FLOW_SCENE_SPECS = MappingProxyType(
    {
        RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID: build_onestep_random_lane2_ramp_scene()
    }
)
TRAFFIC_FLOW_SCENE_IDS = tuple(TRAFFIC_FLOW_SCENE_SPECS)
