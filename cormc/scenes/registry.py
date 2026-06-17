from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from cormc.scenes.basic import BASIC_SCENE_SPECS, BASIC_SCENARIO_IDS
from cormc.scenes.compiler import compile_static_scene, compile_traffic_flow_scene
from cormc.scenes.multimv import RM_MULTIMV_SCENE_SPECS, RM_MULTIMV_SCENARIO_IDS
from cormc.scenes.onestep import RM_ONESTEP_SCENE_SPECS, RM_ONESTEP_SCENARIO_IDS
from cormc.scenes.random_flow import TRAFFIC_FLOW_SCENE_IDS, TRAFFIC_FLOW_SCENE_SPECS
from cormc.scenes.rolling_basic import ROLLING_BASIC_SCENE_SPEC, ROLLING_BASIC_SCENARIO_ID
from cormc.scenes.model import StaticSceneSpec, TrafficFlowSceneSpec


@dataclass(frozen=True)
class SceneMetadata:
    scenario_id: str
    family: str
    static: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "family": self.family,
            "static": self.static,
        }


STATIC_SCENE_IDS: tuple[str, ...] = (
    *BASIC_SCENARIO_IDS,
    ROLLING_BASIC_SCENARIO_ID,
    *RM_ONESTEP_SCENARIO_IDS,
    *RM_MULTIMV_SCENARIO_IDS,
)

STATIC_SCENE_SPECS: Mapping[str, StaticSceneSpec] = {
    **dict(BASIC_SCENE_SPECS),
    ROLLING_BASIC_SCENARIO_ID: ROLLING_BASIC_SCENE_SPEC,
    **dict(RM_ONESTEP_SCENE_SPECS),
    **dict(RM_MULTIMV_SCENE_SPECS),
}

STATIC_SCENE_METADATA: Mapping[str, SceneMetadata] = {
    **{
        scenario_id: SceneMetadata(scenario_id=scenario_id, family="basic")
        for scenario_id in BASIC_SCENARIO_IDS
    },
    ROLLING_BASIC_SCENARIO_ID: SceneMetadata(
        scenario_id=ROLLING_BASIC_SCENARIO_ID,
        family="rolling_basic",
    ),
    **{
        scenario_id: SceneMetadata(scenario_id=scenario_id, family="rm_onestep")
        for scenario_id in RM_ONESTEP_SCENARIO_IDS
    },
    **{
        scenario_id: SceneMetadata(scenario_id=scenario_id, family="rm_multimv")
        for scenario_id in RM_MULTIMV_SCENARIO_IDS
    },
}


def load_scene_config(scenario_id: str) -> dict[str, Any]:
    try:
        return compile_static_scene(STATIC_SCENE_SPECS[scenario_id])
    except KeyError as exc:
        raise ValueError(f"unknown static scene_id: {scenario_id}") from exc


def load_traffic_flow_scene_config(scenario_id: str) -> dict[str, Any]:
    try:
        return compile_traffic_flow_scene(TRAFFIC_FLOW_SCENE_SPECS[scenario_id])
    except KeyError as exc:
        raise ValueError(f"unknown traffic flow scene_id: {scenario_id}") from exc


def get_traffic_flow_scene_spec(scenario_id: str) -> TrafficFlowSceneSpec:
    try:
        return TRAFFIC_FLOW_SCENE_SPECS[scenario_id]
    except KeyError as exc:
        raise ValueError(f"unknown traffic flow scene_id: {scenario_id}") from exc


def load_scene_metadata(scenario_id: str) -> SceneMetadata:
    try:
        return STATIC_SCENE_METADATA[scenario_id]
    except KeyError as exc:
        raise ValueError(f"unknown static scene_id: {scenario_id}") from exc
