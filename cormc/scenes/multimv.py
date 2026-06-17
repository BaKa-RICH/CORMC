from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping

from cormc.scenes.model import StaticSceneSpec, SceneVehicle, lane2, mv, vehicle


RM_MULTIMV_ORIGIN_X_GLOBAL = 6600.0
RM_MULTIMV_DEFAULT_SPEED = 20.0

RM_MULTIMV_M2_S01_SCENARIO_ID = "RM-M2-S01"
RM_MULTIMV_M2_S02_SCENARIO_ID = "RM-M2-S02"
RM_MULTIMV_M2_S03_SCENARIO_ID = "RM-M2-S03"
RM_MULTIMV_M2_S04_SCENARIO_ID = "RM-M2-S04"
RM_MULTIMV_M2_S05_SCENARIO_ID = "RM-M2-S05"
RM_MULTIMV_M2_S06_SCENARIO_ID = "RM-M2-S06"
RM_MULTIMV_M2_S07_SCENARIO_ID = "RM-M2-S07"
RM_MULTIMV_M2_S08_SCENARIO_ID = "RM-M2-S08"
RM_MULTIMV_M3_S01_SCENARIO_ID = "RM-M3-S01"
RM_MULTIMV_M3_S02_SCENARIO_ID = "RM-M3-S02"
RM_MULTIMV_M3_S03_SCENARIO_ID = "RM-M3-S03"
RM_MULTIMV_M3_S04_SCENARIO_ID = "RM-M3-S04"
RM_MULTIMV_M3_S05_SCENARIO_ID = "RM-M3-S05"
RM_MULTIMV_M3_S06_SCENARIO_ID = "RM-M3-S06"
RM_MULTIMV_M3_S07_SCENARIO_ID = "RM-M3-S07"
RM_MULTIMV_M4_S01_SCENARIO_ID = "RM-M4-S01"
RM_MULTIMV_M4_S02_SCENARIO_ID = "RM-M4-S02"
RM_MULTIMV_M4_S03_SCENARIO_ID = "RM-M4-S03"
RM_MULTIMV_M4_S04_SCENARIO_ID = "RM-M4-S04"
RM_MULTIMV_M4_S05_SCENARIO_ID = "RM-M4-S05"
RM_MULTIMV_M4_S06_SCENARIO_ID = "RM-M4-S06"

RM_MULTIMV_M2_SCENARIO_IDS = (
    RM_MULTIMV_M2_S01_SCENARIO_ID,
    RM_MULTIMV_M2_S02_SCENARIO_ID,
    RM_MULTIMV_M2_S03_SCENARIO_ID,
    RM_MULTIMV_M2_S04_SCENARIO_ID,
    RM_MULTIMV_M2_S05_SCENARIO_ID,
    RM_MULTIMV_M2_S06_SCENARIO_ID,
    RM_MULTIMV_M2_S07_SCENARIO_ID,
    RM_MULTIMV_M2_S08_SCENARIO_ID,
)
RM_MULTIMV_M3_SCENARIO_IDS = (
    RM_MULTIMV_M3_S01_SCENARIO_ID,
    RM_MULTIMV_M3_S02_SCENARIO_ID,
    RM_MULTIMV_M3_S03_SCENARIO_ID,
    RM_MULTIMV_M3_S04_SCENARIO_ID,
    RM_MULTIMV_M3_S05_SCENARIO_ID,
    RM_MULTIMV_M3_S06_SCENARIO_ID,
    RM_MULTIMV_M3_S07_SCENARIO_ID,
)
RM_MULTIMV_M4_SCENARIO_IDS = (
    RM_MULTIMV_M4_S01_SCENARIO_ID,
    RM_MULTIMV_M4_S02_SCENARIO_ID,
    RM_MULTIMV_M4_S03_SCENARIO_ID,
    RM_MULTIMV_M4_S04_SCENARIO_ID,
    RM_MULTIMV_M4_S05_SCENARIO_ID,
    RM_MULTIMV_M4_S06_SCENARIO_ID,
)
RM_MULTIMV_SCENARIO_IDS = (
    *RM_MULTIMV_M2_SCENARIO_IDS,
    *RM_MULTIMV_M3_SCENARIO_IDS,
    *RM_MULTIMV_M4_SCENARIO_IDS,
)


@dataclass(frozen=True)
class MultiMVCaseSpec:
    id: str
    scenario_id: str
    mv_count: int
    category: str
    title: str
    x_targets: tuple[float, ...]
    x_m_list: tuple[float, ...]
    planning_order: tuple[str, ...]
    purpose: str
    modules: tuple[str, ...]
    expected: dict[str, Any]
    final_x_targets_after_static_rolling: tuple[float, ...]
    hdv_lane2_indices: tuple[int, ...]


def load_multimv_case_specs() -> dict[str, MultiMVCaseSpec]:
    path = Path(__file__).with_name("data") / "multimv_rolling_cases.json"
    payload = json.loads(path.read_text(encoding="utf-8"))
    case_specs = {
        str(item["scenario_id"]): _case_spec_from_payload(item)
        for item in payload
    }
    _validate_case_specs(case_specs)
    return case_specs


def get_multimv_case_spec(scenario_id: str) -> MultiMVCaseSpec:
    try:
        return RM_MULTIMV_CASE_SPECS[scenario_id]
    except KeyError as exc:
        raise ValueError(f"unknown RM-MULTIMV scenario_id: {scenario_id}") from exc


def build_multimv_static_scene(case: MultiMVCaseSpec) -> StaticSceneSpec:
    vehicles = (
        *(
            mv(
                _mv_vehicle_id(case, index),
                RM_MULTIMV_ORIGIN_X_GLOBAL + x_m,
                speed=RM_MULTIMV_DEFAULT_SPEED,
            )
            for index, x_m in enumerate(case.x_m_list, start=1)
        ),
        *(
            _lane2_vehicle(case, index, x_target)
            for index, x_target in enumerate(case.x_targets, start=1)
        ),
    )
    return StaticSceneSpec(
        scenario_id=case.scenario_id,
        scenario_name=f"{case.scenario_id} {case.title}",
        purpose=case.purpose,
        description=f"{case.category}: {case.title}",
        test_level="integration",
        status="probe",
        derivation_ref=(
            "docs/one_step_algorithm/chatgpt-export_multimv_test_design",
            "multimv_rolling_cases",
        ),
        road_config_ref="paper_fig10_first_version",
        parameter_config_ref="paper_table_i_first_version",
        vehicles=vehicles,
        notes=(
            f"source_case_id={case.id}",
            f"planning_order={','.join(case.planning_order)}",
            "planning_order is metadata only; rolling order is decided by runtime triggers.",
        ),
    )


def multimv_mv_vehicle_ids(case: MultiMVCaseSpec) -> tuple[str, ...]:
    return tuple(_mv_vehicle_id(case, index) for index in range(1, case.mv_count + 1))


def multimv_lane2_vehicle_ids(case: MultiMVCaseSpec) -> tuple[str, ...]:
    return tuple(
        _lane2_vehicle_id(case, index)
        for index in range(1, len(case.x_targets) + 1)
    )


def _case_spec_from_payload(item: Mapping[str, Any]) -> MultiMVCaseSpec:
    return MultiMVCaseSpec(
        id=str(item["id"]),
        scenario_id=str(item["scenario_id"]),
        mv_count=int(item["mv_count"]),
        category=str(item["category"]),
        title=str(item["title"]),
        x_targets=tuple(float(value) for value in item["x_targets"]),
        x_m_list=tuple(float(value) for value in item["x_m_list"]),
        planning_order=tuple(str(value) for value in item["planning_order"]),
        purpose=str(item["purpose"]),
        modules=tuple(str(value) for value in item["modules"]),
        expected=dict(item["expected"]),
        final_x_targets_after_static_rolling=tuple(
            float(value) for value in item["final_x_targets_after_static_rolling"]
        ),
        hdv_lane2_indices=tuple(int(value) for value in item["hdv_lane2_indices"]),
    )


def _validate_case_specs(case_specs: Mapping[str, MultiMVCaseSpec]) -> None:
    scenario_ids = tuple(case_specs)
    if scenario_ids != RM_MULTIMV_SCENARIO_IDS:
        raise ValueError("multimv scenario ids do not match RM_MULTIMV_SCENARIO_IDS")
    for case in case_specs.values():
        if case.mv_count != len(case.x_m_list):
            raise ValueError(f"{case.scenario_id} mv_count does not match x_m_list")


def _lane2_vehicle(case: MultiMVCaseSpec, index: int, x_target: float) -> SceneVehicle:
    vehicle_id = _lane2_vehicle_id(case, index)
    x_global = RM_MULTIMV_ORIGIN_X_GLOBAL + x_target
    if index not in set(case.hdv_lane2_indices):
        return lane2(vehicle_id, x_global, speed=RM_MULTIMV_DEFAULT_SPEED)
    return vehicle(
        vehicle_id,
        vehicle_type="HDV",
        lane_id="lane_2",
        role="mainline",
        state="none",
        compliance_state="not_applicable",
        y=0.0,
        x=x_global,
        speed=RM_MULTIMV_DEFAULT_SPEED,
    )


def _mv_vehicle_id(case: MultiMVCaseSpec, index: int) -> str:
    return f"{case.id.replace('-', '_')}_MV_{index:02d}"


def _lane2_vehicle_id(case: MultiMVCaseSpec, index: int) -> str:
    return f"{case.id.replace('-', '_')}_L2_{index:02d}"


RM_MULTIMV_CASE_SPECS: Mapping[str, MultiMVCaseSpec] = MappingProxyType(
    load_multimv_case_specs()
)
RM_MULTIMV_SCENE_SPECS: Mapping[str, StaticSceneSpec] = MappingProxyType(
    {
        scenario_id: build_multimv_static_scene(case)
        for scenario_id, case in RM_MULTIMV_CASE_SPECS.items()
    }
)
