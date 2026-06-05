from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Mapping

from cormc.mvs.loader import load_scenario_config


BASIC_SCENARIO_IDS: tuple[str, ...] = (
    "BASIC-01",
    "BASIC-02",
    "BASIC-03",
    "BASIC-04",
    "BASIC-05",
    "BASIC-06",
)

BASIC_PRE_CONTROL_IDS: tuple[str, ...] = ("BASIC-01", "BASIC-02", "BASIC-03")
BASIC_RAMP_PRE_IDS: tuple[str, ...] = ("BASIC-04", "BASIC-05", "BASIC-06")


@dataclass(frozen=True)
class BasicScenarioExpectation:
    scenario_id: str
    mv_id: str
    expected_aps_case: str
    expected_active_cv_ids: tuple[str, ...]
    expected_eq10_consumer_ids: tuple[str, ...]
    control_activation_x_global: float
    max_time_s: float
    method: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "mv_id": self.mv_id,
            "expected_aps_case": self.expected_aps_case,
            "expected_active_cv_ids": list(self.expected_active_cv_ids),
            "expected_eq10_consumer_ids": list(self.expected_eq10_consumer_ids),
            "control_activation_x_global": self.control_activation_x_global,
            "max_time_s": self.max_time_s,
            "method": self.method,
        }


BASIC_SCENARIO_EXPECTATIONS: Mapping[str, BasicScenarioExpectation] = {
    "BASIC-01": BasicScenarioExpectation(
        scenario_id="BASIC-01",
        mv_id="B01_MV",
        expected_aps_case="case_2",
        expected_active_cv_ids=("B01_CFV",),
        expected_eq10_consumer_ids=("B01_CFV",),
        control_activation_x_global=6650.0,
        max_time_s=80.0,
        method="pre-control case 2 / CFV Eq.10",
    ),
    "BASIC-02": BasicScenarioExpectation(
        scenario_id="BASIC-02",
        mv_id="B02_MV",
        expected_aps_case="case_3",
        expected_active_cv_ids=("B02_CLV",),
        expected_eq10_consumer_ids=(),
        control_activation_x_global=6650.0,
        max_time_s=80.0,
        method="pre-control case 3 / CLV no Eq.10",
    ),
    "BASIC-03": BasicScenarioExpectation(
        scenario_id="BASIC-03",
        mv_id="B03_MV",
        expected_aps_case="case_4",
        expected_active_cv_ids=("B03_CLV", "B03_CFV"),
        expected_eq10_consumer_ids=("B03_CFV",),
        control_activation_x_global=6650.0,
        max_time_s=80.0,
        method="pre-control case 4 / CLV + CFV",
    ),
    "BASIC-04": BasicScenarioExpectation(
        scenario_id="BASIC-04",
        mv_id="B04_MV",
        expected_aps_case="case_2",
        expected_active_cv_ids=("B04_CFV",),
        expected_eq10_consumer_ids=("B04_CFV",),
        control_activation_x_global=6850.0,
        max_time_s=50.0,
        method="ramp_pre case 2 / CFV Eq.10",
    ),
    "BASIC-05": BasicScenarioExpectation(
        scenario_id="BASIC-05",
        mv_id="B05_MV",
        expected_aps_case="case_3",
        expected_active_cv_ids=("B05_CLV",),
        expected_eq10_consumer_ids=(),
        control_activation_x_global=6850.0,
        max_time_s=50.0,
        method="ramp_pre case 3 / CLV no Eq.10",
    ),
    "BASIC-06": BasicScenarioExpectation(
        scenario_id="BASIC-06",
        mv_id="B06_MV",
        expected_aps_case="case_4",
        expected_active_cv_ids=("B06_CLV", "B06_CFV"),
        expected_eq10_consumer_ids=("B06_CFV",),
        control_activation_x_global=6850.0,
        max_time_s=50.0,
        method="ramp_pre case 4 / CLV + CFV",
    ),
}


def load_basic_scenario(scenario_id: str) -> dict[str, Any]:
    try:
        raw = BASIC_SCENARIO_CONFIGS[scenario_id]
    except KeyError as exc:
        raise ValueError(f"unknown BASIC scenario_id: {scenario_id}") from exc
    return load_scenario_config(deepcopy(raw))


def load_basic_scenarios() -> dict[str, dict[str, Any]]:
    return {scenario_id: load_basic_scenario(scenario_id) for scenario_id in BASIC_SCENARIO_IDS}


def get_basic_expectation(scenario_id: str) -> BasicScenarioExpectation:
    try:
        return BASIC_SCENARIO_EXPECTATIONS[scenario_id]
    except KeyError as exc:
        raise ValueError(f"unknown BASIC scenario_id: {scenario_id}") from exc


def _base_config(
    scenario_id: str,
    *,
    description: str,
    vehicles: list[dict[str, Any]],
) -> dict[str, Any]:
    return {
        "scenario_id": scenario_id,
        "scenario_name": description,
        "description": description,
        "purpose": "BASIC numeric diagnostic scenario from docs/执行计划/6个基础场景.md",
        "test_level": "integration",
        "status": "probe",
        "derivation_ref": ["docs/执行计划/6个基础场景.md#5"],
        "road_config_ref": "paper_fig10_first_version",
        "parameter_config_ref": "paper_table_i_first_version",
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": vehicles,
        "module_overrides": {
            "boundary_generation_enabled": False,
            "random_arrival_enabled": False,
            "random_vehicle_attributes_enabled": False,
            "ordinary_mainline_lane_change_enabled": False,
            "platoon_cmc_enabled": False,
            "mpc_lateral_tracking_enabled": False,
            "test_harness_overrides": {"source": "basic_numeric_diagnostic"},
        },
        "preloaded_assignments": [],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": [],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [],
        "expected_png_features": [],
        "notes": [
            "CAV vehicles use compliance_state=not_applicable to satisfy current schema.",
            "BASIC scenarios are independent from the legacy required MVS suite.",
        ],
    }


def _vehicle(
    vehicle_id: str,
    *,
    x_global: float,
    y: float,
    v: float,
    physical_lane: str,
    road_role: str,
    merge_state: str,
    note: str,
) -> dict[str, Any]:
    return {
        "vehicle_id": vehicle_id,
        "vehicle_type": "CAV",
        "compliance_state": "not_applicable",
        "initial_x_global": float(x_global),
        "initial_y": float(y),
        "initial_v": float(v),
        "initial_a": 0.0,
        "physical_lane": physical_lane,
        "road_role": road_role,
        "lane_change_state": "normal",
        "merge_state": merge_state,
        "spec_overrides": {"basic_note": note},
    }


def _mv(vehicle_id: str, x_global: float, note: str) -> dict[str, Any]:
    return _vehicle(
        vehicle_id,
        x_global=x_global,
        y=-3.5,
        v=20.0,
        physical_lane="on_ramp",
        road_role="on_ramp_mv",
        merge_state="not_started",
        note=note,
    )


def _lane2(vehicle_id: str, x_global: float, note: str) -> dict[str, Any]:
    return _vehicle(
        vehicle_id,
        x_global=x_global,
        y=0.0,
        v=20.0,
        physical_lane="lane_2",
        road_role="mainline",
        merge_state="none",
        note=note,
    )


def _lane1_blocker(vehicle_id: str, x_global: float, note: str) -> dict[str, Any]:
    return _vehicle(
        vehicle_id,
        x_global=x_global,
        y=3.5,
        v=15.0,
        physical_lane="lane_1",
        road_role="mainline",
        merge_state="none",
        note=note,
    )


BASIC_SCENARIO_CONFIGS: dict[str, dict[str, Any]] = {
    "BASIC-01": _base_config(
        "BASIC-01",
        description="pre-control case 2 / CFV Eq.10",
        vehicles=[
            _mv("B01_MV", 6640.0, "pre-control initial MV"),
            _lane2("B01_CLV", 6674.0, "relative +34 m"),
            _lane2("B01_CFV", 6634.0, "relative -6 m"),
            _lane1_blocker("B01_TLV_CFV", 6643.0, "makes CFV target lane unsafe"),
        ],
    ),
    "BASIC-02": _base_config(
        "BASIC-02",
        description="pre-control case 3 / CLV no Eq.10",
        vehicles=[
            _mv("B02_MV", 6640.0, "pre-control initial MV"),
            _lane2("B02_CLV", 6654.0, "relative +14 m"),
            _lane2("B02_CFV", 6614.0, "relative -26 m"),
            _lane1_blocker("B02_TLV_CLV", 6663.0, "makes CLV target lane unsafe"),
        ],
    ),
    "BASIC-03": _base_config(
        "BASIC-03",
        description="pre-control case 4 / CLV + CFV",
        vehicles=[
            _mv("B03_MV", 6640.0, "pre-control initial MV"),
            _lane2("B03_CLV", 6654.0, "relative +14 m"),
            _lane2("B03_CFV", 6634.0, "relative -6 m"),
            _lane1_blocker("B03_TLV_CFV", 6643.0, "makes CFV target lane unsafe"),
            _lane1_blocker("B03_TLV_CLV", 6663.0, "makes CLV target lane unsafe"),
        ],
    ),
    "BASIC-04": _base_config(
        "BASIC-04",
        description="ramp_pre case 2 / CFV Eq.10",
        vehicles=[
            _mv("B04_MV", 6850.0, "ramp_pre initial MV"),
            _lane2("B04_CLV", 6884.0, "relative +34 m"),
            _lane2("B04_CFV", 6844.0, "relative -6 m"),
            _lane1_blocker("B04_TLV_CFV", 6853.0, "makes CFV target lane unsafe"),
        ],
    ),
    "BASIC-05": _base_config(
        "BASIC-05",
        description="ramp_pre case 3 / CLV no Eq.10",
        vehicles=[
            _mv("B05_MV", 6850.0, "ramp_pre initial MV"),
            _lane2("B05_CLV", 6864.0, "relative +14 m"),
            _lane2("B05_CFV", 6824.0, "relative -26 m"),
            _lane1_blocker("B05_TLV_CLV", 6873.0, "makes CLV target lane unsafe"),
        ],
    ),
    "BASIC-06": _base_config(
        "BASIC-06",
        description="ramp_pre case 4 / CLV + CFV",
        vehicles=[
            _mv("B06_MV", 6850.0, "ramp_pre initial MV"),
            _lane2("B06_CLV", 6864.0, "relative +14 m"),
            _lane2("B06_CFV", 6844.0, "relative -6 m"),
            _lane1_blocker("B06_TLV_CFV", 6853.0, "makes CFV target lane unsafe"),
            _lane1_blocker("B06_TLV_CLV", 6873.0, "makes CLV target lane unsafe"),
        ],
    ),
}
