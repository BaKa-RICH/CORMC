from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from cormc.scenes.model import StaticSceneSpec, lane1, lane2, mv


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
        expected_active_cv_ids=("B05_CLV", "B05_CFV"),
        expected_eq10_consumer_ids=("B05_CFV",),
        control_activation_x_global=6850.0,
        max_time_s=50.0,
        method="ramp_pre first case 3 / refreshed local CFV Eq.10",
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


def get_basic_expectation(scenario_id: str) -> BasicScenarioExpectation:
    try:
        return BASIC_SCENARIO_EXPECTATIONS[scenario_id]
    except KeyError as exc:
        raise ValueError(f"unknown BASIC scenario_id: {scenario_id}") from exc


def _scene(
    scenario_id: str,
    *,
    description: str,
    vehicles: tuple[Any, ...],
    cuc_utility_overrides: Mapping[str, Mapping[str, Any]] | None = None,
    freeze_first_aps_assignment_until_cmc: bool = False,
) -> StaticSceneSpec:
    test_harness_overrides: dict[str, Any] = {"source": "basic_numeric_diagnostic"}
    if cuc_utility_overrides:
        test_harness_overrides["cuc_utility_overrides"] = dict(cuc_utility_overrides)
    if freeze_first_aps_assignment_until_cmc:
        test_harness_overrides["freeze_first_aps_assignment_until_cmc"] = True
    return StaticSceneSpec(
        scenario_id=scenario_id,
        scenario_name=description,
        description=description,
        purpose="BASIC numeric diagnostic scenario from docs/execution_plan/basic_6_scenarios",
        test_level="integration",
        status="probe",
        derivation_ref=("docs/execution_plan/basic_6_scenarios.md#5",),
        road_config_ref="paper_fig10_first_version",
        parameter_config_ref="paper_table_i_first_version",
        vehicles=vehicles,
        module_overrides={
            "test_harness_overrides": test_harness_overrides,
        },
        notes=(
            "CAV vehicles use compliance_state=not_applicable to satisfy current schema.",
            "BASIC scenarios are independent from the legacy required MVS suite.",
        ),
    )


def _stay_lane_2_cuc_overrides(*vehicle_ids: str) -> dict[str, dict[str, Any]]:
    return {
        vehicle_id: {
            "recommended_choice": "stay_lane_2",
            "U1": 0.0,
            "U2": 10000.0,
        }
        for vehicle_id in vehicle_ids
    }


def _basic_mv(vehicle_id: str, x: float, note: str):
    return mv(vehicle_id, x, note_key="basic_note", note=note)


def _basic_lane2(vehicle_id: str, x: float, note: str):
    return lane2(vehicle_id, x, note_key="basic_note", note=note)


def _basic_lane1_blocker(vehicle_id: str, x: float, note: str):
    return lane1(vehicle_id, x, note_key="basic_note", note=note)


BASIC_SCENE_SPECS: Mapping[str, StaticSceneSpec] = {
    "BASIC-01": _scene(
        "BASIC-01",
        description="pre-control case 2 / CFV Eq.10",
        vehicles=(
            _basic_mv("B01_MV", 6640.0, "pre-control initial MV"),
            _basic_lane2("B01_CLV", 6674.0, "relative +34 m"),
            _basic_lane2("B01_CFV", 6634.0, "relative -6 m"),
            _basic_lane1_blocker("B01_TLV_CFV", 6643.0, "makes CFV target lane unsafe"),
        ),
    ),
    "BASIC-02": _scene(
        "BASIC-02",
        description="pre-control case 3 / CLV no Eq.10",
        vehicles=(
            _basic_mv("B02_MV", 6640.0, "pre-control initial MV"),
            _basic_lane2("B02_CLV", 6654.0, "relative +14 m"),
            _basic_lane2("B02_CFV", 6560.0, "relative -80 m"),
            _basic_lane1_blocker("B02_TLV_CLV", 6663.0, "makes CLV target lane unsafe"),
        ),
        cuc_utility_overrides=_stay_lane_2_cuc_overrides("B02_CLV"),
        freeze_first_aps_assignment_until_cmc=True,
    ),
    "BASIC-03": _scene(
        "BASIC-03",
        description="pre-control case 4 / CLV + CFV",
        vehicles=(
            _basic_mv("B03_MV", 6640.0, "pre-control initial MV"),
            _basic_lane2("B03_CLV", 6654.0, "relative +14 m"),
            _basic_lane2("B03_CFV", 6634.0, "relative -6 m"),
            _basic_lane1_blocker("B03_TLV_CFV", 6643.0, "makes CFV target lane unsafe"),
            _basic_lane1_blocker("B03_TLV_CLV", 6663.0, "makes CLV target lane unsafe"),
        ),
        cuc_utility_overrides=_stay_lane_2_cuc_overrides("B03_CLV", "B03_CFV"),
    ),
    "BASIC-04": _scene(
        "BASIC-04",
        description="ramp_pre case 2 / CFV Eq.10",
        vehicles=(
            _basic_mv("B04_MV", 6850.0, "ramp_pre initial MV"),
            _basic_lane2("B04_CLV", 6884.0, "relative +34 m"),
            _basic_lane2("B04_CFV", 6844.0, "relative -6 m"),
            _basic_lane1_blocker("B04_TLV_CFV", 6853.0, "makes CFV target lane unsafe"),
        ),
        cuc_utility_overrides=_stay_lane_2_cuc_overrides("B04_CFV"),
    ),
    "BASIC-05": _scene(
        "BASIC-05",
        description="ramp_pre case 3 / CLV no Eq.10",
        vehicles=(
            _basic_mv("B05_MV", 6850.0, "ramp_pre initial MV"),
            _basic_lane2("B05_CLV", 6864.0, "relative +14 m"),
            _basic_lane2("B05_CFV", 6824.0, "relative -26 m"),
            _basic_lane1_blocker("B05_TLV_CLV", 6873.0, "makes CLV target lane unsafe"),
        ),
        cuc_utility_overrides=_stay_lane_2_cuc_overrides("B05_CLV", "B05_CFV"),
    ),
    "BASIC-06": _scene(
        "BASIC-06",
        description="ramp_pre case 4 / CLV + CFV",
        vehicles=(
            _basic_mv("B06_MV", 6850.0, "ramp_pre initial MV"),
            _basic_lane2("B06_CLV", 6864.0, "relative +14 m"),
            _basic_lane2("B06_CFV", 6844.0, "relative -6 m"),
            _basic_lane1_blocker("B06_TLV_CFV", 6853.0, "makes CFV target lane unsafe"),
            _basic_lane1_blocker("B06_TLV_CLV", 6873.0, "makes CLV target lane unsafe"),
        ),
        cuc_utility_overrides=_stay_lane_2_cuc_overrides("B06_CLV", "B06_CFV"),
    ),
}
