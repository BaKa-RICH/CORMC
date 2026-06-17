from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from cormc.scenes.model import StaticSceneSpec, lane1, lane2, mv


ROLLING_BASIC_SCENARIO_ID = "ROLLING-BASIC-04"
ROLLING_BASIC_MV_IDS: tuple[str, ...] = (
    "RB01_MV",
    "RB02_MV",
    "RB03_MV",
    "RB04_MV",
)
ROLLING_BASIC_MAINLINE_VEHICLE_IDS: tuple[str, ...] = (
    "RB01_CLV",
    "RB01_CFV",
    "RB01_TLV_CFV",
    "RB02_CLV",
    "RB02_CFV",
    "RB02_TLV_CFV",
    "RB03_CLV",
    "RB03_CFV",
    "RB03_TLV_CLV",
    "RB04_CLV",
    "RB04_CFV",
    "RB04_TLV_CFV",
    "RB04_TLV_CLV",
)
ROLLING_BASIC_DERIVATION_REF = "docs/execution_plan/rolling_multi_basic_scenario.md"


@dataclass(frozen=True)
class RollingBasicMVExpectation:
    scenario_id: str
    mv_id: str
    expected_aps_case: str
    expected_clv_id: str
    expected_cfv_id: str
    expected_active_cv_ids: tuple[str, ...]
    expected_eq10_consumer_ids: tuple[str, ...]
    control_activation_x_global: float = 6650.0
    max_time_s: float = 120.0
    method: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "mv_id": self.mv_id,
            "expected_aps_case": self.expected_aps_case,
            "expected_clv_id": self.expected_clv_id,
            "expected_cfv_id": self.expected_cfv_id,
            "expected_active_cv_ids": list(self.expected_active_cv_ids),
            "expected_eq10_consumer_ids": list(self.expected_eq10_consumer_ids),
            "control_activation_x_global": self.control_activation_x_global,
            "max_time_s": self.max_time_s,
            "method": self.method,
        }


ROLLING_BASIC_EXPECTATIONS: Mapping[str, RollingBasicMVExpectation] = {
    "RB01_MV": RollingBasicMVExpectation(
        scenario_id=ROLLING_BASIC_SCENARIO_ID,
        mv_id="RB01_MV",
        expected_aps_case="case_2",
        expected_clv_id="RB01_CLV",
        expected_cfv_id="RB01_CFV",
        expected_active_cv_ids=("RB01_CFV",),
        expected_eq10_consumer_ids=("RB01_CFV",),
        method="rolling RB01 / B04-like control-zone case 2",
    ),
    "RB02_MV": RollingBasicMVExpectation(
        scenario_id=ROLLING_BASIC_SCENARIO_ID,
        mv_id="RB02_MV",
        expected_aps_case="case_2",
        expected_clv_id="RB02_CLV",
        expected_cfv_id="RB02_CFV",
        expected_active_cv_ids=("RB02_CFV",),
        expected_eq10_consumer_ids=("RB02_CFV",),
        method="rolling RB02 / B01-like near-control case 2",
    ),
    "RB03_MV": RollingBasicMVExpectation(
        scenario_id=ROLLING_BASIC_SCENARIO_ID,
        mv_id="RB03_MV",
        expected_aps_case="case_3",
        expected_clv_id="RB03_CLV",
        expected_cfv_id="RB03_CFV",
        expected_active_cv_ids=("RB03_CLV",),
        expected_eq10_consumer_ids=(),
        method="rolling RB03 / B02-like upstream case 3",
    ),
    "RB04_MV": RollingBasicMVExpectation(
        scenario_id=ROLLING_BASIC_SCENARIO_ID,
        mv_id="RB04_MV",
        expected_aps_case="case_4",
        expected_clv_id="RB04_CLV",
        expected_cfv_id="RB04_CFV",
        expected_active_cv_ids=("RB04_CLV", "RB04_CFV"),
        expected_eq10_consumer_ids=("RB04_CFV",),
        method="rolling RB04 / B03-like far-upstream case 4",
    ),
}


def get_rolling_basic_expectation(mv_id: str) -> RollingBasicMVExpectation:
    try:
        return ROLLING_BASIC_EXPECTATIONS[mv_id]
    except KeyError as exc:
        raise ValueError(f"unknown rolling BASIC mv_id: {mv_id}") from exc


def get_rolling_basic_expectations() -> dict[str, RollingBasicMVExpectation]:
    return {mv_id: ROLLING_BASIC_EXPECTATIONS[mv_id] for mv_id in ROLLING_BASIC_MV_IDS}


def _stay_lane_2_cuc_overrides(*vehicle_ids: str) -> dict[str, dict[str, Any]]:
    return {
        vehicle_id: {
            "recommended_choice": "stay_lane_2",
            "U1": 0.0,
            "U2": 10000.0,
        }
        for vehicle_id in vehicle_ids
    }


def _rolling_mv(vehicle_id: str, x: float, note: str):
    return mv(vehicle_id, x, note_key="rolling_basic_note", note=note)


def _rolling_lane2(vehicle_id: str, x: float, note: str):
    return lane2(vehicle_id, x, note_key="rolling_basic_note", note=note)


def _rolling_lane1_blocker(vehicle_id: str, x: float, note: str):
    return lane1(vehicle_id, x, note_key="rolling_basic_note", note=note)


ROLLING_BASIC_SCENE_SPEC = StaticSceneSpec(
    scenario_id=ROLLING_BASIC_SCENARIO_ID,
    scenario_name="continuous rolling BASIC queue with four on-ramp MVs",
    description="ROLLING-BASIC-04 continuous multi-BASIC numeric diagnostic scenario",
    purpose=f"Rolling BASIC diagnostic from {ROLLING_BASIC_DERIVATION_REF}",
    test_level="integration",
    status="probe",
    derivation_ref=(ROLLING_BASIC_DERIVATION_REF,),
    road_config_ref="paper_fig10_first_version",
    parameter_config_ref="paper_table_i_first_version",
    vehicles=(
        _rolling_mv("RB01_MV", 6750.0, "RB01, B04-like case 2, already in control zone"),
        _rolling_lane2("RB01_CLV", 6784.0, "RB01 relative +34 m"),
        _rolling_lane2("RB01_CFV", 6744.0, "RB01 relative -6 m"),
        _rolling_lane1_blocker("RB01_TLV_CFV", 6753.0, "makes RB01_CFV target lane unsafe"),
        _rolling_mv("RB02_MV", 6640.0, "RB02, B01-like case 2, near control entry"),
        _rolling_lane2("RB02_CLV", 6674.0, "RB02 relative +34 m"),
        _rolling_lane2("RB02_CFV", 6634.0, "RB02 relative -6 m"),
        _rolling_lane1_blocker("RB02_TLV_CFV", 6643.0, "makes RB02_CFV target lane unsafe"),
        _rolling_mv("RB03_MV", 6540.0, "RB03, B02-like case 3, upstream pre-control"),
        _rolling_lane2("RB03_CLV", 6554.0, "RB03 relative +14 m"),
        _rolling_lane2("RB03_CFV", 6514.0, "RB03 relative -26 m"),
        _rolling_lane1_blocker("RB03_TLV_CLV", 6563.0, "makes RB03_CLV target lane unsafe"),
        _rolling_mv("RB04_MV", 6440.0, "RB04, B03-like case 4, far upstream pre-control"),
        _rolling_lane2("RB04_CLV", 6454.0, "RB04 relative +14 m"),
        _rolling_lane2("RB04_CFV", 6434.0, "RB04 relative -6 m"),
        _rolling_lane1_blocker("RB04_TLV_CFV", 6443.0, "makes RB04_CFV target lane unsafe"),
        _rolling_lane1_blocker("RB04_TLV_CLV", 6463.0, "makes RB04_CLV target lane unsafe"),
    ),
    module_overrides={
        "test_harness_overrides": {
            "source": "rolling_basic_numeric_diagnostic",
            "cuc_utility_overrides": _stay_lane_2_cuc_overrides(
                *ROLLING_BASIC_MAINLINE_VEHICLE_IDS
            ),
        },
    },
    notes=(
        "This is one continuous rolling on-ramp queue, not four isolated BASIC cases.",
        "Overlapping APS windows are intentional and must be diagnosed rather than avoided.",
        "CUC utility is diagnostically pinned to stay_lane_2 for all mainline vehicles in this rolling run.",
        "No frozen APS assignment stopgaps or preloaded assignments are used.",
    ),
)
