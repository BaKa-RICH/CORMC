from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping

from cormc.scenes.model import StaticSceneSpec, lane2, mv


RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID = "RM-ONESTEP-S05-PLAN-STEP0"
RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID = "RM-ONESTEP-S05-ROLLING-ENTRY"
RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID = "RM-ONESTEP-S07-PLAN-STEP0"
RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID = "RM-ONESTEP-S07-ROLLING-ENTRY"
RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID = "RM-ONESTEP-S07-2MV-ROLLING-ENTRY"
RM_ONESTEP_S07_2MV_SCENARIO_IDS: tuple[str, ...] = (
    RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
)

RM_ONESTEP_S05_SCENARIO_IDS: tuple[str, ...] = (
    RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
)
RM_ONESTEP_S07_SCENARIO_IDS: tuple[str, ...] = (
    RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
)
RM_ONESTEP_SCENARIO_IDS: tuple[str, ...] = (
    *RM_ONESTEP_S05_SCENARIO_IDS,
    *RM_ONESTEP_S07_SCENARIO_IDS,
)

RM_ONESTEP_S05_MV_ID = "S05_MV"
RM_ONESTEP_S05_LANE_2_VEHICLE_IDS: tuple[str, ...] = tuple(
    f"S05_L2_{index:02d}" for index in range(1, 8)
)
RM_ONESTEP_S05_MAINLINE_X_LOCAL: tuple[float, ...] = (
    -100.0,
    -50.0,
    0.0,
    50.0,
    100.0,
    160.0,
    240.0,
)
RM_ONESTEP_S05_MAINLINE_X_GLOBAL: tuple[float, ...] = tuple(
    6650.0 + value for value in RM_ONESTEP_S05_MAINLINE_X_LOCAL
)
RM_ONESTEP_S05_GAP_INTERVALS_LOCAL: tuple[tuple[float, float], ...] = tuple(
    zip(RM_ONESTEP_S05_MAINLINE_X_LOCAL, RM_ONESTEP_S05_MAINLINE_X_LOCAL[1:])
)
RM_ONESTEP_S05_GAP_CENTERS_LOCAL: tuple[float, ...] = tuple(
    (x_rear + x_front) / 2.0
    for x_rear, x_front in RM_ONESTEP_S05_GAP_INTERVALS_LOCAL
)

RM_ONESTEP_S07_MV_ID = "S07_MV"
RM_ONESTEP_S07_2MV_REAR_MV_ID = "S07_MV_REAR"
RM_ONESTEP_S07_2MV_MV_IDS: tuple[str, ...] = (
    RM_ONESTEP_S07_MV_ID,
    RM_ONESTEP_S07_2MV_REAR_MV_ID,
)
RM_ONESTEP_S07_LANE_2_VEHICLE_IDS: tuple[str, ...] = tuple(
    f"S07_L2_{index:02d}" for index in range(1, 8)
)
RM_ONESTEP_S07_MAINLINE_X_LOCAL: tuple[float, ...] = (
    -180.0,
    -90.0,
    -25.0,
    30.0,
    110.0,
    190.0,
    250.0,
)
RM_ONESTEP_S07_MAINLINE_X_GLOBAL: tuple[float, ...] = tuple(
    6650.0 + value for value in RM_ONESTEP_S07_MAINLINE_X_LOCAL
)
RM_ONESTEP_S07_GAP_INTERVALS_LOCAL: tuple[tuple[float, float], ...] = tuple(
    zip(RM_ONESTEP_S07_MAINLINE_X_LOCAL, RM_ONESTEP_S07_MAINLINE_X_LOCAL[1:])
)
RM_ONESTEP_S07_GAP_CENTERS_LOCAL: tuple[float, ...] = tuple(
    (x_rear + x_front) / 2.0
    for x_rear, x_front in RM_ONESTEP_S07_GAP_INTERVALS_LOCAL
)


@dataclass(frozen=True)
class OneStepStage1ScenarioExpectation:
    scenario_id: str
    mode: str
    expected_first_check_step: int
    expected_first_check_t: float
    expected_initial_zone_state: str
    expected_first_trigger_reason: str
    expected_mv_x_global_at_check: float
    expected_mv_x_local_at_check: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "mode": self.mode,
            "expected_first_check_step": self.expected_first_check_step,
            "expected_first_check_t": self.expected_first_check_t,
            "expected_initial_zone_state": self.expected_initial_zone_state,
            "expected_first_trigger_reason": self.expected_first_trigger_reason,
            "expected_mv_x_global_at_check": self.expected_mv_x_global_at_check,
            "expected_mv_x_local_at_check": self.expected_mv_x_local_at_check,
        }


@dataclass(frozen=True)
class OneStepRampMergeCaseSpec:
    one_step_case_id: str
    mv_id: str
    lane_2_vehicle_ids: tuple[str, ...]
    mainline_x_local: tuple[float, ...]
    plan_step0_mv_x_global: float
    rolling_entry_mv_x_global: float
    plan_step0_mainline_origin_x_global: float
    rolling_entry_mainline_origin_x_global: float
    stage1_default_max_steps: Mapping[str, int]
    stage2_default_max_steps: Mapping[str, int]
    stage1_expectations: Mapping[str, OneStepStage1ScenarioExpectation]

    @property
    def gap_intervals_local(self) -> tuple[tuple[float, float], ...]:
        return tuple(zip(self.mainline_x_local, self.mainline_x_local[1:]))

    @property
    def gap_centers_local(self) -> tuple[float, ...]:
        return tuple((rear + front) / 2.0 for rear, front in self.gap_intervals_local)

    @property
    def plan_step0_scenario_id(self) -> str:
        return f"RM-ONESTEP-{self.one_step_case_id}-PLAN-STEP0"

    @property
    def rolling_entry_scenario_id(self) -> str:
        return f"RM-ONESTEP-{self.one_step_case_id}-ROLLING-ENTRY"

    @property
    def scenario_ids(self) -> tuple[str, str]:
        return (self.plan_step0_scenario_id, self.rolling_entry_scenario_id)

    def to_dict(self) -> dict[str, Any]:
        return {
            "one_step_case_id": self.one_step_case_id,
            "mv_id": self.mv_id,
            "lane_2_vehicle_ids": list(self.lane_2_vehicle_ids),
            "mainline_x_local": list(self.mainline_x_local),
            "gap_intervals_local": [list(interval) for interval in self.gap_intervals_local],
            "gap_centers_local": list(self.gap_centers_local),
            "plan_step0_mv_x_global": self.plan_step0_mv_x_global,
            "rolling_entry_mv_x_global": self.rolling_entry_mv_x_global,
            "plan_step0_mainline_origin_x_global": self.plan_step0_mainline_origin_x_global,
            "rolling_entry_mainline_origin_x_global": self.rolling_entry_mainline_origin_x_global,
            "stage1_default_max_steps": dict(self.stage1_default_max_steps),
            "stage2_default_max_steps": dict(self.stage2_default_max_steps),
            "stage1_expectations": {
                scenario_id: expectation.to_dict()
                for scenario_id, expectation in self.stage1_expectations.items()
            },
        }


def get_ramp_merge_onestep_stage1_expectation(
    scenario_id: str,
) -> OneStepStage1ScenarioExpectation:
    spec = get_ramp_merge_onestep_case_spec(scenario_id)
    try:
        return spec.stage1_expectations[scenario_id]
    except KeyError as exc:
        raise ValueError(f"unknown RM-ONESTEP scenario_id: {scenario_id}") from exc


def get_ramp_merge_onestep_case_spec(scenario_id: str) -> OneStepRampMergeCaseSpec:
    try:
        case_id = RM_ONESTEP_SCENARIO_TO_CASE_ID[scenario_id]
    except KeyError as exc:
        raise ValueError(f"unknown RM-ONESTEP scenario_id: {scenario_id}") from exc
    return RM_ONESTEP_CASE_SPECS[case_id]


def _build_stage1_expectations(case_id: str) -> Mapping[str, OneStepStage1ScenarioExpectation]:
    plan_id = f"RM-ONESTEP-{case_id}-PLAN-STEP0"
    rolling_id = f"RM-ONESTEP-{case_id}-ROLLING-ENTRY"
    return MappingProxyType(
        {
            plan_id: OneStepStage1ScenarioExpectation(
                scenario_id=plan_id,
                mode="plan_step0",
                expected_first_check_step=0,
                expected_first_check_t=0.0,
                expected_initial_zone_state="control_zone",
                expected_first_trigger_reason="periodic",
                expected_mv_x_global_at_check=6650.0,
                expected_mv_x_local_at_check=0.0,
            ),
            rolling_id: OneStepStage1ScenarioExpectation(
                scenario_id=rolling_id,
                mode="rolling_entry",
                expected_first_check_step=25,
                expected_first_check_t=2.5,
                expected_initial_zone_state="outside_control_zone",
                expected_first_trigger_reason="MV_enter_control_zone",
                expected_mv_x_global_at_check=6650.0,
                expected_mv_x_local_at_check=0.0,
            ),
        }
    )


def _build_case_spec(
    *,
    one_step_case_id: str,
    mv_id: str,
    lane_2_vehicle_ids: tuple[str, ...],
    mainline_x_local: tuple[float, ...],
) -> OneStepRampMergeCaseSpec:
    stage1_expectations = _build_stage1_expectations(one_step_case_id)
    plan_id = f"RM-ONESTEP-{one_step_case_id}-PLAN-STEP0"
    rolling_id = f"RM-ONESTEP-{one_step_case_id}-ROLLING-ENTRY"
    return OneStepRampMergeCaseSpec(
        one_step_case_id=one_step_case_id,
        mv_id=mv_id,
        lane_2_vehicle_ids=lane_2_vehicle_ids,
        mainline_x_local=mainline_x_local,
        plan_step0_mv_x_global=6650.0,
        rolling_entry_mv_x_global=6600.0,
        plan_step0_mainline_origin_x_global=6650.0,
        rolling_entry_mainline_origin_x_global=6600.0,
        stage1_default_max_steps=MappingProxyType(
            {
                plan_id: 5,
                rolling_id: 35,
            }
        ),
        stage2_default_max_steps=MappingProxyType(
            {
                plan_id: 420,
                rolling_id: 420,
            }
        ),
        stage1_expectations=stage1_expectations,
    )


def _scene(
    scenario_id: str,
    spec: OneStepRampMergeCaseSpec,
    *,
    description: str,
    mv_x_global: float,
    mainline_origin_x_global: float,
    extra_mvs: tuple[tuple[str, float], ...] = (),
) -> StaticSceneSpec:
    lane_2_positions = tuple(
        mainline_origin_x_global + value for value in spec.mainline_x_local
    )
    vehicles = (
        mv(spec.mv_id, mv_x_global),
        *(mv(vehicle_id, x_global) for vehicle_id, x_global in extra_mvs),
        *(
            lane2(vehicle_id, x_global)
            for vehicle_id, x_global in zip(spec.lane_2_vehicle_ids, lane_2_positions)
        ),
    )
    return StaticSceneSpec(
        scenario_id=scenario_id,
        scenario_name=description,
        description=description,
        purpose=f"OneStep {spec.one_step_case_id} stage integration and rolling validation",
        test_level="integration",
        status="probe",
        derivation_ref=(
            f"one_step_experiments:{spec.one_step_case_id}",
            "ramp_merge_onestep_stage1",
            "ramp_merge_onestep_stage2",
        ),
        road_config_ref="paper_fig10_first_version",
        parameter_config_ref="paper_table_i_first_version",
        vehicles=vehicles,
        notes=(
            f"This {spec.one_step_case_id} bridge scene is independent from BASIC.",
            "The stage2 chain reuses the rolling OneStep replanning kernel.",
        ),
    )


RM_ONESTEP_CASE_SPECS: Mapping[str, OneStepRampMergeCaseSpec] = MappingProxyType(
    {
        "S05": _build_case_spec(
            one_step_case_id="S05",
            mv_id=RM_ONESTEP_S05_MV_ID,
            lane_2_vehicle_ids=RM_ONESTEP_S05_LANE_2_VEHICLE_IDS,
            mainline_x_local=RM_ONESTEP_S05_MAINLINE_X_LOCAL,
        ),
        "S07": _build_case_spec(
            one_step_case_id="S07",
            mv_id=RM_ONESTEP_S07_MV_ID,
            lane_2_vehicle_ids=RM_ONESTEP_S07_LANE_2_VEHICLE_IDS,
            mainline_x_local=RM_ONESTEP_S07_MAINLINE_X_LOCAL,
        ),
    }
)

RM_ONESTEP_SCENARIO_TO_CASE_ID: Mapping[str, str] = MappingProxyType(
    {
        **{
            scenario_id: case_id
            for case_id, spec in RM_ONESTEP_CASE_SPECS.items()
            for scenario_id in spec.scenario_ids
        },
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID: "S07",
    }
)

RM_ONESTEP_SCENE_SPECS: Mapping[str, StaticSceneSpec] = MappingProxyType(
    {
        RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID: _scene(
            RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_CASE_SPECS["S05"],
            description="OneStep S05 stage plan-step0",
            mv_x_global=RM_ONESTEP_CASE_SPECS["S05"].plan_step0_mv_x_global,
            mainline_origin_x_global=RM_ONESTEP_CASE_SPECS["S05"].plan_step0_mainline_origin_x_global,
        ),
        RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID: _scene(
            RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
            RM_ONESTEP_CASE_SPECS["S05"],
            description="OneStep S05 stage rolling-entry",
            mv_x_global=RM_ONESTEP_CASE_SPECS["S05"].rolling_entry_mv_x_global,
            mainline_origin_x_global=RM_ONESTEP_CASE_SPECS["S05"].rolling_entry_mainline_origin_x_global,
        ),
        RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID: _scene(
            RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_CASE_SPECS["S07"],
            description="OneStep S07 stage plan-step0",
            mv_x_global=RM_ONESTEP_CASE_SPECS["S07"].plan_step0_mv_x_global,
            mainline_origin_x_global=RM_ONESTEP_CASE_SPECS["S07"].plan_step0_mainline_origin_x_global,
        ),
        RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID: _scene(
            RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
            RM_ONESTEP_CASE_SPECS["S07"],
            description="OneStep S07 stage rolling-entry",
            mv_x_global=RM_ONESTEP_CASE_SPECS["S07"].rolling_entry_mv_x_global,
            mainline_origin_x_global=RM_ONESTEP_CASE_SPECS["S07"].rolling_entry_mainline_origin_x_global,
        ),
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID: _scene(
            RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
            RM_ONESTEP_CASE_SPECS["S07"],
            description="OneStep S07 2-MV stage rolling-entry",
            mv_x_global=RM_ONESTEP_CASE_SPECS["S07"].rolling_entry_mv_x_global,
            mainline_origin_x_global=RM_ONESTEP_CASE_SPECS["S07"].rolling_entry_mainline_origin_x_global,
            extra_mvs=((RM_ONESTEP_S07_2MV_REAR_MV_ID, 6540.0),),
        ),
    }
)

RM_ONESTEP_STAGE1_DEFAULT_MAX_STEPS: Mapping[str, int] = MappingProxyType(
    {
        scenario_id: int(step_count)
        for spec in RM_ONESTEP_CASE_SPECS.values()
        for scenario_id, step_count in spec.stage1_default_max_steps.items()
    }
)
RM_ONESTEP_STAGE2_DEFAULT_MAX_STEPS: Mapping[str, int] = MappingProxyType(
    {
        **{
            scenario_id: int(step_count)
            for spec in RM_ONESTEP_CASE_SPECS.values()
            for scenario_id, step_count in spec.stage2_default_max_steps.items()
        },
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID: 420,
    }
)

RM_ONESTEP_S07_DEFAULT_MAX_STEPS = RM_ONESTEP_CASE_SPECS["S07"].stage1_default_max_steps
RM_ONESTEP_S07_SCENARIO_EXPECTATIONS = RM_ONESTEP_CASE_SPECS["S07"].stage1_expectations
