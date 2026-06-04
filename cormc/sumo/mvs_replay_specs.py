from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


@dataclass(frozen=True)
class P171ReplaySpec:
    replay_id: str
    source_scenario_id: str
    replay_max_steps: int
    primary_vehicle_ids: tuple[str, ...]
    track_vehicle_id: str
    role_map: dict[str, str]
    numeric_checks: tuple[str, ...]
    description: str

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["primary_vehicle_ids"] = list(self.primary_vehicle_ids)
        payload["numeric_checks"] = list(self.numeric_checks)
        payload["role_map"] = dict(self.role_map)
        return payload


P17_1_REPLAY_SCENARIOS: tuple[P171ReplaySpec, ...] = (
    P171ReplaySpec(
        replay_id="MVS-E2E-1-extended",
        source_scenario_id="MVS-E2E-1",
        replay_max_steps=140,
        primary_vehicle_ids=("MV_DEMO", "CLV_DEMO", "CFV_DEMO", "BG_LANE1_DEMO"),
        track_vehicle_id="MV_DEMO",
        role_map={
            "MV_DEMO": "mv_on_ramp_active",
            "CLV_DEMO": "clv",
            "CFV_DEMO": "cfv",
            "BG_LANE1_DEMO": "support",
        },
        numeric_checks=(
            "mv_demo_y_from_ramp_to_mainline",
            "eq53_pass",
            "merge_start",
            "lateral_trajectory_completed",
            "no_cooperative_request",
        ),
        description="Extended MVS-E2E-1 numeric replay: APS case 1, no CUC, CMC merge, commit.",
    ),
    P171ReplaySpec(
        replay_id="MVS-CMC-1-extended",
        source_scenario_id="MVS-CMC-1",
        replay_max_steps=90,
        primary_vehicle_ids=("MV_CMC_1", "CLV_CMC_1", "CFV_CMC_1"),
        track_vehicle_id="MV_CMC_1",
        role_map={
            "MV_CMC_1": "mv_on_ramp_active",
            "CLV_CMC_1": "clv",
            "CFV_CMC_1": "cfv",
        },
        numeric_checks=(
            "step0_eq53_pass",
            "step0_merge_start",
            "mv_cmc_1_y_from_ramp_to_mainline",
        ),
        description="Extended MVS-CMC-1 numeric replay: helper CMC gate expanded only for P17.1.",
    ),
    P171ReplaySpec(
        replay_id="MVS-CUC-1A-lanechange",
        source_scenario_id="MVS-CUC-1A_override_choice1",
        replay_max_steps=50,
        primary_vehicle_ids=("CFV_X", "MV_CUC", "CLV_Y", "TLV", "TFV"),
        track_vehicle_id="CFV_X",
        role_map={
            "CFV_X": "cfv_active_cooperative",
            "MV_CUC": "mv_on_ramp_active",
            "CLV_Y": "clv",
            "TLV": "tlv",
            "TFV": "tfv",
        },
        numeric_checks=(
            "cfv_x_y_from_lane2_to_lane1",
            "final_choice_change_to_lane_1",
            "lane_change_command_created",
            "same_step_overlay",
            "lateral_trajectory_completed",
        ),
        description="CUC choice 1 replay: CFV_X changes from lane_2 to lane_1 with same-step overlay.",
    ),
    P171ReplaySpec(
        replay_id="MVS-CUC-2-eq10-window",
        source_scenario_id="MVS-CUC-2",
        replay_max_steps=70,
        primary_vehicle_ids=("CFV_X", "MV_CUC", "CLV_Y", "TLV", "TFV"),
        track_vehicle_id="CFV_X",
        role_map={
            "CFV_X": "cfv_active_cooperative",
            "MV_CUC": "mv_on_ramp_active",
            "CLV_Y": "clv",
            "TLV": "tlv",
            "TFV": "tfv",
        },
        numeric_checks=(
            "cfv_x_stays_lane2",
            "cfv_x_lane_change_state_normal",
            "eq10_spacing_override_consumed",
            "no_cfv_x_lateral_trajectory",
        ),
        description="CUC choice 2 replay: CFV_X stays on lane_2 and consumes Eq.10 spacing override.",
    ),
    P171ReplaySpec(
        replay_id="MVS-SAFE-1B-cap",
        source_scenario_id="MVS-SAFE-1B_executing_cap_lateral_consumption",
        replay_max_steps=320,
        primary_vehicle_ids=("MV_SAFE_EXEC", "CLV_SAFE_EXEC", "CFV_SAFE_EXEC"),
        track_vehicle_id="MV_SAFE_EXEC",
        role_map={
            "MV_SAFE_EXEC": "mv_on_ramp_active",
            "CLV_SAFE_EXEC": "clv",
            "CFV_SAFE_EXEC": "cfv",
        },
        numeric_checks=(
            "mv_safe_exec_remains_executing",
            "speed_cap_binding",
            "lateral_trajectory_consumes_speed_cap",
            "no_merge_complete",
        ),
        description="Safety replay: executing MV is held by boundary cap; this is not a merge-complete scene.",
    ),
    P171ReplaySpec(
        replay_id="MVS-COMMIT-1-full-extended",
        source_scenario_id="MVS-COMMIT-1-full",
        replay_max_steps=140,
        primary_vehicle_ids=("CV_ACTIVE_LC", "MV_ACTIVE_MERGE", "MV_CACHE"),
        track_vehicle_id="MV_ACTIVE_MERGE",
        role_map={
            "CV_ACTIVE_LC": "active_cooperative_cv",
            "MV_ACTIVE_MERGE": "mv_on_ramp_active",
            "MV_CACHE": "mv_on_ramp_active",
            "CLV_CACHE": "clv",
            "CFV_CACHE": "cfv",
            "CLV_MERGE": "clv",
            "CFV_MERGE": "cfv",
            "TLV_ACTIVE": "tlv",
            "TFV_ACTIVE": "tfv",
        },
        numeric_checks=(
            "cv_active_lc_y_to_lane1",
            "mv_active_merge_y_to_mainline",
            "mv_cache_y_to_mainline",
            "commit_sanity_all_pass",
            "unique_commit_per_vehicle_step",
        ),
        description="Commit full replay: active trajectory continuation, non-APS cache, and unique commit.",
    ),
)


P17_1_REPLAY_BY_ID: dict[str, P171ReplaySpec] = {
    spec.replay_id: spec for spec in P17_1_REPLAY_SCENARIOS
}


def get_p17_1_replay_spec(replay_id: str) -> P171ReplaySpec:
    try:
        return P17_1_REPLAY_BY_ID[replay_id]
    except KeyError as exc:
        known = ", ".join(P17_1_REPLAY_BY_ID)
        raise ValueError(f"unknown P17.1 replay_id {replay_id!r}; expected one of: {known}") from exc


def iter_p17_1_replay_specs(selector: str = "all") -> tuple[P171ReplaySpec, ...]:
    if selector == "all":
        return P17_1_REPLAY_SCENARIOS
    return (get_p17_1_replay_spec(selector),)
