from __future__ import annotations

from cormc.mvs.loader import load_builtin_scenario
from cormc.mvs.runner import DETERMINISTIC_SCENARIO_ROUTES
from cormc.sumo.mvs_replay_specs import P17_1_REPLAY_SCENARIOS, P17_1_REPLAY_BY_ID


EXPECTED_REPLAY_IDS = {
    "MVS-E2E-1-extended",
    "MVS-CMC-1-extended",
    "MVS-CUC-1A-lanechange",
    "MVS-CUC-2-eq10-window",
    "MVS-SAFE-1B-cap",
    "MVS-COMMIT-1-full-extended",
}


def test_p17_1_replay_registry_has_exact_first_batch() -> None:
    replay_ids = [spec.replay_id for spec in P17_1_REPLAY_SCENARIOS]

    assert set(replay_ids) == EXPECTED_REPLAY_IDS
    assert len(replay_ids) == 6
    assert len(replay_ids) == len(set(replay_ids))
    assert set(P17_1_REPLAY_BY_ID) == EXPECTED_REPLAY_IDS


def test_p17_1_sources_load_but_required_suite_defaults_are_unchanged() -> None:
    defaults = {key: dict(value) for key, value in DETERMINISTIC_SCENARIO_ROUTES.items()}

    for spec in P17_1_REPLAY_SCENARIOS:
        scenario = load_builtin_scenario(spec.source_scenario_id)
        assert scenario["scenario_id"] == spec.source_scenario_id
        assert spec.replay_max_steps >= 50

    assert defaults == {
        "MVS-E2E-1": {"max_steps": 70},
        "MVS-CUC-1A_override_choice1": {"max_steps": 1},
        "MVS-CUC-2": {"max_steps": 1},
        "MVS-CUC-3": {"max_steps": 1},
        "MVS-SAFE-1A_waiting_cap": {"max_steps": 1},
        "MVS-SAFE-1B_executing_cap_lateral_consumption": {"max_steps": 1},
        "MVS-SAFE-2": {"max_steps": 1},
        "MVS-COMMIT-1-full": {"max_steps": 1},
    }


def test_p17_1_cmc_helper_is_not_promoted_to_deterministic_required_route() -> None:
    assert "MVS-CMC-1" not in DETERMINISTIC_SCENARIO_ROUTES
