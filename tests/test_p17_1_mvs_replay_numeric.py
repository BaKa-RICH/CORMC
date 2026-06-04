from __future__ import annotations

from pathlib import Path

from cormc.sumo.mvs_replay_artifacts import build_p17_1_scenario_artifact
from cormc.sumo.mvs_replay_specs import P17_1_REPLAY_SCENARIOS


def test_p17_1_all_numeric_gates_pass_with_diagnostic_names(tmp_path: Path) -> None:
    for spec in P17_1_REPLAY_SCENARIOS:
        result = build_p17_1_scenario_artifact(spec, tmp_path / spec.replay_id)

        assert result.numeric_gate_status == "passed", result.replay_id
        assert result.replay_fidelity_status == "passed", result.replay_id
        summary_text = Path(result.numeric_summary_path).read_text(encoding="utf-8")
        assert '"numeric_gate_failures": []' in summary_text
        assert spec.replay_id in summary_text


def test_p17_1_numeric_failures_are_scoped_by_replay_vehicle_and_check(tmp_path: Path) -> None:
    spec = P17_1_REPLAY_SCENARIOS[0]
    result = build_p17_1_scenario_artifact(spec, tmp_path / spec.replay_id)
    summary = Path(result.numeric_summary_path).read_text(encoding="utf-8")

    assert "replay_id" in summary
    assert "vehicle_ranges" in summary
    assert "event_hits" in summary
