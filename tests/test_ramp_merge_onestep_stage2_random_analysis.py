from pathlib import Path

from cormc.observation.artifacts import build_observation_plot_artifacts
from cormc.observation.stage2_artifacts import load_stage2_observation_dataset
from cormc.observation.sumo_replay import build_observation_sumo_replay_artifacts
from cormc.onestep.rolling import export_onestep_stage2_analysis
from cormc.onestep.rolling.stage2_random_runner import (
    run_onestep_stage2_random_history,
)
from cormc.scenes import build_onestep_random_lane2_ramp_scene


def test_random_stage2_analysis_exports_observation_and_replay_artifacts(tmp_path: Path) -> None:
    history_run = run_onestep_stage2_random_history(
        build_onestep_random_lane2_ramp_scene(
            density="high",
            horizon_s=30.0,
            max_steps=200,
            safe_spawn_gap_m=5.0,
        ),
        max_steps=200,
        run_id="random-analysis-test",
    )
    result = export_onestep_stage2_analysis(
        history_run.summary,
        tmp_path / history_run.scenario_id / history_run.run_id,
        history=history_run.history,
    )
    output_dir = Path(result.summary_json_path).parent

    for path in (
        result.summary_json_path,
        result.report_path,
        result.trajectory_csv_path,
        result.gap_rows_json_path,
        result.process_x_t_local_plot_path,
        result.process_v_t_plot_path,
        result.process_y_t_plot_path,
        result.lifecycle_timeline_plot_path,
    ):
        assert Path(path).exists()
        assert Path(path).stat().st_size > 0
    assert set(result.summary) == {
        "scenario_summary",
        "round_summaries",
        "mv_summaries",
        "cross_mv_summary",
        "artifact_paths",
    }
    assert "validation passed" in Path(result.report_path).read_text(encoding="utf-8")

    dataset = load_stage2_observation_dataset(output_dir)
    plot_result = build_observation_plot_artifacts(dataset, output_dir / "stage8_plots")
    replay_result = build_observation_sumo_replay_artifacts(
        dataset,
        output_dir / "stage8_sumo_replay",
    )

    assert Path(plot_result.trajectory_csv_path).exists()
    assert Path(replay_result.replay_trajectory_path).exists()
    assert replay_result.replay_fidelity["status"] == "passed"
