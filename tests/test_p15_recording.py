from __future__ import annotations

import ast
from pathlib import Path

from cormc.simulation_core.engine import CormcEngine, build_initial_state_from_scenario_config
from cormc.simulation_core.recording import FullRecorder, MinimalRecorder, NullRecorder
from cormc.scenes import BASIC_SCENARIO_IDS, load_scene_config
from cormc.simulation_core.loop import SimulationLoopConfig, run_deterministic_simulation


def test_p15_full_minimal_null_recorders_do_not_change_engine_result() -> None:
    scenario = load_scene_config(BASIC_SCENARIO_IDS[0])
    initial_state = build_initial_state_from_scenario_config(scenario)
    step_result = CormcEngine(scenario, run_id="p15-recording").advance_one_step(initial_state)

    full = FullRecorder()
    minimal = MinimalRecorder()
    null = NullRecorder()
    for recorder in (full, minimal, null):
        recorder.record_step(step_result, run_id="p15-recording", scenario_id=BASIC_SCENARIO_IDS[0])

    assert full.history.event_records
    assert full.history.trajectory_records
    assert full.unique_expected_png_features()
    assert not minimal.history.event_records
    assert minimal.summaries[0]["advanced_step"] == step_result.advanced_state.step
    assert minimal.summaries[0]["elapsed_seconds"] >= 0.0
    assert not null.history.event_records
    assert step_result.advanced_state == step_result.time_advance_result.advanced_state


def test_p15_engine_hot_path_has_no_file_output_calls() -> None:
    source = Path("cormc/simulation_core/engine.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    forbidden_import_modules = {"cormc.legacy.artifact_reports", "cormc.legacy.paper_artifacts"}
    imported_modules = {
        node.module
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module is not None
    }
    forbidden_call_names = {
        "open",
        "render_time_space_png",
        "write_artifact_manifest",
        "write_regression_report",
        "export_trajectory_history",
        "export_event_history",
        "export_sanity_history",
    }
    called_names = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    called_attrs = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }

    assert forbidden_import_modules.isdisjoint(imported_modules)
    assert forbidden_call_names.isdisjoint(called_names)
    assert {"write_text", "write_bytes", "mkdir"}.isdisjoint(called_attrs)


def test_p15_render_png_false_does_not_create_png(tmp_path: Path) -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id=BASIC_SCENARIO_IDS[0],
            run_id="p15-no-png",
            max_steps=1,
            output_dir=tmp_path,
            render_png=False,
        )
    )

    assert result.png_path is None
    assert not list(tmp_path.rglob("*.png"))


def test_p15_render_png_true_still_runs_at_wrapper_boundary(tmp_path: Path) -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id=BASIC_SCENARIO_IDS[0],
            run_id="p15-with-png",
            max_steps=1,
            output_dir=tmp_path,
            render_png=True,
        )
    )

    assert result.png_path is not None
    png_path = Path(result.png_path)
    assert png_path.exists()
    assert png_path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
