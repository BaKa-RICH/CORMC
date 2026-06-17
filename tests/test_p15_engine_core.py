from __future__ import annotations

import ast
from pathlib import Path

from cormc.simulation_core.engine import (
    CormcEngine,
    EngineStepResult,
    StepLoopTrace,
    StepWorkspace,
    build_initial_state_from_scenario_config,
)
from cormc.scenes import BASIC_SCENARIO_IDS, load_scene_config
from cormc.simulation_core.loop import (
    SimulationLoopConfig,
    run_deterministic_simulation,
    run_one_deterministic_step,
)
from cormc.simulation_core.commit import CommandBuffer, NextStateBuffer


def test_p15_engine_advance_one_step_result_contract() -> None:
    scenario = load_scene_config(BASIC_SCENARIO_IDS[0])
    initial_state = build_initial_state_from_scenario_config(scenario)
    result = CormcEngine(scenario, run_id="p15-core").advance_one_step(initial_state)

    assert isinstance(result, EngineStepResult)
    assert isinstance(result.trace, StepLoopTrace)
    assert isinstance(result.workspace, StepWorkspace)
    assert isinstance(result.command_buffer, CommandBuffer)
    assert isinstance(result.next_state_buffer, NextStateBuffer)
    assert result.advanced_state == result.time_advance_result.advanced_state
    assert result.command_buffer == result.trace.canonical_command_buffer
    assert result.next_state_buffer == result.trace.canonical_next_state_buffer
    assert result.commit_result == result.trace.commit_result
    assert result.time_advance_result == result.trace.time_advance_result
    assert result.elapsed_seconds >= 0.0
    assert result.workspace.step == initial_state.step
    assert result.workspace.time_advance_result is result.time_advance_result


def test_p15_wrapper_one_step_delegates_to_engine_equivalently() -> None:
    scenario = load_scene_config(BASIC_SCENARIO_IDS[0])
    initial_state = build_initial_state_from_scenario_config(scenario)

    engine_result = CormcEngine(scenario, run_id="p15-equivalence").advance_one_step(initial_state)
    wrapper_trace = run_one_deterministic_step(
        initial_state,
        scenario,
        run_id="p15-equivalence",
    )

    assert wrapper_trace == engine_result.trace
    assert wrapper_trace.time_advance_result.advanced_state == engine_result.advanced_state


def test_p15_wrapper_multi_step_uses_engine_semantics() -> None:
    result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id=BASIC_SCENARIO_IDS[0],
            run_id="p15-multistep",
            max_steps=3,
            render_png=False,
        )
    )

    assert len(result.step_traces) == 3
    assert result.final_state == result.step_traces[-1].time_advance_result.advanced_state
    assert result.final_state.step == 3
    assert result.history.trajectory_records
    assert result.history.event_records
    assert result.png_path is None


def test_p15_simulation_loop_no_dual_step_orchestration() -> None:
    source = Path("cormc/simulation_core/loop.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    forbidden = {
        "run_step4a_aps",
        "run_step4b_cmc",
        "run_step5_cooperative_request_conflict_resolution",
        "run_step6_cuc_choice_compliance_lane_change_overlay",
        "run_step7_longitudinal_model_spacing_speedcap",
        "run_step8_lateral_trajectory_planning_speed_progress",
        "commit_step",
        "advance_time_after_commit_and_integration",
    }
    called_names = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }

    assert forbidden.isdisjoint(called_names)
    assert "advance_one_step" in source
