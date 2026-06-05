from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping

from cormc.engine import (
    CormcEngine,
    EngineStepResult,
    Step0To3LoopResult,
    StepLoopTrace,
    StepWorkspace,
    assignment_record_actions_to_candidate_updates,
    build_initial_state_from_scenario_config,
    build_step_command_buffer,
    build_step_next_state_buffer,
    command_cache_updates_to_candidate_updates,
    normalize_maneuver_commands,
)
from cormc.mvs.loader import load_builtin_scenario, load_scenario_config
from cormc.random_generation import (
    DEFAULT_P16_MAX_STEPS,
    DEFAULT_P16_SEED,
    P16_DEMO_SCENARIO_ID,
    SeededRandomProfile,
    build_p16_demo_scenario_config,
    generate_boundary_queue,
    profile_from_mapping,
)
from cormc.recording import FullRecorder
from cormc.step0_3 import SimulationState
from cormc.step9_11 import OutputHistory


@dataclass(frozen=True)
class SimulationLoopConfig:
    scenario: str | Mapping[str, Any] | None = None
    scenario_id: str | None = None
    run_id: str = "p12-run"
    max_steps: int = 1
    stop_conditions: tuple[str | Callable[[SimulationState], bool], ...] = ()
    output_dir: str | Path = "artifacts"
    render_png: bool = True
    deterministic_profile_enabled: bool = True
    random_enabled: bool = False
    seed: int | None = None
    seeded_random_profile: SeededRandomProfile | Mapping[str, Any] | None = None


@dataclass(frozen=True)
class SimulationLoopResult:
    initial_state: SimulationState
    final_state: SimulationState
    history: OutputHistory
    step_traces: tuple[StepLoopTrace, ...]
    expected_png_features: tuple[dict[str, Any], ...]
    png_path: str | None
    status: str
    scenario_id: str
    run_id: str


def run_deterministic_simulation(config: SimulationLoopConfig) -> SimulationLoopResult:
    scenario_config = _load_loop_scenario_config(config)
    scenario_id = str(scenario_config["scenario_id"])
    initial_state = build_initial_state_from_scenario_config(scenario_config)
    state = initial_state
    engine = CormcEngine(scenario_config=scenario_config, run_id=config.run_id)
    traces: list[StepLoopTrace] = []
    recorder = FullRecorder()
    status = "max_steps_reached"

    for _ in range(max(0, int(config.max_steps))):
        step_result = engine.advance_one_step(state)
        trace = step_result.trace
        traces.append(trace)
        recorder.record_step(step_result, run_id=config.run_id, scenario_id=scenario_id)
        state = step_result.advanced_state
        if _stop_conditions_met(state, config.stop_conditions):
            status = "stopped_by_condition"
            break

    history = recorder.history
    expected_png_features = recorder.unique_expected_png_features()
    png_path: str | None = None
    if config.render_png:
        from cormc.p11_output import render_time_space_png

        png_target = Path(config.output_dir) / scenario_id / config.run_id / "time_space.png"
        render = render_time_space_png(
            history.trajectory_records,
            expected_png_features,
            png_target,
            events=history.event_records,
        )
        png_path = render.png_path
        history.png_artifacts.append(render.to_dict())

    return SimulationLoopResult(
        initial_state=initial_state,
        final_state=state,
        history=history,
        step_traces=tuple(traces),
        expected_png_features=expected_png_features,
        png_path=png_path,
        status=status,
        scenario_id=scenario_id,
        run_id=config.run_id,
    )


def run_seeded_random_simulation(config: SimulationLoopConfig | None = None) -> SimulationLoopResult:
    config = config or SimulationLoopConfig(
        scenario_id=P16_DEMO_SCENARIO_ID,
        run_id="p16-seeded-demo",
        max_steps=DEFAULT_P16_MAX_STEPS,
        random_enabled=True,
    )
    if not config.random_enabled:
        return run_deterministic_simulation(config)

    profile = profile_from_mapping(
        config.seeded_random_profile,
        seed=config.seed,
        enabled=True,
    )
    scenario_config = _load_loop_scenario_config(config)
    scenario_id = str(scenario_config["scenario_id"])
    max_steps = max(0, int(config.max_steps))
    initial_state = build_initial_state_from_scenario_config(scenario_config)
    state = initial_state
    max_t = initial_state.t + max_steps * initial_state.dt
    random_queue = generate_boundary_queue(
        profile,
        max_t=max_t,
        start_step=initial_state.step,
        start_t=initial_state.t,
    )
    engine = CormcEngine(
        scenario_config=scenario_config,
        run_id=config.run_id,
        random_queue=random_queue,
        safe_spawn_gap_m=profile.safe_spawn_gap_m,
    )
    traces: list[StepLoopTrace] = []
    recorder = FullRecorder()
    status = "max_steps_reached"

    for _ in range(max_steps):
        step_result = engine.advance_one_step(state)
        trace = step_result.trace
        traces.append(trace)
        recorder.record_step(step_result, run_id=config.run_id, scenario_id=scenario_id)
        state = step_result.advanced_state
        if _stop_conditions_met(state, config.stop_conditions):
            status = "stopped_by_condition"
            break

    history = recorder.history
    expected_png_features = recorder.unique_expected_png_features()
    png_path: str | None = None
    if config.render_png:
        from cormc.p11_output import render_time_space_png

        png_target = Path(config.output_dir) / scenario_id / config.run_id / "time_space.png"
        render = render_time_space_png(
            history.trajectory_records,
            expected_png_features,
            png_target,
            events=history.event_records,
        )
        png_path = render.png_path
        history.png_artifacts.append(render.to_dict())

    return SimulationLoopResult(
        initial_state=initial_state,
        final_state=state,
        history=history,
        step_traces=tuple(traces),
        expected_png_features=expected_png_features,
        png_path=png_path,
        status=status,
        scenario_id=scenario_id,
        run_id=config.run_id,
    )


def run_one_deterministic_step(
    state: SimulationState,
    scenario_config: Mapping[str, Any],
    run_id: str,
) -> StepLoopTrace:
    return CormcEngine(scenario_config=scenario_config, run_id=run_id).advance_one_step(state).trace


def _stop_conditions_met(
    state: SimulationState,
    stop_conditions: tuple[str | Callable[[SimulationState], bool], ...],
) -> bool:
    for condition in stop_conditions:
        if callable(condition) and condition(state):
            return True
        if isinstance(condition, str) and _string_stop_condition_met(state, condition):
            return True
    return False


def _string_stop_condition_met(state: SimulationState, condition: str) -> bool:
    parts = condition.split(":")
    if len(parts) == 4 and parts[0] == "vehicle_state":
        _, vehicle_id, state_name, expected_value = parts
        vehicle_state = state.vehicle_states.get(vehicle_id)
        return vehicle_state is not None and str(getattr(vehicle_state, state_name)) == expected_value
    if len(parts) == 2 and parts[0] == "merge_completed":
        vehicle_state = state.vehicle_states.get(parts[1])
        return vehicle_state is not None and vehicle_state.merge_state == "merged"
    if len(parts) == 2 and parts[0] == "lane_change_completed":
        vehicle_state = state.vehicle_states.get(parts[1])
        return vehicle_state is not None and vehicle_state.lane_change_state == "normal"
    return False


def _load_loop_scenario_config(config: SimulationLoopConfig) -> dict[str, Any]:
    scenario = config.scenario
    if scenario is None:
        scenario = config.scenario_id
    if scenario is None:
        raise ValueError("SimulationLoopConfig requires scenario or scenario_id")
    if isinstance(scenario, str) and scenario == P16_DEMO_SCENARIO_ID:
        return load_scenario_config(build_p16_demo_scenario_config(seed=config.seed or DEFAULT_P16_SEED))
    if isinstance(scenario, str):
        return load_builtin_scenario(scenario)
    return load_scenario_config(dict(scenario))
