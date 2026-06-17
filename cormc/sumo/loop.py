from __future__ import annotations

from dataclasses import asdict, dataclass, field, is_dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping

from cormc.simulation_core.engine import CormcEngine
from cormc.traffic_flow.generation import (
    ArrivalStream,
    SeededRandomProfile,
    compute_spawn_decisions,
    generate_boundary_queue,
)
from cormc.sumo.adapter import SumoRealizedStateAdapter
from cormc.sumo.authority import ActiveControlRegistry, resolve_active_authority
from cormc.sumo.commands import ControlledTrajectoryCommand, RealizationRecord, SumoTrajectoryAuthorityConfig
from cormc.sumo.env import get_sumo_version, import_traci
from cormc.sumo.executor import MoveToXYTrajectoryExecutor, read_realized_vehicle
from cormc.sumo.mapping import to_sumo_position
from cormc.sumo.monitoring import (
    COLLIDED,
    MISMATCH,
    TELEPORTED,
    RealizationMonitor,
    collect_collision_events,
    collect_teleport_events,
)
from cormc.sumo.network import SumoNetworkConfig, build_sumo_network
from cormc.sumo.spawn import SpawnRegistry, SumoSpawnAdapter


OK = "ok"
CORMC_FAILURE = "cormc_failure"
INTEGRATION_FAILURE = "integration_failure"


@dataclass(frozen=True)
class SumoClosedLoopRunnerConfig:
    run_id: str = "p17-sumo-closed-loop"
    output_dir: str | Path | None = None
    temp_dir: str | Path | None = None
    seed: int = 16001
    max_steps: int = 60
    scenario_id: str = "P17-SUMO-CLOSED-LOOP"
    scenario_config: Mapping[str, Any] | None = None
    gui_config_only: bool = False
    step_length: float = 0.1
    lateral_resolution: float = 0.25
    collision_action: str = "warn"
    p16_spawn_t: float = 3.95
    force_active_collision: bool = False
    background_vehicle_count: int = 2
    authority_config: SumoTrajectoryAuthorityConfig = field(default_factory=SumoTrajectoryAuthorityConfig)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SumoClosedLoopSimulationResult:
    run_id: str
    status: str
    sumo_version: str
    net_file: str
    route_file: str
    sumo_config_path: str
    sumocfg_file: str
    steps: int
    realization_records: tuple[RealizationRecord, ...] = ()
    generated_count: int = 0
    blocked_spawn_count: int = 0
    active_controlled_vehicle_ids: tuple[str, ...] = ()
    background_vehicle_ids_sample: tuple[str, ...] = ()
    collision_events: tuple[dict[str, Any], ...] = ()
    teleport_events: tuple[dict[str, Any], ...] = ()
    mismatch_count: int = 0
    collision_count: int = 0
    teleport_count: int = 0
    integration_failure_count: int = 0
    command_count: int = 0
    command_vehicle_ids_by_step: tuple[tuple[int, tuple[str, ...]], ...] = ()
    generated_vehicle_ids: tuple[str, ...] = ()
    spawn_failures: tuple[dict[str, Any], ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def run_sumo_trajectory_authority_simulation(config: Any | None = None) -> SumoClosedLoopSimulationResult:
    cfg = _normalize_config(config)
    out_dir = Path(cfg.output_dir or cfg.temp_dir or Path("artifacts") / "sumo" / cfg.run_id)
    out_dir.mkdir(parents=True, exist_ok=True)

    authority_cfg = _authority_config(cfg)
    network_files = build_sumo_network(
        out_dir,
        SumoNetworkConfig(
            step_length=authority_cfg.step_length,
            lateral_resolution=authority_cfg.lateral_resolution,
            collision_action=authority_cfg.collision_action,
            end=max(1.0, cfg.max_steps * authority_cfg.step_length + 1.0),
        ),
    )
    if cfg.gui_config_only:
        return SumoClosedLoopSimulationResult(
            run_id=cfg.run_id,
            status=OK,
            sumo_version=get_sumo_version(),
            net_file=network_files.net_file,
            route_file=network_files.routes_file,
            sumo_config_path=network_files.sumocfg_file,
            sumocfg_file=network_files.sumocfg_file,
            steps=0,
        )

    traci = import_traci()
    traci.start(
        [
            _sumo_binary(),
            "-c",
            network_files.sumocfg_file,
            "--no-step-log",
            "true",
            "--no-warnings",
            "true",
            "--collision.action",
            authority_cfg.collision_action,
            "--collision.check-junctions",
            "true",
        ]
    )

    records: list[RealizationRecord] = []
    collision_events: list[dict[str, Any]] = []
    teleport_events: list[dict[str, Any]] = []
    command_vehicle_ids_by_step: list[tuple[int, tuple[str, ...]]] = []
    active_controlled_ids: set[str] = set()
    generated_vehicle_ids: list[str] = []
    spawn_failures: list[dict[str, Any]] = []
    status = OK
    integration_failure_count = 0
    background_ids = tuple(f"BG_{index}" for index in range(max(2, cfg.background_vehicle_count)))
    spawn_registry = SpawnRegistry()

    try:
        _seed_sumo_scene(traci, cfg, background_ids=background_ids)
        traci.simulationStep()

        adapter = SumoRealizedStateAdapter(traci_module=traci)
        executor = MoveToXYTrajectoryExecutor(authority_cfg, traci_module=traci)
        monitor = RealizationMonitor(authority_cfg)
        spawn_adapter = SumoSpawnAdapter(traci_module=traci, registry=spawn_registry)
        authority_registry = ActiveControlRegistry(post_merge_hold_steps=authority_cfg.post_merge_hold_steps)
        p16_queue = _p16_on_ramp_queue(cfg)
        engine = CormcEngine(
            _scenario_config(cfg),
            run_id=cfg.run_id,
            random_queue=p16_queue,
            safe_spawn_gap_m=0.0,
        )
        previous_state = None

        for step in range(cfg.max_steps):
            t = round(step * authority_cfg.step_length, 10)
            state = adapter.adapt(
                _snapshot_for_adapter(traci, active_ids={"MV_ACTIVE"}, generated_ids=set(generated_vehicle_ids)),
                previous_state=previous_state,
                step=step,
                t=t,
                dt=authority_cfg.step_length,
            )
            engine_result = engine.advance_one_step(state)

            spawn_decisions = compute_spawn_decisions(
                p16_queue,
                state,
                safe_spawn_gap_m=0.0,
                eps=authority_cfg.step_length / 2.0,
            )
            for spawn_result in spawn_adapter.realize_decisions(spawn_decisions):
                if spawn_result.result == "generated":
                    generated_vehicle_ids.append(spawn_result.vehicle_id)
                elif spawn_result.result == "blocked":
                    pass
                elif spawn_result.result == INTEGRATION_FAILURE:
                    integration_failure_count += 1
                    spawn_failures.append(spawn_result.to_dict())
                    status = INTEGRATION_FAILURE

            authority = resolve_active_authority(
                state,
                engine_result.command_buffer,
                registry=authority_registry,
            )
            commands = _commands_from_engine_result(engine_result, authority)
            command_vehicle_ids_by_step.append((step, tuple(command.vehicle_id for command in commands)))
            active_controlled_ids.update(command.vehicle_id for command in commands)

            for command in commands:
                realized_now = read_realized_vehicle(traci, command.vehicle_id)
                try:
                    executor.apply_command(command, realized_speed_at_t=realized_now.v)
                except Exception as exc:
                    integration_failure_count += 1
                    spawn_failures.append(
                        {
                            "vehicle_id": command.vehicle_id,
                            "result": INTEGRATION_FAILURE,
                            "reason": "executor_apply_failed",
                            "error": str(exc),
                        }
                    )
                    status = INTEGRATION_FAILURE

            traci.simulationStep()
            step_collisions = collect_collision_events(
                traci,
                step=step,
                t=t,
                active_vehicle_ids=active_controlled_ids,
            )
            step_teleports = collect_teleport_events(
                traci,
                step=step,
                t=t,
                active_vehicle_ids=active_controlled_ids,
            )
            collision_events.extend(step_collisions)
            teleport_events.extend(step_teleports)
            collision_ids = _event_vehicle_ids(step_collisions)
            teleport_ids = _event_vehicle_ids(step_teleports)
            for command in commands:
                records.append(
                    monitor.classify_realization(
                        command,
                        executor.read_realized_vehicle(command.vehicle_id),
                        collision_vehicle_ids=collision_ids,
                        teleported_vehicle_ids=teleport_ids,
                    )
                )
            if any(vehicle_id in active_controlled_ids for vehicle_id in collision_ids):
                status = CORMC_FAILURE

            previous_state = engine_result.advanced_state
            if status == INTEGRATION_FAILURE:
                break
    except Exception as exc:
        status = INTEGRATION_FAILURE
        integration_failure_count += 1
        spawn_failures.append({"result": INTEGRATION_FAILURE, "reason": "runner_exception", "error": str(exc)})
    finally:
        traci.close(False)

    mismatch_count = sum(1 for record in records if record.result == MISMATCH)
    teleport_count = len(teleport_events) + sum(1 for record in records if record.result == TELEPORTED)
    collision_count = len(collision_events) + sum(1 for record in records if record.result == COLLIDED)
    blocked_spawn_count = len(spawn_registry.blocked_vehicle_ids)

    return SumoClosedLoopSimulationResult(
        run_id=cfg.run_id,
        status=status,
        sumo_version=get_sumo_version(),
        net_file=network_files.net_file,
        route_file=network_files.routes_file,
        sumo_config_path=network_files.sumocfg_file,
        sumocfg_file=network_files.sumocfg_file,
        steps=len(command_vehicle_ids_by_step),
        realization_records=tuple(records),
        generated_count=len(set(generated_vehicle_ids)),
        blocked_spawn_count=blocked_spawn_count,
        active_controlled_vehicle_ids=tuple(sorted(active_controlled_ids)),
        background_vehicle_ids_sample=background_ids[:5],
        collision_events=tuple(collision_events),
        teleport_events=tuple(teleport_events),
        mismatch_count=mismatch_count,
        collision_count=collision_count,
        teleport_count=teleport_count,
        integration_failure_count=integration_failure_count,
        command_count=sum(len(vehicle_ids) for _, vehicle_ids in command_vehicle_ids_by_step),
        command_vehicle_ids_by_step=tuple(command_vehicle_ids_by_step),
        generated_vehicle_ids=tuple(sorted(set(generated_vehicle_ids))),
        spawn_failures=tuple(spawn_failures),
    )


def _commands_from_engine_result(engine_result: Any, authority: Any) -> tuple[ControlledTrajectoryCommand, ...]:
    active = set(authority.active_vehicle_ids)
    commands: list[ControlledTrajectoryCommand] = []
    previous_state = engine_result.commit_result.previous_state
    next_state = engine_result.commit_result.next_state
    final_candidates = engine_result.commit_result.final_candidates
    for vehicle_id in next_state.active_vehicle_ids:
        if vehicle_id not in active:
            continue
        candidate = final_candidates.get(vehicle_id)
        if candidate is None:
            continue
        vehicle_state = next_state.vehicle_states[vehicle_id]
        maneuver = next_state.active_maneuvers.get(vehicle_id)
        commands.append(
            ControlledTrajectoryCommand(
                vehicle_id=vehicle_id,
                step=previous_state.step,
                t=previous_state.t,
                target_t=previous_state.t + previous_state.dt,
                x_global=float(candidate.x_global),
                y=float(candidate.y),
                v=float(candidate.v),
                a=float(candidate.a),
                physical_lane=vehicle_state.physical_lane,
                road_role=vehicle_state.road_role,
                authority_reason=",".join(authority.reasons_by_vehicle.get(vehicle_id, ())),
                source_candidate_id=getattr(candidate, "candidate_id", None),
                source_command_id=_source_command_id(engine_result.command_buffer, vehicle_id)
                or getattr(maneuver, "source_command_id", None),
                source_maneuver_type=_source_maneuver_type(engine_result.command_buffer, maneuver, vehicle_id),
                assigned_clv_id=_assigned_vehicle_id(engine_result.command_buffer, maneuver, vehicle_id, "assigned_clv_id"),
                assigned_cfv_id=_assigned_vehicle_id(engine_result.command_buffer, maneuver, vehicle_id, "assigned_cfv_id"),
            )
        )
    return tuple(commands)


def _source_command_id(command_buffer: Any, vehicle_id: str) -> str | None:
    for field_name in ("merge_commands", "lane_change_commands", "cooperation_commands"):
        command = getattr(command_buffer, field_name, {}).get(vehicle_id)
        if isinstance(command, Mapping) and command.get("command_id") is not None:
            return str(command["command_id"])
    speed_caps = getattr(command_buffer, "speed_cap_commands", {}).get(vehicle_id)
    if speed_caps:
        first = tuple(speed_caps)[0]
        if isinstance(first, Mapping) and first.get("command_id") is not None:
            return str(first["command_id"])
    return None


def _source_maneuver_type(command_buffer: Any, maneuver: Any, vehicle_id: str) -> str | None:
    if vehicle_id in getattr(command_buffer, "merge_commands", {}):
        return "merge"
    if vehicle_id in getattr(command_buffer, "lane_change_commands", {}):
        return "lane_change"
    return getattr(maneuver, "maneuver_type", None)


def _assigned_vehicle_id(command_buffer: Any, maneuver: Any, vehicle_id: str, key: str) -> str | None:
    command = getattr(command_buffer, "merge_commands", {}).get(vehicle_id)
    if isinstance(command, Mapping) and command.get(key) is not None:
        return str(command[key])
    value = getattr(maneuver, key, None)
    return str(value) if value is not None else None


def _normalize_config(config: Any | None) -> SumoClosedLoopRunnerConfig:
    if config is None:
        return SumoClosedLoopRunnerConfig()
    if isinstance(config, SumoClosedLoopRunnerConfig):
        return config
    if isinstance(config, Mapping):
        return SumoClosedLoopRunnerConfig(**dict(config))
    values = _plain_dataclass_or_object(config)
    known = {field.name for field in SumoClosedLoopRunnerConfig.__dataclass_fields__.values()}
    runner_values = {key: value for key, value in values.items() if key in known}
    if "authority_config" not in runner_values:
        runner_values["authority_config"] = SumoTrajectoryAuthorityConfig(
            **{key: value for key, value in values.items() if key in SumoTrajectoryAuthorityConfig.__dataclass_fields__}
        )
    return SumoClosedLoopRunnerConfig(**runner_values)


def _authority_config(cfg: SumoClosedLoopRunnerConfig) -> SumoTrajectoryAuthorityConfig:
    base = cfg.authority_config
    return SumoTrajectoryAuthorityConfig(
        step_length=cfg.step_length,
        lateral_resolution=cfg.lateral_resolution,
        collision_action=cfg.collision_action,
        post_merge_hold_steps=base.post_merge_hold_steps,
        executor_mode=base.executor_mode,
        mismatch_x_tolerance_m=base.mismatch_x_tolerance_m,
        mismatch_y_tolerance_m=base.mismatch_y_tolerance_m,
        mismatch_v_tolerance_mps=base.mismatch_v_tolerance_mps,
    )


def _seed_sumo_scene(traci: Any, cfg: SumoClosedLoopRunnerConfig, *, background_ids: tuple[str, ...]) -> None:
    _add_vehicle(traci, "MV_ACTIVE", "route_ramp", "cormc_active", x=6888.0, lane="on_ramp", road_role="on_ramp", v=16.0)
    if cfg.force_active_collision:
        _add_vehicle(traci, "BG_COLLIDE", "route_ramp", "sumo_background", x=6888.0, lane="on_ramp", road_role="on_ramp", v=0.0)
    for index, vehicle_id in enumerate(background_ids):
        lane = "lane_2" if index == 0 else "lane_1"
        x = 6890.0 + index * 42.0 + (cfg.seed % 7) * 0.1
        _add_vehicle(traci, vehicle_id, "route_main", "sumo_background", x=x, lane=lane, road_role="mainline", v=20.0 + index)


def _add_vehicle(traci: Any, vehicle_id: str, route_id: str, type_id: str, *, x: float, lane: str, road_role: str, v: float) -> None:
    edge_id, lane_index, depart_pos = to_sumo_position(x, lane, road_role)
    traci.vehicle.add(
        vehicle_id,
        route_id,
        typeID=type_id,
        depart="now",
        departLane=str(lane_index),
        departPos=f"{depart_pos:.6f}",
        departSpeed=f"{v:.6f}",
    )
    traci.vehicle.moveToXY(vehicle_id, edge_id, lane_index, x, -3.5 if lane == "on_ramp" else (3.5 if lane == "lane_1" else 0.0), 90.0, keepRoute=3)


def _snapshot_for_adapter(traci: Any, *, active_ids: set[str], generated_ids: set[str]) -> list[dict[str, Any]]:
    snapshot: list[dict[str, Any]] = []
    for vehicle_id in traci.vehicle.getIDList():
        x_sumo, y_sumo = traci.vehicle.getPosition(vehicle_id)
        is_active = vehicle_id in active_ids or vehicle_id in generated_ids
        snapshot.append(
            {
                "vehicle_id": vehicle_id,
                "x_global": x_sumo,
                "y": y_sumo,
                "v": traci.vehicle.getSpeed(vehicle_id),
                "edge_id": traci.vehicle.getRoadID(vehicle_id),
                "lane_id": traci.vehicle.getLaneID(vehicle_id),
                "lane_position": traci.vehicle.getLanePosition(vehicle_id),
                "vehicle_type": "cav" if is_active else "sumo_background",
                "compliance_state": "not_applicable",
                "is_active": True,
            }
        )
    return snapshot


def _scenario_config(cfg: SumoClosedLoopRunnerConfig) -> Mapping[str, Any]:
    if cfg.scenario_config is not None:
        return cfg.scenario_config
    return {
        "scenario_id": cfg.scenario_id,
        "scenario_name": cfg.scenario_id,
        "purpose": "P17 SUMO closed-loop authority smoke scenario",
        "test_level": "integration",
        "status": "required",
        "initial_time": {"t": 0.0, "step": 0, "dt": cfg.step_length},
        "initial_vehicles": [],
        "module_overrides": {
            "boundary_generation_enabled": True,
            "random_arrival_enabled": True,
            "random_vehicle_attributes_enabled": True,
            "ordinary_mainline_lane_change_enabled": False,
            "platoon_cmc_enabled": False,
            "mpc_lateral_tracking_enabled": False,
        },
        "preloaded_assignments": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": [],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [],
        "expected_png_features": [],
    }


def _p16_on_ramp_queue(cfg: SumoClosedLoopRunnerConfig):
    profile = SeededRandomProfile(
        seed=cfg.seed,
        profile_id="p16_internal_demo_v1",
        arrival_streams=(
            ArrivalStream(
                lane_id="on_ramp",
                shifted_headway=cfg.p16_spawn_t,
                initial_speed=16.0,
                spawn_x=6850.0,
                spawn_y=-3.5,
                mean_headway=cfg.p16_spawn_t + 0.01,
            ),
        ),
        cav_penetration_rate=1.0,
        chv_compliance_rate=1.0,
        safe_spawn_gap_m=0.0,
        max_queue_items_per_lane=1,
    )
    return generate_boundary_queue(profile, max_t=cfg.max_steps * cfg.step_length)


def _event_vehicle_ids(events: Iterable[Mapping[str, Any]]) -> set[str]:
    vehicle_ids: set[str] = set()
    for event in events:
        vehicle_ids.update(str(vehicle_id) for vehicle_id in event.get("vehicle_ids", ()) or ())
        for key in ("vehicle_id", "collider", "victim"):
            if event.get(key):
                vehicle_ids.add(str(event[key]))
    return vehicle_ids


def _sumo_binary() -> str:
    from cormc.sumo.env import ensure_sumo_tools_on_path

    return ensure_sumo_tools_on_path().sumo


def _plain_dataclass_or_object(value: Any) -> dict[str, Any]:
    if is_dataclass(value):
        return asdict(value)
    return {key: getattr(value, key) for key in dir(value) if not key.startswith("_") and not callable(getattr(value, key))}
