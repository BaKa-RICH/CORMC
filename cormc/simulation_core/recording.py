from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any

from cormc.simulation_core.engine import EngineStepResult, collect_step_history, unique_expected_png_features
from cormc.simulation_core.commit import OutputHistory


@dataclass
class FullRecorder:
    history: OutputHistory = field(default_factory=OutputHistory)
    expected_png_features: list[dict[str, Any]] = field(default_factory=list)

    def record_step(
        self,
        step_result: EngineStepResult,
        *,
        run_id: str,
        scenario_id: str,
    ) -> None:
        step_history = collect_step_history(
            step_result.trace,
            run_id=run_id,
            scenario_id=scenario_id,
        )
        self.history.trajectory_records.extend(step_history.trajectory_records)
        self.history.event_records.extend(step_history.event_records)
        self.history.sanity_check_records.extend(step_history.sanity_check_records)
        self.history.png_artifacts.extend(step_history.png_artifacts)
        self.expected_png_features.extend(step_result.trace.expected_png_features)

    def unique_expected_png_features(self) -> tuple[dict[str, Any], ...]:
        return tuple(unique_expected_png_features(self.expected_png_features))


@dataclass
class MinimalRecorder:
    history: OutputHistory = field(default_factory=OutputHistory)
    summaries: list[MappingProxyType] = field(default_factory=list)
    expected_png_features: list[dict[str, Any]] = field(default_factory=list)

    def record_step(
        self,
        step_result: EngineStepResult,
        *,
        run_id: str,
        scenario_id: str,
    ) -> None:
        failed_checks = [
            check
            for check in step_result.trace.actual_sanity_checks
            if check.get("result") == "fail"
        ]
        self.summaries.append(
            MappingProxyType(
                {
                    "run_id": run_id,
                    "scenario_id": scenario_id,
                    "step": step_result.trace.step,
                    "t": step_result.trace.t,
                    "advanced_step": step_result.advanced_state.step,
                    "advanced_t": step_result.advanced_state.t,
                    "elapsed_seconds": step_result.elapsed_seconds,
                    "event_count": len(step_result.trace.actual_events),
                    "sanity_count": len(step_result.trace.actual_sanity_checks),
                    "failed_sanity_count": len(failed_checks),
                }
            )
        )

    def unique_expected_png_features(self) -> tuple[dict[str, Any], ...]:
        return ()


@dataclass
class NullRecorder:
    history: OutputHistory = field(default_factory=OutputHistory)
    expected_png_features: list[dict[str, Any]] = field(default_factory=list)

    def record_step(
        self,
        step_result: EngineStepResult,
        *,
        run_id: str,
        scenario_id: str,
    ) -> None:
        return None

    def unique_expected_png_features(self) -> tuple[dict[str, Any], ...]:
        return ()
