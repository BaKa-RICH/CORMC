from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from cormc.observation.artifacts import build_observation_artifact_bundle
from cormc.onestep.rolling import (
    export_onestep_stage2_analysis,
    run_onestep_stage2_analysis,
    run_onestep_stage2_random_history,
)
from cormc.scenes import (
    RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID,
    RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
    build_onestep_random_lane2_ramp_scene,
)

DEFAULT_BASELINE = Path("tests/golden/phase9_expected_fingerprints.json")
DEFAULT_OUTPUT = Path("artifacts/phase9_regression/check")


@dataclass(frozen=True)
class GoldenCase:
    scenario_id: str
    max_steps: int
    run_id: str
    random: bool = False


GOLDEN_CASES = (
    GoldenCase(
        RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
        420,
        "phase9-golden-s05-plan-step0",
    ),
    GoldenCase(
        RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
        420,
        "phase9-golden-s05-rolling-entry",
    ),
    GoldenCase(
        RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
        420,
        "phase9-golden-s07-plan-step0",
    ),
    GoldenCase(
        RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
        420,
        "phase9-golden-s07-rolling-entry",
    ),
    GoldenCase(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
        700,
        "phase9-golden-s07-2mv-rolling-entry",
    ),
    GoldenCase(
        RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID,
        1000,
        "phase9-golden-random-s07-lane2-ramp-100s-seed645001",
        random=True,
    ),
)


def run_golden(output_root: str | Path) -> dict[str, Any]:
    output = Path(output_root)
    output.mkdir(parents=True, exist_ok=True)
    fingerprints: dict[str, Any] = {}
    for case in GOLDEN_CASES:
        if case.random:
            result = _run_random_case(case, output)
        else:
            result = run_onestep_stage2_analysis(
                case.scenario_id,
                output,
                max_steps=case.max_steps,
                run_id=case.run_id,
            )
        bundle = build_observation_artifact_bundle(
            Path(result.summary_json_path).parent,
            play=False,
            smoke=False,
        )
        fingerprints[case.scenario_id] = build_fingerprint(
            Path(result.summary_json_path).parent,
            bundle_manifest_path=Path(bundle.manifest_path),
        )
    return {"schema": "phase9_golden_fingerprints.v1", "cases": fingerprints}


def _run_random_case(case: GoldenCase, output_root: Path):
    spec = build_onestep_random_lane2_ramp_scene(
        seed=645001,
        horizon_s=100.0,
        max_steps=case.max_steps,
    )
    history_run = run_onestep_stage2_random_history(
        spec,
        max_steps=case.max_steps,
        run_id=case.run_id,
    )
    return export_onestep_stage2_analysis(
        history_run.summary,
        output_root / case.scenario_id / case.run_id,
        history=history_run.history,
    )


def build_fingerprint(
    source_dir: str | Path,
    *,
    bundle_manifest_path: str | Path | None = None,
) -> dict[str, Any]:
    source = Path(source_dir)
    summary = _load_json(source / "stage2_summary.json")
    trajectory_rows = _normalized_trajectory_rows(source / "trajectory.csv")
    gap_rows = _normalize(_load_json(source / "gap_rows.json"))
    manifest_path = Path(bundle_manifest_path or source / "stage8_artifact_manifest.json")
    manifest = _load_json(manifest_path)
    replay = dict(manifest["validation"]["replay_fidelity"])
    return {
        "stage2_summary": _normalize_summary(summary),
        "trajectory_csv": trajectory_rows,
        "gap_rows": gap_rows,
        "observation_bundle": {
            "artifact_schema": manifest.get("artifact_schema"),
            "validation_status": manifest.get("validation", {}).get("status"),
            "replay_fidelity_status": replay.get("status"),
            "status": manifest.get("status"),
        },
    }


def compare_against_baseline(
    baseline_path: str | Path,
    output_root: str | Path,
) -> dict[str, Any]:
    expected = _load_json(Path(baseline_path))
    actual = run_golden(output_root)
    if actual != expected:
        raise AssertionError(_diff_summary(expected, actual))
    return actual


def write_baseline(path: str | Path, output_root: str | Path) -> dict[str, Any]:
    payload = run_golden(output_root)
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return payload


def _normalize_summary(summary: Mapping[str, Any]) -> Any:
    return _normalize(_drop_noise(summary))


def _drop_noise(value: Any) -> Any:
    if isinstance(value, Mapping):
        result: dict[str, Any] = {}
        for key, item in value.items():
            if key == "artifact_paths" or key == "run_id":
                continue
            result[str(key)] = _drop_noise(item)
        return result
    if isinstance(value, list):
        return [_drop_noise(item) for item in value]
    if isinstance(value, tuple):
        return [_drop_noise(item) for item in value]
    return value


def _normalized_trajectory_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        rows: list[dict[str, str]] = []
        for row in reader:
            item = dict(row)
            if "run_id" in item:
                item["run_id"] = "<run_id>"
            rows.append(item)
        return rows


def _normalize(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _normalize(value[key]) for key in sorted(value)}
    if isinstance(value, list):
        return [_normalize(item) for item in value]
    if isinstance(value, tuple):
        return [_normalize(item) for item in value]
    return value


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _diff_summary(expected: Mapping[str, Any], actual: Mapping[str, Any]) -> str:
    expected_cases = set(expected.get("cases", {}))
    actual_cases = set(actual.get("cases", {}))
    missing = sorted(expected_cases - actual_cases)
    extra = sorted(actual_cases - expected_cases)
    changed = [
        scenario_id
        for scenario_id in sorted(expected_cases & actual_cases)
        if expected["cases"][scenario_id] != actual["cases"][scenario_id]
    ]
    return (
        "phase9 golden fingerprints differ: "
        f"missing={missing}, extra={extra}, changed={changed}"
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run Phase 9 golden experiments.")
    parser.add_argument("--write-baseline", type=Path)
    parser.add_argument("--compare", type=Path)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    if bool(args.write_baseline) == bool(args.compare):
        parser.error("choose exactly one of --write-baseline or --compare")
    if args.write_baseline:
        write_baseline(args.write_baseline, args.output)
        print(f"wrote baseline: {args.write_baseline}")
    else:
        compare_against_baseline(args.compare, args.output)
        print(f"compare passed: {args.compare}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
