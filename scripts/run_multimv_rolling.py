from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from cormc.onestep.rolling.stage2_multimv_runner import run_multimv_rolling_archive


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run multi-MV rolling archive batch.")
    parser.add_argument("--output", type=Path, default=Path("artifacts/multimv_rolling"))
    parser.add_argument("--run-id")
    parser.add_argument("--scenario-id", action="append", dest="scenario_ids")
    args = parser.parse_args(argv)

    result = run_multimv_rolling_archive(
        args.output,
        scenario_ids=args.scenario_ids,
        run_id=args.run_id,
    )
    _print_summary(result)
    return 0


def _print_summary(result: dict[str, object]) -> None:
    results = list(result["results"])
    status_counts: dict[str, int] = {}
    for row in results:
        status = str(row["status"])
        status_counts[status] = status_counts.get(status, 0) + 1

    print(f"run_id: {result['run_id']}")
    print(f"output_root: {result['output_root']}")
    print(f"scenario_count: {result['scenario_count']}")
    print(f"status_counts: {status_counts}")
    print(f"manifest: {result['manifest_path']}")
    print(f"summary_csv: {result['summary_csv_path']}")
    print(f"report: {result['report_path']}")
    for row in results:
        print(
            "{scenario_id}: {status} ({completed}/{mv_count} MV, steps={actual_steps})".format(
                scenario_id=row["scenario_id"],
                status=row["status"],
                completed=_completed_mv_count(dict(row.get("mv_lifecycle") or {})),
                mv_count=row["mv_count"],
                actual_steps=row["steps_run"],
            )
        )


def _completed_mv_count(lifecycle_records: dict[str, object]) -> int:
    return sum(
        1
        for lifecycle in lifecycle_records.values()
        if isinstance(lifecycle, dict)
        and lifecycle.get("mainline_conversion_step") is not None
    )


if __name__ == "__main__":
    raise SystemExit(main())
