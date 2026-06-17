from __future__ import annotations

import argparse
import json
from pathlib import Path

from cormc.observation.artifacts import build_observation_artifact_bundle


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build unified observation plots, artifacts, and SUMO replay files.")
    parser.add_argument("--source-dir", required=True)
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--play", action="store_true")
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args(argv)

    if not (args.build or args.play or args.smoke):
        parser.error("one of --build, --play, or --smoke is required")

    bundle = build_observation_artifact_bundle(
        Path(args.source_dir),
        play=args.play,
        smoke=args.smoke,
    )
    print(
        json.dumps(
            {
                "scenario_id": bundle.scenario_id,
                "run_id": bundle.run_id,
                "status": bundle.status,
                "manifest_path": bundle.manifest_path,
                "report_path": bundle.report_path,
            },
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if bundle.status == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
