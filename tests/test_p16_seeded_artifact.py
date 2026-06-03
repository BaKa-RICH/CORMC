from __future__ import annotations

import json
from pathlib import Path

from cormc.p14_artifacts import run_p16_seeded_random_artifact_bundle


def test_p16_seeded_artifact_bundle_writes_formal_outputs(tmp_path: Path) -> None:
    result = run_p16_seeded_random_artifact_bundle(
        run_id="p16-artifact",
        output_root=tmp_path,
        seed=16001,
        max_steps=20,
    )
    output_dir = tmp_path / "p16-artifact"

    assert Path(result.output_dir) == output_dir
    for filename in (
        "trajectory.csv",
        "events.jsonl",
        "sanity.jsonl",
        "time_space.png",
        "artifact_manifest.json",
        "run_report.md",
        "scenario_report.json",
    ):
        path = output_dir / filename
        assert path.exists()
        assert path.stat().st_size > 0

    assert (output_dir / "time_space.png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert result.generated_count > 0


def test_p16_seeded_artifact_manifest_records_random_metadata(tmp_path: Path) -> None:
    result = run_p16_seeded_random_artifact_bundle(
        run_id="p16-manifest",
        output_root=tmp_path,
        seed=16001,
        max_steps=20,
    )
    manifest = json.loads(Path(result.artifact_manifest_path).read_text(encoding="utf-8"))
    report = json.loads((tmp_path / "p16-manifest" / "scenario_report.json").read_text(encoding="utf-8"))

    assert manifest["run_id"] == "p16-manifest"
    assert manifest["scenario_id"] == "P16-RANDOM-DEMO-internal"
    assert manifest["random_enabled"] is True
    assert manifest["seed"] == 16001
    assert manifest["profile_id"] == "p16_internal_demo_v1"
    assert manifest["max_steps"] == 20
    assert manifest["generated_count"] == result.generated_count
    assert manifest["blocked_spawn_count"] == result.blocked_spawn_count
    assert manifest["output_paths"]["trajectory"].endswith("trajectory.csv")
    assert report["random_enabled"] is True
    assert report["generated_count"] == result.generated_count


def test_p16_same_seed_artifact_csv_and_jsonl_are_reproducible(tmp_path: Path) -> None:
    left = run_p16_seeded_random_artifact_bundle(
        run_id="p16-left",
        output_root=tmp_path,
        seed=16001,
        max_steps=20,
    )
    right = run_p16_seeded_random_artifact_bundle(
        run_id="p16-right",
        output_root=tmp_path,
        seed=16001,
        max_steps=20,
    )

    assert _normalized(Path(left.bundle.exports["trajectory"]).read_text(encoding="utf-8")) == _normalized(
        Path(right.bundle.exports["trajectory"]).read_text(encoding="utf-8")
    )
    assert _normalized(Path(left.bundle.exports["events"]).read_text(encoding="utf-8")) == _normalized(
        Path(right.bundle.exports["events"]).read_text(encoding="utf-8")
    )
    assert _normalized(Path(left.bundle.exports["sanity"]).read_text(encoding="utf-8")) == _normalized(
        Path(right.bundle.exports["sanity"]).read_text(encoding="utf-8")
    )


def test_p16_seeded_artifact_does_not_write_baseline_directory(tmp_path: Path) -> None:
    baseline = tmp_path / "baseline" / "pre_p15"
    baseline.mkdir(parents=True)
    marker = baseline / "existing.txt"
    marker.write_text("keep", encoding="utf-8")

    result = run_p16_seeded_random_artifact_bundle(
        run_id="p16-random",
        output_root=tmp_path / "random" / "p16_seeded_demo",
        seed=16001,
        max_steps=20,
    )

    assert marker.read_text(encoding="utf-8") == "keep"
    assert "baseline" not in result.output_dir


def _normalized(text: str) -> str:
    return text.replace("p16-left", "RUN").replace("p16-right", "RUN")
