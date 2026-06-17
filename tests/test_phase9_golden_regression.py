from __future__ import annotations

from pathlib import Path

from scripts.phase9_golden import compare_against_baseline


def test_phase9_golden_regression_matches_baseline(tmp_path: Path) -> None:
    compare_against_baseline(
        Path("tests/golden/phase9_expected_fingerprints.json"),
        tmp_path / "phase9_golden",
    )
