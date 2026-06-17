from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_one_step_gap_builder_public_api_exports() -> None:
    from cormc.onestep.kernel import (
        DerivedParams,
        Gap,
        build_gaps,
        compute_derived_params,
    )

    assert DerivedParams.__name__ == "DerivedParams"
    assert Gap.__name__ == "Gap"
    assert callable(compute_derived_params)
    assert callable(build_gaps)


def test_compute_derived_params_matches_reference_formulas() -> None:
    from cormc.onestep.kernel import compute_derived_params
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_scenario,
    )

    params = compute_derived_params(
        get_reference_scenario(),
        get_reference_algorithm_config(),
    )

    assert params.b == 4.0
    assert params.G_req == 85.0
    assert params.G_adj == 45.0


def test_build_gaps_returns_expected_gap_count_and_identifiers() -> None:
    from cormc.onestep.kernel import (
        Gap,
        build_gaps,
    )
    from cormc.onestep.lab import get_reference_scenario

    gaps = build_gaps(get_reference_scenario())

    assert len(gaps) == 6
    assert all(isinstance(gap, Gap) for gap in gaps)
    assert tuple(gap.gap_id for gap in gaps) == (
        "gap1",
        "gap2",
        "gap3",
        "gap4",
        "gap5",
        "gap6",
    )
    assert tuple(gap.index for gap in gaps) == (0, 1, 2, 3, 4, 5)
    assert gaps[0].x_rear == -180.0
    assert gaps[0].x_front == -90.0
    assert gaps[-1].x_rear == 190.0
    assert gaps[-1].x_front == 250.0


def test_build_gaps_matches_stage1_reference_rows() -> None:
    from cormc.onestep.kernel import build_gaps
    from cormc.onestep.lab import (
        get_reference_expected,
        get_reference_scenario,
    )

    gaps = build_gaps(get_reference_scenario())
    reference_rows = get_reference_expected().gap_rows

    assert len(gaps) == len(reference_rows)

    for gap, row in zip(gaps, reference_rows):
        assert gap.gap_id == row.gap_id
        assert gap.x_rear == row.x_rear
        assert gap.x_front == row.x_front
        assert gap.G_i == row.G_i
        assert gap.c_i == row.c_i
        assert gap.D_i == row.D_i

    by_id = {gap.gap_id: gap for gap in gaps}
    assert by_id["gap1"].G_i == 90.0
    assert by_id["gap2"].c_i == -57.5
    assert by_id["gap3"].D_i == 2.5
    assert by_id["gap4"].c_i == 70.0
    assert by_id["gap5"].G_i == 80.0
    assert by_id["gap6"].D_i == 220.0
