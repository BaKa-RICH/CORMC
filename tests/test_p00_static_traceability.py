from __future__ import annotations

import re
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
P00_PATH = REPO_ROOT / "docs" / "执行计划" / "P00-Spec宪法_权威边界与二维追踪矩阵.md"


def _p00_text() -> str:
    if not P00_PATH.exists():
        pytest.skip(f"P00 static traceability document is not present: {P00_PATH}")
    return P00_PATH.read_text(encoding="utf-8")


def _traceability_rows() -> dict[str, list[str]]:
    rows: dict[str, list[str]] = {}
    for line in _p00_text().splitlines():
        if not line.startswith("| P"):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if cells and re.fullmatch(r"P\d{2}", cells[0]):
            rows[cells[0]] = cells
    return rows


def test_p01_to_p03_are_spec_ready_and_implementation_ready() -> None:
    rows = _traceability_rows()

    for pxx in ("P01", "P02", "P03"):
        assert pxx in rows
        assert rows[pxx][9] == "spec_ready + implementation_ready"
        assert rows[pxx][10] == "是"


def test_p04_to_p16_are_implemented_green_after_p16_closure() -> None:
    rows = _traceability_rows()

    for index in range(4, 17):
        pxx = f"P{index:02d}"
        assert pxx in rows
        assert rows[pxx][9] == "implemented_green"
        assert rows[pxx][10] == "是"


def test_p17_is_implemented_green_and_p18_remains_registered_future_route() -> None:
    rows = _traceability_rows()

    assert rows["P17"][9] == "implemented_green"
    assert rows["P17"][10] == "是"
    for index in range(18, 19):
        pxx = f"P{index:02d}"
        assert pxx in rows
        assert rows[pxx][9] == "trace_registered"
        assert rows[pxx][10] == "否"


def test_every_pxx_has_step_gate_upstream_spec_and_evidence() -> None:
    rows = _traceability_rows()

    for index in range(1, 13):
        pxx = f"P{index:02d}"
        cells = rows[pxx]
        assert cells[1], f"{pxx} missing step scope"
        assert cells[2], f"{pxx} missing required gate"
        assert cells[5], f"{pxx} missing event evidence"
        assert cells[6], f"{pxx} missing sanity evidence"
        assert cells[7], f"{pxx} missing PNG/artifact evidence"
        assert cells[8], f"{pxx} missing upstream spec"


def test_p11_is_delivery_aggregation_not_first_log_sanity_mvs_or_png_stage() -> None:
    p11 = _traceability_rows()["P11"]
    text = _p00_text()

    assert "交付级" in p11[1]
    assert "Step 10" not in p11[1]
    assert "PNG / artifact manifest / regression report" in p11[7]
    assert "正式自然输出包交给 P14" in p11[11]
    assert "不允许 P11 作为 P04-P10 日志、sanity、targeted MVS 或 PNG 口径的首次实现阶段" in text


def test_p12_is_implemented_after_p11_and_not_collapsed_into_delivery() -> None:
    rows = _traceability_rows()
    text = _p00_text()

    assert list(rows).index("P12") > list(rows).index("P11")
    p12 = rows["P12"]
    assert "Deterministic full simulation loop" in p12[1]
    assert "Step0-11 多步推进" in p12[1]
    assert "demo PNG" in p12[7]
    assert p12[9] == "implemented_green"
    assert p12[10] == "是"
    assert "P12 deterministic full simulation loop 已完成" in text
    assert "P13 official required MVS closure 已完成" in text


def test_engineering_patches_require_source_reason_and_engineering_patch_flag() -> None:
    text = _p00_text()

    assert "`source`、`reason`、`is_engineering_patch`" in text
    for patch_name in (
        "first_APS(MV)",
        "assignment invalid",
        "immediate APS refresh",
        "多 MV 共享 CV 仲裁",
        "same-step maneuver relation overlay",
        "每车每步只提交一次",
        "boundary speed cap 不可行时的保守处理入口",
        "unexpected ordinary lane-change attempt",
    ):
        assert patch_name in text


def test_p04_to_p17_current_facts_do_not_backslide_to_registered_only() -> None:
    text = _p00_text()
    rows = _traceability_rows()

    assert "P04-P12 缺完整执行计划不应导致 P00 失败" not in text
    assert "P04-P17 的追踪条目反映当前已完成事实" in text
    assert "P18 的完整执行计划在对应阶段另行编写" in text
    assert "P18 在 P00 中只登记后续路线" in text
    assert "20 required MVS" in text
    assert "pre_p15" in text
    assert "post_p15" in text
    assert "p15_comparison_report.json" in text
    assert "P16 seeded random internal demo" in text
    assert all(rows[f"P{index:02d}"][9] == "implemented_green" for index in range(4, 18))
    assert all(rows[f"P{index:02d}"][10] == "是" for index in range(4, 18))


def test_p16_static_traceability_records_seeded_random_gate_evidence() -> None:
    text = _p00_text()
    row = _traceability_rows()["P16"]

    assert row[9] == "implemented_green"
    assert row[10] == "是"
    assert "seed" in row[5]
    assert "random disabled" in row[2]
    assert "same seed same config" in row[2]
    assert "pre-freeze only" in row[6]
    assert "artifacts/random/p16_seeded_demo" in row[7]
    assert "run_seeded_random_simulation" in row[11]
    assert "run_p16_seeded_random_artifact_bundle" in row[11]
    assert "P17 | SUMO coupling minimal closure" in text
    assert "P18 | Dual-track paper experiment grid" in text
