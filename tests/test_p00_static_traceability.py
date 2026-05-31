from __future__ import annotations

import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
P00_PATH = REPO_ROOT / "docs" / "执行计划" / "P00-Spec宪法_权威边界与二维追踪矩阵.md"


def _p00_text() -> str:
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


def test_p04_to_p12_are_only_trace_registered() -> None:
    rows = _traceability_rows()

    for index in range(4, 13):
        pxx = f"P{index:02d}"
        assert pxx in rows
        assert rows[pxx][9] == "trace_registered"
        assert rows[pxx][10] == "否"
        assert "spec_ready" not in rows[pxx][9]
        assert "implementation_ready" not in rows[pxx][9]


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
    assert "正式 PNG" in p11[7]
    assert "P11 不是首次实现日志 / sanity / MVS runner / PNG 口径" in p11[11]
    assert "不允许 P11 作为 P04-P10 日志、sanity、targeted MVS 或 PNG 口径的首次实现阶段" in text


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


def test_p04_to_p12_do_not_need_full_spec_but_must_be_trace_registered() -> None:
    text = _p00_text()
    rows = _traceability_rows()

    assert "P04-P12 缺完整执行计划不应导致 P00 失败" not in text
    assert "P04-P12 在当前阶段只要求 trace_registered" in text
    assert "不要求已有完整执行计划" in text
    assert "P04-P12 未进入 trace_registered 矩阵占位时失败" in text
    assert all(rows[f"P{index:02d}"][9] == "trace_registered" for index in range(4, 13))
