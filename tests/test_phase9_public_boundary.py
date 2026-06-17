from __future__ import annotations

import importlib

import pytest


_ROOT = "cormc"


def _old(*parts: str) -> str:
    return ".".join((_ROOT, *parts))


@pytest.mark.parametrize(
    "module_name",
    [
        _old("ramp_merge_algorithm"),
        _old("ramp_merge_algorithm", "onestep_kernel"),
        _old("one_step_algorithm"),
        _old("one_step_runner"),
        _old("one_step_experiments"),
        _old("mvs"),
        _old("mvs", "loader"),
        _old("mvs", "matcher"),
        _old("mvs", "runner"),
        _old("random_generation"),
        _old("boundary_flow"),
        _old("step0_3"),
        _old("step4a_aps"),
        _old("step4b_cmc"),
        _old("step5_cooperative_request"),
        _old("step6_cuc"),
        _old("step7_longitudinal"),
        _old("step8_lateral"),
        _old("step9_11"),
    ],
)
def test_phase9_old_public_modules_are_not_importable(module_name: str) -> None:
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module(module_name)


@pytest.mark.parametrize(
    "module_name",
    [
        "cormc.scenes",
        "cormc.onestep",
        "cormc.onestep.rolling",
        "cormc.onestep.kernel",
        "cormc.onestep.lab",
        "cormc.observation",
        "cormc.sumo",
        "cormc.scenario_schema",
        "cormc.simulation_core",
        "cormc.traffic_flow",
        "cormc.legacy",
    ],
)
def test_phase9_formal_packages_import(module_name: str) -> None:
    assert importlib.import_module(module_name)


def test_phase9_top_level_package_exports_only_version() -> None:
    import cormc

    assert cormc.__all__ == ["__version__"]
    assert cormc.__version__ == "0.1.0"
    for name in (
        "build_prefreeze_workspace_from_scenario",
        "run_basic_numeric_suite",
        "run_one_step_fixed_scenario",
        "run_onestep_stage2_history",
        "RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID",
        "SumoArtifactResult",
    ):
        assert not hasattr(cormc, name)
