"""One-Step single-frame lab helpers."""

_EXPORTS = {
    "build_one_step_artifacts_for_reference": ("cormc.onestep.lab.artifacts", "build_one_step_artifacts_for_reference"),
    "get_reference_algorithm_config": ("cormc.onestep.lab.reference_case", "get_reference_algorithm_config"),
    "get_reference_expected": ("cormc.onestep.lab.reference_case", "get_reference_expected"),
    "get_reference_scenario": ("cormc.onestep.lab.reference_case", "get_reference_scenario"),
    "run_one_step_fixed_scenario": ("cormc.onestep.lab.runner", "run_one_step_fixed_scenario"),
    "run_one_step_sweep_scenario": ("cormc.onestep.lab.runner", "run_one_step_sweep_scenario"),
}


def __getattr__(name: str):
    try:
        module_name, attr_name = _EXPORTS[name]
    except KeyError as exc:
        raise AttributeError(f"module 'cormc.onestep.lab' has no attribute {name!r}") from exc
    from importlib import import_module

    value = getattr(import_module(module_name), attr_name)
    globals()[name] = value
    return value


__all__ = list(_EXPORTS)
