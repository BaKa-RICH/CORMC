from __future__ import annotations

import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType


@dataclass(frozen=True)
class SumoBinaryPaths:
    sumo_home: str | None
    bin_dir: str | None
    sumo: str
    netconvert: str
    sumo_gui: str | None = None


def discover_sumo_binaries() -> SumoBinaryPaths | None:
    """Find SUMO command paths from SUMO_HOME/bin first, then PATH."""
    sumo_home = os.environ.get("SUMO_HOME")
    bin_dir = _sumo_bin_dir(sumo_home)

    sumo = _find_binary("sumo", bin_dir)
    netconvert = _find_binary("netconvert", bin_dir)
    if sumo is None or netconvert is None:
        return None

    return SumoBinaryPaths(
        sumo_home=sumo_home,
        bin_dir=str(bin_dir) if bin_dir is not None and bin_dir.exists() else None,
        sumo=sumo,
        netconvert=netconvert,
        sumo_gui=_find_binary("sumo-gui", bin_dir),
    )


def ensure_sumo_tools_on_path() -> SumoBinaryPaths:
    """Make SUMO bin/tools importable and return discovered binary paths."""
    paths = discover_sumo_binaries()
    if paths is None:
        raise RuntimeError("SUMO binaries not found; set SUMO_HOME or add sumo/netconvert to PATH")

    if paths.bin_dir is not None:
        _prepend_env_path("PATH", paths.bin_dir)

    if paths.sumo_home is not None:
        tools_dir = Path(paths.sumo_home) / "tools"
        if tools_dir.exists():
            tools_path = str(tools_dir)
            if tools_path not in sys.path:
                sys.path.insert(0, tools_path)
            _prepend_env_path("PYTHONPATH", tools_path)

    return paths


def import_traci() -> ModuleType:
    """Import traci only inside the SUMO layer."""
    ensure_sumo_tools_on_path()
    import importlib

    return importlib.import_module("traci")


def import_sumolib() -> ModuleType:
    """Import sumolib only inside the SUMO layer."""
    ensure_sumo_tools_on_path()
    import importlib

    return importlib.import_module("sumolib")


def get_sumo_version(sumo_binary: str | None = None) -> str:
    paths = ensure_sumo_tools_on_path()
    binary = sumo_binary or paths.sumo
    completed = subprocess.run(
        [binary, "--version"],
        check=True,
        capture_output=True,
        encoding="utf-8",
    )
    first_line = (completed.stdout or completed.stderr).splitlines()[0].strip()
    if not first_line:
        raise RuntimeError(f"Could not read SUMO version from {binary!r}")
    return first_line


def ensure_sumo_available_or_skip() -> SumoBinaryPaths:
    """Return SUMO paths, or pytest.skip when SUMO is genuinely unavailable."""
    paths = discover_sumo_binaries()
    if paths is not None:
        ensure_sumo_tools_on_path()
        return paths

    try:
        import pytest
    except ModuleNotFoundError as exc:
        raise RuntimeError("SUMO is unavailable and pytest is not importable for skip handling") from exc
    pytest.skip("SUMO binaries not found; set SUMO_HOME or add sumo/netconvert to PATH")


def _sumo_bin_dir(sumo_home: str | None) -> Path | None:
    if not sumo_home:
        return None
    return Path(sumo_home) / "bin"


def _find_binary(name: str, bin_dir: Path | None) -> str | None:
    executable = f"{name}.exe" if os.name == "nt" else name
    if bin_dir is not None:
        candidate = bin_dir / executable
        if candidate.exists():
            return str(candidate)
    return shutil.which(name)


def _prepend_env_path(variable: str, path: str) -> None:
    current = os.environ.get(variable, "")
    entries = [entry for entry in current.split(os.pathsep) if entry]
    if path not in entries:
        os.environ[variable] = os.pathsep.join([path, *entries])
