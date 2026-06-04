from __future__ import annotations

import subprocess

from cormc.sumo.env import (
    discover_sumo_binaries,
    ensure_sumo_available_or_skip,
    ensure_sumo_tools_on_path,
    get_sumo_version,
    import_sumolib,
    import_traci,
)


def test_p17_discovers_real_sumo_binaries() -> None:
    paths = ensure_sumo_available_or_skip()

    assert paths.sumo.endswith(("sumo", "sumo.exe"))
    assert paths.netconvert.endswith(("netconvert", "netconvert.exe"))
    assert discover_sumo_binaries() is not None


def test_p17_reads_sumo_version() -> None:
    ensure_sumo_available_or_skip()

    version = get_sumo_version()

    assert "SUMO" in version.upper()


def test_p17_import_helpers_load_sumo_python_tools() -> None:
    ensure_sumo_available_or_skip()
    ensure_sumo_tools_on_path()

    traci = import_traci()
    sumolib = import_sumolib()

    assert traci.__name__ == "traci"
    assert sumolib.__name__ == "sumolib"


def test_p17_sumo_binary_smoke() -> None:
    paths = ensure_sumo_available_or_skip()

    completed = subprocess.run(
        [paths.sumo, "--version"],
        check=True,
        capture_output=True,
        encoding="utf-8",
    )

    assert "SUMO" in completed.stdout.upper()
