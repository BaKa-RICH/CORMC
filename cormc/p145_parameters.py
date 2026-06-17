from __future__ import annotations

from dataclasses import dataclass


CORMC_PARAMETER_SPEC_SOURCE = "CORMC参数规格.md"
LOCKED_FORMULA_STATUS = "locked_formula"
FIRST_VERSION_PROXY_STATUS = "first_version_proxy"
ENGINEERING_PATCH_STATUS = "engineering_patch"
DISABLED_BY_SCOPE_STATUS = "disabled_by_scope"
TEST_HARNESS_OVERRIDE_STATUS = "test_harness_override"
DEFERRED_TO_P16_STATUS = "deferred_to_P16"


@dataclass(frozen=True)
class CAVParameters:
    k1: float = 0.4
    h_cav: float = 1.2
    u_min: float = -6.0
    u_max: float = 4.0
    a_min: float = -6.0
    a_max: float = 4.0
    v_e: float = 30.0
    d0: float = 2.0


@dataclass(frozen=True)
class CPIDParameters:
    kpx: float = 8.0
    kix: float = 0.0
    kdx: float = 10.0
    kpv: float = 5.0
    kiv: float = 0.0
    kdv: float = 0.0
    default_tau: float = 0.55
    gain_source: str = "first-version-default / to-review"
    default_tau_source: str = "deterministic_midpoint_of_U_0_4_0_7_for_p145"


@dataclass(frozen=True)
class IDMParameters:
    h_chv: float = 2.0
    a_i: float = 1.25
    b_i: float = 2.09
    d0: float = 2.0
    vehicle_length: float = 4.0
    default_desired_speed: float = 30.0


@dataclass(frozen=True)
class CUCParameters:
    alpha: float = -1.0
    beta: float = 1.5
    gamma: float = 0.5
    zeta: float = -0.5
    tt_min: float = 1.5


@dataclass(frozen=True)
class LaneChangeParameters:
    a_p: float = 0.1
    l_centerline: float = 100.0
    lane_width: float = 3.5


CAV = CAVParameters()
CPID = CPIDParameters()
IDM = IDMParameters()
CUC = CUCParameters()
LANE_CHANGE = LaneChangeParameters()

LANE_MAX_SPEED_MPS: dict[str, float] = {
    "lane_1": 33.0,
    "lane_2": 33.0,
    "on_ramp": 30.0,
}
