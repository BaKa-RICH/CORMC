from __future__ import annotations

from cormc.assignment_lifecycle import AssignmentRecord, assignment_lifecycle_manager


def test_legacy_assignment_converts_to_active_control_zone_record() -> None:
    record = assignment_lifecycle_manager.from_legacy_assignment(
        {
            "mv_id": "MV_A",
            "clv_id": "CLV_A",
            "cfv_id": "CFV_A",
            "aps_case": "case_3",
            "col_clv": True,
            "col_cfv": False,
            "desired_spacing_override": None,
            "t_star_mv": 12.0,
            "d_star_clv": 30.0,
            "d_star_cfv": 35.0,
            "status": "valid",
            "source": "aps_cache",
        },
        step=5,
        t=0.5,
    )

    assert isinstance(record, AssignmentRecord)
    assert record.lifecycle_state == "active_control_zone"
    assert record.gap_type == "bounded"
    assert record.mv_id == "MV_A"
    assert record.clv_id == "CLV_A"
    assert record.cfv_id == "CFV_A"
    assert record.t_mv_star == 12.0


def test_record_round_trip_preserves_assignment_fields() -> None:
    record = assignment_lifecycle_manager.from_legacy_assignment(
        {
            "mv_id": "MV_RT",
            "clv_id": "CLV_RT",
            "cfv_id": "CFV_RT",
            "aps_case": "case_2",
            "col_clv": False,
            "col_cfv": True,
            "desired_spacing_override": 58.0,
            "t_mv_star": 9.0,
            "d_star_clv": 44.0,
            "d_star_cfv": 40.0,
            "aps_min_merge_time_gap_s": 1.2,
            "status": "valid",
            "created_at_step": 3,
            "created_at_t": 0.3,
            "source": "aps_cache",
        },
        step=3,
        t=0.3,
    )

    payload = assignment_lifecycle_manager.to_state_dict(record)
    restored = assignment_lifecycle_manager.from_state_dict(payload)

    assert restored == record
    assert payload["record_version"] == 1
    assert payload["lifecycle_state"] == "active_control_zone"
    assert payload["desired_spacing_override"] == 58.0
    assert payload["aps_min_merge_time_gap_s"] == 1.2


def test_retained_failure_and_recovery_views_follow_lifecycle_rules() -> None:
    record = assignment_lifecycle_manager.from_legacy_assignment(
        {
            "mv_id": "MV_RULE",
            "clv_id": "CLV_RULE",
            "cfv_id": "CFV_RULE",
            "aps_case": "case_3",
            "col_clv": True,
            "col_cfv": False,
            "t_star_mv": 10.0,
            "d_star_clv": 20.0,
            "d_star_cfv": 20.0,
            "status": "valid",
        },
        step=10,
        t=1.0,
    )
    state = type("State", (), {"step": 11, "t": 1.1})()

    retained = assignment_lifecycle_manager.retain_after_aps_failure(
        state,
        record,
        "insufficient_candidates",
    )
    recovery = assignment_lifecycle_manager.mark_recovery_required(
        state,
        record,
        "wrong_order",
    )

    assert assignment_lifecycle_manager.derive_step5_view(state, retained) is not None
    assert assignment_lifecycle_manager.derive_step5_view(state, recovery) is None
    assert assignment_lifecycle_manager.derive_cmc_view(state, recovery) is None
