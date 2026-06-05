from __future__ import annotations

from dataclasses import dataclass, fields
from types import MappingProxyType
from typing import Any, Mapping


GAP_TYPES = frozenset({"bounded", "front_only", "rear_only", "unbounded_clear"})
LIFECYCLE_STATES = frozenset(
    {
        "absent",
        "active_control_zone",
        "refresh_failed_retained",
        "active_merge_zone",
        "invalid",
        "recovery_required",
        "completed",
    }
)
STEP5_CONSUMABLE_STATES = frozenset(
    {"active_control_zone", "refresh_failed_retained", "active_merge_zone"}
)
CMC_CONSUMABLE_STATES = frozenset(
    {"active_control_zone", "refresh_failed_retained", "active_merge_zone"}
)
CONTROL_ZONE_GAP_PROTECTION_STATES = frozenset(
    {"active_control_zone", "refresh_failed_retained"}
)


@dataclass(frozen=True)
class AssignmentRecord:
    record_version: int
    mv_id: str
    clv_id: str | None
    cfv_id: str | None
    gap_type: str
    aps_case: str | None
    col_clv: bool
    col_cfv: bool
    desired_spacing_override: float | None
    status: str
    lifecycle_state: str
    created_at_step: int | None
    created_at_t: float | None
    last_updated_step: int | None
    last_updated_t: float | None
    last_validated_step: int | None
    last_validated_t: float | None
    invalid_reason: str | None
    recovery_reason: str | None
    source: str
    t_star_mv: float | None = None
    t_mv_star: float | None = None
    d_star_clv: float | None = None
    d_star_cfv: float | None = None
    aps_min_merge_time_gap_s: float | None = None
    valid_until_next_aps: bool | None = None
    staleness_policy: str | None = None


@dataclass(frozen=True)
class AssignmentStepView:
    mv_id: str
    record: Mapping[str, Any]
    source: str
    consumable_by_step5: bool
    consumable_by_cmc: bool

    @property
    def assignment(self) -> Mapping[str, Any]:
        return self.record


class AssignmentLifecycleManager:
    def from_legacy_assignment(
        self,
        value: Mapping[str, Any],
        *,
        step: int,
        t: float,
    ) -> AssignmentRecord:
        return self.from_state_dict(
            {
                **dict(value),
                "record_version": int(value.get("record_version", 1)),
                "gap_type": str(value.get("gap_type") or "bounded"),
                "lifecycle_state": str(
                    value.get("lifecycle_state") or _default_lifecycle_state(value)
                ),
                "last_updated_step": value.get(
                    "last_updated_step", value.get("created_at_step", step)
                ),
                "last_updated_t": value.get("last_updated_t", value.get("created_at_t", t)),
            }
        )

    def to_state_dict(self, record: AssignmentRecord) -> dict[str, Any]:
        return {
            field.name: getattr(record, field.name)
            for field in fields(record)
            if getattr(record, field.name) is not None
        }

    def from_state_dict(self, value: Mapping[str, Any]) -> AssignmentRecord:
        gap_type = str(value.get("gap_type") or "bounded")
        if gap_type not in GAP_TYPES:
            raise ValueError(f"unsupported assignment gap_type: {gap_type}")
        lifecycle_state = str(value.get("lifecycle_state") or _default_lifecycle_state(value))
        if lifecycle_state not in LIFECYCLE_STATES:
            raise ValueError(f"unsupported assignment lifecycle_state: {lifecycle_state}")
        mv_id = _optional_str(value.get("mv_id"))
        if mv_id is None:
            raise ValueError("assignment record requires mv_id")
        t_star = _optional_float(value.get("t_star_mv", value.get("t_mv_star")))
        return AssignmentRecord(
            record_version=int(value.get("record_version", 1)),
            mv_id=mv_id,
            clv_id=_optional_str(value.get("clv_id")),
            cfv_id=_optional_str(value.get("cfv_id")),
            gap_type=gap_type,
            aps_case=_optional_str(value.get("aps_case")),
            col_clv=_truthy_bool(value.get("col_clv")),
            col_cfv=_truthy_bool(value.get("col_cfv")),
            desired_spacing_override=_optional_float(value.get("desired_spacing_override")),
            status=str(value.get("status") or "valid"),
            lifecycle_state=lifecycle_state,
            created_at_step=_optional_int(value.get("created_at_step")),
            created_at_t=_optional_float(value.get("created_at_t")),
            last_updated_step=_optional_int(value.get("last_updated_step")),
            last_updated_t=_optional_float(value.get("last_updated_t")),
            last_validated_step=_optional_int(value.get("last_validated_step")),
            last_validated_t=_optional_float(value.get("last_validated_t")),
            invalid_reason=_optional_str(value.get("invalid_reason")),
            recovery_reason=_optional_str(value.get("recovery_reason")),
            source=str(value.get("source") or "legacy_assignment"),
            t_star_mv=t_star,
            t_mv_star=t_star,
            d_star_clv=_optional_float(value.get("d_star_clv")),
            d_star_cfv=_optional_float(value.get("d_star_cfv")),
            aps_min_merge_time_gap_s=_optional_float(value.get("aps_min_merge_time_gap_s")),
            valid_until_next_aps=(
                None
                if value.get("valid_until_next_aps") is None
                else _truthy_bool(value.get("valid_until_next_aps"))
            ),
            staleness_policy=_optional_str(value.get("staleness_policy")),
        )

    def create_from_aps_success(self, state: Any, aps_assignment: Any) -> AssignmentRecord:
        value = aps_assignment.to_cache_value(t=state.t, step=state.step)
        value.update(
            {
                "record_version": 1,
                "gap_type": "bounded",
                "lifecycle_state": "active_control_zone",
                "last_updated_step": state.step,
                "last_updated_t": state.t,
                "source": "aps_updated_this_step",
            }
        )
        return self.from_state_dict(value)

    def retain_after_aps_failure(
        self,
        state: Any,
        record: AssignmentRecord,
        failure_reason: str,
    ) -> AssignmentRecord:
        return self._replace(
            record,
            lifecycle_state="refresh_failed_retained",
            status="valid",
            last_updated_step=state.step,
            last_updated_t=state.t,
            recovery_reason=failure_reason,
            source="cache_retained_after_failed_APS",
        )

    def mark_recovery_required(
        self,
        state: Any,
        record: AssignmentRecord,
        reason: str,
    ) -> AssignmentRecord:
        return self._replace(
            record,
            status="invalid",
            lifecycle_state="recovery_required",
            invalid_reason=reason,
            recovery_reason=reason,
            last_validated_step=state.step,
            last_validated_t=state.t,
            last_updated_step=state.step,
            last_updated_t=state.t,
            source="cmc_validation_failed",
        )

    def promote_to_merge_zone(self, state: Any, record: AssignmentRecord) -> AssignmentRecord:
        return self._replace(
            record,
            lifecycle_state="active_merge_zone",
            status="valid",
            last_updated_step=state.step,
            last_updated_t=state.t,
            source="cmc_promoted_to_merge_zone",
        )

    def complete(self, state: Any, record: AssignmentRecord) -> AssignmentRecord:
        return self._replace(
            record,
            lifecycle_state="completed",
            status="completed",
            last_updated_step=state.step,
            last_updated_t=state.t,
            source="merge_completed",
        )

    def derive_step5_view(
        self,
        state: Any,
        record: AssignmentRecord | Mapping[str, Any],
    ) -> AssignmentStepView | None:
        record = self._coerce(record)
        consumable = (
            record.lifecycle_state in STEP5_CONSUMABLE_STATES
            and record.status in {"valid", "available", "ok"}
            and record.gap_type == "bounded"
        )
        if not consumable:
            return None
        return self._view(record, source=record.source, step5=True)

    def derive_cmc_view(
        self,
        state: Any,
        record: AssignmentRecord | Mapping[str, Any],
    ) -> AssignmentStepView | None:
        record = self._coerce(record)
        consumable = (
            record.lifecycle_state in CMC_CONSUMABLE_STATES
            and record.status in {"valid", "available", "ok"}
        )
        if not consumable:
            return None
        return self._view(record, source=record.source, cmc=True)

    def is_control_zone_gap_protection_record(
        self,
        record: AssignmentRecord | Mapping[str, Any],
    ) -> bool:
        record = self._coerce(record)
        return (
            record.lifecycle_state in CONTROL_ZONE_GAP_PROTECTION_STATES
            and record.status in {"valid", "available", "ok"}
            and record.gap_type == "bounded"
        )

    def _coerce(self, value: AssignmentRecord | Mapping[str, Any]) -> AssignmentRecord:
        if isinstance(value, AssignmentRecord):
            return value
        return self.from_state_dict(value)

    def _replace(self, record: AssignmentRecord, **changes: Any) -> AssignmentRecord:
        payload = self.to_state_dict(record)
        payload.update(changes)
        return self.from_state_dict(payload)

    def _view(
        self,
        record: AssignmentRecord,
        *,
        source: str,
        step5: bool = False,
        cmc: bool = False,
    ) -> AssignmentStepView:
        return AssignmentStepView(
            mv_id=record.mv_id,
            record=MappingProxyType(self.to_state_dict(record)),
            source=source,
            consumable_by_step5=step5,
            consumable_by_cmc=cmc,
        )


assignment_lifecycle_manager = AssignmentLifecycleManager()


def assignment_record_to_state_dict(record: AssignmentRecord | Mapping[str, Any]) -> dict[str, Any]:
    return assignment_lifecycle_manager.to_state_dict(
        record if isinstance(record, AssignmentRecord) else assignment_lifecycle_manager.from_state_dict(record)
    )


def _default_lifecycle_state(value: Mapping[str, Any]) -> str:
    status = str(value.get("status") or "valid").lower()
    if status in {"invalid", "failed", "empty"}:
        return "invalid"
    return "active_control_zone"


def _optional_str(value: Any) -> str | None:
    if value in (None, ""):
        return None
    return str(value)


def _optional_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    return float(value)


def _optional_int(value: Any) -> int | None:
    if value in (None, ""):
        return None
    return int(value)


def _truthy_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "y"}
    return bool(value)
