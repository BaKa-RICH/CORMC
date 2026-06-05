# BASIC SUMO Replay: BASIC-01

- scenario_id: `BASIC-01`
- status: `passed`
- actual_steps: `215` / max `900`
- numeric_gate_status: `passed`
- replay_fidelity_status: `passed`
- gui_smoke_status: `not_run`
- expected APS case: `case_2`
- observed APS case: `case_2`
- expected active CVs: `B01_CFV`
- active CVs: `B01_CFV`
- expected Eq.10 consumers: `B01_CFV`
- Eq.10 consumers: `B01_CFV`
- merged past ramp: `True`

## Replay Checks

- checked_records: `860`
- step0_vehicle_count: `4`
- step0_mv_present: `True`
- step0_mv_x_global: `6642.04`
- step0_mv_y: `-3.5`
- pre_control_mv_record_count: `4`
- pre_control_hint_count: `4`
- final_mv_merge_state: `merged`
- final_mv_physical_lane: `lane_2`

## Key Vehicle Ranges

| vehicle_id | x_range | y_range | lane_change_states | merge_states |
| --- | --- | --- | --- | --- |
| `B01_CFV` | `6636.007 -> 7205.375` | `0.000 -> 0.000` | `normal` | `none` |
| `B01_CLV` | `6676.040 -> 7295.004` | `0.000 -> 0.000` | `normal` | `none` |
| `B01_MV` | `6642.040 -> 7250.585` | `-3.500 -> 0.000` | `normal` | `executing, merged, not_started` |
| `B01_TLV_CFV` | `6644.540 -> 7248.626` | `3.500 -> 3.500` | `normal` | `none` |

## Role And Color Legend

| role | color_rgb |
| --- | --- |
| `background` | `160,160,160` |
| `cfv_active_cooperative` | `245,140,0` |
| `clv` | `0,160,90` |
| `mv_on_ramp_active` | `0,90,220` |
| `tlv` | `120,80,220` |

## Boundary

- pre-control segment 6450 -> 6650 is numeric-simulation-only in the current P17 map. SUMO replay uses a visual edge/lane hint for those records and becomes visually authoritative after the MV enters ramp_pre at x >= 6650.
- Internal trajectory replay; it does not replace P17 true closed-loop TraCI authority.
- Use the replay script below; opening the `.sumocfg` directly will not play the trajectory JSONL.

## Manual Command

```powershell
& "D:\PycharmProjects\CORMC\artifacts\sumo\basic_replay\scenarios\BASIC-01\play_gui_replay.ps1"
```

## Smoke Verification

- status: `not_run`
- sumo_gui_started: `False`
- replayed_steps: `0`
- replayed_vehicle_ids: `[]`
- closed_on_finish: `False`
- error: `None`
