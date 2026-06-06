# BASIC SUMO Replay: BASIC-02

- scenario_id: `BASIC-02`
- source_artifact_dir: `D:\PycharmProjects\CORMC\artifacts\basic\basic02_900_assignment_lifecycle_check\BASIC-02`
- status: `passed`
- actual_steps: `446` / max `900`
- numeric_gate_status: `passed`
- replay_fidelity_status: `passed`
- gui_smoke_status: `ok`
- expected APS case: `case_3`
- observed APS case: `case_3`
- expected active CVs: `B02_CLV`
- active CVs: `B02_CLV`
- expected Eq.10 consumers: `none`
- Eq.10 consumers: `none`
- merged past ramp: `True`
- refresh failed retained count: `267`
- cooperative request vehicles: `B02_CLV`
- CUC stay lane_2 vehicles: `B02_CLV`
- CMC recovery front-only: `True`
- CMC recovery leader: `B02_CFV`

## Replay Checks

- checked_records: `1784`
- step0_vehicle_count: `4`
- step0_mv_present: `True`
- step0_mv_x_global: `6642.04`
- step0_mv_y: `-3.5`
- pre_control_mv_record_count: `4`
- pre_control_hint_count: `0`
- final_mv_merge_state: `merged`
- final_mv_physical_lane: `lane_2`

## Key Vehicle Ranges

| vehicle_id | x_range | y_range | lane_change_states | merge_states |
| --- | --- | --- | --- | --- |
| `B02_CFV` | `6616.007 -> 7925.605` | `0.000 -> 0.000` | `normal` | `none` |
| `B02_CLV` | `6656.040 -> 7968.000` | `0.000 -> 0.000` | `normal` | `none` |
| `B02_MV` | `6642.040 -> 7250.088` | `-3.500 -> 0.000` | `normal` | `executing, merged, not_started` |
| `B02_TLV_CLV` | `6664.540 -> 7961.620` | `3.500 -> 3.500` | `normal` | `none` |

## Role And Color Legend

| role | color_rgb |
| --- | --- |
| `background` | `160,160,160` |
| `cfv` | `245,140,0` |
| `clv_active_cooperative` | `160,160,160` |
| `mv_on_ramp_active` | `0,90,220` |
| `tlv` | `120,80,220` |

## Boundary

- pre-control segment 6450 -> 6650 is visible through the ramp_upstream SUMO edge; records below 6450 remain numeric-only and use a replay hint fallback.
- Internal trajectory replay; it does not replace P17 true closed-loop TraCI authority.
- Use the replay script below; opening the `.sumocfg` directly will not play the trajectory JSONL.

## Manual Command

```powershell
& "D:\PycharmProjects\CORMC\artifacts\sumo\basic_replay\scenarios\BASIC-02\play_gui_replay.ps1"
```

## Smoke Verification

- status: `ok`
- sumo_gui_started: `True`
- replayed_steps: `446`
- replayed_vehicle_ids: `('B02_CFV', 'B02_CLV', 'B02_MV', 'B02_TLV_CLV')`
- closed_on_finish: `True`
- error: `None`
