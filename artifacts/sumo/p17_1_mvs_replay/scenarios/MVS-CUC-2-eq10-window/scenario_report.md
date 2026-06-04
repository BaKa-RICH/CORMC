# P17.1 Scenario Replay: MVS-CUC-2-eq10-window

- replay_id: `MVS-CUC-2-eq10-window`
- source_scenario_id: `MVS-CUC-2`
- extended: `true`
- required_suite_default_steps: `1`
- p17_1_replay_max_steps: `70`
- actual_steps: `70`
- t_range: `[0.0, 6.8999999999999915]`
- track_vehicle_id: `CFV_X`
- numeric_gate_status: `passed`
- replay_fidelity_status: `passed`
- gui_smoke_status: `ok`

## Key Vehicle Ranges

| vehicle_id | x_range | y_range | lane_change_states | merge_states |
| --- | --- | --- | --- | --- |
| `CFV_X` | `6846.489 -> 6930.857` | `0.000 -> 0.000` | `normal` | `none` |
| `CLV_Y` | `6852.040 -> 7037.378` | `0.000 -> 0.000` | `normal` | `none` |
| `MV_CUC` | `6852.000 -> 6990.000` | `-3.500 -> -2.291` | `normal` | `executing, not_started` |
| `TFV` | `6840.007 -> 7065.674` | `3.500 -> 3.500` | `normal` | `none` |
| `TLV` | `6922.232 -> 7111.902` | `3.500 -> 3.500` | `normal` | `none` |

## Key Event Hits

| check | status |
| --- | --- |
| `cfv_x_stays_lane2` | `passed` |
| `cfv_x_lane_change_state_normal` | `passed` |
| `eq10_spacing_override_consumed` | `passed` |
| `no_cfv_x_lateral_trajectory` | `passed` |

## Role And Color Legend

| role | color_rgb |
| --- | --- |
| `background` | `160,160,160` |
| `cfv_active_cooperative` | `245,140,0` |
| `clv` | `0,160,90` |
| `mv_on_ramp_active` | `0,90,220` |
| `tfv` | `0,170,180` |
| `tlv` | `120,80,220` |

## Lane Centerline Check

- status: `passed`

## Replay Fidelity Check

- status: `passed`
- checked_records: `350`

## Manual Command

```powershell
& "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-CUC-2-eq10-window\play_gui_replay.ps1"
```

## Smoke Verification

- status: `ok`
- sumo_gui_started: `True`
- replayed_steps: `70`
- closed_on_finish: `True`
- error: `None`

## Scope

- validates: CUC longitudinal intervention through Eq.10 spacing override while CFV_X stays in lane_2.
- does_not_validate: It is an Eq.10 short window, not a complete merge showcase. It is not SUMO-native traffic behavior and does not replace P17 true closed-loop validation.
