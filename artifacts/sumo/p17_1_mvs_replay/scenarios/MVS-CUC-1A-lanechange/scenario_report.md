# P17.1 Scenario Replay: MVS-CUC-1A-lanechange

- replay_id: `MVS-CUC-1A-lanechange`
- source_scenario_id: `MVS-CUC-1A_override_choice1`
- extended: `true`
- required_suite_default_steps: `1`
- p17_1_replay_max_steps: `50`
- actual_steps: `50`
- t_range: `[0.0, 4.899999999999999]`
- track_vehicle_id: `CFV_X`
- numeric_gate_status: `passed`
- replay_fidelity_status: `passed`
- gui_smoke_status: `ok`

## Key Vehicle Ranges

| vehicle_id | x_range | y_range | lane_change_states | merge_states |
| --- | --- | --- | --- | --- |
| `CFV_X` | `6846.007 -> 6973.834` | `0.003 -> 3.500` | `executing, normal` | `none` |
| `CLV_Y` | `6886.040 -> 7013.117` | `0.000 -> 0.000` | `normal` | `none` |
| `MV_CUC` | `6852.000 -> 6950.000` | `-3.500 -> -3.500` | `normal` | `not_started` |
| `TFV` | `6752.040 -> 6879.117` | `3.500 -> 3.500` | `normal` | `none` |
| `TLV` | `6922.232 -> 7053.294` | `3.500 -> 3.500` | `normal` | `none` |

## Key Event Hits

| check | status |
| --- | --- |
| `cfv_x_y_from_lane2_to_lane1` | `passed` |
| `final_choice_change_to_lane_1` | `passed` |
| `lane_change_command_created` | `passed` |
| `same_step_overlay` | `passed` |
| `lateral_trajectory_completed` | `passed` |

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
- checked_records: `250`

## Manual Command

```powershell
& "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-CUC-1A-lanechange\play_gui_replay.ps1"
```

## Smoke Verification

- status: `ok`
- sumo_gui_started: `True`
- replayed_steps: `50`
- closed_on_finish: `True`
- error: `None`

## Scope

- validates: CUC choice 1, lane-change command, same-step overlay, and mainline lateral intervention.
- does_not_validate: It does not show a full CUC-to-CMC merge completion chain. It is not SUMO-native traffic behavior and does not replace P17 true closed-loop validation.
