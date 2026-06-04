# P17.1 Scenario Replay: MVS-E2E-1-extended

- replay_id: `MVS-E2E-1-extended`
- source_scenario_id: `MVS-E2E-1`
- extended: `true`
- required_suite_default_steps: `70`
- p17_1_replay_max_steps: `140`
- actual_steps: `140`
- t_range: `[0.0, 13.899999999999967]`
- track_vehicle_id: `MV_DEMO`
- numeric_gate_status: `passed`
- replay_fidelity_status: `passed`
- gui_smoke_status: `ok`

## Key Vehicle Ranges

| vehicle_id | x_range | y_range | lane_change_states | merge_states |
| --- | --- | --- | --- | --- |
| `BG_LANE1_DEMO` | `6842.040 -> 7236.079` | `3.500 -> 3.500` | `normal` | `none` |
| `CFV_DEMO` | `6801.995 -> 7065.614` | `0.000 -> 0.000` | `normal` | `none` |
| `CLV_DEMO` | `6872.000 -> 7150.000` | `0.000 -> 0.000` | `normal` | `none` |
| `MV_DEMO` | `6832.000 -> 7116.416` | `-3.500 -> 0.000` | `normal` | `executing, merged, not_started` |

## Key Event Hits

| check | status |
| --- | --- |
| `mv_demo_y_from_ramp_to_mainline` | `passed` |
| `eq53_pass` | `passed` |
| `merge_start` | `passed` |
| `lateral_trajectory_completed` | `passed` |
| `no_cooperative_request` | `passed` |

## Role And Color Legend

| role | color_rgb |
| --- | --- |
| `background` | `160,160,160` |
| `cfv` | `245,140,0` |
| `clv` | `0,160,90` |
| `mv_on_ramp_active` | `0,90,220` |
| `support` | `120,80,220` |

## Lane Centerline Check

- status: `passed`

## Replay Fidelity Check

- status: `passed`
- checked_records: `560`

## Manual Command

```powershell
& "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-E2E-1-extended\play_gui_replay.ps1"
```

## Smoke Verification

- status: `ok`
- sumo_gui_started: `True`
- replayed_steps: `140`
- closed_on_finish: `True`
- error: `None`

## Scope

- validates: APS case 1, no CUC request, CMC Eq.53 pass, merge start, merge completion, and commit replay.
- does_not_validate: It has no mainline cooperative intervention. It is not SUMO-native traffic behavior and does not replace P17 true closed-loop validation.
