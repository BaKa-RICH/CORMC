# P17.1 Scenario Replay: MVS-COMMIT-1-full-extended

- replay_id: `MVS-COMMIT-1-full-extended`
- source_scenario_id: `MVS-COMMIT-1-full`
- extended: `true`
- required_suite_default_steps: `1`
- p17_1_replay_max_steps: `140`
- actual_steps: `140`
- t_range: `[2.0, 15.89999999999996]`
- track_vehicle_id: `MV_ACTIVE_MERGE`
- numeric_gate_status: `passed`
- replay_fidelity_status: `passed`
- gui_smoke_status: `ok`

## Key Vehicle Ranges

| vehicle_id | x_range | y_range | lane_change_states | merge_states |
| --- | --- | --- | --- | --- |
| `CFV_CACHE` | `6842.040 -> 7239.513` | `0.000 -> 0.000` | `normal` | `none` |
| `CFV_MERGE` | `6971.840 -> 7322.766` | `0.000 -> 0.000` | `normal` | `none` |
| `CLV_CACHE` | `6941.994 -> 7280.758` | `0.000 -> 0.000` | `normal` | `none` |
| `CLV_MERGE` | `7061.840 -> 7450.697` | `0.000 -> 0.000` | `normal` | `none` |
| `CV_ACTIVE_LC` | `6902.040 -> 7313.850` | `1.057 -> 3.500` | `executing, normal` | `none` |
| `MV_ACTIVE_MERGE` | `7011.800 -> 7364.930` | `-2.196 -> 0.000` | `normal` | `executing, merged` |
| `MV_CACHE` | `6892.000 -> 7199.011` | `-3.500 -> 0.000` | `normal` | `executing, merged, not_started` |
| `TFV_ACTIVE` | `6822.040 -> 7216.079` | `3.500 -> 3.500` | `normal` | `none` |
| `TLV_ACTIVE` | `6962.040 -> 7356.079` | `3.500 -> 3.500` | `normal` | `none` |

## Key Event Hits

| check | status |
| --- | --- |
| `cv_active_lc_y_to_lane1` | `passed` |
| `mv_active_merge_y_to_mainline` | `passed` |
| `mv_cache_y_to_mainline` | `passed` |
| `commit_sanity_all_pass` | `passed` |
| `unique_commit_per_vehicle_step` | `passed` |

## Role And Color Legend

| role | color_rgb |
| --- | --- |
| `active_cooperative_cv` | `245,140,0` |
| `background` | `160,160,160` |
| `cfv` | `245,140,0` |
| `clv` | `0,160,90` |
| `mv_on_ramp_active` | `0,90,220` |
| `tfv` | `0,170,180` |
| `tlv` | `120,80,220` |

## Lane Centerline Check

- status: `passed`

## Replay Fidelity Check

- status: `passed`
- checked_records: `1260`

## Manual Command

```powershell
& "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-COMMIT-1-full-extended\play_gui_replay.ps1"
```

## Smoke Verification

- status: `ok`
- sumo_gui_started: `True`
- replayed_steps: `140`
- closed_on_finish: `True`
- error: `None`

## Scope

- validates: Active trajectory continuation, non-APS cache behavior, and one commit per vehicle per step.
- does_not_validate: It does not add a new APS case 1/2/3/4 long end-to-end strong scenario. It is not SUMO-native traffic behavior and does not replace P17 true closed-loop validation.
