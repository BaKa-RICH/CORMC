# P17.1 Scenario Replay: MVS-CMC-1-extended

- replay_id: `MVS-CMC-1-extended`
- source_scenario_id: `MVS-CMC-1`
- extended: `true`
- required_suite_default_steps: `None`
- p17_1_replay_max_steps: `90`
- actual_steps: `90`
- t_range: `[0.0, 8.899999999999984]`
- track_vehicle_id: `MV_CMC_1`
- numeric_gate_status: `passed`
- replay_fidelity_status: `passed`
- gui_smoke_status: `ok`

## Key Vehicle Ranges

| vehicle_id | x_range | y_range | lane_change_states | merge_states |
| --- | --- | --- | --- | --- |
| `CFV_CMC_1` | `6972.040 -> 7199.470` | `0.000 -> 0.000` | `normal` | `none` |
| `CLV_CMC_1` | `7032.040 -> 7276.609` | `0.000 -> 0.000` | `normal` | `none` |
| `MV_CMC_1` | `7002.040 -> 7235.323` | `-3.496 -> 0.000` | `normal` | `executing, merged` |

## Key Event Hits

| check | status |
| --- | --- |
| `step0_eq53_pass` | `passed` |
| `step0_merge_start` | `passed` |
| `mv_cmc_1_y_from_ramp_to_mainline` | `passed` |

## Role And Color Legend

| role | color_rgb |
| --- | --- |
| `background` | `160,160,160` |
| `cfv` | `245,140,0` |
| `clv` | `0,160,90` |
| `mv_on_ramp_active` | `0,90,220` |

## Lane Centerline Check

- status: `passed`

## Replay Fidelity Check

- status: `passed`
- checked_records: `270`

## Manual Command

```powershell
& "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-CMC-1-extended\play_gui_replay.ps1"
```

## Smoke Verification

- status: `ok`
- sumo_gui_started: `True`
- replayed_steps: `90`
- closed_on_finish: `True`
- error: `None`

## Scope

- validates: CMC Eq.53 pass and immediate merge start/completion from an existing helper gate.
- does_not_validate: It does not promote MVS-CMC-1 into the deterministic required route matrix. It is not SUMO-native traffic behavior and does not replace P17 true closed-loop validation.
