# P17.1 Scenario Replay: MVS-SAFE-1B-cap

- replay_id: `MVS-SAFE-1B-cap`
- source_scenario_id: `MVS-SAFE-1B_executing_cap_lateral_consumption`
- extended: `true`
- required_suite_default_steps: `1`
- p17_1_replay_max_steps: `320`
- actual_steps: `320`
- t_range: `[0.0, 31.900000000000183]`
- track_vehicle_id: `MV_SAFE_EXEC`
- numeric_gate_status: `passed`
- replay_fidelity_status: `passed`
- gui_smoke_status: `ok`

## Key Vehicle Ranges

| vehicle_id | x_range | y_range | lane_change_states | merge_states |
| --- | --- | --- | --- | --- |
| `CFV_SAFE_EXEC` | `7191.640 -> 8114.200` | `0.000 -> 0.000` | `normal` | `none` |
| `CLV_SAFE_EXEC` | `7271.640 -> 8194.200` | `0.000 -> 0.000` | `normal` | `none` |
| `MV_SAFE_EXEC` | `7235.263 -> 7245.995` | `-2.767 -> -2.238` | `normal` | `executing` |

## Key Event Hits

| check | status |
| --- | --- |
| `mv_safe_exec_remains_executing` | `passed` |
| `speed_cap_binding` | `passed` |
| `lateral_trajectory_consumes_speed_cap` | `passed` |
| `no_merge_complete` | `passed` |

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
- checked_records: `960`

## Manual Command

```powershell
& "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-SAFE-1B-cap\play_gui_replay.ps1"
```

## Smoke Verification

- status: `ok`
- sumo_gui_started: `True`
- replayed_steps: `320`
- closed_on_finish: `True`
- error: `None`

## Scope

- validates: Boundary speed cap consumption by an executing lateral trajectory.
- does_not_validate: It is a boundary-cap showcase, not a merge-complete showcase. It is not SUMO-native traffic behavior and does not replace P17 true closed-loop validation.
