# Scenario Summary: MVS-CUC-1A_override_choice1

- run_id: `pre_p15`
- status: `max_steps_reached`
- initial step/t: `0` / `0.0`
- final step/t: `1` / `0.1`
- final active vehicle count: `5`

## Final Vehicles

| vehicle_id | x_global | y | v | physical_lane | road_role | lane_change_state | merge_state |
|---|---:|---:|---:|---|---|---|---|
| CFV_X | 6846.01 | 0.00347838 | 20.0727 | lane_2 | mainline | executing | none |
| CLV_Y | 6886.04 | 0 | 20.4 | lane_2 | mainline | normal | none |
| MV_CUC | 6852 | -3.5 | 20 | on_ramp | on_ramp_mv | normal | not_started |
| TFV | 6752.04 | 3.5 | 20.4 | lane_1 | mainline | normal | none |
| TLV | 6922.23 | 3.5 | 22.32 | lane_1 | mainline | normal | none |

## Active Maneuvers

| vehicle_id | maneuver_type | progress | last_planning_speed |
|---|---|---:|---:|
| CFV_X | lane_change | 0.0200727 | 20.0727 |

## Sanity Summary

- pass count: `37`
- fail count: `0`
- failed check ids: `none`

## Key Event Existence

- APS: `True`
- CMC: `True`
- CUC: `True`
- longitudinal_model: `True`
- lateral_trajectory: `True`
- commit: `True`
- time_advance: `True`

## Formula Status

- cuc_eq11_eq16: `locked_formula`
- cav_eq17_eq27: `locked_formula`
- chv_eq28_eq29: `not_observed`
- front_collision_eq42_eq46: `locked_formula`
- legacy proxy markers present: `none`

## PNG

- time_space.png: `time_space.png`
- registered PNG feature types: `active_maneuver_marker, commit_marker, cooperative_request_marker, cuc_decision_marker, lane_centerline_quicklook, lane_change_intent_marker, lane_change_trajectory_marker, longitudinal_candidate_marker, maneuver_progress_marker, merging_zone_boundary_quicklook, planning_speed_consumption_marker, planning_speed_marker, same_step_overlay_marker, source_chain_marker, trajectory_quicklook`

## Evidence Files

- trajectory.csv: `trajectory.csv`
- events.jsonl: `events.jsonl`
- sanity.jsonl: `sanity.jsonl`
- state_snapshot.json: `state_snapshot.json`
- scenario_report.json: `scenario_report.json`
