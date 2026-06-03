# Scenario Summary: MVS-CUC-2

- run_id: `post_p15`
- status: `max_steps_reached`
- initial step/t: `0` / `0.0`
- final step/t: `1` / `0.1`
- final active vehicle count: `5`

## Final Vehicles

| vehicle_id | x_global | y | v | physical_lane | road_role | lane_change_state | merge_state |
|---|---:|---:|---:|---|---|---|---|
| CFV_X | 6846.49 | 0 | 24.8909 | lane_2 | mainline | normal | none |
| CLV_Y | 6852.04 | 0 | 20.4 | lane_2 | mainline | normal | none |
| MV_CUC | 6852 | -3.5 | 20 | on_ramp | on_ramp_mv | normal | not_started |
| TFV | 6840.01 | 3.5 | 30.0727 | lane_1 | mainline | normal | none |
| TLV | 6922.23 | 3.5 | 22.32 | lane_1 | mainline | normal | none |

## Active Maneuvers

| vehicle_id | maneuver_type | progress | last_planning_speed |
|---|---|---:|---:|
| none | none | 0 |  |

## Sanity Summary

- pass count: `38`
- fail count: `0`
- failed check ids: `none`

## Key Event Existence

- APS: `True`
- CMC: `True`
- CUC: `True`
- longitudinal_model: `True`
- lateral_trajectory: `False`
- commit: `True`
- time_advance: `True`

## Formula Status

- cuc_eq11_eq16: `locked_formula`
- cav_eq17_eq27: `locked_formula`
- chv_eq28_eq29: `not_observed`
- front_collision_eq42_eq46: `not_observed`
- legacy proxy markers present: `none`

## PNG

- time_space.png: `time_space.png`
- registered PNG feature types: `commit_marker, cooperative_request_marker, cuc_decision_marker, eq10_spacing_consumption_marker, lane_centerline_quicklook, longitudinal_candidate_marker, merging_zone_boundary_quicklook, planning_speed_marker, source_chain_marker, spacing_override_marker, target_lane_unsafe_fallback_marker, trajectory_quicklook`

## Evidence Files

- trajectory.csv: `trajectory.csv`
- events.jsonl: `events.jsonl`
- sanity.jsonl: `sanity.jsonl`
- state_snapshot.json: `state_snapshot.json`
- scenario_report.json: `scenario_report.json`
