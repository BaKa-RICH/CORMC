# Scenario Summary: MVS-COMMIT-1-full

- run_id: `pre_p15`
- status: `max_steps_reached`
- initial step/t: `20` / `2.0`
- final step/t: `21` / `2.1`
- final active vehicle count: `9`

## Final Vehicles

| vehicle_id | x_global | y | v | physical_lane | road_role | lane_change_state | merge_state |
|---|---:|---:|---:|---|---|---|---|
| CFV_CACHE | 6842.01 | 0 | 20.08 | lane_2 | mainline | normal | none |
| CFV_MERGE | 6971.8 | 0 | 18 | lane_2 | mainline | normal | none |
| CLV_CACHE | 6941.99 | 0 | 19.88 | lane_2 | mainline | normal | none |
| CLV_MERGE | 7061.8 | 0 | 18 | lane_2 | mainline | normal | none |
| CV_ACTIVE_LC | 6902.01 | 1.05539 | 20.08 | lane_2 | mainline | executing | none |
| MV_ACTIVE_MERGE | 7011.8 | -2.19585 | 18 | on_ramp | on_ramp_mv | normal | executing |
| MV_CACHE | 6892 | -3.5 | 20 | on_ramp | on_ramp_mv | normal | not_started |
| TFV_ACTIVE | 6822 | 3.5 | 20 | lane_1 | mainline | normal | none |
| TLV_ACTIVE | 6962 | 3.5 | 20 | lane_1 | mainline | normal | none |

## Active Maneuvers

| vehicle_id | maneuver_type | progress | last_planning_speed |
|---|---|---:|---:|
| CV_ACTIVE_LC | lane_change | 0.37008 | 20.08 |
| MV_ACTIVE_MERGE | merge | 0.418 | 18 |

## Sanity Summary

- pass count: `41`
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

## PNG

- time_space.png: `time_space.png`
- registered PNG feature types: `active_maneuver_marker, boundary_cap_marker, commit_marker, cooperative_request_marker, executing_continuation_marker, lane_centerline_quicklook, lane_change_trajectory_marker, longitudinal_candidate_marker, maneuver_progress_marker, merge_trajectory_marker, merging_zone_boundary_quicklook, planning_speed_consumption_marker, planning_speed_marker, source_chain_marker, speed_cap_consumption_marker, trajectory_quicklook`

## Evidence Files

- trajectory.csv: `trajectory.csv`
- events.jsonl: `events.jsonl`
- sanity.jsonl: `sanity.jsonl`
- state_snapshot.json: `state_snapshot.json`
- scenario_report.json: `scenario_report.json`
