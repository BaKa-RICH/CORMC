# Scenario Summary: MVS-SAFE-1B_executing_cap_lateral_consumption

- run_id: `pre_p15`
- status: `max_steps_reached`
- initial step/t: `0` / `0.0`
- final step/t: `1` / `0.1`
- final active vehicle count: `3`

## Final Vehicles

| vehicle_id | x_global | y | v | physical_lane | road_role | lane_change_state | merge_state |
|---|---:|---:|---:|---|---|---|---|
| CFV_SAFE_EXEC | 7191.62 | 0 | 16.18 | lane_2 | mainline | normal | none |
| CLV_SAFE_EXEC | 7271.6 | 0 | 16 | lane_2 | mainline | normal | none |
| MV_SAFE_EXEC | 7235.26 | -2.76689 | 2.6295 | on_ramp | on_ramp_mv | normal | executing |

## Active Maneuvers

| vehicle_id | maneuver_type | progress | last_planning_speed |
|---|---|---:|---:|
| MV_SAFE_EXEC | merge | 0.30263 | 2.6295 |

## Sanity Summary

- pass count: `40`
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
- registered PNG feature types: `active_maneuver_marker, boundary_cap_marker, commit_marker, cooperative_request_marker, executing_continuation_marker, lane_centerline_quicklook, longitudinal_candidate_marker, maneuver_progress_marker, merge_trajectory_marker, merging_zone_boundary_quicklook, planning_speed_consumption_marker, planning_speed_marker, source_chain_marker, speed_cap_consumption_marker, trajectory_quicklook`

## Evidence Files

- trajectory.csv: `trajectory.csv`
- events.jsonl: `events.jsonl`
- sanity.jsonl: `sanity.jsonl`
- state_snapshot.json: `state_snapshot.json`
- scenario_report.json: `scenario_report.json`
