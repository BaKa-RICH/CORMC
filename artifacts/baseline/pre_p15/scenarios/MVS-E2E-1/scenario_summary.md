# Scenario Summary: MVS-E2E-1

- run_id: `pre_p15`
- status: `max_steps_reached`
- initial step/t: `0` / `0.0`
- final step/t: `70` / `6.999999999999991`
- final active vehicle count: `4`

## Final Vehicles

| vehicle_id | x_global | y | v | physical_lane | road_role | lane_change_state | merge_state |
|---|---:|---:|---:|---|---|---|---|
| BG_LANE1_DEMO | 6980 | 3.5 | 20 | lane_1 | mainline | normal | none |
| CFV_DEMO | 6935.25 | 0 | 19.1271 | lane_2 | mainline | normal | none |
| CLV_DEMO | 7010 | 0 | 20 | lane_2 | mainline | normal | none |
| MV_DEMO | 6970 | -3.16578 | 20 | on_ramp | on_ramp_mv | normal | executing |

## Active Maneuvers

| vehicle_id | maneuver_type | progress | last_planning_speed |
|---|---|---:|---:|
| MV_DEMO | merge | 0.2 | 20 |

## Sanity Summary

- pass count: `2621`
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
- registered PNG feature types: `active_maneuver_marker, assigned_clv_cfv_marker, boundary_cap_marker, cmc_decision_marker, commit_marker, cooperative_request_marker, executing_continuation_marker, lane_centerline_quicklook, longitudinal_candidate_marker, maneuver_progress_marker, merge_start_marker, merge_trajectory_marker, merging_zone_boundary_quicklook, planning_speed_consumption_marker, planning_speed_marker, source_chain_marker, speed_cap_consumption_marker, trajectory_quicklook`

## Evidence Files

- trajectory.csv: `trajectory.csv`
- events.jsonl: `events.jsonl`
- sanity.jsonl: `sanity.jsonl`
- state_snapshot.json: `state_snapshot.json`
- scenario_report.json: `scenario_report.json`
