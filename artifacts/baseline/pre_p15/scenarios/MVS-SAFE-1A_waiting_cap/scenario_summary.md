# Scenario Summary: MVS-SAFE-1A_waiting_cap

- run_id: `pre_p15`
- status: `max_steps_reached`
- initial step/t: `0` / `0.0`
- final step/t: `1` / `0.1`
- final active vehicle count: `3`

## Final Vehicles

| vehicle_id | x_global | y | v | physical_lane | road_role | lane_change_state | merge_state |
|---|---:|---:|---:|---|---|---|---|
| CFV_SAFE_WAIT | 7201.6 | 0 | 16.005 | lane_2 | mainline | normal | none |
| CLV_SAFE_WAIT | 7246.6 | 0 | 16 | lane_2 | mainline | normal | none |
| MV_SAFE_WAIT | 7235.26 | -3.5 | 2.6295 | on_ramp | on_ramp_mv | normal | waiting |

## Active Maneuvers

| vehicle_id | maneuver_type | progress | last_planning_speed |
|---|---|---:|---:|
| none | none | 0 |  |

## Sanity Summary

- pass count: `41`
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

## PNG

- time_space.png: `time_space.png`
- registered PNG feature types: `assigned_clv_cfv_marker, boundary_cap_marker, cmc_decision_marker, commit_marker, cooperative_request_marker, lane_centerline_quicklook, longitudinal_candidate_marker, merging_zone_boundary_quicklook, planning_speed_marker, source_chain_marker, speed_cap_consumption_marker, trajectory_quicklook, waiting_marker`

## Evidence Files

- trajectory.csv: `trajectory.csv`
- events.jsonl: `events.jsonl`
- sanity.jsonl: `sanity.jsonl`
- state_snapshot.json: `state_snapshot.json`
- scenario_report.json: `scenario_report.json`
