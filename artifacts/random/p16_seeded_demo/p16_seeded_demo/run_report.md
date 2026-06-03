# P16 Seeded Random Internal Simulation

- run_id: `p16_seeded_demo`
- scenario_id: `P16-RANDOM-DEMO-internal`
- random_enabled: `true`
- seed: `16001`
- profile_id: `p16_internal_demo_v1`
- status: `max_steps_reached`
- final step/t: `100` / `10`
- generated vehicle count: `13`
- blocked spawn count: `0`

## Observed Events

- event types: `APS, APS_candidate, CMC, CUC, assignment_cache, assignment_invalid, boundary_generation, cleanup, commit, freeze, geometry, information_integration, longitudinal_model, relation_refresh, speed_cap, time_advance`

## Evidence Files

- trajectory.csv: `trajectory.csv`
- events.jsonl: `events.jsonl`
- sanity.jsonl: `sanity.jsonl`
- time_space.png: `time_space.png`
- artifact_manifest.json: `artifact_manifest.json`
- scenario_report.json: `scenario_report.json`

## Boundary

Random vehicle generation is applied only through Step 1 pre-freeze boundary decisions. The algorithm steps consume frozen `SimulationState` and ordinary `VehicleSpec` data.
