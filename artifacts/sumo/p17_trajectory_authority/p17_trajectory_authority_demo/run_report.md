# P17 SUMO Trajectory Authority Report: p17_trajectory_authority_demo

- scenario_id: P17-SUMO-CLOSED-LOOP
- status: ok
- sumo_version: Eclipse SUMO sumo Version 1.22.0
- sumo_home: D:\software\Eclipse\Sumo\
- executor_mode: move_to_xy_trajectory_authority
- speed_mode_bitset: 0
- lane_change_mode_bitset: 2560
- move_to_xy_keep_route: 3
- step_length: 0.1
- lateral_resolution: 0.25
- collision_action: warn
- seed: 16001
- profile_id: p16_internal_demo_v1
- max_steps: 60
- active_controlled_vehicle_count: 2
- background_vehicle_sample_count: 2
- active_controlled_vehicle_ids: MV_ACTIVE, p16_16001_on_ramp_0000
- background_vehicle_ids_sample: BG_0, BG_1
- generated_count: 1
- blocked_spawn_count: 0
- collision_count: 0
- teleport_count: 0
- realization_mismatch_count: 0

## Boundary

P17 does not do the P18 paper grid; P18 remains the later dual-track experiment route.

## GUI

Use the replay script below for human visualization. It opens sumo-gui through TraCI and replays the recorded trajectory-authority trace.

```powershell
& "artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\play_gui_replay.ps1"
```

Opening `artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\sumo\p17.sumocfg` directly now shows a 120-second static SUMO preview with preview vehicles. It is useful for visual inspection, but it is not the exact TraCI-controlled replay.
The exact replay script uses `artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\sumo\p17.traci.sumocfg` plus `realization.jsonl` to drive the active vehicles through TraCI.

## Outputs

- trajectory_csv: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\trajectory.csv
- events_jsonl: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\events.jsonl
- sanity_jsonl: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\sanity.jsonl
- realization_jsonl: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\realization.jsonl
- artifact_manifest_json: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\artifact_manifest.json
- scenario_report_json: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\scenario_report.json
- run_report_md: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\run_report.md
- gui_replay_script_ps1: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\play_gui_replay.ps1

## SUMO

- sumocfg_file: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\sumo\p17.sumocfg
- net_file: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\sumo\p17.net.xml
- route_file: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\sumo\p17.rou.xml
- nodes_file: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\sumo\p17.nod.xml
- edges_file: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\sumo\p17.edg.xml
- connections_file: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\sumo\p17.con.xml
- traci_sumocfg_file: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\sumo\p17.traci.sumocfg
- preview_route_file: artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\sumo\p17.preview.rou.xml
