# BASIC SUMO Replay Report: basic01_sumo_replay

- status: `passed`
- generated_at: `2026-06-05T11:50:59+00:00`
- git_commit: `5ef28c3b486d3ed152631dbb8c24225278f48788`
- sumo: `D:\software\Eclipse\Sumo\bin\sumo.exe`
- sumo_gui: `D:\software\Eclipse\Sumo\bin\sumo-gui.exe`
- sumo_version: `Eclipse SUMO sumo Version 1.22.0`

## Boundary

- This is SUMO-GUI replay of internal CORMC trajectory records.
- It is not SUMO-native closed-loop traffic behavior and does not replace P17 true closed-loop TraCI authority.
- Do not use a bare `.sumocfg` launch as the replay entrypoint; use the scripts below.

## Scenarios

| scenario_id | numeric | replay fidelity | gui smoke |
| --- | --- | --- | --- |
| `BASIC-01` | `passed` | `passed` | `not_run` |

## Manual Replay Commands

```powershell
& "D:\PycharmProjects\CORMC\artifacts\sumo\basic_replay\scenarios\BASIC-01\play_gui_replay.ps1"
```
