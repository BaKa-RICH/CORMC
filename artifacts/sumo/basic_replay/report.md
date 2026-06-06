# BASIC SUMO Replay Report: basic01_sumo_replay

- status: `passed`
- generated_at: `2026-06-06T07:42:53+00:00`
- git_commit: `82162ffe7a53d25270ba44f6394b2895afad27cb`
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
| `BASIC-02` | `passed` | `passed` | `ok` |

## Manual Replay Commands

```powershell
& "D:\PycharmProjects\CORMC\artifacts\sumo\basic_replay\scenarios\BASIC-02\play_gui_replay.ps1"
```
