# P17.1 MVS Replay Report: p17_1_mvs_replay

- status: `passed`
- generated_at: `2026-06-04T08:58:52+00:00`
- git_commit: `b135f3a8209c27579b07e8475540339cfc05ed30`
- sumo: `D:\software\Eclipse\Sumo\bin\sumo.exe`
- sumo_gui: `D:\software\Eclipse\Sumo\bin\sumo-gui.exe`

## Boundary

- P14/P15 are algorithm numeric evidence.
- P16 is seeded random internal simulation evidence.
- P17.1 is replay evidence that internal algorithm trajectories can be seen in SUMO.
- P17 true closed-loop TraCI trajectory-authority code is not replaced by P17.1 replay code.
- `MVS-E2E-1-extended` has no mainline vehicle intervention.
- `MVS-CUC-2-eq10-window` is an Eq.10 short window, not a complete merge showcase.
- `MVS-SAFE-1B-cap` is a boundary-cap showcase, not a merge-complete showcase.
- Do not use a bare `.sumocfg` launch as the replay entrypoint; use the scripts below.

## Scenarios

| replay_id | source | numeric | replay fidelity | gui smoke |
| --- | --- | --- | --- | --- |
| `MVS-E2E-1-extended` | `MVS-E2E-1` | `passed` | `passed` | `ok` |
| `MVS-CMC-1-extended` | `MVS-CMC-1` | `passed` | `passed` | `ok` |
| `MVS-CUC-1A-lanechange` | `MVS-CUC-1A_override_choice1` | `passed` | `passed` | `ok` |
| `MVS-CUC-2-eq10-window` | `MVS-CUC-2` | `passed` | `passed` | `ok` |
| `MVS-SAFE-1B-cap` | `MVS-SAFE-1B_executing_cap_lateral_consumption` | `passed` | `passed` | `ok` |
| `MVS-COMMIT-1-full-extended` | `MVS-COMMIT-1-full` | `passed` | `passed` | `ok` |

## Manual Replay Commands

```powershell
& "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-E2E-1-extended\play_gui_replay.ps1"
& "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-CMC-1-extended\play_gui_replay.ps1"
& "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-CUC-1A-lanechange\play_gui_replay.ps1"
& "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-CUC-2-eq10-window\play_gui_replay.ps1"
& "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-SAFE-1B-cap\play_gui_replay.ps1"
& "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-COMMIT-1-full-extended\play_gui_replay.ps1"
```
