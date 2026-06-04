param([switch]$Smoke)
$ErrorActionPreference = 'Stop'
$env:PYTHONIOENCODING = 'utf-8'
Set-Location -LiteralPath "D:\PycharmProjects\CORMC"
if ($Smoke) {
  & "D:\PycharmProjects\CORMC\.venv\Scripts\python.exe" -m cormc.sumo.mvs_gui_replay --sumocfg "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-CUC-1A-lanechange\sumo\p17.sumocfg" --replay "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-CUC-1A-lanechange\replay_trajectory.jsonl" --track-vehicle-id "CFV_X" --delay-ms 1 --hold-seconds 0 --post-roll-steps 0 --status-output "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-CUC-1A-lanechange\gui_smoke_status.json"
  exit $LASTEXITCODE
}
& "D:\PycharmProjects\CORMC\.venv\Scripts\python.exe" -m cormc.sumo.mvs_gui_replay --sumocfg "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-CUC-1A-lanechange\sumo\p17.sumocfg" --replay "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-CUC-1A-lanechange\replay_trajectory.jsonl" --track-vehicle-id "CFV_X" --delay-ms 150 --hold-seconds 0 --post-roll-steps 5 --keep-open-after-replay --status-output "D:\PycharmProjects\CORMC\artifacts\sumo\p17_1_mvs_replay\scenarios\MVS-CUC-1A-lanechange\gui_smoke_status.json"
exit $LASTEXITCODE
