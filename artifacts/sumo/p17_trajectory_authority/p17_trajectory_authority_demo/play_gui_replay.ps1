$ErrorActionPreference = 'Stop'
$env:PYTHONIOENCODING = 'utf-8'
Set-Location -LiteralPath "D:\PycharmProjects\CORMC"
python -m cormc.sumo.gui_replay --sumocfg "D:\PycharmProjects\CORMC\artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\sumo\p17.traci.sumocfg" --realization "D:\PycharmProjects\CORMC\artifacts\sumo\p17_trajectory_authority\p17_trajectory_authority_demo\realization.jsonl" --delay-ms 150 --hold-seconds 0 --keep-open-after-replay
