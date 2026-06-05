# BASIC-05 BASIC Numeric Diagnostic

- status: `diagnosed_unresolved`
- actual_steps: `900` / max `900`
- expected APS case: `case_3`
- observed APS case: `case_3`
- active CVs: `B05_CLV`
- Eq.10 consumers: `none`
- merged past ramp: `False`

## Initial Vehicles

| vehicle_id | lane | road_role | x_global | y | v | type |
| --- | --- | --- | ---: | ---: | ---: | --- |
| B05_MV | on_ramp | on_ramp_mv | 6850.0 | -3.5 | 20.0 | cav |
| B05_CLV | lane_2 | mainline | 6864.0 | 0.0 | 20.0 | cav |
| B05_CFV | lane_2 | mainline | 6824.0 | 0.0 | 20.0 | cav |
| B05_TLV_CLV | lane_1 | mainline | 6873.0 | 3.5 | 15.0 | cav |

## Region

- first control-zone step: `0` at x `6850.0`
- first merge-zone step: `40` at x `6950.688787637328`
- illegal pre-control events: `0`

## Timelines

- first APS: `step=0, reason=first_aps, aps_case=case_3, clv_id=B05_CLV, cfv_id=B05_CFV`
- CUC choices: `0:stay_lane_2; 1:stay_lane_2; 2:stay_lane_2; 3:stay_lane_2; 4:stay_lane_2; 5:stay_lane_2; 6:stay_lane_2; 7:stay_lane_2; 8:stay_lane_2; 9:stay_lane_2; 10:stay_lane_2; 11:stay_lane_2; 12:stay_lane_2; 13:stay_lane_2; 14:stay_lane_2; 15:stay_lane_2; 16:stay_lane_2; 17:stay_lane_2; 18:stay_lane_2; 19:stay_lane_2; 20:stay_lane_2; 21:stay_lane_2; 22:stay_lane_2; 23:stay_lane_2; 24:stay_lane_2; 25:stay_lane_2; 26:stay_lane_2; 27:stay_lane_2; 28:stay_lane_2; 29:stay_lane_2; 30:stay_lane_2; 31:stay_lane_2; 32:stay_lane_2; 33:stay_lane_2; 34:stay_lane_2; 35:stay_lane_2; 36:stay_lane_2; 37:stay_lane_2; 38:stay_lane_2; 39:stay_lane_2`
- assignment validity: `40:True; 41:True; 42:True; 43:True; 44:True; 45:True; 46:True; 47:True; 48:True; 49:True; 50:True; 51:True; 52:True; 53:True; 54:True; 55:True; 56:True; 57:True; 58:True; 59:True; 60:True; 61:True; 62:True; 63:True; 64:True; 65:True; 66:True; 67:True; 68:True; 69:True; 70:True; 71:True; 72:True; 73:True; 74:True; 75:True; 76:True; 77:True; 78:True; 79:True; 80:True; 81:True; 82:True; 83:True; 84:True; 85:True; 86:True; 87:True; 88:True; 89:True; 90:True; 91:True; 92:True; 93:True; 94:True; 95:True; 96:True; 97:True; 98:True; 99:True; 100:True; 101:True; 102:True; 103:True; 104:True; 105:True; 106:True; 107:True; 108:True; 109:True; 110:True; 111:True`
- Eq.53: `40:False; 41:False; 42:False; 43:False; 44:False; 45:False; 46:False; 47:False; 48:False; 49:False; 50:False; 51:False; 52:False; 53:False; 54:False; 55:False; 56:False; 57:False; 58:False; 59:False; 60:False; 61:False; 62:False; 63:False; 64:False; 65:False; 66:False; 67:False; 68:False; 69:False; 70:False; 71:False; 72:False; 73:False; 74:False; 75:False; 76:False; 77:False; 78:False; 79:False; 80:False; 81:False; 82:False; 83:False; 84:False; 85:False; 86:False; 87:False; 88:False; 89:False; 90:False; 91:False; 92:False; 93:False; 94:False; 95:False; 96:False; 97:False; 98:False; 99:False; 100:False; 101:False; 102:False; 103:False; 104:False; 105:False; 106:False; 107:False; 108:False; 109:False; 110:False; 111:True`
- merge states: `0:not_started; 112:executing`

## Issues

### Resolved

- none

### Unresolved

- `BASIC-05:outcome:not_merged_past_ramp` [cmc_issue]: MV did not finish as merged past x_ramp_end_global during this run.

## Artifacts

- trajectory: `artifacts\basic\basic_long_dt_900\scenarios\BASIC-05\trajectory.csv`
- events: `artifacts\basic\basic_long_dt_900\scenarios\BASIC-05\events.jsonl`
- sanity: `artifacts\basic\basic_long_dt_900\scenarios\BASIC-05\sanity.jsonl`
- numeric_summary: `artifacts\basic\basic_long_dt_900\scenarios\BASIC-05\numeric_summary.json`
