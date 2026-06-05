# BASIC-01 BASIC Numeric Diagnostic

- status: `passed`
- actual_steps: `215` / max `900`
- expected APS case: `case_2`
- observed APS case: `case_2`
- active CVs: `B01_CFV`
- Eq.10 consumers: `B01_CFV`
- merged past ramp: `True`

## Initial Vehicles

| vehicle_id | lane | road_role | x_global | y | v | type |
| --- | --- | --- | ---: | ---: | ---: | --- |
| B01_MV | on_ramp | on_ramp_mv | 6640.0 | -3.5 | 20.0 | cav |
| B01_CLV | lane_2 | mainline | 6674.0 | 0.0 | 20.0 | cav |
| B01_CFV | lane_2 | mainline | 6634.0 | 0.0 | 20.0 | cav |
| B01_TLV_CFV | lane_1 | mainline | 6643.0 | 3.5 | 15.0 | cav |

## Region

- first control-zone step: `5` at x `6650.5689447424`
- first merge-zone step: `120` at x `6951.481434719551`
- illegal pre-control events: `0`

## Timelines

- first APS: `step=5, reason=first_aps, aps_case=case_2, clv_id=B01_CLV, cfv_id=B01_CFV`
- APS excluded candidates: `none`
- first cached boundary invalidation: `none`
- CUC choices: `5:stay_lane_2; 6:stay_lane_2; 7:stay_lane_2; 8:stay_lane_2; 9:stay_lane_2; 10:stay_lane_2; 11:stay_lane_2; 12:stay_lane_2; 13:stay_lane_2; 14:stay_lane_2; 15:stay_lane_2; 16:stay_lane_2; 17:stay_lane_2; 18:stay_lane_2; 19:stay_lane_2; 20:stay_lane_2; 21:stay_lane_2; 22:stay_lane_2; 23:stay_lane_2; 24:stay_lane_2; 25:stay_lane_2; 26:stay_lane_2; 27:stay_lane_2; 28:stay_lane_2; 29:stay_lane_2; 30:stay_lane_2; 31:stay_lane_2; 32:stay_lane_2; 33:stay_lane_2; 34:stay_lane_2; 35:stay_lane_2; 36:stay_lane_2; 37:stay_lane_2; 38:stay_lane_2; 39:stay_lane_2; 40:stay_lane_2; 41:stay_lane_2; 42:stay_lane_2; 43:stay_lane_2; 44:stay_lane_2; 45:stay_lane_2; 46:stay_lane_2; 47:stay_lane_2; 48:stay_lane_2; 49:stay_lane_2; 50:stay_lane_2; 51:stay_lane_2; 52:stay_lane_2; 53:stay_lane_2; 54:stay_lane_2; 55:stay_lane_2`
- assignment validity: `120:True; 121:True; 122:True; 123:True; 124:True`
- assignment invalid reasons: `120:None; 121:None; 122:None; 123:None; 124:None`
- Eq.53: `120:False; 121:False; 122:False; 123:False; 124:True`
- merge states: `0:not_started; 125:executing; 155:merged`

## Issues

### Resolved

- `BASIC-01:region:pre_control_suppression_fixed`: Pre-control module suppression held until MV entered control_zone.

### Unresolved

- none

## Artifacts

- trajectory: `artifacts\basic\basic_01_06_900_bugcheck_20260605\scenarios\BASIC-01\trajectory.csv`
- events: `artifacts\basic\basic_01_06_900_bugcheck_20260605\scenarios\BASIC-01\events.jsonl`
- sanity: `artifacts\basic\basic_01_06_900_bugcheck_20260605\scenarios\BASIC-01\sanity.jsonl`
- numeric_summary: `artifacts\basic\basic_01_06_900_bugcheck_20260605\scenarios\BASIC-01\numeric_summary.json`
