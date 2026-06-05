# BASIC Numeric Suite

- run_id: `basic_long_dt_900`
- scenario_count: `6`
- status_counts: `{'failed': 3, 'diagnosed_unresolved': 3}`

| scenario | status | case | active CVs | Eq.10 | merged | primary issue |
| --- | --- | --- | --- | --- | --- | --- |
| BASIC-01 | failed | case_2/case_2 | B01_CFV, B01_CLV | B01_CFV | False | Active CV set does not match BASIC expectation. |
| BASIC-02 | failed | case_3/case_3 | B02_CLV, B02_CFV | B02_CFV | False | Active CV set does not match BASIC expectation. |
| BASIC-03 | failed | case_3/case_4 | B03_CLV, B03_CFV | B03_CFV | False | Expected case_4, observed case_3. |
| BASIC-04 | diagnosed_unresolved | case_2/case_2 | B04_CFV | B04_CFV | False | BASIC expected active CVs to stay in lane_2, but CUC selected another final choice. |
| BASIC-05 | diagnosed_unresolved | case_3/case_3 | B05_CLV | none | False | MV did not finish as merged past x_ramp_end_global during this run. |
| BASIC-06 | diagnosed_unresolved | case_4/case_4 | B06_CLV, B06_CFV | B06_CFV | False | BASIC expected active CVs to stay in lane_2, but CUC selected another final choice. |

## Artifacts

- BASIC-01: `artifacts\basic\basic_long_dt_900\scenarios\BASIC-01\scenario_report.md`
- BASIC-02: `artifacts\basic\basic_long_dt_900\scenarios\BASIC-02\scenario_report.md`
- BASIC-03: `artifacts\basic\basic_long_dt_900\scenarios\BASIC-03\scenario_report.md`
- BASIC-04: `artifacts\basic\basic_long_dt_900\scenarios\BASIC-04\scenario_report.md`
- BASIC-05: `artifacts\basic\basic_long_dt_900\scenarios\BASIC-05\scenario_report.md`
- BASIC-06: `artifacts\basic\basic_long_dt_900\scenarios\BASIC-06\scenario_report.md`
