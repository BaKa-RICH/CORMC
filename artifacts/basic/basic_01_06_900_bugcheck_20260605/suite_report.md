# BASIC Numeric Suite

- run_id: `basic_01_06_900_bugcheck_20260605`
- scenario_count: `6`
- status_counts: `{'passed': 1, 'diagnosed_unresolved': 4, 'failed': 1}`

| scenario | status | case | active CVs | Eq.10 | merged | primary issue |
| --- | --- | --- | --- | --- | --- | --- |
| BASIC-01 | passed | case_2/case_2 | B01_CFV | B01_CFV | True |  |
| BASIC-02 | diagnosed_unresolved | case_3/case_3 | B02_CLV | none | False | BASIC expected active CVs to stay in lane_2, but CUC selected another final choice. |
| BASIC-03 | failed | case_3/case_4 | B03_CLV | none | False | Expected case_4, observed case_3. |
| BASIC-04 | diagnosed_unresolved | case_2/case_2 | B04_CFV | B04_CFV | False | BASIC expected active CVs to stay in lane_2, but CUC selected another final choice. |
| BASIC-05 | diagnosed_unresolved | case_3/case_3 | B05_CLV | none | False | BASIC expected active CVs to stay in lane_2, but CUC selected another final choice. |
| BASIC-06 | diagnosed_unresolved | case_4/case_4 | B06_CLV, B06_CFV | B06_CFV | False | BASIC expected active CVs to stay in lane_2, but CUC selected another final choice. |

## Artifacts

- BASIC-01: `artifacts\basic\basic_01_06_900_bugcheck_20260605\scenarios\BASIC-01\scenario_report.md`
- BASIC-02: `artifacts\basic\basic_01_06_900_bugcheck_20260605\scenarios\BASIC-02\scenario_report.md`
- BASIC-03: `artifacts\basic\basic_01_06_900_bugcheck_20260605\scenarios\BASIC-03\scenario_report.md`
- BASIC-04: `artifacts\basic\basic_01_06_900_bugcheck_20260605\scenarios\BASIC-04\scenario_report.md`
- BASIC-05: `artifacts\basic\basic_01_06_900_bugcheck_20260605\scenarios\BASIC-05\scenario_report.md`
- BASIC-06: `artifacts\basic\basic_01_06_900_bugcheck_20260605\scenarios\BASIC-06\scenario_report.md`
