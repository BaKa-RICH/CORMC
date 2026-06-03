# P14 pre-P15 Baseline Run Report

- run_id: `post_p15`
- output root: `artifacts\baseline\post_p15`
- generated: `p15-engine-core-consolidation`

## Deterministic Baseline Scenarios

- `MVS-E2E-1`
- `MVS-CUC-1A_override_choice1`
- `MVS-CUC-2`
- `MVS-CUC-3`
- `MVS-SAFE-1A_waiting_cap`
- `MVS-SAFE-1B_executing_cap_lateral_consumption`
- `MVS-SAFE-2`
- `MVS-COMMIT-1-full`

## Suite Summary

- suite_status: `passed`
- required_green count: `20`
- required_failed: `0`
- required_blocked: `0`
- runner_gaps: `0`
- probe list: `MVS-CUC-1B_real_utility_probe`
- deferred list: `MVS-CUC-1C_real_utility_choice1_locked`

## Scenario Summaries

| scenario_id | status | final step | final t | active vehicles | sanity pass/fail | PNG | summary |
|---|---|---:|---:|---:|---|---|---|
| MVS-E2E-1 | max_steps_reached | 70 | 7 | 4 | 2621/0 | scenarios\MVS-E2E-1\time_space.png | scenarios\MVS-E2E-1\scenario_summary.md |
| MVS-CUC-1A_override_choice1 | max_steps_reached | 1 | 0.1 | 5 | 37/0 | scenarios\MVS-CUC-1A_override_choice1\time_space.png | scenarios\MVS-CUC-1A_override_choice1\scenario_summary.md |
| MVS-CUC-2 | max_steps_reached | 1 | 0.1 | 5 | 38/0 | scenarios\MVS-CUC-2\time_space.png | scenarios\MVS-CUC-2\scenario_summary.md |
| MVS-CUC-3 | max_steps_reached | 1 | 0.1 | 5 | 38/0 | scenarios\MVS-CUC-3\time_space.png | scenarios\MVS-CUC-3\scenario_summary.md |
| MVS-SAFE-1A_waiting_cap | max_steps_reached | 1 | 0.1 | 3 | 41/0 | scenarios\MVS-SAFE-1A_waiting_cap\time_space.png | scenarios\MVS-SAFE-1A_waiting_cap\scenario_summary.md |
| MVS-SAFE-1B_executing_cap_lateral_consumption | max_steps_reached | 1 | 0.1 | 3 | 40/0 | scenarios\MVS-SAFE-1B_executing_cap_lateral_consumption\time_space.png | scenarios\MVS-SAFE-1B_executing_cap_lateral_consumption\scenario_summary.md |
| MVS-SAFE-2 | max_steps_reached | 1 | 0.1 | 3 | 39/0 | scenarios\MVS-SAFE-2\time_space.png | scenarios\MVS-SAFE-2\scenario_summary.md |
| MVS-COMMIT-1-full | max_steps_reached | 21 | 2.1 | 9 | 41/0 | scenarios\MVS-COMMIT-1-full\time_space.png | scenarios\MVS-COMMIT-1-full\scenario_summary.md |

## Formula Status

- legacy proxy markers present: `none`

| formula area | aggregate status | scenario statuses |
|---|---|---|
| cuc_eq11_eq16 | locked_formula | MVS-E2E-1: not_observed, MVS-CUC-1A_override_choice1: locked_formula, MVS-CUC-2: locked_formula, MVS-CUC-3: locked_formula, MVS-SAFE-1A_waiting_cap: not_observed, MVS-SAFE-1B_executing_cap_lateral_consumption: not_observed, MVS-SAFE-2: not_observed, MVS-COMMIT-1-full: not_observed |
| cav_eq17_eq27 | locked_formula | MVS-E2E-1: locked_formula, MVS-CUC-1A_override_choice1: locked_formula, MVS-CUC-2: locked_formula, MVS-CUC-3: locked_formula, MVS-SAFE-1A_waiting_cap: locked_formula, MVS-SAFE-1B_executing_cap_lateral_consumption: locked_formula, MVS-SAFE-2: locked_formula, MVS-COMMIT-1-full: locked_formula |
| chv_eq28_eq29 | locked_formula | MVS-E2E-1: locked_formula, MVS-CUC-1A_override_choice1: not_observed, MVS-CUC-2: not_observed, MVS-CUC-3: locked_formula, MVS-SAFE-1A_waiting_cap: not_observed, MVS-SAFE-1B_executing_cap_lateral_consumption: not_observed, MVS-SAFE-2: not_observed, MVS-COMMIT-1-full: not_observed |
| front_collision_eq42_eq46 | locked_formula | MVS-E2E-1: locked_formula, MVS-CUC-1A_override_choice1: locked_formula, MVS-CUC-2: not_observed, MVS-CUC-3: not_observed, MVS-SAFE-1A_waiting_cap: not_observed, MVS-SAFE-1B_executing_cap_lateral_consumption: locked_formula, MVS-SAFE-2: locked_formula, MVS-COMMIT-1-full: locked_formula |

| subformula area | aggregate status | evidence count |
|---|---|---:|
| cuc_eq14_eq15 | locked_formula | 6 |
| cav_cruising_eq20 | locked_formula | 89 |
| cav_cpid_eq21_eq27 | locked_formula | 5 |
| chv_idm_eq28_eq29 | locked_formula | 141 |
| front_collision_speed_constraint | locked_formula | 27 |

## Run-Level Files

- artifact manifest: `artifact_manifest.json`
- regression report: `regression_report.json`
- baseline comparison contract: `baseline_comparison_contract.json`

## P15 Baseline Note

This directory is the pre-P15 baseline. P15 may read these JSON and CSV files for regression comparison.
For human review, open this run_report.md first, then inspect each scenario_summary.md and time_space.png.
The artifact outputs are evidence only: they do not feed back into vehicle motion, and x_plot remains renderer-derived.
