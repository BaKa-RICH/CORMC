from __future__ import annotations

from cormc.onestep.kernel.config import AlgorithmConfig, ScenarioConfig
from cormc.onestep.kernel.cooperation import (
    compute_cooperation_for_gap,
    controllability_branch_for_gap,
)
from cormc.onestep.kernel.derived import compute_derived_params
from cormc.onestep.kernel.gaps import build_gaps
from cormc.onestep.kernel.models import (
    CooperationResult,
    Gap,
    GapEvaluationRow,
    OneStepEvaluationResult,
    ReachabilityResult,
    ScoreResult,
    TimingResult,
)
from cormc.onestep.kernel.reachability import compute_reachability_for_gap, filter_reachable_gaps
from cormc.onestep.kernel.timing_scoring import (
    FAILED_GAP_SCORE,
    compute_timing_for_gap,
    score_gap,
    select_best_gap,
)


def evaluate_one_step_scenario(
    scenario: ScenarioConfig,
    algorithm: AlgorithmConfig,
) -> OneStepEvaluationResult:
    gaps = build_gaps(scenario)
    derived = compute_derived_params(scenario, algorithm)
    reachability = tuple(
        compute_reachability_for_gap(gap, scenario, derived)
        for gap in gaps
    )
    reachable_gaps = filter_reachable_gaps(gaps, reachability)

    cooperation_results: list[CooperationResult] = []
    timing_results: list[TimingResult] = []
    score_results: list[ScoreResult] = []

    for gap in reachable_gaps:
        coop = compute_cooperation_for_gap(gap, gaps, scenario, algorithm, derived)
        cooperation_results.append(coop)
        if not coop.coop_feasible:
            continue
        timing = compute_timing_for_gap(gap, coop, scenario, algorithm)
        score = score_gap(gap, coop, timing, algorithm)
        timing_results.append(timing)
        score_results.append(score)

    scores = tuple(score_results)
    if scores:
        best_score = select_best_gap(scores)
        best_gap = next(gap for gap in gaps if gap.gap_id == best_score.gap_id)
        status = "solved"
        no_solution_reason = None
    elif not reachable_gaps:
        best_score = None
        best_gap = None
        status = "no_solution"
        no_solution_reason = "no_reachable_gap"
    else:
        best_score = None
        best_gap = None
        status = "no_solution"
        no_solution_reason = "no_coop_feasible_gap"

    gap_rows = build_gap_evaluation_rows(
        gaps=gaps,
        reachability=reachability,
        cooperation=tuple(cooperation_results),
        timing=tuple(timing_results),
        scores=scores,
    )

    return OneStepEvaluationResult(
        scenario=scenario,
        algorithm=algorithm,
        derived=derived,
        gaps=gaps,
        reachability=reachability,
        cooperation=tuple(cooperation_results),
        timing=tuple(timing_results),
        scores=scores,
        gap_rows=gap_rows,
        best_score=best_score,
        best_gap=best_gap,
        status=status,
        no_solution_reason=no_solution_reason,
    )


def build_gap_evaluation_rows(
    *,
    gaps: tuple[Gap, ...],
    reachability: tuple[ReachabilityResult, ...],
    cooperation: tuple[CooperationResult, ...],
    timing: tuple[TimingResult, ...],
    scores: tuple[ScoreResult, ...],
) -> tuple[GapEvaluationRow, ...]:
    reachability_by_id = {result.gap_id: result for result in reachability}
    cooperation_by_id = {result.gap_id: result for result in cooperation}
    timing_by_id = {result.gap_id: result for result in timing}
    scores_by_id = {result.gap_id: result for result in scores}
    selected_score = select_best_gap(scores) if scores else None

    rows: list[GapEvaluationRow] = []
    for gap in gaps:
        reach = reachability_by_id[gap.gap_id]
        coop = cooperation_by_id.get(gap.gap_id)
        time = timing_by_id.get(gap.gap_id)
        score = scores_by_id.get(gap.gap_id)
        failed_coop_score = (
            FAILED_GAP_SCORE
            if coop is not None and not coop.coop_feasible
            else None
        )
        rows.append(
            GapEvaluationRow(
                gap_id=gap.gap_id,
                gap_index=gap.index,
                front_vehicle_id=gap.front_vehicle_id,
                rear_vehicle_id=gap.rear_vehicle_id,
                front_controllable=(
                    coop.front_controllable
                    if coop is not None
                    else gap.front_controllable
                ),
                rear_controllable=(
                    coop.rear_controllable
                    if coop is not None
                    else gap.rear_controllable
                ),
                controllability_branch=(
                    coop.controllability_branch
                    if coop is not None
                    else controllability_branch_for_gap(gap)
                ),
                x_rear=gap.x_rear,
                x_front=gap.x_front,
                G_i=gap.G_i,
                c_i=gap.c_i,
                D_i=gap.D_i,
                direction=reach.direction,
                t_reach=reach.t_reach,
                p_pre=reach.p_pre,
                reachable=reach.reachable,
                Delta=coop.Delta if coop is not None else None,
                G_prev=coop.G_prev if coop is not None else None,
                G_next=coop.G_next if coop is not None else None,
                delta_f_bar=coop.delta_f_bar if coop is not None else None,
                delta_r_bar=coop.delta_r_bar if coop is not None else None,
                coop_feasible=coop.coop_feasible if coop is not None else None,
                gamma_f=coop.gamma_f if coop is not None else None,
                gamma_r=coop.gamma_r if coop is not None else None,
                L=coop.L if coop is not None else None,
                U=coop.U if coop is not None else None,
                delta_f_raw=coop.delta_f_raw if coop is not None else None,
                delta_f_star=coop.delta_f_star if coop is not None else None,
                delta_r_star=coop.delta_r_star if coop is not None else None,
                C_coop=coop.C_coop if coop is not None else None,
                d_i=coop.d_i if coop is not None else None,
                t_0=time.t_0 if time is not None else None,
                a_lim=time.a_lim if time is not None else None,
                t_a=time.t_a if time is not None else None,
                t_v=time.t_v if time is not None else None,
                t_m=time.t_m if time is not None else None,
                p_m=time.p_m if time is not None else None,
                C_ego=score.C_ego if score is not None else None,
                J=score.J if score is not None else failed_coop_score,
                included_in_scoring=score is not None,
                failure_reason=(
                    score.failure_reason
                    if score is not None
                    else coop.failure_reason if coop is not None else None
                ),
                is_selected=(
                    selected_score is not None
                    and selected_score.gap_id == gap.gap_id
                ),
            )
        )
    return tuple(rows)
