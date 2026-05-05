/**
 * DORA + Rework Rate evaluator. Thresholds mirror the simulator constants.
 */

export const DORA_ELITE = {
  LEAD_TIME_HR:          1,
  FAILURE_RATE:          0.05,
  MTTR_MIN:              5,
  REWORK_RATE:           0.05,
  FAST_FEEDBACK_P95_MIN: 10,
  SECURITY_SCAN_P95_MIN: 15,
  PROD_DEPLOY_P95_MIN:   60
} as const;

export interface DoraMetrics {
  lead_time_hr:          number;
  deploy_freq_per_day:   number;
  failure_rate:          number;
  mttr_min:              number;
  rework_rate:           number;
  fast_feedback_p95_min?: number;
  security_scan_p95_min?: number;
  prod_deploy_p95_min?:   number;
}

export interface DoraVerdict {
  elite:   boolean;
  passes:  string[];
  failures: string[];
  /** Fraction of axes that pass (0–1). */
  score:   number;
}

export function evaluateDora(metrics: DoraMetrics): DoraVerdict {
  const checks: { key: string; ok: boolean }[] = [
    { key: 'lead_time_hr',          ok: metrics.lead_time_hr        <= DORA_ELITE.LEAD_TIME_HR },
    { key: 'failure_rate',          ok: metrics.failure_rate        <= DORA_ELITE.FAILURE_RATE },
    { key: 'mttr_min',              ok: metrics.mttr_min            <= DORA_ELITE.MTTR_MIN },
    { key: 'rework_rate',           ok: metrics.rework_rate         <= DORA_ELITE.REWORK_RATE }
  ];
  if (metrics.fast_feedback_p95_min != null) checks.push({ key: 'fast_feedback_p95_min', ok: metrics.fast_feedback_p95_min <= DORA_ELITE.FAST_FEEDBACK_P95_MIN });
  if (metrics.security_scan_p95_min != null) checks.push({ key: 'security_scan_p95_min', ok: metrics.security_scan_p95_min <= DORA_ELITE.SECURITY_SCAN_P95_MIN });
  if (metrics.prod_deploy_p95_min   != null) checks.push({ key: 'prod_deploy_p95_min',   ok: metrics.prod_deploy_p95_min   <= DORA_ELITE.PROD_DEPLOY_P95_MIN });

  const passes   = checks.filter(c => c.ok).map(c => c.key);
  const failures = checks.filter(c => !c.ok).map(c => c.key);
  return { elite: failures.length === 0, passes, failures, score: passes.length / checks.length };
}

/** Compose 3-tier cache hit rate (independent tiers approximation). */
export function composeCacheHit(local: number, remote: number, execution: number): number {
  const clamp = (x: number) => Math.max(0, Math.min(1, x));
  const l = clamp(local), r = clamp(remote), e = clamp(execution);
  return 1 - (1 - l) * (1 - r) * (1 - e);
}

/** Error-budget burn (0–∞ where ≥ 1.0 means budget exhausted). */
export function errorBudgetBurn(observedErrorRate: number, sloTarget: number): number {
  const budget = Math.max(1e-12, 1 - sloTarget);
  return observedErrorRate / budget;
}
