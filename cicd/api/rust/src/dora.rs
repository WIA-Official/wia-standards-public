//! DORA + Rework Rate evaluator. Thresholds mirror the simulator constants.

/// Elite threshold — lead time in hours.
pub const DORA_ELITE_LEAD_TIME_HR:    f64 = 1.0;
/// Elite threshold — change failure ratio.
pub const DORA_ELITE_FAILURE_RATE:    f64 = 0.05;
/// Elite threshold — MTTR in minutes.
pub const DORA_ELITE_MTTR_MIN:        f64 = 5.0;
/// Elite threshold — rework rate.
pub const REWORK_RATE_THRESHOLD:      f64 = 0.05;
/// Elite threshold — fast-feedback p95 minutes.
pub const FAST_FEEDBACK_P95_MIN:      f64 = 10.0;
/// Elite threshold — security scan p95 minutes.
pub const SECURITY_SCAN_P95_MIN:      f64 = 15.0;
/// Elite threshold — production deploy p95 minutes.
pub const PROD_DEPLOY_P95_MIN:        f64 = 60.0;

/// Observed pipeline metrics.
#[derive(Debug, Clone, Default)]
pub struct DoraMetrics {
    /// Lead time (hours).
    pub lead_time_hr:        f64,
    /// Deploys per day.
    pub deploy_freq_per_day: f64,
    /// Change failure ratio.
    pub failure_rate:        f64,
    /// Mean time to recover (minutes).
    pub mttr_min:            f64,
    /// Rework rate.
    pub rework_rate:         f64,
    /// Optional fast-feedback p95 minutes.
    pub fast_feedback_p95_min: Option<f64>,
    /// Optional security scan p95 minutes.
    pub security_scan_p95_min: Option<f64>,
    /// Optional prod deploy p95 minutes.
    pub prod_deploy_p95_min:   Option<f64>,
}

/// Verdict returned by [`evaluate_dora`].
#[derive(Debug, Clone)]
pub struct DoraVerdict {
    /// True when every axis is within its Elite threshold.
    pub elite:    bool,
    /// Names of axes that passed.
    pub passes:   Vec<&'static str>,
    /// Names of axes that failed.
    pub failures: Vec<&'static str>,
    /// Pass-fraction (0–1).
    pub score:    f64,
}

/// Evaluate observed metrics against Elite thresholds.
pub fn evaluate_dora(m: &DoraMetrics) -> DoraVerdict {
    let mut checks: Vec<(&'static str, bool)> = vec![
        ("lead_time_hr", m.lead_time_hr   <= DORA_ELITE_LEAD_TIME_HR),
        ("failure_rate", m.failure_rate   <= DORA_ELITE_FAILURE_RATE),
        ("mttr_min",     m.mttr_min       <= DORA_ELITE_MTTR_MIN),
        ("rework_rate",  m.rework_rate    <= REWORK_RATE_THRESHOLD),
    ];
    if let Some(v) = m.fast_feedback_p95_min { checks.push(("fast_feedback_p95_min", v <= FAST_FEEDBACK_P95_MIN)); }
    if let Some(v) = m.security_scan_p95_min { checks.push(("security_scan_p95_min", v <= SECURITY_SCAN_P95_MIN)); }
    if let Some(v) = m.prod_deploy_p95_min   { checks.push(("prod_deploy_p95_min",   v <= PROD_DEPLOY_P95_MIN)); }

    let passes:   Vec<&'static str> = checks.iter().filter(|(_, ok)|  *ok).map(|(k, _)| *k).collect();
    let failures: Vec<&'static str> = checks.iter().filter(|(_, ok)| !*ok).map(|(k, _)| *k).collect();
    let total = checks.len() as f64;
    DoraVerdict { elite: failures.is_empty(), score: passes.len() as f64 / total, passes, failures }
}

/// Independent-tier composite cache hit rate.
pub fn compose_cache_hit(local: f64, remote: f64, execution: f64) -> f64 {
    let clamp = |x: f64| x.clamp(0.0, 1.0);
    let (l, r, e) = (clamp(local), clamp(remote), clamp(execution));
    1.0 - (1.0 - l) * (1.0 - r) * (1.0 - e)
}

/// Error-budget burn fraction (≥ 1.0 means budget exhausted).
pub fn error_budget_burn(observed_error_rate: f64, slo_target: f64) -> f64 {
    let budget = (1.0 - slo_target).max(1e-12);
    observed_error_rate / budget
}
