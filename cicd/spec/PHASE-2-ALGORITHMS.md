# WIA-CICD — PHASE 2: Algorithms

> 弘益人間 (Benefit All Humanity)

This phase formalises the algorithms invoked by the simulator's eight functions (`simulatePipeline`, `evaluateBuildCache`, `runTestPyramid`, `evaluateSecurityGates`, `simulateDeployment`, `evaluateSLO`, plus helpers `setLang`, `showPanel`).

---

## 1. Cache Key Derivation (Content Hash)

A cache key is a SHA-256 over a deterministic concatenation of inputs. Conformant implementations MUST avoid time-of-day or PID inputs.

```
key = SHA256( join('\n', [
   tool_id,            // e.g. "BAZEL@7.1.0"
   target_label,       // e.g. "//app:server"
   sorted(input_paths_with_hashes),
   sorted(env_vars_used),
   toolchain_digest    // base image / SDK SHA-256
]) )
```

Two-tier reads attempt LOCAL → REMOTE; an EXECUTION cache (e.g. Bazel RBE) MAY satisfy reads without a hit when the action result is reproducible.

---

## 2. Cache Hit Composition

Tier targets (simulator constants):

| Tier        | Target                     |
|-------------|----------------------------|
| `LOCAL`     | ≥ 0.80 (`LOCAL_CACHE_HITRATE_TARGET`) |
| `REMOTE`    | ≥ 0.60 (`REMOTE_CACHE_HITRATE_TARGET`) |
| Overall     | ≥ 0.70 (`OVERALL_CACHE_HITRATE_TARGET`) |

Composite hit rate (independent tiers approximation):

```
overall = 1 − (1 − local) · (1 − remote) · (1 − execution)
```

Implementations MAY substitute observed conditional probabilities when telemetry is available.

---

## 3. Topological Sort (DAG Execution Order)

Pipeline jobs form a DAG. The runner MUST execute jobs in topological order. Tie-break by **(priority desc, id asc)** for stability.

```
function topoSort(nodes, edges):
   inDeg = countInDegrees(nodes, edges)
   ready = priorityQueue(nodes where inDeg[n]==0)
   order = []
   while ready not empty:
      n = ready.popMin()
      order.append(n)
      for m in successors(n):
         inDeg[m] -= 1
         if inDeg[m] == 0: ready.push(m)
   if |order| != |nodes|: error("cycle detected")
   return order
```

Reference DAG (5 nodes): `SOURCE → BUILD → TEST_GATE → SECURITY_GATE → CD_HANDOFF`.

---

## 4. Test Pyramid Composition

Two distribution models are recognised:

| Model     | UNIT | INTEGRATION | E2E | Constant         |
|-----------|------|-------------|-----|------------------|
| Pyramid   | 70   | 20          | 10  | `PYRAMID_RATIO`  |
| Trophy    | 30   | 50          | 20  | `ALT_TROPHY_RATIO` |

Per-layer SLO (in `runTestPyramid`):

| Layer       | SLO         | Constant            |
|-------------|-------------|---------------------|
| UNIT        | ≤ 100 ms    | `UNIT_SLO_MS`       |
| INTEGRATION | ≤ 5 s       | `INTEGRATION_SLO_S` |
| E2E         | ≤ 60 s      | `E2E_SLO_S`         |

Quality gates:

```
pass = (coverage   ≥ 0.80)         // COVERAGE_TARGET
     ∧ (mutation   ≥ 0.70)         // MUTATION_SCORE_TARGET
     ∧ (flaky_rate ≤ 0.01)         // FLAKY_RATE_THRESHOLD
```

---

## 5. Security Gate Aggregation

Findings are aggregated across six gates: `SAST · SCA · DAST · SECRETS · CONTAINER · IAC`. The policy verdict is:

```
totalCritical = SECRETS + max(0, SAST_critical − allowedCritical)
fpOK          = false_positive_rate ≤ 0.05      // POLICY_FALSE_POSITIVE_GATE
verdict       = (mode == 'AUDIT')
              ∨ (totalCritical == 0 ∧ fpOK)
```

Mappings:

- OWASP CICD Top 10: `CICD_SEC_1 .. CICD_SEC_10`
- NIST SSDF groups: `PO · PS · PW · RV`
- KISA diagnostic items: 47 (`KISA_DIAGNOSTIC_ITEMS`)

---

## 6. Canary Traffic Curve

Default progression (`TRAFFIC_CURVE`): `5% → 25% → 50% → 100%`. Each step holds for `step_duration_min`. AnalysisRun (Argo Rollouts terminology) evaluates SLO between steps.

```
function canary(steps=[5,25,50,100], stepMin):
   for s in steps:
      shiftTraffic(s)
      sleep(stepMin minutes)
      if !analysisRunPasses():
         rollback()                  // ≤ ROLLBACK_MAX_MIN minutes
         return ABORTED
   return DEPLOYED
```

---

## 7. AnalysisRun — SLO Evaluation

Service-level objective evaluation per window:

```
errorBudget   = 1 − sloTarget          // STD 0.001 · FIN 0.0005
budgetBurned  = observedErrorRate / errorBudget
postmortem    = budgetBurned ≥ 0.20    // ERROR_BUDGET_POSTMORTEM_TRIGGER
```

When `postmortem == true` the on-call rotation MUST schedule a blameless postmortem within 24 hours and document the result via the `wia-cicd-signal-v1` envelope.

---

## 8. Reference Pseudocode — Composite Pipeline Score

```
function pipelineScore(metrics):
   leadOK    = metrics.lead_time_hr  ≤ 1
   failOK    = metrics.failure_rate  ≤ 0.05
   reworkOK  = metrics.rework_rate   ≤ 0.05
   feedbackOK= metrics.fast_feedback_p95_min ≤ 10
   secOK     = metrics.security_scan_p95_min ≤ 15
   deployOK  = metrics.prod_deploy_p95_min   ≤ 60
   passes    = [leadOK, failOK, reworkOK, feedbackOK, secOK, deployOK]
   return ratio(passes.filter(true).length / passes.length)
```

A score of 1.0 across this set is the operational definition of "DORA Elite" within WIA-CICD.

---

© 2026 WIA · MIT License · 弘益人間
