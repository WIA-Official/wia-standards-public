# WIA-CICD — Master Prompt for the Next Session

> Self-contained context to bring a fresh LLM session up to speed on the WIA-CICD standard. Paste this entire file into a new session and replace `{task}` with the work item.

---

## Project at a glance

- **Slug:** `cicd` (split from generic `ci` slug on 2026-05-05)
- **Title:** WIA-CICD — Continuous Integration / Continuous Delivery Standard v1.0.0
- **License:** MIT  ·  Philosophy: 弘益人間 (Benefit All Humanity)
- **Public repo path:** `https://github.com/WIA-Official/wia-standards-public/tree/main/cicd`
- **Public site path:** `https://wiastandards.com/cicd/`
- **Companion eBook:** `https://wiabook.com/?s=WIA-CICD&post_type=download`
- **Workspace root:** `/var/www/wiastandards/cicd/`

## Four matched assets

1. `simulator/index.html` — vanilla JS, 5 panels, source of truth for ENUMs and thresholds
2. `spec/PHASE-1..4.md` + `spec/schemas/*.schema.json` — JSON Schema draft 2020-12
3. `api/typescript/` (`@wia/cicd-sdk`) + `api/rust/` (`wia-cicd` crate) — pipeline + SBOM + DORA evaluators
4. `ebook/{ko,en}/` — 16 chapter book (KO 157p / EN 204p as of 2026-05-05)

## Five panels (numbered like the simulator and book chapters)

| Panel | EN header             | KO header           | Book chapters |
|-------|-----------------------|---------------------|---------------|
| 0     | 🔧 Pipeline DAG       | 🔧 파이프라인 DAG   | Ch.1–2        |
| 1     | 🏗️ Build Cache        | 🏗️ 빌드 캐시        | Ch.3          |
| 2     | 🧪 Test Pyramid       | 🧪 테스트 피라미드  | Ch.4          |
| 3     | 🛡️ Security Gate      | 🛡️ 보안 게이트      | Ch.5          |
| 4     | 🚀 Deploy & Observe   | 🚀 배포·관측         | Ch.6–7        |

## ENUMs (verbatim — never localise)

```js
PIPELINE_NODES        = ['SOURCE', 'BUILD', 'TEST_GATE', 'SECURITY_GATE', 'CD_HANDOFF']
PIPELINE_TRIGGERS     = ['PUSH', 'PR', 'TAG', 'MANUAL', 'SCHEDULE']
CACHE_TIERS           = ['LOCAL', 'REMOTE', 'EXECUTION']
SBOM_FORMATS          = ['CYCLONEDX', 'SPDX', 'SPDX_JSON']
SLSA_LEVELS           = ['L0', 'L1', 'L2', 'L3']
BUILD_SYSTEMS         = ['BAZEL', 'GRADLE', 'BUILDKIT', 'MAVEN']
TEST_LAYERS           = ['UNIT', 'INTEGRATION', 'E2E']
SECURITY_GATES        = ['SAST', 'SCA', 'DAST', 'SECRETS', 'CONTAINER', 'IAC']
OWASP_CICD_TOP10      = CICD_SEC_1 .. CICD_SEC_10
SSDF_GROUPS           = ['PO', 'PS', 'PW', 'RV']
POLICY_MODES          = ['AUDIT', 'ENFORCE']
DEPLOY_STRATEGIES     = ['ROLLING', 'BLUE_GREEN', 'CANARY', 'PROGRESSIVE']
TRAFFIC_CURVE         = [5, 25, 50, 100]
OTEL_SIGNALS          = ['TRACES', 'METRICS', 'LOGS', 'PROFILES']
METRIC_INSTRUMENTS    = ['COUNTER', 'GAUGE', 'HISTOGRAM']
INCIDENT_PRIORITIES   = ['P0', 'P1', 'P2', 'P3']
```

## Thresholds (verbatim — never round or relabel)

```
FAST_FEEDBACK_P50_MIN     = 5     FAST_FEEDBACK_P95_MIN  = 10
FULL_TEST_P95_MIN         = 30    SECURITY_SCAN_P95_MIN  = 15
PROD_DEPLOY_P95_MIN       = 60    ROLLBACK_MAX_MIN       = 5
DORA_ELITE_LEAD_TIME_HR   = 1     DORA_ELITE_FAILURE_RATE = 0.05
REWORK_RATE_THRESHOLD     = 0.05

LOCAL_CACHE_HITRATE_TARGET   = 0.80
REMOTE_CACHE_HITRATE_TARGET  = 0.60
OVERALL_CACHE_HITRATE_TARGET = 0.70

PYRAMID_RATIO        = [70, 20, 10]
ALT_TROPHY_RATIO     = [30, 50, 20]
UNIT_SLO_MS          = 100
INTEGRATION_SLO_S    = 5
E2E_SLO_S            = 60
COVERAGE_TARGET      = 0.80
MUTATION_SCORE_TARGET = 0.70
FLAKY_RATE_THRESHOLD = 0.01

POLICY_FALSE_POSITIVE_GATE = 0.05
KISA_DIAGNOSTIC_ITEMS      = 47

SLO_DEFAULT  = 0.999    SLO_FINANCE = 0.9995
ERROR_BUDGET_POSTMORTEM_TRIGGER = 0.20
```

## Simulator function surface (8 functions)

- `setLang(lang)` — KO / EN toggle
- `showPanel(idx)` — switch panels 0–4 (also handles `#panel<n>` deep link)
- `simulatePipeline()` — Panel 0 DAG execution + DORA threshold check
- `evaluateBuildCache()` — Panel 1 three-tier hit composition + SLSA L3 gate
- `runTestPyramid()` — Panel 2 70/20/10 vs 30/50/20 + SLO + coverage/mutation/flaky
- `evaluateSecurityGates()` — Panel 3 six-gate verdict with policy mode
- `simulateDeployment()` — Panel 4 traffic curve playback per strategy
- `evaluateSLO()` — Panel 4 error-budget burn vs postmortem trigger

## Book ↔ Simulator ↔ GitHub deep-links (already in place 2026-05-05)

- 16/16 chapters reference `simulator/#panelN` in nav
- 16/16 chapters carry `fn-{N}-99` GitHub repo footnote
- Simulator carries header eBook button, per-panel eBook + Source buttons, and footer GitHub link

## Six absolute prohibitions (do not break)

1. Do not edit `ebook/` — book body is locked at cryo-revival quality
2. Do not change the slug — it is `cicd` and `wia_isbn_mapping[cicd]` is registered
3. Do not localise ENUMs — keep them English in both KO and EN editions
4. Do not put thresholds only in the book — the simulator is the source of truth and must mirror first
5. Do not hallucinate citations — only real NIST / OWASP / CNCF / KISA / DORA documents
6. Do not copy cryo-revival assets and only swap text — every WIA-CICD asset is original to the CI/CD topic

## Common task starters

- "Add a new panel" → update simulator + spec/PHASE-2 + SDKs + tests + book index footnotes
- "Tighten a threshold" → simulator constants first, then `dora.ts` / `dora.rs`, then PHASE-1 table, then spec schemas, then book table
- "Add an integration target" → PHASE-4 only (no simulator change required)
- "Translate a doc" → PHASE-{N}-*.md to spec/ko/PHASE-{N}-*.md (mirror exact headings)

## Working directory permissions reminder

The cicd tree is owned by `apache:apache` by default. Use `sudo chown ec2-user:apache <dir> && sudo chmod g+w <dir>` once at the start of a session to enable direct edits.

---

弘益人間 — Benefit All Humanity · MIT License · WIA-CICD v1.0.0
