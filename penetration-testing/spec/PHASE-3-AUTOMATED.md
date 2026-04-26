# WIA-SEC-019: Penetration Testing — Phase 3 Specification

**Version:** 1.0
**Status:** DRAFT
**Last Updated:** 2025-12-25
**Standard ID:** WIA-SEC-019
**Category:** Security (SEC)
**Color:** #8B5CF6

---

## Phase 3: Automated Continuous Penetration Testing

### 3.1 Overview

Phase 3 introduces continuous, automated security testing integrated into CI/CD pipelines. The goal is to shrink mean time to discovery for new vulnerabilities to under 24 hours by running adversarial probes on every code change, every dependency update, and every infrastructure rotation.

### 3.2 Continuous Security Testing Architecture

```json
{
  "continuousTesting": {
    "integration": {
      "cicd": ["GitHub Actions", "GitLab CI", "Jenkins"],
      "triggers": [
        "Code commit",
        "Pull request",
        "Scheduled (daily/weekly)",
        "On-demand"
      ]
    },
    "testingLayers": [
      {"layer": "SAST", "tools": ["Semgrep","Bandit"], "coverage": "Source code analysis"},
      {"layer": "DAST", "tools": ["OWASP ZAP","Burp Suite"], "coverage": "Running app testing"},
      {"layer": "SCA",  "tools": ["Snyk","Dependabot"],     "coverage": "Dependency vulnerabilities"},
      {"layer": "IaC",  "tools": ["Checkov","tfsec"],       "coverage": "Cloud misconfigurations"},
      {"layer": "Container", "tools": ["Trivy","Clair"],    "coverage": "Image vulnerabilities"}
    ]
  }
}
```

### 3.3 CI/CD Pipeline Integration

```yaml
# .github/workflows/security-scan.yml
name: WIA-SEC-019 Continuous Security Testing
on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]
  schedule:
    - cron: '0 2 * * *'   # Daily 02:00 UTC

jobs:
  sast:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Run SAST
        run: semgrep --config=auto --json -o sast-results.json
      - name: Upload findings
        run: |
          curl -X POST https://api.wia.org/pentest/v1/findings \
            -H "Authorization: Bearer ${{ secrets.WIA_API_TOKEN }}" \
            -F "results=@sast-results.json"

  dast:
    runs-on: ubuntu-latest
    steps:
      - name: Deploy to staging
        run: docker compose up -d
      - name: Run OWASP ZAP
        run: |
          docker run -v $(pwd):/zap/wrk/:rw \
            owasp/zap2docker-stable zap-full-scan.py \
            -t http://staging.example.com \
            -J zap-report.json

  dependency-check:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Run Snyk
        run: |
          snyk test --json > snyk-results.json
          snyk monitor
```

### 3.4 Breach and Attack Simulation (BAS)

```json
{
  "breachSimulation": {
    "platform": "WIA-SEC-019 BAS Engine",
    "scenarios": [
      {
        "id": "SIM-001",
        "name": "Ransomware Attack Chain",
        "stages": [
          "Initial Access via Phishing Email",
          "Malware Execution",
          "Privilege Escalation",
          "Lateral Movement",
          "Data Encryption Simulation"
        ],
        "frequency": "Weekly",
        "controlSurface": [
          "emailFiltering",
          "endpointProtection",
          "privilegeEscalation",
          "networkSegmentation",
          "backupRecovery"
        ]
      }
    ],
    "reporting": {
      "format": "JSON",
      "destination": "SIEM integration",
      "alerts": "High-priority failures"
    }
  }
}
```

### 3.5 Findings Schema

Every finding emitted by an automated probe MUST conform to the WIA-SEC-019 finding schema:

```json
{
  "$schema": "https://standards.wia.example/penetration-testing/schemas/v1/finding.json",
  "findingId": "WIA-PT-2026-00021",
  "discoveredAt": "2026-04-26T13:00:00Z",
  "source": "OWASP ZAP",
  "ruleId": "10202",
  "title": "Cross-Site Scripting (Reflected)",
  "severity": "high",
  "cvss": {"version": "3.1", "vector": "AV:N/AC:L/PR:N/UI:R/S:U/C:H/I:H/A:N"},
  "asset": {"type": "url", "value": "https://staging.example.com/search"},
  "evidence": {"request": "...", "response": "..."},
  "remediation": "Encode user input on output, enable Content-Security-Policy default-src 'self'",
  "owner": "platform-app",
  "status": "open"
}
```

Findings MUST be deduplicated by `(ruleId, asset.value, signature_hash)`. Triagers MAY transition status to `accepted-risk` only with documented business justification.

### 3.6 SLA & Deduplication

| Severity | Triage SLA | Remediation SLA | Re-test SLA |
|----------|------------|------------------|-------------|
| Critical | 4 hours | 7 days | 24 hours after fix |
| High | 1 business day | 30 days | 48 hours after fix |
| Medium | 5 business days | 90 days | weekly batch |
| Low | 30 business days | next major release | quarterly |

The platform MUST publish SLA compliance metrics weekly and page the security on-call when an SLA breach is imminent (≤ 24 hours).

### 3.7 Machine Learning-Assisted Triage

```json
{
  "mlTriage": {
    "capabilities": [
      "Intelligent fuzzing based on code structure",
      "Anomaly detection in API behavior",
      "Predictive vulnerability prioritization",
      "Duplicate finding clustering"
    ],
    "controls": [
      "All ML-suggested actions require human approval before customer notification",
      "Models are retrained quarterly using only customer-approved labels",
      "False-positive feedback loop closed via human-in-the-loop"
    ]
  }
}
```

ML output is **advisory only**: every customer-facing action MUST be reviewed by a qualified human analyst before being released.

### 3.8 Telemetry & Metrics

Every CI run MUST emit:

| Metric | Type | Notes |
|--------|------|-------|
| `wia_pt_runs_total` | counter | per pipeline |
| `wia_pt_findings_total{severity}` | counter | by severity |
| `wia_pt_run_duration_seconds` | histogram | p50/p95/p99 |
| `wia_pt_sla_breach_total` | counter | per severity |
| `wia_pt_false_positive_total` | counter | per source |

Operators MUST surface a live dashboard so engineering managers see security debt trend in real time.

### 3.9 Vendor / Tool Neutrality

WIA-SEC-019 Phase 3 is tool-agnostic. Any SAST, DAST, SCA, IaC, or container scanner that produces findings conformant with §3.5 MAY be substituted. The platform MUST refuse to lock in a specific vendor; users own their data and SHOULD be able to export the entire findings backlog as JSON within 5 minutes.

### 3.10 Replay & Audit

Every CI security run MUST be reproducible. The platform MUST record:

- Tool version and command-line invocation
- Source repository commit SHA
- Environment fingerprint (base image digest, dependency lockfile hash)
- Findings emitted with their dedup keys
- Reviewer decisions and timestamps

A replay command MUST exist:

```
wia-pt replay --run-id <ulid>
```

This re-executes the same scanner version against the same commit so a finding can always be traced back to its origin. Replay output MUST match the original within whitelisted nondeterministic dimensions (e.g. timestamps).

### 3.10.1 Custom Rules

Customers MAY ship in-house Semgrep, OPA Rego, or Falco rules through the platform's `custom-rules/` directory. Each rule MUST include:

- Unique ID prefixed with `wia-pt.custom.<owner>.<slug>`
- Severity (`critical`, `high`, `medium`, `low`, `info`)
- A test fixture demonstrating both a true-positive and a true-negative case
- An owner email and on-call rotation reference
- A retirement date (rules without a planned retirement become "evergreen" and MUST be reviewed annually)

Pull requests adding rules MUST run them against a representative repository sample to estimate false-positive rate before merge.

### 3.10.2 Quarantine Workflow

When a probe detects a high-severity finding in production traffic the platform MUST be able to quarantine the offending request without deploying code:

```
wia-pt quarantine create \
  --rule wia-pt.custom.payments.api1-2023-1 \
  --action "block,page-secops" \
  --ttl 24h
```

Quarantine actions MUST emit an audit-trail entry conformant with Phase 2 §2.6. Owners MUST be paged with the quarantine ID, the matching request shape, and a one-click expire link.

### 3.11 Normative References

- ISO/IEC 27001:2022 — ISMS controls A.5.7 (Threat intelligence) and A.8.29 (Security testing)
- NIST SP 800-218 — Secure Software Development Framework
- OWASP ASVS 4.0 — Application Security Verification Standard
- OWASP API Security Top 10 — 2023 edition
- IETF RFC 9457 — Problem Details for HTTP APIs
- CycloneDX 1.5 — Software Bill of Materials
- SLSA Framework v1.0 — Supply-chain Levels for Software Artifacts

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA - World Certification Industry Association
