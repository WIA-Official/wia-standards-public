# WIA-CICD — PHASE 4: Integration

> 弘益人間 (Benefit All Humanity)

This phase maps WIA-CICD onto the runners and registries operators are most likely to already operate, plus the Korean compliance frameworks.

---

## 1. GitHub Actions

```yaml
# .github/workflows/wia-cicd.yml
name: wia-cicd
on: [push, pull_request]
permissions:
  contents: read
  id-token: write          # required for Sigstore keyless
  attestations: write
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: ./cli/cicd.sh validate pipeline.yml
      - run: ./cli/cicd.sh generate-sbom --format CYCLONEDX --out sbom.cdx.json
      - uses: actions/attest-build-provenance@v1
        with:
          subject-path: dist/*
      - run: cosign verify-attestation --type slsaprovenance dist/*
```

The job emits SLSA L3 provenance via `attest-build-provenance@v1` and verifies the signature locally before publishing.

---

## 2. GitLab CI

```yaml
# .gitlab-ci.yml
include:
  - remote: 'https://wia.standards/cicd/templates/gitlab.v1.yml'

stages: [source, build, test_gate, security_gate, cd_handoff]

build:
  stage: build
  script:
    - ./cli/cicd.sh validate pipeline.yml
    - ./cli/cicd.sh generate-sbom --format SPDX --out sbom.spdx
  artifacts:
    reports:
      cyclonedx: sbom.cdx.json

security_scan:
  stage: security_gate
  script: ./cli/cicd.sh scan --gates SAST,SCA,DAST,SECRETS,CONTAINER,IAC
```

GitLab's built-in DAST / SAST templates are accepted as long as they emit SARIF the security gate ingests.

---

## 3. Jenkins (Declarative)

```groovy
pipeline {
  agent any
  options { timeout(time: 60, unit: 'MINUTES') }
  stages {
    stage('Source')        { steps { checkout scm } }
    stage('Build')         { steps { sh './cli/cicd.sh build' } }
    stage('Test Gate')     { steps { sh './cli/cicd.sh test --pyramid PYRAMID' } }
    stage('Security Gate') { steps { sh './cli/cicd.sh scan --policy ENFORCE' } }
    stage('CD Handoff')    { steps { sh './cli/cicd.sh deploy --strategy CANARY' } }
  }
  post {
    always { sh './cli/cicd.sh emit --signal wia-cicd-signal-v1' }
  }
}
```

---

## 4. Tekton

Tekton Pipelines map 1:1 to WIA-CICD nodes via `Task` and `PipelineRun` objects:

```yaml
apiVersion: tekton.dev/v1
kind: Pipeline
metadata: { name: wia-cicd }
spec:
  tasks:
    - { name: source,         taskRef: { name: source } }
    - { name: build,          runAfter: [source],         taskRef: { name: build } }
    - { name: test-gate,      runAfter: [build],          taskRef: { name: test-gate } }
    - { name: security-gate,  runAfter: [test-gate],      taskRef: { name: security-gate } }
    - { name: cd-handoff,     runAfter: [security-gate],  taskRef: { name: cd-handoff } }
```

---

## 5. Argo Workflows

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Workflow
metadata: { generateName: wia-cicd- }
spec:
  entrypoint: dag
  templates:
    - name: dag
      dag:
        tasks:
          - { name: SOURCE,        template: source }
          - { name: BUILD,         template: build,         dependencies: [SOURCE] }
          - { name: TEST_GATE,     template: test-gate,     dependencies: [BUILD] }
          - { name: SECURITY_GATE, template: security-gate, dependencies: [TEST_GATE] }
          - { name: CD_HANDOFF,    template: cd-handoff,    dependencies: [SECURITY_GATE] }
```

Argo Rollouts handles the canary traffic curve `[5, 25, 50, 100]` and AnalysisRun SLO checks documented in PHASE-2.

---

## 6. KISA 47 — Diagnostic Item Mapping (Korea)

The 47 KISA cloud security diagnostic items align to WIA-CICD security gates and policy mode as follows (illustrative subset):

| KISA item bucket          | WIA-CICD locus                                  |
|---------------------------|-------------------------------------------------|
| Identity & access         | OIDC token flow (PHASE-3 §2)                    |
| Secret management         | `SECRETS` gate (`POLICY_FALSE_POSITIVE_GATE`)   |
| Vulnerability scanning    | `SAST · SCA · DAST · CONTAINER`                 |
| Configuration as code     | `IAC` gate                                      |
| Logging & audit           | `wia-cicd-signal-v1` over OTLP                  |
| Incident response         | KrCERT envelope (PHASE-3 §4)                    |

Operators serving Korean public sector workloads MUST evidence each KISA item via signal payloads or stored provenance.

---

## 7. ISMS-P 102 Control Mapping (Korea)

ISMS-P (정보보호 및 개인정보보호 관리체계) 102 controls overlap WIA-CICD primarily in:

- 1.1 Management policy → PHASE-1 pipeline definition under version control
- 2.4 Software development security → PHASE-2 algorithms + PHASE-1 test SLO
- 2.5 Cryptography → Sigstore signing keys / KMS rotation
- 2.7 Incident response → PHASE-3 §4 KrCERT envelope
- 2.8 Operational management → DORA + Rework Rate dashboards
- 2.10 Personal data protection → SBOM + provenance traceability

---

## 8. CSAP (Korea Public-Sector Cloud)

CSAP (Cloud Security Assurance Program) requires:

1. Hermetic build environment (SLSA L3 hermetic property)
2. Source-to-deploy traceability (provenance + Rekor inclusion proof)
3. Audit log retention ≥ 1 year (OTLP collector → cold storage)
4. Incident reporting to KISA within statutory window (PHASE-3 §4)

---

## 9. Quickstart Cross-Reference

| Need                       | Where                                         |
|----------------------------|-----------------------------------------------|
| Configure pipeline YAML    | PHASE-1 §1 + `schemas/pipeline.schema.json`   |
| Compute cache hit rate     | PHASE-2 §2                                    |
| Sign provenance            | PHASE-3 §2                                    |
| Implement canary           | PHASE-2 §6 + PHASE-4 §5                       |
| Emit signals               | PHASE-3 §1 + `schemas/wia-cicd-signal-v1.schema.json` |
| File KISA / KrCERT         | PHASE-3 §4 + PHASE-4 §6                       |

---

© 2026 WIA · MIT License · 弘益人間
