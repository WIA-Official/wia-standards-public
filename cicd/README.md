# WIA-CICD

> Open standard initiative for **Continuous Integration / Continuous Delivery** pipelines — pipeline DAG, build cache, test pyramid, security gate, deploy & observe.

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()
[![DORA](https://img.shields.io/badge/DORA-2024-orange.svg)]()
[![SLSA](https://img.shields.io/badge/SLSA-L3-red.svg)]()

弘益人間 (Hongik Ingan) — Benefit All Humanity

## 📋 Overview

WIA-CICD is an open standard initiative that codifies the five-node delivery
pipeline (`SOURCE → BUILD → TEST_GATE → SECURITY_GATE → CD_HANDOFF`) into a
reproducible specification, an interactive simulator, and reference SDKs in
TypeScript and Rust. Every ENUM and threshold mirrors a primary source
(NIST SSDF, OWASP CICD Top 10, CNCF OpenTelemetry, DORA Accelerate,
SLSA Provenance, KISA, ISMS-P) so that local pipelines, vendor platforms and
compliance audits can converge on a single shared vocabulary.

The simulator at `/cicd/simulator/index.html` is the canonical source of
truth: any constant cited in this repo, in the SDKs, or in the companion
book is mirrored from there.

### Five Panels (mirrors the book)

| Panel | Topic | Primary Sources |
|-------|-------|-----------------|
| 0 | Pipeline DAG | DORA 2024, SLSA Provenance v1, NIST SSDF |
| 1 | Build Cache | Bazel Remote Execution API, Gradle Build Cache, BuildKit |
| 2 | Test Pyramid | ISO/IEC 25010:2023, Mike Cohn, Kent C. Dodds (Testing Trophy) |
| 3 | Security Gate | OWASP CICD Top 10, NIST SSDF v1.1, KISA 47, ISMS-P 102 |
| 4 | Deploy & Observe | OpenTelemetry 1.x, Argo Rollouts, Google SRE Workbook |

### DORA Elite Thresholds (mirrored across simulator · TS · Rust)

| Axis | Threshold | Source |
|------|-----------|--------|
| Lead Time | ≤ 1 hour | DORA Accelerate |
| Deploy Freq | on demand | DORA Accelerate |
| Change Failure Rate | ≤ 5 % | DORA Accelerate |
| MTTR | ≤ 5 min | DORA Accelerate |
| Rework Rate | ≤ 5 % | DevEx 2024 |

## 🚀 Quick Start

### Open the Simulator (local)

```bash
git clone https://github.com/WIA-Official/wia-standards-public.git
cd wia-standards-public/cicd
python3 -m http.server 8080
# Browse: http://localhost:8080/simulator/
```

### Validate a Pipeline (CLI)

```bash
./cli/cicd.sh validate examples/pipeline.yaml
./cli/cicd.sh generate-sbom --format cyclonedx --out sbom.json
./cli/cicd.sh verify-provenance attestation.intoto.jsonl
```

### Use the SDK

```typescript
// TypeScript
import { validatePipeline, topoSort, evaluateDora, DORA_ELITE } from '@wia/cicd-sdk';

const verdict = evaluateDora({
  lead_time_hr: 0.4, deploy_freq_per_day: 12, failure_rate: 0.02,
  mttr_min: 3, rework_rate: 0.03
});
console.log(verdict.elite, verdict.score);
```

```rust
// Rust
use wia_cicd::{evaluate_dora, DoraMetrics};

let m = DoraMetrics { lead_time_hr: 0.4, deploy_freq_per_day: 12.0,
    failure_rate: 0.02, mttr_min: 3.0, rework_rate: 0.03,
    fast_feedback_p95_min: Some(8.0), security_scan_p95_min: Some(12.0),
    prod_deploy_p95_min: Some(45.0) };
let v = evaluate_dora(&m);
assert!(v.elite);
```

## 📚 Documentation

| Asset | Path | Purpose |
|-------|------|---------|
| Book | [wiabook.com](https://wiabook.com/?s=WIA-CICD&post_type=download) | Full narrative, KO + EN editions |
| Simulator | `simulator/index.html` | 5-panel interactive truth |
| Spec | `spec/PHASE-1~4.md` + `spec/schemas/*.schema.json` | Data formats, algorithms, protocol, integration |
| SDK | `api/typescript/`, `api/rust/` | Reference implementations + tests |
| CLI | `cli/cicd.sh` | Validate / SBOM / Provenance |
| Prompts | `prompts/MASTER-PROMPT.md` | Self-contained LLM context for next session |

## 🧱 Specification Layout

```
spec/
├── PHASE-1-DATA-FORMAT.md     pipeline YAML, in-toto attestation, SBOM, DORA
├── PHASE-2-ALGORITHMS.md      cache key (SHA-256), DAG topo sort, canary curve, AnalysisRun
├── PHASE-3-PROTOCOL.md        OTLP wire, Sigstore Cosign keyless, Argo CD GitOps, KrCERT
├── PHASE-4-INTEGRATION.md     GitHub Actions / GitLab / Jenkins / Tekton / Argo + KISA + ISMS-P + CSAP
└── schemas/
    ├── pipeline.schema.json
    ├── provenance.schema.json
    ├── dora-metrics.schema.json
    └── wia-cicd-signal-v1.schema.json
```

## 🔧 Components

### Specification

JSON Schemas are the machine-readable contract; the four PHASE markdown files
are the human-readable derivation. Every YAML or JSON document validated by
this standard cites a JSON Pointer back into the schema.

### SDKs

- **TypeScript** (`@wia/cicd-sdk`) — Vitest unit tests, no runtime deps
- **Rust** (`wia-cicd` crate) — `forbid(unsafe_code)`, integration tests

### CLI Tool

`cli/cicd.sh` is a POSIX shell wrapper that calls the SDKs over stdin/stdout
so it runs in any container that has `node` or `cargo` available.

## 🌍 Philosophy

**홍익인간 (弘益人間)** — Benefit All Humanity

Continuous delivery is plumbing for human collaboration. WIA-CICD treats the
pipeline as a shared resource between developers, security engineers,
operators and auditors — not as a private optimisation. The standard is
deliberately compatible with KISA, ISMS-P and CSAP so that public-sector
organisations in Korea can adopt it without parallel paperwork, and equally
deliberately compatible with NIST SSDF, OWASP CICD Top 10 and SLSA so that
international vendors can plug in unchanged.

## 🔗 Related Standards

- [`cryo-revival`](../cryo-revival) — reference template for asset layout
- [`ai-human-coexistence`](../ai-human-coexistence) — companion governance standard
- [`health`](../health) — HL7 FHIR R5 alignment

## 📄 License

MIT License — Copyright (c) 2026 WIA (World Intelligence Alliance) / SmileStory Inc.

## 📞 Contact

- **Organization**: WIA (World Intelligence Alliance)
- **Repository**: https://github.com/WIA-Official/wia-standards-public/tree/main/cicd
- **Book**: https://wiabook.com/?s=WIA-CICD&post_type=download
- **Issues**: https://github.com/WIA-Official/wia-standards-public/issues

---

© 2026 WIA / SmileStory Inc. · **弘益人間 (Benefit All Humanity)** 🌍
