# WIA-CICD — PHASE 1: Data Format

> Open standard initiative · Continuous Integration / Continuous Delivery
>
> 弘益人間 (Benefit All Humanity)

This phase defines the canonical data shapes consumed and produced by a WIA-CICD compliant pipeline. The simulator at `/cicd/simulator/index.html` is the **source of truth** for ENUMs and thresholds; this document mirrors them in JSON Schema form.

---

## 1. Pipeline Definition (YAML)

A pipeline is a directed acyclic graph (DAG) of jobs. Each job is composed of stages and steps. Cross-references between jobs are declared via `needs:`.

```yaml
pipeline:
  name: wia-cicd-reference
  trigger: PUSH                 # ENUM: PUSH | PR | TAG | MANUAL | SCHEDULE
  nodes:                        # five logical DAG nodes
    - id: SOURCE
      type: source
    - id: BUILD
      type: build
      needs: [SOURCE]
      cache:
        local:  bazel-disk
        remote: bazel-grpc://cache.example
        execution: bazel-rbe://rbe.example
    - id: TEST_GATE
      type: test
      needs: [BUILD]
      slo:
        unit_ms:        100     # UNIT_SLO_MS
        integration_s:  5       # INTEGRATION_SLO_S
        e2e_s:          60      # E2E_SLO_S
    - id: SECURITY_GATE
      type: security
      needs: [TEST_GATE]
      gates: [SAST, SCA, DAST, SECRETS, CONTAINER, IAC]
      policy: ENFORCE           # ENUM: AUDIT | ENFORCE
    - id: CD_HANDOFF
      type: deliver
      needs: [SECURITY_GATE]
      strategy: CANARY          # ENUM: ROLLING | BLUE_GREEN | CANARY | PROGRESSIVE
      traffic_curve: [5, 25, 50, 100]
```

**ENUM provenance.** All capitalised identifiers above are exported by the simulator's JS constants (`PIPELINE_TRIGGERS`, `PIPELINE_NODES`, `SECURITY_GATES`, `POLICY_MODES`, `DEPLOY_STRATEGIES`). Implementations MUST use the verbatim spelling in both English and Korean editions.

---

## 2. SLSA in-toto Attestation

WIA-CICD adopts SLSA v1.0 provenance attestations encoded as in-toto v1.0 statements. Minimum target is **SLSA L3** (signed, hermetic, isolated builder).

```json
{
  "_type": "https://in-toto.io/Statement/v1",
  "subject": [
    { "name": "pkg:oci/example/app",
      "digest": { "sha256": "<sha256>" } }
  ],
  "predicateType": "https://slsa.dev/provenance/v1",
  "predicate": {
    "buildDefinition": {
      "buildType": "https://wia.standards/cicd/build/v1",
      "externalParameters": { "trigger": "PUSH", "ref": "refs/heads/main" },
      "internalParameters": { "builder": "wia-runner@v1" },
      "resolvedDependencies": [
        { "uri": "git+https://example/repo@<sha>",
          "digest": { "sha1": "<sha>" } }
      ]
    },
    "runDetails": {
      "builder": { "id": "https://wia.standards/builder/L3" },
      "metadata": { "invocationId": "<uuid>", "startedOn": "2026-05-05T00:00:00Z", "finishedOn": "2026-05-05T00:18:31Z" },
      "byproducts": [ { "name": "sbom.cdx.json", "digest": { "sha256": "<sha>" } } ]
    }
  }
}
```

The provenance MUST be signed (Sigstore Cosign keyless or KMS-backed key). See PHASE-3 for the wire protocol.

---

## 3. SBOM (CycloneDX / SPDX)

Every artifact ingested by `CD_HANDOFF` MUST be accompanied by a Software Bill of Materials in one of three formats:

| Format       | MIME                                      | Spec       |
|--------------|-------------------------------------------|------------|
| `CYCLONEDX`  | `application/vnd.cyclonedx+json`          | CycloneDX 1.5 |
| `SPDX`       | `text/spdx`                               | SPDX 2.3 / 3.0 tag-value |
| `SPDX_JSON`  | `application/spdx+json`                   | SPDX 2.3 / 3.0 JSON |

Minimum CycloneDX 1.5 example:

```json
{
  "bomFormat": "CycloneDX",
  "specVersion": "1.5",
  "serialNumber": "urn:uuid:<uuid>",
  "version": 1,
  "metadata": { "timestamp": "2026-05-05T00:18:31Z",
                "tools": [{ "vendor": "WIA", "name": "wia-cicd-cli", "version": "1.0.0" }] },
  "components": [
    { "type": "library",
      "name": "example-lib",
      "version": "1.2.3",
      "purl": "pkg:npm/example-lib@1.2.3" }
  ]
}
```

---

## 4. DORA Four Keys + Rework Rate

WIA-CICD extends the canonical DORA four-key metric set with a fifth axis (Rework Rate):

| Key              | Unit          | Elite threshold (simulator) |
|------------------|---------------|-----------------------------|
| Lead Time        | hours         | ≤ 1 (`DORA_ELITE_LEAD_TIME_HR`) |
| Deploy Frequency | per day       | ≥ 1 (on-demand)             |
| Change Failure   | ratio (0–1)   | ≤ 0.05 (`DORA_ELITE_FAILURE_RATE`) |
| MTTR             | minutes       | ≤ 5 (`ROLLBACK_MAX_MIN`)    |
| Rework Rate      | ratio (0–1)   | ≤ 0.05 (`REWORK_RATE_THRESHOLD`) |

Schema: `schemas/dora-metrics.schema.json`.

---

## 5. Wire-Compatible Signal (`wia-cicd-signal-v1`)

A single JSON envelope used to forward DORA + cache + security + SLO signals to an aggregator (compatible with OTLP/JSON):

```json
{
  "signal":  "wia-cicd-signal-v1",
  "ts":      "2026-05-05T00:18:31Z",
  "source":  { "pipeline": "wia-cicd-reference", "node": "TEST_GATE" },
  "dora":    { "lead_time_hr": 0.6, "failure_rate": 0.03, "mttr_min": 4, "rework_rate": 0.04 },
  "cache":   { "local": 0.82, "remote": 0.65, "execution": 0.45, "overall": 0.945 },
  "security":{ "policy": "ENFORCE", "fp_rate": 0.04, "critical": 0 },
  "slo":     { "tier": "STD", "target": 0.999, "burned": 0.18 }
}
```

Schema: `schemas/wia-cicd-signal-v1.schema.json`.

---

## 6. Companion Schemas

| File                                    | Purpose |
|-----------------------------------------|---------|
| `schemas/pipeline.schema.json`          | Pipeline YAML (validated as JSON) |
| `schemas/provenance.schema.json`        | SLSA in-toto attestation |
| `schemas/dora-metrics.schema.json`      | DORA + Rework Rate envelope |
| `schemas/wia-cicd-signal-v1.schema.json`| Wire signal envelope |

All schemas use Draft 2020-12 (`$schema: https://json-schema.org/draft/2020-12/schema`).

---

© 2026 WIA · MIT License · 弘益人間
