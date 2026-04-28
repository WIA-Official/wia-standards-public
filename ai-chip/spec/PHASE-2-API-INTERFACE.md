# WIA-ai-chip PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-ai-chip
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an
AI-chip vendor or accelerator-fleet operator exposes for
the records defined in PHASE-1. Consumers include MLPerf
benchmark consumers, model-deployment platforms, hyperscale
cloud providers ingesting per-tenant chip telemetry,
security researchers consuming disclosures, and the
operator's own audit and analytics platforms.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 / 6902 (JSON Pointer / Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context
- MLCommons MLPerf submission format
- ONNX
- PCI-SIG / CXL Consortium reference materials

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
operator. Versioning uses `/v1/` path segments. The
OpenAPI 3.1 document at `/v1/openapi.json` is canonical.

Per-inference runtime APIs (CUDA, ROCm, oneAPI runtime
calls) are documented by the vendor's runtime stack and
are not redefined here; this WIA facade is the metadata,
benchmarks, telemetry, firmware, and security disclosure
layer.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-ai-chip",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":           "/v1/programmes",
    "chips":                "/v1/chips",
    "mlperfResults":        "/v1/mlperf-results",
    "compilationLineages":  "/v1/compilation-lineages",
    "runtimeTelemetry":     "/v1/runtime-telemetry",
    "firmwareRevisions":    "/v1/firmware-revisions",
    "securityDisclosures":  "/v1/security-disclosures",
    "evidence":             "/v1/evidence",
    "openapi":              "/v1/openapi.json"
  }
}
```

## §3 Programme and Chip Lifecycle

```
POST   /v1/programmes              — register a programme
GET    /v1/programmes/{pid}        — retrieve programme
PATCH  /v1/programmes/{pid}/status — advance status
POST   /v1/programmes/{pid}/chips  — register a chip
PATCH  /v1/chips/{cid}/end-of-life — announce EOL
GET    /v1/chips/{cid}             — retrieve chip
```

Chip registrations cite the per-jurisdiction export-
control attestation (PHASE-1 §2 `exportControlBinding`);
submissions without the attestation for jurisdictions
that require it return `409` with type
`urn:wia:ai-chip:export-control-attestation-required`.

## §4 MLPerf Benchmark Submissions

```
POST   /v1/chips/{cid}/mlperf-results       — submit a
                                                benchmark
                                                result
PATCH  /v1/mlperf-results/{rid}/publication-status
                                              — update
                                                publication
                                                status
GET    /v1/mlperf-results/{rid}             — retrieve
                                                result
GET    /v1/chips/{cid}/mlperf-results?
       suite={s}&workload={w}                  — query
                                                results
```

Submissions cite the MLCommons submitter identifier;
submissions without a registered submitter return `409`
with type
`urn:wia:ai-chip:mlcommons-submitter-not-registered`.

## §5 Compilation Lineage

```
POST   /v1/programmes/{pid}/compilation-lineages
                                              — register a
                                                compilation
                                                lineage
GET    /v1/compilation-lineages/{lid}        — retrieve
                                                lineage
GET    /v1/chips/{cid}/compilation-lineages?
       toolchain={t}                           — query lineage
                                                by toolchain
```

Lineage submissions whose `compiledDigest` does not match
the actual compiled artefact return `422` with type
`urn:wia:ai-chip:compiled-digest-mismatch`.

## §6 Runtime Telemetry Ingest

```
POST   /v1/chips/{cid}/runtime-telemetry     — append a
                                                telemetry
                                                interval
POST   /v1/bulk/runtime-telemetry            — batched ingest
GET    /v1/runtime-telemetry/{tid}           — retrieve
                                                interval
GET    /v1/chips/{cid}/runtime-telemetry?
       from={t}&to={t}                          — query window
```

Telemetry intervals whose `intervalDurationS` is below
the operator's minimum aggregation window return `422`
with type
`urn:wia:ai-chip:telemetry-window-too-narrow` (the
minimum window protects per-tenant privacy by aggregating
beyond per-inference visibility).

## §7 Firmware Revisions

```
POST   /v1/chips/{cid}/firmware-revisions    — register a
                                                firmware
                                                application
GET    /v1/firmware-revisions/{fid}          — retrieve
                                                revision
GET    /v1/chips/{cid}/firmware-revisions?
       kind={k}                                 — query by
                                                firmware kind
```

Firmware applications whose artefact signature fails
verification return `409` with type
`urn:wia:ai-chip:firmware-signature-required`.

## §8 Security Disclosures

```
POST   /v1/programmes/{pid}/security-disclosures
                                              — register a
                                                disclosure
PATCH  /v1/security-disclosures/{sid}/cve-ref
                                              — attach CVE
                                                identifier
                                                once assigned
PATCH  /v1/security-disclosures/{sid}/remediation-firmware
                                              — attach
                                                remediation
                                                firmware
                                                reference
GET    /v1/security-disclosures/{sid}        — retrieve
                                                disclosure
```

Disclosures with `severityCvssV4 >= 9.0` automatically
escalate to the operator's customer-notification workflow
through the integration described in PHASE-4 §5.

## §9 Errors, Authentication, Caching, Audit

Errors: `application/problem+json` per RFC 9457. Defined
types include those above plus:

- `urn:wia:ai-chip:precision-mismatch`
- `urn:wia:ai-chip:thermal-budget-exceeded`
- `urn:wia:ai-chip:slo-breach-per-tenant`
- `urn:wia:ai-chip:evidence-mismatch`

Authentication: mutually-authenticated TLS for fleet-
operator, MLCommons, regulator, and partner consumers.
Public read-only endpoints (programme summary, MLPerf
verified-published results, security disclosures past
embargo) are reachable without a client certificate.
Caching: stable resources (verified MLPerf results,
applied firmware revisions, signed evidence packages)
cacheable with `Cache-Control: max-age=31536000,
immutable`. Audit logs carry `chipId`, `programmeId`,
`traceId`, the issuing client certificate's subject, and
the operator's clock skew vs the operating jurisdiction's
NTP service.

## §10 Streaming Subscription

Consumers subscribe via Server-Sent Events at:

- `/v1/programmes/{pid}/events` — programme-wide events
  (chip EOL announcements, MLPerf-result publication,
  security-disclosure embargo lift).
- `/v1/chips/{cid}/events` — chip-scoped events (thermal
  excursions, ECC fault rate spikes, firmware-application
  outcomes).

Subscribers reconnect via the `Last-Event-ID` header.

## §11 Bulk and Pagination

```
POST   /v1/bulk/runtime-telemetry           — batched
                                                telemetry
                                                ingest
POST   /v1/bulk/firmware-revisions          — batched
                                                firmware
                                                rollouts
GET    /v1/bulk/{operationId}               — operation
                                                status
```

Cursor-based pagination uses the `cursor` query parameter
and `Link` headers (RFC 8288).

## §12 Provenance and Aggregation

```
GET    /v1/provenance/{recordId}    — provenance entry for
                                       any PHASE-1 record
GET    /v1/aggregate/mlperf-improvement?suite=...&period=...
GET    /v1/aggregate/fault-rate-by-revision?period=...
GET    /v1/aggregate/firmware-rollout-coverage?period=...
```

## §13 Worked Example: Chip to MLPerf to Disclosure

1. Vendor registers chip with silicon stepping and
   precision-support set.
2. Vendor compiles MLPerf workload via TVM / OpenXLA
   and registers compilation lineage with input and
   compiled digests.
3. Vendor runs MLPerf Inference v4 and submits the
   result to MLCommons; status advances to
   `verified-published` on MLCommons verification.
4. Field operators ingest runtime telemetry; firmware
   revisions roll out to address ECC fault patterns
   discovered through telemetry trend analysis.
5. A coordinated security disclosure publishes with
   severity, affected silicon revisions, and remediation
   firmware reference; customer-notification workflow
   triggers under §8.

## §14 Fabric Topology and SLO Compliance Endpoints

```
POST   /v1/programmes/{pid}/fabric-topologies
                                          — register a fabric
                                            topology snapshot
GET    /v1/fabric-topologies/{ftid}       — retrieve topology
POST   /v1/chips/{cid}/slo-compliance     — register SLO
                                            compliance interval
GET    /v1/slo-compliance/{scid}          — retrieve SLO
                                            interval
GET    /v1/chips/{cid}/slo-compliance?
       tenant={t}&from={t1}&to={t2}         — query SLO history
```

SLO interval submissions whose `slaCompliancePct` is below
the operator's per-tenant breach threshold automatically
register a breach event for downstream remediation
workflow.

## §15 Supply-Chain Provenance and Microcode Endpoints

```
POST   /v1/chips/{cid}/supply-chain-provenance
                                          — register
                                            provenance record
GET    /v1/supply-chain-provenance/{spid} — retrieve
                                            provenance
POST   /v1/firmware-revisions/{fid}/microcode-update
                                          — attach microcode
                                            update detail
GET    /v1/microcode-updates/{muid}       — retrieve update
                                            detail
```

Provenance submissions whose `serialisedFingerprintRef`
does not match the chip's runtime-presented PUF
fingerprint return `409 Conflict` with type
`urn:wia:ai-chip:provenance-fingerprint-mismatch` and
the chip is quarantined pending counterfeit
investigation.

## §16 Conformance

A conformant server passes the test vectors published
under `tests/phase-vectors/phase-2-api-interface/`, emits
an OpenAPI 3.1 document, signs evidence packages per RFC
9421, and rejects MLPerf submissions citing a non-
registered MLCommons submitter.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-ai-chip
- **Last Updated:** 2026-04-28
