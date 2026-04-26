# WIA-DIGITAL-ERASURE PHASE 4 — Integration Specification

**Standard:** WIA-DIGITAL-ERASURE
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This document defines integration concerns for WIA-DIGITAL-ERASURE
implementations: regulatory crosswalks (GDPR Article 17, CCPA §1798.105,
LGPD Article 18), evidence packaging via CycloneDX 1.5, and signed-
attestation submission to a competent authority.

References:
- GDPR (EU 2016/679) Article 17 — Right to erasure
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27018:2019 (PII protection in public clouds)
- ISO/IEC 27701:2019 (privacy information management)
- CycloneDX 1.5 (SBOM format), Sigstore (DSSE + Rekor)
- in-toto Attestation Framework 1.0

---

## §1 Integration topologies

Three integration topologies are common; each is supported by the
manifest defined in PHASE-1:

1. **Single-tenant edge** — one runtime per organization, no shared
   state. Used when the organization is the data controller and the
   subjects of erasure are the organization's customers.
2. **Multi-tenant gateway** — one shared runtime serves multiple data
   controllers via header-based isolation. Erasure events are
   propagated to all tenants whose data references the subject.
3. **Federated mesh** — multiple data controllers peer to one another
   and publish erasure manifests to a directory service. Each peer
   signs its own manifest; the directory service signs the aggregated
   index.

## §2 Evidence packaging

Implementations publish a CycloneDX 1.5 SBOM and an in-toto attestation
bundle for every release. The attestation bundle includes the signed
manifests for each protocol stage in PHASE-3 plus the build provenance
for the implementation itself.

Auditors verify the bundle offline using Sigstore's transparency log
entries (RFC 9162-style transparency pattern). Verification does not
require live connectivity to the originating organization.

## §3 Regulatory crosswalk

| Regime          | Article / Section          | Erasure scope            |
|-----------------|----------------------------|--------------------------|
| GDPR (EU)       | Article 17                 | All personal data        |
| CCPA (California) | §1798.105                  | Personal information     |
| LGPD (Brazil)   | Article 18                 | All personal data        |
| PIPL (China)    | Article 47                 | All personal data        |
| K-PIPA (Korea)  | Article 36                 | All personal data        |

Implementations declare the regulatory regimes they cover in their
profile manifest. The directory service uses these declarations to
route subject-erasure requests to the appropriate runtime.

## §4 Verification of erasure

Erasure is verified through three layers:

1. **System-level** — internal job logs prove the physical or
   cryptographic erasure occurred.
2. **Attestation-level** — a signed attestation declares the system-
   level verification outcome to downstream consumers.
3. **Audit-level** — an independent auditor reviews the attestation
   chain and the source-system logs to confirm consistency.

The protocol does not require live connectivity for audit-level
verification; the attestation chain is sufficient.

## §5 Backwards compatibility

Implementations MUST support the prior major version of the manifest
schema for at least 90 days after a new major version is published.
During the overlap window, both major versions are marked Stable in
the WIA registry.

弘益人間 — Hongik Ingan — Benefit All Humanity


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.
