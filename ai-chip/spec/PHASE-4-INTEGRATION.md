# WIA-ai-chip PHASE 4 — INTEGRATION Specification

**Standard:** WIA-ai-chip
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an AI-chip programme integrates
with the systems that surround it: MLCommons (for MLPerf
submission); ONNX / TVM / OpenXLA / OpenAI Triton compiler
toolchains; PCI-SIG and CXL Consortium reference
materials; OCP Open Accelerator Module specification;
hyperscale cloud provider fleet-management platforms;
data-centre BMC and IPMI infrastructure; export-control
regulators (US BIS, EU dual-use authorities, KR Strategic
Trade authorities); CVE Numbering Authorities; customer-
notification systems; and long-term archives.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- MLCommons MLPerf submission infrastructure
- ONNX
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 MLCommons Submission Integration

MLCommons operates the MLPerf submission infrastructure.
Integration carries the operator's MLCommons submitter
identifier, the per-suite submission template, the per-
result auditor-verification workflow, and MLCommons-side
publication channel that the operator's analytics
ingest. MLCommons release schedules (typically twice per
year for Inference and twice per year for Training)
drive the operator's benchmark-publication calendar.

## §2 Compiler Toolchain Integration

Compiler toolchains (TVM, OpenXLA, OpenAI Triton,
vendor-proprietary CUDA / ROCm / oneAPI compilers)
emit compiled-artefact lineage records (PHASE-1 §5).
Integration carries each toolchain's identifier, the
per-version reference, and the deterministic-build
attestation that allows the operator's downstream
consumers to reproduce the compilation given the input
artefact and the toolchain version.

## §3 PCI-SIG / CXL Consortium Integration

Host-attach interface conformance (PCIe 6.0, CXL 3.0)
follows PCI-SIG and CXL Consortium compliance programmes.
Integration carries the operator's PCI-SIG / CXL
membership reference, the per-product compliance test
report, and the consortium's published compliance-list
position.

## §4 OCP Open Accelerator Module Integration

Operators that adopt the OCP OAM form factor integrate
with OCP's specification registry. Integration carries the
operator's OCP membership reference, the per-product OAM
compliance attestation, and the OCP Open System Firmware
binding where the operator participates in OSF.

## §5 Customer Notification System Integration

Customer notifications for security disclosures (PHASE-3
§5) and EOL announcements (PHASE-3 §8) flow through the
operator's customer-notification system. Integration
carries the system's identifier, the per-customer
contact roster, and the per-notification-class delivery
SLA (immediate for critical security disclosures,
30-day-advance for major EOL announcements).

## §6 Cloud Provider Fleet-Management Integration

Hyperscale cloud providers operate accelerator fleets at
scale and consume per-chip telemetry, firmware-revision
events, and security disclosures. Integration carries
each cloud provider's identifier, the per-fleet-segment
telemetry feed, and the per-security-disclosure embargo-
share agreement (cloud providers often coordinate
embargoed disclosure visibility before public
publication so that they can pre-stage firmware
deployments).

## §7 Data-Centre BMC / IPMI Integration

Per-chip telemetry (PHASE-1 §6) ingests through the
operator's data-centre BMC (per Redfish / IPMI / vendor-
proprietary BMC APIs). Integration carries the BMC
vendor's identifier, the per-rack BMC mapping, and the
per-chip telemetry-tag mapping that translates BMC
sensor identifiers into the WIA telemetry schema.

## §8 Export-Control Regulator Integration

Export-control regulators (US BIS, EU national export-
control authorities, KR MOTIE Strategic Trade Bureau,
equivalent authorities) consume per-shipment licence
applications and post-shipment compliance reports.
Integration carries the regulator's identifier, the per-
shipment submission template, and the regulator's
audit-report intake.

## §9 CVE Numbering Authority Integration

For security disclosures (PHASE-1 §8), the operator
integrates with its CVE Numbering Authority (CNA). Where
the operator is its own CNA, the CNA endpoint is
internal; where the operator coordinates through MITRE
or another CNA, the integration carries the upstream
CNA's identifier and the per-disclosure CVE-assignment
workflow.

## §10 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed)
  programme.json               — programme record
  chips/                       — chip records and capability
                                  declarations
  mlperf-results/              — submitted and published
                                  MLPerf results
  compilation-lineages/        — compilation lineage records
                                  for the cited interval
  runtime-telemetry-summaries/ — telemetry summaries (raw
                                  telemetry in the
                                  operator's time-series
                                  database)
  firmware-revisions/          — firmware-application history
  security-disclosures/        — disclosure records
                                  (post-embargo only;
                                  pre-embargo records gated
                                  by the operator's CVD
                                  policy)
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is
signed by the operator's HTTP-message-signature key
(RFC 9421) and counter-signed by the operator's quality
manager when the package supports an external audit or
regulator submission.

## §11 Manifest and Signatures

Verification tools recompute file digests, compare to
the manifest, and reject the package on mismatch with
type `urn:wia:ai-chip:evidence-mismatch`.

## §12 well-known URI Discovery

A conformant operator exposes a discovery document at
`/.well-known/wia-ai-chip` that links to the API root,
the MLCommons submitter binding, the published quality
dossier, the chip-catalogue summary (Production-stage
chips), and the per-jurisdiction export-control binding.

## §13 Long-Term Archive Integration

Operators designate a long-term archive that holds chip
records, MLPerf results, and security disclosures
beyond the operator's primary retention horizon.
Quarterly deposits round-trip content-addresses; on
programme wind-down, remaining records transfer to the
archive with content-addresses preserved.

## §14 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (PCI-SIG /
CXL compliance, MLCommons submission verification, ISO/
IEC 24029 robustness assessment, ISO/IEC 27001
certification) to consumers of W3C Verifiable Credentials
MAY re-issue the attestations as Verifiable Credentials
under the Data Model 2.0 specification. Re-issuance is
optional; the canonical record remains the JSON
evidence-package manifest.

## §15 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds
with `Last-Event-ID` resume support. Subscribers that
disconnect during long telemetry-stream windows resume
from the last seen event identifier without losing
visibility of priority-1 events.

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with
prior-minor clients. Major revisions go through a
deprecation window of at least one full MLPerf release
cycle so that fleet-management and benchmark integrations
have time to migrate.

## §17 Cross-Standard Linkage

Operators that consume adjacent WIA standards (WIA-ai-
assistant for assistant-deployment chip lifecycle, WIA-
generative-ai for foundation-model deployment, WIA-data-
center for data-centre power and cooling integration,
WIA-iot-m2m for accelerator-attached edge devices) emit
cross-standard linkage records.

## §18 Reader Tooling

Operators MAY publish supplementary reader tools
(per-chip MLPerf trend dashboards, per-fleet thermal /
power dashboards, firmware-rollout coverage maps,
security-disclosure timeline visualisers) alongside the
canonical evidence package; the tools are non-normative.

## §19 Public Catalogue and Aggregator Feeds

Operators publish a public catalogue of registered
chips (Production stage), MLPerf verified-published
results, and post-embargo security disclosures through
an Atom or JSON Feed. Aggregator consumers subscribe to
compare accelerator behaviour across the AI-chip
industry.

## §20 Sustainability Reporting Platform Integration

Sustainability-disclosure platforms (CSRD-aligned, GRI,
SASB, ISSB IFRS S2) consume per-chip embodied- and
operating-emission attribution per PHASE-3 §14.
Integration carries the platform's identifier, the per-
disclosure-period methodology, and the per-product
embodied-emission accession reference.

## §21 Compliance Test Lab Integration

PCI-SIG and CXL Consortium host compliance test labs
that issue per-product compliance certificates.
Integration carries the lab's identifier, the per-product
compliance test report, and the consortium-side
publication endpoint.

## §22 Foundry and Packaging Vendor Integration

Operators integrate with foundries (TSMC, Samsung Foundry,
Intel Foundry Services, GlobalFoundries) and advanced-
packaging vendors (TSMC InFO / CoWoS, Intel Foveros,
Amkor / ASE / SPIL packaging providers) for per-wafer
and per-package provenance records (PHASE-1 §12).
Integration carries each vendor's identifier, the per-
lot record reference, and the per-event handoff
acknowledgement.

## §23 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with MLCommons (where the operator submits
MLPerf), at least one compiler toolchain provider, the
relevant host-attach consortium (PCI-SIG / CXL), the
operator's customer-notification system, the relevant
export-control regulator, the operator's CNA, and at
least one long-term archive, and has published at least
one externally citable evidence package.

Sunsetting an integration is announced via the well-
known discovery document at least 90 calendar days
before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-ai-chip
- **Last Updated:** 2026-04-28
