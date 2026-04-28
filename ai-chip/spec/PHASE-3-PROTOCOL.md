# WIA-ai-chip PHASE 3 — PROTOCOL Specification

**Standard:** WIA-ai-chip
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an AI-chip
programme: export-control compliance, MLCommons MLPerf
submission discipline, ISO/IEC 24029 robustness assessment,
firmware-signing discipline, coordinated security
disclosure, per-tenant isolation in shared deployments,
thermal and power envelope discipline, and end-of-life
management.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 24029-1:2021 / 24029-2:2023 (robustness)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27017:2015 (cloud-services information security)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 9457 (Problem Details)
- MLCommons MLPerf submission rules
- US BIS Export Administration Regulations (15 CFR Part 730
  et seq.) including the advanced-computing category for
  AI accelerators
- EU Regulation (EU) 2021/821 (dual-use export control)
- KR Strategic Items Export Control under the Foreign Trade
  Act
- PCI-SIG PCIe 6.0 / CXL Consortium CXL 3.0
- IEC 60825-1 (laser safety; relevant where on-chip optics
  intersect)
- OCP Open Accelerator Module specification

---

## §1 Export-Control Compliance

AI accelerators above the operating jurisdiction's
export-control thresholds (US BIS advanced computing
threshold, EU dual-use list, KR Strategic Items list) are
subject to per-export-event licensing. The operator's
export-control discipline:

- per-customer end-use screening at order intake (US
  Entity List, EU national lists, KR end-user screening);
- per-shipment export-control determination per the
  applicable jurisdiction;
- per-export licence retention per the regulator's rule;
- per-Foreign Direct Product Rule analysis for shipments
  that originate outside the rule's home jurisdiction
  but rely on US-origin technology.

Export-control non-compliance triggers the operator's
notification obligation to the regulator and freezes
affected shipments pending resolution.

## §2 MLCommons MLPerf Submission Discipline

MLPerf Inference v4 / Training v4 submissions follow the
MLCommons submission rules:

- per-suite eligible-system declaration (the system
  configuration that the result represents);
- per-workload reference implementation conformance (the
  submitter's implementation must produce results
  equivalent to the reference within the per-workload
  tolerance);
- per-result auditor verification (the submitter's
  results undergo MLCommons-side verification before
  publication);
- per-result publication-status transition through the
  MLCommons review cycle.

Withdrawn submissions remain addressable for historical
audit.

## §3 ISO/IEC 24029 Robustness Assessment

Per ISO/IEC 24029-1:2021 / 24029-2:2023, the operator
assesses neural-network robustness as part of the chip's
publication evidence:

- per-precision-mode robustness (FP32 vs FP16 vs FP8 vs
  INT8 — quantisation can introduce robustness
  regressions);
- per-workload robustness against the operator-selected
  perturbation suite (per ISO/IEC 24029-2 formal-methods
  guidance where applicable);
- per-vendor robustness-assessment report attached to
  the chip's publication record.

Robustness regressions across silicon steppings or
firmware revisions trigger the operator's customer-
notification workflow.

## §4 Firmware Signing Discipline

Firmware artefacts (PHASE-1 §7) are signed under the
vendor's firmware-signing root key (held in HSM, with
separation-of-duties between signing-key custodian and
firmware author). Discipline:

- per-revision signed manifest with image digest;
- per-revision target chip-class binding;
- per-revision rollback-possible attribute;
- per-revision MFA attestation on the signing event.

Chips refuse to load firmware whose signature fails
verification; refusal events emit customer-notification
through the operator's BMC telemetry channel.

## §5 Coordinated Security Disclosure

Per the operator's CVD policy:

- per-disclosure intake from finder (operator-internal
  red team, external researcher, customer report);
- per-disclosure embargo coordination with affected
  customers and downstream firmware-consuming vendors;
- per-disclosure CVE assignment through CNA pathway (the
  vendor is its own CNA or coordinates with MITRE);
- per-disclosure publication with CVSS v4 score,
  affected silicon revisions, and remediation firmware
  reference;
- per-disclosure post-publication monitoring for
  exploit-in-the-wild reports.

Disclosure embargo lengths follow the operator's published
CVD policy (typically 90 days from intake to publication
unless extended by mutual agreement with affected
parties).

## §6 Per-Tenant Isolation Discipline

Shared accelerator deployments (cloud accelerator-as-a-
service, multi-tenant on-prem) enforce per-tenant
isolation:

- per-tenant memory isolation (per the chip's MMU /
  address-space isolation features);
- per-tenant SLO commitment with per-tenant telemetry
  bound to the tenant identifier;
- per-tenant noisy-neighbour detection and the operator's
  remediation workflow when one tenant's workload
  affects another tenant's SLO;
- per-tenant side-channel mitigation per the relevant
  ISO/IEC 27017 controls.

## §7 Thermal and Power Envelope Discipline

Operators maintain per-chip thermal-and-power envelopes
per the chip's vendor-published TDP and operating-
temperature specification. Telemetry-driven envelope
management:

- per-chip thermal-throttling configuration when
  observed temperature approaches the thermal-design
  limit;
- per-chip power-cap enforcement when site power
  envelope is constrained;
- per-chip rack-level coordination with adjacent chips
  to avoid thermal-coupling cascades.

Persistent envelope excursions trigger the operator's
fleet-management remediation workflow (cooling capacity
review, workload re-distribution, hardware re-seat).

## §8 End-of-Life Management

EOL announcements (PHASE-1 §3 `endOfLifeDate`) trigger:

- per-customer migration assistance (the operator helps
  customers identify successor chip recommendations);
- per-customer firmware-support window (the operator
  commits to firmware support for a published period
  past EOL — typically 5-7 years for enterprise-class
  accelerators);
- per-customer EOL hardware-disposition workflow (return,
  trade-in, certified destruction with ESG-aligned
  e-waste handling).

Decommissioned chips with security-sensitive memory
(NPU SRAM holding model artefacts, host-memory regions
mapped via CXL) are wiped per the operator's data-
sanitisation policy before disposition.

## §9 Records Retention

Programme records — every chip / MLPerf result /
compilation lineage / runtime telemetry / firmware
revision / security disclosure / API audit log — retain
per the operating jurisdiction's records-retention rules
plus the relevant export-control retention obligations.
Security disclosures retain indefinitely.

## §10 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4).
Per-chip telemetry intervals carry the chip's BMC clock
skew vs the reference NTP source so that downstream
analytics can correlate per-chip events with site-wide
events.

## §11 Quality Dossier

The operator's quality dossier records the export-control
binding, the MLCommons submitter registration, the ISO/IEC
24029 robustness-assessment cadence, the firmware-signing
PKI, the CVD policy, the per-jurisdiction industrial-
safety binding, and the operator's incident history. The
dossier is reviewed at least annually by the operator's
quality manager.

## §12 Cross-Jurisdictional Operation

Multi-jurisdiction operators honour each jurisdiction's
export-control and security disclosure rules. Per-chip
governing-jurisdiction tagging supports downstream
regulator-specific reporting.

## §13 Numerics and Floating-Point Conformance

Per IEEE 754-2019 and the per-precision-mode definitions
in PHASE-1 §3, the operator's numerics-conformance
discipline:

- per-precision-mode IEEE 754-2019 conformance attestation
  for FP64, FP32, FP16 modes;
- per-precision-mode operator-published deviation
  statement for non-IEEE precision modes (TF32 truncated
  mantissa, BF16 Google-defined truncation, FP8 E5M2 /
  E4M3 OCP-aligned formats);
- per-rounding-mode coverage (round-to-nearest-even,
  round-toward-zero, round-toward-negative-infinity,
  round-toward-positive-infinity);
- per-special-value coverage (signed zero, infinities,
  NaN propagation).

Numerics regressions across silicon steppings or compiler
revisions are published as part of the chip's
publication-evidence chain.

## §14 Sustainability and Power-Per-Inference Discipline

Operators that publish sustainability disclosures
(corporate-sustainability reporting, customer-side
embodied-emission attribution) record:

- per-chip embodied-emission estimate (per the chip's
  fabrication LCA);
- per-chip operating-emission attribution (kWh per
  inference for the operator's typical workload mix);
- per-chip end-of-life embodied-recovery accounting
  (e-waste recycling rate, material-recovery rate).

Per-inference power telemetry (PHASE-1 §6 `powerWatts`)
feeds the operating-emission attribution; the operator's
sustainability report cites the methodology in force.

## §15 Provenance and Supply-Chain Integrity

Operators that handle accelerator silicon under
geopolitical-strategic-trade restrictions maintain per-
chip supply-chain integrity records:

- per-chip wafer-lot traceability from foundry through
  packaging to the assembled module;
- per-chip bonding records (for HBM stacks, COWOS or
  equivalent advanced-packaging records);
- per-chip burn-in test records (per the operator's
  reliability-acceptance test);
- per-chip serialised-with-provenance attestation that
  the operator can present to customers requesting
  supply-chain assurance.

Suspected-counterfeit detection (mismatched
fingerprints between physically-present chip and
expected provenance) triggers the operator's incident-
response workflow and freezes affected chips from
production deployment.

## §16 Microcode Update Discipline

Microcode revisions that modify execution behaviour
(numerics changes, security mitigations affecting
performance) follow the operator's microcode-update SOP:

- per-revision regression-test against the operator's
  workload portfolio;
- per-revision performance-delta report so that
  customers can plan for any performance impact;
- per-revision security-mitigation attribution if the
  revision addresses a CVE under embargo;
- per-revision rollout-staging plan honouring the
  customer-notification SLA.

## §17 Side-Channel and Adversarial-Robustness Discipline

AI accelerators are subject to side-channel risks (timing,
power, electromagnetic, cache-side-channel) that may leak
model weights, training data, or per-tenant inference
content. The operator's side-channel discipline:

- per-chip side-channel evaluation per the operator's
  threat model (per ETSI ISG SAI Group Reports series
  for AI-specific side-channel guidance);
- per-jurisdiction disclosure of known side-channel
  weaknesses with mitigations available;
- per-firmware-revision side-channel mitigation
  attribution where the revision addresses such a
  weakness.

Adversarial-robustness regressions (model behaviour under
adversarial inputs) intersect with PHASE-3 §3 ISO/IEC
24029 robustness assessment.

## §18 Conformance and Auditing

A programme conformant with WIA-ai-chip publishes its
export-control binding, its MLPerf submission record,
the chip catalogue, the firmware-signing PKI summary,
the security-disclosure register, and its incident
history at major and above, and answers an annual self-
assessment that maps each clause of this PHASE to the
operator's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-ai-chip
- **Last Updated:** 2026-04-28
