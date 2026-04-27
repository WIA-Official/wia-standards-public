# WIA-industrial-robot PHASE 4 — INTEGRATION Specification

**Standard:** WIA-industrial-robot
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited industrial-robot
programme integrates with the systems that surround it: robot-
vendor service portals; system-integrator delivery pipelines;
MES platforms; APM and fleet-monitoring services; CMMS;
occupational-safety authorities that consume incident reports;
operator's cybersecurity operations centre; long-term archives;
and the regulators and certifying bodies that audit deployments.
It also defines the evidence-package format that bundles a
cell's complete record set for external audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO 8601 (date and time)
- ISO 10218-1 / ISO 10218-2 (robot safety)
- ISO/TS 15066 (collaborative robots)
- ISO 9283 (robot performance)
- OPC UA Robotics Companion Specification
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Robot-Vendor Service Portal Integration

Robot vendors operate service portals that consume firmware-
update and maintenance events. The integration is bidirectional:
vendors publish firmware releases (signed by the vendor's
release key) and the operator's CMMS publishes maintenance
events back to the vendor for warranty and reliability
analytics.

Submissions whose firmware signing key is not in the vendor's
registered key chain return `urn:wia:industrial-robot:vendor-
key-unrecognised` from the firmware update endpoint.

## §2 System-Integrator Delivery Pipeline Integration

System integrators that deliver new cells consume the
programme's commissioning-pack format (kinematic descriptors,
work-cell layout, safety configuration template) and emit
commissioning evidence (ISO 9283 baseline test results,
ISO/TS 15066 collaborative-mode validation results) back through
the API. Integrator client certificates are scoped to the cells
they have commissioned.

## §3 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  work-cell.json               — cell record
  robots/                      — robot records and kinematic
                                  descriptors
  tasks/                       — programmed tasks and risk-
                                  assessment artefacts
  motion/                      — motion-sample summaries and
                                  reconstruction-window references
  safety-config/               — safety-configuration history
  safety-incidents/            — incidents and reconstruction
                                  motion windows
  maintenance/                 — maintenance records and ISO 9283
                                  re-test results
  cyber-posture/               — IEC 62443 zone definitions and
                                  controls
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is signed by
the operating programme and counter-signed by the integrator
when the package covers an integrator-delivered cell.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:industrial-robot:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-industrial-robot` that links to the API root,
the public operator and integrator accreditation references,
the published quality dossier, the safety-configuration
publication policy, and the catalogue of cells operated.

## §6 MES Integration

MES platforms schedule robot tasks and consume task-completion
events. The integration is mutually authenticated; MES client
certificates are scoped to the cells the MES manages.

## §7 APM and Fleet-Monitoring Integration

APM and fleet-monitoring services consume robot telemetry and
maintenance records to compute reliability KPIs across the
operator's robot fleet. The integration is read-only; KPIs are
computed by the APM and not mirrored back into the WIA records.

## §8 CMMS Integration

The CMMS holds the work-order register and the maintenance
scheduler. The CMMS integration consumes maintenance records
and emits new work orders triggered by predictive-condition-
based or preventive-scheduled cadence.

## §9 Cybersecurity Operations Centre Integration

The operating organisation's CSIRT consumes priority-1 and
priority-2 security-category events through the streaming
subscription. CSIRT response actions are appended to the event
record so that post-incident review can reconstruct the response
timeline.

## §10 Occupational-Safety Authority Integration

National and regional occupational-safety authorities receive
notifications when safety incidents of `major-injury` severity
or above occur. The integration record carries the authority's
intake endpoint, the notification format the authority requires,
and the maximum notification latency.

## §11 Long-Term Archive Integration

Programmes designate a long-term archive that holds operational
records beyond programme wind-down. Quarterly deposits round-
trip content-addresses; on wind-down, remaining records transfer
to the archive with content-addresses preserved.

## §12 Worked Example: Citation Resolution for an Incident Report

A reader encounters a peer-reviewed publication or industry case
study that cites a robotic-cell safety incident. The reader's
tool resolves the citation by:

1. Parsing the citation to extract the cell ID and manifest
   digest.
2. Fetching the discovery document for the operating programme.
3. Resolving the manifest URL and verifying the manifest
   signatures.
4. Recomputing the manifest digest and comparing it to the
   pinned digest; aborting on mismatch.
5. Retrieving the package; recomputing per-file digests;
   surfacing the resolved evidence (incident record,
   reconstruction motion window, root-cause investigation) to
   the reader.

A conformant tool completes this flow without further input.

## §13 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (industrial-iot
for shared OPC UA infrastructure, predictive-maintenance for
condition-based maintenance, manufacturing-automation for cell-
level orchestration) emit cross-standard linkage records.

## §14 Reader Tooling

Programmes MAY publish supplementary reader hints (visual cell-
layout maps, motion-trajectory replays, repeatability rollup
charts) alongside the canonical evidence package. Reader tools
are non-normative.

## §15 Public Catalogue

Programmes that publish a public catalogue of operating cells
emit an Atom or JSON Feed listing cells with their evidence-
package manifest digests, the ISO 10218 collaborative class,
and the safety-incident history at `severity` ≥ `near-miss`. The
feed does not carry motion telemetry; it is intended for
industry transparency.

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window
of at least one full ISO 9283 re-test cycle.

## §17 Migration from Pre-Standard Records

Programmes that operated before WIA-industrial-robot reached
version 1.0 MAY migrate historical cells by emitting synthetic
cell records with a `legacyImport` flag. Synthetic cells are
accepted by the public catalogue but are not eligible for
evidence-package generation without contemporaneous re-validation
of the safety configuration.

## §18 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose attestations (ISO 10218 type-
certification compliance, ISO/TS 15066 collaborative-mode
commissioning, ISO 9283 baseline-test pass) to consumers of W3C
Verifiable Credentials MAY re-issue the attestations as
Verifiable Credentials under the Data Model 2.0 specification.
Re-issuance is optional; the canonical record remains the JSON
evidence-package manifest.

## §19 Predictive-Maintenance Service Integration

Predictive-maintenance services consume motion-sample summaries
(joint-torque signatures, vibration spectra, harmonic-drive
condition signatures) and emit failure-imminent alerts. The
integration is mediated by APM (§7) but predictive vendors may
also subscribe directly via the streaming endpoint with operator-
issued client certificates scoped to the targeted robot models.

## §20 Streaming Heartbeat

SSE subscribers (§13) receive a heartbeat every 30 seconds;
replays support `Last-Event-ID` headers (W3C EventSource
semantics). Subscribers that disconnect resume from the last
seen event identifier without losing visibility of priority-1
incident events.

## §21 Operator Training Platform Integration

Operator training platforms (vendor-provided e-learning, third-
party VR-based training, in-house competency frameworks) emit
training-completion events that the operating programme consumes
to maintain the per-cell authorised-operator list (PHASE-3 §19).
The integration carries the training platform's identifier, the
competency mapping from training modules to operator scopes, and
the expiry policy for training certifications.

## §22 ISO 9283 Re-Test Laboratory Integration

ISO 9283 baseline and re-test campaigns are typically performed
by accredited testing laboratories (in-house metrology
laboratory or external ISO/IEC 17025-accredited test house).
The integration carries the laboratory's identifier, the
accreditation reference, the re-test cadence agreed with the
operator, and the per-test artefact (RP / PR / AT / path-
accuracy / velocity-accuracy report). The operating programme
records each re-test result against the robot record so that
performance trend analyses can resolve to specific re-test
events.

## §23 Reader Tooling for Safety Reviewers

Safety reviewers, occupational-safety inspectors, and certifying-
body auditors benefit from visualisation tools that render
incident reconstruction motion windows, safety-zone violations,
and ISO 9283 trend charts. Programmes MAY publish reader tools
alongside the canonical evidence package; the tools are non-
normative and remain under operator control.

## §24 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with at least one robot vendor's service portal, at least one
system integrator, at least one MES, at least one APM, the
operator's CMMS, the operator's CSIRT, and the relevant
occupational-safety authority, and has published at least one
externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-industrial-robot
- **Last Updated:** 2026-04-27
