# WIA-fusion-energy PHASE 4 — INTEGRATION Specification

**Standard:** WIA-fusion-energy
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a fusion-energy facility
integrates with the systems that surround it: the
operating jurisdiction's nuclear-or-radiation-safety
regulator (US NRC, UK ONR, EU national regulators per
Council Directive 2009/71/Euratom + 2013/59/Euratom,
JP NRA, KR NSSC); the IAEA where the operating
jurisdiction reports voluntary fusion-safety
information and where IAEA additional-protocol
arrangements apply to tritium accountancy; the
EURATOM safeguards inspectorate where the operating
jurisdiction is an EU Member State; the operating
jurisdiction's radioactive-waste regulator; the host
site's safety committee; the operating jurisdiction's
emergency-management agency; the contracted ASME
NQA-1 quality-assurance auditor (where ASME is
adopted); the contracted IEC 61508 functional-safety
assessor; the plasma-physics community partners
(IAEA Fusion Energy Conference, the IEA Fusion
Implementing Agreement participants, the operating
jurisdiction's domestic plasma-physics community);
and long-term archives.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17021-1:2015 (management-system audit and
  certification)
- ISO 8601 (date and time)
- ICRP Publication 103 (recommendations on radiological
  protection)
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Operating-Jurisdiction Regulator Integration

The operating jurisdiction's regulator is the
authoritative consumer of safety-case submissions,
operating-phase advances, and reportable-event
notifications. Integration carries the regulator's
identifier, the per-submission cooperation workflow,
the per-inspection on-site-cooperation workflow, the
per-enforcement-action response workflow, and the
regulator's response SLA.

For US-jurisdiction facilities, integration with the
NRC follows the NRC's risk-informed performance-based
fusion regulatory framework. For UK-jurisdiction
facilities, integration with ONR follows ONR's fusion-
safety pathway. For EU-Member-State-jurisdiction
facilities, integration with the operating Member
State's nuclear regulator follows the Member State's
transposition of Council Directive 2009/71/Euratom +
2013/59/Euratom. For JP-jurisdiction facilities,
integration with the NRA follows the NRA's adopted
fusion guidance. For KR-jurisdiction facilities,
integration with the NSSC follows the NSSC's adopted
fusion guidance.

## §2 IAEA Integration

The IAEA integration spans:

- voluntary fusion-safety information reporting where
  the operating jurisdiction participates in the
  IAEA Fusion Safety project information exchange;
- IAEA Specific Safety Guides SSG-77 / SSG-78 /
  SSG-79 reference adoption tracking;
- IAEA INSAG and TECDOC consumption (the operator's
  safety-case authoring team consumes IAEA
  publications for community-recognised baselines);
- IAEA safeguards integration where the operating
  jurisdiction's additional-protocol arrangements
  reach the facility's tritium inventory.

## §3 EURATOM Safeguards Inspectorate Integration

For EU-Member-State facilities the EURATOM safeguards
inspectorate consumes tritium-accountancy submissions
under the operating jurisdiction's safeguards regime.
Integration carries the inspectorate's identifier,
the per-period accountancy submission, the per-on-
site-inspection cooperation workflow, and the per-
period reconciliation workflow.

## §4 Radioactive-Waste Regulator Integration

The operating jurisdiction's radioactive-waste
regulator consumes the facility's activated-component
inventory and waste-route declarations. Integration
carries the regulator's identifier, the per-route
acceptance workflow, the per-shipment chain-of-
custody envelope, and the per-cycle reconciliation
between the facility's activated-component inventory
and the regulator's record.

## §5 Host-Site Safety Committee Integration

The host site's safety committee (typically composed
of senior facility staff plus independent members) is
the internal review body for safety-case revisions,
operating-experience feedback, and reportable-event
post-mortems. Integration carries the committee's
identifier, the per-meeting cooperation record, the
per-decision recommendation register, and the
operator's response to committee recommendations.

## §6 Emergency-Management Agency Integration

The operating jurisdiction's emergency-management
agency (the operating jurisdiction's civil-protection
agency that would respond off-site to a radiation
emergency) integrates with the facility's emergency
plan. Integration carries the agency's identifier,
the per-cycle exercise schedule, the per-notification
classification map (the facility's classification
mapped to the agency's emergency-classification
levels), and the per-public-information protocol.

## §7 ASME NQA-1 Quality-Assurance Auditor Integration

Where the operating jurisdiction adopts ASME NQA-1
(typically US-jurisdiction facilities and some non-US
facilities that voluntarily adopt NQA-1), the
contracted NQA-1 auditor consumes the facility's
quality-assurance dossier. Integration carries the
auditor's identifier, the per-cycle audit-report
archive, the per-finding remediation tracker, and
the per-cycle scope statement.

## §8 IEC 61508 Functional-Safety Assessor Integration

The contracted IEC 61508 functional-safety assessor
consumes the protection-system functional-safety
dossier and emits SIL claim assessments. Integration
carries the assessor's identifier, the per-claim
assessment-report archive, and the per-modification
re-assessment workflow.

## §9 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed)
  programme.json               — programme record
  safety-cases/                — safety-case revisions
                                  (current and superseded)
  tritium-inventory/           — tritium-accountancy
                                  history (gated to
                                  regulator and EURATOM
                                  / IAEA inspectorate)
  postulated-events/           — initiating-event
                                  register
  safety-classified-components/
                               — qualification records
  operating-limits/            — operating-limit register
  plasma-operations/           — discharge metadata (
                                  shareable subset for
                                  community partners)
  reportable-events/           — reportable events with
                                  root-cause narratives
  decommissioning/             — decommissioning records
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is
signed by the facility's HTTP-message-signature key
(RFC 9421) and counter-signed by the facility safety
manager when the package supports a regulator
submission.

## §10 Manifest and Signatures

Verification tools recompute file digests, compare to
the manifest, and reject the package on mismatch with
type `urn:wia:fusion-energy:evidence-mismatch`.
Tritium-inventory bundles for regulator submission
carry the per-period mass-balance reconciliation;
public-shareable bundles do not.

## §11 well-known URI Discovery

A conformant facility exposes a discovery document at
`/.well-known/wia-fusion-energy` that links to the API
root, the operating jurisdiction's regulator binding,
the IAEA voluntary-reporting binding, the EURATOM /
domestic-safeguards binding (where applicable), and
the host-site safety committee binding. End-public
disclosure (e.g. emergency-preparedness public
information) flows through the operating jurisdiction's
emergency-management agency channels, not through the
discovery document.

## §12 Long-Term Archive Integration

Facilities designate a long-term archive that holds
safety-case revisions, regulator-correspondence
history, tritium-accountancy history, reportable-event
records, and decommissioning records beyond the
facility's primary retention horizon. Quarterly
deposits round-trip content-addresses; on facility
decommissioning, remaining records transfer to the
archive with content-addresses preserved subject to
the operating jurisdiction's records-retention rules
(nuclear safety records typically retain for facility
life plus thirty years).

## §13 Verifiable-Credential Re-Issuance (optional)

Facilities that wish to expose attestations (regulator-
approved safety case, ASME NQA-1 audit clearance, IEC
61508 functional-safety assessment, ISO/IEC 17021
quality-management-system certification) to consumers
of W3C Verifiable Credentials MAY re-issue the
attestations as Verifiable Credentials under the Data
Model 2.0 specification. Re-issuance is optional; the
canonical record remains the JSON evidence-package
manifest.

## §14 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds
with `Last-Event-ID` resume support. Subscribers that
disconnect during regulator-correspondence windows or
during plasma-campaign windows resume from the last
seen event identifier without losing visibility of
priority-1 events (safety case approved, reportable
event detected, regulator notification deadline
elapsed, tritium mass-balance reconciliation flagged).

## §15 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible
with prior-minor clients. Major revisions go through
a deprecation window of at least one full regulator-
review cycle so that regulator, IAEA, EURATOM /
domestic-safeguards, host-site-safety-committee, NQA-1
auditor, and IEC 61508 assessor integrations have time
to migrate.

## §16 Cross-Standard Linkage

Facilities that consume adjacent WIA standards (WIA-
fuel-cell for hydrogen-handling discipline that
intersects with the deuterium-fuel-handling
discipline at facilities that produce or consume
deuterium without tritium, WIA-energy-storage for the
energy-storage overlay where the facility's pulse-
power supply is itself a regulated energy-storage
asset, WIA-distributed-energy for the grid-
interconnection overlay where the facility supplies
auxiliary power to the grid) emit cross-standard
linkage records.

## §17 Reader Tooling

Facilities MAY publish supplementary reader tools
(per-shot plasma-parameter dashboards, per-period
tritium-inventory mass-balance trackers, per-system
protection-function status dashboards, per-cycle
operating-experience catalogues) alongside the
canonical evidence package; the tools are non-
normative.

## §18 Public Catalogue Feed

Facilities publish a public catalogue feed listing the
in-force safety case (public summary version where
the operating jurisdiction publishes one), the
operating jurisdiction's regulator binding, the
operating-phase declaration, the aggregate
reportable-event count, and the aggregate operating-
hour count. The feed enables regulator and civil-
society discovery of the facility's operating posture.

## §19 IEA Fusion Implementing Agreement Integration

For facilities operated by Implementing Agreement
participants, integration carries the participant's
identifier, the per-cycle technology-collaboration
report submission, and the per-meeting Implementing
Agreement working-group cooperation record.

## §20 Plasma-Physics Community Partner Integration

The plasma-physics community (the IAEA Fusion Energy
Conference series, the EFTC, the operating
jurisdiction's domestic plasma-physics conferences,
and the operating jurisdiction's plasma-physics
research consortia) consumes shareable plasma-
operation datasets for independent analysis.
Integration carries the partner's identifier, the
per-research-purpose data-sharing agreement, the
shareable subset of plasma-operation records (with
tritium-cycle and safety-classified data redacted),
and the facility's publication of partner-attribution
in any derivative research output.

## §21 Public Catalogue Aggregator Integration

Civil-society researchers, energy-policy research
organisations, and academic-research consortia consume
aggregate operating-hour and reportable-event
statistics for independent analysis. Integration
carries the consumer's identifier, the per-research-
purpose data-access agreement, and the facility's
publication of consumer-attribution in any derivative
research output. Per-discharge or per-component
records are NOT shared through this channel; only
aggregate statistics are.

## §22 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with the operating jurisdiction's
regulator, the operating jurisdiction's emergency-
management agency, at least one ASME NQA-1 auditor
(where ASME is adopted) or equivalent national-
standard auditor, at least one IEC 61508 functional-
safety assessor, the operating jurisdiction's
radioactive-waste regulator (where activated-material
disposition is in scope), and at least one long-term
archive, and has published at least one externally
citable evidence package.

Sunsetting an integration is announced via the well-
known discovery document at least 90 calendar days
before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-fusion-energy
- **Last Updated:** 2026-04-28
