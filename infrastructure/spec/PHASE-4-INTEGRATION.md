# WIA-infrastructure PHASE 4 — INTEGRATION Specification

**Standard:** WIA-infrastructure
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited civil-infrastructure asset
programme integrates with the systems that surround it: the
operator's BIM Common Data Environment (CDE), inspection
contractors' field-data-collection platforms, NDT laboratories'
test-result systems, condition-rating analytics services,
maintenance-management systems (CMMS / EAM), pavement-management
systems, bridge-management systems, hydraulic-management systems,
SCADA platforms for monitored assets, regulators' notification
intakes, financial systems for capital-works budgeting, GIS portals
for public-facing maps, long-term archives, and citation tools that
resolve published condition reports to evidence packages. It also
defines the evidence-package format that bundles an asset's
complete record set for external audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO 19650-2 (BIM information delivery)
- ISO 16739 (IFC)
- ISO 23386 / 23387 (data templates)
- ISO 55001 (asset-management-system requirements)
- ISO 8601 (date and time)
- W3C Verifiable Credentials Data Model 2.0 (optional)
- OGC API — Features (geospatial integration)

---

## §1 BIM Common Data Environment Integration

The operator's CDE holds the federation file delivered at hand-
over (PHASE-1 §8) and the design and construction working
documents that produced the file. The integration is bidirectional:
the CDE publishes the federation-file content-address to the WIA
records, and the WIA records emit asset-record updates back to
the CDE so that long-term BIM consumers see the in-service state
of the asset alongside the design state.

Integration submissions whose CDE federation file is not ISO
16739 IFC4-compliant return
`urn:wia:infrastructure:federation-file-invalid` from the hand-
over endpoint.

## §2 Inspection-Contractor Field-Platform Integration

Inspection contractors operate field-data-collection platforms
(rugged tablets, handheld imagers, drone-borne sensor packages).
The integration carries the inspector's institutional identifier,
the device identifier, the calibration record reference for the
sensors used, and the per-observation timestamps. Field-platform
sessions emit inspection-record drafts that the inspector signs
before submission to the API.

## §3 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  asset.json                   — asset record
  components/                  — component decomposition records
  spatial-identities/          — versioned spatial identities
  inspections/                 — inspection records and signed
                                  observations
  condition-assessments/       — condition assessment records
  work-orders/                 — work-order register
  hand-over/                   — federation-file content-address
                                  and surveys
  ndt/                         — NDT laboratory result references
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is signed by the
operating programme and counter-signed by the certifying inspector
when the package supports a regulatory submission.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the manifest,
and reject the package on mismatch with type
`urn:wia:infrastructure:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-infrastructure` that links to the API root, the
ISO 55001 certificate reference, the published quality-management
dossier, the procedure register, and the catalogue of assets
operated.

## §6 Long-Term Archive Integration

Programmes designate a long-term archive that holds asset records
beyond programme wind-down. Quarterly deposits round-trip content-
addresses; on wind-down, remaining records transfer to the archive
with content-addresses preserved. Recognised archives include
national archives, professional-society archives (e.g. ASCE
Library), and university research archives that operate under
ISO 16363 or equivalent trustworthy-digital-repository
accreditation.

## §7 NDT Laboratory Integration

ISO/IEC 17025-accredited NDT laboratories emit signed test results
(ultrasonic thickness measurements, half-cell potential surveys,
GPR scans). The integration carries the laboratory's accreditation
reference, the test-method reference, the calibration certificate
references for the sensors used, and the signed result file. NDT
results bind to the inspection record that requested them.

## §8 Condition-Rating Analytics Integration

Condition-rating analytics services consume inspections and emit
component condition ratings. Analytics services declare the
rating-scheme version they implement and the algorithm provenance
(reference papers, method validation reports). The operator
verifies the analytics service's outputs against a sample of
inspections rated by qualified human inspectors before relying on
the service for production rating.

## §9 CMMS / EAM Integration

The operator's computerised-maintenance-management system or
enterprise-asset-management system holds the work-order register
and the maintenance scheduler. The CMMS integration consumes work-
order records and emits new work orders triggered by condition-
based or preventive-scheduled cadence.

## §10 SCADA Integration for Monitored Assets

Hydraulic structures, urban transmission infrastructure, and
selected smart bridges and tunnels are instrumented with SCADA
sensors (water levels, pressures, vibration, temperature, strain).
The SCADA integration emits monitored telemetry that the operator
uses as condition-assessment inputs. Telemetry transport follows
the operator's industrial-security broker (see §13).

## §11 Pavement-Management-System Integration

Roadway operators that maintain a pavement-management system
emit pavement-condition records (International Roughness Index,
rutting depth, pavement-distress index per AASHTO PP 67, FHWA
Highway Performance Monitoring System format). The integration is
bidirectional: pavement records bind to the WIA road-segment
asset records, and the WIA condition assessments feed the
pavement-management system's maintenance prioritisation.

## §12 Bridge-Management-System Integration

Bridge operators that maintain a bridge-management system (e.g.
AASHTOWare BrM, regional Pontis-derived systems, in-house
systems) emit per-element condition records aligned to AASHTO
Manual for Bridge Element Inspection. The integration is
bidirectional: BMS records bind to the WIA bridge asset records,
and the WIA condition assessments feed the BMS's deterioration
modelling.

## §13 Cybersecurity for SCADA-Monitored Assets

SCADA-monitored assets operate under the IEC 62443 zone
classification that the operator declares for the asset (PHASE-3
§12). Telemetry ingress restricts to the operator's industrial-
security broker; direct external access from the public internet
is not permitted under this PHASE.

## §14 GIS Portal Integration

Public-facing GIS portals consume the asset register, the spatial
identity, and the published condition indices. The integration
emits OGC API — Features collections so that downstream GIS
clients (QGIS, ArcGIS, Mapbox-based viewers) can render the asset
register without bespoke client adapters.

## §15 Regulator Notification Integration

Regulators whose statutes require notification of regulatory-
reportable events (cycle missed, post-event inspection, dam-
safety incident) receive notifications via the regulator's
intake endpoint. The integration record carries the regulator's
identifier, the notification format the regulator requires, and
the maximum notification latency.

## §16 Worked Example: Citation Resolution for a Bridge Condition Report

A reader encounters a peer-reviewed publication or news article
that cites a bridge condition assessment. The reader's tool
resolves the citation by:

1. Parsing the citation to extract the asset ID and manifest
   digest.
2. Fetching the discovery document for the operating programme.
3. Resolving the manifest URL and verifying the manifest
   signatures.
4. Recomputing the manifest digest and comparing it to the
   pinned digest; aborting on mismatch.
5. Retrieving the package; recomputing per-file digests;
   surfacing the resolved evidence (asset record, inspection
   set, condition assessment) to the reader.

A conformant tool completes this flow without further input.

## §17 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (infrastructure-
monitoring for SCADA telemetry, infrastructure-integration for
cross-system bridges, intelligent-transportation for traffic
operations, water-supply for water-utility operations, electric-
grid for transmission) emit cross-standard linkage records.

## §18 Reader Tooling

Programmes MAY publish supplementary reader hints (visual asset-
condition timelines, federation-file viewers, condition-trend
charts) alongside the canonical evidence package. Reader tools
are non-normative and remain under operator control.

## §19 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose attestations (ISO 55001
certification, ISO 19650 BIM hand-over completion, AASHTO Bridge
Inspector qualification) to consumers of W3C Verifiable
Credentials MAY re-issue the attestations as Verifiable
Credentials under the Data Model 2.0 specification. Re-issuance
is optional; the canonical record remains the JSON evidence-
package manifest.

## §20 Streaming Heartbeat

SSE subscribers (PHASE-2 §15) receive a heartbeat every 30
seconds; replays support `Last-Event-ID` headers (W3C
EventSource semantics). Subscribers that disconnect resume from
the last seen event identifier without losing visibility of
priority-1 incident events.

## §21 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window
of at least one full ISO 55001 surveillance cycle.

## §22 Capital-Works Financial-System Integration

Capital-works budgeting is held in the operator's financial
system. The integration is read-only from the WIA side: the
financial system consumes the prioritised work-order recommendations
(PHASE-3 §5) and emits funded budget envelopes that the operator
records against the work-order register so that downstream
auditors can resolve a work order to the funding source that
supported it. Funded amounts and contractor commercial terms
remain in the financial system; the WIA record carries only the
funding-source identifier and the funding-decision date.

## §23 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with at least one BIM CDE, at least one inspection contractor's
field platform, at least one ISO/IEC 17025-accredited NDT
laboratory, the operator's CMMS or EAM, the relevant regulator's
notification intake, and at least one long-term archive, and has
published at least one externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-infrastructure
- **Last Updated:** 2026-04-28
