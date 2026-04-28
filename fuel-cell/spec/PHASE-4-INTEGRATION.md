# WIA-fuel-cell PHASE 4 — INTEGRATION Specification

**Standard:** WIA-fuel-cell
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a fuel-cell deployment
integrates with the systems that surround it: the
operating jurisdiction's authority having jurisdiction
(AHJ); the operating jurisdiction's grid system
operator (where the fuel cell is grid-coupled); the
operating jurisdiction's vehicle-type-approval
authority (where the fuel cell powers a vehicle); the
ISO/IEC 17025-accredited hydrogen-fuel-quality
laboratory; the IECEx certification body (where Ex
zones apply); the IEC 62282 conformity-assessment
body; the IEEE 1547.1-accredited test laboratory; the
manufacturer's reliability and warranty platform; the
hydrogen-supply-chain provider (pipeline operator,
tube-trailer operator, on-site electrolyser per ISO
22734); and long-term archives.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17025:2017 (testing and calibration
  laboratories)
- ISO/IEC 17021-1:2015 (management-system audit and
  certification)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 AHJ Integration

The operating jurisdiction's authority having
jurisdiction (AHJ) — typically a local building or
fire authority for stationary installations, the
operating ministry for industrial installations, the
vehicle type-approval authority for vehicle
installations — coordinates permitting and acceptance.
Integration carries the AHJ's identifier, the per-
permit application endpoint, the per-acceptance
inspection workflow, the per-incident notification
endpoint, and the AHJ's response SLA.

In the United States, AHJs typically adopt NFPA 2 (the
Hydrogen Technologies Code) and the National
Electrical Code (NFPA 70) in parallel with IEC 62282
and IEC 60079. In the European Union, the operating
Member State's transposition of ATEX Directives
2014/34/EU and 1999/92/EC and the Pressure Equipment
Directive 2014/68/EU applies. In Korea, KGS Code
AC112 and the Ministry of Trade, Industry and Energy
hydrogen fuel-cell installation code apply.

## §2 Grid System Operator Integration

For grid-coupled installations integration carries
the grid system operator's identifier, the executed
interconnection agreement reference, the IEEE 1547
ride-through-category declaration, the per-rec point-
of-common-coupling capacity, and the operator's
disturbance-event reporting endpoint. The grid system
operator audits the deployment's IEEE 1547.1
conformance reports on the operating jurisdiction's
re-test cadence.

## §3 Vehicle-Type-Approval Authority Integration

For vehicle-onboard installations integration carries
the vehicle-type-approval authority's identifier, the
per-vehicle UN GTR 13 test report reference, the per-
vehicle UN R134 type-approval certificate reference
(where the operating jurisdiction recognises UN R134),
and the per-vehicle in-service inspection cadence.

## §4 Hydrogen-Fuel-Quality Laboratory Integration

The deployment's contracted ISO/IEC 17025-accredited
laboratory consumes per-sample submissions and emits
ISO 14687 verdicts. Integration carries the
laboratory's identity, its ISO/IEC 17025 accreditation
reference (with the contaminant-panel scope), the
per-sample chain-of-custody envelope, and the per-
verdict signed report. Verdicts of `non-conforming-
supply-rejected` propagate to the supply-chain provider
for source investigation.

## §5 IECEx Certification Body Integration

For Ex-zoned installations integration carries the
IECEx-recognised inspection body's identifier, the
per-equipment Certificate of Conformity references,
the per-installation IEC 60079-14 design verification
report, the per-cycle IEC 60079-17 inspection report,
and the per-modification re-inspection workflow.

## §6 IEC 62282 Conformity-Assessment Body Integration

The IEC 62282 series is supported by manufacturer self-
declaration where the operating jurisdiction permits,
or by third-party conformity assessment under ISO/IEC
17065 where the operating jurisdiction or AHJ
mandates third-party assessment. Integration carries
the conformity-assessment body's identifier, the per-
test report archive, and the per-cycle attestation
refresh.

## §7 IEEE 1547.1-Accredited Test Laboratory Integration

For grid-coupled installations the IEEE 1547.1-2020
test laboratory issues the conformance test report.
Integration carries the laboratory's identity, the
per-system test-report archive, the per-system
firmware build-of-record (so that test results map to
the firmware operating in the field), and the per-
firmware re-test cadence.

## §8 Manufacturer Reliability and Warranty Platform Integration

The manufacturer's reliability and warranty platform
consumes field-service incident records and operating-
hour aggregates. Integration carries the manufacturer's
identifier, the per-stack reliability-data envelope,
the per-event warranty-claim workflow, and the per-
firmware update channel.

## §9 Hydrogen-Supply-Chain Provider Integration

Hydrogen-supply-chain integration spans the per-
delivery chain-of-custody (source — pipeline,
production-site identity, batch identity for cylinder
deliveries, vapouriser-batch identity for liquid
deliveries, electrolyser-cycle identity per ISO 22734
for on-site generation), the per-supply ISO 14687
verdict reference, and the per-supply rejection
workflow when the verdict is non-conforming.

## §10 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed)
  programme.json               — programme record
  stacks/                      — stack records
                                  (commissioned and
                                  retired)
  balance-of-plant/            — BoP records (current
                                  and superseded)
  fuel-quality/                — fuel-quality
                                  verification records
  grid-interconnection/        — IEEE 1547 evidence
                                  (where grid-coupled)
  vehicle-onboard/             — UN GTR 13 / UN R134
                                  evidence (where
                                  vehicle-onboard)
  commissioning/               — commissioning records
  inspections/                 — periodic-inspection
                                  history
  incidents/                   — incident records and
                                  root-cause analyses
  ex-equipment/                — IECEx Certificate of
                                  Conformity references
                                  and IEC 60079
                                  inspection history
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is
signed by the deployment's HTTP-message-signature key
(RFC 9421) and counter-signed by the deployment's
quality manager when the package supports a regulator
submission.

## §11 Manifest and Signatures

Verification tools recompute file digests, compare to
the manifest, and reject the package on mismatch with
type `urn:wia:fuel-cell:evidence-mismatch`.

## §12 well-known URI Discovery

A conformant deployment exposes a discovery document
at `/.well-known/wia-fuel-cell` that links to the API
root, the AHJ binding, the grid-system-operator
binding (where applicable), the vehicle-type-approval
authority binding (where applicable), the contracted
ISO/IEC 17025 laboratory, and the IECEx inspection
body (where applicable).

## §13 Long-Term Archive Integration

Deployments designate a long-term archive that holds
commissioning records, periodic-inspection histories,
incident records, and IECEx CoC references beyond the
deployment's primary retention horizon. Quarterly
deposits round-trip content-addresses; on programme
decommissioning, remaining records transfer to the
archive with content-addresses preserved.

## §14 Verifiable-Credential Re-Issuance (optional)

Deployments that wish to expose attestations (IEC
62282-3-100 conformance, IEEE 1547 conformance, IECEx
CoC adoption, ISO/IEC 17025 laboratory accreditation
of the hydrogen-fuel-quality verification) to consumers
of W3C Verifiable Credentials MAY re-issue the
attestations as Verifiable Credentials under the Data
Model 2.0 specification. Re-issuance is optional; the
canonical record remains the JSON evidence-package
manifest.

## §15 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds
with `Last-Event-ID` resume support. Subscribers that
disconnect during inspection windows or grid
disturbance windows resume from the last seen event
identifier without losing visibility of priority-1
events (incident declared, grid disturbance ride-
through engaged, fuel-quality non-conformance
verdict, IECEx inspection lapsed).

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible
with prior-minor clients. Major revisions go through
a deprecation window of at least one full inspection
cycle so that AHJ, grid-system-operator,
manufacturer, and laboratory integrations have time
to migrate.

## §17 Cross-Standard Linkage

Deployments that consume adjacent WIA standards (WIA-
hydrogen-energy for upstream hydrogen production
discipline, WIA-distributed-energy for the distribution-
energy resource governance overlay where the fuel cell
is one of multiple DERs at the site, WIA-energy-
storage for regenerative-mode operation co-located with
storage, WIA-electric-vehicle-charging for vehicle-
onboard interactions with the refuelling-station
infrastructure) emit cross-standard linkage records.

## §18 Reader Tooling

Deployments MAY publish supplementary reader tools
(per-stack performance dashboards, per-fleet operating-
hour summaries, per-installation IEC 60079-17
inspection-cadence trackers, per-incident root-cause
catalogues) alongside the canonical evidence package;
the tools are non-normative.

## §19 Public Catalogue Feed

Deployments publish a public catalogue feed listing
the in-force IEC 62282 attestation, the IEEE 1547
attestation (where applicable), the UN R134 type-
approval certificate (where applicable), the IECEx
CoC adoption summary (where applicable), the ISO
14687 fuel-quality verification cadence, and the
aggregate operating-hour count. The feed enables
peer-deployment and AHJ discovery of the deployment's
operating posture over time.

## §20 Per-Application Hydrogen Refuelling Station Integration

For applications that pair with a refuelling station
(industrial-electric-truck and vehicle-onboard
applications) integration carries the refuelling
station's ISO 19880-1 conformance reference, the per-
fill SAE J2601 / ISO 19880-1 protocol log, the per-
station fuel-quality verification cadence, and the
per-incident reciprocal notification (a fuel-quality
non-conformance at the station propagates to all
fueled vehicles at the station for pre-emptive
inspection).

## §21 Public Catalogue Aggregator Integration

Civil-society researchers, academic-research consortia,
and energy-policy research organisations consume
aggregate operating-hour and incident statistics for
independent analysis. Integration carries the
consumer's identifier, the per-research-purpose
data-access agreement, and the deployment's
publication of consumer-attribution in any derivative
research output.

## §22 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with the operating jurisdiction's AHJ,
the operating jurisdiction's grid system operator
(for grid-coupled installations), the operating
jurisdiction's vehicle-type-approval authority (for
vehicle-onboard installations), at least one ISO/IEC
17025 laboratory for the contaminant panel, the IECEx
inspection body (for Ex-zoned installations), the
manufacturer's reliability platform, at least one
hydrogen-supply-chain provider, and at least one
long-term archive, and has published at least one
externally citable evidence package.

Sunsetting an integration is announced via the well-
known discovery document at least 90 calendar days
before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-fuel-cell
- **Last Updated:** 2026-04-28
