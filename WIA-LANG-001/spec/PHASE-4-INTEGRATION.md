# WIA-LANG-001 PHASE 4 — INTEGRATION Specification

**Standard:** WIA-LANG-001
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a WIA-LANG-001 programme
integrates with the systems that surround it: guarantee-of-
origin schemes (CertifHy, US DOE 45V, UK LCHS, KEA H2 GoO);
ISO/IEC 17025-accredited fuel-quality laboratories; mobile-
network-operator-equivalent industrial telemetry vendors;
midstream gas-network operators (where hydrogen blends into
the natural-gas network); fuel-cell vehicle OEMs (for
station-to-vehicle quality citations); industrial offtakers
(refining, ammonia, methanol, steel direct-reduction);
emergency-response services; and long-term archives.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- ISO 14687:2019 / 19880-1:2020 / 22734:2019
- SAE J2601 / J2719 / J2799
- IEC 62282 series
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Guarantee-of-Origin Scheme Integration

The operator's guarantee-of-origin scheme integration
carries the scheme operator's identifier (CertifHy
secretariat, US DOE 45V administering agency, UK LCHS
administrator, KEA H2 GoO in Korea, equivalent), the per-
batch issuance template, the batch-retirement endpoint that
downstream offtakers call to consume the guarantee, and the
reconciliation report cadence.

## §2 ISO/IEC 17025 Laboratory Integration

ISO/IEC 17025-accredited fuel-quality laboratories emit
signed test certificates per production batch. Integration
carries the laboratory's accreditation reference, the
certificate's content-address, and the validity expiry.
Laboratory revocations of an accreditation scope freeze the
operator's ability to cite the laboratory for future
batches; in-flight batches retain their issued certificates.

## §3 Industrial Telemetry Vendor Integration

Industrial control systems (DCS, SCADA, IIoT historians)
emit production / inventory / dispensing telemetry that the
operator's WIA facade ingests. Integration carries the
vendor's identifier (Siemens / Honeywell / Yokogawa /
Emerson / Aveva / Schneider, vendor-specific platforms),
the per-system tag mapping, and the ingest backpressure
policy.

## §4 Gas-Network Operator Integration (Blending)

Operators that blend hydrogen into the natural-gas network
integrate with the gas-network operator. Integration
carries the network operator's identifier, the per-blending-
point allowable percentage (per the network operator's
material-compatibility study), the blending-percentage
telemetry endpoint, and the per-batch quality citation that
the network operator forwards to downstream consumers.

## §5 Fuel-Cell Vehicle OEM Integration

Vehicle OEMs consume per-station quality citations and per-
fill envelope data so that warranty and reliability
analytics can correlate vehicle behaviour with station-side
fuel quality. Integration carries the OEM's identifier, the
per-vehicle-class consumption profile, and the warranty-
event notification endpoint.

## §6 Emergency-Response Services Integration

Industrial-safety incidents (PHASE-1 §8 severity `major` or
`critical`) emit notifications to the operating jurisdiction's
emergency-response authority (local fire department,
hazardous-materials response team, KOSHA in Korea, OSHA in
the US, equivalent authorities). Integration carries the
authority's identifier, the per-incident-class notification
template, and the operator's coordinated-response SOP.

## §7 Industrial Offtaker Integration

Industrial offtakers (refineries, ammonia plants, methanol
plants, steel direct-reduction plants) consume per-batch
delivered hydrogen with quality citation and guarantee-of-
origin attribution. Integration carries each offtaker's
identifier, the per-offtaker contract, the per-delivery
SLA, and the offtaker's downstream LCA reporting
attribution chain.

## §8 Evidence Package Format

```
evidence/
  manifest.json              — package manifest (signed)
  programme.json             — programme record
  production/                — production records for the
                                cited interval
  quality-records/           — per-batch quality records
  storage-inventories/       — inventory snapshots
  refuelling-events/         — HRS dispensing records
  transport-events/          — transport history
  safety-incidents/          — incident records and root-
                                cause references
  guarantee-of-origin/       — GoO issuance and retirement
                                records
  audit/                     — API audit log excerpts
```

The package is content-addressable; the manifest is signed
by the operator's HTTP-message-signature key (RFC 9421) and
counter-signed by the operator's quality manager when the
package supports a regulatory or GoO submission.

## §9 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:WIA-LANG-001:evidence-mismatch`.

## §10 well-known URI Discovery

A conformant operator exposes a discovery document at
`/.well-known/wia-WIA-LANG-001` that links to the API
root, the operator's licensing summary, the published
quality dossier, the GoO scheme membership, the catalogue
of operating facilities, and the per-quarter safety-
incident summary.

## §11 Long-Term Archive Integration

Operators designate a long-term archive that holds
production / quality / safety-incident records beyond the
operator's primary retention horizon. Quarterly deposits
round-trip content-addresses; on programme wind-down,
remaining records transfer to the archive with content-
addresses preserved.

## §12 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (operating
licence status, ISO 9001 conformance, ISO 14001
conformance, ISO 45001 OH&S conformance, ISO/IEC 27001
certification, GoO scheme membership) to consumers of W3C
Verifiable Credentials MAY re-issue the attestations as
Verifiable Credentials under the Data Model 2.0
specification. Re-issuance is optional; the canonical
record remains the JSON evidence-package manifest.

## §13 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds with
`Last-Event-ID` resume support. Subscribers that disconnect
during long production / inventory windows resume from the
last seen event identifier without losing visibility of
priority-1 events (cryogenic-temperature excursions,
quality-batch downgrades, safety incidents at major or
above).

## §14 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with
prior-minor clients. Major revisions go through a
deprecation window of at least one full ISO 19880 / SAE
J2601 release cycle so that midstream and downstream
integrations have time to migrate.

## §15 Cross-Standard Linkage

Operators that consume adjacent WIA standards (WIA-fuel-
cell for fuel-cell stack catalogues, WIA-hydrogen-vehicle
for vehicle-side records, WIA-electric-grid for renewable
electricity sourcing for electrolysis, WIA-carbon-capture
for CCS-equipped SMR/ATR) emit cross-standard linkage
records that name the consuming standard and the version
under which the linkage is claimed.

## §16 Public Catalogue and Aggregator Feeds

Operators publish a public catalogue of operating
facilities and per-quarter aggregate production through an
Atom or JSON Feed listing the records with their evidence-
package manifest digests, the carbon-intensity range, and
the GoO issuance summary.

## §17 Reader Tooling

Operators MAY publish supplementary reader tools (real-
time facility-fleet dashboards, per-batch quality dashboards,
HRS service-level dashboards, safety-incident timeline
visualisers) alongside the canonical evidence package; the
tools are non-normative.

## §18 Migration from Pre-Standard Records

Operators of legacy hydrogen value-chain segments that pre-
date WIA-LANG-001 MAY migrate historical records by
emitting synthetic facility records with a `legacyImport`
flag. Synthetic facilities are accepted by the public
catalogue but not eligible for evidence-package generation
without contemporaneous re-validation under PHASE-3 §3.

## §19 Inter-Operator Coordination Integration

Multi-operator value chains (independent producer +
midstream + dispenser) coordinate through a shared
operations channel. The integration carries the per-
counterparty identifier, the per-batch-handover SLA, and
the loss-control reconciliation cadence.

## §20 Vehicle-Authority Integration (FCEV registration)

Vehicle authorities (national vehicle-registration
authorities) ingest fuel-cell vehicle records in
correlation with the operator's HRS network. Integration
carries the authority's identifier and the per-vehicle-
class registration mapping that supports public statistics
on FCEV adoption alongside the operator's HRS coverage map.

## §21 Renewable-Electricity Sourcing Integration (Electrolysis)

Electrolyser operators that bind production to renewable
electricity sources (PPA-backed wind / solar, dedicated
renewable installations, behind-the-meter renewables)
integrate with the renewable-electricity certificate
registry of the operating jurisdiction (REC in the US,
GO in the EU, REGO in the UK, KEA renewable-energy
certificate in Korea). Integration carries the registry's
identifier, the per-MWh certificate retirement record, and
the per-batch additionality attribution documentation per
the GoO scheme of §1.

## §22 Carbon-Capture Operator Integration (CCS-equipped Production)

Operators that produce hydrogen from SMR / ATR with CCS
(blue hydrogen) integrate with the carbon-capture-and-
storage operator (often a separate entity that operates
the pipeline + storage formation). Integration carries the
CCS operator's identifier, the per-batch CO2-captured-and-
delivered handover record, the storage-formation injection
verification, and the leakage-monitoring report cadence
that the operator's GoO / 45V / LCHS attribution chain
depends on.

## §23 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with at least one guarantee-of-origin scheme,
at least one ISO/IEC 17025-accredited laboratory, at least
one industrial-telemetry vendor, the relevant emergency-
response services, at least one industrial offtaker (where
applicable), and at least one long-term archive, and has
published at least one externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before
removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-LANG-001
- **Last Updated:** 2026-04-28
