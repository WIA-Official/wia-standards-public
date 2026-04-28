# WIA-agricultural-iot PHASE 4 — INTEGRATION Specification

**Standard:** WIA-agricultural-iot
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an agricultural-IoT operation
integrates with the systems that surround it: FMIS vendors;
equipment OEMs (John Deere Operations Center, AGCO
FarmHub, CNH AFS Connect, Kubota K-Hub, equivalent
platforms); AgGateway ADAPT plug-in registry; LoRaWAN
network operators; cellular IoT carriers; OGC SensorThings
catalogue services; livestock-traceability authorities;
water-rights administrators; pesticide-product registries;
animal-welfare authorities; and long-term archives.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- ISO 11783 series
- AgGateway ADAPT
- OGC SensorThings API 1.1
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 FMIS Vendor Integration

FMIS vendors (Climate FieldView, Granular, Trimble Ag
Software, Conservis, AgriWebb, AgVend, equivalent
platforms) consume operation, field, observation, and
ISOBUS task records. Integration carries the FMIS vendor's
identifier, the per-vendor adapter mapping (operator
records the FMIS-side schema mapping for each consumed
record class), and the per-record reconciliation cadence.

## §2 Equipment OEM Integration

Equipment OEMs operate manufacturer-specific platforms
that consume task data round-trip. Integration carries
each OEM's identifier, the per-OEM AgGateway ADAPT plug-in
reference, the per-OEM tractor / implement fleet binding,
and the OEM-side telemetry feed that feeds back into the
operator's analytics.

## §3 AgGateway ADAPT Plug-in Registry Integration

AgGateway operates the ADAPT plug-in registry. Integration
carries the registry's identifier, the per-plug-in version
the operator pins, and the registry's notification endpoint
for plug-in revisions. Plug-in revisions trigger the
operator's regression-test cadence (PHASE-3 §2).

## §4 LoRaWAN Network Operator Integration

LoRaWAN-using deployments integrate with a LoRaWAN Network
Server (LNS): The Things Stack, ChirpStack, vendor-managed
LNS. Integration carries the LNS's identifier, the per-
device JoinEUI / DevEUI / AppKey configuration (keys held
in HSM, references-only here), and the LNS-side uplink
endpoint.

## §5 Cellular IoT Carrier Integration

Cellular-IoT (NB-IoT, Cat-M, 5G RedCap) deployments
integrate with the mobile-network operator (MNO). Integration
carries the MNO's identifier, the per-SIM eSIM profile
management endpoint (per GSMA RSP), and the per-device data
plan binding.

## §6 OGC SensorThings Catalogue Service Integration

OGC SensorThings catalogue services consume the operator's
device / Datastream / Observation records to expose them to
broader research and policy consumers. Integration carries
each catalogue's identifier, the per-Datastream public-
catalogue eligibility, and the catalogue's ingest cadence.

## §7 Livestock Traceability Authority Integration

Livestock traceability authorities (national programmes per
PHASE-3 §6) consume per-animal birth, movement, and
slaughter events. Integration carries the authority's
identifier, the per-event submission template, and the
authority's audit-trail export endpoint.

## §8 Water-Rights Administrator Integration

Water-rights administrators (state water boards in the US,
EU national water authorities, KR water-rights authorities
under the Water Use Permit Act) consume per-zone irrigation
plans and as-applied water-application records. Integration
carries the administrator's identifier, the per-allocation
reporting cadence, and the over-allocation notification
intake.

## §9 Pesticide Product Registry Integration

Pesticide product registries (EPA OPP in the US, EU EFSA
under Regulation 1107/2009, KR Pesticide Control Division,
equivalent national registries) provide product label data
that the operator's spraying-task records cite for rate /
buffer / re-entry-interval compliance. Integration carries
the registry's identifier, the per-product registration
reference, and the operator's product-revision pickup
cadence.

## §10 Animal Welfare Authority Integration

Animal welfare authorities (USDA APHIS in the US, EU
national veterinary authorities, KR Animal Health and
Welfare Division, equivalent rules elsewhere) consume
welfare-incident notifications per PHASE-3 §7. Integration
carries the authority's identifier, the per-incident-class
notification template, and the authority's investigation
intake.

## §11 Evidence Package Format

```
evidence/
  manifest.json              — package manifest (signed)
  operation.json             — operation record
  geo-references/            — field / zone geometry
  devices/                   — device inventory
  observations/              — observation summaries for the
                                cited interval
  isobus-tasks/              — task records and as-applied
                                map references
  animals/                   — livestock records (no PII;
                                opaque tokens for human
                                handlers)
  irrigation-plans/          — irrigation plans and applied
                                records
  soil-classifications/      — soil sample classifications
  audit/                     — API audit log excerpts
```

The package is content-addressable; the manifest is signed
by the operator's HTTP-message-signature key (RFC 9421)
and counter-signed by the operator's quality manager when
the package supports a regulator submission.

## §12 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:agricultural-iot:evidence-mismatch`.

## §13 well-known URI Discovery

A conformant operator exposes a discovery document at
`/.well-known/wia-agricultural-iot` that links to the API
root, the operator's per-jurisdiction water-rights binding,
the published quality dossier, the equipment-OEM plug-in
catalogue, and the livestock-traceability authority binding.

## §14 Long-Term Archive Integration

Operators designate a long-term archive that holds
operation records (observations, ISOBUS task archives,
livestock records) beyond the operator's primary retention
horizon. Quarterly deposits round-trip content-addresses;
on operation wind-down, remaining records transfer to the
archive with content-addresses preserved.

## §15 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (water-rights
allocation, pesticide-applicator certification, organic
certification, GAP certification, ISO 22000 conformance,
ISO/IEC 27001 certification) to consumers of W3C
Verifiable Credentials MAY re-issue the attestations as
Verifiable Credentials under the Data Model 2.0
specification. Re-issuance is optional; the canonical
record remains the JSON evidence-package manifest.

## §16 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds with
`Last-Event-ID` resume support. Subscribers that disconnect
during long observation-stream windows resume from the last
seen event identifier without losing visibility of
priority-1 events (water-rights breach warnings, livestock
RFID anomalies, climate-control SOP excursions).

## §17 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with
prior-minor clients. Major revisions go through a
deprecation window of at least one full ISO 11783 series
revision cycle so that OEM and FMIS integrations have time
to migrate.

## §18 Cross-Standard Linkage

Operators that consume adjacent WIA standards (WIA-crop-
monitoring, WIA-food-traceability, WIA-iot-m2m for the
underlying IoT platform layer) emit cross-standard linkage
records that name the consuming standard and the version
under which the linkage is claimed.

## §19 Public Catalogue and Aggregator Feeds

Operators that publish a public catalogue (research
collaboration, agricultural-policy reporting) emit a JSON
Feed listing operation summaries, anonymised per-zone
yield-driver statistics, and per-operation soil-
classification summaries. The feed is intended for
research consumers and never carries per-handler personal
data or per-animal individual-welfare records.

## §20 Sustainability Reporting Integration

Operators that publish sustainability disclosures (CSRD-
aligned, GRI, SASB, equivalent) consume per-operation GHG
inventory aggregates per ISO 14064-1:2018 and emit per-
disclosure period rollups. Integration carries the
disclosure platform's identifier and the per-period
aggregation methodology.

## §21 Carbon Registry Integration

Operators that participate in agricultural carbon programmes
integrate with the chosen registry (US Climate Action
Reserve, Verra, Gold Standard, Indigo Carbon, KR K-ETS).
Integration carries the registry's identifier, the per-
methodology binding, the per-project additionality
attestation, and the per-vintage carbon-credit issuance
record.

## §22 Crop-Insurance Carrier Integration

Operators that bind crop-insurance to per-field practice
records integrate with the crop-insurance carrier (USDA
RMA-approved insurance providers in the US, EU national
crop-insurance schemes, KR Agricultural Insurance
operators). Integration carries the carrier's identifier,
the per-policy field binding, and the per-loss-event
adjustment workflow.

## §23 Conformance and Sunset

An operation conformant with PHASE-4 has integrated
successfully with at least one FMIS vendor, at least one
equipment OEM (where the operator runs ISOBUS equipment),
the AgGateway ADAPT plug-in registry, the relevant water-
rights administrator, the relevant livestock-traceability
authority (where the operator handles livestock), and at
least one long-term archive, and has published at least
one externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before
removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-agricultural-iot
- **Last Updated:** 2026-04-28
