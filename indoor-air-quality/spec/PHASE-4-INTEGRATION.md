# WIA-indoor-air-quality PHASE 4 — INTEGRATION Specification

**Standard:** WIA-indoor-air-quality
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited indoor-air-quality
programme integrates with the systems that surround it: building
management systems (BMS); sensor-package vendors; ISO/IEC 17025
laboratories; mechanical contractors; occupant-engagement platforms
(operator CRM); occupational-health systems; public-health
authorities; long-term archives; and the certifying bodies that
audit IAQ programmes.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IETF RFC 8615 (well-known URIs)
- IETF RFC 8288 (Web Linking)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO 16000 series
- ISO 8601 (date and time)
- ASHRAE Standard 62.1 / 62.2
- WHO Guidelines for Indoor Air Quality

---

## §1 BMS Integration

Building management systems are the primary on-site source of
continuous IAQ telemetry. Integration is mediated by an adapter
that translates between the BMS's native protocol (BACnet, Modbus,
or vendor-specific) and the WIA-native PHASE-1 §4 sample format.
The adapter is owned by the operating programme and is exercised
during BMS upgrades.

Adapters MUST honour the sensor-package categorisation rule
(PHASE-3 §1) and MUST refuse to up-categorise a `consumer-grade`
sensor in the BMS into an `accredited-laboratory-grade` submission
on the API.

## §2 Sensor-Package Vendor Integration

Sensor-package vendors register their packages with the operating
programme via the sensor-register endpoint. Vendor registrations
include the package's calibration history, the analytes covered,
the measurement uncertainties, and the firmware version. Updates
emit new registration records; prior registrations remain
addressable for retrospective audit.

## §3 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  site.json                    — site record
  zones/                       — per-zone configuration and filter
                                  rotation history
  iaq-samples/                 — continuous-sample summaries (full
                                  series referenced by content-
                                  address)
  episodic-samples/            — per-sample certificates
  verifications/               — ventilation-verification reports
  symptoms/                    — aggregate symptom counts (raw
                                  records held under access-
                                  controlled scope)
  investigations/              — source investigations and root
                                  causes
  remediations/                — remediation actions and post-
                                  action verifications
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest carries the SHA-
256 of every file. The manifest is signed by the operating
programme's HTTP-message-signature key (RFC 9421) and counter-
signed by the analysing laboratory whose certificate appears in
the package.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:indoor-air-quality:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-indoor-air-quality` that links to the API root,
the public laboratory accreditation references, the published
quality dossier, the sampling-strategy summary, and the catalogue
of released ventilation-verification reports.

## §6 Mechanical Contractor Integration

Mechanical contractors that perform commissioning, retro-
commissioning, and ventilation verification integrate via the
ventilation-verification endpoint (PHASE-2 §6). The contractor's
client certificate is bound to the contractor's identifier and
signed verification reports flow through the integration in both
directions: contractor uploads verification, operator countersigns
acceptance.

## §7 Occupant-Engagement Platform Integration

Occupant-engagement platforms (operator CRM, occupant-portal
applications, in-building kiosks) integrate via the symptom
endpoint (PHASE-2 §7). The integration honours occupant consent at
collection time, redacts clinical identifiers before submission to
the API, and records the consent reference against each symptom
record.

## §8 Occupational-Health System Integration

Occupational-health systems consume zone-level symptom aggregates
and laboratory-grade IAQ measurements that intersect occupational
exposure. Integration is restricted to the occupational-health
client certificate; raw symptom free-text fields and individual
occupant identifiers are exposed only to authorised occupational-
health roles.

## §9 Public-Health Authority Integration

Public-health authorities consume aggregate IAQ trends from sites
with public-health relevance (schools, childcare, healthcare,
mass-transit). The integration is one-way push at the cadence the
authority requires and uses the privacy-preserving aggregation
endpoints (PHASE-2 §10).

## §10 Long-Term Archive Integration

Programmes designate a long-term archive that holds operational
records beyond programme wind-down. Quarterly deposits round-trip
content-addresses; on programme wind-down, remaining records
transfer to the archive with content-addresses preserved.

## §11 Citation and Pinning

Externally cited IAQ findings (peer-reviewed papers, public-health
guidance updates, building-design case studies) are referenced by
their evidence package's manifest digest. Programmes MUST keep
evidence packages addressable for at least seven years from the
citation event.

## §12 Worked Example: Citation Resolution

A reader encounters a peer-reviewed publication that cites a
school's IAQ improvement programme. The reader's tool resolves the
citation by:

1. Parsing the citation to extract the site ID, the period, and
   the manifest digest.
2. Fetching the discovery document for the operating programme.
3. Resolving the manifest URL and verifying the manifest signatures.
4. Recomputing the manifest digest and comparing it to the pinned
   digest; aborting on mismatch.
5. Retrieving the package; recomputing per-file digests; surfacing
   the resolved evidence (continuous-sample summaries, episodic
   certificates, verification reports, remediation actions) to the
   reader.

A conformant tool completes this flow without further input.

## §13 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (occupational health,
building energy management, school facility management) emit cross-
standard linkage records. Linkages are not transitive; consumers
verify each adjacent standard's evidence directly.

## §14 Reader Tooling

Programmes MAY publish supplementary reader hints (visual time-
series of CO2 across a school day, episodic-sample compliance
rollups, symptom-cluster heat maps with privacy-preserving
aggregation) alongside the canonical evidence package. Reader tools
are non-normative.

## §15 Public Catalogue

Programmes that publish a public catalogue of IAQ findings emit an
Atom or JSON Feed listing the participating sites with their
ventilation-verification dates, last episodic sampling event, and
any active investigations. The feed does not carry occupant
symptom records or unredacted clinical detail.

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window of
at least one full ASHRAE 62.1 / 62.2 revision cycle.

## §17 Threshold-Table Publication Integration

Authoritative threshold tables (WHO, ASHRAE, national authorities)
are published through their respective bodies and consumed into
the operating programme via threshold-table publication adapters.
The adapter signs the imported table with the publishing body's
key chain (where available) and records the table content-address
in the programme's quality dossier so that downstream alerting is
anchored to a verified source.

## §18 Outdoor Air-Quality Feed Integration

Sites in regions with episodic outdoor air-quality events (wildfire
smoke, urban smog, agricultural burn seasons) integrate with
outdoor-air-quality data feeds (national air-quality monitoring,
satellite-derived smoke products) so that the indoor ventilation
control strategy can switch automatically when outdoor conditions
warrant. The feed integration records the outdoor data source, the
update cadence, and the trigger thresholds that activate the
adaptation playbook (PHASE-3 §20).

## §19 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (occupational
health, building energy management, school facility management,
healthcare facility management) emit cross-standard linkage
records. Linkages are not transitive; consumers verify each
adjacent standard's evidence directly.

## §20 Filter-Vendor Lifecycle Integration

Filter-vendor lifecycle integration carries the vendor's filter
specifications, the certified service life, the recommended
rotation cadence, and the disposition guidance for spent filters.
Vendor lifecycle data is consumed by the operating programme's
maintenance planner so that filter rotations are scheduled within
the certified life and the disposition pathway is honoured.

## §21 Pre-Standard Migration

Programmes that operated before WIA-indoor-air-quality reached
version 1.0 MAY migrate historical IAQ programmes by emitting
synthetic site records with a `legacyImport` flag. Synthetic
sites are accepted by the public catalogue but are not eligible
for evidence-package generation without contemporaneous
re-validation under PHASE-3 §3.

## §22 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose IAQ attestations (ventilation
verification compliance, episodic-sample acceptance) to consumers
of W3C Verifiable Credentials MAY re-issue the attestations as
Verifiable Credentials under the Data Model 2.0 specification.
Re-issuance is optional; the canonical record remains the JSON
evidence-package manifest.

## §23 Reader Tooling

Programmes MAY publish supplementary reader hints (visual time
series of CO2 across school days, episodic-sample compliance
rollups, symptom-cluster heat maps with privacy-preserving
aggregation) alongside the canonical evidence package. Reader
tools are non-normative.

## §24 Outdoor Air-Quality Aggregator Integration

Outdoor air-quality aggregators (national air-quality monitoring
networks, satellite-derived smoke products, citizen-science
monitoring projects) consume per-region indoor-air-quality
aggregates so that joint outdoor-indoor analyses can identify
infiltration pathways and exposure-stratification by indoor
function. Aggregator integrations carry the aggregator's
identifier, the access scope, and the de-identification controls
that the operating programme applies before publishing
aggregates.

## §25 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with at least one BMS, at least one sensor-package vendor, at
least one ISO/IEC 17025 laboratory, at least one mechanical
contractor, and the relevant public-health authority where
applicable, and has published at least one externally citable
evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-indoor-air-quality
- **Last Updated:** 2026-04-27
