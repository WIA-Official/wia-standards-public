# WIA-mars-mission PHASE 4 — INTEGRATION Specification

**Standard:** WIA-mars-mission
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited Mars-mission programme
integrates with the systems that surround it: ground-station
networks; partner mission operations centres; planetary-data
archives; international planetary-protection bodies; inter-agency
review boards; press and public outreach systems; and long-term
archives that preserve mission products beyond programme wind-down.
It also defines the evidence-package format that bundles a mission's
record set for citation and audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IETF RFC 8615 (well-known URIs)
- IETF RFC 8288 (Web Linking)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO/IEC 27001:2022 (information security management)
- ISO 8601 (date and time)
- CCSDS 132.0-B / 232.0-B (Space Data Link Protocols)
- CCSDS 633.0-B (Mission Operations Services)
- CCSDS 727.0-B (CFDP)
- COSPAR Planetary Protection Policy

---

## §1 Ground-Station Network Integration

Ground-station networks (DSN, ESTRACK, the Chinese Deep Space
Network, commercial providers) integrate with mission MOCs through
the booking and tracking interfaces the network operator publishes.
The mission programme records each booking, the actual track that
took place, and the per-pass quality metrics. Integration is
mutually authenticated; the network operator's client certificate
authorises the mission to consume the network's resources within
the booked window.

## §2 Partner MOC Integration

Multi-agency missions exchange records across partner MOCs. The
exchange honours the records release policy negotiated at mission
inception: some products are exchanged immediately, others on a
delayed cadence aligned with each agency's domestic release rules.
Partner MOC integrations carry the partner's identifier, the agreed
exchange schedule, and the audit log of every record exchanged.

## §3 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  mission.json                 — mission record
  spacecraft/                  — per-spacecraft records
  trajectory/                  — trajectory products and uncertainty
                                  representations
  ttc/                         — TT&C packet catalogues and archive
                                  references
  observation-requests/        — per-request records and approvals
  science-products/            — product records and PDS-aligned
                                  metadata
  planetary-protection/        — bioburden assays and approvals
  spectrum/                    — spectrum file revisions
  anomalies/                   — anomaly investigations
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest carries the SHA-256
of every file. The manifest is signed by the operating MOC's
HTTP-message-signature key (RFC 9421) and counter-signed by the
planetary-protection officer.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the manifest,
and reject the package on mismatch with type
`urn:wia:mars-mission:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-mars-mission` that links to the API root, the
public mission registration, the published quality dossier, the
release-policy schedule, the catalogue of released science products,
and the planetary-protection officer's accreditation reference.

## §6 Planetary-Data Archive Integration

PDS-aligned planetary-data archives consume science products at the
release schedule the mission publishes. The integration record
carries the archive's identifier, the data-product profile the
archive accepts, and the deposit cadence. Deposit verification
re-pins content-addresses on each deposit so that the archived copy
matches the operationally-released copy.

## §7 Inter-Agency Review Board Integration

Inter-agency review boards (mission-critical-activity reviews,
anomaly review boards, planetary-protection review panels) receive
records appropriate to their scope through dedicated client
certificates. The integration record carries each board's identifier,
the records the board is authorised to access, and the review
schedule.

## §8 Press and Public Outreach

Press and public outreach systems consume Level-1 product summaries
and derived narrative content. The integration is one-way (mission
to outreach) and is rate-limited; outreach systems do not re-write
into the mission record set. Press releases that cite mission
products carry the evidence-package manifest digest so that
downstream readers can resolve the release back to the underlying
records.

## §9 Long-Term Archive Integration

Beyond the PDS-aligned archive, missions designate a long-term
archive that holds operational records (TT&C catalogues, anomaly
investigations, audit logs) that PDS does not host. Quarterly
deposits round-trip content-addresses; on programme wind-down,
remaining records transfer to the archive with content-addresses
preserved.

## §10 International Planetary-Protection Body Integration

Programmes register with the international planetary-protection
body (the COSPAR Panel on Planetary Protection or the equivalent
body the programme's jurisdiction recognises). Registration records
the mission's category, the responsible officer, and the timeline
under which the programme is willing to release planetary-
protection records to the body for retrospective audit.

## §11 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose mission attestations to consumers of
W3C Verifiable Credentials MAY re-issue the planetary-protection
record or the science-product release attestation as a Verifiable
Credential. Re-issuance is optional; the canonical record remains
the JSON evidence-package manifest.

## §12 Worked Example: Citation Resolution

A reader encounters a peer-reviewed publication that cites a
mission's surface observation. The reader's tool resolves the
citation by:

1. Parsing the citation to extract the science-product ID and
   manifest digest.
2. Fetching the discovery document for the issuing mission.
3. Resolving the manifest URL and verifying the manifest signatures.
4. Recomputing the manifest digest and comparing it to the pinned
   digest; aborting on mismatch.
5. Retrieving the package; recomputing per-file digests; surfacing
   the resolved evidence (observation request, TT&C catalogue,
   science product, planetary-protection record) to the reader.

A conformant tool completes this flow without further input.

## §13 Cross-Standard Linkage

A mission that supports adjacent WIA standards (a sample-return
chain that consumes a WIA-quarantine standard, a crewed mission
that consumes a WIA-life-support standard) emits cross-standard
linkage records.

## §14 Reader Tooling

Programmes MAY publish supplementary reader hints (visual
trajectory plots, observation-target maps, time-line visualisations
of TT&C activity) alongside the canonical evidence package.
Supplementary hints are non-normative.

## §15 Sample-Return Curatorial Integration

Sample-return missions integrate with curatorial facilities that
hold returned samples under climate- and atmospheric-controlled
conditions. Curatorial integrations consume the sample-return chain
records (PHASE-1 §11) and emit per-sample subsample-distribution
records that record which subsamples were issued to which
investigators under what consent/agreement. Integrators MUST honour
the quarantine release criteria recorded in PHASE-3 §15 before
distributing subsamples.

## §16 Public-Access Outreach Integration

Mission programmes deploy public-access outreach systems that expose
selected science products and narrative content to general
audiences. The integration is read-only and rate-limited, and is
governed by the release-policy schedule recorded in the well-known
discovery document. Public-access systems do not consume operational
records (TT&C catalogues, anomaly investigations) by default;
opt-in disclosures are recorded under a separate scope.

## §17 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-minor
clients. Major revisions go through a deprecation window of at
least one full mission planning cycle (typically two Earth years).

## §18 Environmental-Data Aggregator Integration

Climate-modelling teams and future-mission planners aggregate
environmental observations across missions. The integration consumes
the observations published under each contributing mission's
release schedule and emits aggregated time series that re-credit
the source missions. Aggregator integrations carry the aggregator's
identifier and the access scope; out-of-scope queries return
`403 Forbidden` with type
`urn:wia:mars-mission:aggregator-scope`.

## §19 Reader Tooling for Mission Timelines

Mission timelines are dense and visual; programmes MAY publish
supplementary timeline visualisers that present TT&C pass schedules,
sol activity execution traces, and science-product availability as
human-readable charts. These visualisers are non-normative reader
hints; the canonical record remains the JSON evidence package.

## §20 Cross-Standard Linkage

Missions that consume adjacent WIA standards (sample-return chain
under WIA-quarantine, crewed-mission life-support under
WIA-life-support) emit cross-standard linkage records that name the
consuming standard and the version under which the linkage is
claimed. Linkages are not transitive; consumers verify each
adjacent standard's evidence directly.

## §21 Migration from Pre-Standard Records

Programmes operated before WIA-mars-mission reached version 1.0 MAY
migrate historical missions by emitting synthetic mission records
that carry the original mission's identifying information plus a
`legacyImport` flag. Synthetic missions are accepted by the public
catalogue but are not eligible for evidence-package generation
without contemporaneous re-validation of their record set.

## §22 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose mission attestations (planetary-
protection clearance, science-product release attestation,
mission-critical-activity review outcome) to consumers of W3C
Verifiable Credentials MAY re-issue the attestations as Verifiable
Credentials under the Data Model 2.0 specification. Re-issuance is
optional; the canonical record remains the JSON evidence-package
manifest.

## §23 Citation-Indexing Adapters

Bibliographic indexing services and library catalogues consume
machine-readable citation entries so that a paper that cites a
mission product can be discovered by readers who start from the
mission's catalogue. Programmes MAY publish citation-index adapters
that emit citation entries in the indexing service's preferred
format (BibTeX, JSON-LD, Atom-of-CitationStyle); adapters are
non-normative integration extensions.

## §24 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully with
at least one ground-station network, at least one PDS-aligned
archive, the relevant international planetary-protection body, and
at least one partner MOC, and has published at least one externally
citable evidence package.

Sunsetting an integration is announced via the well-known discovery
document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-mars-mission
- **Last Updated:** 2026-04-27
