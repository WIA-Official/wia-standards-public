# WIA-interplanetary-travel PHASE 4 — INTEGRATION Specification

**Standard:** WIA-interplanetary-travel
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an interplanetary-travel programme
integrates with the systems that surround it: launch service
providers; deep-space tracking networks; partner space
agencies (in cooperative missions); the COSPAR Panel on
Planetary Protection; the operating jurisdiction's space-
activities authority; the operating agency's Radiation Health
Office (for crewed missions); planetary-data archives that
ingest mission products; conjunction-screening services; and
long-term archives that preserve mission records.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- ISO 24113:2023 (debris mitigation)
- CCSDS 301.0-B / 503.0-B / 504.0-B / 508.0-B / 727.0-B
- COSPAR Planetary Protection Policy
- IADC Space Debris Mitigation Guidelines
- ITU-R RR Article 22
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Launch Service Provider Integration

The launch service provider operates the Earth-departure
stage and delivers the interplanetary spacecraft to the
declared injection orbit. Integration carries:

- the launch service provider's identifier;
- the per-launch payload-integration agreement;
- the per-launch launch-corridor schedule;
- the per-launch range-safety termination provision.

Launch postponements (technical scrub, weather hold, range-
conflict) emit notifications through the streaming
subscription so that downstream consumers (deep-space tracking
networks, planetary-protection officers monitoring approval
windows) can plan accordingly.

## §2 Deep-Space Tracking Network Integration

Tracking networks (NASA DSN, ESA ESTRACK, JAXA, China DSN,
commercial providers) consume the operator's tracking-pass
booking requests and emit per-pass telemetry capture
acknowledgements. The integration carries each network's
identifier, the per-pass allocation calendar, the operator's
priority class, and the network's published tracking-pass
service catalogue.

## §3 Partner Space Agency Integration

Cooperative missions integrate with each partner agency's
mission-operations system. The integration record carries the
partner's identifier, the per-instrument data-rights
schedule, the per-component planetary-protection
responsibility split, and the joint mission-operations
protocol.

## §4 COSPAR Panel on Planetary Protection Integration

The COSPAR Panel on Planetary Protection consumes the
operator's category assignment, bioburden assays, and PP
documentation through the operator's PP officer of record.
Integration carries the Panel's identifier, the per-category
documentation template, and the per-mission PP review cycle.

## §5 Operating Jurisdiction Space Authority Integration

The operating jurisdiction's space-activities authority
consumes launch-licence applications, post-launch reports, and
end-of-mission disposition records. Integration carries the
authority's identifier, the per-licence-class submission
template, and the authority's incident-notification intake.

## §6 Agency Radiation Health Office Integration (Crewed)

Crewed-mission operators integrate with the operating space
agency's Radiation Health Office for per-crew dose-limit
governance. Integration carries the Office's identifier, the
per-crew-member dose-limit policy in force, and the
agency-internal medical-records continuity arrangement that
preserves career dose totals across missions.

## §7 Planetary-Data Archive Integration

PDS-aligned archives (NASA Planetary Data System, ESA
Planetary Science Archive, JAXA DARTS, equivalent national
archives) ingest mission science products at the agreed
release schedule. Integration carries the archive's
identifier, the per-product profile the archive accepts, the
deposit cadence, and the archive accession identifier the
operator records against the released product.

## §8 Conjunction Screening Service Integration

Conjunction screening services (the operator's primary catalog
provider, the secondary cross-check provider, the partner
agency's catalog where applicable) emit Conjunction Data
Messages per CCSDS 508.0-B. The integration record carries
each provider's identifier, the per-screening cadence, and
the operator's Pc-model alignment policy.

## §9 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed)
  mission.json                 — mission record
  reference-frames/            — frame definitions in force
  trajectories/                — per-iteration trajectory and
                                  OEM artefact
  consumable-budgets/          — budgets through the cited
                                  interval
  radiation-ledgers/           — ledger summaries (no PII;
                                  crew-member tokens only)
  conjunctions/                — conjunction history and
                                  decisions
  pp-compliances/              — per-state PP records and
                                  bioburden assay references
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is signed
by the operator's HTTP-message-signature key (RFC 9421) and
counter-signed by the planetary-protection officer.

## §10 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:interplanetary-travel:evidence-mismatch`.

## §11 well-known URI Discovery

A conformant operator exposes a discovery document at
`/.well-known/wia-interplanetary-travel` that links to the
API root, the launch-licence summary, the published quality
dossier, the COSPAR PP category, and the catalogue of
mission products released to PDS-aligned archives.

## §12 Long-Term Archive Integration

Beyond the PDS-aligned archive, the operator designates a
long-term archive that holds operational records (audit logs,
in-cruise navigation history, conjunction history) beyond
end-of-mission. Quarterly deposits round-trip content-
addresses; on end-of-mission, remaining records transfer to
the archive with content-addresses preserved.

## §13 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (launch-licence
status, COSPAR PP category, Radiation Health Office
clearance for crewed missions) to consumers of W3C
Verifiable Credentials MAY re-issue the attestations as
Verifiable Credentials under the Data Model 2.0
specification. Re-issuance is optional; the canonical
record remains the JSON evidence-package manifest.

## §14 Cross-Standard Linkage

Operators that consume adjacent WIA standards (WIA-mars-
mission for the destination-specific records when the
mission is a Mars mission, WIA-moon-base for Earth-Moon
checkout phases, WIA-disaster-relief-coordination for
Earth-side recovery operations) emit cross-standard linkage
records.

## §15 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds with
`Last-Event-ID` resume support; subscribers that disconnect
during long cruise windows resume from the last seen event
identifier without losing visibility of priority-1 events
(conjunction high-Pc, radiation-ledger limit warnings,
consumable-budget breach warnings).

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with
prior-minor clients. Major revisions go through a
deprecation window of at least one full mission planning
cycle (typically two Earth years for interplanetary missions).

## §17 Public Catalogue and Aggregator Feeds

Operators publish a public catalogue of released science
products, end-of-mission reports, and PP disposition records
through an Atom or JSON Feed. The feed never carries crew-
member identity for crewed missions; ledger-related public
disclosures are aggregate-only.

## §18 Reader Tooling

Operators MAY publish supplementary reader tools (visual
trajectory plots, conjunction-timeline maps, consumable-
budget burn-down charts, ledger trend charts for crew teams
with appropriate authorisation) alongside the canonical
evidence package; the tools are non-normative.

## §19 Migration from Pre-Standard Records

Operators of legacy missions that pre-date WIA-interplanetary-
travel MAY migrate historical records by emitting synthetic
mission records with a `legacyImport` flag. Synthetic
missions are accepted by the public catalogue but are not
eligible for evidence-package generation without
contemporaneous re-validation against PHASE-3 §3
expectations.

## §20 End-of-Mission and Disposition Integration

End-of-mission disposition records (PHASE-3 §8) flow to the
operating jurisdiction's debris office and to the
operator's long-term archive. Integration carries the debris
office's identifier and the per-disposition-class submission
template; disposition decisions become public after the
disposition is executed and confirmed.

## §21 Anomaly Investigation Board Integration

Mission-loss or off-nominal arrival events trigger an
anomaly investigation board. Integration with the agency's
incident-response infrastructure carries the board's
identifier, the per-incident chair, the documentation
template the board adopts, and the board's published
findings-disclosure schedule. Findings flow back into the
mission's audit chain and into the operating jurisdiction's
post-event report.

## §22 Spectrum Coordination Integration

Operators integrate with the operating jurisdiction's
spectrum management authority and with the ITU-R BR for
deep-space frequency assignments under Article 22 of the
Radio Regulations. Integration carries the assigned
frequencies per band, the per-frequency protection-criteria
attestation, and the operator's compliance with the deep-
space band coordination notice procedures.

## §23 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with at least one launch service provider, at
least one deep-space tracking network, the COSPAR Panel on
Planetary Protection, the operating jurisdiction's space
authority, at least one PDS-aligned archive, and (for crewed
missions) the operating agency's Radiation Health Office,
and has published at least one externally citable evidence
package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-interplanetary-travel
- **Last Updated:** 2026-04-28
