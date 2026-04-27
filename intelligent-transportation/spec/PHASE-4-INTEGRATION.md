# WIA-intelligent-transportation PHASE 4 — INTEGRATION Specification

**Standard:** WIA-intelligent-transportation
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited intelligent-transportation
programme integrates with the systems that surround it: national
spectrum regulators; type-approval bodies; SCMS operators;
neighbouring TMCs (mutual aid); transit agencies; emergency-
services dispatch (CAD); journey-planning aggregators; map
vendors; weather-information service providers; long-term
archives; and citation tools that resolve published incident or
research reports to their evidence packages.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- IEEE 1609.2-2022 (V2X security services)
- ETSI TS 102 941 (security credential management)
- TMDD v3.1 (mutual-aid messaging)
- GTFS-Realtime v2.0
- SIRI v2.1
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Spectrum Regulator Integration

The national radio regulator is the primary external counterpart
for the programme's RSU radio operations. Integration carries the
regulator identifier, the programme's spectrum licence reference,
the per-band masks that the licence authorises, and the
notification cadence the regulator requires (typically annual
operating reports plus per-incident notifications for harmful
interference).

## §2 Type-Approval Body Integration

Type-approval bodies emit signed certificates per RSU and OBU
hardware revision. The integration record carries the type-
approval reference, the issuing date, and the validity expiry.
Type-approval revisions that lapse trigger an alert through the
streaming subscription so that the operator can plan radio-stack
updates in advance.

## §3 SCMS Operator Integration

The SCMS operator emits long-term enrolment certificates and
pseudonym certificates per IEEE 1609.2 / ETSI TS 102 941. The
integration carries the SCMS operator's identifier, the
programme's enrolment authority binding, the pseudonym
provisioning cadence, and the misbehaviour reporting endpoint.
SCMS-side certificate revocations refresh the programme's CRL
cache per the SCMS operator's distribution schedule.

## §4 Mutual-Aid Partner TMC Integration

Adjacent TMCs exchange incidents, detours, and signal-coordination
state through TMDD v3.1 messaging over a mutually-authenticated
TLS channel. The integration record carries each partner's
identifier, the mutual-aid agreement reference, and the per-
record-class authorisation matrix.

Cross-boundary detour acknowledgement (PHASE-3 §6) flows through
this integration: operator A publishes the detour proposal,
operator B acknowledges or rejects, and operator A activates the
detour only when acknowledgement is received.

## §5 Transit Agency Integration

Transit agencies publish GTFS-Realtime VehiclePositions,
TripUpdates, and Alerts feeds. The programme's WIA facade
consumes the feed and re-emits position records (PHASE-1 §9)
that resolve against the network reference (PHASE-1 §3) so that
journey-planning consumers can correlate transit positions with
arterial signal-coordination state.

## §6 Emergency-Services CAD Integration

Computer-Aided Dispatch systems consume incident notifications
and emit emergency-vehicle preemption requests. The integration
record carries the dispatch agency's identifier, the per-incident-
class subscription scope, and the preemption authorisation
matrix recorded in PHASE-3 §7.

## §7 Journey-Planning Aggregator Integration

Multimodal journey-planning aggregators consume incident,
detour, and transit-position records. The integration is read-
only and is rate-limited; aggregator client certificates are
issued by the operator and bound to the aggregator's terms-of-use
agreement.

## §8 Map Vendor Integration

Map vendors consume the network reference records (PHASE-1 §3)
and emit vendor-internal identifiers that the WIA facade can
look up at decode time (PHASE-2 §4). The integration carries
the map vendor's identifier, the cadence of map-version refresh,
and the dispute-resolution path when a network reference no
longer resolves cleanly to a vendor identifier.

## §9 Weather-Information Provider Integration

Weather-impact incidents (PHASE-1 §8 classification
`weather-impact`) are sourced from weather-information providers
operating per WMO conventions. The integration record carries
the weather provider's identifier, the per-area subscription, and
the latency budget for impactful weather notifications (typically
under 90 seconds for high-impact phenomena).

## §10 Evidence Package Format

The evidence package is the externally-citable artefact for an
incident, a research dataset, or a programme audit. It is
produced by the API endpoint at `/v1/evidence` and is a tarball
with the following layout:

```
evidence/
  manifest.json                — package manifest (signed, see §11)
  programme.json               — programme record
  incidents/                   — incident records and root-cause
                                  references
  detours/                     — detour plans
  rsus/                        — RSU records, firmware history
  obus/                        — OBU summaries (no per-OBU PII)
  v2x-captures/                — capture metadata and archive
                                  references (privacy-filter
                                  version recorded)
  signal-control-states/       — control-state windows for the
                                  cited interval
  transit-positions/           — transit positions for the cited
                                  interval
  mutual-aid/                  — mutual-aid exchange logs
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is signed by
the operator's HTTP-message-signature key (RFC 9421) and counter-
signed by the regulator when the package supports a regulatory
submission.

## §11 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:intelligent-transportation:evidence-mismatch`.

## §12 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-intelligent-transportation` that links to the
API root, the public spectrum-licence summary, the published
quality dossier, the catalogue of material incidents, and the
mutual-aid partner list.

## §13 Long-Term Archive Integration

Programmes designate a long-term archive that holds operational
records (V2X capture archives at their declared retention,
incident root-cause investigations, audit logs) beyond the
programme wind-down horizon. Quarterly deposits round-trip
content-addresses; on wind-down, remaining records transfer to
the archive with content-addresses preserved.

## §14 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (electric-grid
for electrified bus depots, infrastructure for monitored bridges
on the road network, infrastructure-monitoring for SHM data on
those bridges) emit cross-standard linkage records that name
the consuming standard and the version under which the linkage
is claimed.

## §15 Public Catalogue and Aggregator Feeds

Programmes that publish a public catalogue of material incidents
emit an Atom or JSON Feed listing incidents with their evidence-
package manifest digests, the affected reference, the severity,
and the resolution timeline. The feed is intended for
transparency to the public served by the programme.

## §16 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose attestations (spectrum licence
status, type-approval validity, ISO 39001 conformance) to
consumers of W3C Verifiable Credentials MAY re-issue the
attestations as Verifiable Credentials under the Data Model 2.0
specification. Re-issuance is optional; the canonical record
remains the JSON evidence-package manifest.

## §17 Streaming Heartbeat and Replay

SSE subscribers receive a heartbeat every 30 seconds with
`Last-Event-ID` resume support. Subscribers that disconnect
during long incident or weather windows resume from the last
seen event identifier without losing visibility of priority-1
incident events.

## §18 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window
of at least one full TMDD revision cycle so that mutual-aid
partner integrations have time to migrate.

## §19 Reader Tooling for Operations

Operators, regulators, and certifying bodies benefit from
visualisation tools that surface incident-timeline maps, signal-
coordination plans, and V2X capture summaries. Programmes MAY
publish reader tools alongside the canonical evidence package;
the tools are non-normative.

## §20 Federation Adapter for Multi-TMC Operators

Operators that run multiple TMCs (regional authority operating
several metropolitan TMCs) integrate through a federation
adapter that translates between per-TMC WIA records and a
combined view. The adapter honours each TMC's authorisation
scope so that federated queries respect the source TMC's
spectrum licence and IEC 62443 zone classification.

## §21 Programme Wind-Down

A programme that ceases operations transfers indefinite-retention
records to a recognised long-term archive, notifies regulators
and mutual-aid partners, and publishes a sunset timeline for
in-flight detours and incidents. Roadside infrastructure is
decommissioned per PHASE-3 §9 and SCMS certificates are revoked
through the SCMS operator integration of §3.

## §22 CAV Mapping Service Integration

Connected and automated vehicle (CAV) mapping services consume
work-zone CAV advisories (PHASE-1 §11 `cavAdvisoryRef`) and
incident detour plans so that CAV planning stacks can adjust
trajectories in advance of the operator's roadside broadcast.
The integration carries the mapping service's identifier and the
per-record-class subscription scope; CAV mapping services do not
write back into the operator's records.

## §23 Probe-Vehicle Aggregator Integration

Probe-vehicle aggregators (commercial telematics providers,
smartphone-based location services, freight-fleet telematics
providers) emit anonymised speed and travel-time observations.
The integration consumes the probe feed against the operator's
network reference (PHASE-1 §3) so that operator analytics can
correlate probe-derived speeds with signal-control state and
incident posture. Probe data is consumed under data-licence
agreements that the operator records in the integration record;
data licences that limit re-publication are honoured at the
operator's downstream evidence-package boundary.

## §24 Air-Quality Monitor Integration

Roadside and near-road air-quality monitors emit pollutant
observations (NO2, PM2.5, PM10, ozone) that operators consume to
support low-emission-zone enforcement, dynamic congestion
pricing, and public-health reporting. The integration carries
the monitor operator's identifier, the per-monitor calibration
chain (per ISO/IEC 17025 §3 of the adjacent WIA-air-quality
standard where applicable), and the publication cadence.

## §25 Multimodal Trip-Planner Open Standards

Programmes that publish trip-planning data into open
multimodal-trip-planner ecosystems emit GTFS, GTFS-RT, GBFS,
SIRI, and OJP feeds through the operator's open-data portal.
The integration carries each consuming planner's identifier,
the per-feed access cadence, and the per-feed update latency
budget that planners can rely on.

## §26 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with the spectrum regulator, at least one type-approval body,
the SCMS operator, at least one mutual-aid partner TMC, at
least one transit agency, the relevant CAD system, and at least
one long-term archive, and has published at least one externally
citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-intelligent-transportation
- **Last Updated:** 2026-04-28
