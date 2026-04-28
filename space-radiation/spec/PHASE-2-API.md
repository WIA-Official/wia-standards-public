# WIA-SPACE-009 — Phase 2: API Interface

> Space-radiation canonical Phase 2: API surface (missions + dose-records + dosimeters + SPE-alerts + telemetry + audit).

# WIA-SPACE-009: Space Radiation Protection Standard
## Version 1.0

**Status:** Active
**Published:** 2025-01-26
**Category:** Space Safety / Radiation Protection
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---


## 5. Monitoring Systems

### 5.1 Personal Dosimeters

**Passive Dosimeters:**
- TLD (Thermoluminescent Dosimeter): Minimum 2 per crew member
- OSL (Optically Stimulated Luminescence): Backup dosimeter
- Read-out frequency: Weekly minimum, post-mission

**Active Dosimeters:**
- Real-time dose and dose rate display
- Alarming capability (threshold: 2x background)
- Data logging (minimum 1-hour resolution)
- Wireless data transmission to health monitoring system

**Required Measurements:**
- Absorbed dose (Gy)
- Dose equivalent (Sv)
- Dose rate (mSv/day)
- LET spectrum (for quality factor determination)

### 5.2 Area Monitoring

**Fixed Monitors:**
- Location: All habitable modules
- Measurement: Continuous dose rate
- Alert: Automatic when dose rate exceeds threshold
- Data: Archived for mission duration

**Environmental Monitors:**
- External radiation environment
- Particle flux and energy spectrum
- Real-time SPE detection

### 5.3 Data Management

**Real-time Requirements:**
- Data transmission: Every 24 hours minimum
- Emergency alerts: Immediate transmission
- Ground analysis: Daily dose assessment
- Crew notification: Within 1 hour of analysis

**Database:**
- Individual cumulative dose tracking
- Historical mission data
- Comparison with models
- Long-term health correlation

---




---

## A.1 Endpoint reference

```http
POST /space-radiation/v1/missions                  # register mission
GET  /space-radiation/v1/missions/{id}             # fetch mission
POST /space-radiation/v1/dose-records              # submit dose
GET  /space-radiation/v1/dose-records/{id}         # fetch dose
GET  /space-radiation/v1/dose-records/crew/{cid}   # crew dose history
POST /space-radiation/v1/dosimeters                # register dosimeter
GET  /space-radiation/v1/dosimeters/{id}/state     # dosimeter state
POST /space-radiation/v1/spe-alerts                # SPE alert ingest
WS   /space-radiation/v1/state/stream              # real-time stream
GET  /space-radiation/v1/audit/{id}                # audit trail
```

Every endpoint follows the discovery convention at
`/.well-known/wia-space-radiation`. Crew-dose-record endpoints
require operator-medical-officer credential plus NASA Privacy Act /
HIPAA / equivalent crew-protected-data scope.

## A.2 Mission-registration API

`POST /missions` accepts the Phase 1 §A.6 envelope. The endpoint
validates the trajectory envelope against the operator's mission-
manifest database, resolves the shielding-configuration envelope
to the spacecraft-of-record's design data, and computes the
mission-radiation-budget envelope using the operator-selected
HZETRN configuration. Mission-budget envelopes carry an explicit
confidence percentile, an explicit worst-case SPE assumption, and
an explicit GCR solar-modulation assumption; downstream consumers
MUST use these assumptions consistently.

## A.3 Dose-record submission API

`POST /dose-records` accepts the Phase 1 §A.2 envelope. The endpoint
deduplicates by content-addressed SHA-256 hash, links the dose
record to the registered crew member and dosimeter envelope,
computes the rolling 30-day and annual dose-totals against the
§A.3 risk-criterion envelope, and emits a dose-threshold alert
when the rolling total approaches 50% / 75% / 90% of the
applicable limit. Crew-personal dose data is encrypted at rest with
a key bound to the operator's medical-officer credential per
NIST SP 800-57 Part 1 §6.

## A.4 Dosimeter-management API

`POST /dosimeters` registers a dosimeter with the §A.5 envelope.
Calibration-due dates trigger automatic state transition to
`out-of-calibration` with a warning event; deployment-position
changes are recorded as immutable events. Dosimeter-state retrieval
returns the most recent measured dose-rate, the cumulative dose
since deployment, and the on-orbit health envelope (battery, data
buffer fill, last-contact timestamp for active dosimeters).

## A.5 SPE-alert ingest API

`POST /spe-alerts` accepts SPE-alert envelopes from upstream
forecasters (NOAA SWPC GOES proton-flux, ESA SEPEM, Antarctic
neutron monitors per Bartol Research Institute, real-time CME
reports from SOHO/STEREO/Parker Solar Probe). The endpoint
validates the alert envelope against the operator's alert-policy
(threshold for "minor" / "moderate" / "major" / "extreme" per the
NOAA S-scale), aggregates duplicate alerts from multiple sources,
and pushes a single normalised alert downstream to the SPE-response
protocol per Phase 3 §A.4.

## A.6 Telemetry WebSocket

The state-stream WebSocket multiplexes per-mission events: real-
time dose-rate updates, SPE-alert events, GCR-modulation update
events, dosimeter-state transitions, crew-dose-threshold-crossing
events, and shelter-occupancy events for missions with dedicated
SPE shelters. Subscribers can filter by mission-id, crew-id, dose-
threshold-class, and alert-severity-class. Rate limits: 1000 req/h
authenticated, 10000 req/h trusted-partner. WebSocket subscriptions
are bounded at 100 simultaneous per credential.

## A.7 Audit endpoint envelope

`GET /audit/{id}` returns the immutable audit trail: mission
registration, every dosimeter event, every dose-record submission,
every SPE-alert event, every threshold-crossing event, every
medical-officer review-and-acknowledgement event, every crew-
exposure-deviation event with the corresponding waiver per the
operator's flight-rules document, and the post-mission lifetime
career-dose update event. The audit-trail integrity is anchored
into a Merkle tree per-mission and into the operator's archival
record per NASA NPR 1441.1G or equivalent.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/space-radiation/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-space-radiation-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/space-radiation-host:1.0.0` ships every space-radiation envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/space-radiation.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Space-radiation deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

## Z.6 Logging and observability hooks

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: ISO 8601 UTC timestamp per RFC 3339, host
identifier, tenant identifier, envelope class, envelope identifier,
operation outcome, and an opaque trace identifier propagated end-
to-end per W3C Trace Context (`traceparent` header) so a single
operation can be reconstructed across hosts. Phase 2 surfaces this
trace identifier as the `X-WIA-Trace-Id` response header. Phase 3
protocol exchanges propagate the trace identifier inside the
exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (e.g., Splunk, Elastic, Sumo
Logic, Wazuh, Microsoft Sentinel) per OpenTelemetry semantic
conventions, with `wia.standard.slug` and `wia.standard.phase` as
required attributes.

## Z.7 Versioning, deprecation, and capability discovery

Within the 1.x line, hosts MAY publish a capabilities document at
`/.well-known/wia-space-radiation-capabilities` that enumerates which
optional fields, optional endpoints, and optional protocol exchanges
the host implements. Clients MUST treat unsupported capabilities
as absent rather than as an error condition; a client that needs
a capability the host does not advertise MUST surface a clear
configuration error rather than silently degrade. Hosts moving
from one minor version to the next MUST publish the change in
the host's release notes with the per-capability migration window
per IETF RFC 8594 (Sunset header) + RFC 9745 (Deprecation header)
+ RFC 9651 (Structured Field Values) so machine consumers can
plan migration without waiting for human-channel notification.

## Z.8 Privacy and data-minimisation envelope

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California CPRA
per Cal. Civ. Code §1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Where the envelope ships across jurisdictional borders,
the operator's per-jurisdiction transfer envelope (SCC for EU, UK
IDTA, APEC CBPR, ASEAN MCC) MUST be referenced inside the audit
record. Subject-rights endpoints (access, rectification, erasure,
portability, restriction, objection) compose with WIA-OMNI-API per
its §5 subject-rights surface and need not be re-implemented
per-standard.

弘益人間 — Benefit All Humanity.
