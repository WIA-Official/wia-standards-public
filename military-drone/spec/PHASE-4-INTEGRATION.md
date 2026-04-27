# WIA-military-drone PHASE 4 — Integration Specification

**Standard:** WIA-military-drone
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how a UAS deployment integrates the data, APIs,
and protocols from PHASEs 1–3 with the operational picture: airframe
fleet management, ground control station infrastructure, mission
planning system, civil-airspace coordination, video / KLV storage,
fire-control consumers (WIA-missile-defense), and coalition exchange.
It is non-prescriptive about specific vendors; it specifies the
integration *contracts* a deployment must satisfy.

References (CITATION-POLICY ALLOW only):
- STANAG 4586 — UAV Control System interfaces
- STANAG 4609 — NATO Digital Motion Imagery (MISB)
- ICAO Doc 4444 — air-traffic management procedures
- ICAO Annex 2 — Rules of the Air
- WIA-missile-defense (PHASE 1–4) — for engagement-decision cross-reference
- WIA-military-communication (PHASE 1–4) — for cross-coalition transport
- WIA-laser-weapon (PHASE 1–4) — for UAS-mounted laser-designator integration

---

## §1 Airframe fleet registry

The deployment maintains a registry of every fielded airframe:

- `airframeRef` — URN
- vendor, model, serial number
- MTOW band, endurance band, payload capability set
- supported link classes
- flight-control firmware version + last update date
- TLS client certificate fingerprint and expiry
- maintenance schedule + last-flight-hour count
- operating-area policy (geographic + RoE scope)

An airframe not in the registry, or with expired certificate or
firmware out of currency, is refused at PHASE 2 §1 mission-plan
intake.

## §2 Ground control station integration

GCS integration follows STANAG 4586:

- The GCS authenticates with mTLS + JWS-signed messages
- Operator identity is bound to the GCS's authentication; multi-
  operator GCSs use per-operator authentication for accountability
- The GCS forwards mission-plan submissions and re-tasking to the
  boundary; the boundary returns the canonical record URN
- The GCS subscribes to airframe state, payload state, and link
  state streams via PHASE 2 §2

Multi-GCS deployments (one airframe controlled by different GCSs at
different mission phases) coordinate via the boundary's hand-over
records.

## §3 Mission planning system

The mission planner is a separate service that owns:

- Route construction (waypoints, legs, climb/descent profiles)
- Geofence drafting
- Civil-airspace coordination message generation (ICAO-compatible
  flight plans, NOTAM submission, dynamic re-route)
- Lost-link behaviour selection
- Weapon-loadout planning (for strike missions)

The planner submits the resulting mission plan via PHASE 2 §1; the
boundary's role is gatekeeping and audit.

## §4 Civil-airspace coordination

UAS operating in civil airspace require coordination with ATC:

- ICAO Annex 2 rules apply except where the deployment's national
  authority exempts (e.g., dedicated military-only airspace,
  pre-coordinated training zones)
- Pre-flight: NOTAM submission and acknowledgement
- In-flight: ATC clearance for any altitude / route changes
- Post-flight: NOTAM closure

The deployment policy enumerates which airspaces require which
level of coordination; the boundary refuses missions whose route
crosses unauthorised airspace.

## §5 Video / KLV storage

Full-motion video and associated KLV metadata flow to the
deployment's video-storage system:

- Video transport: per-airframe video link (FMV stream over the C2
  bandwidth or a dedicated wideband link)
- KLV metadata: per-frame KLV stream paired with the video
- Storage: video-indexed-by-time + KLV-indexed-by-time; the
  deployment's playback tools join them for analysis
- Retention: per-airspace retention policy (typically ≥ 12 months
  for ISR, ≥ 7 years for strike-mission video)

PHASE 2 §3 observation publication references the storage URI; the
canonical record at the boundary references the storage URI rather
than duplicating the bulk data.

## §6 Fire-control consumer (WIA-missile-defense)

For strike missions, the WIA-missile-defense fusion engine is a
consumer of the airframe's track and the released munition's track:

- Airframe state stream feeds the fusion engine (the airframe is a
  friendly track)
- Released munition (PHASE 1 §6) creates a new munition-track in
  WIA-missile-defense via cross-domain reference
- Outcome (kinetic-kill / miss / partial) is recorded in
  WIA-missile-defense PHASE 1 §8 outcome record with reference back
  to the WIA-military-drone release record

This bridge ties two audit chains across the strike lifecycle.

## §7 Coalition exchange

Multi-national coalition operations exchange mission plans, observations,
and (where authorised) weapon-release authority bilaterally under a
federation manifest:

- Manifest enumerates which records flow which direction, which
  airframes each party may control on the other's airspace, and the
  release authorities for cross-party content
- The boundary honours the manifest at every cross-national release
- Manifest expiry suspends cross-national flows; renewal is signed
  and recorded as an AuditEvent

Ad-hoc cross-coalition tasking outside the manifest requires both
nations' release officers to sign individually.

## §8 Operational SLAs

| Concern                                          | Default SLA              |
|--------------------------------------------------|--------------------------|
| Mission-plan submission p95 latency              | ≤ 200 ms                 |
| Airframe state stream sample latency             | ≤ 100 ms (1-10 Hz stream)|
| Weapon-release authorisation turnaround          | ≤ 500 ms                 |
| Sensor observation publication                   | ≤ 1 s after sensor frame |
| Lost-link detection threshold                     | per airframe class (typ. 30 s) |
| Audit chain entry available after operation      | ≤ 1 s                    |
| RoE authorisation cache refresh                   | ≤ 5 minutes              |
| Civil-airspace coordination acknowledgement       | per ATC SLA              |

Tighter SLAs negotiable per deployment; loosening requires
operational sign-off.

## §9 Acceptance criteria

A deployment claims conformance when:

1. Every fielded airframe is in the registry with current certificate
   and firmware
2. Every mission in the past quarter has a matching audit chain entry
   with verifiable inclusion proof
3. Every weapon-release event has dual signatures on file
4. Every civil-airspace overlap has matching ATC coordination evidence
5. Geofence-evidence records are on file for every mission
6. Lost-link events have recovery records or mishap-investigation
   evidence
7. Cross-coalition releases have both release-authority signatures
8. Quarterly compliance report has no integrity-check failures

A deployment failing any of these reports the gap in its compliance
package rather than concealing it.

## Annex A — Common pitfalls (informative)

- **Civil-airspace dynamic re-route mismatch** — ATC issues dynamic
  re-routes that differ from the airframe's filed flight plan; the
  GCS must propagate the re-route into the mission record before the
  airframe acts on it (or the geofence-evidence record will show a
  breach the deployment cannot defend at audit)
- **KLV metadata staleness** — KLV streams have per-frame timestamps;
  storage that loses synchronisation between video and KLV produces
  unreplayable observations. The deployment SHOULD validate
  video-KLV alignment at storage ingest
- **Lost-link behaviour mismatch** — operator-set and airframe-
  programmed lost-link behaviour MUST agree; mismatch surfaces only
  at link-loss, when recovery is impossible
- **Multi-GCS hand-over drift** — hand-over events lose audit
  visibility if the prior GCS's session ends before the receiving
  GCS's begins; the deployment SHOULD ensure overlap or use signed
  hand-over records

## Annex B — Decommissioning (informative)

When an airframe is decommissioned: outstanding missions complete or
transfer to a successor airframe, the system is removed from the
registry, final flight-hour count is recorded, and audit records are
preserved per WIA-payment-system retention rules. The decommissioning
manifest is itself an audit event.

## Annex C — Quarterly compliance report (informative)

The boundary emits a quarterly compliance report covering total
missions by type, observation volumes, weapon-release events with
authorisation chain, geofence-breach incidence, lost-link events
and recovery rates, civil-airspace coordination status, federation
peer activity, and audit-chain integrity check results.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex D — Worked civil-airspace coordination (informative)

Pre-flight:

1. Mission planner constructs the route + flight plan
2. Mission planner submits NOTAM to the controlling national authority
3. National authority issues NOTAM identifier; coordination message
   includes the NOTAM URN
4. Mission plan PHASE 2 §1 submission includes
   `civilAirspaceCoordinationRef` referencing the NOTAM
5. Boundary verifies NOTAM exists and covers the route

In-flight dynamic change:

1. ATC issues a re-route via voice or CPDLC
2. Operator confirms; mission planner updates the route + NOTAM
3. Re-tasking via PHASE 2 §1 with the updated coordination reference
4. Boundary records the change; the airframe acts on the new route

Post-flight:

1. NOTAM is closed at the controlling authority
2. Recovery record references the NOTAM closure for audit completeness

## Annex E — Versioning and deprecation (informative)

Versioning follows Semantic Versioning 2.0.0. STANAG 4586 versions
bump independently from this PHASE; the deployment policy maps each
PHASE version to the STANAG 4586 version it honours so coalition
partners know what their counterpart's GCS expects. Deprecation
enters a 12-month sunset window with migration notes recorded in
the audit chain.

## Annex F — Lost-link mishap investigation (informative)

A lost-link event whose mishap-flag is true triggers a formal
investigation:

1. Audit-chain entries from the mission start through the lost-link
   window are preserved in a forensic store
2. Last-known airframe state, lost-link behaviour, mission-plan
   parameters, weather + airspace conditions are extracted
3. If physical evidence is recoverable, it is preserved under
   ISO 17025-style chain-of-custody
4. A written investigation report is filed within 30 days
5. Lessons-learned are fed back to the mission-planning system
   (e.g., updated geofence margins, revised lost-link behaviour
   defaults, revised airframe-class operational envelope)

The investigation report is itself signed and recorded in the
audit chain.

## Annex G — Quarterly compliance report (informative)

The boundary emits a quarterly compliance report covering:

- Total missions by type and outcome
- Sensor-observation volumes by sensor type and storage destination
- Weapon-release events with authorisation chain
- Geofence-breach incidence by airframe and corridor
- Lost-link events and recovery rates
- Civil-airspace coordination acknowledgement timeliness
- Federation peer activity (cross-coalition flows)
- Audit-chain integrity check results

The report is signed and is itself in scope for the audit chain.
