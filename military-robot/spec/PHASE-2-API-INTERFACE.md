# WIA-military-robot PHASE 2 — API Interface Specification

**Standard:** WIA-military-robot
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a military-robot deployment exposes
for mission-plan submission and re-tasking, platform / link state
reporting, sensor observation publication, weapon-release authorisation
and event reporting, geofence evidence collection, autonomy-level
declaration, mishap reporting, and lost-link / link-recovery records.
The shape is HTTP/JSON for C2 / mission planners; tactical fire-control
and tele-operation use the constrained binary mapping described in
PHASE 3.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS), RFC 8259 (JSON)
- STANAG 4856 — UGV common architecture
- STANAG 4677 — Dismounted Soldier Reference Architecture
- STANAG 5516 — for J11.x cross-domain weapon-direction
- IEC 61508 — Safety Integrity Levels
- WIA-military-drone PHASE 2 — referenced for shared GCS endpoints
- WIA-missile-defense PHASE 2 — referenced for cross-domain weapon-release

---

## §1 Mission-plan submission

```
POST /missions HTTP/1.1
Host: rob.coalition-c2.example
Authorization: Bearer eyJhbGciOiJFUzI1NiJ9...
Content-Type: application/wia-md-rob+json
WIA-Doctrine: NATO-AAP-15

{
  "missionId": "urn:wia:md-rob:mission:rok-army:m-91a7-2026-04-27",
  "platformRef": "urn:wia:md-rob:platform:rok-army:eod-bot-12",
  "missionType": "eod-investigation",
  "route": [...waypoints...],
  "geofence": {...},
  "autonomyLevel": "operator-on-loop",
  "rulesOfEngagement": "urn:wia:md-rob:roe:rok-army.adcc:eod-rules-2026",
  "lostLinkBehaviour": "safe-shutdown-in-place"
}
```

The boundary validates the platform's registry presence, the route
against the platform's mobility profile, the autonomy-level declaration
against the platform's safety SIL, and the operations-authority
signature, then creates the mission record. Re-tasking uses
`PUT /missions/<missionId>/$retask`.

## §2 Platform / link state

The platform pushes state events:

```
POST /platforms/<platformRef>/state HTTP/1.1
{
  "stateTimestamp": "2026-04-27T09:31:14.523+09:00",
  "position": {"lat": 37.5, "lon": 127.0, "alt": 0},
  "velocity": {...},
  "attitude": {...},
  "energyRemainingPct": 0.78,
  "linkStates": [...],
  "manipulatorState": {...},
  "sensorPayloadState": {...}
}
```

Streamed at 1 Hz routine, 10 Hz during armed operations or critical
manoeuvres. Subscribers consume via Server-Sent Events on
`/platforms/<platformRef>/state/$subscribe`.

For UUVs operating below acoustic-modem latency tolerance, state
events are buffered locally and flushed to the boundary on next
acoustic uplink window; the boundary inserts them into the chain
at the recorded timestamps.

## §3 Sensor observation publication

```
POST /observations HTTP/1.1
{
  "obsId": "urn:wia:md-rob:obs:eod-bot-12:2026-04-27T09:31:14:0001",
  "platformRef": "urn:wia:md-rob:platform:rok-army:eod-bot-12",
  "payloadRef": "...",
  "sensorType": "chemical-sniffer",
  "geometry": {...},
  "targetLocation": {...},
  "observationTimestamp": "...",
  "observationProductRef": "...",
  "metadataRef": "..."
}
```

NBC-sensor observations cross-reference WIA-nbc-defense PHASE 2 §1
for ingestion into the NBC-defence operational picture.

## §4 Weapon-release authorisation (armed platforms)

```
POST /releases/$authorise HTTP/1.1
{
  "missionRef": "...",
  "decisionRef": "...",
  "munitionType": "less-lethal-disrupt",
  "targetRef": "...",
  "aimpoint": {...},
  "releaseConditions": {...}
}
```

Dual signatures required (operator + release authority). The boundary
also verifies the platform's autonomy-level declaration permits the
release category at the declared autonomy level (e.g., a
`bounded-autonomy` platform cannot release lethal munitions without
explicit operator-on-loop confirmation).

## §5 Weapon-release event reporting

```
POST /releases HTTP/1.1
{
  "releaseId": "urn:wia:md-rob:release:eod-bot-12:r-91a7",
  ...all PHASE 1 §6 fields...
}
```

The boundary chains the release to authorisation and emits an
AuditEvent. Outcome reporting follows WIA-missile-defense PHASE 1 §8
shape with `outcomeSourceRobotRelease` reference.

## §6 Geofence evidence collection

```
POST /platforms/<platformRef>/geofence-evidence HTTP/1.1
{
  "geofenceEvidenceId": "...",
  "legRef": "...",
  "actualTrack": [...],
  "geofenceBreachEvents": []
}
```

UUV geofence evidence includes 3D positions (lat/lon/depth).

## §7 Autonomy-level declaration

```
POST /missions/<missionRef>/autonomy-declaration HTTP/1.1
{
  "declaredAutonomyLevel": "bounded-autonomy",
  "humanOnLoopOperator": "urn:wia:md-rob:operator:...",
  "escalationThresholds": {...},
  "safetyEnvelope": {"sil": 2, "envelope": {...}},
  "declaringAuthority": "..."
}
```

The declaration is signed; subsequent platform behaviour is monitored
against the declaration and exceedances trigger alerts.

## §8 Mishap reporting

```
POST /mishaps HTTP/1.1
{
  "mishapType": "geofence-breach-with-consequence",
  "incidentTimestamp": "...",
  "incidentLocation": {...},
  "platformRef": "...",
  "missionRef": "...",
  "priorAuditWindowMinutes": 30,
  "investigationLeadAuthority": "..."
}
```

The boundary creates the mishap record, freezes the platform's
session token, snapshots the audit-chain window for forensic
preservation, and starts the investigation workflow.

## §9 Errors and warnings

| URI                                              | Status | Meaning                                       |
|--------------------------------------------------|-------:|-----------------------------------------------|
| `urn:wia:md-rob:problem:platform-not-registered` | 404    | platform URN not in registry                   |
| `urn:wia:md-rob:problem:autonomy-mismatch`       | 422    | declared autonomy level exceeds platform SIL  |
| `urn:wia:md-rob:problem:dual-signature-required` | 403    | weapon-release lacks signatures                |
| `urn:wia:md-rob:problem:roe-not-authorised`      | 403    | mission RoE outside operator's authorisation   |
| `urn:wia:md-rob:problem:envelope-violation`      | 422    | platform behaviour exceeds declared envelope   |
| `urn:wia:md-rob:problem:link-prerequisite-missing`| 503   | mandated link not currently tracking          |
| `urn:wia:md-rob:problem:audit-unavailable`       | 503    | audit chain write failed                       |

Warnings (200-OK with caveats) use `Warning:` headers per RFC 7234
§5.5 with codes namespaced under `wia-md-rob-`.

## Annex A — Capability advertisement (informative)

```
GET /.well-known/wia/military-robot/capabilities HTTP/1.1
```

Response advertises supported platform domains (UGV/USV/UUV),
supported autonomy levels, supported payload types, RoE authorities,
federation peers, and conformance level. Capability documents are
signed.

## Annex B — Pagination and rate limiting

State and observation queries paginate at ≤ 1000 entries per page.
Per-token rate limits default to 10 mission-plan submissions per
hour per operator, 1000 state samples per second per platform.

## Annex C — Idempotency

Mission-plan submissions and weapon-release authorisations accept
`Idempotency-Key`; retries within 24 h with the same key return the
original response.

## Annex D — Worked autonomy escalation (informative)

A `bounded-autonomy` route-clearance robot encounters an obstacle
not in its envelope:

1. Robot self-flags `envelope-violation`
2. Boundary alerts the operator on the GCS
3. Operator reviews sensor data and either:
   - Authorises a route deviation (raising autonomy level for
     this leg only), signed via PHASE 2 §7
   - Aborts the mission (`PUT /missions/<>/$abort`)
4. Both decisions are recorded in the audit chain

## Annex E — Worked mishap investigation (informative)

After a mishap is reported (PHASE 2 §8):

1. Investigation lead retrieves the audit-chain window
2. Forensic store provides the prior 30 minutes of state, observations,
   and link transitions
3. Findings (e.g., autonomy-envelope misconfigured, sensor-data
   ambiguity) are recorded as additional entries on the mishap
4. Corrective actions (firmware update, doctrine revision, retraining)
   are linked from the mishap record
5. Investigation closes within 30 days; extensions require operations-
   command sign-off

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex F — Worked weapon-release authorisation (informative)

```
POST /releases/$authorise HTTP/1.1
Authorization: Bearer <operator-token>
WIA-Operator-Signature: <JWS>
WIA-Release-Authority-Signature: <JWS>

{
  "missionRef": "urn:wia:md-rob:mission:rok-army:m-91a7-2026-04-27",
  "decisionRef": "urn:wia:md-rob:decision:eod-disrupt:d-91a7",
  "munitionType": "less-lethal-disrupt",
  "targetRef": "urn:wia:md-rob:target:eod-bot-12:t-91a7",
  "aimpoint": {"lat": 37.4516, "lon": 126.6531, "alt": 0},
  "releaseConditions": {"standoffM": 6, "approachAngle": 90}
}
```

Response on accepted:

```
201 Created
Location: /releases/r-91a7
WIA-Audit-Event-Id: urn:wia:md-rob:audit:release-auth:7f2c
```

The boundary verifies dual-signatures, RoE authorisation, autonomy-
level compatibility (`bounded-autonomy` permits the requested release
type), and the platform's current state allows the release conditions
before issuing the fire instruction.

## Annex F — Worked NBC-sensor cross-ingest (informative)

A robot's chemical-sniffer detects a Class-A agent; the observation
flows both into the robot's PHASE 2 §3 stream and into WIA-nbc-defense
PHASE 2 §1 ingest:

```
POST /observations HTTP/1.1
{
  "obsId": "urn:wia:md-rob:obs:eod-bot-12:o-91a7",
  "sensorType": "chemical-sniffer",
  ...all PHASE 1 §5 fields...,
  "crossDomainNbcRef": "urn:wia:nbc:event:eod-bot-12:e-91a7"
}
```

The boundary forwards the same observation to the WIA-nbc-defense
boundary with the cross-domain reference; the NBC-defense fusion engine
correlates with other sensors and may promote the agent identification
to `confirmed`.

## Annex G — Capability discovery (informative)

```
GET /.well-known/wia/military-robot/capabilities HTTP/1.1
Accept: application/json
```

Response advertises supported domains (ugv/usv/uuv), supported
autonomy levels, supported payload types, RoE authorities, federation
peers, conformance level, and per-domain idiomatic fields (e.g.,
`uuv.acousticBandwidthBudgetKbps`, `ugv.maxObstacleHeightM`).
Capability documents are signed.

## Annex H — Operational SLAs (informative, restated)

| Concern                                          | Default SLA              |
|--------------------------------------------------|--------------------------|
| Mission-plan submission p95 latency              | ≤ 200 ms                 |
| Platform state stream sample latency (RF/wired)  | ≤ 100 ms                 |
| Acoustic-modem state-flush per dive (UUV)        | per acoustic window      |
| Weapon-release authorisation turnaround          | ≤ 500 ms                 |
| Mishap response — token freeze                   | ≤ 5 s                    |
| Mishap response — chain snapshot                 | ≤ 60 s                   |
| Audit chain entry available after operation      | ≤ 1 s (live link); per acoustic window (UUV) |

## Annex I — Worked acoustic-modem flush (informative, UUV)

After surfacing, the UUV pushes its locally-buffered records:

```
POST /platforms/<platformRef>/state/$flush HTTP/1.1
Authorization: Bearer ...
Content-Type: application/wia-md-rob+json

{
  "platformRef": "urn:wia:md-rob:platform:rok-navy:uuv-7",
  "diveStart": "2026-04-27T07:00:00+09:00",
  "diveEnd": "2026-04-27T15:00:00+09:00",
  "buffer": [
    {"stateTimestamp": "...", "position": {...}, "inferredFromInsAtTimeRef": "...", "insDriftEstimateMetres": 4.2, ...},
    ...
  ]
}
```

The boundary inserts each record into the audit chain at the
recorded timestamp + INS-drift annotation. Records whose timestamp
falls inside an already-sealed daily root are routed to the
late-arrival queue.

## Annex J — Idempotency policy (extension)

Mission-plan submission, weapon-release authorisation, autonomy declaration submission, and mishap reporting accept Idempotency-Key. Boundary stores the key for 24 h; retries within window with same key return original response. Different bodies under the same key return urn:wia:md-rob:problem:idempotency-conflict.

## Annex K — Worked status query
GET /platforms/<platformRef>/status returns the current state record + active mission ref + active autonomy declaration. Useful for periodic dashboards.
