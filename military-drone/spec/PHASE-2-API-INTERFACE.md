# WIA-military-drone PHASE 2 — API Interface Specification

**Standard:** WIA-military-drone
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a UAS deployment exposes for
mission-plan submission and re-tasking, airframe / payload / link
state reporting, sensor observation publication (including KLV-
metadata streaming), weapon-release authorisation and event reporting,
geofence evidence collection, lost-link tracking, and recovery
record. The shape is HTTP/JSON for C2 / mission planners; tactical
fire-control loops use the constrained binary mapping described in
PHASE 3 (J3.x for tracks, J11.x for weapon-direction).

References (CITATION-POLICY ALLOW only):
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS), RFC 8259 (JSON)
- STANAG 4586 — UAV Control System interfaces
- STANAG 4609 — NATO Digital Motion Imagery (MISB)
- MISB ST 0601 — UAS Datalink Local Set
- MISB ST 0903 — Moving-Target-Indicator metadata
- ICAO Doc 4444 — air-traffic management procedures
- WIA-missile-defense PHASE 2 — referenced for cross-domain weapon-direction
- WIA-military-communication PHASE 2 — referenced for cross-coalition release

---

## §1 Mission-plan submission

```
POST /missions HTTP/1.1
Host: uas.coalition-c2.example
Authorization: Bearer eyJhbGciOiJFUzI1NiJ9...
Content-Type: application/wia-md-uas+json
WIA-Doctrine: NATO-AAP-15

{
  "missionId": "urn:wia:md-uas:mission:rok-army:m-91a7-2026-04-27",
  "airframeRef": "urn:wia:md-uas:airframe:rok-army:tail-37",
  "missionType": "isr-named-target",
  "route": [
    {"waypointId": "wp-1", "lat": 37.50, "lon": 127.00, "alt": 6000, "speed": 80, "legDuration": 1800},
    {"waypointId": "wp-2", "lat": 37.45, "lon": 127.05, "alt": 6000, "speed": 80, "legDuration": 600}
  ],
  "geofence": {"type": "Polygon", "coordinates": [[...]]},
  "rulesOfEngagement": "urn:wia:md-uas:roe:rok-army.adcc:roe-2026",
  "lostLinkBehaviour": "return-to-base",
  "civilAirspaceCoordinationRef": "urn:wia:md-uas:atc-coord:icao-2026-04-27-0915"
}
```

The boundary validates the route against geofence + civil-airspace
coordination, the airframe's registry presence, the rules-of-engagement
authorisation, and the operations-authority signature, then creates
the mission record. On rejection the response carries the relevant
problem URI (`urn:wia:md-uas:problem:airspace-conflict`,
`urn:wia:md-uas:problem:roe-not-authorised`, etc.).

Re-tasking in flight uses `PUT /missions/<missionId>/$retask` with
the new route; the prior plan is preserved in version history.

## §2 Airframe / payload / link state

The airframe pushes state events:

```
POST /airframes/<airframeRef>/state HTTP/1.1
{
  "stateTimestamp": "2026-04-27T09:31:14.523+09:00",
  "position": {"lat": 37.5, "lon": 127.0, "alt": 6000},
  "velocity": {"speed": 80, "heading": 270, "vertical": 0},
  "attitude": {"roll": 2, "pitch": -1, "yaw": 270},
  "fuelRemainingPct": 0.78,
  "linkStates": [
    {"linkClass": "los-rf", "state": "tracking", "signalQualityBand": "strong"},
    {"linkClass": "blos-satcom", "state": "tracking", "signalQualityBand": "nominal"}
  ]
}
```

State updates are streamed at the deployment-configured rate
(typically 1 Hz for routine ops, 10 Hz for engagement windows).
Subscribers consume via Server-Sent Events on
`/airframes/<airframeRef>/state/$subscribe`.

## §3 Sensor observation publication

```
POST /observations HTTP/1.1
{
  "obsId": "urn:wia:md-uas:obs:tail-37:2026-04-27T09:31:14:0001",
  "airframeRef": "urn:wia:md-uas:airframe:rok-army:tail-37",
  "payloadRef": "urn:wia:md-uas:payload:tail-37:gimbal-1",
  "sensorType": "eo",
  "geometry": {...},
  "targetGroundFootprint": {...},
  "observationTimestamp": "2026-04-27T09:31:14.523+09:00",
  "observationProductRef": "https://uas-storage.example/products/...",
  "klvMetadataRef": "https://uas-storage.example/klv/..."
}
```

Streaming KLV metadata flows on a separate channel:

```
GET /airframes/<airframeRef>/klv/$subscribe HTTP/1.1
Authorization: Bearer ...
Accept: application/vnd.misb.0601+klv
```

The KLV stream pairs with the FMV (full-motion video) stream
delivered out-of-band via the deployment's video-storage system.

## §4 Weapon-release authorisation

```
POST /releases/$authorise HTTP/1.1
{
  "missionRef": "urn:wia:md-uas:mission:...",
  "decisionRef": "urn:wia:md:decision:...",
  "munitionType": "AGM-114",
  "targetRef": "urn:wia:md:track:...",
  "aimpointGeometry": {...},
  "releaseConditions": {...}
}
```

The release-authority's signed authorisation must be present; the
boundary refuses release without dual-signatures (operator + release
authority). On acceptance, the release record (PHASE 1 §6) is
created and the airframe receives the fire instruction over the
tactical link.

## §5 Weapon-release event reporting

After release:

```
POST /releases HTTP/1.1
{
  "releaseId": "urn:wia:md-uas:release:tail-37:r-91a7",
  ...all PHASE 1 §6 fields...
}
```

The boundary chains the release event to the authorisation, records
the airframe's signed event, and emits an AuditEvent. Outcome
reporting follows WIA-missile-defense PHASE 1 §8 outcome shape with
`outcomeSourceUasRelease` reference.

## §6 Geofence evidence collection

The airframe reports geofence compliance as a stream:

```
POST /airframes/<airframeRef>/geofence-evidence HTTP/1.1
{
  "geofenceEvidenceId": "...",
  "legRef": "urn:wia:md-uas:leg:m-91a7:wp-1-to-wp-2",
  "actualTrack": [...samples...],
  "geofenceBreachEvents": []
}
```

End-of-mission summary records the geofence-evidence URN in the
recovery record (PHASE 2 §8).

## §7 Lost-link tracking

```
POST /airframes/<airframeRef>/lost-link HTTP/1.1
{
  "lostFromTimestamp": "...",
  "lastKnownLinkClass": "blos-satcom",
  "behaviourTriggered": "return-to-base"
}
```

The boundary records the lost-link event; subsequent recovery uses
`PUT /airframes/<airframeRef>/lost-link/<lostLinkId>` with the
recovery evidence.

## §8 Recovery record

```
POST /missions/<missionRef>/$recover HTTP/1.1
{
  "recoveryType": "runway-landing",
  "recoveryLocation": {...},
  "flightDurationSeconds": 7200,
  "fuelRemainingPct": 0.18,
  "payloadStatus": "intact",
  "airframeCondition": "operational",
  "geofenceEvidenceRef": "urn:wia:md-uas:geofenceEvidence:..."
}
```

Recovery closes the mission's audit chain. The boundary indexes
recovery records per airframe so flight-hour tracking + maintenance
scheduling join the chain.

## §9 Errors and warnings

| URI                                              | Status | Meaning                                            |
|--------------------------------------------------|-------:|----------------------------------------------------|
| `urn:wia:md-uas:problem:airspace-conflict`       | 422    | mission route conflicts with civil-airspace coordination |
| `urn:wia:md-uas:problem:roe-not-authorised`      | 403    | mission RoE outside operator's authorisation       |
| `urn:wia:md-uas:problem:airframe-not-registered` | 404    | airframe URN not in deployment registry            |
| `urn:wia:md-uas:problem:dual-signature-required` | 403    | weapon-release lacks operator + release-authority signatures |
| `urn:wia:md-uas:problem:geofence-breach-pending` | 422    | mission segment requires resolution of pending breach |
| `urn:wia:md-uas:problem:link-prerequisite-missing`| 503   | mandated link-class not currently tracking         |
| `urn:wia:md-uas:problem:audit-unavailable`       | 503    | audit chain write failed                            |

Warnings (200-OK with caveats) use `Warning:` headers per RFC 7234
§5.5 with codes namespaced under `wia-md-uas-`.

## Annex A — Capability advertisement (informative)

```
GET /.well-known/wia/military-drone/capabilities HTTP/1.1
```

Response advertises supported airframe MTOW bands, payload
capabilities, supported link classes, supported MISB metadata
versions, the deployment's RoE authorities, federation peers, and
the conformance level. Capability documents are signed.

## Annex B — Pagination + rate limiting (informative)

Observation and state queries paginate at ≤ 1000 entries per page.
Per-token rate limits default to 10 mission-plan submissions per
hour per operator, 1000 state samples per second per airframe.
Rate-limit refusals carry `urn:wia:md-uas:problem:rate-limited`.

## Annex C — Idempotency

Mission-plan submissions and weapon-release authorisations accept
`Idempotency-Key`; retries within 24 h with the same key return the
original response.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex D — Worked re-task sequence (informative)

In-flight re-task:

```
PUT /missions/<missionId>/$retask HTTP/1.1
Authorization: Bearer ...
WIA-Operator-Signature: <JWS by operator>
WIA-Release-Authority-Signature: <JWS by release authority, when re-task crosses RoE bands>

{
  "newRoute": [...],
  "reason": "target relocated; updated coordinates from JTAC"
}
```

Re-tasking that stays within the same RoE band requires only
operator signature; re-tasking that crosses RoE bands requires
both operator + release-authority signatures (per §1 mission-plan
submission rules). The boundary refuses cross-RoE re-tasks without
dual signatures.

## Annex E — Worked sensor observation publication (informative)

```
POST /observations HTTP/1.1
Content-Type: application/wia-md-uas+json

{
  "obsId": "urn:wia:md-uas:obs:tail-37:2026-04-27T09:31:14:0001",
  "airframeRef": "urn:wia:md-uas:airframe:rok-army:tail-37",
  "payloadRef": "urn:wia:md-uas:payload:tail-37:gimbal-1",
  "sensorType": "eo",
  "geometry": {
    "airframeState": {"lat": 37.5, "lon": 127.0, "alt": 6000, "heading": 270, "speed": 80},
    "gimbalAttitude": {"roll": 0, "pitch": -25, "yaw": 270},
    "fovHorizontal": 12.5,
    "fovVertical": 9.8
  },
  "targetGroundFootprint": {"type": "Polygon", "coordinates": [[...]]},
  "observationTimestamp": "2026-04-27T09:31:14.523+09:00",
  "observationProductRef": "https://uas-storage.example/products/2026-04-27/tail-37/0001.mp4",
  "klvMetadataRef": "https://uas-storage.example/klv/2026-04-27/tail-37/0001.klv"
}
```

The boundary records the observation, indexes by target ground
footprint for spatial queries, and emits an AuditEvent.

## Annex F — Worked weapon-release authorisation (informative)

```
POST /releases/$authorise HTTP/1.1
Authorization: Bearer <operator-token>
WIA-Operator-Signature: <JWS by operator over body>
WIA-Release-Authority-Signature: <JWS by release authority over body>

{
  "missionRef": "urn:wia:md-uas:mission:rok-army:m-91a7-2026-04-27",
  "decisionRef": "urn:wia:md:decision:d-91a7",
  "munitionType": "AGM-114",
  "munitionCount": 1,
  "targetRef": "urn:wia:md:track:fusion-A:91a7",
  "aimpointGeometry": {"lat": 37.4516, "lon": 126.6531, "alt": 0},
  "releaseConditions": {"speed": 80, "altitude": 6000, "rangeM": 4500}
}
```

Response on accepted:

```
201 Created
Location: /releases/r-91a7
WIA-Audit-Event-Id: urn:wia:md-uas:audit:release-auth:7f2c
```

The boundary verifies both signatures, the RoE authorisation, the
airframe's current state allows the release conditions, and the
target-track is live in WIA-missile-defense before issuing the fire
instruction over the tactical link.

## Annex G — Capability discovery (informative)

```
GET /.well-known/wia/military-drone/capabilities HTTP/1.1
Accept: application/json
```

Response advertises supported airframe classes (MTOW bands), supported
payload types, supported KLV metadata versions, RoE authorities,
federation peers, and the deployment's conformance level (Surface /
Verified / Anchored). Cached capability documents expire on the
deployment's session-token lifetime.
