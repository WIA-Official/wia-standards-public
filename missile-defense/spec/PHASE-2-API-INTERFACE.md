# WIA-missile-defense PHASE 2 — API Interface Specification

**Standard:** WIA-missile-defense
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a missile-defence boundary exposes
for sensor observation ingest, track query, threat-assessment
publication, engagement decision authorisation, weapon-status
reporting, and outcome retrieval. The shape is HTTP/JSON for
command-and-control planes; for tactical fire-control loops, a
constrained-binary form is described in PHASE 3.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS), RFC 8259 (JSON)
- STANAG 5516 — Tactical Data Link 16 (J-series messages)
- STANAG 5522 — Link 22
- MIL-STD-6016 — J-series message realisation
- WIA-military-communication (referenced for transport)
- WIA-medical-data-privacy (referenced for casualty integration)

---

## §1 Sensor observation ingest

```
POST /observations HTTP/1.1
Host: md.coalition-c2.example
Authorization: Bearer eyJhbGciOiJFUzI1NiIsImtpZCI6ImsxIn0...
Content-Type: application/wia-md+json
WIA-Doctrine: NATO-AAP-15
WIA-Sensor-Id: urn:wia:md:sensor:fielded:radar-7e2

{
  "timestamp": "2026-04-27T09:31:14.523+09:00",
  "position": {"lat": 37.4516, "lon": 126.6531, "alt": 12500, "covarianceM2": [...]},
  "velocity": {"vx": 280, "vy": -50, "vz": 1500, "covarianceM2s2": [...]},
  "kinematicCategory": "tactical-ballistic",
  "signature": {"rcsM2": 0.18, "uncert": 0.05}
}
```

The boundary validates units, covariance well-formedness, and the
sensor's authority to ingest, then forwards to the fusion engine
(PHASE 4 §4) and emits an AuditEvent. Observations from sensors not
in the registry (PHASE 4 §3) are rejected with
`urn:wia:md:problem:unknown-sensor`.

## §2 Track query

```
GET /tracks?identification=hostile,suspect
            &kinematic=tactical-ballistic,medium-range-ballistic
            &area=polygon:37.4,126.6;37.6,126.6;37.6,126.8;37.4,126.8
            &since=2026-04-27T09:30:00+09:00
HTTP/1.1
Authorization: Bearer ...
```

The boundary returns tracks matching the filter, gated by the
caller's releasability scope. Aggregated views (per-area count,
per-class count) are at `/tracks/$summary` with the same gate.

## §3 Threat-assessment publication

```
POST /assessments HTTP/1.1
{
  "trackRef": "urn:wia:md:track:fusion-A:91a7",
  "threatLevel": "high",
  "threatType": "tbm-class-A",
  "predictedImpact": {
    "area": {...},
    "timeWindow": {"start": "...", "end": "..."},
    "uncertainty": "medium"
  },
  "confidenceBand": "probable"
}
```

The boundary validates the track reference, applies the
deployment's threat-catalogue check, and emits the assessment to
the engagement-authority queue. Subscribers to threat alerts
receive the assessment over Server-Sent Events on a separate
channel (PHASE 2 Annex C).

## §4 Engagement decision

```
POST /decisions HTTP/1.1
WIA-Engagement-Authority: urn:wia:org:rok-army.adcc

{
  "assessmentRef": "urn:wia:md:assessment:9c0a",
  "engagementType": "intercept",
  "roeBasis": ["roe-2026-04-mainland-defence-§7"],
  "weaponSystemRef": "urn:wia:md:ws:patriot-pac3-bty-7"
}
```

The boundary validates the engagement authority's signed token
and the weapon system's readiness. On acceptance, the decision is
hashed into the audit chain and pushed to the weapon system. The
weapon system acknowledges receipt before the decision is
considered live.

## §5 Weapon-status reporting

```
POST /weapons/<wsRef>/status HTTP/1.1
{
  "decisionRef": "urn:wia:md:decision:d-91a7",
  "state": "launched",
  "interceptorRef": "urn:wia:md:int:i-91a7-01",
  "telemetry": {
    "position": {...},
    "velocity": {...},
    "propellantRemaining": 0.78,
    "guidanceLock": true
  },
  "stateTimestamp": "2026-04-27T09:31:18.220+09:00"
}
```

State transitions are signed by the weapon system's signing key.
The boundary refuses status updates that do not chain from the
prior status (e.g., `terminal` cannot follow `slewed` without
intermediate states) so that timeline reconstruction is consistent.

## §6 Outcome retrieval

```
GET /outcomes?since=2026-04-27T09:00:00+09:00
              &decisionRef=...
HTTP/1.1
```

Returns outcome records and any subsequent revisions. The
boundary applies the same release-authority gate as decisions;
outcomes the requester cannot reach are filtered out.

## §7 Track correlation

When two fusion engines determine they hold the same physical
object, they create a federation track:

```
POST /federation/correlations HTTP/1.1
{
  "trackRefs": [
    "urn:wia:md:track:fusion-A:91a7",
    "urn:wia:md:track:fusion-B:7e2c"
  ],
  "evidence": ["range-bearing-overlap", "shared-iff-mode-5", "kinematic-co-track"],
  "correlatingPrincipal": "urn:wia:md:fusion:coalition-fed-01"
}
```

The boundary creates a federation track referencing both
authorities' tracks and emits an AuditEvent visible to both
authorities so the decision can be audited.

## §8 Subscription channels

For real-time fire-control loops, the boundary supports streaming:

- `GET /tracks/$subscribe?identification=hostile&kinematic=...`
- `GET /assessments/$subscribe?threatLevel=high,critical`
- `GET /weapons/<wsRef>/status/$subscribe`

Streams use Server-Sent Events with each event carrying the
WIA-Audit-Event-Id so subscribers can replay the audit chain.
Subscriptions inherit the same release-authority gate as one-shot
queries.

## §9 Errors and warnings

| URI                                              | Status | Meaning                                    |
|--------------------------------------------------|-------:|--------------------------------------------|
| `urn:wia:md:problem:unknown-sensor`              | 400    | sensor not in registered fleet              |
| `urn:wia:md:problem:invalid-covariance`          | 422    | covariance matrix not positive-semidefinite |
| `urn:wia:md:problem:track-correlation-conflict`  | 409    | proposed correlation conflicts with existing|
| `urn:wia:md:problem:engagement-authority-lacking`| 403    | principal lacks engagement authority        |
| `urn:wia:md:problem:weapon-not-ready`            | 409    | weapon system not in a state to engage      |
| `urn:wia:md:problem:state-transition-invalid`    | 422    | weapon-state transition out of order        |
| `urn:wia:md:problem:audit-unavailable`           | 503    | audit chain write failed                    |

Warnings (200-OK with content caveats) use `Warning:` headers per
RFC 7234 §5.5 with codes namespaced under `wia-md-`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked engagement decision sequence (informative)

1. Sensor radar-7e2 emits an observation:
   `POST /observations` → `urn:wia:md:obs:radar-7e2:...`
2. Fusion engine fuses observations into track
   `urn:wia:md:track:fusion-A:91a7` with kinematic-category
   `tactical-ballistic` and identification `hostile` (NCTR + flight-
   pattern). Track update emits AuditEvent.
3. Threat-assessment engine surfaces a `high` threat-level
   assessment with predicted impact area; this is published and
   subscribers (engagement-authority consoles) receive the alert.
4. Engagement authority `urn:wia:org:rok-army.adcc` reviews the
   recommended Patriot-PAC3 engagement, signs the decision, and
   submits `POST /decisions`.
5. Boundary verifies the authority's `wia_engagement_scope`
   covers the track's predicted-impact area and the kinematic
   class. Decision committed; weapon system tasked.
6. Patriot-PAC3 weapon system acknowledges, transitions through
   `slewed → armed → launched → boost → mid-course → terminal →
   intercept-attempt → success`. Each transition emits a
   weapon-status record.
7. Outcome record published with `outcome: success` and
   confidence-basis `sensor-reacquisition-loss` plus
   `optical-signature-burst`. Audit chain captures the complete
   timeline.

## Annex B — Capability advertisement (informative)

```
GET /.well-known/wia/missile-defense/capabilities HTTP/1.1
```

Response carries the deployment's doctrine, supported sensor
modalities, supported weapon-system classes, federation peers,
and conformance level. Cached capability documents expire on the
deployment's session-token lifetime.

## Annex C — Pagination and rate limiting (informative)

Track and observation queries paginate at ≤ 1000 results per page.
Per-token rate limit defaults: 1000 observation ingest calls per
second per sensor, 100 query calls per second per operator.
Rate-limit refusals carry `urn:wia:md:problem:rate-limited` and are
themselves audit events.

## Annex D — Outcome revision worked example (informative)

Initial outcome:

```
POST /outcomes
{
  "decisionRef": "urn:wia:md:decision:d-91a7",
  "outcome": "success",
  "confidenceBasis": ["sensor-reacquisition-loss"],
  "outcomeTimestamp": "2026-04-27T09:31:24.480+09:00"
}
```

Two minutes later, sensor re-acquires what may be debris but
could be a remaining payload:

```
POST /outcomes
{
  "decisionRef": "urn:wia:md:decision:d-91a7",
  "outcome": "partial",
  "confidenceBasis": ["sensor-reacquisition-loss", "debris-track-residual"],
  "supersedes": "urn:wia:md:outcome:o-91a7",
  "outcomeTimestamp": "2026-04-27T09:33:42.115+09:00"
}
```

The revision is a new record referencing the prior; both are
preserved in the audit chain so the reasoning timeline is
reconstructable.

## Annex E — Long-running operation pattern (informative)

For long-running queries (broad-area searches, historical reanalysis),
the boundary uses the FHIR-style asynchronous pattern: client
submits request, receives `202 Accepted` with a status URI, polls
the status URI until completion. Asynchronous results are signed
on completion and held at the result URI for the deployment's
result-retention policy.

The asynchronous pattern is preferred when the synchronous response
would exceed the deployment's response-latency SLA; it is required
when the result set exceeds the boundary's pagination cap.

## Annex F — Capability discovery worked example (informative)

```
GET /.well-known/wia/missile-defense/capabilities HTTP/1.1
Accept: application/json
```

```
200 OK
Content-Type: application/json

{
  "wia.doctrine": "NATO-AAP-15",
  "wia.confidenceModel": "standard",
  "wia.supportedKinematicClasses": ["aircraft", "cruise-missile", "tactical-ballistic", "medium-range-ballistic", "hypersonic-glide", "space-object"],
  "wia.supportedWeaponClasses": ["patriot-pac3", "thaad", "aegis-bmd", "sm-3", "sm-6"],
  "wia.federations": [
    {"peerOrgRef": "urn:wia:org:us-army.380adabde", "manifestExpiry": "2027-04-27"}
  ],
  "wia.releaseAuthorities": [
    {"role": "tactical-engagement-authority", "scope": "tactical-ballistic, area: KAOR"},
    {"role": "national-engagement-authority", "scope": "all kinematicClasses, area: ROK national airspace"}
  ],
  "wia.signature": "<JWS detached>"
}
```

Clients verify the signature against the deployment's JWKS before
honouring any advertised capability.

## Annex G — Pagination header

Paginated responses set `Link: <...>; rel="next"` and `X-WIA-Total-Count` headers consistent with the boundary's release policy. Counts honour the same gate as records, so totals never leak the existence of records the caller cannot see.
