# WIA-nbc-defense PHASE 2 — API Interface Specification

**Standard:** WIA-nbc-defense
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a NBC-defence deployment exposes
for sensor ingest, event query, plume model invocation, decontamination
work-order management, and allied report rendering. The shape is
HTTP/JSON for command-and-control integration; for tactical bandwidth-
constrained links, a constrained-binary form is described in PHASE 3.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS), RFC 8259 (JSON)
- STANAG 5066 — bandwidth-constrained tactical data exchange (HF radio)
- STANAG 2103 / AArtyP-1 — NBC report formats
- HL7 FHIR R5 — for medical-integration endpoints
- ISO 19115 — geographic information metadata (used in plume areas)
- WHO IHR (2005) — public-health notification flows

---

## §1 Sensor ingest

Sensors push events with `POST /events`:

```
POST /events HTTP/1.1
Host: nbc.coalition-c2.example
Authorization: Bearer eyJhbGciOiJFUzI1NiIsImtpZCI6InMxIn0...
Content-Type: application/wia-nbc+json
WIA-Doctrine: NATO-AJP-3.8
WIA-Sensor-Id: urn:wia:nbc:sensor:fielded:cpd-91a7

{
  "agentRef": {"domain": "chemical", "vocab": "OPCW", "code": "GB"},
  "confidence": "presumptive",
  "timestamp": "2026-04-27T09:31:14+09:00",
  "position": {"lat": 37.4516, "lon": 126.6531, "alt": 12, "h_uncert_m": 2.5},
  "concentration": {"value": 0.18, "unit": "mg/m3", "uncert": 0.03},
  "metadata": {"model": "...", "firmware": "...", "calibration": "2026-03-15"}
}
```

The boundary validates the agent code, unit, and confidence band,
hash-chains the event into the audit chain (PHASE 3 §4), and returns:

```
201 Created
Location: /events/urn:wia:nbc:event:cpd-91a7:2026-04-27T09:31:14+09:00:0001
WIA-Audit-Event-Id: urn:wia:nbc:audit:2026-04-27T09:31:14+09:00:7f2a
```

Events from sensors not in the fleet registry (PHASE 4 §3) are
rejected with `urn:wia:nbc:problem:unknown-sensor`.

## §2 Event query

```
GET /events?agent.domain=chemical
            &confidence=confirmed,validated
            &area=polygon:37.45,126.65;37.46,126.65;37.46,126.66;37.45,126.66
            &since=2026-04-27T00:00:00+09:00
HTTP/1.1
Authorization: Bearer ...
WIA-Doctrine: NATO-AJP-3.8
```

The boundary returns DICOM-style JSON describing the matching events.
Results are paginated (≤ 1000 per page); each page emits its own
AuditEvent. The boundary inserts `wia.releaseAuthority` and
`wia.releasedTo` fields naming the release authority that approved
the query and the receiving organisation so that downstream
auditors can replay the disclosure chain.

## §3 Confidence promotion

A `confirmed` event is created by promoting a set of `presumptive`
events:

```
POST /events/$promote-to-confirmed
{
  "presumptiveRefs": [...],
  "sensorTechnologies": ["IMS", "FPA"],
  "operatorRef": "urn:wia:org:tdf-9-cbrn-platoon"
}
```

A `validated` event is created from `confirmed` plus a sample
laboratory result:

```
POST /events/$promote-to-validated
{
  "confirmedRef": "urn:wia:nbc:event:...",
  "sampleId": "urn:wia:nbc:sample:s-4a7c-9f01",
  "labRef": "urn:wia:org:lab:opcw-designated-A12",
  "labReportRef": "https://lab.opcw-A12.example/reports/2026-04-27/r-9120"
}
```

Each promotion emits an AuditEvent and updates the canonical
record's `confidence` field. Promotions are append-only; a record
cannot be demoted (a refutation creates a new event with
`agentRef.domain: refutation` referencing the prior record).

## §4 Plume model invocation

```
POST /plume-runs
{
  "sourceTerm": {"agentRef": ..., "release": {...}},
  "meteorology": {"profileRef": "urn:wia:nbc:met:wpfp-2026-04-27T09:00"},
  "terrain": {"demRef": "urn:wia:nbc:dem:srtm-1arc-v3"},
  "horizon": {"start": "...", "duration": "PT4H", "step": "PT300S"},
  "model": "HPAC-5.4"
}
```

The boundary validates the inputs against the registered model's
input schema, queues the run, and returns a `202 Accepted` with a
status URI:

```
202 Accepted
Location: /plume-runs/r-91a7
```

Polling the status URI returns the run's progress and, on completion,
references to the output products (gridded concentration fields,
contour polygons, time-stamped NBC-3 report).

## §5 Decontamination work-orders

```
POST /work-orders
{
  "triggerEventIds": [...],
  "targetArea": {"type": "Polygon", "coordinates": [[...]]},
  "methodCodes": ["water-with-detergent", "mechanical-removal"],
  "issuedBy": "urn:wia:org:tdf-9-cbrn-platoon",
  "personnel": [...]
}
```

Updates use `PUT /work-orders/<id>` to record commencement,
completion, and verification. The boundary refuses to mark a
work-order complete without a referenced `verificationEvent`
(PHASE 1 §7).

## §6 Casualty triage

Triage records integrate with the medical record system via
FHIR R5:

- `POST /casualties` — create a triage record (this PHASE)
- The boundary projects the record into FHIR Observation +
  DiagnosticReport for the receiving medical system
- The pseudonymous subject identifier is shared with WIA-medical-
  data-privacy PHASE 1 §2; the FHIR projection inherits that
  standard's consent gate

Updates after initial triage flow through the medical system; the
NBC-defence boundary holds the original triage record as canonical
for forensic reconstruction of the incident timeline.

## §7 Allied report rendering

Allied-format reports are derived projections rendered on demand:

- `GET /reports/nbc1?eventRef=...` — render NBC-1 from a
  `presumptive` event
- `GET /reports/nbc4?eventsAfter=...&eventsBefore=...` — render
  NBC-4 from confirmed events in a time window
- `GET /reports/nbc5?asOf=...` — render NBC-5 contaminated-area
  polygon from validated events

Rendered reports are signed by the originating organisation; a
recipient can replay the rendering inputs against the canonical
records to confirm fidelity. The boundary refuses to render
NBC-6 (detailed information) outside the originating organisation's
release authority.

## §8 Public-health notification

When a deployment is bound to public-health reporting (WHO IHR
or national equivalent), the boundary emits notifications:

- `POST /public-health/notifications` — submit an IHR-compatible
  notification referencing the underlying NBC events
- The notification carries the WHO IHR JSON shape; the receiver
  is the national IHR Focal Point

The boundary applies a release-authority gate before submitting;
unauthorised public-health emission is rejected with
`urn:wia:nbc:problem:release-not-authorised`.

## §9 Errors and warnings

| URI                                          | Status | Meaning                              |
|----------------------------------------------|-------:|--------------------------------------|
| `urn:wia:nbc:problem:unknown-sensor`         | 400    | sensor not in registered fleet        |
| `urn:wia:nbc:problem:unknown-agent`          | 400    | agent code outside §2 vocabulary      |
| `urn:wia:nbc:problem:invalid-unit`           | 422    | concentration unit outside §3 list    |
| `urn:wia:nbc:problem:release-not-authorised` | 403    | release authority lacking             |
| `urn:wia:nbc:problem:audit-unavailable`      | 503    | audit chain write failed              |
| `urn:wia:nbc:problem:custody-broken`         | 422    | sample chain-of-custody broken        |

Warnings (200-OK with content caveats) use `Warning:` headers per
RFC 7234 §5.5 with codes namespaced under `wia-nbc-`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Capability advertisement (informative)

The deployment advertises its capabilities at
`/.well-known/wia/nbc-defense/capabilities`. The advertisement
includes:

- `wia.doctrine` — NATO / KR / US / coalition-X / etc.
- `wia.confidenceModel` — one of standard, extended (for nation-
  specific extensions to PHASE 1 §4)
- `wia.tacticalEncoding.supported` — list of supported constrained
  encodings (PHASE 3 §5)
- `wia.federations[]` — listed federation peers and their manifest
  expiries
- `wia.releaseAuthorities[]` — listed release authorities and their
  scope of authority

Capability documents are signed; clients verify against the
deployment's JWKS before honouring any advertised capability.

## Annex B — Pagination and rate limiting (informative)

Event queries paginate at ≤ 1000 results per page. The boundary
applies a per-token rate limit to prevent operational-picture
exfiltration: a token issuing more than 100 queries per minute is
refused with `urn:wia:nbc:problem:rate-limited` until the rate
returns under the threshold. Rate-limit refusals are themselves
audit events.

## Annex C — Worked plume run sequence (informative)

```
POST /plume-runs
{
  "sourceTerm": {
    "agentRef": {"domain": "chemical", "vocab": "OPCW", "code": "HD"},
    "release": {
      "type": "point",
      "rateKgs": 0.05,
      "elevationM": 5,
      "durationSeconds": 600
    }
  },
  "meteorology": {"profileRef": "urn:wia:nbc:met:icao-2026-04-27T09:00Z"},
  "terrain": {"demRef": "urn:wia:nbc:dem:srtm-1arc-v3"},
  "horizon": {"start": "2026-04-27T09:31:00+09:00", "duration": "PT4H", "step": "PT300S"},
  "model": "HPAC-5.4"
}
```

Response:

```
202 Accepted
Location: /plume-runs/r-91a7
WIA-Audit-Event-Id: urn:wia:nbc:audit:2026-04-27T09:31:30+09:00:7f30
```

Polling the status URI:

```
GET /plume-runs/r-91a7
{
  "id": "r-91a7",
  "status": "completed",
  "outputs": {
    "concentrationGrid": "https://...",
    "contourPolygon": "https://...",
    "nbc3Report": "https://..."
  }
}
```

The boundary signs each output URI; recipients verify the signature
before treating the output as authoritative.

## Annex D — Streaming subscription (informative)

For real-time operational picture, the boundary supports a
streaming subscription:

```
GET /events/$subscribe?confidence=confirmed,validated
                       &area=...
HTTP/1.1
Authorization: Bearer ...
Accept: text/event-stream
WIA-Doctrine: NATO-AJP-3.8
```

Server pushes Server-Sent Events as new events arrive matching the
filter. Each event carries the WIA-Audit-Event-Id so that the
subscriber can replay the audit chain. The subscription is gated
by the same release-authority check as one-shot queries; an event
that the subscriber's purpose grant does not cover is filtered out.

## Annex E — Capability discovery worked example (informative)

```
GET /.well-known/wia/nbc-defense/capabilities HTTP/1.1
Accept: application/json
```

```
200 OK
Content-Type: application/json

{
  "wia.doctrine": "NATO-AJP-3.8",
  "wia.confidenceModel": "standard",
  "wia.tacticalEncoding.supported": ["stanag-5066-nbc1-96byte"],
  "wia.federations": [
    {"peerOrgRef": "urn:wia:org:nation-X.cbrn-cmd", "manifestExpiry": "2027-04-27"},
    {"peerOrgRef": "urn:wia:org:nation-Y.cbrn-cmd", "manifestExpiry": "2026-12-31"}
  ],
  "wia.releaseAuthorities": [
    {"role": "field-release-officer", "scope": "tactical events ≤ confirmed"},
    {"role": "national-release-officer", "scope": "validated events, public-health bridge"}
  ],
  "wia.signature": "<JWS detached>"
}
```

Clients verify the signature against the deployment's JWKS before
honouring any advertised capability. Cached capability documents
SHOULD expire within the deployment's session-token lifetime.

## Annex F — Cross-reference index (informative)

The following table maps each PHASE 2 endpoint to the canonical
record it acts upon and the audit-event class it emits:

| Endpoint                            | Record type                  | AuditEvent class             |
|-------------------------------------|------------------------------|------------------------------|
| `POST /events`                      | sensor event (presumptive)   | nbc.sensor.ingest            |
| `POST /events/$promote-to-confirmed`| sensor event (confirmed)     | nbc.sensor.promote.confirmed |
| `POST /events/$promote-to-validated`| sensor event (validated)     | nbc.sensor.promote.validated |
| `POST /plume-runs`                  | plume run                    | nbc.plume.invoke             |
| `POST /work-orders`                 | decontamination work-order   | nbc.work-order.issue         |
| `PUT /work-orders/<id>`             | decontamination work-order   | nbc.work-order.update        |
| `POST /casualties`                  | triage record                | nbc.triage.create            |
| `GET /reports/...`                  | derived projection           | nbc.report.render            |
| `POST /public-health/notifications` | IHR notification             | nbc.publichealth.notify      |
