# WIA-military-communication PHASE 2 — API Interface Specification

**Standard:** WIA-military-communication
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a military-communications boundary
exposes for message origination, addressing lookup, classification
validation, spectrum coordination, link health query, and cross-link
bridging. The shape is HTTP/JSON for command-and-control planes; for
tactical narrowband links, a constrained-binary form is described in
PHASE 3.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS), RFC 8259 (JSON)
- STANAG 4406 — Military Message Handling System
- STANAG 5066 — bandwidth-constrained tactical data exchange
- STANAG 4774 / 4778 — Confidentiality and Information labelling
- MIL-STD-2045-47001D — Variable Message Format
- ITU-R Recommendations for spectrum coordination
- WIA-medical-data-privacy / WIA-nbc-defense (referenced for
  cross-domain integration)

---

## §1 Message origination

```
POST /messages HTTP/1.1
Host: milcomms.coalition-c2.example
Authorization: Bearer eyJhbGciOiJFUzI1NiIsImtpZCI6ImsxIn0...
Content-Type: application/wia-milcomms+json
WIA-Doctrine: NATO-AAP-15

{
  "originator": "urn:wia:milcomms:ep:platform:nato-ramcis:awacs-tail-12",
  "addressees": [
    {"to": "urn:wia:milcomms:ep:unit:rok-army:roa-2-bde-3-bn", "role": "To"},
    {"to": "urn:wia:milcomms:ep:unit:us-army:1id", "role": "Cc"}
  ],
  "precedence": "immediate",
  "classification": {
    "level": "secret",
    "releasability": ["NATO", "ROK"]
  },
  "releaseAuthority": "urn:wia:org:nato.cao.j2",
  "subject": "FRAGO 27-04 — see attached",
  "body": {"contentType": "text/plain; charset=utf-8", "value": "..."},
  "references": []
}
```

The boundary validates classification, addresses, releasability,
spectrum availability for the chosen waveform, and the originator's
authority to release at the declared classification. Failure modes
return RFC 9457 Problem Details with the URI namespace
`urn:wia:milcomms:problem:*`.

## §2 Addressing lookup

```
GET /endpoints?scope=unit&authority=rok-army&pattern=roa-2-bde-* HTTP/1.1
Authorization: Bearer ...
WIA-Doctrine: NATO-AAP-15
```

Returns matching endpoints as JSON. The boundary filters results
by the requester's coalition releasability — endpoints in catalogues
the requester cannot reach (releasability mismatch) are silently
omitted. Each result emits a single combined AuditEvent so that
catalog-scraping is observable to the deployment's security
operations team.

## §3 Classification validation

```
POST /messages/$validate HTTP/1.1
{
  "originator": "...",
  "addressees": [...],
  "classification": {...},
  "body": {...}
}
```

Performs the classification + addressee + releasability check
without committing the message. Useful for clients that want to
warn an operator about a classification mismatch before send. The
response carries the boundary's would-be decision and a list of
caveats that would apply.

## §4 Spectrum coordination

```
POST /spectrum/allocations HTTP/1.1
{
  "frequencyRange": {"start": 30000000, "end": 30500000},
  "geoArea": {...},
  "period": {"start": "...", "end": "..."},
  "users": [...],
  "coordinationAuthority": "urn:wia:org:hostnation-c-fmt"
}
```

The boundary checks for conflicts with existing allocations in the
same geo area and period and refuses overlapping allocations with
`urn:wia:milcomms:problem:spectrum-conflict`. On acceptance, the
allocation is signed by the coordination authority and visible to
all operators authorised to read the deployment's spectrum
register.

## §5 Link health query

```
GET /links?terminalRef=urn:wia:milcomms:ep:terminal:...
           &since=2026-04-27T00:00:00Z
           &state=degraded,lost
HTTP/1.1
Authorization: Bearer ...
```

Returns link-quality observations. The boundary applies the same
release-authority gate as message endpoints; observations on links
the requester cannot reach (mismatch on releasability) are filtered
out. Aggregated views (per-link health summary, per-area summary)
are available at `/links/$summary` with the same gate.

## §6 Cross-link bridging

```
POST /bridges HTTP/1.1
{
  "sourceMessageRef": "urn:wia:milcomms:msg:...",
  "destinationWaveform": "urn:wia:milcomms:wf:link-22-bridged",
  "translationPolicy": "lossless"
}
```

The boundary applies the bridge classification check, ensures the
destination waveform is compatible with the source's classification
markings, and records the bridge event (PHASE 1 §8). Lossy bridges
require an explicit `translationPolicy: "lossy-acknowledged"` flag
plus a recorded justification and a release-authority signature.

## §7 EMCON status

Radio-silence postures (EMCON) are posted as control records:

```
POST /emcon HTTP/1.1
{
  "area": {...},
  "level": "alpha",
  "period": {...},
  "issuedBy": "urn:wia:org:..."
}
```

Active EMCON suspends transmission API endpoints (`POST /messages`)
for affected endpoints unless the message bears a `precedence:
flash` and a release authority overriding EMCON. Override events
are themselves AuditEvents.

## §8 ALE channel scan

ALE-capable terminals report scan results:

```
POST /ale/scans HTTP/1.1
{
  "terminalRef": "urn:wia:milcomms:ep:terminal:...",
  "scanWindow": {"start": "...", "end": "..."},
  "channelsTested": [...],
  "reachablePeers": [...]
}
```

The boundary aggregates scans across the deployment's ALE network
to support automatic link establishment decisions. Scan records
are themselves classified at the level appropriate to the listed
channels and peers.

## §9 Errors and warnings

| URI                                                | Status | Meaning                                  |
|----------------------------------------------------|-------:|------------------------------------------|
| `urn:wia:milcomms:problem:classification-invalid`  | 400    | classification marking malformed         |
| `urn:wia:milcomms:problem:releasability-mismatch`  | 403    | addressee outside releasability          |
| `urn:wia:milcomms:problem:release-not-authorised`  | 403    | originator lacks release authority       |
| `urn:wia:milcomms:problem:emcon-active`            | 423    | EMCON forbids transmission               |
| `urn:wia:milcomms:problem:spectrum-conflict`       | 409    | requested spectrum overlaps existing     |
| `urn:wia:milcomms:problem:waveform-incompatible`   | 422    | destination waveform cannot carry source |
| `urn:wia:milcomms:problem:catalog-not-found`       | 404    | addressing catalogue offline             |
| `urn:wia:milcomms:problem:audit-unavailable`       | 503    | audit chain write failed                 |

Warnings (200-OK with content caveats) use `Warning:` headers per
RFC 7234 §5.5 with codes namespaced under `wia-milcomms-`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked spectrum-coordination example (informative)

```
POST /spectrum/allocations HTTP/1.1
Host: milcomms.coalition-c2.example
Authorization: Bearer eyJhbGciOiJFUzI1NiJ9...

{
  "frequencyRange": {"start": 30000000, "end": 30500000},
  "geoArea": {"type": "Polygon", "coordinates": [[[126.5,37.4],[126.7,37.4],[126.7,37.6],[126.5,37.6],[126.5,37.4]]]},
  "period": {"start": "2026-04-27T00:00:00+09:00", "end": "2026-04-30T23:59:59+09:00"},
  "users": ["urn:wia:milcomms:ep:unit:rok-army:roa-2-bde"],
  "coordinationAuthority": "urn:wia:org:rok-mof.fmt",
  "priority": "operational"
}
```

Response on success:

```
201 Created
Location: /spectrum/allocations/a-91a7
WIA-Audit-Event-Id: urn:wia:milcomms:audit:2026-04-27T08:00:00+09:00:9c11
```

Response on conflict:

```
409 Conflict
Content-Type: application/problem+json

{
  "type": "urn:wia:milcomms:problem:spectrum-conflict",
  "title": "Spectrum overlaps existing allocation in same area and period",
  "status": 409,
  "detail": "Allocation a-7d12 (priority: mission-critical) covers 30000000-30200000 Hz in the requested area through 2026-05-01.",
  "instance": "/spectrum/allocations"
}
```

## Annex B — Catalogue refresh schedule (informative)

Catalogues for endpoint URNs (PHASE 1 §2) are refreshed at
intervals declared by the issuing authority. The boundary caches
catalogue entries for the cache lifetime declared in the
deployment policy and refreshes asynchronously to avoid blocking
calls. A catalogue offline beyond the cache lifetime suspends
new endpoint additions but allows existing endpoints to continue
operating.

## Annex C — Worked link-health subscription (informative)

```
GET /links/$subscribe?terminalRef=urn:wia:milcomms:ep:terminal:...
                       &states=degraded,lost
HTTP/1.1
Authorization: Bearer ...
Accept: text/event-stream
WIA-Doctrine: NATO-AAP-15
```

The boundary streams Server-Sent Events as link state transitions
on the listed terminal occur. Each event carries the WIA-Audit-Event-Id
so subscribers can replay the audit chain. Subscriptions inherit
the same release-authority gate as one-shot queries; events on
links the subscriber's grant does not cover are silently filtered.

## Annex D — Releasability filter examples (informative)

| Caller's `wia_releasability[]` | Endpoint releasability set | Visible? |
|---------------------------------|----------------------------|----------|
| `["NATO"]`                       | `["NATO"]`                  | yes      |
| `["NATO"]`                       | `["NATO","ROK"]`            | yes (intersection non-empty) |
| `["FVEY"]`                       | `["NATO"]`                  | no       |
| `["NATO","NOFORN"]`              | `["NATO"]`                  | no (NOFORN caveat applies) |
| `["NATO","KR-US"]`               | `["KR-US"]`                 | yes      |

Boundary precomputes the visibility decision per call so latency
remains predictable; the decision is part of the AuditEvent.

## Annex E — Pagination and rate limiting (informative)

Endpoint queries paginate at ≤ 1000 results per page. Per-token
rate limit defaults: 100 message-origination calls per minute,
500 lookup calls per minute. Rate-limit refusals carry
`urn:wia:milcomms:problem:rate-limited` and are themselves audit
events.

## Annex F — Capability discovery (informative)

```
GET /.well-known/wia/military-communication/capabilities HTTP/1.1
```

Response carries the deployment's doctrine, supported waveforms,
federation peers, and conformance level. Cached capability documents
expire on the deployment's session-token lifetime.

## Annex G — EMCON override worked example (informative)

```
POST /messages HTTP/1.1
Host: milcomms.coalition-c2.example
Authorization: Bearer ...
WIA-Doctrine: NATO-AAP-15
WIA-Emcon-Override: yes
WIA-Emcon-Override-Authority: urn:wia:org:rok-army.cdr-2-bde
WIA-Justification: TIC release confirmed at FOB-Echo, mass-casualty advisory inbound

{
  "originator": "urn:wia:milcomms:ep:platform:rok-army:fob-echo-cmd",
  "addressees": [
    {"to": "urn:wia:milcomms:ep:unit:rok-army:roa-2-bde", "role": "To"},
    {"to": "urn:wia:milcomms:ep:unit:nato-c2:cao-j2", "role": "Cc"}
  ],
  "precedence": "flash",
  "classification": {"level": "secret", "releasability": ["NATO","ROK"]},
  "releaseAuthority": "urn:wia:org:rok-army.cdr-2-bde",
  "subject": "TIC RELEASE CONFIRMED — IMMEDIATE EVAC",
  "body": {"contentType": "text/plain", "value": "..."}
}
```

Successful response carries the EMCON-override audit ID and the
original EMCON posture ID so the override appears in both records:

```
201 Created
Location: /messages/m-91a7
WIA-Audit-Event-Id: urn:wia:milcomms:audit:override:7f2c
WIA-Emcon-Posture-Override-Id: urn:wia:milcomms:emcon:override:e-91a7
WIA-Emcon-Posture-Ref: urn:wia:milcomms:emcon:p-7f00
```

The override review backlog (PHASE 4 §5) clocks from this audit
event; failure to review within 24 hours surfaces as an operational
incident.
