# WIA-military-satellite PHASE 2 — API Interface Specification

**Standard:** WIA-military-satellite
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a military-satellite ground
segment exposes for orbital-asset registry queries, payload
tasking-order intake, telemetry/product publication,
conjunction-assessment exchange, anti-jam posture reporting,
and coalition-release workflows. The shape is HTTP/JSON for
ground-segment planes and coalition exchanges; on-orbit
command/telemetry uses the binary CCSDS encodings in PHASE 3.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS), RFC 7807-bis
- CCSDS 502.0-B-2 — Orbit Data Messages (carried in JSON projection here)
- CCSDS 503.0-B-1 — Tracking Data Message
- MISB ST 0102 — security metadata in product responses
- ITU Radio Regulations — for spectrum-coordination payloads
- WIA-military-communication PHASE 2 — for transport delegation
- WIA-missile-defense PHASE 2 — for early-warning product handoff

---

## §1 Tasking-order intake

```
POST /commands/task-orders HTTP/1.1
Authorization: Bearer <jws-task-authority-jwt>
Content-Type: application/json

{
  "taskOrderId": "urn:wia:msat:task:rok-acomd:t-2026-04-27-2210",
  "assetRef": "urn:wia:msat:asset:rok-acomd:eo-1",
  "taskType": "image",
  "windowStart": "2026-04-28T02:14:00+09:00",
  "windowEnd": "2026-04-28T02:18:30+09:00",
  "priorityClass": "priority",
  "targetGeometry": { "type": "polygon", "coordinates": [/* … */] },
  "expectedProductType": "eo-frame",
  "coalitionReleaseList": ["urn:wia:auth:rok", "urn:wia:auth:us-coalition-cell"],
  "commandAuthorityRef": "urn:wia:auth:rok-acomd-tasking-cell"
}
```

Successful intake responds 202 Accepted with the boundary's
internal queue position and an estimated uplink window.
Refusal responses use RFC 9457 Problem Details with type
URIs in the `urn:wia:msat:problem:` namespace:

| problem URN                                  | meaning                              |
|----------------------------------------------|--------------------------------------|
| `urn:wia:msat:problem:asset-unknown`         | assetRef not in registry             |
| `urn:wia:msat:problem:classification-exceeded` | taskOrder exceeds asset ceiling     |
| `urn:wia:msat:problem:release-list-disallowed` | release inconsistent with policy   |
| `urn:wia:msat:problem:window-infeasible`     | window does not contain a pass       |
| `urn:wia:msat:problem:authority-unknown`     | commandAuthorityRef unrecognised     |

Tasking responses include a `taskAck` URN that the boundary
publishes to the audit chain.

## §2 Tasking-order status

```
GET /commands/task-orders/{taskOrderId} HTTP/1.1
```

Returns the lifecycle of a tasking order:

```json
{
  "taskOrderId": "urn:wia:msat:task:…",
  "state": "scheduled",
  "scheduledPass": {
    "ascendingNode": "2026-04-28T02:14:12+09:00",
    "groundStationRef": "urn:wia:gs:rok-acomd:gs-1",
    "expectedUplinkWindow": ["2026-04-28T01:58:00+09:00", "2026-04-28T02:03:00+09:00"]
  },
  "history": [
    {"at": "…", "state": "received"},
    {"at": "…", "state": "validated"},
    {"at": "…", "state": "scheduled"}
  ]
}
```

Lifecycle states: `received`, `validated`, `scheduled`,
`uplinked`, `executing`, `completed`, `failed`, `cancelled`.
Failure responses populate `failureReason` with a
domain-specific code.

## §3 Telemetry stream

Telemetry is exchanged as a server-sent-event stream or via
periodic POST batches; both use the same envelope. The streamed
form:

```
GET /assets/{assetRef}/telemetry/stream HTTP/1.1
Accept: text/event-stream
```

Each event is one telemetry record from PHASE 1 §5. Resume
tokens are issued on every event so consumers can reconnect
without loss. Bulk POST batches are used by ground stations
operating in store-and-forward mode (e.g., polar-orbit pass
playback over a single contact).

Subscribers may filter by subsystem with `?subsystems=power,thermal`.
Subsystem snapshots not requested are omitted from the event.
Filter changes do not cause a stream restart; they take effect
on the next event boundary.

## §4 Product publication and retrieval

### §4.1 Publish

Ground-segment processors publish products:

```
POST /products HTTP/1.1
Authorization: Bearer <jws-processing-element-jwt>
Content-Type: application/json
```

Body is a PHASE 1 §6 product record. The boundary verifies
the security metadata is well-formed (MISB ST 0102), the
classification ceiling is honoured, the release list is
consistent with the asset's release-authority policy, and the
collection window is contained in the originating tasking
order's authorised window.

Product binaries (frames, cubes, cuts) are referenced by URI
in the record; the boundary does not store binaries inline.
Default URI scheme is `s3://` or `https://` for trusted
mission-storage; coalition-shared products use a partner-
provided URI in the partner's `urn:wia:auth:` zone.

### §4.2 Retrieve

Authorised consumers retrieve products:

```
GET /products/{productId} HTTP/1.1
```

Responses include the security metadata; binaries are fetched
through the URI carried in the metadata (subject to that
URI's own auth). The boundary records every retrieval in the
audit chain, indexed by the consumer's authority URN.

A consumer not on the product's release list receives 403 with
problem URN `urn:wia:msat:problem:release-denied`.

## §5 Conjunction-assessment exchange

```
POST /conjunctions HTTP/1.1
```

Body is a PHASE 1 §7 CA record. The boundary distributes CA
records to interested parties: the asset's operations cell,
the secondary object's operator (if known and disclosure
permitted), and the coalition space-domain-awareness cell.

```
GET /conjunctions?primaryRef=…&since=…
```

Returns the catalog of CA records for an asset since a given
epoch.

A CA record above the deployment's `pcThreshold` triggers
PHASE 4 §6 collision-avoidance workflow (typically a
delta-v-burn tasking order is generated and submitted via §1).

## §6 Anti-jam posture exchange

```
POST /posture HTTP/1.1
```

Body is a PHASE 1 §8 posture record. Posture transitions to
`denied` or `deceptive-suspected` are pushed to the
spectrum-coordination cell via PHASE 4 §7 within a deployment-
declared latency budget (typically < 30 s).

```
GET /posture/{assetRef}/current
```

Returns the current posture and history of state transitions
for the requested asset.

## §7 Coalition-release workflow

When a national-only product becomes coalition-releasable,
the originating deployment posts a release request:

```
POST /release-requests HTTP/1.1

{
  "releaseRequestId": "urn:wia:msat:release:rok-acomd:r-2026-04-27-001",
  "productRefs": ["urn:wia:msat:product:rok-acomd:eo-1:p-991"],
  "newReleaseList": ["urn:wia:auth:rok", "urn:wia:auth:us-coalition-cell"],
  "rationale": "supports coalition early-warning analysis (free text, optional)",
  "decisionAuthorityRef": "urn:wia:auth:rok-release-board"
}
```

The release board's decision is posted as a
counter-signature; the boundary updates the product record's
release list only after the counter-signature is verified.
All release-request traffic is mirrored to the coalition's
audit witness for cross-coalition non-repudiation.

## §8 Capability discovery

```
GET /.well-known/wia/military-satellite HTTP/1.1
```

Returns the deployment's capability document:

```json
{
  "wia.standardVersion": "1.0",
  "wia.implementationVersion": "rok-acomd-3.4.1",
  "supportedAssetMissions": ["imagery-eo","imagery-sar","sigint","milsatcom"],
  "supportedTaskTypes": ["image","intercept","relay","ranging","delta-v-burn"],
  "telemetryDelivery": ["sse","batch-post"],
  "productDelivery": ["uri-reference","coalition-mirror"],
  "antijam": {"supportedMitigations": ["frequency-hop","nulling-antenna","crypto-rolled"]},
  "manifest": "https://gs.rok-acomd.example/.well-known/wia/military-satellite/manifest.jws"
}
```

The capability document is signed (JWS detached) so
coalition partners verify compatibility before exchange.

## §9 Authentication and rate-limiting

All requests authenticate with a JWS-signed JWT (PHASE 3 §1).
The boundary enforces per-authority quotas; a quota-exceeded
response uses problem URN `urn:wia:msat:problem:quota-exceeded`
with `retryAfter` (seconds). Per-authority quotas are recorded
in the deployment policy and may be raised by the deployment's
spectrum-coordination cell in `flash` priority cases.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Idempotency

Tasking-order POST and product POST accept `Idempotency-Key`.
Boundary stores keys for 24 h; replays within that window
return the original response. Different bodies under the same
key return `urn:wia:msat:problem:idempotency-conflict`.

## Annex B — Pagination

List endpoints (`/products`, `/conjunctions`, `/posture/.../history`)
support cursor pagination. Cursors are opaque, signed by the
boundary, and short-lived (10 min). Replay across page
boundaries is consistent with respect to the page-build
snapshot epoch returned in the page envelope.

## Annex C — Negative-test vectors (informative)

| Stimulus                                           | Expected response                              |
|----------------------------------------------------|------------------------------------------------|
| Tasking with unknown assetRef                      | 404 + `urn:wia:msat:problem:asset-unknown`     |
| Tasking exceeding asset classification ceiling     | 422 + `urn:wia:msat:problem:classification-exceeded` |
| Product publish missing securityMetadata           | 422 + `urn:wia:msat:problem:metadata-missing`  |
| Retrieve by consumer not on releaseList            | 403 + `urn:wia:msat:problem:release-denied`    |
| Posture POST with invalid frequencyBand            | 422 + `urn:wia:msat:problem:band-unknown`      |
| Repeated tasking with same Idempotency-Key, different body | 409 + idempotency-conflict             |

## Annex D — Cross-domain delegation

Products of type `early-warning` (e.g., IR detection of
boost-phase events) are simultaneously published to
WIA-missile-defense PHASE 2 §3 with the same productId; the
missile-defense boundary treats them as engagement-decision
inputs. Idempotency on the productId prevents double-counting
across domains.

## Annex E — Capability versioning

Capability documents declare both `wia.standardVersion` and
`wia.implementationVersion` so that coalition partners can
verify compatibility independently of vendor build numbers.
A standard-version mismatch is a hard refusal; an
implementation-version mismatch is logged but not refusing.

## Annex F — Bulk product retrieval

Bulk product retrieval for archival or analytical replay is
served by a separate endpoint:

```
GET /products/bulk?since=…&until=…&assetRef=…
Accept: application/x-ndjson
```

Each line of the response is a PHASE 1 §6 product record;
binaries are still by URI reference. Bulk retrieval is
gated by the deployment's bulk-quota policy and is logged
in the audit chain with `kind=product-bulk-retrieved`.

## Annex G — Operator action endpoints

Operator-actuated state transitions (anomaly acknowledge,
safe-mode entry/exit, propulsion enable, coalition release-
list editing under emergency authority) are served by the
`/operator-actions` endpoint with a JSON body declaring the
action type, target object, justification, and the operator
authority URN. Every action emits a signed audit-chain
entry with `kind=operator-action`.

## Annex H — Subscription endpoints

For systems that need pushed updates (intelligence-fusion,
missile-defense early-warning), the boundary supports
webhook subscriptions:

```
POST /subscriptions HTTP/1.1

{
  "subscriptionId": "urn:wia:msat:sub:fusion-cell:s-001",
  "callbackUrl": "https://fusion.example/webhooks/msat",
  "eventClasses": ["product-published","posture-changed"],
  "filters": {"productTypes": ["eo-frame","sigint-cut"]},
  "subscriberAuthorityRef": "urn:wia:auth:fusion-cell"
}
```

Webhook delivery uses TLS 1.3 and is signed with a JWS
detached signature in a `Wia-Signature` header. Failed
deliveries are retried on an exponential schedule for up
to 24 hours; persistent failure escalates to the
operations cell.
