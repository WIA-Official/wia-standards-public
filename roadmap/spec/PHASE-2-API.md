# PHASE 2 — API Interface

> HTTP surface for milestone publication, dependency declaration,
> release notice, progress streaming, and commitment-change
> publication. All requests carry HTTP Message Signatures
> (RFC 9421); errors use Problem Details (RFC 9457).

## 2.1 Endpoint surface

```
POST /roadmap/v1/milestones                Publish a milestone envelope
GET  /roadmap/v1/milestones                Query milestones
GET  /roadmap/v1/milestones/{id}           Fetch a milestone

POST /roadmap/v1/dependencies              Publish a dependency declaration
GET  /roadmap/v1/dependencies?ms_id=...    Query dependencies for a milestone

POST /roadmap/v1/releases                  Publish a release envelope
GET  /roadmap/v1/releases?ms_id=...        Query releases

POST /roadmap/v1/progress                  Publish a progress update
GET  /roadmap/v1/progress?ms_id=...        Time-ordered progress history

POST /roadmap/v1/commitments/changes       Publish a commitment change
GET  /roadmap/v1/commitments/changes?ms_id=...  Change history

GET  /roadmap/v1/stream                    SSE: live envelope stream

GET  /.well-known/wia-roadmap              Capability advertisement
```

## 2.2 Authentication and signing

Every POST request MUST carry an HTTP Message Signature
(RFC 9421) over the mandatory `(request-target)`, `@authority`,
`content-digest`, `@created`, and `@expires` covered components.
The signing key is the publisher's tenant key, registered in the
WIA-OMNI-API trust fabric. Bearer-token-only authentication is
rejected.

## 2.3 Idempotency

POST endpoints accept an `Idempotency-Key` header (UUIDv7
recommended). A repeated request with the same key within 24
hours returns the original response without re-creating the
resource. Idempotency is important for roadmap publication
because retries during network partitions otherwise create
duplicate envelopes that pollute the audit trail.

## 2.4 Pagination

List endpoints use cursor pagination (`?cursor=...`) with `Link`
headers (RFC 8288). Page size is capped at 200 envelopes per
page to bound replay-defence cache pressure.

## 2.5 Streaming semantics

The `/stream` endpoint emits Server-Sent Events. Each event is a
complete signed envelope (no partial frames). The server replays
the last 60 seconds on connect to absorb client reconnects without
missing emissions; the client deduplicates by envelope identifier.

## 2.6 Capability advertisement

```
GET /.well-known/wia-roadmap

200 OK
{
  "wia_roadmap_version": "1.0.0",
  "type": "capability_advertisement",
  "publisher_id": "did:wia:org:...",
  "endpoint_surface_version": "roadmap/v1",
  "supported_envelope_types": [
    "milestone", "dependency", "release",
    "progress_update", "commitment_change",
    "calibration_report"
  ],
  "supported_languages": ["en", "ko", "ja", "zh"],
  "issue_tracker_bridges": ["github", "gitlab", "linear"],
  "rate_limits": { "rpm": 600, "burst_per_second": 100 },
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The capability advertisement is itself signed; consumers pin the
publisher's identity at integration time and detect impersonation
attempts that would otherwise be invisible.

## 2.7 Error envelope

Errors are returned as Problem Details (RFC 9457) with
roadmap-specific extension fields:

```
{
  "type": "https://wiastandards.com/roadmap/errors/duplicate-milestone",
  "title": "Milestone with this id already exists",
  "status": 409,
  "detail": "milestone_id 01HXR... is already published",
  "milestone_id": "01HXR..."
}
```

## 2.8 Versioning

The wire format version is carried in the envelope itself
(`wia_roadmap_version`). The HTTP surface is versioned in the URL
(`/roadmap/v1`). Breaking changes are released under a new URL
prefix; non-breaking additions are advertised in the capability
document.

## 2.9 Rate limits

Default rate limit is 600 requests per minute per publisher
client identity. Burst capacity is 100 requests in any one second.
Rate-limit responses use HTTP 429 with `Retry-After` seconds.

## 2.10 Content negotiation

Envelope payloads are JSON by default; CBOR (RFC 8949) is
negotiated via `Accept: application/cbor`. The signature covers the
canonical JSON form regardless of the on-wire encoding.

## 2.11 Webhook callbacks

Publishers may register webhook callbacks for downstream consumers
(stakeholder dashboards, journalists, regulators) to receive
envelopes asynchronously. Callback delivery uses exponential
backoff (1s, 2s, 4s, 8s, 16s, 32s, 64s, then dead-letter at the
8th attempt). The receiver MUST verify the envelope signature
before acting.

## 2.12 Bulk export

For investor due-diligence, regulatory inspection, or media
research, the bulk export endpoint returns a signed manifest plus
NDJSON envelope content:

```
POST /roadmap/v1/exports
{ "project_id": "did:wia:project:...",
  "from": "RFC 3339", "to": "RFC 3339" }

→ 202 Accepted, Location: /roadmap/v1/exports/{export_id}

GET /roadmap/v1/exports/{export_id}/manifest
GET /roadmap/v1/exports/{export_id}/content
```

The manifest carries a Merkle root over included envelopes so
the receiver can verify completeness without re-signing every
envelope individually.

## 2.13 Health and observability

```
GET /roadmap/v1/health    → liveness probe
GET /roadmap/v1/ready     → readiness probe
GET /roadmap/v1/metrics   → Prometheus exposition
```

Health endpoints do not require authentication and do not return
publisher-specific data.

## 2.14 Backwards compatibility

Implementations migrating from a legacy roadmap publication system
(WordPress, Notion, GitHub README, CSV) MAY operate a translator
that emits standard envelopes from the legacy source. The translator
is operated by the publisher; the standard does not require legacy
systems to upgrade for the standard to be useful.

## 2.15 Operational considerations

Roadmap publication is bursty by nature: most days have zero
publications, planning-week days have hundreds. The standard's
rate limits accommodate the burst pattern; sustained high-volume
traffic is more characteristic of progress-update streams from
large multi-team projects, where weekly bursts of 50–200 updates
are normal.

弘益人間 — Benefit All Humanity. The API surface exists so that even a
five-person team can publish a verifiable roadmap that an investor,
a regulator, and a journalist can all consume without re-keying.

## 2.16 Read-only consumer guidance

Investor dashboards, journalist-facing trackers, and regulator-side
tools typically need only a read-only subset of this surface. The
standard provides a `read_only_credentials` endpoint that mints a
scoped token (milestone + release endpoints only, no progress
update writes, no commitment-change writes) for those use cases:

```
POST /roadmap/v1/credentials/read-only
{ "purpose": "investor-dashboard" | "research" | "regulator-readonly" }
→ 201 Created, returns a short-lived bearer token bound to the
  caller's tenant and scope.
```

This avoids the anti-pattern of consumers re-using full publisher
credentials for read-only display.

## 2.17 Long-poll fallback

Environments that cannot maintain SSE connections (corporate
proxies, some mobile carriers) MAY use long-poll on
`GET /roadmap/v1/poll?since=<envelope_id>`. The server holds the
connection up to 30 seconds awaiting new envelopes; on timeout
the response is HTTP 204 No Content and the client re-polls.

## 2.18 Field-level encryption

Sensitive fields (officer identity in `approved_by`, embargoed
milestone names) MAY be field-level encrypted using AES-256-GCM
with per-tenant keys held in the tenant's HSM. The encrypted form
appears in the canonical envelope; the signature covers the
encrypted form, not the plaintext, so signature verification does
not require key access. Decryption is gated by Phase 3 federation
scope.

## 2.19 Versioned schema registry

A schema registry is exposed at
`GET /roadmap/v1/schemas/{type}/{version}` returning the JSON
Schema for the named envelope type at the named version. Schemas
are immutable; new versions are published under new identifiers.

## 2.20 Test surface

A read-only test surface is exposed at
`https://sandbox.wiastandards.com/roadmap/v1` returning realistic
synthetic envelope traffic. The sandbox is useful for client-side
conformance testing, dashboard development, and regulator-side
familiarisation. Sandbox traffic is signed by the WIA test key;
signatures will not verify against production trust roots, by
design.

## 2.21 Webhook callback worked example

```
POST /roadmap/v1/webhooks
{
  "url": "https://acme.example/wia-roadmap-callback",
  "events": ["milestone", "release", "commitment_change"],
  "publisher_filter": "did:wia:org:upstream-vendor",
  "shared_secret_hint": "k_2026q3",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The `shared_secret_hint` allows downstream consumers to rotate the
HMAC key used in the `X-WIA-Signature` callback header without
re-registering the webhook. The actual secret is exchanged out of
band during initial onboarding.

## 2.22 Rate-limit shaping

The default 600 RPM / 100 burst rate limit is configurable per
tenant. Heavy-volume publishers (e.g., aggregators serving
hundreds of subscribers) negotiate higher limits via the
operational-coordination channel of the WIA-OMNI-API trust fabric,
not via this surface.

## 2.23 Bridging conventions

Bridges (Phase 4) interact with the API as ordinary publishers:
they sign envelopes with their own tenant key and emit them via
the standard endpoints. Bridges that translate from a single
upstream system (e.g., GitHub) SHOULD record the upstream identifier
in an `upstream_ref` extension on the envelope so that a downstream
auditor can pivot from the WIA envelope to the original upstream
event in one hop.

## 2.24 Conformance attestation

Implementations publish a `conformance_attestation` envelope
declaring which version of the conformance corpus they pass. The
attestation is itself signed and is queryable by federated peers
during the handshake (Phase 3 §3.10).

弘益人間 — Benefit All Humanity. The API exists so that the smallest
roadmap publisher can be a peer to the largest.

## 2.25 Long-form descriptive endpoints

For human consumers — particularly journalists, investors, and
regulator staff — the standard provides long-form descriptive
endpoints that render envelope content as Markdown or HTML
suitable for direct embedding in a publication:

```
GET /roadmap/v1/render/{milestone_id}.md
GET /roadmap/v1/render/{milestone_id}.html
GET /roadmap/v1/render/{milestone_id}.txt
```

The rendered output is non-canonical; only the underlying signed
envelope is authoritative. The renderer is provided to reduce the
incentive for downstream consumers to scrape narrative roadmap
pages, which historically has been the largest source of
information drift.

## 2.26 Multi-publisher batch endpoints

A federated aggregator that needs to fetch the latest milestones
from many publishers in one request uses the multi-publisher batch
endpoint:

```
POST /roadmap/v1/batch/milestones
{
  "publishers": ["did:wia:org:a", "did:wia:org:b", ...],
  "since": "RFC 3339"
}
```

The aggregator's request must be signed; the response is a
multi-part NDJSON stream with one envelope per line. Batching is
purely a network-efficiency optimisation; the underlying
signatures and replay-defence semantics are unchanged.

## Operational coda

Adoption of this standard is incremental: an organisation can
begin by publishing a single milestone envelope and incrementally
expand to dependencies, releases, progress updates, commitment
changes, and federation. The standard is intentionally additive.
There is no minimum subset of envelope types that must be emitted
to participate; the only requirement is that the envelopes that
are emitted be properly signed and structurally correct.

The cumulative behaviour of all such organisations is a
roadmap commons that no single party owns and that downstream
consumers can analyse without negotiating bilateral access. That
commons is the public-good outcome the standard exists to enable.

弘益人間 — Benefit All Humanity. The roadmap is a public document by
intention and a verifiable document by construction.
