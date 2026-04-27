# PHASE 2 — API Interface

> HTTP surface for advisor onboarding, portfolio query, rebalance
> publication, fee disclosure retrieval, and event streaming.
> All requests carry HTTP Message Signatures (RFC 9421); error
> responses use Problem Details (RFC 9457).

## 2.1 Endpoint surface

```
POST /ra/v1/investors                    Create investor profile
GET  /ra/v1/investors/{id}               Fetch latest signed profile
GET  /ra/v1/investors/{id}/history       Profile history (audit)

POST /ra/v1/portfolios                   Bind a new portfolio
GET  /ra/v1/portfolios/{id}              Latest portfolio snapshot
GET  /ra/v1/portfolios/{id}/series       Time series (TWR daily)

POST /ra/v1/rebalance                    Publish a rebalance event
GET  /ra/v1/rebalance/{id}               Rebalance detail

GET  /ra/v1/fees/{investor_id}           Latest fee disclosure
GET  /ra/v1/fees/{investor_id}/history   Disclosure history

POST /ra/v1/tlh                          Publish a tax-loss harvest event
GET  /ra/v1/tlh?portfolio_id=...         Query TLH events

GET  /ra/v1/stream/{investor_id}         SSE: live envelope stream
```

## 2.2 Authentication and signing

Every request MUST carry an HTTP Message Signature over the
mandatory `(request-target)`, `@authority`, `content-digest`,
`@created`, and `@expires` covered components. The signing key is
the advisor's tenant key registered with the WIA-OMNI-API trust
fabric. Bearer-token-only authentication is rejected.

## 2.3 Idempotency

POST endpoints accept an `Idempotency-Key` header (UUIDv7
recommended). A repeated request with the same key within 24 hours
returns the original response without re-creating the resource.

## 2.4 Pagination

List endpoints use cursor pagination (`?cursor=...`) with `Link`
headers (RFC 8288). Page size is capped at 200 envelopes per page
to bound replay-defence cache pressure.

## 2.5 Streaming semantics

The `/stream/{investor_id}` endpoint emits Server-Sent Events. Each
event is a complete signed envelope (no partial frames). The server
replays the last 60 seconds of events on connect to absorb client
reconnects without missing emissions; the client is responsible for
deduplicating by envelope identifier.

## 2.6 Error envelope

Errors are returned as Problem Details (RFC 9457) with extension
fields specific to robo-advisor semantics:

```
{
  "type": "https://wiastandards.com/robo-advisor/errors/insufficient-cash",
  "title": "Insufficient settled cash for rebalance",
  "status": 422,
  "detail": "Required: 12,400.00 USD; Available: 9,820.00 USD",
  "investor_id": "did:wia:investor:...",
  "rebalance_id": "..."
}
```

## 2.7 Versioning

The wire format version is carried in the envelope itself
(`wia_robo_advisor_version`). The HTTP surface is versioned in the
URL (`/ra/v1`). Breaking changes to either are released under a new
URL prefix; non-breaking additions are advertised in the
`/.well-known/wia-robo-advisor` capability document.

## 2.8 Capability discovery

```
GET /.well-known/wia-robo-advisor
```

Returns a signed `capability_advertisement` envelope listing the
endpoint surface, supported envelope types, supported jurisdictions
for tax-loss-harvesting wash-sale defence, and the active algorithm
attestations. Capability discovery is what allows downstream
aggregators (consumer dashboards, regulatory reporting tools) to
adapt automatically to advisor implementation differences.

## 2.9 Rate limits

The default rate limit is 600 requests per minute per advisor
client identity. Burst capacity is 100 requests in any single
second. Rate-limit responses use HTTP 429 with `Retry-After`
seconds.

## 2.10 Content negotiation

Envelope payloads are JSON by default; CBOR (RFC 8949) is
negotiated via `Accept: application/cbor`. The signature covers the
canonical JSON form regardless of the on-wire encoding.

## 2.11 Webhook callback envelope

Advisors may register webhook callbacks for downstream consumers
(custodian, compliance vendor, investor dashboard) to receive
envelopes asynchronously without polling. The callback payload is
the full signed envelope; the receiver MUST verify the signature
before acting on the content. Callback delivery uses exponential
backoff (1s, 2s, 4s, 8s, 16s, 32s, 64s, then dead-letter at the
8th attempt).

```
POST /ra/v1/webhooks
{
  "url": "https://...",
  "events": ["portfolio", "rebalance", "fee_disclosure", "tlh"],
  "investor_filter": "did:wia:investor:..." | null,
  "shared_secret_hint": "k_2026q2",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The `shared_secret_hint` allows downstream consumers to rotate the
HMAC key used in the `X-WIA-Signature` callback header without
re-registering the webhook. The actual secret is exchanged out of
band during initial onboarding.

## 2.12 Bulk export

For regulatory inspections and investor data portability requests, a
bulk export endpoint returns a signed manifest plus per-envelope
content streamed as NDJSON:

```
POST /ra/v1/exports
{ "investor_id": "...", "from": "RFC 3339", "to": "RFC 3339" }

→ 202 Accepted, Location: /ra/v1/exports/{export_id}

GET /ra/v1/exports/{export_id}/manifest
GET /ra/v1/exports/{export_id}/content
```

The manifest carries a Merkle root over the included envelopes so
that the receiver can verify completeness without re-signing every
envelope individually. Bulk exports are subject to investor consent
under GDPR Article 20, KR 개인정보 이동권 (Article 35-2), and
analogous portability rules in other jurisdictions.

## 2.13 Health and observability

```
GET /ra/v1/health        → liveness probe
GET /ra/v1/ready         → readiness probe
GET /ra/v1/metrics       → Prometheus exposition (RFC 9415-aligned)
```

Health endpoints do not require authentication and do not return
investor-specific data. The metrics endpoint is gated by mTLS to
the internal observability mesh.

## 2.14 Backwards compatibility

Implementations migrating from a private API SHOULD bridge
incoming requests to the standard surface during a transitional
window of at least 18 months. The bridge SHOULD emit a
`deprecation_warning` extension on Problem Details responses so that
client teams can plan the migration without a hard cutover.

## 2.15 Custodian-side bridge

Custodians implementing this surface as the primary investor-facing
API SHOULD additionally expose a bridge to legacy SWIFT MT940/MT942
account statement consumers. The bridge receives the bulk-export
manifest above and emits MT940/MT942 messages on a daily cycle so
that investors with legacy treasury workflows are not forced to
adopt the JSON envelope before they are ready.

## 2.16 Read-only consumer guidance

Investor-facing dashboards, comparison tools, and journalists
typically need only a read-only subset of this surface. The
standard provides a `read_only_credentials` endpoint that mints a
scoped token (portfolio + fee endpoints only, no rebalance, no
TLH) for those use cases:

```
POST /ra/v1/credentials/read-only
{ "purpose": "investor-dashboard" | "research" | "regulator-readonly" }
→ 201 Created, returns a short-lived bearer token bound to the
  caller's tenant and scope.
```

This avoids the anti-pattern of consumers re-using full advisor
credentials for read-only display.

## 2.17 Operational considerations

Latency budgets for portfolio queries SHOULD target p95 < 250 ms
under normal load. Rebalance publication is asynchronous; the
acknowledgement is HTTP 202 with a `Location` header pointing to
the eventual `rebalance` envelope. Settlement reconciliation
(matching executed trades to custodian confirmations) is out of
scope for this Phase but is the natural integration point for
ISO 20022 messaging defined in Phase 4.

## 2.18 Long-poll fallback

Environments that cannot maintain SSE connections (corporate proxies,
some mobile carriers) MAY use long-poll on `GET /ra/v1/poll/{investor_id}`
with the `?since=<envelope_id>` parameter. The server holds the
connection for up to 30 seconds awaiting new envelopes; on timeout
the response is HTTP 204 No Content and the client re-polls. The
standard documents long-poll as a fallback rather than a primary
path because connection churn under long-poll triples the server-
side load relative to SSE for the same delivery cadence.

## 2.19 Investor data export

The investor-portability subprotocol (Phase 3 §3.9) is exposed
through this surface as:

```
POST /ra/v1/portability/export
{
  "destination_advisor_id": "did:wia:advisor:...",
  "scope": ["all-portfolios", "fees-history", "tlh-history"]
}
```

The response is HTTP 202 with a destination-readable bundle URL.
Bundle download is signed by the investor's hardware key; the
destination advisor verifies the investor's signature before
importing the bundle. Bundle import emits a `portability_receipt`
envelope back to the originating advisor.

## 2.20 Replay and idempotency cache

The replay-defence cache (Phase 1 §3.3) is a per-tenant in-memory
structure with a target footprint of 100 MB per million envelopes/
day. Implementations MUST NOT persist nonces across process
restarts beyond a 600-second window; persistence longer than the
acceptance window provides no defensive value and complicates
rolling deployments.

## 2.21 Backwards compatibility

Pre-standard advisor APIs that returned tabular extracts (CSV,
Excel) MAY continue to do so under URL paths outside the `/ra/v1`
prefix. Tabular extracts are not signed and do not constitute
canonical evidence; the signed envelopes under `/ra/v1` are
canonical for all audit purposes.

## 2.22 Field-level encryption

Sensitive fields (national identifiers, account numbers, beneficiary
names) are field-level encrypted using AES-256-GCM with per-tenant
keys held in the tenant's HSM. The encrypted form is what appears
in the canonical envelope; the signature covers the encrypted form,
not the plaintext, so that signature verification does not require
key access. Decryption is gated by Phase 3 federation scope and
investor consent.

## 2.23 Versioned schema registry

A schema registry is exposed at `GET /ra/v1/schemas/{type}/{version}`
returning the JSON Schema for the named envelope type at the named
version. Schemas are immutable; new versions are published under
new identifiers. Conforming consumers MUST validate envelopes
against the schema version named in the envelope's
`wia_robo_advisor_version` field.

## 2.24 Test surface

A read-only test surface is exposed at `https://sandbox.wiastandards.com/ra/v1`
returning realistic synthetic envelope traffic. The sandbox is
useful for client-side conformance testing, dashboard development,
and regulator-side familiarisation. Sandbox traffic is signed by
the WIA test key; signatures will not verify against production
trust roots, by design.

## 2.25 Capability advertisement worked example

```
GET /.well-known/wia-robo-advisor

200 OK
Content-Type: application/json
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "capability_advertisement",
  "advisor_id": "did:wia:advisor:acme-eu",
  "endpoint_surface_version": "ra/v1",
  "supported_envelope_types": [
    "investor_profile", "portfolio", "rebalance",
    "fee_disclosure", "tax_loss_harvest",
    "algorithm_attestation", "glide_path_attestation",
    "settlement_confirmation", "freshness_attestation"
  ],
  "supported_jurisdictions_tax": ["US", "GB", "DE", "KR", "JP"],
  "supported_currencies": ["USD", "EUR", "GBP", "KRW", "JPY"],
  "active_algorithm_attestations": [ "...", "..." ],
  "rate_limits": { "rpm": 600, "burst_per_second": 100 },
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

A consumer that has never integrated with the advisor before reads
this single document and learns everything needed to begin
correctly: which envelope types to expect, which jurisdictions are
in scope for tax handling, which currencies are supported, and
which algorithms are currently in production.

The capability document is itself signed by the advisor, so a
consumer can pin the advisor's identity at integration time and
detect impersonation attempts that would otherwise be invisible.

弘益人間 — Benefit All Humanity. The standard exists so that the smallest organisation can sign one envelope and be heard by the largest.
