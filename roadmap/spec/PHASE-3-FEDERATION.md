# PHASE 3 — Federation Protocol

> Cross-organisation roadmap federation, replay defence,
> external dependency declaration, and downstream subscription
> management.

## 3.1 Why federate at all

The most useful question a roadmap can answer is rarely about a
single organisation. Most consequential roadmap questions are
cross-organisational: when will the post-quantum cryptographic
ecosystem be ready end-to-end; when will the major browsers all
support a given web feature; when will the major cloud providers
all offer a given regulatory certification. Answering those
questions today requires reading prose roadmaps from a dozen
organisations and assembling the answer manually. Federation
lets the assembly happen automatically against signed envelope
streams.

## 3.2 The federation handshake

```
{
  "wia_roadmap_version": "1.0.0",
  "type": "federation_handshake",
  "handshake_id": "ULID",
  "initiator": "did:wia:org:...",
  "counterparty": "did:wia:org:...",
  "scope": ["milestone:read", "release:read",
            "progress:read", "commitment_change:read"],
  "max_envelope_age_days": 730,
  "tlp": "GREEN",
  "valid_until": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

Most roadmap federation operates at TLP GREEN or CLEAR — roadmaps
are usually intended for broad consumption — but the protocol
supports the full TLP range so that organisations with sensitive
strategic milestones can federate selectively.

## 3.3 Replay defence

Federated envelopes carry a 96-bit nonce and are accepted within a
±300-second wall-clock skew. The receiving organisation maintains a
replay cache for at least 600 seconds; any envelope whose nonce
was seen previously within that window is silently dropped. A
`federation_audit` envelope records duplicate-nonce drops for
mutual visibility.

## 3.4 External dependency declaration

When organisation A's milestone depends on organisation B's
milestone, A publishes a `dependency` envelope with `depends_on`
pointing to B's `milestone_id` and `depends_on_owner` set to B's
DID. The dependency declaration is unilateral — A does not need
B's permission to declare a dependency on B's published roadmap —
but the declaration becomes meaningful when B's milestone slips,
because A's downstream consumers can automatically infer that A's
schedule is at risk.

Organisations may publish a `dependency_acknowledgement` envelope
counter-signing a dependency declaration to confirm that the
upstream is aware of the downstream consumer. Acknowledgements
are advisory; they do not create contractual obligations.

## 3.5 Per-consumer revocation

A publisher may revoke a federation handshake at any time by
emitting a signed `federation_revocation`. Subsequent envelopes
are not delivered to the revoked counterparty. Revocation is the
mechanism by which a publisher can cut off a counterparty that has
been observed misusing the federated data (for example,
republishing TLP AMBER content as TLP CLEAR).

## 3.6 Cross-jurisdictional considerations

Roadmap envelopes are typically not personal data and rarely
trigger GDPR / KR PIPA / CN PIPL constraints. They MAY contain
personal data when, for example, an `approved_by` field names an
individual officer; in that case the publisher is responsible for
the personal-data handling, not the protocol.

## 3.7 Subscription endpoints

A consumer subscribes to a publisher's federated envelope stream
via:

```
POST /roadmap/v1/subscriptions
{
  "subscriber": "did:wia:org:...",
  "envelope_types": ["milestone", "release", "commitment_change"],
  "delivery_url": "https://...",
  "filter_tags": ["security", "pqc"]
}
```

Subscription delivery uses the same exponential backoff as
webhook callbacks (Phase 2 §2.11). Subscriptions are revocable
from either side.

## 3.8 Federation telemetry

Each peer publishes a `federation_telemetry` envelope on a daily
cadence summarising envelope volume, replay-cache hit rate,
revocation latency, and subscription count. The telemetry stream
is queryable by both peers and is the operational basis for SLA
monitoring across the federation.

## 3.9 Worked example: post-quantum readiness

A non-profit consortium tracking post-quantum cryptographic
readiness across the major TLS implementations subscribes to
federated milestone streams from a dozen library publishers
(OpenSSL, BoringSSL, NSS, wolfSSL, mbedTLS, rustls, s2n-tls,
GnuTLS, JDK TLS, Go crypto/tls, .NET TLS, AWS-LC). The consortium
emits a quarterly `aggregated_readiness_report` envelope summarising
the cross-organisation state. The report is itself a published
envelope, so other consumers (browsers, cloud providers, government
agencies) can subscribe to the consortium's report stream rather
than re-implementing the aggregation themselves.

The above flow involves no central coordinator, no shared
database, and no manual report assembly. Each library publishes
its own milestones; the consortium aggregates them; downstream
consumers read the aggregate. The chain of signatures from the
original library's milestone to the consortium's aggregate is
fully verifiable.

## 3.10 Conformance test corpus

A reference test corpus is maintained at
`https://wiastandards.com/roadmap/conformance/` containing valid
federation handshakes, invalid handshakes (expired, missing nonce,
replayed nonce, scope overreach), revocation flows with timing
assertions, and aggregator subscription scenarios. Conforming
implementations MUST pass the entire corpus and MUST publish
their conformance score as a signed `conformance_attestation`
envelope.

## 3.11 Operational considerations

Federation introduces additional latency to consumer dashboards.
Implementations SHOULD cache the latest signed envelope per
publisher per envelope-type for at most 60 seconds during the
publisher's local business hours and 5 minutes off-hours, with
on-demand refresh.

## 3.12 Backwards compatibility

Implementations migrating from a private roadmap publication API
SHOULD bridge incoming requests to the standard surface during a
transitional window of at least 18 months. The bridge SHOULD
emit `deprecation_warning` extensions on Problem Details responses
so that client teams can plan the migration without a hard
cutover.

## 3.13 Reference list

- W3C DID Core 1.0 — identity references
- ISO/IEC 27001 — security management baseline
- RFC 8785 — JSON Canonicalisation Scheme (JCS)
- RFC 8949 — CBOR
- RFC 9421 — HTTP Message Signatures
- RFC 9457 — Problem Details

弘益人間 — Benefit All Humanity. Federation exists so that the smallest
project's roadmap can be a peer to the largest organisation's.

## 3.14 Aggregator topology

Federation is hub-and-spoke at small scale and mesh at larger
scale. The standard does not mandate a topology; a single
publisher may participate in both hub-and-spoke arrangements (a
small project subscribing to a single aggregator) and mesh
arrangements (a major library publishing directly to dozens of
direct subscribers). The federation envelope semantics are
identical regardless of topology.

## 3.15 Trust escalation

A counterparty that observes another counterparty publishing
envelopes that fail signature verification, replay defence, or
TLP discipline MAY emit a `trust_escalation` envelope citing the
contested envelopes. The escalation is advisory; it does not
automatically revoke any handshake. Repeated escalations against
the same counterparty by independent observers are a signal that
downstream consumers may want to reduce reliance on the
counterparty.

## 3.16 Independent observer role

The standard recognises a third class of federation participant:
the independent observer. Observers do not publish their own
roadmaps; they subscribe to other publishers' streams and emit
analyses (aggregates, calibration scoring, slip-rate rankings).
Observers operate under their own DID identity; their analyses
are signed envelopes traceable to the observer.

The observer role is what makes roadmap honesty self-policing.
A publisher that habitually slips can be ranked publicly by an
observer, and the ranking is itself a signed envelope, so the
ranking cannot be retroactively edited if the publisher's
performance later improves.

## 3.17 Federation across the WIA family

When a publisher operates multiple WIA-family standards (e.g.,
both WIA Roadmap and WIA Standards), the federation handshakes
for each standard are independent. A counterparty that has
federated for roadmap data does not automatically have access to
standards-specification data, and vice versa. Scope isolation is
a deliberate property of the protocol.

## 3.18 Worked example: independent observer

A non-profit observer subscribes to milestone and commitment-
change streams from 50 publishers in the post-quantum cryptographic
ecosystem. The observer maintains a published `pqc_readiness_index`
envelope updated weekly, summarising aggregate readiness, top
slippers, top calibration outperformers, and consensus
estimated-completion date. Three downstream consumers (a browser
vendor, a cloud provider, a national security agency) subscribe to
the observer's index rather than re-implementing the aggregation.

The observer's signature is what gives the index its authority,
and the underlying publisher signatures are what make the index
verifiable end-to-end.

## 3.19 Federation onboarding playbook

A new publisher onboarding to federation typically follows this
sequence:

1. Register a tenant DID with WIA-OMNI-API and publish the
   organisation's signing public key.
2. Stand up a Phase 2 endpoint surface and publish the capability
   advertisement.
3. Produce 3 to 6 months of milestone and progress envelope
   history to populate the historical record.
4. Initiate federation handshakes with the consortia and
   aggregators relevant to the publisher's domain.
5. Maintain weekly progress updates and quarterly calibration
   reports as a steady-state cadence.

The onboarding cost is dominated by item 1 (tenant key
provisioning) and item 3 (back-filling history). Both are one-time
costs; steady-state federation has near-zero per-envelope marginal
cost.

弘益人間 — Benefit All Humanity. Federation makes a small project's
calendar visible alongside a multinational's.

## 3.20 Federation health metrics

The standard defines a baseline set of health metrics that every
federation participant SHOULD publish:

- envelopes published per day (with 7-day rolling p95)
- envelopes received per federated peer per day
- replay-cache hit rate (per peer)
- handshake renewal latency (median, p95)
- subscription delivery success rate (per subscriber)

The metrics are exposed via the Phase 2 `/metrics` endpoint and
are also queryable via the federated `federation_telemetry`
envelope stream. Mutual visibility into health is what allows a
peer to identify a degrading counterparty before the degradation
becomes a verification failure.

## 3.21 Independence and decentralisation

The standard intentionally does not designate a single root of
trust for federation. Each publisher is its own trust root, and
the WIA-OMNI-API trust fabric is one of several possible
trust-anchoring substrates. Publishers MAY anchor to other
substrates (a national regulator's identity portal, a
self-hosted DID method, a commercial PKI) without losing
interoperability, provided the publisher's public key is
discoverable via the chosen substrate.

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
