# PHASE 3 — Federation Protocol

> Cross-advisor and cross-jurisdiction federation, replay defence,
> investor-portability handshake, and aggregator trust binding.

## 3.1 Why federate at all

A typical investor today holds positions across multiple advisors,
custodians, and account types (taxable, retirement, education,
trust). The investor's view of their financial life is a stitched-
together composite that no single advisor controls. The federation
protocol exists so that the investor — not any single advisor — owns
the stitching authority, and so that aggregators can verify the
authenticity of stitched data without fragile screen-scraping.

## 3.2 The federation handshake

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "federation_handshake",
  "handshake_id": "ULID",
  "initiator": "did:wia:advisor:...",
  "counterparty": "did:wia:advisor:..." | "did:wia:aggregator:...",
  "investor_consent_ref": "did:wia:consent:...",
  "scope": ["portfolio:read", "fee:read", "tlh:read"],
  "max_envelope_age_days": 365,
  "tlp": "AMBER",
  "valid_until": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

Every handshake is gated by a referenced `investor_consent`; an
advisor cannot federate another advisor's data without explicit
investor authorisation, period.

## 3.3 Replay defence

Federated envelopes carry a 96-bit nonce and are accepted within a
±300-second wall-clock skew. The receiving advisor maintains a
replay cache for at least 600 seconds; any envelope whose nonce was
seen previously within that window is silently dropped (and a
`federation_audit` event is emitted with the duplicate-nonce
reason).

## 3.4 Aggregator trust binding

Aggregators (Yodlee, Plaid, KFTC OpenBanking-style brokers) operate
as a special class of counterparty. Their handshake includes an
additional `aggregator_certification` field referencing an
external attestation that the aggregator complies with the
applicable jurisdictional rules (US Reg E for ACH, EU PSD2 for
account information service providers, KR 마이데이터 for Korea).

## 3.5 Per-investor revocation

Investors revoke a federation grant by signing an
`investor_consent_revocation` envelope. The advisor MUST stop
emitting envelopes to the counterparty within 60 seconds of
receiving a verified revocation; the counterparty MUST stop
querying within the same window. The 60-second SLA exists because
revocation is the most time-sensitive control in the entire
protocol.

## 3.6 Cross-jurisdictional considerations

When the initiator and counterparty are in different jurisdictions,
the handshake records both `kyc_jurisdiction` values and the
`data_localisation_constraints` block. Some jurisdictions (KR
개인정보 국외이전, EU GDPR Chapter V, CN PIPL) require additional
attestation for cross-border data flow; the standard provides the
binding fields but does not adjudicate the legal sufficiency.

## 3.7 Disputes

A dispute between counterparties (e.g., advisor A claims aggregator
B mis-attributed performance) is recorded as a `federation_dispute`
envelope referencing the contested envelopes by identifier. The
dispute does not unwind the underlying envelopes — those remain
signed history — but it does flag downstream consumers to apply
caution.

## 3.8 Aggregator data freshness contract

Aggregators consuming federated portfolio data publish a
`freshness_attestation` envelope that names, per investor, the
maximum age of any envelope they will surface in their UI. The
intent is to prevent the worst aggregator failure mode — stale
balances surfaced as "current" — by making freshness a public
contract.

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "freshness_attestation",
  "aggregator_id": "did:wia:aggregator:...",
  "investor_id": "did:wia:investor:...",
  "max_envelope_age_seconds": 0,
  "attested_at": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

Freshness attestations are emitted at least once per hour during
business days and once per day on weekends and holidays.

## 3.9 Investor data portability

The investor-portability subprotocol allows an investor to migrate
their full envelope history from one advisor to another without
losing the chain of trust. The receiving advisor verifies each
envelope's signature against the originating advisor's public key
(retained in WIA-OMNI-API trust fabric) and re-emits a
`portability_receipt` envelope acknowledging the imported history.

The originating advisor MUST NOT charge an exit fee for the
portability transfer beyond the actual cryptographic and
infrastructure cost; this is an enforceable conformance
requirement, not a recommendation.

## 3.10 Federation transport independence

Federation envelopes are carried over mutual-TLS HTTP by default,
with optional support for AMQP 1.0, NATS JetStream, and Kafka
between authenticated peers. The signature discipline is the
integrity guarantee; the transport is interchangeable. Aggregators
may negotiate the transport during the handshake.

## 3.11 Conformance test corpus

A reference test corpus is maintained at
`https://wiastandards.com/robo-advisor/conformance/` containing:

- 24 valid federation handshakes spanning every scope combination;
- 12 invalid handshakes (expired consent, missing nonce, replayed
  nonce, jurisdiction mismatch, scope overreach);
- 6 revocation flows with timing assertions;
- 3 portability scenarios across advisor mergers and acquisitions.

Conforming implementations MUST pass the entire corpus and
MUST publish their conformance score as a signed
`conformance_attestation` envelope.

## 3.12 Operational considerations

Federation introduces additional latency to investor-facing dashboards;
implementations SHOULD cache the latest signed envelope per investor
per counterparty for at most 60 seconds during business hours and 5
minutes off-hours, refreshing on demand. The cache discipline keeps
the federation read path responsive without overstating freshness.

Aggregators with rate-limit constraints from upstream advisors SHOULD
implement adaptive backoff using the `Retry-After` header (Phase 2
§2.9). Hard rate-limit failures are recorded as `federation_audit`
events with the upstream advisor as the named counterparty so that
mutual visibility into rate-limit pressure is preserved.

## 3.13 Backwards compatibility

Implementations migrating from screen-scraping aggregation MAY emit
both screen-scraped balances and federated `portfolio` envelopes
during a transitional window. The federated envelope is canonical;
the screen-scraped balance is advisory and MUST be discarded once
federation has been established for an investor.

## 3.14 Cross-reference to Phase 4

Phase 4 covers integration with custodian, market data, and
regulatory reporting systems. Federation in Phase 3 is the
investor-facing peer-to-peer surface; Phase 4 is the
business-to-business integration surface. The two share the
envelope format but operate under different consent and trust
models.

## 3.15 Investor self-service tooling

A reference investor self-service surface ships under
`tools/portability/` with subcommands `consent grant`, `consent
revoke`, `export-history`, and `import-history`. The CLI is signed
by the investor's hardware-bound key (FIDO2 / passkey) and emits
the same envelopes the network protocol uses. The intent is that
an individual investor, not just an institutional advisor, can
exercise portability without depending on a vendor portal.

The CLI is also the reference test surface for the conformance
corpus described above; conforming aggregators MUST be exercisable
from the CLI without modification.

## 3.16 Audit retention

`federation_audit` envelopes are retained by both peers for a
minimum of 7 years to align with the longest applicable
record-retention obligation across the major jurisdictions in
scope (US SEC Rule 17a-4: 6 years; KR 자본시장법: 10 years for
some categories; EU MiFID II: 5 years minimum, 7 years on
extension). The 7-year baseline absorbs the common case and the
standard provides a clean override field for tenants subject to
longer regimes.

## 3.17 Reference list

- ISO/IEC 27001 — security management baseline for advisor tenants
- ISO 20022 — payment messaging used in Phase 4 settlement linkage
- RFC 9421 — HTTP Message Signatures
- RFC 9457 — Problem Details for HTTP APIs
- RFC 8785 — JSON Canonicalisation Scheme (JCS)
- RFC 8949 — CBOR

## 3.18 Federation telemetry

Each peer publishes a `federation_telemetry` envelope on a daily
cadence summarising envelope volume, replay-cache hit rate,
revocation latency, and dispute volume. The telemetry stream is
queryable by both peers and is the operational basis for SLA
monitoring across the federation.

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "federation_telemetry",
  "peer_a": "did:wia:advisor:...",
  "peer_b": "did:wia:advisor:...",
  "window_start": "RFC 3339",
  "window_end":   "RFC 3339",
  "envelope_count_a_to_b": 0,
  "envelope_count_b_to_a": 0,
  "replay_drop_count": 0,
  "revocation_latency_p95_ms": 0,
  "dispute_count": 0,
  "signature_a": "Ed25519",
  "signature_b": "Ed25519"
}
```

## 3.19 Worked example: aggregator onboarding

An investor signs a `federation_handshake` consent that names
aggregator A as the recipient, scope `portfolio:read fee:read`,
and TLP AMBER. Within 60 seconds the originating advisor begins
emitting matching envelopes to aggregator A's inbound endpoint.
Three weeks later the investor revokes the grant; the originating
advisor stops emissions within 60 seconds of receiving the signed
revocation, the aggregator stops queries within the same window,
and a `federation_audit` envelope on both sides records the
revocation timestamp. The investor's view of the audit chain
proves both the start and end of the data flow without either
peer needing to be trusted on its bare word.

## 3.20 Korean MyData boundary

Korean MyData (마이데이터) consent flows are gated by the Korea
Financial Telecommunications and Clearings Institute's central
consent broker. Advisors operating in Korea bind their
`investor_consent` envelopes to MyData consent identifiers via
the `mydata_consent_ref` field. The dual reference allows a
Korean regulator to verify both the WIA-signed envelope and the
MyData broker record without manual reconciliation.

The standard does not duplicate MyData's consent management; it
provides the interoperability binding so that an advisor
operating in Korea and several other jurisdictions does not need
a fully separate consent stack per market.

## 3.21 Cross-jurisdictional dispute resolution

When a dispute (§3.7) crosses jurisdictions, the standard provides
no single arbitration mechanism — that is properly a matter for
each jurisdiction's regulator and courts. The standard does
provide a `dispute_referral` envelope that names the chosen
forum (regulator, ombudsman, arbitration body) and the contested
envelope identifiers, so that downstream consumers can flag the
contested data even while resolution is pending. The chain of
envelopes from the original handshake through revocation and
dispute referral forms a complete, signed record that the chosen
forum can evaluate without depending on either party's narrative.

## 3.22 Implementation maturity model

A reference implementation maturity model is published with three
levels: **Federated-Read** (consumes signed envelopes from upstream
peers), **Federated-Read-Write** (also emits envelopes consumed by
downstream peers), and **Federation-Quorum-Member** (participates
in dispute arbitration and maintains the conformance corpus
locally for offline verification). The maturity level is
self-declared in the capability advertisement and is not a
gating requirement for participation.

## 3.23 Reference list

- WIA-OMNI-API — credential and trust-fabric companion standard
- ISO/IEC 27001 — information security management
- ISO/IEC 27701 — privacy information management extension
- RFC 8949 — CBOR (alternative envelope encoding)
- RFC 9421 — HTTP Message Signatures
- RFC 9457 — Problem Details
- KR 마이데이터 운영 가이드 (Financial Services Commission)

弘益人間 — Benefit All Humanity. The standard exists so that the smallest organisation can sign one envelope and be heard by the largest.
