# WIA-legal-system-harmonization PHASE 3 — Protocol Specification

**Standard:** WIA-legal-system-harmonization
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the wire-level protocols used by
WIA-legal-system-harmonization participants for
discovery, ELI / ECLI resolution, Akoma Ntoso
distribution, signed publication, treaty depositary
exchange, harmonisation-map propagation, and audit
evidence transport.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP Semantics), RFC 9112 (HTTP/1.1), RFC 9114 (HTTP/3)
- IETF RFC 8446 (TLS 1.3), RFC 6797 (HSTS)
- IETF RFC 8615 (Well-Known URIs), RFC 7515 (JWS), RFC 7517 (JWK)
- IETF RFC 9421 (HTTP Message Signatures), RFC 9530 (Digest Fields)
- IETF RFC 5005 (Atom paged feeds), RFC 8259 (JSON), RFC 8785 (JCS)
- OASIS Akoma Ntoso 1.0, OASIS LegalRuleML 1.0
- ELI / ECLI Council Conclusions
- W3C JSON-LD 1.1, W3C SHACL

---

## §1 Scope

This PHASE defines the on-the-wire behaviour between
clients and registries; between official-publishing
authorities and the registry; between treaty
depositaries and ratifying parties; and between
harmonisation-map publishers and downstream consumers.

## §2 Discovery

A WIA-legal-system-harmonization registry serves a
discovery document at:

```
GET /.well-known/wia/legal-system-harmonization
```

Response (`application/json`):

```json
{
  "registry": "https://lsh.wiastandards.com",
  "openapi": "https://lsh.wiastandards.com/openapi.json",
  "operationGroups": ["/v1/works", "/v1/expressions",
                      "/v1/manifestations", "/v1/items",
                      "/v1/authorities", "/v1/citations",
                      "/v1/treaties", "/v1/harmonisation",
                      "/v1/registry"],
  "schemas": {
    "akomaNtoso": "1.0",
    "legalRuleML": "1.0",
    "eli": "2012/C 325/02",
    "ecli": "2011/C 127/01"
  },
  "keySet": "https://lsh.wiastandards.com/.well-known/jwks.json"
}
```

The discovery document is signed (RFC 9421).

## §3 Transport

HTTPS with TLS 1.3 (RFC 8446) and HSTS preload.
Atom feeds support both pull and push (WebSub /
PubSubHubbub for delta delivery).

## §4 Content negotiation

| Accept                                | Use                                      |
|---------------------------------------|------------------------------------------|
| `application/akn+xml`                 | Akoma Ntoso 1.0 XML                      |
| `application/legalruleml+xml`         | LegalRuleML 1.0                          |
| `application/json`                    | record bodies                            |
| `application/ld+json`                 | JSON-LD records                          |
| `application/pdf`                     | PDF manifestations                       |
| `application/epub+zip`                | EPUB manifestations                      |
| `application/atom+xml`                | Atom feeds                               |
| `application/problem+json`            | error                                    |

## §5 ELI resolution

Clients resolve ELI URIs against the registry:

```
GET /resolve?eli=/eli/<jurisdiction>/<type>/<year>/<number>
```

The registry returns a 303 redirect to the canonical
expression URL when a single match exists, or a
disambiguation page when more than one match exists.

## §6 ECLI resolution

ECLI URIs follow the same `GET /resolve` flow:

```
GET /resolve?ecli=ECLI:<country>:<court>:<year>:<id>
```

Resolution returns the canonical expression record of
the corresponding court decision.

## §7 Signed publication

Expressions and manifestations are signed with
detached JWS (RFC 7515) over the canonical document
(RFC 8785 for JSON, c14n for XML). The `kid`
references the publishing authority's key in the
JWKS. Verification chains:

1. JWKS signature against authority record key set;
2. Authority LEI verification at GLEIF;
3. Document digest verification against the
   manifestation `digestRef`.

## §8 Identifiers

| Identifier         | Format                                          |
|--------------------|-------------------------------------------------|
| `workRef`          | ELI URI                                         |
| `expressionRef`    | ELI URI with language and date                  |
| `manifestationRef` | ELI URI with format                             |
| `itemRef`          | repository-prefixed URI                         |
| `authorityRef`     | URI                                             |
| `citationRef`      | UUID                                            |
| `treatyRef`        | URI                                             |
| LEI                | ISO 17442                                       |
| ECLI               | per Council Conclusion 2011/C 127/01            |

## §9 Caching and immutability

Manifestations and items are immutable; they carry
`Cache-Control: public, max-age=31536000, immutable`.
Expressions are mutable through consolidation; they
carry strong `ETag` and short max-age.

## §10 Federation

Federated registries form a directed graph in the
discovery document. Cross-registry queries follow
the graph. National official-publishing registries
typically peer with regional registries (e.g. EUR-Lex,
EUR-CJEU, OAS, AU); peering is bilateral and
revocable.

## §11 Replay and unpublishing

Expressions are not deleted; they are tombstoned by
withdrawal or repeal. Withdrawal events emit a new
expression record with `state` set to `withdrawn`.
The original expression remains queryable for
historical research. Replay attacks on signed
publications are prevented by `iat`/`exp` JWS
claims (max 24h on issuance).

## §12 Error semantics

Errors are `application/problem+json` (RFC 9457).
Protocol-level codes:

| Code | Meaning                                              |
|------|------------------------------------------------------|
| 200  | success                                              |
| 303  | ELI / ECLI resolution redirect                       |
| 304  | conditional GET unchanged                            |
| 400  | malformed Akoma Ntoso, ELI, ECLI                     |
| 401  | missing or invalid token                             |
| 403  | authority not authorised for the act type            |
| 410  | tombstone (repealed, withdrawn)                      |
| 422  | schema violation                                     |
| 426  | TLS upgrade required                                 |
| 503  | federation peer unavailable                          |

## §13 Observability

Servers SHOULD emit OpenTelemetry traces with
`wia.lsh.operation`, `wia.lsh.jurisdiction`,
`wia.lsh.actType`, and `wia.lsh.workRef`
attributes.

## Annex A — Conformance levels

- **Tier 1 — Self-declared:** discovery served,
  expressions signed.
- **Tier 2 — Verified:** Akoma Ntoso schema audited;
  ELI / ECLI resolution exercised; LEI verification
  exercised.
- **Tier 3 — Anchored:** continuous evidence stream
  per PHASE-4 Annex G; Hague Apostille e-register
  interop verified.

## Annex B — Atom feed format

Each operation group publishes an Atom 1.0 feed at
`/v1/<group>/feed` with:

- `<id>`: feed URI;
- `<updated>`: most recent change;
- `<entry>` for each record with `<published>`,
  `<updated>`, `<title>`, `<summary>`, `<link>`.

WebSub hub URLs are advertised in the feed `<link
rel="hub">`.

## Annex C — XML canonicalisation

XML Akoma Ntoso documents are canonicalised per
W3C Canonical XML 1.1 prior to signing. The
canonical form excludes white-space-only text nodes
and normalises namespace declarations.

## Annex D — Discovery document signature

The signature over `/.well-known/wia/legal-system-
harmonization` covers `@authority`, `@path`,
`content-digest` (RFC 9530), and `content-type`.

## Annex E — Cross-Origin Resource Sharing

Read endpoints serve `Access-Control-Allow-Origin: *`
with `ETag`, `Link`, and `X-WIA-Federation-Path`
exposed. Write endpoints require preflight.

## Annex F — Trust anchor rotation

Authority signing keys rotate per the authority's
own policy; the registry verifies at the JWKS URL
declared in the authority record. Recommended
cadence is 12 months for high-volume authorities and
36 months otherwise.

## Annex G — Connection management

HTTP/2 connection coalescing applies to the registry
sub-domains served by the same certificate. Atom
feed clients use long-poll or WebSub push to receive
deltas without polling.

## Annex H — Apostille e-register interop

When an item carries `apostilleRef`, the registry
exposes the Hague Apostille e-register binding via
the `/v1/items/{itemRef}/apostille` endpoint per
PHASE-2 Annex J.

## Annex I — Multilingual content negotiation

Expressions carry one record per language per the
work's official languages. Content negotiation
follows RFC 4647 lookup; clients receive a 300
Multiple Choices response when more than one
language version applies.

## Annex J — Citation graph traversal limits

Citation walks are bounded at 1024 hops by default
to prevent loop-storm scenarios. The cap is
configurable per registry deployment.

## Annex K — XAdES legacy interop

National publication systems that use XAdES
(W3C XML Advanced Electronic Signatures) wrap
publications in XAdES envelopes; the registry
unwraps XAdES and re-issues a JWS for downstream
consumption. The XAdES envelope is preserved in the
manifestation record's `signatureRef` for archival
purposes.

## Annex L — WebSub push delivery

Atom feeds publish a `<link rel="hub">` pointing to
the registry's WebSub hub. Subscribers register at
the hub with a callback URL and a verification token.
The hub pushes new entries to subscribers within 60
seconds of publication.

## Annex M — Connection compression

Akoma Ntoso XML documents are large; the registry
declares `Accept-Encoding: br, zstd, gzip` on its
read endpoints. Brotli is preferred for HTTP/2
clients; gzip is the universal fallback.

## Annex N — Authority key proof-of-possession

Authority signing keys MAY be hardware-bound (HSM
attestation, FIDO2 with attestation). When
hardware-bound, the registry records the
attestation in the authority record so that
downstream consumers can verify the binding
independently.

## Annex O — Caching rules for ephemeral consolidations

Consolidated text generated for a date that is not
a published consolidation cut-off is cached for at
most 24 hours and tagged with `Vary: Accept-Date`.
This prevents stale ephemeral consolidations from
serving as authoritative.

## Annex P — Resolution latency budget

ELI / ECLI resolution targets sub-200 ms p95
latency. Registries that operate read replicas
publish per-region latency in
`/v1/registry/health/regions`. Requests that exceed
the budget are surfaced as `slow-resolution` events
in the audit feed.

## Annex Q — Citation back-fill propagation

When a registry mirrors EUR-Lex or another upstream
publication, it back-fills citation records derived
from the upstream artefact. Back-filled citations
carry a `derived: true` flag so that downstream
consumers can distinguish between authoritative
citations published by the source authority and
derivative citations introduced by the mirror.

## Annex Q1 — Federation peer trust matrix

The discovery document carries a peer trust matrix
listing each peer's accepted operation groups and
the JWKS URL for verification. Trust matrix changes
are surfaced as audit events so that downstream
clients can adjust their resolution paths.

## Annex R — Treaty depositary message format

Treaty depositaries exchange ratification
notifications as signed JSON-LD documents conforming
to a registry-published schema. Each message carries
the depositary's signature, the ratifying party's
signature, and any reservation or declaration
references. Messages are persisted in the registry
for at least the treaty's effective lifespan plus
50 years.

弘益人間 (Hongik Ingan) — Benefit All Humanity
