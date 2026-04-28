# WIA-legal-system-harmonization PHASE 2 — API Interface Specification

**Standard:** WIA-legal-system-harmonization
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surfaces that WIA-legal-
system-harmonization participants expose so that
official-publishing authorities, courts, regulatory
agencies, treaty depositaries, citizen-information
services, legal-research providers, and audit
authorities can publish and consume legal artefacts,
resolve cross-jurisdiction citations, and reconcile
harmonisation maps through a single contract.

References (CITATION-POLICY ALLOW only):
- OpenAPI Specification 3.1
- IETF RFC 9110 (HTTP Semantics), RFC 9457 (Problem Details)
- IETF RFC 7515 (JWS), RFC 7519 (JWT), RFC 9421 (HTTP Message Signatures)
- IETF RFC 5005 (Atom paged feeds), RFC 9530 (Digest Fields)
- OASIS LegalDocML (Akoma Ntoso 1.0), OASIS LegalRuleML 1.0
- ELI / ECLI Council Conclusions
- W3C JSON-LD 1.1, W3C SHACL
- ISO 17442:2020 (LEI)

---

## §1 Scope

This PHASE specifies the HTTP-based interfaces between
publishing authorities, repositories, citizen-
information services, courts, treaty depositaries,
legal-research providers, harmonisation-map
publishers, and audit authorities.

## §2 Operation groups

| Prefix              | Group                                           |
|---------------------|-------------------------------------------------|
| `/v1/works`         | work registry                                    |
| `/v1/expressions`   | expression registry, consolidated text           |
| `/v1/manifestations`| manifestation registry (binary artefacts)        |
| `/v1/items`         | item registry (repository copies)                |
| `/v1/authorities`   | authority register                               |
| `/v1/citations`     | citation graph                                   |
| `/v1/treaties`      | treaty registry and ratifications                |
| `/v1/harmonisation` | harmonisation maps                               |
| `/v1/registry`      | registry directory                               |

## §3 Authentication

Read endpoints are public. Write endpoints require a
JWT bearer (RFC 7519) bound to an authority record
with the corresponding LEI. Treaty depositary
endpoints accept signatures from depositary
identities only.

## §4 Work / expression operations

### 4.1 Publish work

```
POST /v1/works
```

Body: work record (PHASE-1 §3) signed by the
enacting authority. The registry verifies the
signature against the authority's JWKS.

### 4.2 Publish expression

```
POST /v1/expressions
```

Body: expression record (PHASE-1 §4) plus the
Akoma Ntoso XML. The registry verifies the document
against Akoma Ntoso 1.0 schema.

### 4.3 Lookup expression

```
GET /v1/expressions/{expressionRef}
```

Returns the canonical expression record. Conditional
GET via `ETag` is honoured.

### 4.4 Consolidated text

```
GET /v1/expressions/{expressionRef}/consolidated?at=<ISO8601>
```

Returns the consolidated text effective at the given
date. The response carries the list of expressions
consolidated and their effective dates.

## §5 Manifestation / item operations

### 5.1 Publish manifestation

```
POST /v1/manifestations
```

Body: manifestation record (PHASE-1 §5) plus the
binary artefact (PDF, EPUB, HTML, RTF, AKN).

### 5.2 Lookup item

```
GET /v1/items/{itemRef}
```

Returns the canonical item record and the URL of the
holding repository.

## §6 Authority operations

### 6.1 Register authority

```
POST /v1/authorities
```

Body: authority record (PHASE-1 §7) including LEI
attestation.

### 6.2 Lookup

```
GET /v1/authorities?lei={lei}
```

Returns the authority record. The LEI lookup is
delegated to GLEIF where the authority does not
self-publish.

## §7 Citation graph

### 7.1 Resolve citation

```
GET /v1/citations?source={uri}&target={uri}
```

Returns the citation record(s) describing the
relationship.

### 7.2 Walk graph

```
POST /v1/citations/walk
```

Body: a starting URI and a relation filter (`amends`,
`repeals`, `transposes`). Response: an ordered list
of citations forming the walk.

## §8 Treaty operations

### 8.1 Publish treaty

```
POST /v1/treaties
```

Body: treaty record (PHASE-1 §9) signed by the
treaty depositary.

### 8.2 Append ratification

```
POST /v1/treaties/{treatyRef}/ratifications
```

Body: a ratification record (party LEI, date, scope,
reservations, declarations) signed by the ratifying
state's authority.

## §9 Harmonisation operations

### 9.1 Publish map

```
POST /v1/harmonisation
```

Body: harmonisation map record (PHASE-1 §10).

### 9.2 Resolve mapping

```
GET /v1/harmonisation?src={uri}&tgt={uri}
```

Returns mappings between the source and target
expressions. Mappings carry the `mappingKind`,
provenance, and effective date.

## §10 Error semantics

Errors are `application/problem+json` (RFC 9457)
namespaced under
`https://wiastandards.com/errors/legal-system-harmonization/`.

## §11 Caching and rate limits

Manifestation and item endpoints serve immutable
content with `Cache-Control: public, max-age=31536000,
immutable`. Expression and citation endpoints serve
mutable content with strong `ETag` and short max-age.
Rate-limit headers follow the draft-ietf-httpapi-
ratelimit-headers convention.

## Annex A — OpenAPI 3.1 fragment

```yaml
openapi: 3.1.0
info: {title: WIA-legal-system-harmonization API, version: 1.0.0}
paths:
  /v1/expressions:
    post:
      summary: Publish an expression
      requestBody:
        required: true
        content:
          application/json:
            schema: {$ref: 'ExpressionRecord.schema.json'}
      responses:
        '201': {description: Expression published}
```

## Annex B — Idempotency

Mutating operations honour `Idempotency-Key`. Results
are persisted for 24h.

## Annex C — Webhook subscriptions

Subscribers receive events on `expression.published`,
`expression.consolidated`, `treaty.ratified`,
`citation.added`, `harmonisation.published`. Delivery
is signed with HMAC-SHA-256.

## Annex D — Atom paged feeds

The registry exposes Atom feeds (RFC 5005) for each
operation group so that legal-research providers can
ingest deltas without polling individual resources.

## Annex E — Bulk export

`POST /v1/registry/export` returns a signed URL to a
`tar.zst` of the deployment's records filtered by
jurisdiction, date range, and act type.

## Annex F — Sandbox endpoints

`/v1/sandbox` mirrors the production surface with
synthetic jurisdictions and ephemeral state.

## Annex G — Quotas

Per-authority publish quotas default to 1,000
expressions per hour and 10,000 per day. The registry
surfaces remaining quota in `X-Quota-Remaining`.

## Annex H — Webhook payload shape

```json
{
  "event": "expression.consolidated",
  "expressionRef": "/eli/eu/dir/2016/679/oj/2026-04-28",
  "consolidatedFrom": ["/eli/eu/dir/2016/679/oj/2016-04-27"],
  "effectiveAt": "2026-04-28"
}
```

## Annex I — Audit feed

`GET /v1/registry/audit?since=<timestamp>` returns
mutating-operation events. JWT scope
`audit-feed:read` required.

## Annex J — Apostille e-register interop

```
GET /v1/items/{itemRef}/apostille
```

Returns the Hague Apostille e-register record bound
to the item, when the publishing authority is a
Hague Convention apostille issuer.

## Annex K — Public introspection

`GET /v1/registry/stats` returns aggregate counters
(authority count, expression count, citation count,
treaty count). Counters are eventually consistent.

## Annex L — Concept scheme management

```
POST /v1/registry/concept-schemes
GET  /v1/registry/concept-schemes/{schemeId}
```

Submits and retrieves SKOS concept schemes used as
subject matter taxonomies. Schemes are versioned
with Semantic Versioning 2.0.0; major bumps require
a new scheme identifier with the prior scheme
preserved for citation continuity.

## Annex M — Citation graph diff

```
POST /v1/citations/diff
```

Body: two timestamps. Response: the citation graph
delta between the two timestamps (added, removed,
re-targeted citations). Auditors use the diff to
verify graph consistency against authoritative
publication events.

## Annex N — Treaty signature ceremony

```
POST /v1/treaties/{treatyRef}/signature-ceremony
```

Body: signing authority LEI, ceremony location,
witness LEIs. Response: ceremony record with
attached signed protocol. Used for high-profile
treaty signings where multiple state parties append
signatures concurrently.

## Annex O — Researcher subscription

```
POST /v1/registry/subscriptions
```

Body: a SHACL filter over the artefact graph (e.g.
"all GDPR-related expressions across EU member
states"). Response: a webhook endpoint that
receives matching events.

## Annex P — Bulk consolidation

```
POST /v1/expressions/bulk-consolidate
```

Body: a list of `expressionRef` and an effective
date. Response: a single consolidated expression
record bound to all inputs and signed by the
publishing authority.

## Annex Q — Document compare

```
POST /v1/expressions/compare
```

Body: two `expressionRef`. Response: a structural
diff at the Akoma Ntoso eId level showing additions,
deletions, and reordering between the two
expressions.

## Annex R0 — Court hierarchy endpoint

```
GET /v1/authorities/{authorityRef}/courts
```

Returns the court hierarchy beneath the authority.
Used by citation walkers to resolve appellate
relationships.

## Annex R1 — Reservation list endpoint

```
GET /v1/treaties/{treatyRef}/reservations
```

Returns the per-party reservation and declaration
records. Pagination is mandatory beyond 100 records.

## Annex R — Harmonisation map review

```
POST /v1/harmonisation/{harmonisationRef}/review
```

Body: a reviewer LEI and a verdict (`endorsed`,
`disputed`, `superseded`). Reviews are signed by the
reviewer and accumulate as a publication trail; the
registry surfaces the latest endorsement state on
the map record.

## Annex S — Provenance chain endpoint

```
GET /v1/citations/{citationRef}/provenance
```

Returns the per-authority provenance chain for the
citation: publisher LEI, signature, timestamp, and
any subsequent amendments. The response is paginated
when the chain exceeds 100 hops.

## Annex T — Long-term archival hand-off

```
POST /v1/manifestations/{manifestationRef}/archive
```

Body: archival authority LEI and target archive
identifier (e.g. LOCKSS network, Portico repository,
sovereign national archives). Response: archival
event record with the archive's confirmation
signature.

## Annex U — ELI / ECLI registration

```
POST /v1/registry/eli
POST /v1/registry/ecli
```

Submits a new ELI or ECLI binding to the registry.
Bindings are reviewed by the publishing authority
before activation; the binding becomes resolvable
once the authority signs the activation event.

## Annex V — Multi-language consolidation

When a work is consolidated across multiple language
expressions concurrently, the publisher submits the
consolidation as a multi-part request:

```
POST /v1/expressions/multi-consolidate
Content-Type: multipart/form-data
```

Body: one part per language, each containing the
language tag and the Akoma Ntoso XML. Response: a
list of `expressionRef`, one per language.

弘益人間 (Hongik Ingan) — Benefit All Humanity
