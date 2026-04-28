# WIA-interoperability-registry PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-interoperability-registry
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an
interoperability-registry operator exposes for the records
defined in PHASE-1. Consumers include data architects,
integration engineers, runtime adapter binders, cross-
registry harvesters, conformance reviewers, and the operator's
own analytics platform.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 / 6902 (JSON Pointer / Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- IETF RFC 6920 (Naming Things with Hashes)
- ISO 8601 (date and time)
- ISO/IEC 11179 (metadata registries)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context
- W3C SKOS / OWL 2 / SHACL

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operator.
Versioning uses `/v1/` path segments. The OpenAPI 3.1 document
at `/v1/openapi.json` is canonical.

Artefact bodies (schema files, OWL fragments, SKOS expansions)
are content-addressed; the API returns metadata and a
content-address that consumers fetch separately so that large
artefacts do not bloat metadata responses.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-interoperability-registry",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "registries":          "/v1/registries",
    "artefacts":           "/v1/artefacts",
    "dataElements":        "/v1/data-elements",
    "valueSets":           "/v1/value-sets",
    "messageSchemas":      "/v1/message-schemas",
    "mappingSets":         "/v1/mapping-sets",
    "ontologyFragments":   "/v1/ontology-fragments",
    "adapterManifests":    "/v1/adapter-manifests",
    "harvests":            "/v1/harvests",
    "evidence":            "/v1/evidence",
    "openapi":             "/v1/openapi.json"
  }
}
```

## §3 Registry Lifecycle

```
POST   /v1/registries                  — register a registry
GET    /v1/registries/{rid}            — retrieve registry
PATCH  /v1/registries/{rid}/status     — advance status
```

Status `frozen` reflects an operator decision to stop accepting
new artefacts (typically before a successor registry takes
over); existing artefacts remain addressable and consumable
under the frozen registry's identifier.

## §4 Artefact Lifecycle

```
POST   /v1/registries/{rid}/artefacts          — register
                                                  artefact
                                                  metadata
PATCH  /v1/artefacts/{aid}/registration-status — advance
                                                  registration
                                                  status per
                                                  ISO/IEC
                                                  11179-6
PATCH  /v1/artefacts/{aid}/superseded-by       — record
                                                  successor
GET    /v1/artefacts/{aid}                     — retrieve
                                                  metadata
GET    /v1/artefacts/{aid}/content             — fetch the
                                                  content-
                                                  addressed
                                                  body
```

Registration-status transitions that violate the ISO/IEC
11179-6 state machine return `422` with type
`urn:wia:interoperability-registry:invalid-status-transition`.

## §5 Per-Class Artefact Endpoints

The per-class endpoints are syntactic sugar over the
`/v1/artefacts` endpoints; they are provided so that consumers
that filter by class can discover artefacts more
ergonomically:

```
GET    /v1/data-elements?registry={rid}&status={s}
GET    /v1/value-sets?codeSystem={c}&status={s}
GET    /v1/message-schemas?encoding={e}&status={s}
GET    /v1/mapping-sets?source={u}&target={u}
GET    /v1/ontology-fragments?profile={p}
GET    /v1/adapter-manifests?adapterKind={k}
```

## §6 Cross-Registry Harvest

```
POST   /v1/registries/{rid}/harvests   — initiate a harvest
                                          from a remote
                                          registry
GET    /v1/harvests/{hid}              — retrieve harvest
                                          status and outcome
GET    /v1/harvests/{hid}/conflicts    — list per-artefact
                                          conflicts requiring
                                          operator review
PATCH  /v1/harvests/{hid}/conflicts/{cid}/resolution
                                       — record conflict
                                          resolution
```

Harvest conflict resolutions follow the protocol in PHASE-3 §6.

## §7 Search and Discovery

```
GET    /v1/search?q={text}&class={c}&status={s}
                                       — text search across
                                          artefact slugs and
                                          definitions
GET    /v1/data-elements/by-binding?valueSet={u}
                                       — list data elements
                                          that bind a given
                                          value set
GET    /v1/value-sets/by-code-system?codeSystem={c}
                                       — list value sets
                                          drawn from a given
                                          code system
GET    /v1/mapping-sets/by-pair?source={u}&target={u}
                                       — list mapping sets
                                          between a vocabulary
                                          pair
```

Search responses respect the registry's authorisation matrix
so that draft artefacts do not leak through search to
consumers without the appropriate scope.

## §8 Errors

All error responses are `application/problem+json` per RFC
9457. Defined types include:

- `urn:wia:interoperability-registry:invalid-status-transition`
- `urn:wia:interoperability-registry:naming-collision`
- `urn:wia:interoperability-registry:harvest-conflict`
- `urn:wia:interoperability-registry:dangling-reference`
- `urn:wia:interoperability-registry:content-digest-mismatch`
- `urn:wia:interoperability-registry:evidence-mismatch`

## §9 Authentication and Authorisation

Mutually-authenticated TLS for registrar, harvester, and
adapter-binder consumers. Public read-only endpoints (artefact
metadata for `standard` and `preferred-standard` artefacts,
public search) are reachable without a client certificate;
the artefact body is content-addressed so it is reachable
through the operator's content store under the same
authorisation rules that gate the metadata.

## §10 Caching and Concurrency

Stable resources (artefacts whose `registrationStatus` is
`standard` or `preferred-standard`, signed evidence packages)
are cacheable with `Cache-Control: max-age=31536000,
immutable`. Mutable resources (candidate or qualified
artefacts, in-flight harvests) are cacheable for 60 seconds.
ETags are mandatory on every PATCH endpoint.

## §11 Streaming Subscription

Consumers subscribe via Server-Sent Events at:

- `/v1/registries/{rid}/events` — registry-wide events
  (artefact-status changes, harvest completions, conflict
  notifications).
- `/v1/artefacts/{aid}/events` — artefact-scoped events
  (status transitions, supersession notices, dependency
  changes).

Subscribers reconnect via the `Last-Event-ID` header (W3C
EventSource semantics).

## §12 Bulk Operations

```
POST   /v1/bulk/artefacts          — batched artefact import
POST   /v1/bulk/value-set-expansions — batched expansion
                                       refresh
GET    /v1/bulk/{operationId}      — operation status
```

Cursor-based pagination uses the `cursor` query parameter and
`Link` headers (RFC 8288); cursors persist for at least 24
hours.

## §13 Audit and Observability

Every endpoint emits structured logs with `registryId`,
`artefactId` (where applicable), `traceId`, the issuing
client certificate's subject, and the operator's clock skew vs
the reference NTP source.

## §14 Worked Example: Element Registration to Adapter Binding

1. The data architect submits a candidate data-element record
   with definition, value-domain reference, and data type.
2. After internal review, the registrar advances the artefact
   to `qualified` and then to `standard`.
3. An integration engineer registers a message schema that
   binds the data element through `bindingDataElements`.
4. An adapter manifest is registered that consumes the
   message schema and the data element.
5. A consuming runtime fetches the adapter manifest at boot
   and binds its data flows against the registry's content-
   addressed identifiers.

## §15 Provenance and Aggregate Endpoints

```
GET    /v1/provenance/{recordId}    — retrieve provenance entry
                                       for any artefact
                                       (custodian, registrar
                                       approvals, supersession
                                       chain)
GET    /v1/aggregate/artefact-counts-by-status?registry={r}
GET    /v1/aggregate/refresh-rate-by-codesystem?period=...
GET    /v1/aggregate/conflict-volume-by-federation?period=...
```

Aggregate queries that request per-artefact attribution where
the operator's policy disallows it return `403 Forbidden`
with type `urn:wia:interoperability-registry:attribution-restricted`.

## §16 Federation-Conflict Adjudication Endpoint

```
POST   /v1/federations/{fid}/conflicts/{cid}/adjudication
                                       — submit an adjudication
                                          decision from the
                                          joint naming-authority
                                          committee
GET    /v1/federations/{fid}/conflicts/{cid}/adjudication
                                       — retrieve the decision
                                          and its rationale
```

Adjudication submissions require committee-level
authorisation; submissions from individual member registrars
return `403 Forbidden` with type
`urn:wia:interoperability-registry:committee-authorisation-required`.

## §17 Dependency-Graph Endpoint

```
GET    /v1/artefacts/{aid}/dependencies
                                       — list direct upstream
                                          and downstream
                                          artefacts
GET    /v1/artefacts/{aid}/dependencies/transitive
                                       — list transitive
                                          dependency closure
                                          (server-rendered, with
                                          configurable depth
                                          limit to protect
                                          response size)
GET    /v1/artefacts/orphans?registry={r}
                                       — list orphan artefacts
                                          (no inbound
                                          dependencies)
```

Transitive-closure responses larger than the operator's
declared depth limit return `200 OK` with a partial result
and a `Link: rel="next"` header that lets the consumer
continue traversal in chunks.

## §18 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an
OpenAPI 3.1 document, signs evidence packages per RFC 9421,
and rejects content submissions whose payload digest disagrees
with the artefact's `contentDigest`.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-interoperability-registry
- **Last Updated:** 2026-04-28
