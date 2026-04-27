# WIA-i18n PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-i18n
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
i18n / l10n programme exposes for the records defined in PHASE-1.
Consumers include software-publisher build pipelines, language-
service-vendor (LSV) management systems, individual translators
and reviewers, terminology managers, and quality-assurance services.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 / 9111 / 9112 / 9457 (HTTP Semantics, Caching,
  HTTP/1.1, Problem Details)
- IETF RFC 6901 / 6902 / 5789 (JSON Pointer / Patch / PATCH method)
- IETF RFC 8259 / 8288 / 9421 (JSON, Linking, Message Signatures)
- IETF RFC 5646 / BCP 47 (language tags)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
programme. Versioning uses `/v1/` path segments and Semantic
Versioning 2.0.0. The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-i18n",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "projects":      "/v1/projects",
    "localeData":    "/v1/locale-data",
    "segments":      "/v1/segments",
    "translations":  "/v1/translations",
    "glossary":      "/v1/glossary",
    "tm":            "/v1/tm",
    "qualityReviews":"/v1/quality-reviews",
    "buildBundles":  "/v1/build-bundles",
    "evidence":      "/v1/evidence",
    "openapi":       "/v1/openapi.json"
  }
}
```

## §3 Projects and Locale Data

```
POST   /v1/projects                     — register a project
GET    /v1/projects/{pid}               — retrieve project record
PATCH  /v1/projects/{pid}/target-locales — adjust target locale set
POST   /v1/locale-data                  — register a locale-data
                                           pinning (CLDR revision)
GET    /v1/locale-data?tag={t}          — retrieve current pinning
```

Submissions whose target locale tag does not parse under BCP 47
return `422` with type `urn:wia:i18n:locale-tag-invalid`.

## §4 Source Segments

```
POST   /v1/projects/{pid}/segments       — register a source segment
GET    /v1/segments/{sid}                — retrieve source segment
PATCH  /v1/segments/{sid}/status         — advance segment status
POST   /v1/bulk/segments                 — batched source extraction
```

Segments are immutable beyond `source-extracted`; subsequent
`PATCH` attempts that mutate `sourceText` return `409 Conflict` with
type `urn:wia:i18n:source-immutable`.

## §5 Translations

```
POST   /v1/segments/{sid}/translations   — submit a translation
GET    /v1/translations/{tid}            — retrieve translation
PATCH  /v1/translations/{tid}/status     — advance status
PATCH  /v1/translations/{tid}/approval   — append approval
```

Translations carrying machine-translation or fuzzy-match origin
include the `matchScore` field; submissions that omit it for
non-`human-translator` authorKinds return `422` with type
`urn:wia:i18n:match-score-missing`.

## §6 Glossary and Terminology

```
POST   /v1/projects/{pid}/glossary       — register a term
GET    /v1/glossary/{tid}                — retrieve term
PATCH  /v1/glossary/{tid}/forbidden      — extend forbidden list
GET    /v1/projects/{pid}/glossary?domain={d}
                                          — list terms by domain
```

Glossary submissions that introduce a `forbiddenTranslation`
matching an existing approved translation return `409 Conflict`
with type `urn:wia:i18n:glossary-conflict`; the operating
programme resolves the conflict before accepting the new entry.

## §7 Translation Memory

```
POST   /v1/projects/{pid}/tm-entries     — register a TM entry
GET    /v1/tm/{eid}                      — retrieve TM entry
GET    /v1/projects/{pid}/tm-search?source={text}&context={hash}
                                          — query for matches
```

TM search returns ICE matches (context hash matches), 100% matches,
and fuzzy matches with their `matchScore`. Programmes pin a default
fuzzy-match floor (typically 75) below which matches are not
returned for leverage.

## §8 Quality Reviews and Build Bundles

```
POST   /v1/translations/{tid}/quality-reviews   — register a review
GET    /v1/quality-reviews/{rid}                — retrieve review
POST   /v1/projects/{pid}/build-bundles         — register a build
                                                  bundle
GET    /v1/build-bundles/{bid}                  — retrieve bundle
GET    /v1/build-bundles/{bid}/artefact         — fetch built bundle
```

Build bundles are immutable; corrections emit a new bundle and the
prior bundle remains addressable for retrospective audit.

## §9 Evidence Package

```
POST   /v1/projects/{pid}/evidence       — request package generation
GET    /v1/evidence/{packageId}          — retrieve a package
GET    /v1/evidence/{packageId}/manifest — manifest only
```

The evidence-package format is governed by PHASE-4 §3 and contains
project, locale-data, segments, translations, glossary, TM
summaries, quality reviews, build bundles, and the signed manifest.

## §10 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:i18n:locale-tag-invalid`
- `urn:wia:i18n:source-immutable`
- `urn:wia:i18n:match-score-missing`
- `urn:wia:i18n:glossary-conflict`
- `urn:wia:i18n:bundle-build-failure`
- `urn:wia:i18n:evidence-mismatch`

## §11 Authentication

The API uses mutually-authenticated TLS for LSV, individual
translator (via vendor proxy), and build-pipeline connections.
Public read-only endpoints (released build bundles, the OpenAPI
document) are reachable without a client certificate.

## §12 Caching

Stable resources (released build bundles, signed evidence packages)
are cacheable with `Cache-Control: max-age=31536000, immutable`.
Mutable resources are cacheable for 60 seconds.

## §13 Streaming and Bulk

Continuous-localisation pipelines subscribe to project events via
Server-Sent Events at `/v1/projects/{pid}/events`. Topics include
new source segments, translation status changes, glossary updates,
and build-bundle releases. Bulk endpoints accept arrays for source
extraction, TM entries, and translation submissions.

## §14 Worked Example: From Source Extraction to Build

1. Build pipeline POSTs new source segments at extract time.
2. LSV consumes ready-for-translation segments via the streaming
   endpoint and dispatches to translators.
3. Translators submit translations; reviewers register quality
   reviews; the operating programme appends approvals.
4. Build pipeline retrieves approved translations and registers a
   build bundle.
5. Citation tool requests an evidence package for the release
   pinning the manifest digest as the citation anchor.

## §15 Pseudo-Locale and Selection-Catalogue Endpoints

```
POST   /v1/projects/{pid}/pseudo-locales       — register pseudo-
                                                  locale config
GET    /v1/pseudo-locales/{plid}               — retrieve config
POST   /v1/segments/{sid}/selection-catalogues — register selection
                                                  catalogue
GET    /v1/selection-catalogues/{cid}          — retrieve catalogue
```

Pseudo-locale records cannot be referenced as a `targetLocale` in
build-bundle submissions; attempts return `409 Conflict` with
type `urn:wia:i18n:pseudo-locale-not-shippable`.

## §16 Pagination and Audit

List endpoints use cursor-based pagination via the `cursor` query
parameter and `Link` headers (RFC 8288). Audit logs carry
`projectId`, `traceId`, the issuing client certificate's subject,
and the LSV / build-pipeline clock skew vs the reference NTP source.

## §17 Privacy-Preserving Aggregation

Aggregate consumers (research collaboratives, public-policy
analysts) fetch population-level statistics through endpoints that
emit counts, means, and dispersions (e.g. average review-cycle
latency by language pair, glossary-deviation rates by domain).

```
GET    /v1/aggregate/review-latency?language-pair=...&period=...
GET    /v1/aggregate/glossary-deviation?domain=...&period=...
```

Out-of-policy queries return `403 Forbidden` with type
`urn:wia:i18n:cohort-too-small`.

## §18 Dispatch Endpoints

```
POST   /v1/projects/{pid}/dispatches      — register a dispatch
GET    /v1/dispatches/{did}               — retrieve dispatch
PATCH  /v1/dispatches/{did}/reassign      — record reassignment
GET    /v1/projects/{pid}/dispatches?vendor={v}&from={t}
                                          — list by vendor / window
```

Dispatch endpoints surface project-level dispatch state to the
operating programme manager; LSV-internal dispatch detail (which
individual translator received which batch) flows through the
vendor's CAT-tool integration and is never exposed externally.

## §19 Provenance Endpoint

```
GET    /v1/provenance/{recordId}    — retrieve the provenance entry
                                       for any PHASE-1 record
```

Provenance entries trace a build bundle to its parents (project,
locale-data pinning, source segments, approved translations,
glossary state) so that auditors can walk the chain end-to-end.

## §20 Format Adapter Endpoints

```
POST   /v1/projects/{pid}/format-adapters    — register an adapter
GET    /v1/format-adapters/{aid}             — retrieve adapter
PATCH  /v1/format-adapters/{aid}/rules       — update rules
```

Adapter rules are content-addressed; updates emit new content-
addresses. Build pipelines pin a particular adapter rule revision
in their integration so that re-extraction at later builds is
reproducible.

## §21 Streaming Subscription Heartbeat and Replays

SSE subscriptions to project events (PHASE-2 §13) emit a heartbeat
every 30 seconds. Reconnections support replay via the
`Last-Event-ID` header (W3C EventSource semantics) so that build
pipelines that disconnect during long releases do not lose
visibility of approval / glossary / build events.

## §22 FHIR-Adjacent Bridges (optional)

Programmes whose i18n content includes clinical-document
translation MAY expose a read-only FHIR R5 facade that translates
selected build-bundle metadata into FHIR `Composition` and
`Bundle` resources for clinical-document workflows. The facade is
read-only and is consumed by clinical-document software that
prefers FHIR-native discovery.

## §23 Embargo Endpoint

```
PATCH  /v1/build-bundles/{bid}/embargo     — schedule or release
                                              an embargo
GET    /v1/build-bundles/{bid}/embargo     — retrieve embargo
                                              schedule
```

Embargoed bundles return `403 Forbidden` with type
`urn:wia:i18n:embargo-active` to non-authorised clients before the
release time; the embargo endpoint enforces the release time
server-side. Authorised pre-release consumers (a press team, a
regulator counterpart) carry their authorisation in the access-
control list referenced from the embargo record.

## §24 Quality-Aggregate Endpoints

```
GET    /v1/aggregate/quality-by-vendor?language-pair=...&period=...
GET    /v1/aggregate/error-distribution?quality-model=...&period=...
```

Quality-aggregate endpoints return cohort-size-protected statistics
to operators that compare multiple LSVs across language pairs and
quality models. Out-of-policy queries (cohort below threshold,
language-pair scope outside the consumer's allowed set) return
`403 Forbidden` with type `urn:wia:i18n:cohort-too-small`.

## §25 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1
document, and signs evidence packages per RFC 9421.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-i18n
- **Last Updated:** 2026-04-27
