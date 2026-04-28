# WIA-a11y-wiabooks PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-a11y-wiabooks
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that a wiabooks
accessibility programme exposes for the records defined in
PHASE-1. Consumers include accessibility metadata aggregators,
EPUB reading-system vendors, library and education-procurement
platforms, screen-reader vendors that ingest publisher
accessibility statements, and the operator's own audit and
analytics platforms.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 / 6902 (JSON Pointer / Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context
- W3C EPUB Accessibility 1.1
- Schema.org Accessibility properties

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
operator. Versioning uses `/v1/` path segments. The OpenAPI
3.1 document at `/v1/openapi.json` is canonical.

EPUB and DAISY artefact bytes are content-addressed; the
API returns metadata and content-addresses, with the
artefact body retrievable through the operator's content
store under the same authorisation scheme that gates the
metadata.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-a11y-wiabooks",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":               "/v1/programmes",
    "publications":             "/v1/publications",
    "accessibilityStatements":  "/v1/accessibility-statements",
    "alternativeFormats":       "/v1/alternative-formats",
    "mediaOverlays":            "/v1/media-overlays",
    "atCompatibilityTests":     "/v1/at-compatibility-tests",
    "readingPreferences":       "/v1/reading-preferences",
    "evidence":                 "/v1/evidence",
    "openapi":                  "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
POST   /v1/programmes              — register a programme
GET    /v1/programmes/{pid}        — retrieve programme
PATCH  /v1/programmes/{pid}/status — advance status
PATCH  /v1/programmes/{pid}/accessibility-framework
                                    — update the operator's
                                       accessibility-
                                       conformance framework
                                       enrolment
```

Programme submissions for jurisdictions with statutory
accessibility obligations (EAA-2025, ADA-Section-508,
KR-Disability-Discrimination-Act) require a valid
accessibility-framework enrolment; submissions without the
matching framework return `409` with type
`urn:wia:a11y-wiabooks:framework-mismatch-jurisdiction`.

## §4 Publication Lifecycle

```
POST   /v1/programmes/{pid}/publications     — register a
                                                publication
PATCH  /v1/publications/{pubid}/status       — advance status
PATCH  /v1/publications/{pubid}/primary-format
                                              — register the
                                                primary EPUB 3
                                                package
GET    /v1/publications/{pubid}              — retrieve
                                                publication
```

Publications cannot reach `ready-for-distribution` without
an attached accessibility statement (PHASE-1 §4) of
conformance profile `EPUB-A-1.1-A` or higher; submissions
without the statement return `422` with type
`urn:wia:a11y-wiabooks:accessibility-statement-required`.

## §5 Accessibility Statements

```
POST   /v1/publications/{pubid}/accessibility-statements
                                              — register a
                                                statement
PATCH  /v1/accessibility-statements/{sid}/conformance
                                              — update
                                                conformance
                                                profile
PATCH  /v1/accessibility-statements/{sid}/known-limitations
                                              — append known
                                                limitations
GET    /v1/accessibility-statements/{sid}    — retrieve
                                                statement
```

Statements claiming `EPUB-A-1.1-AA` or higher require a
`certifiedBy` reference to the operator's accessibility-
certification body; submissions without the reference
return `422` with type
`urn:wia:a11y-wiabooks:certification-body-required`.

## §6 Alternative Formats

```
POST   /v1/publications/{pubid}/alternative-formats
                                              — register an
                                                alternative
                                                format
GET    /v1/alternative-formats/{fid}         — retrieve
                                                format
GET    /v1/publications/{pubid}/alternative-formats?
       formatKind={k}                          — list formats
```

Refreshable-Braille format submissions require a Braille
code reference (PHASE-1 §5 `brailleCodeRef`) so that
downstream Braille devices can apply the correct code
table.

## §7 Media Overlays

```
POST   /v1/publications/{pubid}/media-overlays
                                              — register a
                                                media overlay
GET    /v1/media-overlays/{oid}              — retrieve
                                                overlay
                                                metadata
GET    /v1/media-overlays/{oid}/smil         — retrieve
                                                SMIL document
```

TTS-narrated overlays cite a per-passage SSML override
artefact when the publisher's PLS lexicon is insufficient
for proper-noun and term pronunciation; pure-human
narration omits both lexicon and SSML overrides.

## §8 Assistive-Technology Compatibility Tests

```
POST   /v1/publications/{pubid}/at-compatibility-tests
                                              — register a
                                                test result
PATCH  /v1/at-compatibility-tests/{tid}/defects-filed
                                              — append filed
                                                defect
                                                references
GET    /v1/at-compatibility-tests/{tid}      — retrieve test
GET    /v1/publications/{pubid}/at-compatibility-tests?
       at={t}&reading-system={s}               — query tests
```

Tests with outcome `incompatible` against a major reading
system or AT block the publication's distribution to
catalogues that filter on the corresponding
`accessibilitySupport` claim; the API surfaces the block
through the streaming subscription.

## §9 Reading Preferences

```
POST   /v1/publications/{pubid}/reading-preferences
                                              — register a
                                                reader
                                                preference
PATCH  /v1/reading-preferences/{rid}         — update
                                                preference
GET    /v1/reading-preferences/{rid}         — retrieve
                                                preference
```

Reading preferences carry only opaque reader tokens; the
operator's CRM mediates between the token and the reader's
clinical identity per the operator's data-protection
policy.

## §10 Errors

All error responses are `application/problem+json` per RFC
9457. Defined types include those above plus:

- `urn:wia:a11y-wiabooks:hazard-disclosure-incomplete`
- `urn:wia:a11y-wiabooks:braille-code-mismatch`
- `urn:wia:a11y-wiabooks:tts-lexicon-required`
- `urn:wia:a11y-wiabooks:evidence-mismatch`

## §11 Authentication

Mutually-authenticated TLS for catalogue-aggregator,
reading-system-vendor, AT-vendor, and regulator consumers.
Public read-only endpoints (per-publication accessibility
statement, alternative-format catalogue, AT compatibility
matrix) are reachable without a client certificate so that
end-users and assistive-technology software can discover
accessibility metadata without authentication.

## §12 Caching, Concurrency, Audit

Stable resources (released accessibility statements,
content-addressed alternative formats, signed evidence
packages) are cacheable with `Cache-Control: max-age=
31536000, immutable`. Mutable resources (in-flight tests,
preference updates) are cacheable for 60 seconds. ETags
are mandatory on every PATCH endpoint. Audit logs carry
`programmeId`, `publicationId`, `traceId`, the issuing
client certificate's subject, and the operator's clock
skew vs the operating jurisdiction's NTP service.

## §13 Streaming Subscription

Consumers subscribe via Server-Sent Events at:

- `/v1/programmes/{pid}/events` — programme-wide events
  (statement issuance, AT-compatibility regressions
  affecting multiple titles).
- `/v1/publications/{pubid}/events` — publication-scoped
  events (alternative-format readiness, statement
  revisions, AT-test outcomes).

Subscribers reconnect via the `Last-Event-ID` header (W3C
EventSource semantics).

## §14 Worked Example: Publication to AT-Verified Distribution

1. The publisher registers a publication and uploads the
   primary EPUB 3 package via `primaryFormatRef`.
2. The accessibility-certification body issues an
   `EPUB-A-1.1-AA` accessibility statement against the
   primary format with full a11yMode / a11yFeature /
   a11yHazard disclosure.
3. The publisher registers alternative formats: a DAISY
   navigable-audio for blind readers and a refreshable-
   Braille BRF / PEF for Braille readers.
4. The QA team registers AT compatibility tests against
   NVDA + JAWS + VoiceOver + TalkBack on the major reading
   systems; outcomes are `nominal` or `partial-functionality`.
5. The publication advances to `ready-for-distribution`
   and the catalogue aggregator ingests the accessibility
   statement and AT-compatibility matrix.

## §15 Bulk and Pagination

```
POST   /v1/bulk/publications              — batched
                                              publication import
                                              from publisher
                                              workflow
POST   /v1/bulk/at-compatibility-tests    — batched test
                                              ingest
POST   /v1/bulk/accessibility-statements  — batched statement
                                              ingest
GET    /v1/bulk/{operationId}             — operation status
```

Cursor-based pagination uses the `cursor` query parameter
and `Link` headers (RFC 8288); cursors persist for at
least 24 hours.

## §16 Provenance and Aggregation

```
GET    /v1/provenance/{recordId}    — provenance entry for
                                       any PHASE-1 record
GET    /v1/aggregate/conformance-distribution?period=...
GET    /v1/aggregate/at-compatibility-by-vendor?period=...
GET    /v1/aggregate/alternative-format-coverage?period=...
```

Aggregate consumers fetch population-level statistics; per-
publication attribution requires the operator's
authorisation.

## §17 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an
OpenAPI 3.1 document, signs evidence packages per RFC 9421,
and rejects publication-status advancement past
`ready-for-distribution` without an attached accessibility
statement.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-a11y-wiabooks
- **Last Updated:** 2026-04-28
