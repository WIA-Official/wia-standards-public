# WIA-data-visualization PHASE 2 — API Interface Specification

**Standard:** WIA-data-visualization
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that
a data-visualization operator (BI platform
vendor, open-data dashboard publisher, news-
media operator, scientific-visualization
operator, inclusive-visualization provider,
industrial-presentation operator, SVG-rendering
library publisher, certification body) exposes
for the records defined in PHASE-1. The
contract carries the visualization-specification
registration, accessibility-profile attestation,
colour-space binding, display-calibration
recording, provenance recording, and chain-of-
custody anchoring endpoints.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110, RFC 9111, RFC 9457, RFC 8288,
  RFC 6901 / 6902, RFC 8259, RFC 4122, RFC
  9421, RFC 8615
- W3C Trace Context, W3C SVG 2, W3C SVG 1.1
  Second Edition, W3C WAI-ARIA 1.2, W3C WCAG
  2.2, W3C HTML Living Standard, W3C CSS
  Color Module Level 4, W3C JSON-LD 1.1, W3C
  SHACL, W3C Verifiable Credentials Data
  Model v2.0
- ISO 9241-303:2011, ISO 9241-307:2008, ISO
  9241-306:2018, ISO 9241-110:2020, ISO 9241-
  210:2019, ISO 14915 series, IEC 61506:1997
- ICC.1:2010, ICC.2:2019, CIE 15:2018
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015,
  ISO/IEC 17065:2012
- EN 301 549 v3.2.1, US Section 508, KR 한국
  형 웹 콘텐츠 접근성 지침

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published
by the operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical. Schema changes
follow the non-breaking conventions in PHASE-1
§2. Every endpoint carries a per-request
signature using HTTP Message Signatures (RFC
9421) anchored to the operator's accreditation
reference (the ISO/IEC 27001 certification, the
per-jurisdiction accessibility-conformance
reference); the signature key set is published
at
`/.well-known/wia/data-visualization/keys.json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-data-visualization",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":          "/v1/programmes",
    "vizSpecs":            "/v1/viz-specs",
    "accessibilityProfiles": "/v1/accessibility-profiles",
    "colourSpaces":        "/v1/colour-spaces",
    "displayCalibrations": "/v1/display-calibrations",
    "provenanceRecords":   "/v1/provenance-records",
    "custody":             "/v1/custody-events",
    "openapi":             "/v1/openapi.json",
    "wellKnown":           "/.well-known/wia/data-visualization"
  }
}
```

## §3 Visualization-Specification Endpoints

### §3.1 Register a visualization specification

```
POST /v1/viz-specs
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §3 record from
PHASE-1 (vizKind, vegaLiteSpecRef, svgRef,
encodingChannels, dataSourceRef). The server
validates the per-channel encoding declarations
against the declared `vizKind`: a `bar-chart`
declared with a `position-x` and `position-y`
encoding is accepted; a `bar-chart` declared
without `position-x` or `position-y` is
rejected with `422 Unprocessable Entity` at
`/problems/encoding-channel-vizkind-mismatch`.

### §3.2 Retrieve a visualization specification

```
GET /v1/viz-specs/{vizSpecId}
Accept: application/json
```

### §3.3 Render a visualization specification

```
GET /v1/viz-specs/{vizSpecId}/render
Accept: image/svg+xml | image/png | application/pdf
```

The endpoint renders the per-spec visualization
in the requested format. SVG is the canonical
format (per W3C SVG 2); PNG and PDF are
secondary rasterisation formats with the
operator-declared resolution envelope.

## §4 Accessibility-Profile Endpoints

### §4.1 Attest the accessibility profile

```
POST /v1/accessibility-profiles
Content-Type: application/json
Signature: <RFC 9421 signature from the
            inclusive-content team>
```

Request body carries the §4 record from
PHASE-1. The server validates that the per-
chart `wcagConformance` declaration is
consistent with the per-chart attestation
evidence: a `WCAG-2.2-AA` declaration MUST
be backed by per-chart `altText`,
`longDescription`, and `dataTableFallback`,
and the per-chart `contrastRatio` MUST meet
or exceed the WCAG 2.2 SC 1.4.3 thresholds.

### §4.2 Retrieve an accessibility profile

```
GET /v1/accessibility-profiles/{accessibilityProfileId}
Accept: application/json
```

## §5 Colour-Space Endpoints

### §5.1 Bind a colour space

```
POST /v1/colour-spaces
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §5 record from
PHASE-1 (iccProfileRef, workingSpace,
colorimetricIntent, cieLABReference). The
server validates the `colorimetricIntent`
against the ICC.1:2010 enumeration.

### §5.2 Retrieve a colour-space record

```
GET /v1/colour-spaces/{colourSpaceId}
Accept: application/json
```

## §6 Display-Calibration Endpoints

### §6.1 Record a display calibration

```
POST /v1/display-calibrations
Content-Type: application/json
Signature: <RFC 9421 signature from a per-
            jurisdiction calibration
            laboratory>
```

Request body carries the §6 record from
PHASE-1. The server validates the
`measurementMethod` against the ISO 9241-307
enumeration.

### §6.2 Retrieve a display-calibration record

```
GET /v1/display-calibrations/{displayCalibrationId}
Accept: application/json
```

## §7 Provenance Endpoints

### §7.1 Record provenance

```
POST /v1/provenance-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §7 record from
PHASE-1. The server validates that the per-
chart `dataProvenance` references at least
one upstream data source per the operator's
data-provenance discipline.

### §7.2 Retrieve provenance

```
GET /v1/provenance-records/{provenanceId}
Accept: application/json
```

## §8 Custody and Error Reporting

### §8.1 Anchor a custody event

```
POST /v1/custody-events
Content-Type: application/json
Signature: <RFC 9421 signature>
```

### §8.2 Error envelope

Errors are returned using RFC 9457 Problem
Details. Validation errors carry a `pointer`
(RFC 6901). The server emits a per-request
`traceparent` header (W3C Trace Context).

## §9 Concurrency and Cache

Every retrieval endpoint emits an `ETag`
header (RFC 9110 §8.8.3). Conditional
requests are honoured.

## §10 Bulk Export for Catalogue Consumers

```
GET /v1/viz-specs:bulk
Accept: application/x-ndjson
```

A catalogue consumer (a corporate-data-
catalogue platform, an open-data portal)
ingests the operator's visualization-spec
register as a newline-delimited JSON stream.

## §11 Per-Chart Embed Endpoint

```
GET /v1/viz-specs/{vizSpecId}/embed
Accept: text/html
```

The endpoint returns the per-chart embeddable
HTML envelope per W3C HTML Living Standard.
The envelope carries the per-chart accessibility-
profile binding so that the embedded chart
remains accessible in the host page.

## §12 Per-Chart Static Snapshot Endpoint

```
GET /v1/viz-specs/{vizSpecId}/snapshot
?at={iso8601}
Accept: image/svg+xml | image/png
```

A consumer requesting a per-chart static
snapshot at a declared timestamp receives the
per-chart frozen rendering with the per-source
data as it was at the declared timestamp. The
snapshot is signed by the operator's signing-
key set so that a downstream archival system
can verify the snapshot envelope.

## §13 Webhook Endpoint for Spec-Change Notifications

```
POST /v1/programmes/{programmeId}/webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A downstream consumer (a host page embedding
the per-chart, an archival system retaining
the per-chart, an audit observer) registers
a webhook to receive a push notification when
a per-chart specification is republished, when
a per-chart accessibility profile is re-
attested, or when a per-chart provenance
record is updated.

## §14 Authorities and Roles

| Role                      | Capabilities |
|---------------------------|------|
| `viz-author`              | Register visualization specification, render |
| `inclusive-content-team`  | Attest accessibility profile |
| `colour-management-engineer` | Bind colour space |
| `display-calibration-laboratory` | Record display calibration |
| `editorial-quality-team`  | Record provenance, attest authorship |
| `audit-observer`          | Read full operator state under engagement |
| `regulator`               | Read full operator state under accessibility mandate |

The operator's API enforces the per-role
authorisation policy.

## §15 Schema-Validation and Conformance

The OpenAPI 3.1 document at
`/v1/openapi.json` carries JSON Schema 2020-12
schemas for every request and response
envelope.

```
GET /v1/conformance/test-vectors
Accept: application/json
```

The operator publishes the conformance test
vectors used to qualify the API implementation
against this specification.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §16 Per-Chart Idempotency-Key Discipline

```
POST /v1/viz-specs
Idempotency-Key: <UUID>
```

Every per-chart registration carries an
`Idempotency-Key` header so that a per-chart
re-registration arising from a network retry
does not produce a duplicate per-chart record.
The operator's API records the per-key envelope
for 24 hours; a re-registration with the same
key returns the original per-chart record.

## §17 Multi-Language Per-Chart Surface

```
GET /v1/viz-specs/{vizSpecId}?lang={lang}
Accept: application/json
```

The per-chart text envelope is published in
each operator-declared language (English,
Korean, Japanese, German, French, Spanish,
Arabic, Chinese) with the per-chart axis
labels, the per-chart legend entries, the per-
chart annotations, the per-chart `altText`,
and the per-chart `longDescription` translated.
The per-chart encoding declarations and the
per-chart numeric values are language-neutral.

## §18 Per-Chart Conformance-Test Endpoint

```
POST /v1/viz-specs/{vizSpecId}/conformance-test
Content-Type: application/json
Signature: <RFC 9421 signature from a per-
            jurisdiction conformance-test
            authority>
```

A per-jurisdiction conformance-test authority
runs the per-chart accessibility-conformance
test against the operator's API. The test
envelope carries the per-chart axe-core /
WAVE / Pa11y test-tool reference, the per-
chart ARC Toolkit reference, the per-chart
KR-WAH (Korean Web Accessibility Hub)
evaluation reference, or an equivalent per-
jurisdiction-recognised test-tool reference.
The test outcome is recorded against the per-
chart accessibility-profile envelope.
