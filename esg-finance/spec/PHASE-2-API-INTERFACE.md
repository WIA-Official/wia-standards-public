# WIA-esg-finance PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-esg-finance
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that an
ESG-disclosure operator (corporate reporting entity,
financial-market participant under SFDR, asset
manager under UN PRI, credit institution under the
Net-Zero Banking Alliance, or assurance provider)
exposes for the records defined in PHASE-1. The
contract is consumed by the entity's board (audit
committee and sustainability committee), the
external assurance provider, the supervisory
authority's examination tooling, the public-disclosure
surface (XBRL ESEF / ESRS digital tagging), the
investor-facing reporting channel (CDP, GRI,
ratings-agency feeds), and the operator's compliance-
and-audit functions.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web Linking),
  RFC 8259 (JSON), RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022, ISO/IEC 42001:2023
- ISO 14064-1 + 14064-3
- ISO 17442 LEI
- W3C Trace Context, W3C Verifiable Credentials Data
  Model 2.0
- IFRS Foundation ISSB IFRS S1 / S2
- EU CSRD (2022/2464) + ESRS (2023/2772) + ESEF
  Regulation (Reg 2018/815) for digital tagging
- EU Taxonomy Regulation (2020/852) + Climate
  Delegated Act (2021/2139) + Disclosures Delegated
  Act (2021/2178)
- EU SFDR (2019/2088)
- GHG Protocol Corporate + Scope 3
- SASB Industry Standards
- UN PRI Reporting Framework

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by
the operator. Versioning uses `/v1/` path segments.
The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical for the JSON surface. The XBRL ESEF /
ESRS digital tagging schema published by the EU
Commission is canonical for the EU CSRD digital
report.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-esg-finance",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "materialityAssessments":  "/v1/materiality-assessments",
    "issbDisclosures":         "/v1/issb-disclosures",
    "esrsDisclosures":         "/v1/esrs-disclosures",
    "taxonomyAlignments":      "/v1/taxonomy-alignments",
    "sfdrProducts":            "/v1/sfdr-products",
    "ghgInventories":          "/v1/ghg-inventories",
    "sbtiTargets":             "/v1/sbti-targets",
    "csdddDueDiligence":       "/v1/csddd-due-diligence",
    "assurance":               "/v1/assurance",
    "publicDisclosure":        "/v1/public-disclosure",
    "examination":             "/v1/examination",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 Reporting-Period and Materiality Endpoints

```
GET    /v1/programmes
GET    /v1/programmes/{programmeId}
POST   /v1/programmes
GET    /v1/materiality-assessments?period={periodId}
POST   /v1/materiality-assessments
GET    /v1/materiality-assessments/{assessmentId}
```

## §4 ISSB / ESRS Disclosure Endpoints

```
GET    /v1/issb-disclosures?period={periodId}
POST   /v1/issb-disclosures
GET    /v1/issb-disclosures/{disclosureId}
GET    /v1/esrs-disclosures?period={periodId}
POST   /v1/esrs-disclosures
GET    /v1/esrs-disclosures/{disclosureId}
GET    /v1/esrs-disclosures/{disclosureId}/digital-tagged
       (XBRL ESEF / ESRS-tagged digital report)
```

The digital-tagged endpoint produces the ESEF + ESRS
2023/2772-compliant XBRL document the entity files
with the operating jurisdiction's officially
appointed mechanism for storage of regulated
information.

## §5 Taxonomy and SFDR Endpoints

```
GET    /v1/taxonomy-alignments?period={periodId}
POST   /v1/taxonomy-alignments
GET    /v1/taxonomy-alignments/{alignmentId}
GET    /v1/sfdr-products
POST   /v1/sfdr-products
PATCH  /v1/sfdr-products/{productId}    (re-
                                          classification
                                          where the
                                          underlying
                                          investment
                                          composition
                                          changes)
```

## §6 GHG Inventory and SBTi Endpoints

```
GET    /v1/ghg-inventories?period={periodId}
POST   /v1/ghg-inventories
GET    /v1/ghg-inventories/{inventoryId}
POST   /v1/ghg-inventories/{inventoryId}/verify
       (record verification by an ISO 14064-3
        accredited provider)
GET    /v1/sbti-targets
POST   /v1/sbti-targets
PATCH  /v1/sbti-targets/{targetId}/progress
```

## §7 CSDDD Due-Diligence Endpoints

```
GET    /v1/csddd-due-diligence?period={periodId}
POST   /v1/csddd-due-diligence
GET    /v1/csddd-due-diligence/{dueDiligenceId}
POST   /v1/csddd-due-diligence/{dueDiligenceId}/complaints
       (channel for complaints under CSDDD Article
        14)
```

## §8 Assurance Endpoints

```
GET    /v1/assurance?period={periodId}
POST   /v1/assurance
GET    /v1/assurance/{assuranceId}
```

## §9 Public-Disclosure Surface

```
GET    /v1/public-disclosure/me/annual-report?period={periodId}
GET    /v1/public-disclosure/me/sustainability-statement?period={periodId}
GET    /v1/public-disclosure/me/ghg-summary?period={periodId}
GET    /v1/public-disclosure/me/transition-plan
GET    /v1/public-disclosure/me/csddd-statement
```

The public-disclosure surface exposes the entity's
publicly filed disclosures with their digital-
tagging artefacts so investors, civil society, and
ratings agencies can access the canonical record.

## §10 Examination Surface

```
GET    /v1/examination/programmes
GET    /v1/examination/issb-disclosures
GET    /v1/examination/esrs-disclosures
GET    /v1/examination/taxonomy-alignments
GET    /v1/examination/ghg-inventories
GET    /v1/examination/csddd-due-diligence
GET    /v1/examination/audit-events
```

The examination scope is read-only and bound to
the supervisory authority's identity (ESMA + Member-
State NCA in EU; SEC for US-jurisdiction filers; KR
FSC + KCSE for KR-jurisdiction filers; CDP / GRI
ratings-agency consumers under separate contractual
scope).

## §11 Authentication, HTTP Status, and Caching

Bearer tokens conform to OAuth 2.1; per-surface
audiences distinguish entity-internal, public-
disclosure, examination, and assurance scopes.
Standard HTTP status codes apply. Public-disclosure
responses follow the operator's published cache-
control policy (typically `Cache-Control: public,
max-age=86400` for the static ESEF document and
`no-store` for the live materiality-assessment
working draft).

## §12 Webhook and Event Surface

The operator publishes lifecycle events through a
webhook channel:

- `materiality-assessment.completed`
- `disclosure.filed-with-supervisor`
- `assurance.issued`
- `taxonomy-alignment.published`
- `csddd-complaint.received`
- `sbti-progress.updated`
- `transition-plan.updated`

Webhook signatures use HTTP Message Signatures
(RFC 9421).

## §13 Bulk-Export Surface

```
POST   /v1/bulk-export
GET    /v1/bulk-export/{exportId}/status
GET    /v1/bulk-export/{exportId}/manifest
```

Bulk exports support investor-facing data calls
(CDP submission, ratings-agency periodic refresh,
UN PRI annual reporting). The export's manifest
declares the cryptographic digest of each NDJSON
file produced.

## §14 Rate Limiting and Examination-Burst Surface

Per-client rate limits are declared on the OpenAPI
document. Examination-scope endpoints carry a higher
burst budget so that the supervisory authority's
periodic data-call traffic is not throttled at the
boundary; the burst budget is documented in the
operator's compliance procedure and aligned with the
authority's published examination cadence.

## §15 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the public-disclosure
surface with the operating jurisdiction's digital-
tagging artefacts, expose the supervisory examination
surface, and propagate trace-context across the
materiality-to-assurance chain.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-esg-finance
- **Last Updated:** 2026-04-28
