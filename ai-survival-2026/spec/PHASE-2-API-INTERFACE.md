# WIA-ai-survival-2026 PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-ai-survival-2026
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that an
AI-survival operator exposes for the records defined
in PHASE-1. The contract is consumed by the
operator's internal AI governance committee, the
operator's red-team and safety function, the
supervisory authority's examination tooling (US AISI
+ EU AI Office + Member-State NCAs + KR AI 안전
연구소), the international coordination bodies, the
operator's workforce-transition stakeholders, and
the public-disclosure surface for transparency
reports under EU AI Act Article 51.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 42001:2023 + 22989 + 23053 + 23894 +
  24029 + TR 24027 + TR 24028 + TS 4213 + 38507
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web
  Linking), RFC 8259 (JSON), RFC 9421 (HTTP Message
  Signatures)
- ISO 8601, ISO 17442 LEI
- ISO/IEC 27001:2022
- W3C Trace Context, W3C Verifiable Credentials Data
  Model 2.0
- NIST AI RMF + AI 600-1
- EU AI Act Articles 50, 51, 52, 53, 54, 55, 71,
  72, 73
- US AISI Consortium voluntary commitments
- KR AI 기본법 reporting discipline

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by
the operator. Versioning uses `/v1/` path segments.
The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical.

The operator exposes:

- The HTTPS / JSON RESTful surface for the
  programme, AI-system inventory, AIMS, workforce-
  transition, frontier-policy, safety-test,
  incident, supply-chain, and dual-use records.
- The public-disclosure surface for AI Act Article
  51 systemic-risk transparency.
- The supervisory examination surface.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-ai-survival-2026",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "aiSystems":               "/v1/ai-systems",
    "aimsRecords":             "/v1/aims-records",
    "workforceTransition":     "/v1/workforce-transition",
    "humanOversight":          "/v1/human-oversight",
    "frontierPolicies":        "/v1/frontier-policies",
    "safetyTests":             "/v1/safety-tests",
    "incidents":               "/v1/incidents",
    "supplyChainIntegrity":    "/v1/supply-chain-integrity",
    "dualUse":                 "/v1/dual-use",
    "publicDisclosure":        "/v1/public-disclosure",
    "examination":             "/v1/examination",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 AI-System Inventory Endpoints

```
GET    /v1/ai-systems
GET    /v1/ai-systems/{systemId}
POST   /v1/ai-systems
PATCH  /v1/ai-systems/{systemId}/risk-classification
GET    /v1/ai-systems/{systemId}/technical-documentation
GET    /v1/ai-systems/{systemId}/fria          (Article
                                                27
                                                Fundamental
                                                Rights
                                                Impact
                                                Assessment)
```

## §4 AIMS Record Endpoints

```
GET    /v1/aims-records
POST   /v1/aims-records
GET    /v1/aims-records/{recordId}
GET    /v1/aims-records/{recordId}/risk-assessment
GET    /v1/aims-records/{recordId}/impact-assessment
GET    /v1/aims-records/{recordId}/management-review
```

## §5 Workforce-Transition and Human-Oversight
       Endpoints

```
GET    /v1/workforce-transition
POST   /v1/workforce-transition
PATCH  /v1/workforce-transition/{recordId}/programme-progress
GET    /v1/human-oversight?system={systemId}
POST   /v1/human-oversight
GET    /v1/human-oversight/{recordId}/competency-attestation
```

## §6 Frontier-Policy and Safety-Test Endpoints

```
GET    /v1/frontier-policies
POST   /v1/frontier-policies          (declare
                                       frontier
                                       commitments
                                       per AI Act
                                       Art 55)
GET    /v1/safety-tests?system={systemId}
POST   /v1/safety-tests
GET    /v1/safety-tests/{recordId}/findings
GET    /v1/safety-tests/{recordId}/systemic-risk-indicators
```

## §7 Incident-Reporting Endpoints

```
POST   /v1/incidents                  (record an
                                       incident or
                                       near-miss)
GET    /v1/incidents/{incidentId}
PATCH  /v1/incidents/{incidentId}/root-cause
PATCH  /v1/incidents/{incidentId}/eu-art-73-report
PATCH  /v1/incidents/{incidentId}/us-aisi-notify
PATCH  /v1/incidents/{incidentId}/kr-ai-기본법-report
```

## §8 Supply-Chain-Integrity Endpoints

```
GET    /v1/supply-chain-integrity?component={componentId}
POST   /v1/supply-chain-integrity     (record a
                                       provenance
                                       attestation)
GET    /v1/supply-chain-integrity/{recordId}/sbom
GET    /v1/supply-chain-integrity/{recordId}/model-card
GET    /v1/supply-chain-integrity/{recordId}/datasheet
```

## §9 Dual-Use and Export-Control Endpoints

```
GET    /v1/dual-use?system={systemId}
POST   /v1/dual-use                   (record an
                                       export-control
                                       classification)
GET    /v1/dual-use/{recordId}/licence-references
```

## §10 Public-Disclosure Endpoints (AI Act Article 51)

```
GET    /v1/public-disclosure/me/transition-plan
GET    /v1/public-disclosure/me/safety-summary
GET    /v1/public-disclosure/me/responsible-scaling-policy
GET    /v1/public-disclosure/me/incident-summary?period={iso}
```

The public-disclosure surface exposes the AI Act
Article 51 systemic-risk transparency artefacts and
the operator's voluntary AI Action Summit / Seoul
AI Summit / UK AISI commitment disclosures.

## §11 Examination Endpoints

```
GET    /v1/examination/programmes
GET    /v1/examination/ai-systems
GET    /v1/examination/aims-records
GET    /v1/examination/incidents
GET    /v1/examination/safety-tests
GET    /v1/examination/dual-use
GET    /v1/examination/audit-events
```

The examination scope is read-only and bound to the
authority's identity (US AISI + EU AI Office +
Member-State NCA + KR AI 안전연구소 + UN AI Advisory
Body).

## §12 Authentication and Authorisation

Bearer tokens conform to OAuth 2.1 with audiences
declared per surface. The frontier-policy endpoints
require elevated scope and the safety-test endpoints
require the four-eyes review on systemic-risk-
indicator escalation.

## §13 HTTP Status Codes

Standard codes apply (200 / 201 / 202 / 400 / 401 /
403 / 404 / 409 / 422 / 429 / 503) with Problem
Details bodies.

## §14 Webhook and Event Surface

Lifecycle events:

- `ai-system.deployed`, `ai-system.retired`,
  `ai-system.risk-reclassified`
- `aims.management-review-completed`
- `safety-test.completed`,
  `safety-test.systemic-risk-indicator-raised`
- `incident.reported`, `incident.root-cause-
  determined`, `incident.regulator-notified`
- `frontier-policy.published`,
  `frontier-policy.amended`
- `workforce-transition.programme-launched`

Webhook signatures use HTTP Message Signatures
(RFC 9421).

## §15 Bulk-Export Surface

```
POST   /v1/bulk-export
GET    /v1/bulk-export/{exportId}/manifest
GET    /v1/examination/audit-events.csv
```

Supports the supervisory authority's annual data
calls and the operator's annual transparency report.

## §16 AI-Safety-Institute Pre-Deployment Testing
        Surface

For frontier providers participating in AISI pre-
deployment testing:

```
POST   /v1/aisi/pre-deployment-testing/request
GET    /v1/aisi/pre-deployment-testing/{requestId}/status
GET    /v1/aisi/pre-deployment-testing/{requestId}/findings
POST   /v1/aisi/post-deployment-monitoring
```

The pre-deployment testing endpoint registers the
operator's frontier model with the relevant AISI
under the published memorandum of understanding;
findings are exchanged through the trusted-channel
infrastructure (cryptographically signed, access-
controlled). The post-deployment monitoring endpoint
forwards real-world performance data per AI Act
Article 72.

## §17 International Coordination Reporting Surface

```
GET    /v1/coordination/un-advisory-body/inputs
POST   /v1/coordination/gpai/working-group-inputs
GET    /v1/coordination/oecd-ai/observatory-disclosure
GET    /v1/coordination/unesco/ethics-implementation-report
```

The coordination surface forwards the operator's
contributions to the multilateral AI governance
bodies; the receiving channels follow each body's
published submission format.

## §18 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the public-disclosure
surface for AI Act Article 51 transparency, expose
the supervisory examination surface, and propagate
trace-context across the deployment-to-incident
chain.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-ai-survival-2026
- **Last Updated:** 2026-04-29
