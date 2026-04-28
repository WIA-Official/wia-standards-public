# WIA-food-traceability PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-food-traceability
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that a food-
traceability operator exposes for the records defined
in PHASE-1. Two complementary surfaces are described:
the GS1 EPCIS 2.0 RESTful capture-and-query surface;
and the HTTPS / JSON RESTful surface for operational
visibility, the FSMA 204 KDE record submission, the
consumer-facing channel (GS1 SmartLabel / Digital
Link), the recall-and-withdrawal coordination, and
the regulatory examination scope.

References (CITATION-POLICY ALLOW only):

- GS1 EPCIS 2.0 (REST + JSON-LD)
- GS1 Core Business Vocabulary 2.0
- GS1 Global Traceability Standard 2.0
- GS1 Digital Link 1.4
- GS1 SmartLabel
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web
  Linking), RFC 8259 (JSON), RFC 9421 (HTTP Message
  Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022, ISO 22000:2018
- W3C Trace Context, W3C Verifiable Credentials Data
  Model 2.0

---

## §1 Scope and Versioning

The operator exposes:

- The GS1 EPCIS 2.0 capture-and-query endpoints
  served from `/epcis/`.
- The GS1 Digital Link resolver served from the
  GS1 Digital Link domain registered for the GS1
  company prefix.
- The HTTPS / JSON RESTful surface served from a
  domain published by the operator under `/v1/`.

The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical for the JSON surface; the EPCIS 2.0 OpenAPI
schema published by GS1 is canonical for the EPCIS
endpoints.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-food-traceability",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "products":                "/v1/products",
    "locations":               "/v1/locations",
    "logisticUnits":           "/v1/logistic-units",
    "epcis":                   "/epcis/",
    "kdeRecords":              "/v1/kde-records",
    "supplyChainPartners":     "/v1/supply-chain-partners",
    "fsmsRecords":             "/v1/fsms-records",
    "recalls":                 "/v1/recalls",
    "consumerDisclosures":     "/v1/consumer-disclosures",
    "examination":             "/v1/examination",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 GS1 EPCIS 2.0 Capture-and-Query Surface

```
POST   /epcis/capture
GET    /epcis/queries
POST   /epcis/queries
GET    /epcis/queries/{queryName}
GET    /epcis/queries/{queryName}/events
GET    /epcis/events                  (query interface)
POST   /epcis/subscriptions
DELETE /epcis/subscriptions/{subscriptionId}
GET    /epcis/eventTypes
GET    /epcis/dispositions
GET    /epcis/bizSteps
```

EPCIS payloads follow JSON-LD per the EPCIS 2.0
specification; subscriptions support the partner-to-
partner event distribution.

## §4 GS1 Digital Link Resolver Surface

```
GET    /01/{gtin}                  (GTIN-keyed
                                    resolver)
GET    /01/{gtin}/10/{lot}         (GTIN + lot)
GET    /01/{gtin}/21/{serial}      (GTIN + serial)
GET    /01/{gtin}?linkType={linkType}
                                   (linkType-filtered
                                    resolver — e.g.,
                                    productInfo,
                                    recipeInfo,
                                    pip
                                    productInformation,
                                    recallInformation)
```

## §5 Product, Location, and Logistic Unit Endpoints

```
GET    /v1/products?gtin={gtin}
POST   /v1/products
GET    /v1/products/{productId}
GET    /v1/locations?gln={gln}
POST   /v1/locations
GET    /v1/locations/{locationId}
GET    /v1/logistic-units?sscc={sscc}
POST   /v1/logistic-units
GET    /v1/logistic-units/{unitId}/contents
```

## §6 KDE Record Endpoints (FSMA 204)

```
GET    /v1/kde-records?lot={traceabilityLotCode}
POST   /v1/kde-records
GET    /v1/kde-records/{kdeId}
GET    /v1/kde-records/by-cte?cte={cteKind}&from={iso}&to={iso}
```

## §7 Supply-Chain-Partner Endpoints

```
GET    /v1/supply-chain-partners
POST   /v1/supply-chain-partners
GET    /v1/supply-chain-partners/{partnerId}
GET    /v1/supply-chain-partners/{partnerId}/fsvp-verification
GET    /v1/supply-chain-partners/{partnerId}/gfsi-certification
```

## §8 FSMS Record Endpoints

```
GET    /v1/fsms-records
POST   /v1/fsms-records
GET    /v1/fsms-records/{recordId}
PATCH  /v1/fsms-records/{recordId}/corrective-actions
```

## §9 Recall Coordination Endpoints

```
POST   /v1/recalls                  (initiate a
                                     recall)
GET    /v1/recalls?gtin={gtin}
GET    /v1/recalls/{recallId}
GET    /v1/recalls/{recallId}/affected-trace
       (full trace of affected lots — upstream
        suppliers, downstream customers)
PATCH  /v1/recalls/{recallId}/units-recovered
GET    /v1/recalls/{recallId}/consumer-notice
```

The recall surface integrates with the operating
jurisdiction's recall-management channel — FDA
Recall Coordinator (US), RASFF (EU), MFDS 회수·폐기
명령 (KR).

## §10 Consumer-Disclosure Endpoints

```
GET    /v1/consumer-disclosures?gtin={gtin}
POST   /v1/consumer-disclosures
GET    /portal/v1/products/{gtin}/labels
       (consumer-facing product information page)
GET    /portal/v1/recalls/active
```

## §11 Examination Endpoints

```
GET    /v1/examination/programmes
GET    /v1/examination/products
GET    /v1/examination/kde-records
GET    /v1/examination/supply-chain-partners
GET    /v1/examination/fsms-records
GET    /v1/examination/recalls
GET    /v1/examination/audit-events
```

The examination scope is read-only and bound to the
authority's identity (US FDA + USDA for US-
jurisdiction; EU EFSA + Member-State competent
authorities for EU-jurisdiction; KR 식약처 + 식품
의약품안전평가원 for KR-jurisdiction).

## §12 Authentication and Authorisation

Bearer tokens conform to OAuth 2.1 with audiences
declared per surface. EPCIS endpoints follow the GS1
EPCIS 2.0 OAuth profile; recall-coordination
endpoints require elevated scope.

## §13 HTTP Status Codes

Standard codes apply (200 / 201 / 202 / 400 / 401 /
403 / 404 / 409 / 422 / 429 / 503) with Problem
Details bodies.

## §14 Webhook and Event Surface

Lifecycle events:

- `epcis-event.captured`
- `kde-record.captured`
- `recall.initiated`, `recall.escalated`,
  `recall.closed`
- `fsms.audit-completed`,
  `fsms.deviation-recorded`
- `consumer-disclosure.published`,
  `consumer-disclosure.amended`

Webhook signatures use HTTP Message Signatures (RFC
9421).

## §15 Bulk-Export and Examination Surface

```
POST   /v1/bulk-export
GET    /v1/bulk-export/{exportId}/manifest
GET    /v1/examination/recall-summary?period={iso}
```

The recall-summary endpoint surfaces the operator's
recall metrics (count, units affected, time-to-
notification, time-to-removal) for the requested
period — supporting the operator's annual food-safety-
performance report.

## §16 Lab-and-Testing Integration Surface

```
POST   /v1/lab/sample-events        (capture sample
                                     drawn from a
                                     specific lot)
POST   /v1/lab/results              (record test
                                     result against
                                     the sample)
GET    /v1/lab/results?sample={sampleId}
```

Sample events are EPCIS-aligned with bizStep
"sampling"; results carry the test-method, target-
analyte, threshold, and outcome. ISO/IEC 17025
accredited laboratory results carry the lab's
accreditation reference.

## §17 Sustainability and ESG Endpoints

```
GET    /v1/sustainability/carbon-footprint?gtin={gtin}
GET    /v1/sustainability/water-footprint?gtin={gtin}
GET    /v1/sustainability/origin-attestation?gtin={gtin}
```

The sustainability endpoints expose the per-product
ISO 14067 carbon-footprint, ISO 14046 water-
footprint, and the supply-chain-origin attestation
for use in consumer-facing channels.

## §18 EPCIS Subscription and Notification Surface

The push-style EPCIS event distribution:

```
GET    /epcis/subscriptions
POST   /epcis/subscriptions          (register a
                                      subscription
                                      with delivery
                                      URL + filter)
DELETE /epcis/subscriptions/{subscriptionId}
GET    /epcis/subscriptions/{subscriptionId}/status
```

Subscriptions deliver events via webhook (HTTPS POST
with signed body) to subscriber-registered URLs.
The subscription filter is expressed in the EPCIS
2.0 query language.

## §19 Examination Bulk-Download Surface

```
GET    /v1/examination/kde-records.csv
GET    /v1/examination/recall-summary?period={iso}
GET    /v1/examination/fsms-records.csv
GET    /v1/examination/audit-events.csv
```

The CSV downloads support the supervisory authority's
trend analysis and the operator's annual food-safety
performance report. Per-export digest declarations
preserve integrity for the receiving authority. The
recall-summary aggregates time-to-notification and
time-to-recovery metrics for the requested period.

## §20 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the EPCIS 2.0 capture-
and-query surface with the GS1 CBV 2.0 vocabulary,
expose the GS1 Digital Link resolver, expose the
recall-coordination surface integrated with the
operating jurisdiction's recall channel, and propagate
trace-context across the food-supply chain.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-food-traceability
- **Last Updated:** 2026-04-28
