# WIA-economic-integration PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-economic-integration
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract
that an economic-integration operator (customs
administration, single-window operator,
national statistical office, chamber of
commerce, authorised economic operator,
freight forwarder, financial institution, or
economic-union secretariat) exposes for the
records defined in PHASE-1. The contract
carries the trade-declaration ingestion, UN/
EDIFACT message exchange, ISO 20022 payment
publication, UCP 600 documentary-credit
publication, certificate-of-origin issuance,
UN COMTRADE statistics submission, and
chain-of-custody anchoring endpoints. The API
is the canonical interoperability layer
between the operator, the cross-border partner
agency, the financial institution, the freight
forwarder, and the trade-statistics
secretariat.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110, RFC 9111, RFC 9457, RFC 8288,
  RFC 6901 / 6902, RFC 8259, RFC 4122, RFC
  9421, RFC 8615
- W3C Trace Context, W3C ODRL 2.2, W3C VC
  v2.0
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015
- WCO SAFE Framework, WCO Data Model v3, WCO
  HS-2022, WCO AEO Programme
- UN/EDIFACT (ISO 9735-1 to -10), UN/CEFACT
  Recommendations 1, 16, 21, 33, 36
- UN COMTRADE submission specification
- ISO 20022 message family
- ISO 4217, ISO 3166-1, ISO 9362 (BIC), ISO
  13616 (IBAN), ISO 17442 (LEI)
- ICC Incoterms 2020, ICC UCP 600, ICC ISBP
  745, ICC URDG 758, ICC URBPO 750
- WTO TFA, WTO GATT 1994, WTO GPA, WTO TRIPS
- KR 관세법, KR 대외무역법, KR 외국환거래법

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published
by the operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical. Schema
changes follow the non-breaking conventions
in PHASE-1 §2. Every endpoint carries a per-
request signature using HTTP Message
Signatures (RFC 9421) anchored to the
operator's institutional identifier (the
operator's customs-system trader registration,
the AEO certificate identifier, or the
operator's national-authority registration);
the signature key set is published at
`/.well-known/wia/economic-integration/keys.
json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-economic-integration",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":         "/v1/programmes",
    "declarations":       "/v1/declarations",
    "edifactMessages":    "/v1/edifact-messages",
    "paymentRecords":     "/v1/payment-records",
    "documentaryCredits": "/v1/documentary-credits",
    "originCertificates": "/v1/origin-certificates",
    "comtradeFeeds":      "/v1/comtrade-feeds",
    "custody":            "/v1/custody-events",
    "openapi":            "/v1/openapi.json",
    "wellKnown":          "/.well-known/wia/economic-integration"
  }
}
```

## §3 Trade Declaration Endpoints

### §3.1 Lodge a declaration

```
POST /v1/declarations
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §3 record from PHASE-1
(declarationType, consigneeRef, consignorRef,
goodsDescription, declaredCustomsValue,
incoterm, transportDetails). The server
validates the `goodsDescription[].hsCode`
against the WCO HS-2022 commodity register;
an unknown ten-digit code returns `422
Unprocessable Entity` at `/problems/wco-hs-
2022-unknown-code` and the offending field's
JSON Pointer (RFC 6901).

The server validates the `incoterm` against
the WTO Customs Valuation methods declared in
`declaredCustomsValue.method`: a `DDP`
incoterm with a Method 1 transaction value
that does not include the duty and tax in the
declared value is rejected as inconsistent
with the incoterm's risk-and-cost allocation.

### §3.2 Retrieve a declaration

```
GET /v1/declarations/{declarationId}
Accept: application/json
```

### §3.3 Search declarations

```
GET /v1/declarations?type={type}
&hsChapter={chapter}&consigneeLei={lei}
&lodgedBetween={iso8601}/{iso8601}
&page={cursor}&size={size}
```

### §3.4 AEO mutual-recognition lookup

```
GET /v1/declarations/{declarationId}/aeo-
   mutual-recognition
Accept: application/json
```

The endpoint returns the consignee's and
consignor's AEO status against the operator's
declared mutual-recognition partner list per
the WCO SAFE Framework MRA register.

## §4 UN/EDIFACT Endpoints

### §4.1 Transmit an EDIFACT message

```
POST /v1/edifact-messages
Content-Type: application/edifact
Signature: <RFC 9421 signature>
```

Request body is the complete EDIFACT message
with the UNB / UNH / UNT / UNZ envelope per
ISO 9735-1. The server validates the message
against the declared `edifactDirectory` (D-
21B) and the declared `messageType` schema
(the IFTMIN segment table, the CUSDEC segment
table, the INVOIC segment table).

### §4.2 Retrieve an EDIFACT message

```
GET /v1/edifact-messages/{messageId}
Accept: application/edifact
```

### §4.3 Translate EDIFACT to UN/CEFACT XML

```
GET /v1/edifact-messages/{messageId}/cefact-xml
Accept: application/xml
```

The server returns the UN/CEFACT XML
representation per the per-message XML
schema published by UN/CEFACT.

## §5 ISO 20022 Payment Endpoints

### §5.1 Instruct a payment

```
POST /v1/payment-records
Content-Type: application/json
Signature: <RFC 9421 signature from the
            initiating financial institution's
            BIC>
```

Request body carries the §5 record from PHASE-1
(iso20022Message, initiatingParty,
beneficiaryParty, instructedAmount,
paymentPurpose, uniqueEnd-toEndTransactionRef).
The server validates the `iso20022Message`
against the operator's declared message
family and the per-message ISO 20022 schema.

The `uniqueEnd-toEndTransactionRef` (UETR)
follows the SWIFT GPI discipline so that the
per-payment leg can be tracked across the
correspondent-banking chain.

### §5.2 Retrieve a payment record

```
GET /v1/payment-records/{paymentId}
Accept: application/json
```

The response carries the per-leg status
(initiated, in-transit-via-correspondent,
beneficiary-credited, returned, rejected) so
that the consumer can track the cross-border
payment.

## §6 Documentary Credit Endpoints

### §6.1 Issue a documentary credit

```
POST /v1/documentary-credits
Content-Type: application/json
Signature: <RFC 9421 signature from the
            issuing bank's BIC>
```

Request body carries the §6 record from PHASE-1
(creditType, issuingBank, beneficiaryBank,
applicantRef, beneficiaryRef, creditAmount,
expirePlace, expiryDate, documentsRequired,
isbpProfile).

### §6.2 Present documents under a credit

```
POST /v1/documentary-credits/{creditId}/
   documents
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature from the
            beneficiary or its bank>
```

The multipart body carries the per-document
attachments listed in `documentsRequired`. The
server records the presentation timestamp and
the per-document compliance state per UCP
600 Articles 14-16.

### §6.3 Retrieve a documentary credit

```
GET /v1/documentary-credits/{creditId}
Accept: application/json
```

## §7 Certificate-of-Origin Endpoints

### §7.1 Issue an origin certificate

```
POST /v1/origin-certificates
Content-Type: application/json
Signature: <RFC 9421 signature from the
            issuing chamber of commerce or
            competent authority>
```

Request body carries the §7 record from PHASE-1
(issuingChamberRef, beneficiaryRef,
goodsDescription, agreementRef,
validityPeriod). The server validates the
`agreementRef` against the operator's
declared preferential-trade agreement set.

### §7.2 Retrieve an origin certificate

```
GET /v1/origin-certificates/{certificateId}
Accept: application/json
```

## §8 UN COMTRADE Submission Endpoints

### §8.1 Submit a per-period dataset

```
POST /v1/comtrade-feeds
Content-Type: application/json
Signature: <RFC 9421 signature from the
            national statistical office>
```

Request body carries the per-period dataset
(month, quarter, year). The server validates
the per-row commodity code against the WCO
HS-2022 commodity register and the partner-
country code against ISO 3166-1.

### §8.2 Retrieve a UN COMTRADE feed

```
GET /v1/comtrade-feeds/{feedId}
Accept: application/json | text/csv
```

## §9 Custody and Error Reporting

### §9.1 Anchor a custody event

```
POST /v1/custody-events
Content-Type: application/json
Signature: <RFC 9421 signature>
```

### §9.2 Error envelope

Errors are returned using RFC 9457 Problem
Details. Validation errors carry a `pointer`
field (RFC 6901). The server emits a per-
request `traceparent` header (W3C Trace
Context).

## §10 Concurrency and Cache Discipline

Every retrieval endpoint emits an `ETag`
header (RFC 9110 §8.8.3). Conditional
requests are honoured.

## §11 Bulk Export for Trade-Statistics
       Aggregators

```
GET /v1/declarations:bulk
Accept: application/x-ndjson
Authorization: <bearer token from a national
                 statistical office or an
                 international trade
                 organisation>
```

A national statistical office or an inter-
national trade organisation runs a bulk
ingest of the operator's declaration register.

## §12 Single-Window Federation

```
POST /v1/declarations/{declarationId}/single-
   window-route
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A trade-facilitation single-window operator
participating in a regional single-window
network (the ASEAN Single Window, the EU
Customs Single Window) routes the declaration
to the relevant peer authority's intake
endpoint per UN/CEFACT Recommendation 36.

## §13 Schema-Validation and Conformance

The OpenAPI 3.1 document at `/v1/openapi.json`
carries JSON Schema 2020-12 schemas. The
EDIFACT endpoint follows the ISO 9735-1
syntax rules and the per-directory message
schemas. The ISO 20022 endpoints follow the
per-message ISO 20022 schemas published by
the SWIFT MyStandards reference set.

```
GET /v1/conformance/test-vectors
Accept: application/json
```

The operator publishes the conformance test
vectors used to qualify the API implementation.
Each vector references the relevant WCO Data
Model, UN/EDIFACT directory, ISO 20022
message, ICC UCP 600, or WTO TFA clause.

## §14 Webhook Endpoint for Customs-Decision Notifications

```
POST /v1/programmes/{programmeId}/webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A trader, a freight forwarder, or a financial
institution registers a webhook to receive a
push notification when a customs decision
(release, hold, query, seizure) is recorded
on a declaration that the registrant is bound
to.
