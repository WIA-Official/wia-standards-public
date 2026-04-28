# WIA-ci-octave PHASE 2 — API Interface Specification

**Standard:** WIA-ci-octave
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that
a CI-OCTAVE risk-management operator (CISO
office, public-sector agency, critical-
infrastructure operator, healthcare-IT risk
manager, financial-services risk officer, MSSP,
third-party-risk programme operator, ISO 27001
certification body) exposes for the records
defined in PHASE-1. The contract carries the
critical-asset registration, area-of-concern
recording, risk-score upload, risk-treatment
decision, audit-cycle recording, and chain-of-
custody anchoring endpoints.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110, RFC 9111, RFC 9457, RFC 8288,
  RFC 6901 / 6902, RFC 8259, RFC 4122, RFC
  9421, RFC 8615
- W3C Trace Context
- ISO/IEC 27001:2022, ISO/IEC 27002:2022,
  ISO/IEC 27005:2022, ISO/IEC 27017:2015,
  ISO/IEC 27018:2019, ISO/IEC 27036 series
- ISO 31000:2018, ISO/IEC 31010:2019
- ISO/IEC 17021-1:2015, ISO/IEC 17065:2012
- NIST SP 800-30 Rev. 1, NIST SP 800-37 Rev. 2,
  NIST SP 800-53 Rev. 5, NIST CSF 2.0
- US FIPS PUB 199, FIPS PUB 200, FedRAMP
- EU NIS2 Directive 2022/2555, EU DORA
  Regulation 2022/2554, EU CRA Regulation
  2024/2847
- Open FAIR (The Open Group), CMU SEI CERT
  OCTAVE Allegro and OCTAVE FORTE
- W3C Verifiable Credentials Data Model v2.0

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
FedRAMP authorisation, the KR-ISMS-P
certification, the SEI CERT OCTAVE-trained-
evaluator designation); the signature key set
is published at
`/.well-known/wia/ci-octave/keys.json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-ci-octave",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":          "/v1/programmes",
    "assets":              "/v1/assets",
    "areasOfConcern":      "/v1/areas-of-concern",
    "riskScores":          "/v1/risk-scores",
    "treatments":          "/v1/treatments",
    "auditRecords":        "/v1/audit-records",
    "custody":             "/v1/custody-events",
    "openapi":             "/v1/openapi.json",
    "wellKnown":           "/.well-known/wia/ci-octave"
  }
}
```

## §3 Asset Endpoints

### §3.1 Register a critical asset

```
POST /v1/assets
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §3 record from
PHASE-1 (assetName, assetType, fipsCategorisation,
rationale). The server validates the FIPS PUB
199 categorisation envelope (each of
confidentiality, integrity, availability is
"low", "moderate", or "high"). A categorisation
that omits one of the three properties is
rejected with `422 Unprocessable Entity` at
`/problems/fips-199-incomplete-categorisation`.

### §3.2 Retrieve an asset

```
GET /v1/assets/{assetId}
Accept: application/json
```

### §3.3 Search assets

```
GET /v1/assets?type={type}&fipsHigh={true|false}
&page={cursor}&size={size}
```

The `fipsHigh` filter selects assets whose
FIPS PUB 199 categorisation has any of CIA at
"high" — these assets are the operator's high-
value-target subset and are subject to the
operator's elevated control discipline.

## §4 Area-of-Concern Endpoints

### §4.1 Identify an area of concern

```
POST /v1/areas-of-concern
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §4 record from
PHASE-1 (assetRef, threatScenario, threatActor,
threatVector, consequenceArea). The server
enforces that the `threatActor` and
`threatVector` declarations are consistent
with the operator's documented threat-modelling
discipline.

### §4.2 Retrieve an area of concern

```
GET /v1/areas-of-concern/{concernId}
Accept: application/json
```

## §5 Risk-Score Endpoints

### §5.1 Score a risk

```
POST /v1/risk-scores
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §5 record from
PHASE-1. The server enforces the NIST SP
800-30 Appendix I Table I-2 mapping from
likelihood × impact to overallRisk. A submission
whose declared overallRisk does not match the
table is rejected with `422 Unprocessable
Entity` at `/problems/nist-800-30-table-i-2-
mismatch` so that the operator's risk
classification stays anchored to the published
table.

### §5.2 Retrieve a risk score

```
GET /v1/risk-scores/{riskScoreId}
Accept: application/json
```

### §5.3 Bulk-export risk scores

```
GET /v1/risk-scores:bulk
Accept: application/x-ndjson
Authorization: <bearer token from the audit
                 committee, the certification
                 body, or the supervisory
                 regulator>
```

A regulator running a sectoral-risk audit
requests the operator's risk-score register
as a newline-delimited JSON stream. The
endpoint streams the risk-score records in
score-date order; a consumer resuming the
stream provides an `If-Resume-After` header
carrying the last-received score timestamp.

## §6 Risk-Treatment Endpoints

### §6.1 Decide a risk treatment

```
POST /v1/treatments
Content-Type: application/json
Signature: <RFC 9421 signature from the
            named risk owner>
```

Request body carries the §6 record from
PHASE-1. The server enforces that an "accept"
treatment-decision MUST carry a non-empty
`acceptanceRationale` and that a "transfer"
treatment-decision MUST carry a `transferBinding`
referencing the cyber-insurance policy or the
contractual-transfer envelope.

### §6.2 Retrieve a treatment

```
GET /v1/treatments/{treatmentId}
Accept: application/json
```

### §6.3 Per-control mitigation tracking

```
GET /v1/treatments/{treatmentId}/controls
Accept: application/json
```

The response carries the per-control
implementation status, the per-control owner,
and the per-control deadline.

## §7 Audit-Cycle Endpoints

### §7.1 Open an audit cycle

```
POST /v1/audit-records
Content-Type: application/json
Signature: <RFC 9421 signature from the
            audit committee chair or the
            internal audit director>
```

### §7.2 Close an audit cycle

```
PATCH /v1/audit-records/{auditRecordId}
Content-Type: application/json
Signature: <RFC 9421 signature>
```

The PATCH request body carries the closing
findings count, the management response, and
the per-cycle audit-completion timestamp.

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

## §10 Webhook Endpoint for Sectoral Notifications

```
POST /v1/programmes/{programmeId}/webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A sectoral CSIRT, a sector-specific ISAC
(the EU CSIRTs Network, the US sector-specific
ISACs), or the supervisory regulator
registers a webhook to receive a push
notification when a critical-severity finding
is issued, when a treatment-decision deadline
is breached, or when a per-asset incident
record is published.

## §11 Schema-Validation and Conformance

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

## §12 Authorities and Roles

| Role                   | Capabilities |
|------------------------|------|
| `risk-analyst`         | Register asset, identify area of concern, score risk |
| `risk-owner`           | Decide treatment, accept residual risk |
| `audit-committee`      | Open audit cycle, close audit cycle |
| `internal-audit`       | Read full operator state, publish audit finding |
| `external-auditor`     | Read scoped operator state under engagement contract |
| `regulator`            | Read full operator state under sectoral mandate |
| `mssp-evaluator`       | Read scoped customer state under contract |

The operator's API enforces the per-role
authorisation policy.

## §13 NIST RMF Authorisation Endpoint

```
POST /v1/programmes/{programmeId}/rmf-authorisation
Content-Type: application/json
Signature: <RFC 9421 signature from the
            authorising official>
```

A US federal-agency operator running the
NIST RMF authorisation cycle submits the per-
system authorisation envelope per NIST SP
800-37 Rev. 2. The envelope carries the
authorisation-decision (Authorisation to
Operate, Authorisation to Use, Common Control
Authorisation, Denial of Authorisation), the
per-system authorisation-termination date,
and the per-system continuous-monitoring
strategy.

## §14 Verifiable-Credentials Re-Issuance

```
GET /v1/audit-records/{auditRecordId}/credential
Accept: application/json
```

An ISO/IEC 27001 certification or a NIST RMF
ATO is re-issuable as a W3C Verifiable
Credential signed by the certification
body's signing-key set so that a downstream
counterparty can verify the certification
without contacting the certification body
directly.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §15 Per-Cycle Comparison Endpoint

```
GET /v1/programmes/{programmeId}/cycle-comparison
?fromCycle={cycleId}&toCycle={cycleId}
Accept: application/json
```

The operator's API publishes the per-cycle
comparison envelope between two declared
cycles — the per-asset risk-trend (improving,
stable, deteriorating), the per-treatment
implementation-status delta, and the per-
finding remediation-status delta. The
comparison is consumed by the operator's
audit committee, the certification body, and
the supervisory regulator.

## §16 Continuous-Monitoring Endpoint

```
POST /v1/programmes/{programmeId}/continuous-monitoring
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A federal-agency operator running the NIST
RMF continuous-monitoring discipline publishes
the per-period continuous-monitoring envelope
per NIST SP 800-37 Step 7. The envelope
carries the per-control monitoring outcome,
the per-control deviation, the per-deviation
remediation envelope, and the per-period
authorising-official information report.

## §17 Multi-Language Programme Surface

```
GET /v1/programmes/{programmeId}?lang={lang}
Accept: application/json
```

The operator's programme record is published
in each operator-declared language with the
programme's title, the operator's name, and
the per-record narrative descriptions
translated. The classification codes (FIPS
PUB 199 categorisation, NIST SP 800-30 risk
levels, ISO/IEC 27002 control identifiers)
are language-neutral and carried in the
canonical form across all retrievals.
