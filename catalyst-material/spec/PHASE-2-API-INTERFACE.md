# WIA-catalyst-material PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-catalyst-material
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that a
catalyst-research operator (research laboratory,
reference-material producer, manufacturer
qualification office, process licensor's qualification
team, or proficiency-testing organiser) exposes for
the records defined in PHASE-1. The contract carries
the catalyst registry, characterisation upload,
performance data exchange, certified-reference-
material certificate publication, inter-laboratory
comparison ingestion, and chain-of-custody anchoring
endpoints.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details for HTTP APIs),
  RFC 8288 (Web Linking), RFC 7807 (legacy Problem
  Details), RFC 6901 / 6902 (JSON Pointer / Patch),
  RFC 8259 (JSON), RFC 4122 (UUID)
- IETF RFC 9421 (HTTP Message Signatures) — used for
  the per-request signature scheme that anchors the
  laboratory upload to the laboratory's accreditation
  certificate
- IETF RFC 8615 (well-known URIs)
- W3C Trace Context — used for the per-request trace
  identifier carried across the laboratory-to-
  certification-body call chain
- ISO/IEC 27001:2022 (information-security management)
- ISO 17034:2016, ISO/IEC 17025:2017, ISO Guide 31:
  2015, ISO Guide 35:2017
- ISO 5725-1:2023 / 5725-2:2019 / 5725-6:1994
  (accuracy and precision)
- ISO 9277:2022, ISO 13322-1:2014 / 13322-2:2021,
  ISO 15901-1:2016 / 15901-2:2022 / 15901-3:2007
  (characterisation test methods cited in the test-
  method enumeration carried by §4 endpoints)
- ASTM D3663-20, ASTM D4222-20, ASTM D4567-19,
  ASTM D4641-17, ASTM D4824-13 (characterisation
  test methods cited in the test-method enumeration
  carried by §4 endpoints)
- IUPAC Recommendations 2007 — Manual of Methods and
  Procedures for Catalyst Characterization (Pure and
  Applied Chemistry)
- IUPAC Quantities, Units and Symbols in Physical
  Chemistry (the "Green Book") — the unit-and-symbol
  vocabulary carried by the JSON envelope
- EU REACH Regulation (EC) No 1907/2006 and EU CLP
  Regulation (EC) No 1272/2008 (the substance-
  classification fields carried by the registration
  endpoint in §3)

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical for the WIA endpoints
declared in this PHASE. Schema changes follow the
non-breaking conventions in PHASE-1 §2 (additive
fields, additive enum values, no semantic re-use of
existing field names). Every endpoint carries a per-
request signature using HTTP Message Signatures
(RFC 9421) anchored to the laboratory's ISO/IEC 17025
accreditation certificate or the reference-material
producer's ISO 17034 accreditation certificate; the
signature key set is published at
`/.well-known/wia/catalyst-material/keys.json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-catalyst-material",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":             "/v1/programmes",
    "catalysts":              "/v1/catalysts",
    "characterisations":      "/v1/characterisations",
    "performance":            "/v1/performance-records",
    "crmCertificates":        "/v1/crm-certificates",
    "ilcRecords":             "/v1/ilc-records",
    "custody":                "/v1/custody-events",
    "openapi":                "/v1/openapi.json",
    "wellKnown":              "/.well-known/wia/catalyst-material"
  }
}
```

## §3 Catalyst Registration Endpoints

### §3.1 Register a catalyst

```
POST /v1/catalysts
Content-Type: application/json
Signature: <RFC 9421 signature over the body>
```

Request body carries the §3 record from PHASE-1
(catalystId, identifierBindings, catalystClass,
composition, preparationMethod, pretreatment,
hazardLabelling). The server returns `201 Created`
with the canonical resource URL at
`/v1/catalysts/{catalystId}` and the assigned
operator-internal sequence number.

Where the catalyst contains a substance with an
existing REACH registration, the server validates the
`identifierBindings.reachRegistrationNumber` against
the issuing-authority discipline (the registration-
number format defined by the EU REACH legal text
Annex VI). A mismatch returns `422 Unprocessable
Entity` with an RFC 9457 problem document carrying
`type: /problems/reach-registration-format` and the
position of the offending field expressed as a JSON
Pointer (RFC 6901).

### §3.2 Retrieve a catalyst

```
GET /v1/catalysts/{catalystId}
Accept: application/json
```

Returns the registered catalyst record together with
the link set covering its characterisations,
performance records, and CRM certificates. The
response carries a `Link` header (RFC 8288) pointing
at the related resources so that a downstream client
can traverse the record graph without searching the
collection endpoint.

### §3.3 Search the catalyst registry

```
GET /v1/catalysts?class={class}&support={support}
&containsActiveComponent={cas}&hazardClass={class}
&accreditationScope={scope}
&page={cursor}&size={size}
```

The query envelope mirrors the FHIR R5 search-style
parameter convention adapted for catalyst attributes.
The response is an RFC 8288 `Link`-paginated
collection.

## §4 Characterisation Upload Endpoints

### §4.1 Upload a characterisation result

```
POST /v1/characterisations
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature over the parts>
```

The multipart body carries one JSON part with the §4
record from PHASE-1 (technique, instrument,
testStandard, derivedMetrics, uncertaintyBudget) and
one or more file parts holding the raw-data files
referenced by `rawData`. The server stores the raw-
data files under a content-addressable URI (the
SHA-256 hex digest is used as the path segment) so
that PHASE-3 §6 can later reference the artefact by
hash.

The server enforces the `testStandard` enumeration
against the `technique` enumeration: a `BET-N2-77K`
technique with a `testStandard` other than ISO 9277,
ASTM D3663, ASTM D4222, ASTM D4567, or ASTM D4641
returns `422 Unprocessable Entity` with a problem
document that lists the allowed test methods for the
declared technique.

### §4.2 Retrieve a characterisation result

```
GET /v1/characterisations/{characterisationId}
Accept: application/json
```

Returns the characterisation envelope. The response
links to the raw-data file at
`/v1/characterisations/{characterisationId}/raw` and
to the sibling characterisations carried by the same
catalyst.

### §4.3 List characterisations for a catalyst

```
GET /v1/catalysts/{catalystId}/characterisations
?technique={technique}&testStandard={standard}
&measuredAfter={iso8601}&measuredBefore={iso8601}
&page={cursor}&size={size}
```

## §5 Performance Endpoints

### §5.1 Submit a performance record

```
POST /v1/performance-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §5 record from PHASE-1
(reactionDescriptor, testRig, operatingPoint,
performanceMetrics, deactivationCurve). The server
verifies that every reactant and product CAS Registry
Number is well-formed (the IUPAC-published CAS-RN
check-digit rule), and that `performanceMetrics.
turnoverFrequency` and `performanceMetrics.
apparentActivationEnergy` carry the unit symbols
declared by IUPAC Green Book §2 (`s^-1` and
`kJ.mol^-1` respectively). A unit-symbol mismatch
returns `422 Unprocessable Entity` with a problem
document.

### §5.2 Retrieve a performance record

```
GET /v1/performance-records/{performanceId}
Accept: application/json
```

### §5.3 Search performance records

```
GET /v1/performance-records
?catalystRef={id}&reactionName={iupac}
&temperatureMinK={K}&temperatureMaxK={K}
&pressureMinPa={Pa}&pressureMaxPa={Pa}
&minConversion={fraction}
&page={cursor}&size={size}
```

## §6 CRM Certificate Endpoints

### §6.1 Publish a certificate

```
POST /v1/crm-certificates
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Only an operator with an active ISO 17034
accreditation reference in `programmeId.
accreditationStatus` is authorised to publish a CRM
certificate. The server verifies the
`characterisationDesign` enumeration, the
`assignedValues` per-property uncertainty against
the ISO Guide 35 expanded-uncertainty rule (the
expanded uncertainty MUST equal the coverage factor
multiplied by the combined standard uncertainty
declared in `homogeneityStudy` and `stabilityStudy`),
and rejects a publication where the `intendedUse`
binding does not name the characterisation technique
declared in the underlying characterisation records.

### §6.2 Retrieve a certificate

```
GET /v1/crm-certificates/{certificateId}
Accept: application/json
```

The response carries the certificate envelope, the
homogeneity and stability study summaries, and a
machine-readable form of the intended-use declaration
suitable for re-issuance into a customer's
laboratory-information-management system.

## §7 Inter-Laboratory Comparison Endpoints

### §7.1 Open a comparison round

```
POST /v1/ilc-records
Content-Type: application/json
Signature: <RFC 9421 signature from the comparison
            organiser's accreditation certificate>
```

The request envelope carries the §7 record from
PHASE-1 (comparisonScheme, participantSet,
assignedReference, precisionEstimates,
outlierTreatment).

### §7.2 Submit a participant result

```
PUT /v1/ilc-records/{comparisonId}
   /participants/{participantId}/result
Content-Type: application/json
Signature: <RFC 9421 signature from the participant's
            accreditation certificate>
```

The participant's accreditation reference declared in
the request signature MUST match the participant's
accreditation reference declared in the comparison
round; a mismatch returns `403 Forbidden`.

### §7.3 Close the round

```
POST /v1/ilc-records/{comparisonId}/close
Signature: <RFC 9421 signature from the organiser>
```

The server runs the declared outlier-treatment
algorithm (Cochran / Grubbs per ISO 5725-2 §7 or the
robust-Z per ISO 13528 §9), publishes the per-
laboratory z-score, and locks the round against
further submission.

## §8 Chain-of-Custody Endpoint

### §8.1 Anchor a custody event

```
POST /v1/custody-events
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §8 record from PHASE-1
(custodyEvent, eventTimestamp, performingParty,
observerParty, hashOfArtefacts). The server appends
the event to the per-catalyst custody chain and
returns the position index. PHASE-3 §6 specifies how
the custody chain is anchored to a public
transparency log so that the chain cannot be silently
mutated after the event.

## §9 Error Reporting

Errors are returned using RFC 9457 Problem Details.
The problem-type identifiers are stable strings rooted
at `/problems/` and are documented in the OpenAPI
document. Validation errors carry a `pointer` field
whose value is a JSON Pointer (RFC 6901) into the
offending request body. The server emits a per-
request `traceparent` header (W3C Trace Context) so
that a downstream call chain can be reconstructed for
post-incident review.
