# WIA-fuel-cell PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-fuel-cell
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that a
fuel-cell deployment exposes for the records defined
in PHASE-1. Consumers include the system manufacturer,
the system integrator, the operating jurisdiction's
authority having jurisdiction (AHJ), the operating
jurisdiction's grid system operator (where the fuel
cell is grid-coupled), the operating jurisdiction's
vehicle-type-approval authority (where the fuel cell
powers a vehicle), the operating jurisdiction's
hydrogen-fuel-quality auditor, the IECEx certification
body (where Ex zones apply), and the deployment's own
maintenance and operations platforms.

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
- ISO 14687 (hydrogen fuel quality)
- IEC 62282 series
- IEEE 1547-2018 / IEEE 1547.1-2020
- IECEx + IEC 60079 series
- UN GTR 13 / UN Regulation No. 134
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
deployment. Versioning uses `/v1/` path segments. The
OpenAPI 3.1 document at `/v1/openapi.json` is canonical.

This API is the deployment-facing facade for fuel-cell
records. Real-time stack telemetry (per-cell voltage,
per-cell temperature, fuel utilisation) flows through
the manufacturer's SCADA-equivalent surface; this API
records the artefacts of regulatory-grade significance
(fuel-quality verification, IEEE 1547 conformance,
AHJ acceptance, incident-record submissions).

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-fuel-cell",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":             "/v1/programmes",
    "stacks":                 "/v1/stacks",
    "balanceOfPlant":         "/v1/balance-of-plant",
    "fuelQualityRecords":     "/v1/fuel-quality-records",
    "gridInterconnections":   "/v1/grid-interconnections",
    "vehicleOnboards":        "/v1/vehicle-onboards",
    "commissioningRecords":   "/v1/commissioning-records",
    "periodicInspections":    "/v1/periodic-inspections",
    "incidents":              "/v1/incidents",
    "evidence":               "/v1/evidence",
    "openapi":                "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
POST   /v1/programmes              — register a programme
GET    /v1/programmes/{pid}        — retrieve programme
PATCH  /v1/programmes/{pid}/status — advance status
PATCH  /v1/programmes/{pid}/hazardous-area-classification
                                   — record IEC 60079-
                                     10-1 area
                                     classification
PATCH  /v1/programmes/{pid}/grid-interconnection
                                   — record grid
                                     interconnection
                                     mode
```

Programmes that declare `applicationClass=stationary-*`
without an `iec62282-3-100` reference for the IEC
62282-3-100 stationary-power-systems-safety attestation
return `409` with type
`urn:wia:fuel-cell:iec-62282-3-100-required`.

## §4 Stack Records

```
POST   /v1/programmes/{pid}/stacks — register a stack
PATCH  /v1/stacks/{sid}/iec-62282-test
                                   — attach IEC 62282
                                     test report
                                     reference
GET    /v1/stacks/{sid}            — retrieve stack
GET    /v1/programmes/{pid}/stacks — list stacks
```

Stack submissions whose `fuelInletGrade` is below the
`fuelQualityRecords` registered for the programme
return `409` with type
`urn:wia:fuel-cell:iso-14687-grade-mismatch`.

## §5 Balance-of-Plant Records

```
POST   /v1/programmes/{pid}/balance-of-plant
                                   — register the BoP
                                     for a programme
PATCH  /v1/balance-of-plant/{bid}/iecex-equipment
                                   — record IECEx
                                     Certificate of
                                     Conformity
                                     reference for an
                                     Ex-zoned BoP
                                     equipment item
GET    /v1/balance-of-plant/{bid}  — retrieve BoP
                                     record
```

BoP submissions in zoned `hazardousAreaClassification`
without `iecExEquipmentRefs` for each equipment item
intended to operate in the zone return `422` with type
`urn:wia:fuel-cell:iecex-coc-missing-for-zoned-equipment`.

## §6 Fuel-Quality Records

```
POST   /v1/programmes/{pid}/fuel-quality-records
                                   — register a fuel-
                                     quality verification
                                     per ISO 14687
PATCH  /v1/fuel-quality-records/{fid}/conformance-verdict
                                   — record verdict
                                     from accredited
                                     laboratory
GET    /v1/fuel-quality-records/{fid}
                                   — retrieve fuel-
                                     quality record
```

Fuel-quality submissions whose laboratory does not
hold ISO/IEC 17025 accreditation for the contaminant
panel return `422` with type
`urn:wia:fuel-cell:iso-17025-accreditation-required`.
Operations on a programme whose most-recent fuel-
quality verdict is `non-conforming-supply-rejected`
return `409` with type
`urn:wia:fuel-cell:fuel-supply-rejected-cannot-operate`.

## §7 Grid-Interconnection Records

```
POST   /v1/programmes/{pid}/grid-interconnections
                                   — register a grid-
                                     interconnection
                                     event (initial
                                     and periodic
                                     re-test)
PATCH  /v1/grid-interconnections/{gid}/utility-agreement
                                   — record the grid
                                     system operator's
                                     interconnection
                                     agreement
                                     reference
GET    /v1/grid-interconnections/{gid}
                                   — retrieve record
```

Grid-interconnection submissions whose
`ieee1547TestRef` does not include a category-i / ii /
iii ride-through declaration return `422` with type
`urn:wia:fuel-cell:ieee-1547-ride-through-category-
required`.

## §8 Vehicle-Onboard Records

```
POST   /v1/programmes/{pid}/vehicle-onboards
                                   — register a
                                     vehicle-onboard
                                     deployment
PATCH  /v1/vehicle-onboards/{vid}/un-r134-type-approval
                                   — record UN R134
                                     type-approval
                                     certificate
                                     reference
GET    /v1/vehicle-onboards/{vid}  — retrieve vehicle-
                                     onboard record
```

Vehicle-onboard submissions whose `vehicleClass` is in
the M / N classes without `unGtr13TestRef` and
`unR134TypeApprovalRef` (where the operating
jurisdiction recognises UN R134 type approval) return
`409` with type
`urn:wia:fuel-cell:un-gtr-13-r134-evidence-required`.

## §9 Commissioning Records

```
POST   /v1/programmes/{pid}/commissioning-records
                                   — register
                                     commissioning
PATCH  /v1/commissioning-records/{cid}/ahj-acceptance
                                   — record AHJ
                                     acceptance
                                     reference
GET    /v1/commissioning-records/{cid}
                                   — retrieve record
```

Commissioning submissions for stationary programmes
without an IEC 62282-3-300 installation-inspection
reference return `422` with type
`urn:wia:fuel-cell:iec-62282-3-300-installation-evidence-
required`.

## §10 Periodic Inspections

```
POST   /v1/programmes/{pid}/periodic-inspections
                                   — register an
                                     inspection
PATCH  /v1/periodic-inspections/{iid}/remediation
                                   — record remediation
                                     actions
GET    /v1/periodic-inspections/{iid}
                                   — retrieve record
```

For Ex-zoned programmes, the IEC 60079-17 inspection
cadence (initial detailed inspection, periodic close /
visual / sampling inspections) is enforced as a
machine invariant; lapsed cadence emits a programme-
level event so that the operations team can schedule
the inspection.

## §11 Incidents

```
POST   /v1/programmes/{pid}/incidents
                                   — register an
                                     incident
PATCH  /v1/incidents/{iid}/root-cause
                                   — record root-cause
                                     analysis narrative
PATCH  /v1/incidents/{iid}/ahj-notification
                                   — record AHJ
                                     notification
                                     reference
GET    /v1/incidents/{iid}         — retrieve incident
```

Incident submissions whose `severityClass` is
`personnel-injury` or `environmental-release` without
an `ahjNotifiedAt` field within the operating
jurisdiction's reporting deadline return `409` with
type
`urn:wia:fuel-cell:ahj-notification-required-for-severity`.

## §12 Errors, Authentication, Caching, Audit

Errors: `application/problem+json` per RFC 9457 with
the types named above plus
`urn:wia:fuel-cell:evidence-mismatch`. Authentication:
mutually-authenticated TLS for AHJ, grid-system-
operator, type-approval-authority, manufacturer, and
auditor consumers. Caching: stable resources (closed
incidents, accepted commissioning records, archived
programmes, retired stacks) cacheable with
`Cache-Control: max-age=31536000, immutable`. Audit
logs carry `programmeId`, `stackId`, `traceId`, the
issuing client certificate's subject, and the
deployment's clock skew vs the operating
jurisdiction's NTP service.

## §13 Streaming Subscription, Bulk, Pagination, Provenance

SSE at `/v1/programmes/{pid}/events` for programme-
wide events (fuel-quality verdict issued, IEEE 1547
test executed, IECEx inspection lapsed, incident
declared). Subscribers reconnect via `Last-Event-ID`.
Bulk endpoints: `/v1/bulk/stacks`, `/v1/bulk/fuel-
quality-records`, `/v1/bulk/periodic-inspections`.
Cursor-based pagination via `cursor` and `Link`
headers. Provenance via `/v1/provenance/{recordId}`
emits the in-toto attestation chain for any record.

## §14 Worked Example: Stationary PEMFC Commissioning

1. Manufacturer registers the programme with
   `applicationClass=stationary-power`,
   `fuelCellChemistry=pemfc`,
   `gridInterconnection=grid-paralleled-low-voltage`,
   `hazardousAreaClassification=iec-60079-zone-2`.
2. Stack registered via `POST /stacks` with the
   manufacturer's IEC 62282-2 module-test reference.
3. BoP registered via `POST /balance-of-plant` with
   IEEE 1547-2018 inverter conformance reference and
   IECEx Certificate-of-Conformity references for
   each Ex-zoned equipment item.
4. Pre-energising fuel-quality verification via
   `POST /fuel-quality-records` from an ISO/IEC 17025-
   accredited laboratory; verdict `conforming`.
5. Grid-interconnection test via `POST /grid-
   interconnections` with IEEE 1547.1-2020 test
   report and the grid system operator's
   interconnection agreement.
6. Commissioning recorded via `POST /commissioning-
   records` with AHJ acceptance and IEC 62282-3-300
   installation-inspection reference.
7. Periodic-inspection cadence runs per IEC 60079-17
   Ex-zone schedule and the manufacturer's
   recommended-maintenance schedule.

## §15 Performance Test Endpoint (IEC 62282-3-200)

```
POST   /v1/stacks/{sid}/performance-tests
                                   — register a
                                     performance test
                                     (commissioning
                                     and periodic re-
                                     test) per IEC
                                     62282-3-200
GET    /v1/stacks/{sid}/performance-tests
                                   — list performance
                                     tests
```

Performance-test submissions record rated power,
voltage regulation, response time, electrical
efficiency, and (where the BoP supports it) thermal
efficiency.

## §16 Aggregate and Provenance Endpoints

```
GET    /v1/provenance/{recordId}
GET    /v1/aggregate/operating-hours?period=...
GET    /v1/aggregate/incident-rate?period=...&kind=...
GET    /v1/aggregate/fuel-quality-non-conformance-rate?period=...
```

## §17 Conformance

A conformant server passes the test vectors published
under `tests/phase-vectors/phase-2-api-interface/`,
emits an OpenAPI 3.1 document, signs evidence packages
per RFC 9421, refuses operation under non-conforming
fuel quality, refuses zoned BoP without IECEx CoC for
each zoned equipment item, and refuses grid-
interconnection submissions without IEEE 1547 ride-
through category declarations.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-fuel-cell
- **Last Updated:** 2026-04-28
