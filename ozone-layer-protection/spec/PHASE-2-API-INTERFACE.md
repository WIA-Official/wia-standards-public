# WIA-ozone-layer-protection PHASE 2 — API Interface Specification

**Standard:** WIA-ozone-layer-protection
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface an ozone-layer-protection
deployment exposes for substance-roster lookup, party-reporting
intake, consumption / production / import / export aggregation,
phase-out compliance scoring, atmospheric-observation ingestion,
EESC trajectory queries, exemption management, illegal-trade
case tracking, refrigerant-life-cycle event publication, and
Article 7 submission to UNEP Ozone Secretariat.

References (CITATION-POLICY ALLOW only):
- UNEP Ozone Secretariat Article 7 reporting forms (XLS/CSV templates)
- WMO GAW data exchange formats
- WOUDC (World Ozone and UV Radiation Data Centre) submission protocol
- NetCDF Climate and Forecast Conventions v1.10
- OGC Sensor Observation Service (SOS) 2.0
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS)

---

## §1 Substance-roster lookup

```
GET /substances/{substanceRef} HTTP/1.1
GET /substances?annex=Annex-C-Group-I&controlled-since=2013
GET /substances?cas=75-69-4
GET /substances?ashrae34=R-22
```

Returns substance metadata per PHASE 1 §2.

```
GET /substances/$schedule?substanceRef=...&partyStatus=article-5
```

Returns the phase-out schedule that applies to the
substance × party-status combination per PHASE 1 §5.

## §2 Production / import / export reporting

```
POST /transactions HTTP/1.1
Authorization: Bearer <NOU-jwt>
Content-Type: application/json
```

Body is a PHASE 1 §3 transaction record. The boundary
verifies the NOU's authority for the declared party,
the substance × year is in the controlling schedule,
and the verification reference exists.

```
GET /transactions?partyRef=...&reportingYear=2025
GET /transactions/{transactionId}
PUT /transactions/{transactionId}/state    (correction within reporting window)
```

Bulk submission for annual filing:

```
POST /transactions/$bulk-submit HTTP/1.1
Content-Type: application/x-ndjson
```

Each line is a PHASE 1 §3 transaction. The boundary
processes in order, returns per-line acknowledgements,
and groups by (party, year) for downstream aggregation.

## §3 Consumption aggregation

```
GET /consumption?partyRef=KOR&reportingYear=2025&substanceClass=Annex-F-Group-I
```

Returns the PHASE 1 §4 consumption record computed from
underlying transactions. The boundary recomputes on demand
to absorb any post-deadline corrections.

```
GET /consumption/$schedule-compliance?partyRef=KOR&reportingYear=2025
```

Returns per-substance-class compliance status against the
phase-out schedule, flagging any over-target consumption.

## §4 Atmospheric-observation ingestion

```
POST /observations HTTP/1.1
Authorization: Bearer <station-jwt>
Content-Type: application/json
```

Body is a PHASE 1 §6 observation record. For high-volume
satellite ingest:

```
POST /observations/$bulk HTTP/1.1
Content-Type: application/x-ndjson
```

Or NetCDF file ingest:

```
POST /observations/$netcdf HTTP/1.1
Content-Type: application/x-netcdf
```

The boundary parses NetCDF per CF-1.10 conventions and
emits per-pixel observation records.

```
GET /observations?stationRef=...&from=...&to=...
GET /observations/$gridded?bbox=...&date=2026-04-28
```

## §5 EESC trajectory queries

```
GET /eesc/{eescId}
GET /eesc?latitudeBand=mid-latitude-NH&referenceYear=2025
```

Returns the PHASE 1 §7 EESC trajectory.

```
GET /eesc/$recovery-projection?latitudeBand=polar-SH
```

Returns the projected recovery year per the most recent
WMO Scientific Assessment.

## §6 Critical-use / essential-use exemption management

```
POST /exemptions HTTP/1.1
Authorization: Bearer <party-government-jwt>
```

Body is a PHASE 1 §8 exemption record. The boundary verifies:

- The party's eligibility per the relevant Article (2A-2I)
- The MOP decision reference exists in the deployment's
  decision roster
- Annual use is within the authorised quantity

```
GET /exemptions?partyRef=...&substanceRef=urn:wia:ozone:substance:Annex-E-MB
PUT /exemptions/{exemptionId}/actual-use
GET /exemptions/$audit-trail?exemptionId=...
```

## §7 Illegal-trade case tracking

```
POST /illegal-trade HTTP/1.1
GET /illegal-trade?partyRef=KOR&from=2024
PUT /illegal-trade/{caseId}/state
POST /illegal-trade/{caseId}/interpol-notice
```

Cross-coalition information exchange for inter-party
co-operation:

```
POST /illegal-trade/{caseId}/cross-party-share HTTP/1.1
{
  "sharingPartyRefs": ["urn:wia:ozone:party:JPN", "urn:wia:ozone:party:CHN"],
  "sharedFields": ["substanceRef", "quantityKg", "seizureLocation"],
  "rationale": "iPIC consultation request"
}
```

Sharing follows the iPIC informal Prior Informed Consent
network protocol.

## §8 Refrigerant-life-cycle publication

```
POST /service-events HTTP/1.1
Authorization: Bearer <technician-jwt>
```

Body is a PHASE 1 §10 service-event record.

```
GET /service-events?equipmentRef=...
GET /equipment/{equipmentRef}/refrigerant-history
GET /equipment/{equipmentRef}/leak-test-history
```

## §9 Article 7 submission to UNEP

```
POST /article-7-submissions HTTP/1.1
Authorization: Bearer <ozone-secretariat-jwt>
Content-Type: application/json

{
  "submissionId": "urn:wia:ozone:a7:KOR:2025",
  "partyRef": "urn:wia:ozone:party:KOR",
  "reportingYear": 2025,
  "transactionRefs": [...all production / import / export records...],
  "consumptionRefs": [...all consumption calculations...],
  "exemptionRefs": [...active exemptions...],
  "compiledAt": "2026-04-28T00:00:00+09:00",
  "compiledBy": "urn:wia:ozone:nou:KOR-officer-12",
  "submitterSignature": "<jws-detached>"
}
```

The boundary validates the submission against the controlling
schedule, computes per-substance compliance status, and
forwards to the UNEP Ozone Secretariat via the agreed
exchange channel.

## §10 Capability discovery

```
GET /.well-known/wia/ozone-layer-protection HTTP/1.1
```

Returns the capability document:

```json
{
  "wia.standardVersion": "1.0",
  "wia.implementationVersion": "kr-nou-2.4.1",
  "partyStatus": "non-article-5",
  "supportedScopes": ["a7-reporting", "atmospheric-observation", "refrigerant-lifecycle", "illegal-trade"],
  "ozoneSecretariatLink": "urn:wia:ozone:authority:UNEP-OS",
  "manifest": "https://kr-nou.example/.well-known/wia/ozone-layer-protection/manifest.jws"
}
```

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Idempotency

Write endpoints accept `Idempotency-Key`. Bulk submissions
include per-line idempotency keys for fine-grained replay.

## Annex B — Pagination

List endpoints support cursor pagination. Cursors are signed
by the boundary, valid for 30 minutes.

## Annex C — Negative-test vectors (informative)

| Stimulus                                              | Expected response                            |
|-------------------------------------------------------|----------------------------------------------|
| Transaction POST referencing unknown substance         | 422 + `substance-not-recognised`             |
| Transaction with year before substance was scheduled   | 422 + `substance-not-controlled-this-year`   |
| Consumption exceeding phase-out target                 | accepted; flags `over-target`                |
| Exemption POST without MOP decision reference          | 422 + `mop-decision-missing`                 |
| Article 7 submission with missing substance categories | 422 + `submission-incomplete`                |
| Service-event by uncertified technician                | 403 + `technician-not-certified`             |

## Annex D — Webhook subscriptions

Event classes: `transaction-published`,
`consumption-recalculated`, `compliance-flag-raised`,
`exemption-state-changed`, `illegal-trade-case-opened`,
`atmospheric-observation-anomaly`,
`a7-submission-acknowledged`.

## Annex E — Authorities and roles

| Role                          | Scope                                           |
|-------------------------------|-------------------------------------------------|
| `nou-officer`                 | NOU read/write within party scope               |
| `customs-officer`             | import / export verification                    |
| `gaw-station-operator`        | atmospheric-observation publication             |
| `satellite-operator`          | satellite-observation publication                |
| `certified-technician`        | refrigerant service-event publication            |
| `secretariat-officer`         | Article 7 receiving                             |
| `auditor`                     | read-only across engagement scope               |

## Annex F — Worked atmospheric-observation ingestion (informative)

```json
{
  "observationId": "urn:wia:ozone:obs:gaw-Boulder-OD:2026-04-28T18:00:00",
  "stationRef": "urn:wia:ozone:station:gaw:BLD-Dobson",
  "observationKind": "total-column-ozone",
  "instrument": "dobson-spectrophotometer",
  "observationTimestamp": "2026-04-28T18:00:00+00:00",
  "latitude": 40.0167,
  "longitude": -105.2667,
  "altitude": 1655,
  "value": 295,
  "unit": "DU",
  "uncertainty": 4.0,
  "qualityFlag": "validated"
}
```

## Annex G — Capability versioning

`wia.standardVersion` + `wia.implementationVersion` declared.
Standard-version mismatch is a hard refusal.

## Annex H — Audit-chain replay

```
GET /audit/chain?since=...&kind=transaction-published,exemption-state-changed
Accept: application/x-ndjson
```

## Annex I — Bulk-export for cross-party analytics

```
GET /export/transactions?reportingYear=2025&aggregation=per-substance-class
```

Aggregate exports honour party-side data-publication policies;
party-confidential transaction-level details are NOT exported
without explicit cross-party agreement.

## Annex J — Per-region recovery dashboard

```
GET /recovery-dashboard?latitudeBand=polar-SH
```

Returns aggregated EESC trajectory + projected recovery year
per the most recent WMO Scientific Assessment.
