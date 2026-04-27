# WIA-mineral-mining PHASE 2 — API Interface Specification

**Standard:** WIA-mineral-mining
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a mining operation
exposes for asset registry, exploration data submission,
resource and reserve filings, production publication,
assay intake, environmental sample publication, worker-
exposure reporting, and supply-chain handover. The shape
is HTTP/JSON.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS)
- CRIRSCO International Reporting Template
- ISO 17025 — laboratory accreditation reference
- OECD DDG — handover status framework
- WIA-supply-chain PHASE 2 — for handover delegation

---

## §1 Asset registry endpoints

```
POST /assets HTTP/1.1
Authorization: Bearer <jws-operator-jwt>
Content-Type: application/json
```

Body is a PHASE 1 §2 asset registry record. Successful
intake returns 201 Created with the boundary's URN binding.
Updates use PUT against the asset URN; closure-bond
mutations require an additional regulator counter-signature
delivered via Annex C.

Lookups:

```
GET /assets/{assetRef}
GET /assets?operator={operatorRef}
GET /assets?commodity=Cu
```

List responses paginate (Annex B); per-asset detail responses
include the most recent registry version plus a link to the
audit-chain history.

## §2 Exploration data submission

```
POST /exploration HTTP/1.1
```

Body is a PHASE 1 §3 exploration record. Successful intake
returns 202 Accepted with the boundary's URN binding and a
queued status indicator; full processing (e.g., lab QA/QC
linkage to assay records) completes asynchronously.

```
GET /exploration/{explorationId}
```

Returns the status: `received`, `qaqc-pending`, `validated`,
`linked`, `published`, `withdrawn`. A `withdrawn` event
requires a competent-person signature explaining the
withdrawal.

## §3 Resource and reserve filing

Filings under a CRIRSCO-family code:

```
POST /estimates HTTP/1.1
```

Body is a PHASE 1 §4 estimate record. The boundary verifies:

- `code` matches a recognised CRIRSCO-family code
- `competentPersonRef` matches a competent person registered
  for the operation
- `effectiveDate` is in the past or current
- `reportRef` is reachable

Refusal responses use RFC 9457 Problem Details:

| problem URN                                  | meaning                              |
|----------------------------------------------|--------------------------------------|
| `urn:wia:mm:problem:code-unknown`            | classification code not recognised   |
| `urn:wia:mm:problem:cp-unknown`              | competent person not registered      |
| `urn:wia:mm:problem:effective-date-future`   | effective date in the future         |
| `urn:wia:mm:problem:report-unreachable`      | reportRef returns 404 or non-200     |
| `urn:wia:mm:problem:reserve-without-resource`| reserve filing without backing resource |

```
GET /estimates/{estimateId}
GET /estimates?assetRef=…&code=JORC
```

## §4 Production publication

```
POST /production HTTP/1.1
```

Body is a PHASE 1 §5 production record. Successful intake
returns 202 Accepted; gate-of-mine reconciliation (PHASE 4
§6) runs asynchronously.

```
GET /production?assetRef=…&shiftStart=…&shiftEnd=…
```

List paginates and supports CSV/NDJSON output for analytics
consumers.

## §5 Assay intake

Laboratories or operator's lab gateway submit assays:

```
POST /assays HTTP/1.1
Authorization: Bearer <jws-laboratory-jwt>
```

Body is a PHASE 1 §6 assay record. The boundary verifies the
laboratory's ISO 17025 scope covers the analyte/method
combination declared. Out-of-scope analytes are refused.

QA/QC linkage is enforced asynchronously: the boundary
reconciles each assay against its declared QA/QC packages
within a deployment-declared window (typically 30 days).
Failures appear in the lab's reconciliation report and
escalate to the operator.

## §6 Environmental sample publication

```
POST /environmental HTTP/1.1
```

Body is a PHASE 1 §7 environmental sample record. The
boundary computes the exceedance flag against the cited
regulatory threshold; an exceedance triggers PHASE 4 §7
incident workflow within the deployment-declared latency
budget.

```
GET /environmental?assetRef=…&mediaType=surface-water&since=…
```

For exceedance events, the response includes the open
incident URN if one has been raised.

## §7 Worker-exposure reporting

```
POST /exposure HTTP/1.1
```

Body is a PHASE 1 §8 worker-exposure record. The boundary
enforces the privacy contract: worker identity is held only
as a pseudonym in this surface; binding to the underlying
HR identity is in a separate, access-controlled service per
PHASE 3 §9.

```
GET /exposure?assetRef=…&workerRef=…&shiftRef=…
```

Restricted to authorised health-and-safety personnel; ordinary
operator roles see aggregate-only views.

## §8 Supply-chain handover

```
POST /handover HTTP/1.1
```

Body is a PHASE 1 §9 handover record. The boundary verifies:

- `fromAssetRef` exists
- `oecdDdgStatus` is one of `green`, `red`, `requires-additional-action`
- For `red` operations, the handover is refused
- For `requires-additional-action` operations, the handover
  succeeds but is flagged for downstream review

The successful response includes the URN that the receiving
custody chain references back to verify origin.

## §9 Capability discovery

```
GET /.well-known/wia/mineral-mining HTTP/1.1
```

Returns the deployment's capability document:

```json
{
  "wia.standardVersion": "1.0",
  "wia.implementationVersion": "operator-x-2.1.0",
  "supportedCommodities": ["Cu","Au","Zn"],
  "supportedAssetTypes": ["open-pit","underground-mine","tailings-storage"],
  "supportedCodes": ["JORC","SAMREC"],
  "isoCertifications": ["ISO-14001","ISO-45001"],
  "manifest": "https://operator-x.example/.well-known/wia/mineral-mining/manifest.jws"
}
```

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Idempotency

Production POST and assay POST accept `Idempotency-Key`.
Boundary stores keys for 30 days (long enough to cover
typical lab QA/QC reconciliation windows). Replays return
the original response.

## Annex B — Pagination

List endpoints support cursor pagination with cursors signed
by the boundary, valid for 30 minutes. Cursor envelopes
declare the snapshot epoch so consumers can detect drift.

## Annex C — Regulator counter-signature

Closure-bond mutations and other regulator-witnessed events
require a regulator counter-signature delivered via:

```
POST /regulator-witness HTTP/1.1
Authorization: Bearer <jws-regulator-jwt>
```

Body is a JWS detached signature over the mutation it
endorses. The boundary verifies the regulator's authority
URN against the asset's `coordinatingAuthority` and applies
the mutation only after successful verification.

## Annex D — Negative-test vectors (informative)

| Stimulus                                            | Expected response                              |
|-----------------------------------------------------|------------------------------------------------|
| Estimate with unknown competent person              | 422 + `urn:wia:mm:problem:cp-unknown`          |
| Reserve filing without backing resource             | 422 + `urn:wia:mm:problem:reserve-without-resource` |
| Assay submission outside lab ISO 17025 scope        | 422 + `urn:wia:mm:problem:lab-scope-mismatch`  |
| Handover with `red` DDG status                      | 403 + `urn:wia:mm:problem:ddg-red-handover`    |
| Worker-exposure write by non-HSE role               | 403 + `urn:wia:mm:problem:role-denied`         |

## Annex E — Bulk export

For analytical replay or audit, bulk export endpoints serve
NDJSON over a date range:

```
GET /export/production?from=…&to=…&assetRef=…
GET /export/assays?from=…&to=…
```

Bulk export is gated by the deployment's bulk-quota policy
and audit-logged with `kind=bulk-export`.

## Annex F — Webhook subscriptions

For consumers requiring pushed updates (downstream supply-
chain receivers, regulator monitoring), webhook subscriptions
are accepted at `/subscriptions`. Webhook delivery uses TLS
1.3 with detached JWS in a `Wia-Signature` header. Failed
deliveries retry on an exponential schedule for up to 24
hours; persistent failure escalates to operations.

## Annex G — Reconciliation reports

Gate-of-mine reconciliation reports are served at:

```
GET /reconciliation?assetRef=…&period=2026-Q1
```

The report compares mined tonnes/grade against the
contributing resource estimate, classifies the variance,
and lists material-level explanations. Reconciliation is
routinely audited under Anchored conformance.

## Annex H — Authorities and roles

| Role                | Scope                                                |
|---------------------|------------------------------------------------------|
| `operator`          | full read/write on owned assets                      |
| `laboratory`        | assay intake on accredited scopes                    |
| `competent-person`  | exploration & estimate sign-off                      |
| `regulator`         | counter-signatures, monitoring read                  |
| `hse-officer`       | exposure & environmental write                       |
| `auditor`           | read-only across assets within engagement scope      |

## Annex I — Worked production publication

A worked example for a daily production claim:

```
POST /production HTTP/1.1
Authorization: Bearer <jws-operator-jwt>
Content-Type: application/json
Idempotency-Key: 2026-04-27-day-shift-north-pit-bench-3

{
  "productionId": "urn:wia:mm:production:operator-x:p-2026-04-27-d-001",
  "assetRef": "urn:wia:mm:asset:operator-x:north-pit",
  "shiftStart": "2026-04-27T07:00:00+09:00",
  "shiftEnd": "2026-04-27T19:00:00+09:00",
  "locationRef": "urn:wia:mm:location:operator-x:north-pit:bench-3",
  "tonnesMined": {"value": 14820.0, "unit": "t", "uncert": 75.0},
  "gradeSampleRefs": [
    "urn:wia:mm:sample:operator-x:s-2026-04-27-001",
    "urn:wia:mm:sample:operator-x:s-2026-04-27-002"
  ],
  "commodityProduced": ["Cu"],
  "equipmentRefs": [
    "urn:wia:mm:equip:operator-x:truck-12",
    "urn:wia:mm:equip:operator-x:loader-3"
  ],
  "dilutionFactor": 1.04,
  "recoveryFactor": 0.96,
  "signatures": [/* foreman + survey JWS detached */]
}
```

The boundary returns 202 Accepted with the queued URN; the
gate-of-mine reconciliation job (PHASE 4 §6) runs against
this record asynchronously.

## Annex J — Time-series export endpoints

Time-series data (continuous environmental monitoring,
equipment telemetry) is served via a streaming endpoint:

```
GET /timeseries/environmental?assetRef=…&from=…&to=…&analyte=…
Accept: text/event-stream
```

Each event is a single sample with timestamp, value,
quality flag, and station URN. Stream resumption is
supported via opaque resume tokens.

## Annex K — Capability versioning

Capability documents declare both `wia.standardVersion` and
`wia.implementationVersion` so that partners and regulators
can verify compatibility independently of vendor build
numbers. A standard-version mismatch is a hard refusal; an
implementation-version mismatch is logged but not refusing.

## Annex L — Audit-chain replay endpoint

For regulators and Anchored auditors, the boundary serves a
selective audit-chain replay:

```
GET /audit/chain?since=…&kind=incident-opened,exceedance-flagged
Accept: application/x-ndjson
```

Each event is one signed audit-chain entry. The endpoint
verifies the requesting authority's role allows access to
each requested kind; restricted kinds return only entries
under operations the requester is authorised for.

## Annex M — Operation closure handover

When an operation reaches end-of-life and enters closure:

```
POST /closure HTTP/1.1

{
  "closureId": "urn:wia:mm:closure:operator-x:c-2026-q3",
  "assetRef": "urn:wia:mm:asset:operator-x:north-pit",
  "closureBondRef": "urn:wia:mm:bond:closure:b-001",
  "closurePlanRef": "urn:wia:mm:plan:closure:p-001",
  "regulatorCounterSignature": "<jws>"
}
```

Closure events propagate to handover registers, supply-chain
receivers, and the deployment's attestation surface.
