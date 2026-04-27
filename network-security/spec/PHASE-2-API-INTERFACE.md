# WIA-network-security PHASE 2 — API Interface Specification

**Standard:** WIA-network-security
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a network-security
deployment exposes for asset inventory, security-event
ingestion, IOC management, threat-intelligence exchange
(TAXII-compatible), vulnerability tracking, incident
lifecycle, alert subscription, and OpenC2-aligned response
actions. The shape is HTTP/JSON; high-volume event ingestion
also offers a streaming binary projection in PHASE 3.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS)
- OASIS TAXII 2.1 — for STIX-aligned exchange
- OASIS OpenC2 — for response-action verbs
- OASIS Sigma — for correlation-rule references
- WIA-pq-crypto PHASE 2 — for transport-key handling

---

## §1 Asset inventory endpoints

```
POST /assets HTTP/1.1
Authorization: Bearer <jws-asset-owner-jwt>
Content-Type: application/json
```

Body is a PHASE 1 §2 inventory record. Successful intake
returns 201 Created; updates are PUTs against the asset URN.

```
GET /assets/{assetRef}
GET /assets?criticalityClass=tier-1&assetType=host
```

List responses paginate (Annex B). Asset-owner role tokens
mutate only their own assets; SOC roles read across
authorised scopes.

## §2 Security-event ingestion

```
POST /events HTTP/1.1
Authorization: Bearer <jws-detector-jwt>
Content-Type: application/json
```

Body is a single PHASE 1 §3 event or an array. The boundary
deduplicates on `(detectorRef, rawEventRef-hash)`; replays
return the original event URN with the existing ingestion
status.

For high-volume ingestion the streaming endpoint is
preferred:

```
POST /events/stream HTTP/1.1
Content-Type: application/x-ndjson
```

Each line is one event record. Acknowledgement is delivered
via a `Wia-Stream-Ack` trailer naming the last accepted
event URN; consumers retransmit unacknowledged tail events
on reconnect.

## §3 IOC management

```
POST /iocs HTTP/1.1
GET /iocs/{iocId}
GET /iocs?type=ipv4&validUntilAfter=…&tlpMarking=green
PUT /iocs/{iocId}/expiry  (analyst lifecycle action)
```

POST submits new IOCs (PHASE 1 §4); the boundary
canonicalises values per type before persisting (e.g.,
domain lower-cased, IPv6 short-form). IOCs without TLP
marking are refused.

```
DELETE /iocs/{iocId}
```

Soft-deletion: marks the IOC `withdrawn` and propagates
withdrawal to subscribed consumers via the webhook surface
(Annex F). Hard deletion is reserved for legal-hold
interactions and requires a regulator counter-signature.

## §4 Threat-intelligence exchange (TAXII-compatible)

The boundary exposes a TAXII-2.1 compatible surface for
STIX 2.1-aligned objects:

```
GET /taxii/collections HTTP/1.1
GET /taxii/collections/{collectionId}
GET /taxii/collections/{collectionId}/objects?added_after=…
POST /taxii/collections/{collectionId}/objects
```

Authorised partners consume and contribute STIX objects
within the collection's TLP marking. The boundary validates
STIX objects on intake (well-formed `id`, valid `type`,
present `created`/`modified`); invalid objects are returned
with RFC 9457 problem details.

## §5 Vulnerability tracking

```
POST /vulns HTTP/1.1
GET /vulns/{vulnId}
GET /vulns?productCpe=…&kev=true&remediationStatus=pending
```

POST accepts CSAF 2.0 documents directly; the boundary
extracts vulnerability records (PHASE 1 §6) and resolves
`affectedAssetCount` from the software-inventory join. The
resolution job runs asynchronously; an interim count of
`null` is returned until resolution completes.

```
PUT /vulns/{vulnId}/remediation
```

Updates `remediationStatus`. Setting `accepted-risk` requires
an attached signed risk-acceptance record from the asset
owner; the boundary refuses without it.

## §6 Incident lifecycle

```
POST /incidents HTTP/1.1
GET /incidents/{incidentId}
GET /incidents?state=triage&severity=critical
PUT /incidents/{incidentId}/state
PUT /incidents/{incidentId}/assign
```

POST opens an incident from an alert or analyst-initiated
case (PHASE 1 §7). State transitions are validated against
the ISO 27035 lifecycle order; out-of-order transitions
return `urn:wia:nsec:problem:state-transition-invalid`.

```
POST /incidents/{incidentId}/lessons-learned
```

Attaches a post-mortem document URI; the boundary records
the URI in the audit chain at incident closure.

## §7 Alert subscription

```
GET /alerts HTTP/1.1
Accept: text/event-stream
```

Live SSE stream of alerts within the analyst's authorised
scope. Filter with `?severity=…&analystState=…&since=…`;
filter changes do not restart the stream.

```
PUT /alerts/{alertId}/state
```

Updates `analystState`. Marking `false-positive` adds the
alert to a feedback queue used for correlation-rule tuning
(PHASE 4 §3).

## §8 Response actions (OpenC2-aligned)

```
POST /actions HTTP/1.1
```

Body is a PHASE 1 §9 response-action record. The boundary
verifies the analyst's role authorises the action type,
the target asset is within the analyst's scope, and the
action's blast radius does not exceed deployment-declared
limits without elevated approval.

```
GET /actions/{actionId}
PUT /actions/{actionId}/outcome
```

The outcome update is typically posted by the executing
control-plane component (firewall, EDR, IAM) once the action
completes. Failed actions trigger PHASE 4 §5 escalation.

## §9 Capability discovery

```
GET /.well-known/wia/network-security HTTP/1.1
```

Returns the capability document:

```json
{
  "wia.standardVersion": "1.0",
  "wia.implementationVersion": "soc-x-3.5.0",
  "supportedDetectors": ["edr-vendor-a","ndr-vendor-b","siem-vendor-c"],
  "stixVersion": "2.1",
  "taxiiVersion": "2.1",
  "openc2ProfileSupport": ["slpf","jadn-ad"],
  "tlpVersion": "2.0",
  "manifest": "https://soc-x.example/.well-known/wia/network-security/manifest.jws"
}
```

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Idempotency

Event POST and IOC POST accept `Idempotency-Key`. The
boundary stores keys for 24 hours. Replays return the
original response; conflicting bodies under the same key
return `urn:wia:nsec:problem:idempotency-conflict`.

## Annex B — Pagination

List endpoints support cursor pagination with cursors signed
by the boundary, valid for 30 minutes. Page envelopes
declare the snapshot epoch so consumers can detect drift.

## Annex C — Negative-test vectors (informative)

| Stimulus                                            | Expected response                              |
|-----------------------------------------------------|------------------------------------------------|
| Event with unknown detectorRef                      | 422 + `urn:wia:nsec:problem:detector-unknown`  |
| IOC without TLP marking                             | 422 + `urn:wia:nsec:problem:tlp-missing`       |
| TAXII POST of malformed STIX object                 | 422 + `urn:wia:nsec:problem:stix-invalid`      |
| Vuln remediation `accepted-risk` without acceptance record | 403 + `urn:wia:nsec:problem:risk-acceptance-missing` |
| Action by analyst whose scope excludes target asset | 403 + `urn:wia:nsec:problem:scope-denied`      |
| Out-of-order incident state transition              | 422 + `urn:wia:nsec:problem:state-transition-invalid` |

## Annex D — Bulk export

For analytical replay, bulk export endpoints serve NDJSON
over a date range:

```
GET /export/events?from=…&to=…
GET /export/incidents?from=…&to=…
```

Bulk export is gated by the deployment's bulk-quota policy
and audit-logged with `kind=bulk-export`. Exported records
honour their TLP markings; consumers must respect them on
re-disclosure.

## Annex E — Webhook subscriptions

Push subscriptions for downstream consumers (TIP feeders,
ticket systems, partner SIEMs):

```
POST /subscriptions HTTP/1.1

{
  "subscriptionId": "urn:wia:nsec:sub:tip-feeder:s-001",
  "callbackUrl": "https://tip.example/webhooks/wia",
  "eventClasses": ["alert-created","ioc-published","incident-state-changed"],
  "filters": {"severityAtLeast": "medium", "tlpMarkingAtMost": "amber"},
  "subscriberAuthorityRef": "urn:wia:auth:tip-feeder"
}
```

Webhook delivery uses TLS 1.3 with a detached JWS in a
`Wia-Signature` header. Failed deliveries retry on an
exponential schedule for up to 24 hours; persistent failure
escalates to operations.

## Annex F — Authorities and roles

| Role                  | Scope                                              |
|-----------------------|----------------------------------------------------|
| `asset-owner`         | inventory mutations on owned assets                |
| `detector`            | event ingestion within declared detector scope     |
| `analyst`             | event read, alert/incident write within assigned scope |
| `soc-lead`            | cross-shift assignment, incident escalation        |
| `tip-feeder`          | IOC and TIO contribution                           |
| `partner`             | TAXII consumption within agreed collections        |
| `regulator`           | counter-signatures, monitoring read                |

## Annex G — Worked OpenC2 action

A worked block-network-flow action against a credential-
stuffing source:

```json
{
  "actionId": "urn:wia:nsec:action:soc-x:a-2026-04-27-014",
  "incidentRef": "urn:wia:nsec:incident:soc-x:i-2026-04-27-001",
  "targetRef": "urn:wia:nsec:asset:soc-x:edge-fw-1",
  "actionType": "block-network-flow",
  "actionParameters": {
    "match": {"source_ipv4": "198.51.100.42/32"},
    "duration": "PT24H"
  },
  "executedBy": "urn:wia:auth:soc-x-analyst-3",
  "outcome": "pending"
}
```

The control-plane component (here, the edge firewall)
acknowledges and updates `outcome` to `succeeded` once the
rule is installed.

## Annex H — Capability versioning

Capability documents declare both `wia.standardVersion` and
`wia.implementationVersion`. A standard-version mismatch is
a hard refusal; an implementation-version mismatch is logged
but not refusing.

## Annex I — Worked TAXII collection layout

A typical deployment exposes at least three TAXII collections:

- `internal-iocs` — IOCs derived from the deployment's own
  detection; TLP:AMBER+STRICT by default
- `partner-share` — IOCs and TIOs shared by the deployment
  with named partners; TLP per agreement
- `inbound-feeds` — STIX objects ingested from upstream
  intelligence vendors; TLP per source

Each collection's manifest declares the contributing source,
the TLP marking, the access roster, and the marking change
policy. Marking changes propagate to subscribers; access
roster changes propagate to authentication tokens.

## Annex J — Audit-chain replay

For regulators and Anchored auditors, the boundary serves a
selective audit-chain replay:

```
GET /audit/chain?since=…&kind=incident-opened,risk-accepted
Accept: application/x-ndjson
```

Each event is one signed audit-chain entry. The endpoint
verifies the requesting authority's role allows access to
each requested kind; restricted kinds return only entries
under operations the requester is authorised for.

## Annex K — Detector calibration endpoints

The boundary tracks per-detector confidence calibration via:

```
GET /detectors/{detectorRef}/calibration
PUT /detectors/{detectorRef}/calibration  (SOC-lead only)
```

Calibration tracks observed precision against the detector's
declared confidence band over a rolling window. Persistent
deviation triggers a re-onboarding review under the PHASE 3
Annex G handshake.

## Annex L — Risk-acceptance record schema

For `accepted-risk` vulnerability remediation status, the
attached record schema:

```json
{
  "riskAcceptanceId": "urn:wia:nsec:risk-accept:asset-owner-x:r-2026-04-27-001",
  "vulnRef": "urn:wia:nsec:vuln:cve-2026-12345",
  "assetRefs": ["urn:wia:nsec:asset:asset-owner-x:host-3"],
  "rationale": "compensating control: network segmentation + monitoring",
  "compensatingControlRefs": ["urn:wia:nsec:control:c-001"],
  "validUntil": "2027-04-27",
  "ownerSignature": "<jws-detached>",
  "securityLeadCounterSignature": "<jws-detached>"
}
```

The record is itself audit-chained at `kind=risk-accepted`.
