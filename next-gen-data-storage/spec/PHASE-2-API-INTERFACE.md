# WIA-next-gen-data-storage PHASE 2 — API Interface Specification

**Standard:** WIA-next-gen-data-storage
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the resource-oriented API surface
for next-gen storage operations: storage-domain and
namespace management, volume / file-system / bucket
provisioning, performance-profile and QoS control,
durability and replication management, encryption-
envelope lifecycle, content-addressed artefact ingest
(S3 / OCI Distribution), tier-promotion / tier-
demotion, fault and health monitoring, and capacity /
performance reporting.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP), RFC 9112 (HTTP/1.1), RFC 9113 (HTTP/2)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 9530 (Content-Digest)
- DMTF Redfish 2024.x; SNIA Swordfish 1.2.6; SNIA SMI-S 1.8
- NVM Express Base Specification 2.0; NVMe-MI 1.2; NVMe-oF 1.1
- AWS S3 API; S3 Object Lock; S3 Multi-Region Access Points
- OCI Distribution Specification 1.1; OCI Image Specification 1.1
- DMTF Redfish OEM Extensions registry
- IEEE 802.1Qbv / Qbu (Time-Sensitive Networking, where TSN-bound)

---

## §1 Endpoint root

API root is implementation-controlled. All endpoints
are TLS 1.3 (RFC 8446). Management endpoints
(Swordfish / Redfish) follow the DMTF base API. S3
endpoints follow the de-facto S3 contract; OCI
Distribution endpoints follow the OCI Distribution
specification.

## §2 Storage-domain endpoints

```
POST   /v1/domains                          register storage domain
GET    /v1/domains/{ref}                    retrieve
PATCH  /v1/domains/{ref}                    amend (admin)
GET    /v1/domains?topology=&media=         list / filter
GET    /v1/domains/{ref}/health             health-check status
```

Health-check responses include per-component status,
firmware version, and the last fault event.

## §3 Namespace endpoints

```
POST   /v1/namespaces                       create namespace
GET    /v1/namespaces/{ref}                 retrieve
PATCH  /v1/namespaces/{ref}                 amend (capacity / quota)
DELETE /v1/namespaces/{ref}                 release (with WORM gate)
GET    /v1/namespaces?domain=&kind=         list / filter
POST   /v1/namespaces/{ref}/zns/zone-actions  ZNS zone management
                                              (open / close / finish / reset)
```

ZNS zone-actions follow the NVMe ZNS Command Set
semantics (TP 4053a).

## §4 Volume / file-system / bucket endpoints

```
POST   /v1/volumes                          create volume (block / fs / bucket)
GET    /v1/volumes/{ref}                    retrieve
PATCH  /v1/volumes/{ref}                    amend (capacity / quota)
DELETE /v1/volumes/{ref}                    release
POST   /v1/volumes/{ref}/snapshot           create snapshot
POST   /v1/volumes/{ref}/clone              clone from snapshot
```

Object-bucket operations follow the S3 API in addition
to this resource-oriented surface.

## §5 Performance-profile endpoints

```
POST   /v1/performance-profiles             create profile
GET    /v1/performance-profiles/{ref}       retrieve
POST   /v1/namespaces/{ref}/profile         apply profile
GET    /v1/namespaces/{ref}/qos-status      QoS adherence summary
```

QoS adherence reports per-percentile actual versus
target; sustained miss raises a stewardship task.

## §6 Durability and replication endpoints

```
POST   /v1/replication-topologies           define topology
PATCH  /v1/replication-topologies/{ref}     amend members
POST   /v1/replication-topologies/{ref}/$test  trigger consistency test
GET    /v1/replication-topologies/{ref}/lag site-to-site replication lag
```

Replication-lag reports per-site lag in bytes / time;
exceeding the RPO budget emits a critical alert.

## §7 Encryption-envelope endpoints

```
POST   /v1/encryption-envelopes              register envelope
PATCH  /v1/encryption-envelopes/{ref}/rotate trigger key rotation
GET    /v1/encryption-envelopes/{ref}        retrieve (gated)
POST   /v1/encryption-envelopes/{ref}/wrap   wrap a DEK
POST   /v1/encryption-envelopes/{ref}/unwrap unwrap a DEK (audited)
```

Wrap / unwrap operations sign with the requesting
service's authentication; unwrap operations record the
audit-event and the requesting service identity.

## §8 Content-addressed artefact endpoints (S3 / OCI)

S3-compatible (per AWS S3):

```
GET    /{bucket}/                            list objects
PUT    /{bucket}/{key}                       put object
GET    /{bucket}/{key}                       get object
DELETE /{bucket}/{key}                       delete object
POST   /{bucket}/?uploads                    multipart upload init
PUT    /{bucket}/{key}?partNumber=N&uploadId=  upload part
POST   /{bucket}/{key}?uploadId=             complete multipart upload
```

OCI Distribution (per OCI Distribution v1.1):

```
GET    /v2/                                  Discovery
GET    /v2/{name}/manifests/{reference}      pull manifest
PUT    /v2/{name}/manifests/{reference}      push manifest
GET    /v2/{name}/blobs/{digest}             pull blob
POST   /v2/{name}/blobs/uploads/             initiate blob upload
PATCH  /v2/{name}/blobs/uploads/{uuid}       chunked upload
PUT    /v2/{name}/blobs/uploads/{uuid}       complete upload
```

## §9 Tier-promotion / tier-demotion endpoints

```
POST   /v1/objects/{ref}/promote-to-tier     move artefact to faster tier
POST   /v1/objects/{ref}/demote-to-tier      move to slower tier
GET    /v1/objects/{ref}/tier                current tier
```

Tier transitions emit cost / performance impact
estimates. Sponsor-policy may auto-tier based on
access-pattern telemetry.

## §10 Fault / health monitoring endpoints

```
GET    /v1/health                            domain-wide health summary
GET    /v1/faults                            recent faults
POST   /v1/faults/{ref}/acknowledge          acknowledge fault
GET    /v1/faults?severity=&since=           list faults
```

Faults follow the SNIA Swordfish / Redfish fault
taxonomy (informational, warning, critical, fatal).

## §11 Capacity / performance report endpoints

```
GET    /v1/reports/capacity                  per-domain capacity report
GET    /v1/reports/performance               per-namespace performance report
POST   /v1/reports/$run                      run ad-hoc report
GET    /v1/reports/{ref}                     retrieve report
```

## §12 Error model (RFC 9457)

```json
{
  "type":   "urn:wia:storage:problem:worm-locked",
  "title":  "Object is WORM-locked",
  "status": 423,
  "detail": "Object o-2026-001 retained until 2030-12-31",
  "instance": "/v1/objects/o-2026-001"
}
```

Common type URIs:

| Type URI suffix              | HTTP | Meaning                                       |
|------------------------------|-----:|-----------------------------------------------|
| `worm-locked`                | 423  | object retention policy disallows mutation    |
| `quota-exceeded`             | 507  | namespace quota reached                        |
| `replication-lag-exceeded`   | 507  | sustained replication lag above policy        |
| `qos-violation`              | 503  | service degraded; QoS target unmet             |
| `kek-unavailable`            | 503  | KMS unreachable                                |
| `firmware-mismatch`          | 409  | controller firmware mismatch in domain         |
| `media-unavailable`          | 503  | back-end media offline                         |

## §13 Bulk export

```
GET  /v1/$export?_type=Domain,Namespace,Volume
GET  /v1/$status/{exportId}
GET  /v1/$result/{exportId}/{file}
```

NDJSON output. Storage reports may also export to a
sponsor-internal data lake via the change-event
stream.

## §14 Audit headers

| Header                  | Meaning                                       |
|-------------------------|-----------------------------------------------|
| `X-Request-Id`          | client-set, echoed                            |
| `X-Audit-Event-Id`      | server-set, links to PHASE 3 audit chain      |
| `Content-Digest`        | RFC 9530 SHA-256 of the response body         |

## Annex A — OpenAPI reference

A canonical OpenAPI 3.1 description is published at
`api/openapi-3.1.yaml`.

## Annex B — Worked Swordfish provisioning (informative)

```http
POST /v1/namespaces HTTP/1.1
Authorization: Bearer ...
Content-Type: application/json

{
  "domainRef": "domain-pod-A",
  "namespaceKind": "nvme-namespace",
  "capacityBytes": 1099511627776,
  "lbaSize": 4096,
  "performanceProfileRef": "perf-tier-0"
}
```

Response 201 returns the namespace identifier and the
NVMe-CNTLID binding for the controller.

## Annex C — Webhook surface

Implementations expose webhooks for `fault-raised`,
`replication-lag-exceeded`, `quota-warning`, and
`encryption-key-rotated` events. Payloads sign with
RFC 7515 JWS; receivers verify against
`/.well-known/wia-storage-keys.json`. Delivery is at-
least-once; receivers are expected to be idempotent on
`eventId`.

## Annex D — Conformance disclosure

Implementations declare the management protocols
served, the NVMe / NVMe-oF revisions, the S3 API
version, the OCI Distribution version, the encryption
ciphers supported, and the durability tiers offered.

## Annex E — Async export pattern

```
POST   /v1/$export                      → 202 Accepted, Content-Location
GET    /v1/$status/{id}                 → 202 in-progress / 200 manifest
GET    /v1/$result/{id}/{file}          → 200 NDJSON
DELETE /v1/$status/{id}                 → 202 cancellation
```

The 202 response carries `Retry-After` for the
polling client.

## Annex F — Discovery service

```
GET   /v1/discovery                            top-level discovery endpoint
GET   /v1/discovery/nvme-oF                    NVMe-oF discovery service
GET   /v1/discovery/s3-endpoints               S3 endpoint inventory
GET   /v1/discovery/oci-distribution           OCI Distribution origin set
GET   /.well-known/wia-storage                 sponsor-published discovery doc
```

Discovery responses include the supported management
protocols, the per-protocol endpoint URI, and the
supported feature flags so a client provisions
without bilateral configuration.

## Annex G — Long-running operations

Some storage operations (large rebuild, bulk fixity
audit, capacity expansion) are long-running. The
implementation exposes them via the async export
pattern (Annex E) plus a per-operation status:

```
POST   /v1/operations                          start operation
GET    /v1/operations/{ref}                    retrieve state + progress
DELETE /v1/operations/{ref}                    cancel (where permissible)
GET    /v1/operations?type=&status=            list operations
```

Operation events emit to the audit chain so a post-
hoc inspection can correlate operations to outcomes.

Long-running operations also publish progress to
webhook receivers (Annex C) so dashboards reflect
real-time progress without polling.
