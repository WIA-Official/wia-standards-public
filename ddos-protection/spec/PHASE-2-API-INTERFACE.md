# WIA-ddos-protection PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-ddos-protection
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that a
DDoS-protection operator exposes for the records
defined in PHASE-1. Two complementary surfaces are
described: the DOTS messaging surface (RFC 9132
signal channel + RFC 8783 data channel) — the
canonical wire format for upstream mitigation
requests; and the HTTPS / JSON RESTful surface for
operational visibility, the operator's compliance-
and-audit functions, and the supervisory or CERT
examination scope.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9132 (DOTS Signal Channel; obsoletes
  RFC 8782)
- IETF RFC 8783 (DOTS Data Channel)
- IETF RFC 8973 (DOTS Architecture)
- IETF RFC 8612 (DOTS Requirements)
- IETF RFC 8955 (BGP FlowSpec)
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web
  Linking), RFC 8259 (JSON), RFC 9421 (HTTP Message
  Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022, ISO/IEC 27035-1/-2/-3
- W3C Trace Context

---

## §1 Scope and Versioning

The operator exposes:

- The DOTS signal channel endpoint (CoAP / CoAPs
  per RFC 9132 — UDP port 4646 by default with
  DTLS).
- The DOTS data channel endpoint (RESTCONF over
  HTTPS per RFC 8783).
- The HTTPS / JSON RESTful surface served from a
  domain published by the operator under `/v1/`.

The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical for the JSON surface; the operator's DOTS
capability declaration is canonical for the DOTS
endpoints.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-ddos-protection",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "protectedServices":       "/v1/protected-services",
    "trafficBaselines":        "/v1/traffic-baselines",
    "attackDetections":        "/v1/attack-detections",
    "dotsExchanges":           "/v1/dots-exchanges",
    "mitigationActions":       "/v1/mitigation-actions",
    "postIncidentAnalysis":    "/v1/post-incident-analysis",
    "bcpAttestations":         "/v1/bcp-attestations",
    "cooperationRecords":      "/v1/cooperation-records",
    "examination":             "/v1/examination",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 DOTS Signal-Channel Surface (RFC 9132)

The DOTS signal-channel uses CoAP over DTLS. The
canonical resource path is:

```
POST /.well-known/dots/mitigate
GET  /.well-known/dots/mitigate/{mid}
DELETE /.well-known/dots/mitigate/{mid}
PUT  /.well-known/dots/config
```

The mitigation-request payload carries:

- `cuid` (Client Unique Identifier).
- `mid` (Mitigation Identifier).
- `target-prefix` (the IPv4 / IPv6 prefixes
  requesting mitigation).
- `target-port-range` (optional port-range
  scoping).
- `target-protocol` (optional transport-protocol
  scoping).
- `lifetime` (mitigation lifetime in seconds).

The DOTS server responds with the mitigation-status
report carrying `mid`, `lifetime`, `mitigation-
start`, `mitigation-status` (per RFC 9132 §4.4.1
status enumeration).

## §4 DOTS Data-Channel Surface (RFC 8783)

The DOTS data-channel uses RESTCONF over HTTPS. The
canonical resource path is:

```
GET  /restconf/data/ietf-dots-data-channel:dots-data
POST /restconf/data/ietf-dots-data-channel:dots-data/dots-client/{cuid}/aliases
PUT  /restconf/data/ietf-dots-data-channel:dots-data/dots-client/{cuid}/aliases/{name}
DELETE /restconf/data/ietf-dots-data-channel:dots-data/dots-client/{cuid}/aliases/{name}
POST /restconf/data/ietf-dots-data-channel:dots-data/dots-client/{cuid}/acls
PUT  /restconf/data/ietf-dots-data-channel:dots-data/dots-client/{cuid}/acls/{name}
DELETE /restconf/data/ietf-dots-data-channel:dots-data/dots-client/{cuid}/acls/{name}
```

The data-channel maintains the long-lived
configuration that the signal-channel references at
attack time (target alias definitions, baseline
ACL filters).

## §5 Protected-Service and Baseline Endpoints

```
GET    /v1/protected-services
GET    /v1/protected-services/{serviceId}
POST   /v1/protected-services
PATCH  /v1/protected-services/{serviceId}
GET    /v1/traffic-baselines?service={serviceId}
POST   /v1/traffic-baselines
GET    /v1/traffic-baselines/{baselineId}
```

## §6 Attack-Detection and Mitigation Endpoints

```
GET    /v1/attack-detections?service={serviceId}&from={iso}&to={iso}
GET    /v1/attack-detections/{detectionId}
POST   /v1/attack-detections
POST   /v1/mitigation-actions      (engage a
                                    mitigation —
                                    rate-limit,
                                    scrubbing-
                                    divert, BGP
                                    FlowSpec
                                    announcement)
PATCH  /v1/mitigation-actions/{actionId}/release
GET    /v1/mitigation-actions?detection={detectionId}
```

## §7 BCP 38 / BCP 84 Attestation Endpoints

```
GET    /v1/bcp-attestations
POST   /v1/bcp-attestations
GET    /v1/bcp-attestations/{attestationId}
```

## §8 Cooperation Record Endpoints

```
GET    /v1/cooperation-records
POST   /v1/cooperation-records
GET    /v1/cooperation-records/{recordId}
```

## §9 Examination Endpoints

```
GET    /v1/examination/programmes
GET    /v1/examination/protected-services
GET    /v1/examination/attack-detections
GET    /v1/examination/mitigation-actions
GET    /v1/examination/bcp-attestations
GET    /v1/examination/cooperation-records
GET    /v1/examination/audit-events
```

The examination scope is read-only and bound to the
authority's identity (national CERT — KrCERT-CC,
US-CERT, JPCERT-CC, ENISA at EU-level — or the
operating jurisdiction's information-security
supervisor).

## §10 Authentication and Authorisation

DOTS-channel authentication uses DTLS / TLS with
client and server certificates per RFC 9132 §6.1.
The HTTPS / JSON surface uses OAuth 2.1 bearer
tokens with audiences declared per surface.
Internal subsystem-to-subsystem calls use mutual
TLS with the operator's internal certificate
authority.

## §11 HTTP Status Codes

- `200 OK` — read or search success
- `201 Created` — create success (Location header)
- `202 Accepted` — mitigation request accepted; the
  status endpoint will surface the evolving state
- `400 Bad Request` — malformed payload
- `401 Unauthorized` — missing or invalid bearer
  token / DTLS certificate
- `403 Forbidden` — discipline rejection (the
  Problem Details references the rejecting policy)
- `404 Not Found` — resource not registered
- `409 Conflict` — version-mismatch on update
- `422 Unprocessable Content` — validation failure
- `429 Too Many Requests` — rate-limit exceeded
- `503 Service Unavailable` — upstream DOTS server
  or scrubbing-centre unreachable

## §12 Caching, Trace-Context and Observability

`ETag` carries the resource's version-id. Trace-
context (`traceparent`) is propagated across the
operator's pipeline. The operator's observability
exports include flow telemetry (NetFlow v9, IPFIX
RFC 7011), packet sampling (sFlow), and DNS query
telemetry — these feed the attack-detection
record (PHASE-1 §5).

## §13 Webhook and Event Surface

The operator publishes lifecycle events through a
webhook channel:

- `attack.detected`, `attack.escalated`,
  `attack.de-escalated`, `attack.ended`.
- `dots.mitigation-requested`,
  `dots.mitigation-accepted`,
  `dots.mitigation-withdrawn`.
- `mitigation.applied`, `mitigation.released`.
- `bcp-attestation.updated`.
- `incident.review-completed`.

Webhook signatures use HTTP Message Signatures
(RFC 9421).

## §14 Bulk-Export and Forensic-Reconstruction Surface

```
POST   /v1/bulk-export
GET    /v1/bulk-export/{exportId}/status
GET    /v1/bulk-export/{exportId}/manifest
GET    /v1/forensic/reconstruction?detection={detectionId}
```

The forensic-reconstruction endpoint produces a
deterministic per-detection reconstruction packaging
the IPFIX flow records, sFlow samples, DOTS exchanges,
mitigation actions, and post-incident analysis
narrative. The packaging is signed using the
operator's signing key so that downstream forensic
consumers (national CERT, law enforcement under
MLAT, the operator's incident-response auditor)
can verify integrity end-to-end.

## §15 Per-Tenant Access Discipline

For multi-tenant scrubbing-as-a-service operators
the API surface is partitioned per tenant — every
PHASE-1 §3 / §5 / §6 / §7 record is scoped to the
authenticated tenant's identifier. Cross-tenant
queries are rejected at the API boundary; the
operator's compliance scope spans the per-tenant
audit logs and is exposed through the examination
surface only.

## §16 Telemetry-Sharing Surface

For threat-intelligence sharing the operator exposes
a STIX 2.1 / TAXII 2.1 endpoint:

```
GET    /taxii2/collections
GET    /taxii2/collections/{id}/objects?added_after={iso}
POST   /taxii2/collections/{id}/objects
```

Sharing scope is governed by the operator's TLP
(Traffic Light Protocol) tagging policy and the
sector-ISAC's published sharing-rules. The operator's
indicators-of-compromise are emitted as STIX 2.1
indicator + observed-data + malware-analysis
objects; operator-private indicators are tagged
TLP:RED and excluded from cross-sector sharing.

## §17 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the DOTS signal and
data channels with the operator's published cuid /
mid binding, expose the supervisory / CERT
examination surface, and propagate trace-context
across the detection-to-mitigation chain.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-ddos-protection
- **Last Updated:** 2026-04-28
