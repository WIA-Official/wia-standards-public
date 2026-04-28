# WIA-multiverse-interface PHASE 3 — Protocol Specification

**Standard:** WIA-multiverse-interface (WIA-QUA-017)
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the wire-level protocols used by
WIA-multiverse-interface participants for
discovery, signed publication of research records,
checkpoint-payload retrieval, observation
acquisition, and federation between consortia. The
protocol is layered over HTTP and standard
scientific-data exchange formats so that existing
HPC, observation, and analytics infrastructure
carries the workload.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP Semantics), RFC 9112 (HTTP/1.1), RFC 9114 (HTTP/3)
- IETF RFC 8446 (TLS 1.3), RFC 6797 (HSTS)
- IETF RFC 8615 (Well-Known URIs), RFC 7517 (JWK), RFC 7515 (JWS)
- IETF RFC 9421 (HTTP Message Signatures), RFC 9530 (Digest Fields)
- IETF RFC 8259 (JSON), RFC 8785 (JCS)
- W3C PROV-O 1.0, W3C SHACL, W3C JSON-LD 1.1
- HDF5 (The HDF Group, informative), Zarr v3 (informative)
- Apache Parquet 2 (informative)

---

## §1 Scope

This PHASE defines the on-the-wire behaviour
between consortia, simulation hubs, observation
arrays, ethics-review boards, and audit
authorities.

## §2 Discovery

A WIA-multiverse-interface registry serves a
discovery document at:

```
GET /.well-known/wia/multiverse-interface
```

Response (`application/json`):

```json
{
  "registry": "https://muv.wiastandards.com",
  "openapi": "https://muv.wiastandards.com/openapi.json",
  "operationGroups": ["/v1/protocols", "/v1/hypotheses",
                      "/v1/scenarios", "/v1/checkpoints",
                      "/v1/observations", "/v1/identities",
                      "/v1/timelines", "/v1/events",
                      "/v1/provenance", "/v1/registry"],
  "storage": ["hdf5", "zarr-v3", "parquet"],
  "keySet": "https://muv.wiastandards.com/.well-known/jwks.json"
}
```

The discovery document is signed (RFC 9421).

## §3 Transport

HTTPS with TLS 1.3 and HSTS preload. Large
checkpoint and observation payloads use HTTP/3 with
QUIC streams to amortise latency over high-RTT
research networks.

## §4 Content negotiation

| Accept                                | Use                                      |
|---------------------------------------|------------------------------------------|
| `application/json`                    | record bodies                            |
| `application/ld+json`                 | PROV-O graphs                            |
| `application/x-hdf5`                  | HDF5 checkpoints                         |
| `application/x-zarr`                  | Zarr v3 checkpoints                      |
| `application/parquet`                 | Parquet datasets                         |
| `application/x-fits`                  | FITS observation files (informative)     |
| `application/problem+json`            | error                                    |

## §5 Signed publication

Records are signed with detached JWS (RFC 7515)
over the canonical JSON form (RFC 8785). The `kid`
references the principal investigator's key in the
consortium's JWKS. Verification:

1. JWKS signature against consortium key set;
2. Consortium identity verification;
3. Document digest verification against the
   record's `digestRef` or `stateHash`.

## §6 Identifiers

| Identifier         | Format                                          |
|--------------------|-------------------------------------------------|
| `protocolRef`      | UUID (RFC 4122)                                 |
| `hypothesisRef`    | URI                                             |
| `scenarioRef`      | URI                                             |
| `checkpointRef`    | UUID; client-provided                           |
| `observationRef`   | UUID                                            |
| `identityRef`      | UUID                                            |
| `timelineRef`      | URI                                             |
| `eventRef`         | UUID                                            |
| Researcher DID     | per W3C DID 1.0                                 |

## §7 Caching and immutability

Checkpoint and observation payloads are immutable
once published; they carry `Cache-Control: public,
max-age=31536000, immutable`. Hypothesis and
scenario records are mutable through versioning;
they carry strong `ETag`.

## §8 Federation

Federated registries form a directed graph in the
discovery document. Cross-registry queries follow
the graph and carry an `X-WIA-Federation-Path`
header. Trust between consortia is anchored in the
JWKS sets exposed at `/.well-known/jwks.json`.

## §9 Replay protection

Signed publications carry `iat`/`exp` JWS claims
(max 24h). Simulation jobs carry per-job nonces;
duplicate job submissions are rejected as replay.

## §10 Error semantics

Errors are `application/problem+json` (RFC 9457).
Protocol-level codes:

| Code | Meaning                                              |
|------|------------------------------------------------------|
| 200  | success                                              |
| 304  | conditional GET unchanged                            |
| 400  | malformed JSON / SHACL violation                     |
| 401  | missing or invalid token                             |
| 403  | researcher not authorised under the protocol         |
| 410  | tombstone (withdrawn identity, retired protocol)     |
| 422  | record schema violation                              |
| 426  | TLS upgrade required                                 |
| 503  | federation peer unavailable                          |

## §11 Observability

Servers SHOULD emit OpenTelemetry traces with
`wia.muv.operation`, `wia.muv.protocolRef`,
`wia.muv.scenarioRef`, and `wia.muv.researcher`
(pseudonymised) attributes.

## Annex A — Conformance levels

- **Tier 1 — Self-declared:** discovery served,
  records signed.
- **Tier 2 — Verified:** PROV-O graph audited;
  ethics-review references resolve.
- **Tier 3 — Anchored:** continuous evidence
  stream per PHASE-4 Annex G; reproducibility
  exercised across at least two independent
  consortia.

## Annex B — Discovery document signature

The signature over `/.well-known/wia/multiverse-
interface` covers `@authority`, `@path`,
`content-digest` (RFC 9530), and `content-type`.

## Annex C — Cross-Origin Resource Sharing

Read endpoints serve `Access-Control-Allow-Origin:
*` with `ETag`, `Link`, and `X-WIA-Federation-Path`
exposed.

## Annex D — Trust anchor rotation

Consortium signing keys rotate per the consortium's
data-management plan. Recommended cadence is 24
months with a 6-month overlap.

## Annex E — Connection management

Long-lived simulation runs use HTTP/2 or HTTP/3
streams. Checkpoint uploads are streamed; the
registry verifies the digest after stream
completion.

## Annex F — Federation hop cap

Federated provenance walks carry an
`X-WIA-Federation-Hops` header; queries with `Hops
> 4` are dropped.

## Annex G — Storage interop

Checkpoint payloads accepted in HDF5, Zarr v3, and
Parquet. The registry records the storage format
in the checkpoint record so that downstream
consumers can configure their reader.

## Annex H — Replay-resistant observation

Observation submissions carry `iat`/`exp` claims
(max 1h) so that a captured submission cannot be
replayed against a different consortium.

## Annex I — TLS profile baseline

TLS 1.3 with PFS-only cipher suites; NIST SP
800-52 Rev. 2 baseline.

## Annex J — High-throughput firehose

Observation arrays emitting at high rate (e.g.
photon-counting cameras, gravitational-wave
interferometers) submit via dedicated
`/v1/observations/firehose` over a long-lived
HTTP/3 stream. The firehose endpoint applies the
deployment's quota and back-pressure policy.

## Annex K — Bulk checkpoint upload streaming

Bulk checkpoint uploads use chunked HTTP/2 streams
with per-chunk SHA-512 digests. The registry
verifies digests as chunks arrive and aborts the
stream on first mismatch.

## Annex L — Provenance graph signing

PROV-O graphs are signed by the principal
investigator over the canonical JSON-LD form
(per JSON Canonicalisation Scheme RFC 8785 applied
after JSON-LD framing). Signatures are mirrored at
`/v1/provenance/{ref}/signature`.

## Annex M — Cross-Origin scientific clients

Scientific notebook clients (Jupyter, Observable,
RStudio) fetch records from the registry over CORS.
The registry sets `Access-Control-Allow-Origin: *`
on read endpoints and exposes the relevant scientific
metadata headers.

## Annex N — Replay-resistant federation publish

When a consortium pushes a record across federation,
the push includes a unique federation-job nonce.
Receiving consortia maintain a sliding window of
recently-seen nonces to drop duplicates.

## Annex O — Connection-level rate limits

Per-consortium rate limits default to 10,000
records per hour and 100,000 per day across the
write surface. The registry surfaces remaining
quota in `X-Quota-Remaining` and returns 429 with
`Retry-After` once exhausted.

## Annex P — Long-poll webhook delivery

Webhooks are delivered with at-least-once semantics
and exponential backoff (2, 4, 8, 16, 32, 64, 128,
256, 512 seconds) capped at 9 attempts. Failed
deliveries enter a dead-letter queue inspectable
via the registry's audit feed.

## Annex Q — Discovery cache TTL

The discovery document carries
`Cache-Control: public, max-age=300,
stale-while-revalidate=60` so that consumers
cache it briefly without missing key rotations. A
fresh discovery fetch on each interaction is not
required.

## Annex R — Digest verification budget

Checkpoint payloads exceed common HTTP body
limits; verification is streamed. The registry
budgets digest verification per consortium so that
a malicious upload cannot exhaust verifier
resources. Verifier load is surfaced in
`/v1/registry/health/verifier`.

## Annex S — JSON-LD framing for PROV-O

PROV-O graphs use JSON-LD 1.1 framing per the W3C
recommended profile. Framing is documented at
`/v1/provenance/framing` so that consumers can
reproduce the canonicalisation step before
verifying the JWS signature.

## Annex T — Storage-format conformance

Read endpoints negotiate storage format per the
client's `Accept` header. The registry MAY
transcode between HDF5, Zarr v3, and Parquet on the
fly when both formats are equivalent for the
checkpoint's tensor shape. Transcoding is
informative and does not change the canonical
record.

## Annex U — Researcher proof-of-possession

Researcher signing keys MAY be hardware-bound
(HSM, FIDO2 with attestation, smart card). When
hardware-bound, the registry records the
attestation in the researcher record so that
downstream verifiers can confirm the binding
independently.

## Annex V — Streaming response budgets

Long checkpoint or PROV-O responses are streamed
with backpressure honoured per RFC 9112 §6.1.
Clients that fall behind the server's send rate
are temporarily paused; persistent fall-behind
triggers connection close with `wia.muv.flow.lag`
recorded in the audit feed.

## Annex W — Multi-region replication semantics

Federation replication writes records to the
target consortium with `replicaSource` set to the
origin. Replicas are tagged read-only at the
target until the origin signs a promotion event;
this prevents premature derivative work on
incomplete replicas.

## Annex X — Sandbox endpoints

`/v1/sandbox` mirrors the production surface with
synthetic protocols, fictional researchers, and
ephemeral state. Sandbox responses carry
`X-WIA-Sandbox: true`. Sandbox state clears on a
24h rolling window so that integration tests
remain deterministic.

## Annex Y — Audit-feed retention

The audit feed is retained for at least 7 years
under the consortium's data-management plan.
Funders and ethics-review boards consume the audit
feed for compliance verification.

## Annex Z — Per-instrument calibration sync

Observation arrays publish calibration records on a
documented cadence (typically nightly for
ground-based instruments, per-orbit for space
instruments). Calibration sync events are tagged
in the audit feed so that observation reproducers
can locate the calibration that was active at the
acquisition timestamp.

弘益人間 (Hongik Ingan) — Benefit All Humanity
