# WIA-predictive-maintenance PHASE 3 — Protocol Specification

**Standard:** WIA-predictive-maintenance (WIA-IND-026)
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the wire-level protocols used
by WIA-predictive-maintenance participants for
discovery, telemetry transport (OPC UA / MTConnect
/ MQTT / AMQP), signed publication of anomalies
and prognoses, work-order lifecycle, twin-model
exchange, and federation.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP Semantics), RFC 9112 (HTTP/1.1), RFC 9114 (HTTP/3)
- IETF RFC 8446 (TLS 1.3), RFC 6797 (HSTS)
- IETF RFC 8615 (Well-Known URIs), RFC 7517 (JWK), RFC 7515 (JWS)
- IETF RFC 9421 (HTTP Message Signatures), RFC 9530 (Digest Fields)
- IEC 62541 (OPC UA) including Pub/Sub
- ISO 23247-2 Digital Twin Manufacturing
- MTConnect Specification 2.4
- MQTT v5.0 (OASIS), AMQP 1.0 (OASIS), Sparkplug B (informative)
- IEC 62443 series (Industrial cybersecurity)
- IEEE 802.1Q TSN (informative)

---

## §1 Scope

This PHASE defines the on-the-wire behaviour
between operators, OEMs, MRO providers, and audit
authorities for the predictive-maintenance domain.

## §2 Discovery

A WIA-predictive-maintenance registry serves a
discovery document at:

```
GET /.well-known/wia/predictive-maintenance
```

Response (`application/json`):

```json
{
  "registry": "https://pdm.wiastandards.com",
  "openapi": "https://pdm.wiastandards.com/openapi.json",
  "operationGroups": ["/v1/assets", "/v1/sensors",
                      "/v1/streams", "/v1/conditions",
                      "/v1/anomalies", "/v1/prognoses",
                      "/v1/orders", "/v1/twins",
                      "/v1/registry"],
  "transports": ["opc-ua", "mtconnect", "mqtt-5", "amqp-1.0"],
  "iec62443Zone": "L4 / Operations / Manufacturing",
  "keySet": "https://pdm.wiastandards.com/.well-known/jwks.json"
}
```

The discovery document is signed (RFC 9421).

## §3 Transport

HTTPS with TLS 1.3 and HSTS preload at the
registry layer. Field-level transports follow:

- OPC UA over TCP with security policy
  `Aes256_Sha256_RsaPss`;
- MQTT v5.0 over TLS 1.3 with client certificates;
- AMQP 1.0 over TLS 1.3 with SASL EXTERNAL;
- MTConnect over HTTP/2 with mutual TLS at the
  IT/OT boundary.

## §4 Content negotiation

| Accept                                | Use                                      |
|---------------------------------------|------------------------------------------|
| `application/json`                    | record bodies                            |
| `application/cbor`                    | compact telemetry payloads               |
| `application/protobuf`                | high-rate telemetry envelopes            |
| `application/avro+binary`             | Sparkplug-style payloads                 |
| `application/x-mtconnect+xml`         | MTConnect probes / current               |
| `application/problem+json`            | error                                    |

## §5 Signed publication

Anomaly, prognosis, order, and completion records
are signed with detached JWS (RFC 7515) over the
canonical JSON form (RFC 8785). The `kid`
references the operator or OEM key in the
registry's JWKS.

## §6 Identifiers

| Identifier         | Format                                          |
|--------------------|-------------------------------------------------|
| `assetRef`         | UUID (RFC 4122)                                 |
| `sensorRef`        | UUID                                            |
| `streamRef`        | URI                                             |
| `conditionRef`     | UUID                                            |
| `anomalyRef`       | UUID                                            |
| `prognosisRef`     | UUID                                            |
| `orderRef`         | UUID                                            |
| `twinRef`          | URI                                             |
| LEI                | ISO 17442                                       |

## §7 Caching and immutability

Replay archives, anomaly records, and order
completion records are immutable. Anomaly and
prognosis records are append-only; corrections
emit a new record with `corrects` set to the
prior.

## §8 Federation

Federated registries form a directed graph in the
discovery document. Cross-registry queries follow
the graph and carry an `X-WIA-Federation-Path`
header. Trust between operators and OEMs is
anchored in the JWKS sets exposed at
`/.well-known/jwks.json`.

## §9 Replay protection

Signed publications carry `iat`/`exp` JWS claims
(max 24h). Telemetry replay endpoints are gated
by per-operator JWT scopes so that replay archives
cannot be retrieved without authorisation.

## §10 Error semantics

Errors are `application/problem+json` (RFC 9457).
Protocol-level codes:

| Code | Meaning                                              |
|------|------------------------------------------------------|
| 200  | success                                              |
| 304  | conditional GET unchanged                            |
| 400  | malformed JSON / OPC UA reference                    |
| 401  | missing or invalid token                             |
| 403  | LEI not authorised for the asset                     |
| 410  | tombstone (decommissioned asset)                     |
| 422  | schema violation                                     |
| 426  | TLS upgrade required                                 |
| 503  | OPC UA server / MQTT broker unavailable              |

## §11 Observability

Servers SHOULD emit OpenTelemetry traces with
`wia.pdm.operation`, `wia.pdm.assetRef`,
`wia.pdm.sensorRef`, and `wia.pdm.failureMode`
attributes.

## §12 Cybersecurity (IEC 62443)

Conformant deployments declare their IEC 62443
zone (L0..L4) and the conduits crossing zone
boundaries. Records crossing the IT/OT boundary
are signed and routed through gateways that
enforce the deployment's cybersecurity policy.

## Annex A — Conformance levels

- **Tier 1 — Self-declared:** discovery served,
  records signed.
- **Tier 2 — Verified:** OPC UA / MQTT / MTConnect
  interop tested; ISO 14224 taxonomy compliance.
- **Tier 3 — Anchored:** continuous evidence
  stream per PHASE-4 Annex G; IEC 62443 zone
  certification.

## Annex B — Discovery document signature

The signature over `/.well-known/wia/predictive-
maintenance` covers `@authority`, `@path`,
`content-digest` (RFC 9530), and `content-type`.

## Annex C — Cross-Origin Resource Sharing

Read endpoints serve `Access-Control-Allow-Origin:
*` with `ETag`, `Link`, and IEC-62443 zone headers
exposed.

## Annex D — Trust anchor rotation

Operator and OEM signing keys rotate per the
operator's policy. Recommended cadence is 12
months for high-volume operators and 36 months
otherwise.

## Annex E — Federation hop cap

Federated lookups carry an
`X-WIA-Federation-Hops` header; queries with
`Hops > 3` are dropped.

## Annex F — TLS profile baseline

TLS 1.3 with PFS-only cipher suites. Industrial
deployments operating older PLCs MAY use TLS 1.2
on conduits if the operator's policy permits.

## Annex G — Telemetry firehose

Operators emitting at high rate (e.g.
high-frequency vibration sampling at 50 kHz) use
dedicated `/v1/streams/firehose` over a long-lived
HTTP/3 stream or via the transport-native MQTT/AMQP
flow.

## Annex H — OPC UA security policy

OPC UA endpoints declare the security policy
(`Aes256_Sha256_RsaPss` is required for new
deployments; `Basic256Sha256` is acceptable for
legacy hardware). User authentication uses X.509
client certificates issued under the operator's
PKI.

## Annex I — MQTT v5.0 properties

Telemetry over MQTT v5.0 uses User Properties to
carry the `streamRef` and `sensorRef`. The
broker's authorisation policy maps the publisher
client identity to the operator LEI.

## Annex J — TSN time synchronisation

Time-Sensitive Networking-aware telemetry uses
IEEE 802.1Q TSN with PTPv2 time synchronisation.
Synchronised clocks ensure that cross-asset
correlations remain accurate at sub-millisecond
granularity.

## Annex K — Sparkplug B compatibility

Where the operator's edge gateway speaks Sparkplug
B (NBIRTH / NDEATH / NDATA / NCMD), the registry
accepts Sparkplug envelopes and transcodes to the
canonical PHASE-1 records. The Sparkplug client
identifier maps to the operator LEI plus a
deployment scope.

## Annex L — Process-safety zone gate

Records originating from SIL-rated zones cross the
IT/OT boundary only via attested gateways. The
gateway adds an `X-WIA-Safety-Zone` header that
downstream consumers verify before processing.

## Annex M — Replay archive integrity

Replay archives carry a SHA-512 digest in the
`Digest` header (RFC 9530) and are signed with
detached JWS. Auditors verify both before using
the archive in compliance reviews.

## Annex N — Asset Administration Shell transport

Asset Administration Shells (IEC 63278-1) are
transported as JSON-LD or AASX (zip) over HTTPS
with detached JWS signatures. Submodel updates
emit `aas.submodel-changed` events so that
downstream consumers can reconcile asset state.

## Annex O — High-frequency vibration sampling

High-frequency vibration sampling (≥10 kHz) uses
either OPC UA Pub/Sub over UADP or MQTT v5.0 with
chunked payloads. Receivers reassemble chunks per
the declared chunk size and verify the chunk
digest before forwarding to analytics consumers.

## Annex P — Federation peer trust

Operator and OEM federation peers exchange peer
assertions signed under their LEIs. Trust is
revocable; a revoked peer's records are
quarantined for 30 days then purged.

## Annex Q — Discovery cache TTL

The discovery document carries
`Cache-Control: public, max-age=300,
stale-while-revalidate=60` so that consumers
cache it briefly without missing key rotations.

## Annex R — Time synchronisation across assets

Cross-asset correlation requires synchronised
clocks; PTPv2 is preferred at sub-millisecond
precision, NTPv4 stratum-2 acceptable for
millisecond-class correlations. Asset records
declare the achieved synchronisation precision so
that downstream analyses know the temporal floor.

## Annex S — Backpressure on telemetry stream

When a stream exceeds the deployment's quota, the
broker emits a flow-control event upstream. The
edge gateway may buffer or drop per the operator's
policy. Drop events are logged and surfaced in
the audit feed so that telemetry gaps can be
analysed.

## Annex T — Hardware-backed operator key

Operator signing keys MAY be hardware-bound (HSM,
FIDO2, smart card). When hardware-bound, the
registry records the attestation in the operator
record. Hardware binding is mandatory for
high-stakes maintenance orders (`safety-critical`
priority).

## Annex U — Sandbox endpoints

`/v1/sandbox` mirrors production with synthetic
assets and ephemeral state. Sandbox responses
carry `X-WIA-Sandbox: true`.

## Annex V — Cross-Origin Resource Sharing

Read endpoints serve `Access-Control-Allow-Origin:
*` with `ETag`, `Link`, and IEC-62443 zone headers
exposed. Browser clients fetching telemetry
replays MUST set `crossorigin="anonymous"` so
that integrity verification can be enforced.

## Annex W — Connection coalescing

HTTP/2 clients MAY coalesce connections across
sub-domains served by the same certificate. The
registry publishes the coalescing policy in the
discovery document.

## Annex X — JSON canonicalisation

JSON-bearing records are canonicalised per RFC
8785 prior to JWS signature.

弘益人間 (Hongik Ingan) — Benefit All Humanity
