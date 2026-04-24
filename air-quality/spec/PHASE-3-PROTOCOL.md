# WIA-ENE-017: Air Quality Monitoring Standard
## PHASE 3: PROTOCOL

**Document Version:** 1.1
**Status:** Active
**Last Updated:** April 25, 2026
**Philosophy:** 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

Phase 3 covers **how Phase 1 payloads move from the sensor to the application**: transport bindings, reliability semantics, payload compression for constrained radios, cryptographic signing of observations, and the bridge rules that keep MQTT, AMQP, CoAP, LoRaWAN, and NB-IoT telemetry mutually translatable.

Phase 2 defined the query-side contract (REST, WebSocket, GraphQL). Phase 3 defines the **publish-side contract** and the server-to-server transports that feed the REST layer.

### 1.1 Normative references

- OASIS MQTT 5.0
- OASIS AMQP 1.0 (ISO/IEC 19464)
- RFC 7252 — Constrained Application Protocol (CoAP)
- RFC 7959 — Block-Wise Transfers in CoAP
- RFC 8323 — CoAP over TCP/TLS/WebSockets
- LoRa Alliance TS001-1.0.4 — LoRaWAN Regional Parameters
- 3GPP TS 23.401 — NB-IoT data transport
- RFC 5280 — X.509 Public Key Infrastructure
- RFC 7515 — JSON Web Signature (JWS)
- RFC 8446 — TLS 1.3
- RFC 3161 — Time-Stamp Protocol (optional for Tier 1 audit trails)

---

## 2. Transport selection matrix

Implementers SHOULD pick the minimum-power transport that meets latency and reliability targets:

| Transport      | Typical sensor        | Uplink latency | Payload cap  | Net cost     | WIA QoS tier |
|----------------|-----------------------|----------------|--------------|--------------|--------------|
| HTTP/2 REST    | Mains-powered station | ≤ 1 s          | Megabytes    | High         | Tier 1–2     |
| MQTT 5.0 / TLS | Rooftop reference     | ≤ 1 s          | 256 KiB      | Medium       | Tier 1–3     |
| AMQP 1.0       | Broker federation     | ≤ 5 s          | Megabytes    | Medium-high  | Tier 1       |
| CoAP / DTLS    | Solar / LPWAN         | ≤ 30 s         | 1 KiB / frag | Low          | Tier 3       |
| LoRaWAN        | Remote, battery       | 10–60 s        | 51–222 B     | Very low     | Tier 3–4     |
| NB-IoT UDP     | Urban low-power       | 5–15 s         | 1 KiB        | Very low     | Tier 3–4     |

Regardless of transport, the **wire-format identity** is preserved: the Phase 1 JSON object (or its CBOR/binary encoding per § 6) is recoverable at the first bridge node.

---

## 3. MQTT 5.0 binding (mandatory for Tier 1 ingest)

### 3.1 Topic structure

```
wia/airquality/{country}/{city}/{station_id}/readings       # QoS 1, non-retained
wia/airquality/{country}/{city}/{station_id}/status         # QoS 1, retained
wia/airquality/{country}/{city}/{station_id}/alerts         # QoS 2, non-retained
wia/airquality/_cmd/{station_id}/calibrate                  # QoS 2 (downlink)
wia/airquality/_sys/deadletter                              # broker-internal
```

- `{country}` — ISO 3166-1 alpha-2 (e.g., `KR`, `US`)
- `{city}` — UN/LOCODE second segment (e.g., `Seoul` → `SEL`)
- `{station_id}` — Phase 1 regex `^WIA-AQ-[A-Z]{2}-[A-Za-z0-9-]+$`

Subscribers SHOULD use **shared subscriptions** (`$share/<group>/wia/airquality/#`) when horizontally scaling ingestion — this distributes one copy of each message to exactly one consumer in the group and is the only way to avoid duplicate writes at the datastore.

### 3.2 Required MQTT 5.0 properties

Every PUBLISH carrying a reading MUST include:

| Property                      | Value                                 |
|-------------------------------|---------------------------------------|
| `Content-Type`                | `application/json` or `application/cbor` |
| `Payload Format Indicator`    | `1` (UTF-8 text) or `0` (binary)      |
| `Message Expiry Interval`     | ≤ 3600 s (stale readings are dropped) |
| `Correlation Data`            | Idempotency key (UUIDv7 recommended)  |
| `User Property: qc_flag`      | `valid` / `suspect` / `calibration`   |
| `User Property: sensor_model` | Vendor/model string from Phase 1 metadata |

### 3.3 Last Will and retained status

On connect, every station MUST register:

```
Will Topic:   wia/airquality/KR/Seoul/001/status
Will Payload: {"state":"offline","since":"2026-04-25T14:30:00Z","reason":"will"}
Will QoS:     1
Will Retain:  true
```

On graceful shutdown it MUST PUBLISH the same topic with `state=offline` and `reason=clean`. Monitors can therefore distinguish an LTE drop from a calibration window by inspecting the retained message.

### 3.4 Session expiry and keepalive

- `Session Expiry Interval`: 3600 s for mains-powered, 86 400 s for battery (matches sleep cycle).
- `Keep Alive`: 60 s default; LPWAN bridges MAY negotiate up to 1200 s.

### 3.5 Broker quotas

Brokers SHOULD enforce: **Receive Maximum** = 64, **Maximum Packet Size** = 262 144, **Topic Alias Maximum** = 16. Clients exceeding these MUST receive `DISCONNECT` reason `0x95` (Packet too large) or `0x93` (Receive Maximum exceeded).

---

## 4. AMQP 1.0 binding (broker federation)

Use case: exporting a national network's feed to a regional aggregator (e.g., Korea → East-Asia) where guaranteed-once semantics and durable queues matter more than minimal overhead.

### 4.1 Address grammar

```
/exchange/wia.airquality/{country}.{city}.{station_id}.readings
/queue/wia.airquality.{consumer}.inbox
```

### 4.2 Message properties

- `message-id` (UUID) = Phase 1 idempotency key
- `content-type` = `application/json` or `application/cbor`
- `subject` = `reading.{pollutant}` — facilitates routing on body-less filters
- `application-properties[qc_flag]` mirrors the MQTT user property
- `absolute-expiry-time` enforces the stale-drop rule

Delivery MUST use **settled=false, state=accepted** once the record is durable in the consumer's datastore.

---

## 5. CoAP / DTLS for constrained sensors

### 5.1 URI template

```
coaps://{gateway}/wia/aq/r?s={station_id}
```

### 5.2 CBOR payload (RFC 8949)

Instead of JSON, constrained nodes MAY transmit the CBOR encoding of the Phase 1 object. Field names are preserved (no dictionary substitution at Tier 1) so a gateway can decode without out-of-band state.

```
Header: POST
Options:
  Uri-Path: wia, aq, r
  Content-Format: application/cbor (60)
  Accept:         application/cbor
  Proxy-Uri:      <set by gateway to REST endpoint>
Payload: CBOR(<Phase-1 object>)
```

Observations larger than the 1 KiB MTU MUST use RFC 7959 block-wise transfer with `SZX=6` (1024 B blocks). Retries follow the CoAP exponential back-off (ACK_TIMEOUT=2 s, ACK_RANDOM_FACTOR=1.5, MAX_RETRANSMIT=4).

---

## 6. LoRaWAN payload codec

LoRaWAN uplinks are too small for JSON; a deterministic 24-byte codec is defined so any decoder recovers the Phase 1 object.

### 6.1 Binary layout (v1 codec)

| Offset | Size | Field                           | Encoding                   |
|-------:|-----:|---------------------------------|----------------------------|
| 0      | 1    | `version` (high nibble) / `flags` (low nibble) | `0x1X`          |
| 1      | 4    | `timestamp_epoch_seconds`       | Big-endian uint32          |
| 5      | 2    | `pm25 × 10`                     | uint16, unit 0.1 μg/m³     |
| 7      | 2    | `pm10 × 10`                     | uint16, unit 0.1 μg/m³     |
| 9      | 2    | `o3_ppb`                        | uint16                     |
| 11     | 2    | `no2_ppb`                       | uint16                     |
| 13     | 2    | `so2_ppb`                       | uint16                     |
| 15     | 2    | `co_ppm × 100`                  | uint16                     |
| 17     | 1    | `temperature_C + 50`            | uint8 (range −50…205)      |
| 18     | 1    | `humidity_pct`                  | uint8 (0–100)              |
| 19     | 1    | `battery_pct`                   | uint8                      |
| 20     | 1    | `qc_flags_bitmap`               | bit0=pm25 valid …          |
| 21     | 3    | `station_id_lo24`               | last 3 bytes of station hash |

A LoRaWAN Network Server SHALL decode using the reference JavaScript codec bundled in `cli/lorawan-codec.js`; the decoder re-emits canonical Phase 1 JSON with `qc_flag` derived from the bitmap.

### 6.2 Downlink commands

Calibration triggers and AQI threshold changes are framed as TLV blocks in downlink port 2; see `spec/PHASE-4-INTEGRATION.md` for the operational flow.

---

## 7. NB-IoT UDP framing

Carriers that price per-byte (NB-IoT, LTE-M) SHOULD pack the CBOR encoding plus a 4-byte CRC32 into a single UDP datagram to the gateway:

```
+----+------------------------+--------+
| 0x | CBOR(<Phase-1 object>) | CRC32  |
+----+------------------------+--------+
  1B          N bytes             4B
```

Leading byte `0xAQ` (ASCII mnemonic for *Air Quality*) lets multiplexed NB-IoT collectors route without a heavier header. Lost datagrams are recovered by the store-and-forward rule (§ 9).

---

## 8. Cryptographic envelope

### 8.1 Transport security

- TLS 1.3 for MQTT/AMQP/HTTP.
- DTLS 1.3 (or 1.2 with PSK for legacy CoAP hardware).
- Cipher suites restricted to: `TLS_AES_128_GCM_SHA256`, `TLS_AES_256_GCM_SHA384`, `TLS_CHACHA20_POLY1305_SHA256`.
- Perfect Forward Secrecy is REQUIRED (ECDHE only).

### 8.2 mTLS certificate profile (Tier 1)

Station certificates MUST include:

```
X.509 v3, ECDSA P-256
SubjectAltName:
  DNS: station-001.kr.wia-airquality.org
  URI: urn:wia:airquality:station:WIA-AQ-KR-Seoul-001
ExtendedKeyUsage: clientAuth
Issued by:  "WIA AirQuality Station CA" (intermediate under WIA Root)
Validity:   ≤ 1 year, auto-renewed 30 days before expiry (ACME EAB)
```

Certificate revocation is published both via CRL (`crl.airquality.wia.org/stations.crl`) and OCSP stapling.

### 8.3 Payload signing (JWS) — recommended for regulatory evidence

For Tier 1 compliance reports, observations SHOULD be wrapped in a **JWS compact** over the canonicalised Phase 1 JSON (JCS, RFC 8785). The resulting envelope is:

```
eyJhbGciOiJFUzI1NiIsImtpZCI6InN0bi0wMDEtMjAyNi0wNCJ9.<payload>.<sig>
```

Verifiers:
1. fetch the station's current public key from `/stations/{id}/keys`;
2. validate the JWS signature with `ES256`;
3. compare the inner `station_id` and `timestamp` to the surrounding MQTT topic and `Message Expiry`.

Any mismatch disqualifies the record from regulatory submission and SHOULD set the ingest `qc_flag` to `suspect`.

### 8.4 Key rotation

- Station private keys SHOULD be stored in a TPM 2.0 / secure element (`object-attributes=fixedTPM|sensitiveDataOrigin|userWithAuth`).
- Rotation on failure or every 365 days, whichever is sooner.
- Old keys remain valid for verification for the regulator-mandated archive period (typically 5 years) via `/keys?history=true`.

---

## 9. Reliability rules

### 9.1 Store-and-forward

Every station MUST maintain a local ring buffer sized to cover at least 72 h of its nominal reporting interval. On re-connection, buffered readings are republished in chronological order with their **original** `timestamp` (never backdated to the present).

### 9.2 Duplicate detection

Consumers MUST dedupe on the tuple `(station_id, timestamp, pollutant)`. Idempotency keys (MQTT `Correlation Data`, AMQP `message-id`) act as secondary index; clashes on the tuple with different payloads raise `409 duplicate_reading` at the REST bridge.

### 9.3 Clock skew

Stations MUST keep their clock within ±2 s of UTC (NTP/PPS/GNSS). Drift exceeding that threshold flags the device `status=clock_drift` and server-side timestamping is used as an advisory `received_at` field while `timestamp` remains the station's own best estimate.

### 9.4 Dead-letter queue

Malformed messages are routed to `wia/airquality/_sys/deadletter` with User Property `reason`. Operators MUST alert when DLQ growth exceeds 0.1 % of traffic.

---

## 10. Bridge normalization

A bridge converts between transports without altering the Phase 1 payload. The canonical flow for a station publishing by MQTT and appearing in REST is:

```
Station  ──MQTT/TLS──▶  Broker  ──MQTT→HTTP bridge──▶  Ingest API
                          │
                          └──AMQP shovel──▶  Regional aggregator
                          │
                          └──Kafka sink──▶  Analytics lake (Phase 4)
```

Bridge requirements:
- topic → REST path mapping is 1:1 (§ 3.1);
- MQTT User Properties map to HTTP headers (`x-wia-qc-flag`, `x-wia-sensor-model`);
- CBOR payloads are re-encoded to JSON by the bridge, **never** re-timestamped;
- JWS envelopes pass through unchanged — verification happens at the REST boundary.

---

## 11. Worked examples

### 11.1 Publishing to MQTT with QoS 1 and idempotency

```python
import paho.mqtt.client as mqtt
import json, uuid, ssl

client = mqtt.Client(protocol=mqtt.MQTTv5, client_id="station-001")
client.tls_set("ca.pem", "station-001.crt", "station-001.key",
               tls_version=ssl.PROTOCOL_TLSv1_3)
client.will_set(
    "wia/airquality/KR/Seoul/001/status",
    json.dumps({"state": "offline", "reason": "will"}),
    qos=1, retain=True,
)
client.connect("mqtt.airquality.wia.org", 8883, keepalive=60)

props = mqtt.Properties(mqtt.PacketTypes.PUBLISH)
props.ContentType = "application/json"
props.MessageExpiryInterval = 3600
props.CorrelationData = uuid.uuid4().bytes
props.UserProperty = [("qc_flag", "valid"), ("sensor_model", "BAM-1020")]

reading = {
    "station_id": "WIA-AQ-KR-Seoul-001",
    "timestamp": "2026-04-25T14:30:00Z",
    "measurements": {"pm25": {"value": 35.4, "unit": "ug/m3",
                               "qc_flag": "valid"}}
}

client.publish("wia/airquality/KR/Seoul/001/readings",
               json.dumps(reading), qos=1, properties=props)
```

### 11.2 Verifying a JWS-signed reading (Node.js)

```js
import { importJWK, compactVerify } from "jose";

const { payload, protectedHeader } = await compactVerify(jws, async (hdr) => {
  const jwk = await fetch(`https://api.airquality.wia.org/v1/stations/` +
                          `${extractSta(jws)}/keys/${hdr.kid}.jwk`)
                .then(r => r.json());
  return importJWK(jwk, "ES256");
});

const reading = JSON.parse(new TextDecoder().decode(payload));
if (reading.station_id !== topicStation) throw new Error("station mismatch");
```

### 11.3 Decoding a LoRaWAN uplink in a Chirpstack codec

```js
function decodeUplink(input) {
  const b = input.bytes;
  if ((b[0] & 0xf0) !== 0x10) return { errors: ["bad version"] };
  const ts = (b[1]<<24 | b[2]<<16 | b[3]<<8 | b[4]) >>> 0;
  return {
    data: {
      station_id_hash: ((b[21]<<16)|(b[22]<<8)|b[23]).toString(16).padStart(6,"0"),
      timestamp: new Date(ts * 1000).toISOString(),
      measurements: {
        pm25: { value: ((b[5]<<8)|b[6]) / 10, unit: "ug/m3",
                qc_flag: (b[20] & 0x01) ? "valid" : "suspect" },
        pm10: { value: ((b[7]<<8)|b[8]) / 10, unit: "ug/m3" },
        o3:   { value: (b[9]<<8)|b[10], unit: "ppb" },
        no2:  { value: (b[11]<<8)|b[12], unit: "ppb" },
        so2:  { value: (b[13]<<8)|b[14], unit: "ppb" },
        co:   { value: ((b[15]<<8)|b[16]) / 100, unit: "ppm" },
      },
      meteorology: { temperature: b[17] - 50, humidity: b[18] },
      device_status: { battery_level: b[19] }
    }
  };
}
```

---

## 12. Operations and deployment guidance

- **Firewall**: outbound TCP 8883 (MQTTS), 5671 (AMQPS), 443 (HTTPS); no inbound requirement at the station.
- **Clock**: GNSS-backed NTP; fall-back to `time.cloudflare.com`/`time.google.com`.
- **Out-of-band management**: the `_cmd` topic MUST be ACL-restricted to operator role (Phase 2 § 3.3).
- **Observability**: MQTT brokers SHOULD emit per-topic rate, RTT, and DLQ counters to OpenTelemetry.
- **Air-gapped deployments**: replace the public PKI with an offline WIA-compatible CA; CRL distribution via USB courier is explicitly permitted for Tier 3–4.

---

## 13. Compliance checklist

**Phase 3 conformance requires:**

- [ ] TLS 1.3 (or DTLS 1.2 for LPWAN) on every transport
- [ ] mTLS with the WIA X.509 profile for Tier 1 stations
- [ ] MQTT 5.0 user properties set on every PUBLISH
- [ ] Last Will message retained on status topic
- [ ] Store-and-forward buffer ≥ 72 h
- [ ] Duplicate detection on `(station_id, timestamp, pollutant)`
- [ ] Clock kept within ±2 s of UTC; skew alerts emitted
- [ ] LoRaWAN codec v1 matches reference for at least one sensor model
- [ ] JWS signing available for all Tier 1 regulatory exports
- [ ] DLQ present and monitored

---

## 14. References

1. OASIS MQTT 5.0 specification
2. ISO/IEC 19464 — AMQP 1.0
3. RFC 7252 / 8323 / 7959 — CoAP core, over TCP/TLS/WebSockets, and block-wise transfer
4. LoRa Alliance TS001-1.0.4 — LoRaWAN Regional Parameters
5. 3GPP TS 23.401 — NB-IoT user-plane optimisations
6. RFC 5280 — X.509 v3 PKI profile
7. RFC 7515 / 8785 — JWS and JSON Canonicalization
8. RFC 8446 — TLS 1.3
9. RFC 8949 — CBOR

---

**Document Status:** ACTIVE
**Effective Date:** April 25, 2026
**Review Date:** April 25, 2028

© 2026 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
