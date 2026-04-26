# WIA-MED-003 Phase 3: Communication Protocol Standard

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-01-15

## Overview

Phase 3 defines the communication protocols for transmitting vital sign data between devices and applications.

## Supported Protocols

1. **Bluetooth Low Energy (BLE)**
2. **WiFi/TCP**
3. **MQTT**
4. **WebSocket**

## Bluetooth LE Protocol

### GATT Service Definition

**Service UUID:** `00001820-0000-1000-8000-00805f9b34fb`

**Characteristics:**

| Characteristic | UUID | Properties | Description |
|----------------|------|------------|-------------|
| Device Info | `00002a1e-...` | Read | Manufacturer, model, firmware |
| Signal Config | `00002a1f-...` | Read/Write | Sampling rate, filters |
| Data Stream | `00002a20-...` | Notify | Real-time vital sign data |
| Control | `00002a21-...` | Write | START, STOP, PAUSE commands |
| Status | `00002a22-...` | Notify | Battery, quality, errors |

### Connection Parameters

- **Connection Interval:** 20-50 ms (real-time streaming)
- **Slave Latency:** 0-2
- **Supervision Timeout:** 2000 ms

## WiFi/TCP Protocol

### TLS Socket Connection

- **Port:** 8883 (standard MQTT-TLS port)
- **Encryption:** TLS 1.3
- **Certificate:** Device certificate for authentication

### Data Format

Binary packets with header, payload, and CRC:
```
[Header 4 bytes][Payload N bytes][CRC 2 bytes]
```

## MQTT Protocol

### Topic Structure

```
wia-med-003/
  ├─ {deviceId}/
  │   ├─ info                 # Device info (retained)
  │   ├─ status               # Status updates
  │   ├─ data/
  │   │   ├─ ecg             # ECG data
  │   │   ├─ spo2            # SpO2 data
  │   │   └─ bp              # Blood pressure data
  │   ├─ alerts              # Alerts
  │   └─ control             # Control commands
  └─ broadcast/
      └─ discovery           # Device discovery
```

### QoS Levels

- **data/*:** QoS 0 (real-time, next sample coming soon)
- **alerts:** QoS 1 (important but duplicates acceptable)
- **control:** QoS 2 (exactly once execution required)
- **info:** QoS 1 + retained (new subscribers need info)

## WebSocket Protocol

### Connection

```
wss://api.example.com/stream?deviceId={deviceId}&token={jwt}
```

### Message Format

JSON messages for commands and data:

```json
{
  "type": "command|data|ping|pong",
  "payload": { ... }
}
```

### Keep-Alive

Ping/Pong messages every 30 seconds to maintain connection.

## Security Requirements

### Encryption

- **Transport Layer:** TLS 1.3 minimum
- **Data Layer:** AES-256-GCM for sensitive data
- **Key Exchange:** ECDHE (Elliptic Curve Diffie-Hellman Ephemeral)

### Authentication

- **Device:** X.509 certificates or pre-shared keys
- **User:** OAuth 2.0 or JWT tokens
- **Session:** Time-limited session tokens with refresh

### Compliance

- **HIPAA:** Encryption at rest and in transit
- **GDPR:** User consent, data portability, right to deletion
- **PIPEDA:** Canadian privacy requirements

---

## Federation Protocol

Multiple WIA Vital Sign Streaming hosts (a hospital ICU, an ambulance,
a remote monitoring service) federate so a patient's signal can follow
the patient across care boundaries without re-instrumentation.

### Roles

| Role | Description |
|------|-------------|
| **Producer** | Holds the device-side stream and signs frames |
| **Consumer** | Subscribes to one or more streams (clinician dashboard, EHR bridge, alert routing service) |
| **Custodian** | Holds the patient's consent envelope and authorises which consumers may subscribe |
| **Auditor** | Holds the audit log of every subscription and replay request |

A single legal entity MAY play multiple roles. Trust between roles is
established by federation handshake and recorded in signed receipts.

### Federation Handshake

```
   IDLE
     │ peer presents credential + ephemeral key
     ▼
   PENDING (origin verifies signature)
     │ valid
     ▼
   ACCEPTED (origin issues receipt; both sides persist)
     │ optional revocation
     ▼
   REVOKED
```

PENDING MUST resolve within 30 seconds; otherwise the origin returns a
problem of type `…/handshake-timeout`.

### Replay Defence

Each signed envelope (frame, descriptor, alert, subscribe, replay
request) carries a 96-bit nonce and an RFC 3339 timestamp. Receivers
MUST:

1. Reject envelopes with skew > ±300 s.
2. Reject envelopes whose `(signer, nonce)` tuple has been seen within
   the last 600 s.
3. Maintain the seen-nonce cache for at least 600 s.

For frames the cache MAY use a Bloom filter to absorb high frame
rates; false positives MUST result in a re-fetch via replay rather
than a silent drop.

### Patient Consent Envelope

```json
{
  "wia_vital_sign_streaming_version": "1.0.0",
  "type": "patient_consent",
  "patient_id": "did:wia:patient:01HXY",
  "custodian_id": "did:wia:custodian:hospital-A",
  "scope": ["icu", "remote_monitoring"],
  "valid_from": "2026-04-27T00:00:00Z",
  "valid_until": "2026-05-27T00:00:00Z",
  "consent_artefact_url": "https://omni.example/consent/01HXY/2026-04-27.pdf",
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

Producers and consumers MUST refuse subscription requests whose patient
identifier lacks an active consent envelope from the custodian, except
in declared emergency scope (`scope = ["emergency"]`) which auto-expires
after 24 hours and triggers a mandatory custodian review.

### Audience Controls

| Audience | Visibility |
|----------|------------|
| `clinician`        | full waveform + identifying metadata |
| `nurse_station`    | full waveform, redacted identifier |
| `ehr_bridge`       | summary samples (per-minute aggregates) |
| `research`         | de-identified waveform per HIPAA Safe Harbor §164.514(b)(2) |
| `family`           | snapshot only (most recent value per channel) |
| `emergency_service`| full waveform + identifying metadata under emergency consent |

### Cryptographic Suite

| Use | Algorithm | Reference |
|-----|-----------|-----------|
| Identity signing | Ed25519 | IETF RFC 8032 |
| HTTP message signing | Ed25519 over RFC 9421 | RFC 9421 |
| Hashing | SHA-256 | FIPS 180-4 |
| Transport | TLS 1.3 | IETF RFC 8446 |
| Data-layer | AES-256-GCM | FIPS 197 / NIST SP 800-38D |

### Conformance

A Phase 3 conformant implementation MUST:

1. Implement the federation handshake state machine.
2. Honour replay-defence bounds.
3. Enforce patient consent before serving any non-emergency stream.
4. Apply audience-based read controls.
5. Maintain an append-only audit log of every subscribe / replay
   request signed by the requesting consumer.

## Appendix A — Worked Federation Trace

```
α = ICU host did:wia:operator:icu-A
β = remote monitoring host did:wia:operator:rmt-B
λ = patient did:wia:patient:01HXY
```

```
T-1d  λ's custodian publishes patient_consent (scope=remote_monitoring)
T+0   β → α: handshake (consumer role, scope=remote_monitoring)
T+5s  α verifies consent + receipt, ACCEPTED
T+10s β: GET /vss/stream/stream-001 (subscribe)
T+11s α: SSE open, frames flow at 250 Hz
T+1h  λ leaves remote-monitoring scope; custodian publishes new consent
T+1h+1s α: emits notice envelope; β closes the SSE
T+1h+2s β: refreshes consent, re-subscribes if scope still permits
```

A failed consent verification at any point yields a problem document
of type `…/consent-not-found` and the SSE is closed with
`event: error` per WHATWG HTML §9.2.

## Appendix B — Replay Cache Sizing

For a busy ICU host receiving 250 Hz × 8 channels × 12 beds = 24 000
frames per second, the seen-nonce cache must hold roughly
`24 000 × 600 = 14.4 M` entries to enforce the 600-second window.
With 16-byte nonce keys plus a 4-byte timestamp, the strict cache
footprint is approximately `14.4 M × 24 ≈ 350 MiB`. Hosts at this
scale SHOULD use a Bloom filter sized for 1 % false positive rate
(~ 50 MiB) and treat false positives as triggers for a
replay-segment fetch rather than silent drops.

## Appendix C — Privacy Threats and Mitigations

| Threat | Mitigation |
|--------|------------|
| Bulk export of patient frames by compromised consumer | Per-audience read controls; consumer revocation on first detected anomaly; rate-limit consumer's replay window |
| Side-channel timing on signature verify | Constant-time Ed25519 implementations REQUIRED |
| Long-term metadata accumulation by EHR bridge | Bridge receives per-minute aggregates by default; full waveform requires elevated audience |
| Re-identification of de-identified research export | HIPAA Safe Harbor §164.514(b)(2) plus k-anonymity ≥ 5 over the export window |
| Family / next-of-kin endpoint abuse | Snapshot-only audience; rate-limited; identity proof via SSO required for the family endpoint |

## Appendix D — Operator Failover

When a host fails over from primary to standby region, the standby
MUST:

1. Reload the persistent seen-nonce cache before resuming envelope
   processing.
2. Re-issue handshakes to peers whose receipts are not present in the
   standby's storage.
3. Replay any missed alert envelopes from the primary's append-only
   log before accepting new alerts.
4. Notify peers via a `notice` envelope that primary→standby
   switchover has occurred, with an estimated `restore_at`.

## Appendix E — Trust List Maintenance

Each host maintains a signed trust list:

```json
{
  "wia_vital_sign_streaming_version": "1.0.0",
  "type": "trust_list",
  "operator_id": "did:wia:operator:icu-A",
  "issued_at": "2026-04-01T00:00:00Z",
  "valid_until": "2026-05-01T00:00:00Z",
  "entries": [
    { "peer_id": "did:wia:bridge:epic-eu",  "role": "ehr_bridge",  "score": 0.98 },
    { "peer_id": "did:wia:operator:rmt-B",  "role": "consumer",    "score": 0.95 },
    { "peer_id": "did:wia:research:univA",  "role": "research",    "score": 0.85 }
  ],
  "signature": { "alg": "Ed25519", "value": "…" }
}
```

Trust lists are republished at least monthly; peers refuse stale lists
older than 60 days. A peer may self-publish a `revocation` envelope to
immediately drop trust between list refresh windows.

## Appendix F — Audit Log Shape

Every subscribe and replay request MUST be logged in append-only
storage:

```json
{
  "wia_vital_sign_streaming_version": "1.0.0",
  "type": "audit_event",
  "event_id": "ae_01HZA…",
  "event_kind": "subscribe",
  "stream_id": "stream-001",
  "patient_id": "did:wia:patient:01HXY",
  "consumer_id": "did:wia:clinician:09…",
  "consumer_audience": "clinician",
  "consent_envelope_id": "consent_01HXY",
  "captured_at": "2026-04-27T10:00:00Z",
  "outcome": "accepted",
  "signature": { "alg": "Ed25519", "value": "…" }
}
```

`event_kind` is one of `subscribe`, `unsubscribe`, `replay_request`,
`replay_served`, `consent_check_failed`. The audit log is read-only
by every role except the auditor; the auditor MAY query but MUST
NOT mutate.

## Appendix G — References

* IETF RFC 8032 — EdDSA
* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* FIPS 180-4 — SHA family
* FIPS 197 — AES
* NIST SP 800-38D — GCM mode
* HIPAA Safe Harbor §164.514(b)(2)
* IEEE 11073 — Personal health device communication
* WHATWG HTML — Server-Sent Events

---

**Copyright 2025 WIA / SmileStory Inc.**
**License:** MIT
**弘益人間 · Benefit All Humanity**
