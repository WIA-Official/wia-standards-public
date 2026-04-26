# WIA-MED-003 Phase 2: API Interface Standard

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-01-15

## Overview

Phase 2 defines the standard API for software applications to communicate with vital sign monitoring devices.

## VitalSignStream Class

The core API for all WIA-MED-003 implementations:

```typescript
class VitalSignStream {
  // Device Management
  async discoverDevices(filter?: DeviceFilter): Promise<Device[]>
  async connect(deviceId: string): Promise<Connection>
  async disconnect(deviceId: string): Promise<void>
  getDeviceStatus(deviceId: string): DeviceStatus

  // Data Streaming
  async startStream(deviceId: string, config?: StreamOptions): Promise<Stream>
  async stopStream(streamId: string): Promise<void>
  onData(callback: (data: VitalSignData) => void): void
  onError(callback: (error: Error) => void): void

  // Data Storage
  async saveData(data: VitalSignData, storage?: StorageConfig): Promise<string>
  async loadData(query: DataQuery): Promise<VitalSignData[]>
  async exportData(format: ExportFormat, query: DataQuery): Promise<Blob>

  // Alert System
  setAlert(config: AlertConfig): string
  removeAlert(alertId: string): void
  onAlert(callback: (alert: Alert) => void): void
}
```

## Device Discovery

### Discovery Protocol

Devices advertise using standard service UUIDs:
- **Bluetooth LE:** Service UUID `00001820-0000-1000-8000-00805f9b34fb`
- **WiFi/mDNS:** Service type `_wia-med-003._tcp`

### Example

```javascript
const devices = await stream.discoverDevices({
  signalType: 'ECG',
  manufacturer: 'AcmeMedical'
});
```

## Data Streaming

### Stream Configuration

```typescript
interface StreamOptions {
  signalType: SignalType;
  samplingRate?: number;
  duration?: number;        // 0 = infinite
  compression?: 'none' | 'gzip' | 'delta';
  realtime?: boolean;
  filters?: {
    lowpass?: number;
    highpass?: number;
    notch?: number;
  };
}
```

### Example

```javascript
await stream.startStream(deviceId, {
  signalType: 'ECG',
  samplingRate: 512,
  compression: 'delta',
  realtime: true
});

stream.onData((data) => {
  console.log('Heart Rate:', data.analysis?.heartRate);
});
```

## Alert System

### Alert Configuration

```typescript
interface AlertConfig {
  name: string;
  signalType: SignalType;
  condition: string | ((data: VitalSignData) => boolean);
  severity: 'low' | 'medium' | 'high' | 'critical';
  actions: AlertAction[];
  cooldown?: number;
}
```

### Example

```javascript
stream.setAlert({
  name: 'High Heart Rate',
  signalType: 'ECG',
  condition: 'heartRate > 120',
  severity: 'high',
  actions: [
    { type: 'notification', message: 'Heart rate too high!' },
    { type: 'sound', sound: 'alert.mp3' }
  ]
});
```

## Error Handling

### Error Codes

- **1xxx:** Connection errors (DEVICE_NOT_FOUND, CONNECTION_FAILED, etc.)
- **2xxx:** Streaming errors (STREAM_START_FAILED, BUFFER_OVERFLOW, etc.)
- **3xxx:** Data errors (INVALID_DATA_FORMAT, SCHEMA_VALIDATION_FAILED, etc.)
- **4xxx:** System errors (LOW_BATTERY, SENSOR_FAILURE, etc.)

### Example

```javascript
stream.onError((error) => {
  if (error.code === ErrorCode.CONNECTION_LOST && error.recoverable) {
    // Attempt reconnection
    await stream.connect(deviceId);
  }
});
```

---

## HTTP Surface

In addition to the in-process `VitalSignStream` SDK above, every WIA
Vital Sign Streaming host exposes the following HTTP surface so that
non-SDK consumers (clinical dashboards, EHR bridges, conformance test
runners) can subscribe and replay without depending on a vendor SDK.

### Transport

* HTTPS only, TLS 1.3 (IETF RFC 8446) or later.
* HTTP/2 RECOMMENDED for the streaming endpoints; HTTP/1.1 SHALL be
  supported as fallback.
* `Strict-Transport-Security` per IETF RFC 6797.
* Compression: `gzip`, `br`.

### Discovery

```
GET https://<host>/.well-known/wia-vital-sign-streaming
```

```json
{
  "wia_vital_sign_streaming_version": "1.0.0",
  "operator_id": "did:wia:operator:icu-A",
  "endpoints": {
    "stream":   "https://stream.example/vss/stream",
    "snapshot": "https://stream.example/vss/snapshot",
    "replay":   "https://stream.example/vss/replay",
    "alert":    "https://stream.example/vss/alert"
  },
  "supported_signatures": ["Ed25519"],
  "supported_channels": ["ecg","ppg","spo2","hr","sbp","dbp","map","temp","rr","etco2"],
  "rate_limits": {
    "anonymous":     { "rps": 1,  "burst": 5 },
    "authenticated": { "rps": 50, "burst": 200 }
  }
}
```

Discovery responses are cacheable for 300 s.

### Stream Subscribe

```
GET /vss/stream/{stream_id}?since={cursor}
Accept: text/event-stream
Authorization: WIA-Sig keyid="…",signature="…"
```

Server-Sent Events (WHATWG HTML §9.2). Each event payload is a Phase 1
frame envelope. Reconnecting subscribers pass the most recent event id
as `Last-Event-Id`.

### Snapshot

```
GET /vss/snapshot/{stream_id}
```

Returns the most recent value per channel for low-rate clinical
dashboards that do not need full waveform.

### Replay

```
GET /vss/replay/{stream_id}?from={ts}&to={ts}
Accept: application/json
```

Returns historical frames within the requested window. Hosts MUST
return a `Range: bytes` style cursor when the window is too large for
one response, with `next_cursor` in the body.

### Alert Routing

```
POST /vss/alert
Content-Type: application/json
Authorization: WIA-Sig keyid="…",signature="…"
```

Body is a signed alert envelope; the host fans out to subscribed
consumers (nurse station, on-call mobile, EHR notification queue).

### Authentication

Writes (alert submissions, replay if private) require HTTP Message
Signatures per IETF RFC 9421. Reads MAY be anonymous subject to the
discovery rate limit; PHI-bearing reads MUST be authenticated.

### Errors

Errors follow IETF RFC 9457 problem details. Reserved error types live
under `https://wiastandards.com/vital-sign-streaming/errors/`.

```json
{
  "type": "https://wiastandards.com/vital-sign-streaming/errors/cursor-expired",
  "title": "Replay cursor expired",
  "status": 410,
  "detail": "Cursor older than 7 days; request the snapshot endpoint instead."
}
```

### Rate Limiting

`RateLimit-Limit`, `RateLimit-Remaining`, `RateLimit-Reset` headers
MUST be emitted per IETF draft `draft-ietf-httpapi-ratelimit-headers`.
Streaming endpoints MAY apply per-stream concurrency caps; the
discovery document advertises the cap.

### Conformance Checklist

A Phase 2 conformant host MUST:

1. Publish `/.well-known/wia-vital-sign-streaming` with all endpoints.
2. Implement stream subscribe over SSE with `Last-Event-Id` recovery.
3. Implement snapshot and replay endpoints.
4. Honour `Idempotency-Key` for alert submissions.
5. Emit problem-detail JSON for 4xx/5xx.

The companion simulator at `simulator/index.html` exercises each surface
interactively.

## Appendix A — Reference Request / Response

```http
GET /vss/stream/stream-001?since=fr_AAAA HTTP/1.1
Host: stream.example
Accept: text/event-stream
Authorization: WIA-Sig keyid="…",signature="…"

HTTP/1.1 200 OK
Content-Type: text/event-stream
Cache-Control: no-store

id: fr_AAAB
event: frame
data: { "wia_vital_sign_streaming_version":"1.0.0", "type":"frame", "channel":"ecg", "...":"..." }

id: fr_AAAC
event: frame
data: { "wia_vital_sign_streaming_version":"1.0.0", "type":"frame", "channel":"ecg", "...":"..." }
```

## Appendix B — Conformance Test Suite

A black-box test suite is published at
`https://github.com/WIA-Official/wia-vital-sign-streaming-conformance`
and walks through:

1. Discovery document round-trip.
2. Stream subscribe over SSE with `Last-Event-Id` recovery after a
   forced disconnect at frame 250 of 1 000.
3. Snapshot endpoint returning the most recent value per channel.
4. Replay endpoint cursor pagination across 60 minutes of 250 Hz ECG.
5. Alert submission with idempotency key collision.
6. Rate-limit headers and exhaustion behaviour at 50 rps.
7. Authentication failure paths returning RFC 9457 problem details.
8. Cross-origin pre-flight cache for read endpoints.

Hosts publishing `bridge_profile=Full` SHOULD additionally pass the
suite's EHR bridge tests for at least one supported FHIR vendor.

## Appendix C — Operational Recommendations

* Hosts MUST persist accepted alert envelopes durably before
  acknowledging them; an at-most-once acknowledgement followed by
  storage loss would silently drop a clinical alert.
* Hosts SHOULD provide a separate read-replica endpoint for replay
  reads so subscribe traffic stays responsive during incident
  investigations that pull large historical windows.
* Hosts SHOULD expose a `/vss/health` liveness probe outside the
  public rate-limit accounting; tooling consumes it but it MUST NOT be
  advertised in `/.well-known/wia-vital-sign-streaming`.
* Hosts SHOULD retain problem-detail responses emitted to identified
  peers for 30 days to support cross-host debugging during disputes
  about missed alerts or dropped frames.

## Appendix D — Health and Readiness Probes

Hosts SHOULD expose three liveness signals on a separate port:

| Probe | Path | Payload |
|-------|------|---------|
| Liveness  | `/healthz`  | `{"status":"ok"}` |
| Readiness | `/readyz`   | `{"deps":{"db":"ok","queue":"ok"}}` |
| Startup   | `/startupz` | `{"phase":"warming","ready_at":"2026-…"}` |

These MUST NOT be advertised in
`/.well-known/wia-vital-sign-streaming` and MUST NOT be subject to
public rate limits.

## Appendix E — Backwards Compatibility Promise

Within the 1.x line every endpoint listed in this document MUST remain
reachable and MUST continue to honour the documented status codes and
content shapes. Hosts MAY add optional query parameters, response
fields, new endpoints, or media types. Hosts MUST NOT remove or
repurpose existing ones. Breaking changes ride a major version bump
and MUST be preceded by a 12-month deprecation window per IETF
RFC 8594 and RFC 9745; deprecated routes MUST emit `Deprecation` and
`Sunset` headers throughout the window.

## Appendix F — Reserved Endpoint Paths

Implementations MUST NOT serve unrelated content under the following
prefixes; future minor versions reserve them for additional features.

| Prefix | Reserved for |
|--------|--------------|
| `/vss/audit/`     | Future audit-log read endpoint |
| `/vss/integrity/` | Future cryptographic transparency log |
| `/vss/regulator/` | Future regulator inbound feed |
| `/vss/research/`  | Future de-identified research export |
| `/vss/family/`    | Future patient-family snapshot endpoint |

Hosts that need to surface unrelated tooling on the same origin MUST
use a distinct subdomain or path prefix outside the reserved namespace.

## Appendix G — Operational Telemetry

A conformant host SHOULD expose Prometheus-style metrics on a separate
port:

| Metric | Type | Labels |
|--------|------|--------|
| `wia_vss_subscribe_open{stream_id}` | gauge | per-stream open subscribers |
| `wia_vss_frames_emitted_total{channel}` | counter | per-channel emission |
| `wia_vss_alert_routed_total{level}` | counter | by severity |
| `wia_vss_replay_seconds_served_total` | counter | total replay byte-load |
| `wia_vss_consent_violations_total` | counter | refused subscriptions |
| `wia_vss_request_duration_seconds` | histogram | route, status |

Telemetry MUST NOT include patient identifiers as label values; doing
so would violate the privacy promise of Phase 3 §Audience Controls.

## Appendix H — References

* IETF RFC 8446 — TLS 1.3
* IETF RFC 6797 — HSTS
* IETF RFC 9421 — HTTP Message Signatures
* IETF RFC 9457 — Problem Details for HTTP APIs
* IETF RFC 9745 — HTTP Deprecation header
* IETF RFC 8594 — HTTP Sunset header
* WHATWG HTML — Server-Sent Events
* IEEE 11073-10408 — Personal health device thermometers
* HL7 FHIR R5 — Observation resource

---

**Copyright 2025 WIA / SmileStory Inc.**
**License:** MIT
**弘益人間 · Benefit All Humanity**
