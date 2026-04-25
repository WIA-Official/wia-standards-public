# WIA-AI-019 PHASE 3: Protocol Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25

## Overview

This specification defines communication protocols for edge AI systems including model distribution, telemetry collection, and federated learning coordination.

## Model Distribution Protocol

### Model Discovery

**Endpoint:** `GET /api/v1/models`

**Request:**
```json
{
  "deviceInfo": {
    "platform": "ios|android|linux",
    "architecture": "arm64|x86_64",
    "accelerators": ["npu", "gpu"]
  },
  "filter": {
    "category": "image-classification|object-detection|nlp",
    "maxSize": 50000000, // bytes
    "minAccuracy": 0.90
  }
}
```

**Response:**
```json
{
  "models": [
    {
      "modelId": "model-123",
      "version": "1.2.3",
      "name": "MobileNet V3",
      "category": "image-classification",
      "size": 25000000,
      "accuracy": 0.923,
      "downloadUrl": "https://cdn.example.com/models/model-123-v1.2.3.tflite",
      "checksum": "sha256:abc123...",
      "metadata": "https://cdn.example.com/models/model-123-v1.2.3.json"
    }
  ],
  "nextPage": "https://api.example.com/models?page=2"
}
```

### Model Download

**Endpoint:** `GET /api/v1/models/{modelId}/download`

**Request Headers:**
```
Authorization: Bearer <token>
Accept-Encoding: gzip
```

**Response Headers:**
```
Content-Type: application/octet-stream
Content-Length: 25000000
Content-Disposition: attachment; filename="model-123-v1.2.3.tflite"
X-Checksum-SHA256: abc123...
X-Model-Version: 1.2.3
```

**Response:** Binary model file

### Version Checking

**Endpoint:** `GET /api/v1/models/{modelId}/version`

**Response:**
```json
{
  "modelId": "model-123",
  "currentVersion": "1.2.3",
  "latestVersion": "1.3.0",
  "updateAvailable": true,
  "updateSize": 5000000,
  "updateType": "minor",
  "releaseNotes": "Improved accuracy by 2%, reduced latency by 10%",
  "releaseDate": "2025-12-20T00:00:00Z"
}
```

## Telemetry Protocol

### Event Submission

**Endpoint:** `POST /api/v1/telemetry/events`

**Request:**
```json
{
  "deviceId": "hashed-device-id",
  "events": [
    {
      "type": "inference",
      "timestamp": "2025-12-25T10:00:00.123Z",
      "modelId": "model-123",
      "modelVersion": "1.2.3",
      "data": {
        "latency": 25.3,
        "confidence": 0.95,
        "accelerator": "npu",
        "success": true
      }
    }
  ],
  "metadata": {
    "platform": "ios",
    "osVersion": "17.2",
    "appVersion": "2.1.0"
  }
}
```

**Response:**
```json
{
  "accepted": 100,
  "rejected": 0,
  "nextFlushAfter": 300 // seconds
}
```

### Aggregate Query

**Endpoint:** `GET /api/v1/telemetry/aggregates`

**Request:**
```json
{
  "modelId": "model-123",
  "metric": "latency|accuracy|throughput",
  "aggregation": "p50|p95|p99|mean",
  "startTime": "2025-12-20T00:00:00Z",
  "endTime": "2025-12-25T00:00:00Z",
  "groupBy": "platform|accelerator|version"
}
```

**Response:**
```json
{
  "metric": "latency",
  "aggregation": "p95",
  "groups": [
    {
      "key": "ios-npu",
      "value": 28.5,
      "count": 1500000
    },
    {
      "key": "android-gpu",
      "value": 45.2,
      "count": 800000
    }
  ]
}
```

## Federated Learning Protocol

### Training Round Invitation

**Endpoint:** `GET /api/v1/federated/rounds/current`

**Response:**
```json
{
  "roundId": "round-456",
  "roundNumber": 42,
  "modelId": "model-123",
  "baseModelVersion": "1.2.3",
  "baseModelUrl": "https://cdn.example.com/fl/model-123-round-41.tflite",
  "trainingConfig": {
    "epochs": 5,
    "batchSize": 32,
    "learningRate": 0.001,
    "minSamples": 100
  },
  "deadline": "2025-12-26T00:00:00Z",
  "eligibility": {
    "minBatteryLevel": 50,
    "requireWiFi": true,
    "requireCharging": false
  }
}
```

### Update Submission

**Endpoint:** `POST /api/v1/federated/rounds/{roundId}/updates`

**Request:**
```json
{
  "deviceId": "hashed-device-id",
  "modelUpdate": "<base64-encoded-gradients-or-weights>",
  "numSamples": 1000,
  "trainingMetrics": {
    "loss": 0.234,
    "accuracy": 0.91,
    "epochs": 5,
    "duration": 120 // seconds
  },
  "dataDistribution": {
    "classes": [100, 150, 200, 50, ...],
    "total": 1000
  }
}
```

**Response:**
```json
{
  "accepted": true,
  "contribution": 0.0015, // percentage of total round
  "estimatedAggregationTime": "2025-12-26T06:00:00Z"
}
```

### Global Model Download

**Endpoint:** `GET /api/v1/federated/rounds/{roundId}/globalModel`

**Response Headers:**
```
Content-Type: application/octet-stream
X-Round-Number: 42
X-Model-Version: 1.2.4
X-Participants: 50000
X-Accuracy-Improvement: 0.02
```

**Response:** Binary model file

## Privacy-Preserving Protocols

### Differential Privacy

Parameters for adding noise to updates:
```json
{
  "mechanism": "gaussian|laplace",
  "epsilon": 1.0, // privacy budget
  "delta": 1e-5,
  "sensitivity": 0.1,
  "clipNorm": 1.0
}
```

### Secure Aggregation

Protocol for encrypted aggregation:
1. **Key Exchange:** Devices exchange public keys
2. **Masking:** Each device adds pairwise masks to update
3. **Submission:** Encrypted updates sent to server
4. **Aggregation:** Server aggregates without decrypting individual updates
5. **Result:** Masks cancel out, revealing only aggregate

## Authentication and Authorization

### Device Authentication

**Endpoint:** `POST /api/v1/auth/device`

**Request:**
```json
{
  "deviceId": "unique-device-id",
  "publicKey": "<base64-encoded-public-key>",
  "attestation": "<platform-specific-attestation>"
}
```

**Response:**
```json
{
  "accessToken": "jwt-token",
  "refreshToken": "refresh-token",
  "expiresIn": 3600, // seconds
  "permissions": ["model:download", "telemetry:submit", "federated:participate"]
}
```

### Token Refresh

**Endpoint:** `POST /api/v1/auth/refresh`

**Request:**
```json
{
  "refreshToken": "refresh-token"
}
```

**Response:**
```json
{
  "accessToken": "new-jwt-token",
  "expiresIn": 3600
}
```

## Error Responses

### Standard Error Format

```json
{
  "error": {
    "code": "MODEL_NOT_FOUND",
    "message": "The requested model does not exist",
    "details": {
      "modelId": "model-999",
      "suggestion": "Check available models at /api/v1/models"
    },
    "timestamp": "2025-12-25T10:00:00Z",
    "requestId": "req-abc-123"
  }
}
```

### HTTP Status Codes

- `200 OK` - Successful request
- `201 Created` - Resource created
- `400 Bad Request` - Invalid request format
- `401 Unauthorized` - Authentication required
- `403 Forbidden` - Insufficient permissions
- `404 Not Found` - Resource not found
- `429 Too Many Requests` - Rate limit exceeded
- `500 Internal Server Error` - Server error
- `503 Service Unavailable` - Temporary outage

## Rate Limiting

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1640433600
```

Limits:
- Model downloads: 10 per hour per device
- Telemetry submission: 100 events per minute
- Federated updates: 1 per training round

## Versioning

API version in URL path: `/api/v1/...`
- v1: Current stable version
- Breaking changes require new major version

## Security Requirements

1. **TLS 1.3:** All communications MUST use TLS 1.3 or higher
2. **Certificate Pinning:** Clients SHOULD pin server certificates
3. **Request Signing:** Critical operations MUST include request signatures
4. **Replay Protection:** Requests MUST include nonces or timestamps
5. **Input Validation:** All inputs MUST be validated server-side

---



## Reference Standards Alignment

The Phase 3 protocols are layered above well-established IoT and IT primitives so that conformant implementations interoperate at the wire.

| Concern | Reference |
|---------|-----------|
| Real-Time Transport | RFC 3550 (RTP) |
| RTP Profiles | RFC 3551 |
| Secure RTP | RFC 3711 (SRTP) |
| DTLS for SRTP keying | RFC 5764 (DTLS-SRTP) |
| QUIC | RFC 9000 |
| HTTP/3 over QUIC | RFC 9114 |
| WebSocket | RFC 6455 |
| Server-Sent Events | W3C EventSource |
| MQTT 5.0 | OASIS MQTT 5.0 |
| MQTT-SN | OASIS MQTT-SN |
| OPC UA | IEC 62541 series |
| Modbus | Modbus Application Protocol Specification V1.1b3, Modbus over Serial Line Specification V1.02 |
| BACnet | ASHRAE 135-2020 / ISO 16484-5 |
| Industrial real-time Ethernet | IEC 61784 series |
| TSN | IEEE 802.1 Time-Sensitive Networking |
| TLS 1.3 | RFC 8446 |
| DTLS 1.3 | RFC 9147 |
| Time synchronisation (sub-µs) | IEEE 1588-2019 (PTPv2) |
| Time synchronisation (ms) | RFC 5905 (NTPv4) |
| Provenance | W3C PROV-O |
| Sensor metadata | W3C SOSA/SSN |
| OpenTelemetry | OpenTelemetry Specification (CNCF) |
| Time encoding | ISO 8601:2019 |

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

## Conformance

A Phase 3 implementation is conformant when:

1. At least one wire protocol from the §Reference Standards Alignment is offered for telemetry and control plane traffic.
2. TLS 1.3 (or DTLS 1.3 for UDP) is supported, with TLS 1.2 as the minimum acceptable version.
3. Time synchronisation regime is declared per IEEE 1588-2019 PTPv2 or RFC 5905 NTPv4.
4. Provenance assertions follow W3C PROV-O.
5. Telemetry uses OpenTelemetry semantic conventions.

## Implementation Appendix

### A. Latency budgets

Edge AI deployments publish their measured latency budget across capture, inference, transport, and rendering. The reference deployment achieves the budget on commodity edge hardware when each stage is measured with the timers documented in the reference instrumentation.

### B. Backpressure

Telemetry and inference streams expose backpressure semantics so that downstream consumers can throttle when overwhelmed. The reference protocol uses HTTP 429 with `Retry-After` for HTTP-based streams and MQTT QoS 1 with publish-rate limiting for MQTT-based streams.

### C. Network resilience

The protocol assumes unreliable edge networks. Key resilience properties include out-of-order delivery handling, duplicate detection by sequence numbers, store-and-forward buffering during disconnection, and automatic reconciliation on reconnection.

### D. Multi-Sensor Synchronisation

Edge deployments often combine inputs from multiple sensors. Synchronisation across sensors uses two regimes: sub-microsecond IEEE 1588-2019 PTPv2 with hardware timestamping, and millisecond RFC 5905 NTPv4 software timestamping. The active synchronisation regime is recorded in the metadata so downstream consumers know which uncertainty bound to apply.

### E. Industrial Network Compatibility

For factory and machine-vision deployments where edge-AI inference shares a network with PLCs and motion controllers, the implementation supports IEC 61784-2 Real-time Ethernet profiles (EtherCAT IEC 61158-3-12, PROFINET IRT IEC 61158-3-10), IEC 62439-3 PRP/HSR for fault-tolerant cells, and IEEE 802.1 TSN for converged IT/OT links.

### F. MQTT 5.0 Profiles

For fleet telemetry over MQTT, the reference profile uses QoS 1 for at-least-once delivery, retain flags for last-known-state topics, topic alias and shared subscription features of MQTT 5.0, and TLS 1.3 with mutual-TLS authentication for sensor-to-broker pairing.

### G. Diagnostic Logging

All security-relevant events are emitted in structured form per RFC 5424 syslog severity classes, including the `instance` URI from the matching RFC 9457 problem-details object so traces link control-plane errors with data-plane drops.

### H. Compliance Levels

Three compliance levels are defined: Level 1 (Basic) for single-sensor inference with optional encryption, Level 2 (Standard) for multi-sensor synchronisation with TLS encryption, and Level 3 (Advanced) for hardware synchronisation, end-to-end encryption, and real-time FEC.

### I. Reference Test Vectors

The Phase 3 conformance test suite covers wire interoperability against the reference test endpoints. Each test vector documents the input, the expected output, the active synchronisation regime, and the active compliance level so that any implementation can self-assess against the same baseline.

### J. Cross-Reference

Phase 3 protocol parameters interact with Phase 2 API rate limits and Phase 4 deployment topology choices. The reference deployment publishes a cross-reference matrix so operators can match wire profile to deployment scale and security posture without reading the full specification end-to-end.

### K. Future Compatibility

Phase 3 explicitly accepts that wire-protocol primitives will evolve. Implementations declare the wire-protocol version they offer in the metadata so that capability discovery (Phase 2 §I) can negotiate compatible version pairs prior to high-volume traffic.

---

**Copyright © 2025 World Certification Industry Association (WIA)**
**License:** CC BY 4.0
**弘益人間** - Benefit All Humanity
