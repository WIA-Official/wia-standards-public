# WIA-SEMI-001: Phase 2 - API Interface Standards

Version: 1.0
Status: Final
Date: 2025-01-15

## Overview

Phase 2 defines standardized software interfaces for semiconductor control and monitoring. Building on Phase 1's data formats, Phase 2 enables unified programmatic access to chip functionality through RESTful APIs, driver interfaces, and SDKs.

## Design Principles

1. **RESTful Architecture**: HTTP-based APIs using standard methods
2. **Stateless**: Each request contains all necessary information
3. **Resource-Oriented**: Expose chip capabilities as resources
4. **Versioned**: API versions in URL path
5. **Self-Documenting**: OpenAPI/Swagger specifications
6. **Security-First**: Built-in authentication and authorization

## Base API Structure

### URL Format

```
https://{chip-hostname}/api/v{version}/{resource}/{action}
```

Examples:
- `https://soc.local/api/v1/chip/info`
- `https://192.168.1.100/api/v1/monitor/temperature`

### API Versioning

- Version in URL path: `/api/v1/`
- Current version: v1
- Future versions: v2, v3, etc.
- Backward compatibility maintained for 2 major versions

## Core API Endpoints

### 1. Information APIs (Read-Only)

#### GET /api/v1/chip/info

Returns complete chip specification in WIA-SEMI-001 format.

**Request:**
```http
GET /api/v1/chip/info HTTP/1.1
Host: soc.local
Authorization: Bearer {token}
```

**Response:**
```json
{
  "wiaVersion": "1.0",
  "metadata": {
    "id": "WIA-SOC-2025-001",
    "productName": "ExampleSoC Gen 5"
  },
  "architecture": { ... },
  "power": { ... }
}
```

#### GET /api/v1/chip/capabilities

Lists available features and optional capabilities.

**Response:**
```json
{
  "cpu": {
    "cores": 8,
    "features": ["DVFS", "power-gating", "SMT"]
  },
  "gpu": {
    "present": true,
    "features": ["ray-tracing", "VRS"]
  },
  "npu": {
    "present": true,
    "performance": {"value": 50, "unit": "TOPS"}
  }
}
```

#### GET /api/v1/chip/architecture

Detailed architecture information.

#### GET /api/v1/chip/interfaces

Available I/O interfaces and their configurations.

### 2. Configuration APIs

#### PUT /api/v1/config/power-mode

Set power/performance mode.

**Request:**
```http
PUT /api/v1/config/power-mode HTTP/1.1
Content-Type: application/json

{
  "mode": "performance",
  "sustainedDuration": 60,
  "autoRevert": true
}
```

**Modes:**
- `efficiency`: Minimum power consumption
- `balanced`: Balance between power and performance
- `performance`: Maximum performance

**Response:**
```json
{
  "status": "success",
  "currentMode": "performance",
  "effectiveAt": "2025-01-15T10:30:00Z",
  "estimatedPower": {"value": 10.5, "unit": "W"}
}
```

#### PUT /api/v1/config/frequency

Set frequency limits for CPU/GPU.

**Request:**
```json
{
  "cpu": {
    "min": 1000,
    "max": 3500,
    "unit": "MHz"
  },
  "gpu": {
    "min": 400,
    "max": 1000,
    "unit": "MHz"
  }
}
```

#### PUT /api/v1/config/thermal-limit

Set thermal throttling threshold.

**Request:**
```json
{
  "threshold": 90,
  "unit": "°C",
  "action": "gradual-throttle"
}
```

#### PUT /api/v1/config/power-limit

Set maximum power consumption.

**Request:**
```json
{
  "limit": 10,
  "unit": "W",
  "enforcement": "hard"
}
```

### 3. Monitoring APIs

#### GET /api/v1/monitor/power

Current power consumption.

**Response:**
```json
{
  "timestamp": "2025-01-15T10:30:15.234Z",
  "total": {"value": 8.5, "unit": "W"},
  "breakdown": {
    "cpu": {"value": 4.2, "unit": "W"},
    "gpu": {"value": 3.1, "unit": "W"},
    "other": {"value": 1.2, "unit": "W"}
  }
}
```

#### GET /api/v1/monitor/temperature

Temperature from all sensors.

**Response:**
```json
{
  "timestamp": "2025-01-15T10:30:15.234Z",
  "sensors": [
    {
      "location": "CPU-Core0",
      "temperature": {"value": 72.5, "unit": "°C"},
      "status": "normal"
    },
    {
      "location": "GPU",
      "temperature": {"value": 68.2, "unit": "°C"},
      "status": "normal"
    }
  ]
}
```

#### GET /api/v1/monitor/frequency

Current operating frequencies.

**Response:**
```json
{
  "timestamp": "2025-01-15T10:30:15.234Z",
  "cpu": {
    "cluster0": [3200, 3150],
    "cluster1": [1800, 1750, 1800, 1750],
    "unit": "MHz"
  },
  "gpu": {"value": 950, "unit": "MHz"}
}
```

#### GET /api/v1/monitor/utilization

Component utilization percentages.

**Response:**
```json
{
  "timestamp": "2025-01-15T10:30:15.234Z",
  "cpu": {
    "overall": 45,
    "perCore": [85, 82, 15, 12, 20, 18, 25, 22]
  },
  "gpu": 78,
  "npu": 0,
  "memory": 62,
  "unit": "%"
}
```

#### GET /api/v1/monitor/counters

Performance counters and statistics.

## WebSocket Streaming API

For high-frequency monitoring (up to 1000 Hz):

### Connection

```javascript
const ws = new WebSocket('wss://soc.local/api/v1/stream/telemetry');
```

### Subscribe

```javascript
ws.send(JSON.stringify({
  action: 'subscribe',
  metrics: ['power', 'temperature', 'frequency'],
  sampleRate: 100 // Hz
}));
```

### Receive Data

```javascript
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  // {timestamp, power, temperature, frequency}
};
```

## Authentication and Security

### Supported Methods

1. **API Key** (Development)
   ```http
   Authorization: ApiKey your-api-key
   ```

2. **OAuth 2.0** (Production)
   ```http
   Authorization: Bearer eyJhbGciOiJSUzI1NiIs...
   ```

3. **JWT Token** (Recommended)
   ```http
   Authorization: Bearer eyJhbGciOiJIUzI1NiIs...
   ```

4. **Mutual TLS** (High Security)
   - Client certificate required
   - Certificate-based authentication

### Authorization Scopes

- `chip:read` - Read chip information
- `chip:monitor` - Access monitoring data
- `chip:config` - Modify configuration
- `chip:debug` - Debug features
- `chip:admin` - Full access

### OAuth 2.0 Token Request

```http
POST /oauth/token HTTP/1.1
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=your_client_id
&client_secret=your_client_secret
&scope=chip:read chip:monitor
```

## Error Handling

### Standard Error Response

```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "Frequency exceeds maximum",
    "details": {
      "parameter": "maxFrequency",
      "requestedValue": 4500,
      "maximumValue": 3800
    },
    "timestamp": "2025-01-15T10:30:15Z",
    "requestId": "req_1234567890"
  }
}
```

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| INVALID_PARAMETER | 400 | Invalid request parameter |
| UNAUTHORIZED | 401 | Missing/invalid authentication |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource doesn't exist |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| THERMAL_THROTTLED | 503 | Operation unavailable due to thermal state |

## Driver Interfaces

### Linux Driver

```c
#include <linux/wia-semi.h>

struct wia_semi_chip_info {
    const char *manufacturer;
    const char *product_name;
    const char *chip_id;
};

struct wia_semi_pm_ops {
    int (*set_power_mode)(struct device *dev, enum wia_power_mode mode);
    int (*get_power_consumption)(struct device *dev, u32 *milliwatts);
};

int wia_semi_register_driver(struct wia_semi_driver *driver);
```

### Windows Driver

```c
#include <wiasemi.h>

typedef struct _WIA_SEMI_CHIP_INFO {
    UNICODE_STRING Manufacturer;
    UNICODE_STRING ProductName;
    UNICODE_STRING ChipId;
} WIA_SEMI_CHIP_INFO;

#define IOCTL_WIA_SEMI_GET_INFO \
    CTL_CODE(FILE_DEVICE_UNKNOWN, 0x800, METHOD_BUFFERED, FILE_ANY_ACCESS)
```

## SDK Reference

### TypeScript SDK

```typescript
import { WIASemiChip, PowerMode } from '@wia/semi-sdk';

const chip = new WIASemiChip({
  host: 'soc.local',
  apiKey: process.env.WIA_API_KEY
});

// Get info
const info = await chip.getInfo();

// Set power mode
await chip.config.setPowerMode(PowerMode.Performance);

// Monitor telemetry
chip.monitor.on('telemetry', (data) => {
  console.log(`Power: ${data.power.value}W`);
});
```

### Python SDK

```python
from wia_semi import WIASemiChip, PowerMode

chip = WIASemiChip(host='soc.local', api_key=os.environ['WIA_API_KEY'])
info = chip.get_info()
chip.config.set_power_mode(PowerMode.PERFORMANCE)

for telemetry in chip.monitor.stream_telemetry(sample_rate=100):
    print(f"Power: {telemetry['power']['value']}W")
```

## Rate Limiting

- Information APIs: 100 requests/minute
- Configuration APIs: 10 requests/minute
- Monitoring APIs: 1000 requests/minute
- WebSocket streaming: 1 connection per client

## Compliance Requirements

To be Phase 2 compliant:

1. ✓ Implement all Information APIs
2. ✓ Implement Configuration APIs (power-mode required, others optional)
3. ✓ Implement Monitoring APIs (power, temperature required)
4. ✓ Support at least one authentication method
5. ✓ Provide OpenAPI specification
6. ✓ Implement standard error responses
7. ✓ Support API versioning

## Testing

Use the official test suite:

```bash
npm install -g wia-semi-api-test
wia-semi-api-test --host soc.local --api-key your-key
```

---

**Previous**: [Phase 1 - Data Format](PHASE-1-DATA-FORMAT.md)
**Next**: [Phase 3 - Protocol Implementation](PHASE-3-PROTOCOL.md)

© 2025 SmileStory Inc. / WIA
弘益人間 - Benefit All Humanity

## P.2 API Surface Cross-References

The API surface defined in this Phase consumes and emits the data formats from
Phase 1 and is transported by the protocol layer in Phase 3. Operators deploy
the surface using the integration patterns in Phase 4.

### P.2.1 Resource Naming

Resource paths follow REST conventions with snake_case segments. Identifier
segments use the canonical UUID encoding from Phase 1.

```
/v1/{collection}                        # collection
/v1/{collection}/{id}                    # member
/v1/{collection}/{id}/{sub_collection}   # nested collection
/v1/{collection}/{id}:{action}           # custom action (POST)
```

### P.2.2 Pagination

List endpoints support cursor-based pagination:

| Param | Default | Max | Description |
|-------|---------|-----|-------------|
| `page_size` | 50 | 500 | Items per page |
| `page_token` | empty | — | Opaque continuation token |

Servers MUST return `next_page_token` when the result set is truncated and an
empty string when the final page has been delivered.

### P.2.3 Idempotency

State-changing operations accept the `Idempotency-Key` header (RFC-style).
Servers MUST cache the response keyed by `(principal, key)` for at least 24 h
and replay the same response on retry.

### P.2.4 Field Masks

Partial-update operations use field masks (Google AIP-161 style) to avoid
clobbering unspecified fields. Masks are dot-paths into the canonical schema
with `*` wildcards.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of system-semiconductor so that conformance claims at any
Phase remain unambiguous.*

