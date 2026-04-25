# WIA-SOC-007 Phase 3: Communication Protocol Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 3 defines the communication protocols, security requirements, and network specifications for public transportation systems. This includes HTTPS/TLS, WebSocket, MQTT, and device discovery protocols.

## 2. Transport Layer Security

### 2.1 TLS Requirements
- **Minimum Version**: TLS 1.3
- **Certificate**: Valid SSL/TLS certificate from recognized CA
- **Cipher Suites**: Only strong ciphers (AES-256-GCM, ChaCha20-Poly1305)
- **HSTS**: Enforce HTTPS with Strict-Transport-Security header

### 2.2 Certificate Pinning (Mobile Apps)
```javascript
const pinnedCertificates = [
  'sha256/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=',
  'sha256/BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB='
];
```

## 3. HTTP Protocol Specifications

### 3.1 HTTP/2 Support
All API endpoints MUST support HTTP/2 for improved performance:
- Server push for critical resources
- Header compression
- Multiplexing multiple requests

### 3.2 CORS Configuration
```http
Access-Control-Allow-Origin: *
Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS
Access-Control-Allow-Headers: Content-Type, Authorization, X-API-Key
Access-Control-Max-Age: 86400
```

### 3.3 Compression
Support gzip and brotli compression:
```http
Content-Encoding: br
Accept-Encoding: br, gzip, deflate
```

## 4. WebSocket Protocol

### 4.1 Connection Establishment
```
wss://api.transit.example.com/v1/realtime
```

### 4.2 Heartbeat/Keepalive
- Client sends ping every 30 seconds
- Server responds with pong
- Connection terminates if no pong within 60 seconds

```json
{
  "type": "ping",
  "timestamp": "2025-12-26T14:32:15Z"
}
```

### 4.3 Subscription Management
```json
{
  "type": "subscribe",
  "channels": [
    "route:101",
    "stop:12345",
    "vehicle:bus-1234",
    "alerts:all"
  ],
  "apiKey": "your-api-key"
}
```

### 4.4 Message Acknowledgment
```json
{
  "type": "ack",
  "messageId": "msg-123456",
  "status": "received"
}
```

## 5. MQTT Protocol (IoT Devices)

### 5.1 Broker Configuration
```
Host: mqtt.transit.example.com
Port: 8883 (TLS), 1883 (non-TLS, internal only)
Protocol: MQTT 5.0
```

### 5.2 Topic Structure
```
transit/{agency_id}/vehicles/{vehicle_id}/position
transit/{agency_id}/vehicles/{vehicle_id}/status
transit/{agency_id}/stops/{stop_id}/arrivals
transit/{agency_id}/alerts
transit/{agency_id}/system/health
```

### 5.3 QoS Levels
- **QoS 0**: Vehicle positions (acceptable loss)
- **QoS 1**: Arrival predictions (at least once delivery)
- **QoS 2**: Payment transactions (exactly once delivery)

### 5.4 Retained Messages
```
Topic: transit/mta/system/status
Payload: {"status": "operational", "timestamp": "2025-12-26T14:32:15Z"}
Retained: true
```

## 6. Authentication & Authorization

### 6.1 API Key Authentication
```http
X-API-Key: wia_apikey_<32+chars>   # opaque token issued by the operator
```

### 6.2 JWT Token Structure
```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT"
  },
  "payload": {
    "sub": "user-12345",
    "iss": "https://auth.transit.example.com",
    "aud": "https://api.transit.example.com",
    "exp": 1640000000,
    "iat": 1639999700,
    "scope": "read:trips write:favorites"
  }
}
```

### 6.3 OAuth 2.0 Flow
```
Authorization Endpoint: https://auth.transit.example.com/oauth/authorize
Token Endpoint: https://auth.transit.example.com/oauth/token
Grant Types: authorization_code, refresh_token, client_credentials
```

## 7. Data Encryption

### 7.1 At Rest
- **Algorithm**: AES-256-GCM
- **Key Management**: AWS KMS, Google Cloud KMS, or Azure Key Vault
- **Encryption**: All PII and payment data

### 7.2 In Transit
- **TLS 1.3**: All API communications
- **Perfect Forward Secrecy**: Ephemeral key exchange
- **Certificate Transparency**: CT logs required

## 8. Rate Limiting & Throttling

### 8.1 Token Bucket Algorithm
```python
class RateLimiter:
    def __init__(self, rate, capacity):
        self.rate = rate  # tokens per second
        self.capacity = capacity  # max tokens
        self.tokens = capacity
        self.last_update = time.time()
    
    def allow_request(self):
        now = time.time()
        elapsed = now - self.last_update
        self.tokens = min(self.capacity, self.tokens + elapsed * self.rate)
        self.last_update = now
        
        if self.tokens >= 1:
            self.tokens -= 1
            return True
        return False
```

### 8.2 Rate Limit Headers
```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 87
X-RateLimit-Reset: 1640000000
Retry-After: 60
```

## 9. Service Discovery

### 9.1 DNS-SD (Bonjour/Avahi)
```
_transit._tcp.local. PTR mta-api._transit._tcp.local.
mta-api._transit._tcp.local. SRV 0 0 443 api.transit.example.com.
mta-api._transit._tcp.local. TXT "version=1.0" "protocol=https"
```

### 9.2 mDNS Advertisement
```
Name: Transit API
Type: _transit._tcp
Port: 443
TXT: version=1.0,agency=MTA,realtime=true
```

## 10. Error Handling & Retry Logic

### 10.1 Exponential Backoff
```javascript
async function retryWithBackoff(fn, maxRetries = 3) {
  for (let i = 0; i < maxRetries; i++) {
    try {
      return await fn();
    } catch (error) {
      if (i === maxRetries - 1) throw error;
      const delay = Math.min(1000 * Math.pow(2, i), 10000);
      await new Promise(resolve => setTimeout(resolve, delay));
    }
  }
}
```

### 10.2 Circuit Breaker Pattern
```javascript
class CircuitBreaker {
  constructor(threshold, timeout) {
    this.threshold = threshold;
    this.timeout = timeout;
    this.failures = 0;
    this.state = 'CLOSED';  // CLOSED, OPEN, HALF_OPEN
    this.nextAttempt = Date.now();
  }
  
  async execute(fn) {
    if (this.state === 'OPEN') {
      if (Date.now() < this.nextAttempt) {
        throw new Error('Circuit breaker is OPEN');
      }
      this.state = 'HALF_OPEN';
    }
    
    try {
      const result = await fn();
      this.onSuccess();
      return result;
    } catch (error) {
      this.onFailure();
      throw error;
    }
  }
}
```

## 11. Content Negotiation

### 11.1 Accept Header
```http
Accept: application/json
Accept: application/json; version=1
Accept: application/xml
Accept: application/protobuf
```

### 11.2 Response Format
```http
Content-Type: application/json; charset=utf-8
Content-Type: application/vnd.transit.v1+json
```

## 12. Versioning Strategy

### 12.1 URI Versioning
```
https://api.transit.example.com/v1/routes
https://api.transit.example.com/v2/routes
```

### 12.2 Header Versioning
```http
Accept: application/vnd.transit.v1+json
API-Version: 2025-12-26
```

### 12.3 Deprecation Policy
```http
Deprecation: true
Sunset: Wed, 26 Dec 2026 00:00:00 GMT
Link: <https://docs.transit.example.com/migration>; rel="deprecation"
```

## 13. Monitoring & Logging

### 13.1 Request Tracing
```http
X-Request-ID: req-20251226-143000-abc123
X-Correlation-ID: corr-20251226-143000-xyz789
```

### 13.2 Performance Metrics
```json
{
  "requestId": "req-123",
  "endpoint": "/stops/12345/arrivals",
  "method": "GET",
  "statusCode": 200,
  "responseTime": 45,
  "dataSize": 2048,
  "cacheHit": true,
  "timestamp": "2025-12-26T14:32:15Z"
}
```

### 13.3 Health Check Endpoint
```http
GET /health HTTP/1.1

Response:
{
  "status": "healthy",
  "version": "1.0.0",
  "uptime": 3600000,
  "checks": {
    "database": "healthy",
    "realtime": "healthy",
    "cache": "healthy"
  }
}
```

## 14. Security Best Practices

### 14.1 Input Validation
- Validate all input parameters
- Sanitize user-provided data
- Enforce maximum request sizes
- Reject malformed requests

### 14.2 SQL Injection Prevention
```python
# Good: Parameterized queries
cursor.execute("SELECT * FROM stops WHERE stop_id = ?", (stop_id,))

# Bad: String concatenation
cursor.execute(f"SELECT * FROM stops WHERE stop_id = '{stop_id}'")
```

### 14.3 XSS Prevention
```javascript
// Sanitize output
function escapeHtml(text) {
  return text
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#039;');
}
```

## 15. Bandwidth Optimization

### 15.1 Response Compression
```http
Accept-Encoding: br, gzip
Content-Encoding: br
```

### 15.2 Field Filtering
```http
GET /stops?fields=stopId,stopName,stopLat,stopLon
```

### 15.3 Partial Responses
```http
GET /routes/101
Range: items=0-49
```

---

**Previous**: [Phase 2 - API Interface](PHASE-2-API.md)  
**Next**: [Phase 4 - Integration](PHASE-4-INTEGRATION.md)

弘益人間 - Benefit All Humanity

© 2025 WIA / SmileStory Inc.

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-SOC-007 (Public Transportation) is evaluated across three tiers, applied to schedules · stop inventory · real-time arrivals · accessibility metadata:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | None (annual self-review recommended) |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST clearly disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the following published standards. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- GTFS Reference — General Transit Feed Specification (open spec, MobilityData)
- GTFS-Realtime — protobuf schema for live updates (open spec, MobilityData)
- ISO 17572-3:2018 — Intelligent transport systems — Location referencing
- ISO/IEC 18004:2015 — QR Code (ticket encoding reference)
- IETF RFC 7946 — GeoJSON (stop and shape geometry)

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/public-transportation/api/` — TypeScript SDK skeleton
- `wia-standards/standards/public-transportation/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/public-transportation/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
