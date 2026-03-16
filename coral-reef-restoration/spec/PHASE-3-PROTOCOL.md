# WIA Coral Reef Restoration - Phase 3: Protocol

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-12-25

---

## 1. Overview

This specification defines the communication protocols, data exchange mechanisms, and operational procedures for coral reef restoration systems. The protocol ensures secure, reliable, and interoperable data flow among monitoring stations, research vessels, satellite systems, and conservation organizations.

### 1.1 Protocol Stack

```
┌─────────────────────────────────────┐
│     Application Layer               │  Coral Reef Monitoring App
├─────────────────────────────────────┤
│     Message Format Layer            │  JSON/Protobuf
├─────────────────────────────────────┤
│     Security Layer                  │  TLS 1.3 + OAuth 2.0
├─────────────────────────────────────┤
│     Transport Layer                 │  HTTPS / MQTT / WebSocket
├─────────────────────────────────────┤
│     Network Layer                   │  TCP/IP + IPv6
└─────────────────────────────────────┘
```

### 1.2 Protocol Types

1. **Real-time Streaming:** Continuous sensor data (temperature, pH)
2. **Batch Transfer:** Periodic assessment uploads (daily reports)
3. **Event-driven:** Alerts and notifications (bleaching events)
4. **Request-Response:** API queries and commands

---

## 2. Communication Patterns

### 2.1 Monitoring Station → Cloud

**Pattern:** Publish-Subscribe (MQTT)

```
Topic Structure:
  coral-reef/{region}/{reefId}/{metric}

Examples:
  coral-reef/gbr/REEF-GBR-A0123/temperature
  coral-reef/caribbean/REEF-CAR-B0456/bleaching-alert
  coral-reef/indo-pacific/REEF-INDO-C0789/biodiversity
```

**Message Format:**
```json
{
  "topic": "coral-reef/gbr/REEF-GBR-A0123/temperature",
  "timestamp": "2025-12-25T08:30:00Z",
  "payload": {
    "value": 28.5,
    "unit": "celsius",
    "sensor": "TEMP-SENSOR-001",
    "depth": 12.5,
    "quality": 0.98
  },
  "qos": 1,
  "retain": false
}
```

### 2.2 Cloud → Stakeholders

**Pattern:** WebSocket Subscriptions

```javascript
// Subscribe to reef updates
ws://api.coral-reef.wiastandards.org/v1/subscribe

{
  "action": "subscribe",
  "channels": [
    "reef.REEF-GBR-A0123.health",
    "alerts.bleaching.*",
    "restoration.*.completed"
  ],
  "auth": "Bearer <token>"
}
```

**Update Stream:**
```json
{
  "channel": "reef.REEF-GBR-A0123.health",
  "event": "health.updated",
  "timestamp": "2025-12-25T08:30:15Z",
  "data": {
    "healthScore": 92.5,
    "change": -0.3,
    "trigger": "temperature_increase"
  }
}
```

### 2.3 Peer-to-Peer Research Data

**Pattern:** Direct Transfer (HTTPS + Resumable Upload)

```http
POST /v1/research/data-exchange
Content-Type: multipart/form-data
X-Resume-Token: upload-abc123

{
  "fromOrganization": "ORG-001",
  "toOrganization": "ORG-002",
  "dataType": "underwater-imagery",
  "reefId": "REEF-GBR-A0123",
  "size": 5368709120,
  "checksum": "sha256:abc123...",
  "encrypted": true
}
```

---

## 3. Data Streaming Protocol

### 3.1 Sensor Stream Configuration

**Stream Initialization:**
```json
{
  "protocol": "MQTT",
  "version": "5.0",
  "broker": "mqtt.coral-reef.wiastandards.org:8883",
  "clientId": "STATION-GBR-A0123",
  "keepAlive": 60,
  "cleanSession": false,
  "qos": 1,
  "tls": {
    "enabled": true,
    "version": "1.3",
    "clientCert": "cert.pem",
    "caCert": "ca.pem"
  }
}
```

### 3.2 Stream Message Format

```protobuf
message SensorReading {
  string sensor_id = 1;
  string reef_id = 2;
  int64 timestamp = 3;
  string metric_type = 4;
  double value = 5;
  string unit = 6;
  double latitude = 7;
  double longitude = 8;
  double depth = 9;
  float quality_score = 10;
  map<string, string> metadata = 11;
}
```

### 3.3 Stream Quality of Service

| QoS Level | Description | Use Case |
|-----------|-------------|----------|
| 0 | At most once | Non-critical metrics |
| 1 | At least once | Standard monitoring |
| 2 | Exactly once | Bleaching alerts, critical events |

### 3.4 Stream Compression

```
Supported: gzip, brotli, zstd
Header: Content-Encoding: gzip
Ratio: ~70% reduction for sensor data
```

---

## 4. Alert and Notification Protocol

### 4.1 Alert Severity Levels

```yaml
CRITICAL:
  severity: 5
  response_time: "< 5 minutes"
  escalation: "immediate"
  channels: ["sms", "email", "push", "webhook"]

HIGH:
  severity: 4
  response_time: "< 15 minutes"
  escalation: "1 hour"
  channels: ["email", "push", "webhook"]

MODERATE:
  severity: 3
  response_time: "< 1 hour"
  escalation: "4 hours"
  channels: ["email", "webhook"]

LOW:
  severity: 2
  response_time: "< 4 hours"
  escalation: "24 hours"
  channels: ["email"]

INFO:
  severity: 1
  response_time: "best effort"
  escalation: "none"
  channels: ["webhook"]
```

### 4.2 Alert Message Format

```json
{
  "alertId": "ALERT-2025-12-25-001",
  "alertType": "BLEACHING_RISK",
  "severity": "HIGH",
  "issuedAt": "2025-12-25T10:00:00Z",
  "expiresAt": "2025-12-25T22:00:00Z",
  "reef": {
    "reefId": "REEF-CAR-B0456",
    "name": "Caribbean Reef, Belize",
    "location": {
      "latitude": 17.2512,
      "longitude": -87.5344
    }
  },
  "trigger": {
    "metric": "water_temperature",
    "threshold": 30.0,
    "currentValue": 31.2,
    "duration": "14 days"
  },
  "prediction": {
    "bleachingRisk": 78.5,
    "confidence": 0.89,
    "timeToImpact": "48-72 hours"
  },
  "recommendations": [
    "Increase monitoring frequency to daily",
    "Prepare emergency response team",
    "Notify local diving operators"
  ],
  "acknowledgement": {
    "required": true,
    "deadline": "2025-12-25T12:00:00Z"
  }
}
```

### 4.3 Alert Acknowledgement

```http
POST /v1/alerts/{alertId}/acknowledge

{
  "acknowledgedBy": "OBS-001",
  "acknowledgedAt": "2025-12-25T10:15:00Z",
  "actionTaken": "Deployed additional monitoring equipment",
  "notes": "Daily monitoring scheduled for next 14 days"
}
```

---

## 5. Synchronization Protocol

### 5.1 Offline Data Sync

```json
{
  "syncProtocol": "differential",
  "direction": "bidirectional",
  "lastSync": "2025-12-24T18:00:00Z",
  "conflicts": "server-wins",
  "compression": true,
  "encryption": true
}
```

**Sync Request:**
```json
{
  "deviceId": "DEVICE-MOBILE-123",
  "lastSyncTimestamp": "2025-12-24T18:00:00Z",
  "pendingUploads": [
    {
      "recordId": "MON-LOCAL-001",
      "timestamp": "2025-12-25T09:00:00Z",
      "data": { /* monitoring data */ }
    }
  ],
  "pullUpdates": true
}
```

**Sync Response:**
```json
{
  "syncId": "SYNC-2025-12-25-001",
  "serverTimestamp": "2025-12-25T10:00:00Z",
  "uploadStatus": [
    {
      "recordId": "MON-LOCAL-001",
      "status": "accepted",
      "serverId": "MON-2025-12-25-00123"
    }
  ],
  "updates": [
    {
      "reefId": "REEF-GBR-A0123",
      "timestamp": "2025-12-25T08:30:00Z",
      "changes": { /* updated data */ }
    }
  ],
  "deletions": [],
  "nextSyncRecommended": "2025-12-25T22:00:00Z"
}
```

### 5.2 Conflict Resolution

```javascript
ConflictResolution:
  1. Server-wins (default for monitoring data)
  2. Client-wins (for offline field observations)
  3. Merge (for non-overlapping fields)
  4. Manual (for critical discrepancies)

Example:
{
  "conflict": {
    "field": "coralCoverage",
    "serverValue": 65.5,
    "serverTimestamp": "2025-12-25T08:30:00Z",
    "clientValue": 64.8,
    "clientTimestamp": "2025-12-25T08:32:00Z"
  },
  "resolution": "server-wins",
  "reason": "Server data from certified sensor"
}
```

---

## 6. Security Protocol

### 6.1 Transport Security

```yaml
TLS Configuration:
  version: "1.3"
  cipherSuites:
    - TLS_AES_256_GCM_SHA384
    - TLS_CHACHA20_POLY1305_SHA256
  certificateValidation: "strict"
  ocspStapling: true
  hsts: "max-age=31536000; includeSubDomains"
```

### 6.2 Message Signing

```json
{
  "message": { /* monitoring data */ },
  "signature": {
    "algorithm": "Ed25519",
    "publicKey": "did:wia:observer:OBS-001",
    "signature": "base64-encoded-signature",
    "timestamp": "2025-12-25T08:30:00Z"
  }
}
```

### 6.3 End-to-End Encryption

```
Research Data Exchange:
  1. Sender generates ephemeral keypair
  2. ECDH key agreement with recipient's public key
  3. Derive AES-256-GCM key
  4. Encrypt data + authenticate
  5. Include ephemeral public key in message

Format:
{
  "encrypted": true,
  "algorithm": "ECDH-ES+A256GCM",
  "ephemeralPublicKey": "base64...",
  "ciphertext": "base64...",
  "authTag": "base64...",
  "recipient": "did:wia:org:ORG-002"
}
```

---

## 7. Data Validation Protocol

### 7.1 Input Validation

```javascript
ValidationRules:
  1. Schema validation (JSON Schema)
  2. Range validation (metric bounds)
  3. Format validation (timestamps, IDs)
  4. Logical validation (cross-field checks)
  5. Business rules (certification requirements)

Example:
{
  "validator": "coral-reef-monitoring-v1",
  "rules": {
    "coralCoverage": {
      "type": "number",
      "minimum": 0,
      "maximum": 100,
      "precision": 1
    },
    "waterTemperature": {
      "type": "number",
      "minimum": 0,
      "maximum": 50,
      "unit": "celsius",
      "sensorCalibrated": true
    }
  }
}
```

### 7.2 Data Quality Scoring

```json
{
  "dataQuality": {
    "overallScore": 0.98,
    "components": {
      "accuracy": 0.99,
      "completeness": 0.97,
      "timeliness": 1.0,
      "consistency": 0.98
    },
    "sensorHealth": {
      "calibrationDate": "2025-12-01",
      "batteryLevel": 0.87,
      "signalStrength": 0.95
    },
    "observerCredentials": {
      "certified": true,
      "experienceYears": 8
    }
  }
}
```

---

## 8. Interoperability Protocol

### 8.1 Standard Compliance

```yaml
Standards:
  - ISO 19115: Geographic Information Metadata
  - OGC SensorThings API: Sensor data
  - IOOS DMZ: Marine data interoperability
  - WFS 2.0: Geographic features
  - NetCDF CF-1.8: Climate forecast conventions

Formats:
  - JSON-LD: Linked data
  - GeoJSON: Geographic data
  - CSV: Tabular data export
  - Shapefile: GIS integration
```

### 8.2 Cross-Platform Integration

```json
{
  "integrations": [
    {
      "platform": "NOAA Coral Reef Watch",
      "protocol": "REST API",
      "dataMapping": {
        "waterTemperature": "sea_surface_temperature",
        "bleachingStatus": "bleaching_alert_level"
      },
      "syncFrequency": "hourly"
    },
    {
      "platform": "Global Coral Reef Monitoring Network",
      "protocol": "GraphQL",
      "authentication": "API Key",
      "syncFrequency": "daily"
    }
  ]
}
```

---

## 9. Versioning Protocol

### 9.1 API Versioning

```
Scheme: URI versioning
Format: /v{major}/
Example: /v1/reefs, /v2/reefs

Version Lifecycle:
  - Beta: /vX-beta (unstable)
  - Stable: /vX (production)
  - Deprecated: /vX (sunset notice)
  - End of Life: /vX removed

Deprecation Notice: 12 months minimum
Support: Current + Previous version
```

### 9.2 Protocol Version Negotiation

```http
GET /v1/reefs/REEF-GBR-A0123
Accept: application/vnd.wia.coral-reef.v1+json
X-API-Version: 1.2.0

Response:
200 OK
Content-Type: application/vnd.wia.coral-reef.v1+json
X-API-Version: 1.2.0
X-API-Deprecated: false
```

---

## 10. Error Recovery Protocol

### 10.1 Retry Strategy

```yaml
RetryPolicy:
  maxAttempts: 3
  backoff: exponential
  initialDelay: 1s
  maxDelay: 30s
  jitter: true
  retryableErrors:
    - 408: Request Timeout
    - 429: Rate Limit
    - 500: Server Error
    - 502: Bad Gateway
    - 503: Service Unavailable
    - 504: Gateway Timeout
```

### 10.2 Circuit Breaker

```javascript
CircuitBreaker:
  failureThreshold: 5 (consecutive failures)
  timeout: 10s
  resetTimeout: 60s

States:
  CLOSED: Normal operation
  OPEN: All requests fail immediately
  HALF_OPEN: Test if service recovered

Example:
{
  "circuitBreaker": {
    "state": "OPEN",
    "failures": 5,
    "lastFailure": "2025-12-25T10:00:00Z",
    "nextAttempt": "2025-12-25T10:01:00Z"
  }
}
```

### 10.3 Graceful Degradation

```
Priority Levels:
  1. Critical: Bleaching alerts
  2. High: Real-time monitoring
  3. Medium: Restoration tracking
  4. Low: Historical reports

Degradation Strategy:
  - Under load: Drop low priority requests
  - Partial outage: Cache recent data
  - Full outage: Queue for later sync
```

---

## 11. Monitoring and Observability

### 11.1 Health Check Protocol

```http
GET /health

Response:
{
  "status": "healthy",
  "version": "1.0.0",
  "timestamp": "2025-12-25T10:00:00Z",
  "services": {
    "database": "healthy",
    "messageQueue": "healthy",
    "storage": "healthy"
  },
  "metrics": {
    "uptime": 2592000,
    "requestsPerMinute": 450,
    "errorRate": 0.002
  }
}
```

### 11.2 Metrics Collection

```
Prometheus format:
# HELP coral_reef_api_requests_total Total API requests
# TYPE coral_reef_api_requests_total counter
coral_reef_api_requests_total{method="GET",endpoint="/reefs"} 12450

# HELP coral_reef_bleaching_alerts Active bleaching alerts
# TYPE coral_reef_bleaching_alerts gauge
coral_reef_bleaching_alerts{severity="high"} 3
```

### 11.3 Distributed Tracing

```json
{
  "traceId": "abc123-def456-ghi789",
  "spanId": "span-001",
  "parentSpanId": null,
  "operation": "POST /reefs/monitor",
  "startTime": "2025-12-25T10:00:00.000Z",
  "duration": 145,
  "tags": {
    "reef.id": "REEF-GBR-A0123",
    "http.status": 201
  }
}
```

---

## 12. Compliance and Auditing

### 12.1 Audit Log Protocol

```json
{
  "auditId": "AUDIT-2025-12-25-001",
  "timestamp": "2025-12-25T10:00:00Z",
  "actor": {
    "id": "OBS-001",
    "type": "observer",
    "ip": "203.0.113.42"
  },
  "action": "CREATE",
  "resource": {
    "type": "monitoring_record",
    "id": "MON-2025-12-25-00123"
  },
  "outcome": "success",
  "changes": {
    "before": null,
    "after": { /* monitoring data */ }
  },
  "metadata": {
    "userAgent": "WIA Mobile App v2.1.0",
    "location": "Great Barrier Reef"
  }
}
```

### 12.2 Data Retention

```yaml
RetentionPolicy:
  monitoringData:
    hot: 90 days (fast access)
    warm: 2 years (standard access)
    cold: 10 years (archive)
    deletion: never (historical value)

  alerts:
    active: realtime
    resolved: 5 years

  auditLogs:
    retention: 7 years (compliance)
    immutable: true
```

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
