# WIA Water Scarcity Response Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red - ENE Category)

---

## Table of Contents

1. [Protocol Overview](#protocol-overview)
2. [Communication Protocols](#communication-protocols)
3. [Data Exchange Standards](#data-exchange-standards)
4. [Real-Time Monitoring](#real-time-monitoring)
5. [Alert Distribution](#alert-distribution)
6. [Interoperability](#interoperability)
7. [Security Protocols](#security-protocols)
8. [Compliance](#compliance)

---

## Protocol Overview

### 1.1 Purpose

The WIA Water Scarcity Response Protocol defines standardized communication methods for water resource monitoring, drought prediction, and emergency response coordination across municipal, agricultural, and industrial systems.

### 1.2 Protocol Stack

```
┌─────────────────────────────────────┐
│   Application Layer (REST/GraphQL)  │
├─────────────────────────────────────┤
│   Transport Layer (HTTP/2, WSS)     │
├─────────────────────────────────────┤
│   Security Layer (TLS 1.3, OAuth)   │
├─────────────────────────────────────┤
│   Network Layer (IPv4/IPv6)         │
└─────────────────────────────────────┘
```

### 1.3 Supported Protocols

| Protocol | Use Case | Port |
|----------|----------|------|
| HTTPS | RESTful API | 443 |
| WSS | Real-time data streams | 443 |
| MQTT | IoT sensor data | 8883 |
| CoAP | Constrained devices | 5684 |
| GraphQL | Complex queries | 443 |

---

## Communication Protocols

### 2.1 HTTP/2 REST API

**Standard Headers**:
```http
POST /api/v1/water-levels HTTP/2
Host: api.wia-water.org
Content-Type: application/json
Authorization: Bearer {token}
X-API-Key: {api-key}
X-WIA-Standard: WIA-ENE-053
X-Request-ID: {uuid}
X-Timestamp: {iso8601}
```

**Response Headers**:
```http
HTTP/2 200 OK
Content-Type: application/json
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1642251600
X-Response-Time: 45ms
```

### 2.2 WebSocket Protocol

**Connection Establishment**:
```javascript
// Client initiates WebSocket connection
ws://api.wia-water.org/v1/stream
wss://api.wia-water.org/v1/stream (secure)

// Handshake
GET /v1/stream HTTP/1.1
Host: api.wia-water.org
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Authorization: Bearer {token}
```

**Message Format**:
```json
{
  "type": "water-level-update",
  "timestamp": "2025-01-15T10:00:00Z",
  "source": "source-lakemead",
  "data": {
    "level": 327.5,
    "capacity": 34,
    "trend": "declining"
  }
}
```

### 2.3 MQTT for IoT Sensors

**Topic Structure**:
```
wia/water/{region}/{source-type}/{source-id}/{metric}

Examples:
wia/water/southwest/reservoir/lakemead/level
wia/water/southwest/reservoir/lakemead/consumption
wia/water/southwest/aquifer/sw-001/depth
```

**Message Payload**:
```json
{
  "timestamp": "2025-01-15T10:00:00Z",
  "value": 327.5,
  "unit": "meters",
  "quality": "good",
  "sensor_id": "sensor-12345"
}
```

**QoS Levels**:
- QoS 0: Non-critical metrics (temperature, humidity)
- QoS 1: Standard metrics (water levels, consumption)
- QoS 2: Critical alerts (drought warnings, contamination)

---

## Data Exchange Standards

### 3.1 JSON Data Format

**Standard Message Structure**:
```json
{
  "header": {
    "standard": "WIA-ENE-053",
    "version": "1.0.0",
    "messageId": "msg-12345",
    "timestamp": "2025-01-15T10:00:00Z",
    "source": "system-identifier"
  },
  "payload": {
    "type": "water-monitoring",
    "data": {}
  },
  "signature": "digital-signature-here"
}
```

### 3.2 Protocol Buffers (Protobuf)

For high-performance scenarios:

```protobuf
syntax = "proto3";

message WaterLevel {
  string source_id = 1;
  double level = 2;
  string unit = 3;
  int64 timestamp = 4;
  double capacity_percent = 5;
  string trend = 6;
}

message DroughtAlert {
  string alert_id = 1;
  string region = 2;
  int32 risk_score = 3;
  string severity = 4;
  int64 issued_at = 5;
  repeated string affected_sources = 6;
}
```

### 3.3 GraphQL Schema

```graphql
type WaterSource {
  id: ID!
  name: String!
  type: SourceType!
  location: Location!
  currentLevel: WaterLevel!
  capacity: Capacity!
  trend: Trend!
  alerts: [Alert!]!
  historicalData(timeframe: String!): HistoricalData
}

type Query {
  waterSources(region: String, type: SourceType): [WaterSource!]!
  droughtForecast(region: String!, timeframe: String!): DroughtForecast!
  consumption(region: String!, sector: Sector): ConsumptionData!
}

type Subscription {
  waterLevelUpdates(sourceId: ID!): WaterLevel!
  droughtAlerts(region: String!): DroughtAlert!
}
```

---

## Real-Time Monitoring

### 4.1 Data Streaming Protocol

**Server-Sent Events (SSE)**:
```javascript
// Client connects to SSE endpoint
const eventSource = new EventSource(
  'https://api.wia-water.org/v1/stream/water-levels'
);

eventSource.addEventListener('water-update', (event) => {
  const data = JSON.parse(event.data);
  console.log('Water level update:', data);
});
```

**Server Response**:
```
event: water-update
data: {"source":"lakemead","level":327.5,"capacity":34}
id: 12345
retry: 10000

event: drought-alert
data: {"region":"southwest","severity":"high","score":78}
id: 12346
```

### 4.2 Time Series Data Protocol

**InfluxDB Line Protocol**:
```
water_level,source=lakemead,region=southwest value=327.5 1642251600000000000
consumption,sector=residential,region=southwest value=1125000 1642251600000000000
drought_risk,region=southwest score=78,severity="high" 1642251600000000000
```

### 4.3 Batch Data Transfer

**Bulk Upload Format**:
```json
{
  "batch_id": "batch-2025-01-15",
  "timestamp": "2025-01-15T10:00:00Z",
  "records": [
    {
      "source": "lakemead",
      "measurements": [
        {"timestamp": "2025-01-15T09:00:00Z", "level": 327.4},
        {"timestamp": "2025-01-15T10:00:00Z", "level": 327.5}
      ]
    }
  ]
}
```

---

## Alert Distribution

### 5.1 Multi-Channel Alert Protocol

**Alert Priority Levels**:
```
P0: Critical (immediate action required)
P1: High (action required within 1 hour)
P2: Medium (action required within 24 hours)
P3: Low (informational)
```

**Distribution Channels**:
```json
{
  "alert": {
    "id": "alert-drought-001",
    "priority": "P0",
    "severity": "critical",
    "message": "Severe drought conditions detected"
  },
  "distribution": {
    "email": {
      "to": ["emergency@wateragency.gov"],
      "subject": "[CRITICAL] Drought Alert - Southwest Region"
    },
    "sms": {
      "numbers": ["+1-555-0100"],
      "message": "URGENT: Severe drought alert for southwest region"
    },
    "webhook": {
      "url": "https://alerts.wateragency.gov/webhook",
      "method": "POST",
      "retry": 3
    },
    "push": {
      "tokens": ["device-token-123"],
      "title": "Critical Drought Alert"
    }
  }
}
```

### 5.2 Alert Acknowledgment Protocol

```http
POST /api/v1/alerts/{alert-id}/acknowledge
{
  "acknowledged_by": "operator-id-123",
  "timestamp": "2025-01-15T10:05:00Z",
  "notes": "Emergency response team activated",
  "actions_taken": [
    "Activated water restrictions",
    "Increased desalination production",
    "Notified agricultural sector"
  ]
}
```

---

## Interoperability

### 6.1 Standard Data Exchange Formats

**CSV Export**:
```csv
timestamp,source_id,source_name,level,capacity,trend
2025-01-15T10:00:00Z,source-lakemead,Lake Mead,327.5,34,declining
2025-01-15T10:00:00Z,source-lakepowell,Lake Powell,1075.8,28,stable
```

**XML Format**:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<WaterData xmlns="https://wia-standards.org/water/v1">
  <Source id="source-lakemead">
    <Name>Lake Mead</Name>
    <Level unit="meters">327.5</Level>
    <Capacity percent="34"/>
    <Trend>declining</Trend>
    <Timestamp>2025-01-15T10:00:00Z</Timestamp>
  </Source>
</WaterData>
```

### 6.2 Integration with External Systems

**SCADA Integration**:
```json
{
  "protocol": "Modbus TCP",
  "endpoint": "192.168.1.100:502",
  "registers": {
    "water_level": { "address": 30001, "type": "float" },
    "flow_rate": { "address": 30002, "type": "float" },
    "valve_status": { "address": 10001, "type": "boolean" }
  }
}
```

**GIS Integration**:
```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "Point",
        "coordinates": [-114.7443, 36.0155]
      },
      "properties": {
        "source_id": "lakemead",
        "name": "Lake Mead",
        "level": 327.5,
        "capacity": 34,
        "status": "critical"
      }
    }
  ]
}
```

---

## Security Protocols

### 7.1 Encryption Standards

**Transport Security**:
- TLS 1.3 required for all connections
- Certificate pinning for critical infrastructure
- Perfect Forward Secrecy (PFS)

**Data Encryption**:
```json
{
  "encrypted": true,
  "algorithm": "AES-256-GCM",
  "encrypted_data": "base64-encoded-encrypted-payload",
  "iv": "initialization-vector",
  "tag": "authentication-tag"
}
```

### 7.2 Authentication & Authorization

**OAuth 2.0 Scopes**:
```
water:read          - Read water level data
water:write         - Submit water measurements
drought:forecast    - Access drought predictions
alerts:manage       - Manage alert subscriptions
admin:full          - Full administrative access
```

**JWT Token Structure**:
```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT"
  },
  "payload": {
    "sub": "user-id-123",
    "iss": "https://auth.wia-water.org",
    "aud": "https://api.wia-water.org",
    "exp": 1642255200,
    "iat": 1642251600,
    "scopes": ["water:read", "drought:forecast"]
  }
}
```

### 7.3 Data Integrity

**Digital Signatures**:
```json
{
  "data": {...},
  "signature": {
    "algorithm": "Ed25519",
    "public_key": "base64-encoded-public-key",
    "signature": "base64-encoded-signature",
    "timestamp": "2025-01-15T10:00:00Z"
  }
}
```

---

## Compliance

### 8.1 Regulatory Compliance

**Required Metadata**:
```json
{
  "compliance": {
    "standards": ["WIA-ENE-053", "ISO-24516"],
    "certifications": ["WaterSense", "EPA-Certified"],
    "audit_trail": true,
    "data_retention_days": 365,
    "privacy_compliant": true
  }
}
```

### 8.2 Audit Logging Protocol

**Log Format**:
```json
{
  "log_id": "log-12345",
  "timestamp": "2025-01-15T10:00:00Z",
  "action": "water_level_update",
  "user": "system-sensor-001",
  "source": "source-lakemead",
  "data": {
    "old_value": 327.4,
    "new_value": 327.5
  },
  "ip_address": "192.168.1.100",
  "result": "success"
}
```

### 8.3 Data Retention Policy

| Data Type | Retention Period | Archive Format |
|-----------|------------------|----------------|
| Real-time metrics | 90 days | Hot storage |
| Historical data | 7 years | Cold storage |
| Alert logs | 5 years | Compressed |
| Audit trails | 10 years | Encrypted |

---

**Document Control**
© 2025 WIA Standards Committee
License: MIT
Contact: protocol@wia.org
弘益人間 · Benefit All Humanity
