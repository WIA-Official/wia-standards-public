# WIA-SOC-008 Phase 2: API Interface Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 2 defines RESTful API endpoints, WebSocket interfaces, and GraphQL schemas for water supply system integration. All implementations MUST support the core REST API.

## 2. Base API Specification

### 2.1 Endpoint Structure

```
https://api.{utility-domain}/wia/soc-008/v1/{resource}
```

**Required Headers:**
- `Content-Type: application/json`
- `Authorization: Bearer {token}`
- `X-WIA-Version: 1.0.0`
- `X-Request-ID: {UUID}` (for tracing)

### 2.2 Authentication

**Supported Methods:**
- OAuth 2.0 (REQUIRED)
- API Keys (OPTIONAL for internal systems)
- mTLS (RECOMMENDED for machine-to-machine)

**Token Scopes:**
- `read:water-quality`
- `read:consumption`
- `read:network`
- `write:alerts`
- `admin:system`

### 2.3 Rate Limiting

| Tier | Requests/minute | Burst | Cost |
|------|----------------|-------|------|
| Public | 60 | 10 | Free |
| Registered | 600 | 100 | Free |
| Premium | 6000 | 1000 | Paid |
| Enterprise | Unlimited | Unlimited | Contract |

## 3. Core API Endpoints

### 3.1 System Information

#### GET /system/info
Get system identity and capabilities

**Response:**
```json
{
  "systemId": "WS-2025-CITY-001",
  "utility": "City Water Authority",
  "certificationLevel": "ADVANCED",
  "capabilities": ["water-quality", "leak-detection", "smart-metering"],
  "coverage": {
    "population": 500000,
    "networkLength": 1200.5,
    "zones": 15
  },
  "status": "operational"
}
```

### 3.2 Water Quality

#### GET /water-quality/current
Get current water quality readings

**Query Parameters:**
- `zoneId` (optional): Filter by zone
- `stationId` (optional): Filter by monitoring station
- `parameters` (optional): Comma-separated list (e.g., "pH,turbidity,chlorine")

**Response:**
```json
{
  "timestamp": "2025-12-26T14:32:15Z",
  "readings": [
    {
      "stationId": "SENSOR-001",
      "location": { "lat": 37.5665, "lon": 126.9780 },
      "parameters": {
        "pH": { "value": 7.3, "status": "normal" },
        "turbidity": { "value": 0.8, "status": "normal" },
        "chlorine": { "value": 0.5, "status": "normal" }
      },
      "compliance": "compliant"
    }
  ],
  "count": 1
}
```

#### GET /water-quality/history
Get historical water quality data

**Query Parameters:**
- `startDate`: ISO 8601 datetime (REQUIRED)
- `endDate`: ISO 8601 datetime (REQUIRED)
- `stationId`: Filter by station
- `parameters`: Comma-separated parameters
- `interval`: `5min|15min|1hour|1day` (default: 15min)
- `limit`: Max records (default: 1000, max: 10000)

### 3.3 Network Status

#### GET /network/status
Get real-time network status

**Response:**
```json
{
  "timestamp": "2025-12-26T14:32:15Z",
  "overall": {
    "status": "operational",
    "efficiency": 94.2,
    "waterLoss": 8.5
  },
  "zones": [
    {
      "zoneId": "ZONE-A",
      "name": "Downtown",
      "pressure": { "average": 4.2, "min": 3.8, "max": 4.6 },
      "flowRate": { "current": 2450, "average": 2300 },
      "population": 50000,
      "status": "normal"
    }
  ],
  "alerts": 3,
  "warnings": 7
}
```

### 3.4 Leak Detection

#### GET /leaks/active
Get all active leak events

**Response:**
```json
{
  "timestamp": "2025-12-26T14:32:15Z",
  "count": 3,
  "leaks": [
    {
      "eventId": "LEAK-2025-001",
      "detectionTime": "2025-12-26T12:10:00Z",
      "location": {
        "coordinates": { "lat": 37.5665, "lon": 126.9780 },
        "address": "Main St & 5th Ave",
        "accuracy": 15.5
      },
      "severity": "high",
      "estimatedLoss": { "rate": 850, "volume": 2125 },
      "status": "repairing",
      "assignedTeam": "Team-Alpha",
      "eta": "2025-12-26T16:00:00Z"
    }
  ]
}
```

#### POST /leaks/report
Report a new leak (public endpoint)

**Request Body:**
```json
{
  "reportedBy": "citizen|sensor|patrol",
  "location": {
    "coordinates": { "lat": 37.5665, "lon": 126.9780 },
    "address": "123 Main St",
    "description": "Water pooling on street"
  },
  "urgency": "low|medium|high",
  "contactInfo": {
    "phone": "+821012345678",
    "email": "reporter@example.com"
  }
}
```

**Response:**
```json
{
  "reportId": "REPORT-2025-12345",
  "status": "received",
  "estimatedResponse": "2025-12-26T16:00:00Z",
  "trackingUrl": "https://status.utility.com/reports/REPORT-2025-12345"
}
```

### 3.5 Smart Metering

#### GET /meters/{meterId}/consumption
Get consumption data for a specific meter

**Query Parameters:**
- `startDate`: ISO 8601 datetime
- `endDate`: ISO 8601 datetime
- `interval`: `hourly|daily|monthly`

**Response:**
```json
{
  "meterId": "METER-123456",
  "customerId": "CUST-789012",
  "period": {
    "start": "2025-12-01T00:00:00Z",
    "end": "2025-12-26T23:59:59Z"
  },
  "consumption": {
    "total": 45.3,
    "average": 1.74,
    "peak": 3.2,
    "unit": "m³"
  },
  "readings": [
    {
      "timestamp": "2025-12-26T00:00:00Z",
      "value": 1.8,
      "cumulative": 45.3
    }
  ],
  "billing": {
    "amount": 45.30,
    "currency": "USD",
    "dueDate": "2026-01-15"
  }
}
```

### 3.6 Alerts and Notifications

#### GET /alerts
Get system alerts

**Query Parameters:**
- `severity`: `info|warning|error|critical`
- `status`: `active|acknowledged|resolved`
- `limit`: Max records (default: 100)

**Response:**
```json
{
  "count": 2,
  "alerts": [
    {
      "alertId": "ALERT-2025-456",
      "timestamp": "2025-12-26T14:15:00Z",
      "severity": "critical",
      "type": "water-quality",
      "message": "Chlorine level below threshold in Zone C",
      "location": { "zoneId": "ZONE-C", "stationId": "SENSOR-042" },
      "status": "active",
      "actionRequired": "Increase chlorination",
      "affectedPopulation": 25000
    }
  ]
}
```

#### POST /alerts/{alertId}/acknowledge
Acknowledge an alert

**Request Body:**
```json
{
  "acknowledgedBy": "operator-id",
  "comment": "Chlorination adjustment initiated",
  "estimatedResolution": "2025-12-26T15:00:00Z"
}
```

## 4. WebSocket API

### 4.1 Connection

```
wss://api.{utility-domain}/wia/soc-008/v1/stream
```

**Authentication:** Via query parameter or upgrade header
```
wss://api.example.com/stream?token={bearer-token}
```

### 4.2 Subscription

**Client Subscribe:**
```json
{
  "action": "subscribe",
  "channels": ["water-quality", "leaks", "alerts"],
  "filters": {
    "zoneId": "ZONE-A",
    "severity": ["warning", "critical"]
  }
}
```

**Server Acknowledgment:**
```json
{
  "action": "subscribed",
  "channels": ["water-quality", "leaks", "alerts"],
  "subscriptionId": "SUB-123456"
}
```

### 4.3 Real-time Updates

**Water Quality Update:**
```json
{
  "channel": "water-quality",
  "timestamp": "2025-12-26T14:32:15Z",
  "data": {
    "stationId": "SENSOR-001",
    "parameters": {
      "pH": { "value": 7.3, "status": "normal" }
    }
  }
}
```

**Leak Alert:**
```json
{
  "channel": "leaks",
  "timestamp": "2025-12-26T14:32:15Z",
  "event": "new-leak",
  "data": {
    "eventId": "LEAK-2025-002",
    "severity": "high",
    "location": { "lat": 37.5665, "lon": 126.9780 }
  }
}
```

## 5. GraphQL Schema (Optional)

```graphql
type WaterSupplySystem {
  systemId: ID!
  utility: String!
  certificationLevel: CertificationLevel!
  coverage: Coverage!
  waterQuality: [WaterQuality!]!
  network: NetworkStatus!
  leaks: [LeakEvent!]!
  alerts: [Alert!]!
}

type WaterQuality {
  stationId: ID!
  timestamp: DateTime!
  location: Coordinates!
  pH: Parameter!
  turbidity: Parameter!
  chlorine: Parameter!
  compliance: ComplianceStatus!
}

type Parameter {
  value: Float!
  unit: String!
  status: ParameterStatus!
}

enum ParameterStatus {
  NORMAL
  WARNING
  CRITICAL
}

type Query {
  system: WaterSupplySystem!
  waterQuality(stationId: ID, zoneId: ID): [WaterQuality!]!
  leaks(status: LeakStatus): [LeakEvent!]!
  alerts(severity: AlertSeverity): [Alert!]!
}

type Subscription {
  waterQualityUpdates(stationId: ID): WaterQuality!
  newLeaks(severity: LeakSeverity): LeakEvent!
  systemAlerts(severity: AlertSeverity): Alert!
}
```

## 6. Error Handling

### 6.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "The specified zone ID does not exist",
    "details": {
      "parameter": "zoneId",
      "value": "ZONE-X",
      "validValues": ["ZONE-A", "ZONE-B", "ZONE-C"]
    },
    "timestamp": "2025-12-26T14:32:15Z",
    "requestId": "REQ-123456"
  }
}
```

### 6.2 Standard Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| INVALID_REQUEST | 400 | Malformed request |
| UNAUTHORIZED | 401 | Invalid or missing credentials |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| INTERNAL_ERROR | 500 | Server error |
| SERVICE_UNAVAILABLE | 503 | Temporary outage |

## 7. Performance Requirements

| Metric | Requirement |
|--------|-------------|
| Response Time (p95) | < 200ms |
| Response Time (p99) | < 500ms |
| Availability | > 99.9% |
| Throughput | > 1000 req/sec per instance |
| WebSocket Latency | < 100ms |

## 8. Versioning and Deprecation

- API version in URL path: `/v1/`, `/v2/`
- Backward compatibility for minor versions
- Deprecation notice: 12 months minimum
- Support period: 24 months after deprecation
- Migration guide required for major versions

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA / SmileStory Inc. · MIT License
