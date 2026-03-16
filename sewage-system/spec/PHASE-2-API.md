# WIA-SOC-009 Phase 2: API Interface Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 2 defines the RESTful API and WebSocket interfaces for smart sewage systems. All endpoints MUST support JSON-LD content negotiation and return appropriate HTTP status codes.

## 2. Base URL Structure

```
https://api.{municipality}.sewage.example.com/v1/
```

Authentication: Bearer token (OAuth 2.0) or API key in header

## 3. System Information Endpoints

### 3.1 Get System Information

**GET** `/system/info`

Returns comprehensive system information including capacity, service area, and capabilities.

**Response 200:**
```json
{
  "systemId": "SYS-2025-001",
  "municipality": "Example City",
  "capacity": 150000,
  "servingPopulation": 250000,
  "capabilities": [
    "real_time_monitoring",
    "predictive_maintenance",
    "water_quality_testing",
    "overflow_prediction"
  ]
}
```

### 3.2 Get System Status

**GET** `/system/status`

Returns current operational status of the entire sewage system.

**Response 200:**
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "overallStatus": "normal|warning|critical",
  "flowRate": 4.2,
  "activePumps": 12,
  "treatmentEfficiency": 97.5,
  "alerts": 3,
  "lastUpdate": "2025-12-26T14:30:00Z"
}
```

## 4. Monitoring Endpoints

### 4.1 Get Sensor Readings

**GET** `/sensors/readings`

Query parameters:
- `sensorId` (optional): Filter by specific sensor
- `type` (optional): Filter by sensor type (flow|quality|level|pressure)
- `zone` (optional): Filter by geographic zone
- `from` (optional): Start timestamp (ISO8601)
- `to` (optional): End timestamp (ISO8601)
- `limit` (default: 100, max: 1000): Number of results

**Response 200:**
```json
{
  "count": 150,
  "next": "https://api.example.com/v1/sensors/readings?offset=100",
  "results": [
    {
      "sensorId": "SENS-001",
      "type": "flow",
      "timestamp": "2025-12-26T14:30:00Z",
      "value": 4.2,
      "unit": "m³/s",
      "quality": "good"
    }
  ]
}
```

### 4.2 Get Water Quality

**GET** `/water-quality/current`

Query parameters:
- `location` (optional): Specific sampling location
- `parameters` (optional): Comma-separated list of parameters

**Response 200:**
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "location": "Influent",
  "parameters": {
    "pH": 7.2,
    "DO": 5.8,
    "BOD": 180,
    "COD": 420,
    "TSS": 220,
    "ammoniaNitrogen": 35
  },
  "compliance": "compliant"
}
```

### 4.3 Get Flow Data

**GET** `/flow/current`

Returns current flow rates at all monitoring points.

**Response 200:**
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "monitoringPoints": [
    {
      "locationId": "MP-001",
      "name": "Main Influent",
      "flowRate": 4.2,
      "velocity": 1.8,
      "depth": 2.3,
      "trend": "increasing|stable|decreasing"
    }
  ],
  "totalInflow": 4.2,
  "totalOutflow": 4.0,
  "bypass": 0.0
}
```

## 5. Treatment Process Endpoints

### 5.1 Get Treatment Status

**GET** `/treatment/status`

**Response 200:**
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "stages": [
    {
      "stage": "primary",
      "status": "operational",
      "efficiency": 65,
      "flowRate": 4.2,
      "removalRate": {
        "TSS": 60,
        "BOD": 30
      }
    },
    {
      "stage": "secondary",
      "status": "operational",
      "efficiency": 95,
      "flowRate": 4.0,
      "removalRate": {
        "BOD": 90,
        "ammonia": 85
      }
    }
  ],
  "overallEfficiency": 97.5
}
```

### 5.2 Update Treatment Parameters

**POST** `/treatment/control`

Request body:
```json
{
  "stage": "secondary",
  "parameter": "aeration_rate",
  "value": 85,
  "reason": "Optimize DO levels"
}
```

**Response 200:**
```json
{
  "commandId": "CMD-12345",
  "status": "accepted",
  "executedAt": "2025-12-26T14:31:00Z"
}
```

## 6. Alert and Event Endpoints

### 6.1 Get Active Alerts

**GET** `/alerts/active`

Query parameters:
- `severity` (optional): Filter by severity level
- `category` (optional): Filter by alert category

**Response 200:**
```json
{
  "count": 3,
  "alerts": [
    {
      "alertId": "ALT-001",
      "timestamp": "2025-12-26T14:25:00Z",
      "severity": "warning",
      "category": "water_quality",
      "message": "Elevated ammonia levels detected",
      "location": "Zone 3",
      "parameters": {
        "threshold": 2.0,
        "actual": 2.3,
        "deviation": 15
      },
      "acknowledged": false
    }
  ]
}
```

### 6.2 Acknowledge Alert

**POST** `/alerts/{alertId}/acknowledge`

Request body:
```json
{
  "acknowledgedBy": "operator_name",
  "notes": "Investigating source"
}
```

**Response 200:**
```json
{
  "alertId": "ALT-001",
  "acknowledgedAt": "2025-12-26T14:32:00Z",
  "status": "acknowledged"
}
```

### 6.3 Get Event History

**GET** `/events/history`

Query parameters:
- `from` (required): Start date (ISO8601)
- `to` (required): End date (ISO8601)
- `type` (optional): Event type filter
- `limit` (default: 100): Number of results

**Response 200:**
```json
{
  "count": 42,
  "events": [
    {
      "eventId": "EVT-001",
      "timestamp": "2025-12-25T18:30:00Z",
      "type": "overflow",
      "location": "CSO-5",
      "severity": "high",
      "duration": 1800,
      "volume": 150,
      "resolved": true
    }
  ]
}
```

## 7. Reporting Endpoints

### 7.1 Generate Compliance Report

**POST** `/reports/compliance`

Request body:
```json
{
  "period": "monthly|quarterly|annual",
  "startDate": "2025-01-01",
  "endDate": "2025-01-31",
  "format": "pdf|json|csv"
}
```

**Response 202:** (Async processing)
```json
{
  "reportId": "RPT-001",
  "status": "processing",
  "estimatedCompletion": "2025-12-26T14:35:00Z",
  "downloadUrl": null
}
```

**GET** `/reports/{reportId}/status`

**Response 200:**
```json
{
  "reportId": "RPT-001",
  "status": "completed",
  "downloadUrl": "https://api.example.com/v1/reports/RPT-001/download",
  "expiresAt": "2025-12-27T14:35:00Z"
}
```

### 7.2 Get Performance Metrics

**GET** `/metrics/performance`

Query parameters:
- `period` (required): daily|weekly|monthly|yearly
- `startDate` (required): Start date
- `endDate` (optional): End date

**Response 200:**
```json
{
  "period": "monthly",
  "startDate": "2025-12-01",
  "endDate": "2025-12-31",
  "metrics": {
    "averageInflowRate": 3.8,
    "totalVolumeTreated": 9936000,
    "overallEfficiency": 97.2,
    "complianceRate": 100,
    "overflowEvents": 2,
    "energyConsumption": 125000,
    "chemicalCost": 18500,
    "maintenanceCost": 42000
  }
}
```

## 8. Predictive Analytics Endpoints

### 8.1 Get Overflow Prediction

**GET** `/predictions/overflow`

Query parameters:
- `lookahead` (default: 24): Hours to predict ahead
- `zone` (optional): Specific zone

**Response 200:**
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "predictions": [
    {
      "location": "CSO-5",
      "riskLevel": "high|medium|low",
      "probability": 0.75,
      "expectedTime": "2025-12-26T20:00:00Z",
      "estimatedVolume": 200,
      "confidence": 0.85
    }
  ]
}
```

### 8.2 Get Equipment Failure Prediction

**GET** `/predictions/equipment`

**Response 200:**
```json
{
  "predictions": [
    {
      "equipmentId": "PUMP-003",
      "equipmentType": "centrifugal_pump",
      "failureProbability": 0.68,
      "expectedFailureDate": "2026-01-15",
      "confidence": 0.82,
      "recommendedAction": "Schedule preventive maintenance",
      "estimatedCost": 5000
    }
  ]
}
```

## 9. WebSocket Real-time Streaming

**WebSocket** `/ws/stream`

Connection headers:
- `Authorization`: Bearer {token}
- `Sec-WebSocket-Protocol`: wia-soc-009-v1

Subscribe to channels:
```json
{
  "action": "subscribe",
  "channels": ["flow", "water_quality", "alerts", "equipment"]
}
```

Receive real-time updates:
```json
{
  "channel": "flow",
  "timestamp": "2025-12-26T14:30:05Z",
  "data": {
    "locationId": "MP-001",
    "flowRate": 4.3,
    "change": 0.1
  }
}
```

## 10. Error Responses

Standard HTTP status codes:
- `200 OK`: Success
- `201 Created`: Resource created
- `202 Accepted`: Async operation initiated
- `400 Bad Request`: Invalid request
- `401 Unauthorized`: Authentication required
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: Temporary unavailability

Error format:
```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "Parameter 'flowRate' must be positive",
    "details": {
      "parameter": "flowRate",
      "value": -1.5
    }
  }
}
```

---

© 2025 WIA · MIT License
