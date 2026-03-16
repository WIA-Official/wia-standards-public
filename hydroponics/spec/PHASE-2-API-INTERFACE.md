# WIA-AGRI-027: Hydroponics Standard
## Phase 2: API Interface Standards

**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26
**Emoji:** 💧

---

## 1. Overview

This document defines the RESTful API interface standards for the WIA-AGRI-027 Hydroponics Standard. These APIs enable seamless communication between hydroponic systems, sensors, controllers, and third-party applications.

### 1.1 Philosophy

弘益人間 (홍익인간) - Benefit All Humanity

Our APIs are designed to be intuitive, well-documented, and accessible to developers of all skill levels.

### 1.2 Base URL

```
Production: https://api.wia.org/agri027/v1
Staging: https://staging-api.wia.org/agri027/v1
Development: http://localhost:8080/api/v1
```

---

## 2. Authentication

### 2.1 API Key Authentication

All API requests must include an API key in the header:

```http
Authorization: Bearer YOUR_API_KEY
```

**Example:**

```bash
curl -X GET https://api.wia.org/agri027/v1/system/status \
  -H "Authorization: Bearer sk_live_abcdef123456"
```

### 2.2 OAuth 2.0 (Optional)

For third-party integrations, OAuth 2.0 is supported:

```http
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret"
}
```

**Response:**

```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

---

## 3. System Management APIs

### 3.1 Get System Status

Retrieve the current status of a hydroponic system.

**Endpoint:** `GET /system/{systemId}/status`

**Parameters:**

- `systemId` (path, required): Unique system identifier

**Response:**

```json
{
  "systemId": "HYDRO-SYS-001",
  "status": "operational",
  "uptime": 86400,
  "lastUpdate": "2025-12-26T10:30:00Z",
  "reservoir": {
    "level": 85,
    "capacity": 100,
    "unit": "liters"
  },
  "sensors": {
    "active": 12,
    "inactive": 0,
    "maintenance": 1
  },
  "actuators": {
    "active": 5,
    "inactive": 0
  },
  "currentConditions": {
    "ph": 6.2,
    "ec": 2.1,
    "temperature": 21.5,
    "dissolvedOxygen": 8.3
  }
}
```

**Status Codes:**

- `200 OK`: Success
- `404 Not Found`: System not found
- `401 Unauthorized`: Invalid API key
- `500 Internal Server Error`: Server error

---

### 3.2 List All Systems

Get a list of all registered hydroponic systems.

**Endpoint:** `GET /systems`

**Query Parameters:**

- `page` (optional): Page number (default: 1)
- `limit` (optional): Items per page (default: 20, max: 100)
- `status` (optional): Filter by status (operational, maintenance, offline)
- `type` (optional): Filter by system type (NFT, DWC, etc.)

**Response:**

```json
{
  "total": 45,
  "page": 1,
  "limit": 20,
  "systems": [
    {
      "systemId": "HYDRO-SYS-001",
      "systemName": "Greenhouse A - NFT System",
      "systemType": "NFT",
      "status": "operational",
      "location": "Zone A",
      "cropType": "lettuce",
      "lastUpdate": "2025-12-26T10:30:00Z"
    }
  ]
}
```

---

### 3.3 Create System

Register a new hydroponic system.

**Endpoint:** `POST /systems`

**Request Body:**

```json
{
  "systemName": "Greenhouse B - DWC System",
  "systemType": "DWC",
  "location": {
    "facility": "Main Farm",
    "zone": "Zone B",
    "coordinates": {
      "latitude": 37.7749,
      "longitude": -122.4194
    }
  },
  "reservoir": {
    "capacity": 200,
    "material": "food-grade plastic"
  }
}
```

**Response:**

```json
{
  "systemId": "HYDRO-SYS-046",
  "systemName": "Greenhouse B - DWC System",
  "status": "created",
  "createdAt": "2025-12-26T10:30:00Z"
}
```

---

## 4. Sensor Data APIs

### 4.1 Get Current Sensor Data

Retrieve real-time sensor readings.

**Endpoint:** `GET /system/{systemId}/sensors/current`

**Response:**

```json
{
  "systemId": "HYDRO-SYS-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "sensors": [
    {
      "deviceId": "SENSOR-PH-001",
      "deviceType": "ph_sensor",
      "location": "reservoir_1",
      "value": 6.2,
      "unit": "pH",
      "quality": "excellent"
    },
    {
      "deviceId": "SENSOR-EC-001",
      "deviceType": "ec_sensor",
      "location": "reservoir_1",
      "value": 2.1,
      "unit": "mS/cm",
      "quality": "excellent"
    }
  ]
}
```

---

### 4.2 Get Historical Sensor Data

Retrieve historical sensor readings with time-series data.

**Endpoint:** `GET /system/{systemId}/sensors/history`

**Query Parameters:**

- `deviceId` (optional): Filter by specific device
- `parameter` (optional): Filter by parameter (ph, ec, temperature, etc.)
- `startTime` (required): ISO 8601 timestamp
- `endTime` (required): ISO 8601 timestamp
- `aggregation` (optional): raw, hourly, daily (default: raw)

**Response:**

```json
{
  "systemId": "HYDRO-SYS-001",
  "parameter": "pH",
  "startTime": "2025-12-25T00:00:00Z",
  "endTime": "2025-12-26T00:00:00Z",
  "aggregation": "hourly",
  "dataPoints": [
    {
      "timestamp": "2025-12-25T00:00:00Z",
      "value": 6.1,
      "quality": "excellent"
    },
    {
      "timestamp": "2025-12-25T01:00:00Z",
      "value": 6.2,
      "quality": "excellent"
    }
  ],
  "statistics": {
    "min": 5.9,
    "max": 6.5,
    "average": 6.2,
    "median": 6.2,
    "stdDev": 0.15
  }
}
```

---

## 5. Nutrient Management APIs

### 5.1 Get Nutrient Formula

Retrieve a specific nutrient formula.

**Endpoint:** `GET /nutrients/formulas/{formulaId}`

**Response:**

```json
{
  "formulaId": "FORMULA-LETTUCE-VEG-001",
  "formulaName": "Lettuce Vegetative Stage",
  "version": "1.0",
  "cropType": "lettuce",
  "growthStage": "vegetative",
  "targetPH": 6.0,
  "targetEC": 2.0,
  "macronutrients": {
    "N": 120,
    "P": 50,
    "K": 180,
    "Ca": 100,
    "Mg": 40,
    "S": 30
  },
  "micronutrients": {
    "Fe": 2.5,
    "Mn": 0.8,
    "Zn": 0.3,
    "Cu": 0.1,
    "B": 0.5,
    "Mo": 0.05
  }
}
```

---

### 5.2 Adjust Nutrients

Initiate a nutrient adjustment for a system.

**Endpoint:** `POST /system/{systemId}/nutrients/adjust`

**Request Body:**

```json
{
  "targetPH": 6.0,
  "targetEC": 2.0,
  "nutrients": {
    "N": 120,
    "P": 50,
    "K": 180
  },
  "method": "automatic",
  "priority": "high"
}
```

**Response:**

```json
{
  "adjustmentId": "ADJ-20251226-001",
  "systemId": "HYDRO-SYS-001",
  "status": "initiated",
  "estimatedCompletionTime": 300,
  "message": "Nutrient adjustment in progress",
  "timestamp": "2025-12-26T10:30:00Z"
}
```

---

### 5.3 Get Adjustment Status

Check the status of a nutrient adjustment.

**Endpoint:** `GET /system/{systemId}/nutrients/adjustments/{adjustmentId}`

**Response:**

```json
{
  "adjustmentId": "ADJ-20251226-001",
  "systemId": "HYDRO-SYS-001",
  "status": "completed",
  "progress": 100,
  "startedAt": "2025-12-26T10:30:00Z",
  "completedAt": "2025-12-26T10:35:00Z",
  "actualChanges": {
    "ph": {
      "before": 6.5,
      "after": 6.0,
      "target": 6.0
    },
    "ec": {
      "before": 1.8,
      "after": 2.0,
      "target": 2.0
    }
  }
}
```

---

## 6. Alert & Notification APIs

### 6.1 Get Active Alerts

Retrieve all active alerts for a system.

**Endpoint:** `GET /system/{systemId}/alerts/active`

**Response:**

```json
{
  "systemId": "HYDRO-SYS-001",
  "totalAlerts": 2,
  "alerts": [
    {
      "alertId": "550e8400-e29b-41d4-a716-446655440000",
      "timestamp": "2025-12-26T14:15:00Z",
      "severity": "warning",
      "category": "nutrient",
      "parameter": "pH",
      "currentValue": 7.2,
      "thresholdValue": 6.5,
      "message": "pH level exceeds maximum threshold",
      "acknowledged": false
    }
  ]
}
```

---

### 6.2 Create Alert Configuration

Set up alert rules for specific parameters.

**Endpoint:** `POST /system/{systemId}/alerts/config`

**Request Body:**

```json
{
  "parameter": "pH",
  "min": 5.5,
  "max": 6.5,
  "severity": "warning",
  "notificationChannels": ["email", "sms", "webhook"],
  "recipients": ["operator@farm.com"],
  "webhookUrl": "https://your-app.com/webhooks/alerts"
}
```

**Response:**

```json
{
  "configId": "CONFIG-PH-001",
  "systemId": "HYDRO-SYS-001",
  "status": "active",
  "createdAt": "2025-12-26T10:30:00Z"
}
```

---

### 6.3 Acknowledge Alert

Mark an alert as acknowledged.

**Endpoint:** `PUT /system/{systemId}/alerts/{alertId}/acknowledge`

**Request Body:**

```json
{
  "acknowledgedBy": "operator@farm.com",
  "notes": "Investigating pH spike, adding pH down solution"
}
```

**Response:**

```json
{
  "alertId": "550e8400-e29b-41d4-a716-446655440000",
  "acknowledged": true,
  "acknowledgedBy": "operator@farm.com",
  "acknowledgedAt": "2025-12-26T14:16:00Z"
}
```

---

## 7. Control & Automation APIs

### 7.1 Execute Control Action

Manually trigger a control action.

**Endpoint:** `POST /system/{systemId}/control/execute`

**Request Body:**

```json
{
  "action": "start_pump",
  "deviceId": "PUMP-001",
  "duration": 300,
  "priority": "manual_override"
}
```

**Response:**

```json
{
  "actionId": "ACTION-20251226-001",
  "systemId": "HYDRO-SYS-001",
  "deviceId": "PUMP-001",
  "status": "executing",
  "startedAt": "2025-12-26T10:30:00Z",
  "estimatedCompletion": "2025-12-26T10:35:00Z"
}
```

---

### 7.2 Get Automation Rules

Retrieve configured automation rules.

**Endpoint:** `GET /system/{systemId}/automation/rules`

**Response:**

```json
{
  "systemId": "HYDRO-SYS-001",
  "rules": [
    {
      "ruleId": "RULE-PH-AUTO-001",
      "name": "Auto pH Adjustment",
      "enabled": true,
      "condition": {
        "parameter": "pH",
        "operator": "greater_than",
        "value": 6.5
      },
      "action": {
        "type": "adjust_ph",
        "target": 6.0,
        "deviceId": "DOSER-PH-DOWN"
      }
    }
  ]
}
```

---

## 8. Harvest & Reporting APIs

### 8.1 Log Harvest

Record a harvest event.

**Endpoint:** `POST /system/{systemId}/harvest/log`

**Request Body:**

```json
{
  "cropType": "lettuce",
  "variety": "Green Butterhead",
  "plantingDate": "2025-11-01T00:00:00Z",
  "harvestDate": "2025-12-26T08:00:00Z",
  "quantity": 50,
  "weight": 12.5,
  "unit": "kg",
  "quality": "premium",
  "operator": "John Doe",
  "notes": "Excellent quality, uniform size"
}
```

**Response:**

```json
{
  "harvestId": "HARVEST-20251226-001",
  "systemId": "HYDRO-SYS-001",
  "batchNumber": "BATCH-2025-W52-001",
  "status": "logged",
  "timestamp": "2025-12-26T10:30:00Z"
}
```

---

### 8.2 Generate Report

Generate a comprehensive system report.

**Endpoint:** `POST /system/{systemId}/reports/generate`

**Request Body:**

```json
{
  "reportType": "performance",
  "startDate": "2025-12-01T00:00:00Z",
  "endDate": "2025-12-26T23:59:59Z",
  "format": "pdf",
  "includeCharts": true,
  "sections": ["summary", "sensor_data", "nutrient_history", "harvest_log"]
}
```

**Response:**

```json
{
  "reportId": "REPORT-20251226-001",
  "status": "generating",
  "estimatedTime": 60,
  "downloadUrl": null
}
```

---

## 9. Webhooks

### 9.1 Webhook Configuration

Configure webhooks to receive real-time notifications.

**Endpoint:** `POST /webhooks`

**Request Body:**

```json
{
  "url": "https://your-app.com/webhooks/wia-agri027",
  "events": ["alert.created", "sensor.threshold_exceeded", "harvest.logged"],
  "secret": "your_webhook_secret",
  "active": true
}
```

**Response:**

```json
{
  "webhookId": "WEBHOOK-001",
  "url": "https://your-app.com/webhooks/wia-agri027",
  "status": "active",
  "createdAt": "2025-12-26T10:30:00Z"
}
```

### 9.2 Webhook Payload Example

```json
{
  "event": "alert.created",
  "timestamp": "2025-12-26T14:15:00Z",
  "data": {
    "alertId": "550e8400-e29b-41d4-a716-446655440000",
    "systemId": "HYDRO-SYS-001",
    "severity": "warning",
    "parameter": "pH",
    "currentValue": 7.2,
    "thresholdValue": 6.5,
    "message": "pH level exceeds maximum threshold"
  },
  "signature": "sha256=abc123..."
}
```

---

## 10. Rate Limiting

To ensure fair usage and system stability:

- **Free tier:** 100 requests per minute
- **Pro tier:** 1,000 requests per minute
- **Enterprise tier:** Custom limits

Rate limit headers:

```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1640520000
```

---

## 11. Error Handling

### 11.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "The pH value must be between 4.0 and 8.0",
    "field": "targetPH",
    "timestamp": "2025-12-26T10:30:00Z",
    "requestId": "req_abc123"
  }
}
```

### 11.2 Common Error Codes

- `INVALID_API_KEY`: Invalid or missing API key
- `INVALID_PARAMETER`: Invalid parameter value
- `RESOURCE_NOT_FOUND`: Requested resource not found
- `RATE_LIMIT_EXCEEDED`: Too many requests
- `SERVER_ERROR`: Internal server error

---

## 12. Conclusion

The WIA-AGRI-027 Phase 2 API specifications provide a comprehensive, RESTful interface for hydroponic system management. These APIs enable developers to build powerful applications and integrations.

---

**Document Information:**

- **Standard ID:** WIA-AGRI-027
- **Phase:** 2 - API Interface Standards
- **Status:** Active
- **Maintained by:** WIA (World Certification Industry Association)
- **Contact:** api@wia.org

弘益人間 (홍익인간) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
