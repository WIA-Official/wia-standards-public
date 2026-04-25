# Chapter 5: API Interfaces

## Phase 2: RESTful APIs for Environmental Sensor Data

---

## 5.1 RESTful API Design Principles

The WIA-ENE-027 Phase 2 specification defines RESTful APIs following industry best practices.

### URL Structure

```
https://{host}/api/v{version}/{resource}
```

**Examples:**
```
https://api.example.com/api/v1/sensors
https://api.example.com/api/v1/sensors/ENV-AIR-001/data
https://api.example.com/api/v1/sensors/ENV-AIR-001/data/latest
```

### HTTP Methods

| Method | Purpose | Idempotent | Safe |
|--------|---------|------------|------|
| GET | Retrieve resources | Yes | Yes |
| POST | Create resources, submit data | No | No |
| PUT | Update resources | Yes | No |
| DELETE | Remove resources | Yes | No |

### Standard Response Codes

| Code | Meaning | Use Case |
|------|---------|----------|
| 200 | OK | Successful request |
| 201 | Created | Resource created successfully |
| 400 | Bad Request | Invalid request data |
| 401 | Unauthorized | Authentication required |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

---

## 5.2 Sensor Discovery and Registration

### List All Sensors

```http
GET /api/v1/sensors
```

**Query Parameters:**
- `type`: Filter by sensor type (`air_quality`, `water_quality`, `soil`, `meteorological`)
- `location`: Geographic bounding box `lat1,lon1,lat2,lon2`
- `status`: Filter by status (`active`, `inactive`, `maintenance`)
- `limit`: Maximum results (default 100, max 1000)
- `offset`: Pagination offset

**Example Request:**
```http
GET /api/v1/sensors?type=air_quality&status=active&limit=50
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "total": 1547,
  "count": 50,
  "offset": 0,
  "sensors": [
    {
      "deviceId": "ENV-AIR-001",
      "type": "air_quality",
      "location": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "altitude": 38.5
      },
      "status": "active",
      "lastUpdate": "2025-01-09T10:30:00Z",
      "capabilities": ["pm2_5", "pm10", "temperature", "humidity"]
    },
    {
      "deviceId": "ENV-AIR-002",
      "type": "air_quality",
      "location": {
        "latitude": 37.5700,
        "longitude": 126.9800,
        "altitude": 42.0
      },
      "status": "active",
      "lastUpdate": "2025-01-09T10:32:00Z",
      "capabilities": ["pm2_5", "pm10", "co2", "voc"]
    }
  ],
  "links": {
    "self": "/api/v1/sensors?type=air_quality&status=active&limit=50&offset=0",
    "next": "/api/v1/sensors?type=air_quality&status=active&limit=50&offset=50"
  }
}
```

### Get Sensor Details

```http
GET /api/v1/sensors/{deviceId}
```

**Example Request:**
```http
GET /api/v1/sensors/ENV-AIR-001
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "deviceId": "ENV-AIR-001",
  "type": "air_quality",
  "manufacturer": "AirSense Corp",
  "model": "AS-3000",
  "firmware": "v2.4.1",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude": 38.5,
    "accuracy": 10.0
  },
  "status": "active",
  "installDate": "2024-06-15T09:00:00Z",
  "lastCalibration": "2024-11-15T09:00:00Z",
  "nextCalibration": "2025-05-15T09:00:00Z",
  "capabilities": {
    "pm2_5": {
      "range": [0, 500],
      "unit": "μg/m³",
      "accuracy": 2.0,
      "method": "laser_scattering"
    },
    "pm10": {
      "range": [0, 1000],
      "unit": "μg/m³",
      "accuracy": 4.5,
      "method": "laser_scattering"
    },
    "temperature": {
      "range": [-40, 85],
      "unit": "°C",
      "accuracy": 0.3
    }
  },
  "metadata": {
    "owner": "Seoul Metropolitan Government",
    "contact": "env@seoul.go.kr",
    "notes": "Downtown monitoring station"
  }
}
```

### Register New Sensor

```http
POST /api/v1/sensors
```

**Request Body:**
```json
{
  "deviceId": "ENV-AIR-NEW-001",
  "type": "air_quality",
  "manufacturer": "AirSense Corp",
  "model": "AS-3000",
  "firmware": "v2.4.1",
  "location": {
    "latitude": 37.5650,
    "longitude": 126.9750,
    "altitude": 40.0
  },
  "capabilities": ["pm2_5", "pm10", "temperature", "humidity"]
}
```

**Response (201 Created):**
```json
{
  "deviceId": "ENV-AIR-NEW-001",
  "status": "registered",
  "registrationDate": "2025-01-09T10:45:00Z",
  "message": "Sensor registered successfully"
}
```

---

## 5.3 Data Submission Endpoints

### Submit Sensor Data

```http
POST /api/v1/sensors/{deviceId}/data
```

**Request Body (WIA-ENE-027 Phase 1 Format):**
```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    "pm2_5": {
      "value": 15.3,
      "unit": "μg/m³",
      "method": "laser_scattering"
    },
    "pm10": {
      "value": 22.8,
      "unit": "μg/m³",
      "method": "laser_scattering"
    }
  },
  "quality": {
    "overall": "good",
    "flags": []
  }
}
```

**Response (201 Created):**
```json
{
  "status": "accepted",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "dataId": "data-abc123",
  "validation": {
    "passed": true,
    "warnings": []
  }
}
```

**Error Response (400 Bad Request):**
```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid data format",
    "details": {
      "field": "readings.pm2_5.value",
      "issue": "Value 1500 exceeds maximum range of 500"
    },
    "timestamp": "2025-01-09T10:30:00Z"
  }
}
```

### Batch Data Submission

```http
POST /api/v1/sensors/{deviceId}/data/batch
```

**Request Body:**
```json
{
  "data": [
    {
      "timestamp": "2025-01-09T10:00:00.000Z",
      "readings": {"pm2_5": {"value": 14.5, "unit": "μg/m³"}}
    },
    {
      "timestamp": "2025-01-09T10:05:00.000Z",
      "readings": {"pm2_5": {"value": 15.1, "unit": "μg/m³"}}
    },
    {
      "timestamp": "2025-01-09T10:10:00.000Z",
      "readings": {"pm2_5": {"value": 15.8, "unit": "μg/m³"}}
    }
  ]
}
```

---

## 5.4 Data Retrieval and Querying

### Get Latest Reading

```http
GET /api/v1/sensors/{deviceId}/data/latest
```

**Response (200 OK):**
```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    "pm2_5": {"value": 15.3, "unit": "μg/m³"},
    "pm10": {"value": 22.8, "unit": "μg/m³"}
  }
}
```

### Get Historical Data

```http
GET /api/v1/sensors/{deviceId}/data
```

**Query Parameters:**
- `start`: ISO 8601 start timestamp (required)
- `end`: ISO 8601 end timestamp (required)
- `parameters`: Comma-separated list (e.g., `pm2_5,temperature`)
- `aggregation`: `none` | `hourly` | `daily` | `weekly` | `monthly`
- `format`: `json` | `csv` | `xml`
- `limit`: Maximum records (default 1000)
- `offset`: Pagination offset

**Example Request:**
```http
GET /api/v1/sensors/ENV-AIR-001/data?start=2025-01-08T00:00:00Z&end=2025-01-09T00:00:00Z&aggregation=hourly&parameters=pm2_5
```

**Response (200 OK):**
```json
{
  "deviceId": "ENV-AIR-001",
  "start": "2025-01-08T00:00:00Z",
  "end": "2025-01-09T00:00:00Z",
  "aggregation": "hourly",
  "count": 24,
  "data": [
    {
      "timestamp": "2025-01-08T00:00:00Z",
      "pm2_5": {
        "mean": 12.5,
        "min": 8.2,
        "max": 18.3,
        "stddev": 2.1,
        "count": 12
      }
    },
    {
      "timestamp": "2025-01-08T01:00:00Z",
      "pm2_5": {
        "mean": 14.2,
        "min": 10.5,
        "max": 19.8,
        "stddev": 2.5,
        "count": 12
      }
    }
  ]
}
```

### Bulk Data Retrieval

```http
POST /api/v1/data/bulk
```

**Request Body:**
```json
{
  "sensorIds": ["ENV-AIR-001", "ENV-AIR-002", "ENV-AIR-003"],
  "start": "2025-01-09T00:00:00Z",
  "end": "2025-01-09T12:00:00Z",
  "parameters": ["pm2_5", "temperature"]
}
```

---

## 5.5 Real-time Streaming Interfaces

### WebSocket Connection

**Connection URL:**
```
wss://api.example.com/v1/stream
```

**Subscribe Message:**
```json
{
  "action": "subscribe",
  "sensors": ["ENV-AIR-001", "ENV-AIR-002"],
  "parameters": ["pm2_5", "temperature"]
}
```

**Data Message:**
```json
{
  "type": "measurement",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-01-09T10:30:00Z",
  "readings": {
    "pm2_5": {"value": 15.3, "unit": "μg/m³"}
  }
}
```

### Server-Sent Events

```http
GET /api/v1/sensors/{deviceId}/stream
Accept: text/event-stream
```

**Response Stream:**
```
event: measurement
data: {"timestamp":"2025-01-09T10:30:00Z","pm2_5":15.3}

event: measurement
data: {"timestamp":"2025-01-09T10:35:00Z","pm2_5":15.5}

event: heartbeat
data: {"timestamp":"2025-01-09T10:36:00Z"}
```

---

## 5.6 Authentication and Authorization

### API Keys

```http
GET /api/v1/sensors
Authorization: Bearer YOUR_API_KEY
```

### OAuth 2.0

```http
GET /api/v1/sensors
Authorization: Bearer {access_token}
```

### Role-Based Access Control

| Role | Permissions |
|------|-------------|
| Public Viewer | Read public sensor data |
| Data Consumer | Read all sensor data |
| Sensor Operator | Read data, submit data, update config |
| Administrator | Full access |

---

## 5.7 Error Handling and Rate Limiting

### Error Response Format

```json
{
  "error": {
    "code": "SENSOR_NOT_FOUND",
    "message": "Sensor with ID 'ENV-AIR-999' not found",
    "details": {
      "requestedId": "ENV-AIR-999"
    },
    "timestamp": "2025-01-09T10:30:00Z",
    "requestId": "req_abc123"
  }
}
```

### Rate Limiting Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1704067200
```

### Rate Limit Tiers

| Tier | Requests/Hour | Burst Limit |
|------|---------------|-------------|
| Free | 1,000 | 100/minute |
| Standard | 10,000 | 500/minute |
| Professional | 100,000 | 2,000/minute |
| Enterprise | Unlimited | Custom |

---

## 5.8 Review Questions and Key Takeaways

### Review Questions

1. Design API endpoints for a water quality monitoring system with 50 sensors. What endpoints would you implement?

2. A sensor submits data every 5 minutes. Calculate monthly API calls and recommend appropriate rate limit tier.

3. Create a query to retrieve hourly aggregated PM2.5 data for last 7 days from 10 sensors.

4. Design error handling for a sensor that submits PM2.5 value of 2000 μg/m³ (above max range of 500).

### Key Takeaways

1. **RESTful Design**: Standard HTTP methods and URL structures enable intuitive API usage.

2. **Data Submission**: POST /sensors/{id}/data endpoint accepts Phase 1 formatted data.

3. **Querying**: Flexible query parameters (time range, aggregation, parameters) enable diverse use cases.

4. **Real-time Streaming**: WebSocket and SSE interfaces provide real-time data access.

5. **Authentication**: API keys, OAuth 2.0, and RBAC support different security requirements.

6. **Rate Limiting**: Tiered limits prevent abuse while supporting legitimate high-volume use.

---

© 2025 WIA Standards Committee. 弘益인간 (홍익인간) - Benefit All Humanity

**Next Chapter: [Chapter 6: Communication Protocols](06-protocol.md)**
