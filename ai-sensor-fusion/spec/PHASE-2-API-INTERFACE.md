# WIA AI Sensor Fusion API Interface Specification

**Phase 2: API Interface Standard**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Primary Color**: #14B8A6 (Teal)

---

## Overview

### 1.1 Purpose

WIA AI Sensor Fusion API defines RESTful interfaces and WebSocket protocols for sensor registration, fusion configuration, real-time data streaming, and fused output access. This API enables developers to integrate multi-modal sensor fusion into AI-powered applications.

### 1.2 Scope

- **In Scope**:
  - REST API for sensor management
  - REST API for fusion configuration
  - WebSocket for real-time data streaming
  - Query APIs for fused outputs
  - Calibration management APIs
  - Status and health monitoring

- **Out of Scope**:
  - Binary protocols (Phase 3)
  - External system integration (Phase 4)

### 1.3 Design Principles

1. **RESTful**: Follow REST architectural style
2. **Real-time**: WebSocket for streaming data
3. **Versioned**: API versioning for compatibility
4. **Authenticated**: Secure access control
5. **Well-documented**: OpenAPI 3.0 specification

---

## Base Configuration

### 2.1 Base URL

```
Production:  https://api.wia.live/sensor-fusion/v1
Development: http://localhost:8080/api/v1
```

### 2.2 Authentication

```http
Authorization: Bearer <JWT_TOKEN>
```

### 2.3 Common Headers

```http
Content-Type: application/json
Accept: application/json
X-WIA-Version: 1.0.0
X-Request-ID: <UUID>
```

### 2.4 Rate Limits

| Tier | Requests/minute | WebSocket Connections |
|------|-----------------|----------------------|
| Free | 60 | 1 |
| Basic | 300 | 5 |
| Professional | 1200 | 20 |
| Enterprise | Unlimited | Unlimited |

---

## Sensor Management API

### 3.1 Register Sensor

Register a new sensor in the fusion system.

**Endpoint**: `POST /sensors`

**Request**:
```json
{
  "sensor_id": "camera_front",
  "sensor_type": "camera",
  "description": "Front-facing RGB camera",
  "frame_id": "camera_front_optical",
  "capabilities": {
    "resolution": {"width": 1920, "height": 1080},
    "frame_rate_hz": 30,
    "field_of_view_deg": 90
  },
  "transform": {
    "translation": {"x": 0.15, "y": 0.0, "z": 0.3},
    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  }
}
```

**Response**: `201 Created`
```json
{
  "sensor_id": "camera_front",
  "status": "registered",
  "created_at": "2025-01-15T10:30:00Z",
  "endpoints": {
    "data_stream": "ws://api.wia.live/sensors/camera_front/stream",
    "metadata": "/sensors/camera_front"
  }
}
```

### 3.2 List Sensors

**Endpoint**: `GET /sensors`

**Query Parameters**:
- `type` (optional): Filter by sensor type
- `status` (optional): Filter by status (active, inactive, error)
- `limit` (optional): Number of results (default: 50)
- `offset` (optional): Pagination offset

**Response**: `200 OK`
```json
{
  "sensors": [
    {
      "sensor_id": "camera_front",
      "sensor_type": "camera",
      "status": "active",
      "last_update": "2025-01-15T10:30:15Z"
    },
    {
      "sensor_id": "lidar_top",
      "sensor_type": "lidar",
      "status": "active",
      "last_update": "2025-01-15T10:30:15Z"
    }
  ],
  "total": 5,
  "limit": 50,
  "offset": 0
}
```

### 3.3 Get Sensor Details

**Endpoint**: `GET /sensors/{sensor_id}`

**Response**: `200 OK`
```json
{
  "sensor_id": "camera_front",
  "sensor_type": "camera",
  "description": "Front-facing RGB camera",
  "status": "active",
  "frame_id": "camera_front_optical",
  "capabilities": {
    "resolution": {"width": 1920, "height": 1080},
    "frame_rate_hz": 30,
    "field_of_view_deg": 90
  },
  "transform": {
    "translation": {"x": 0.15, "y": 0.0, "z": 0.3},
    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  },
  "statistics": {
    "frames_received": 18523,
    "frames_dropped": 12,
    "average_latency_ms": 15.2,
    "last_frame": "2025-01-15T10:30:15.123Z"
  }
}
```

### 3.4 Update Sensor

**Endpoint**: `PATCH /sensors/{sensor_id}`

**Request**:
```json
{
  "status": "inactive",
  "transform": {
    "translation": {"x": 0.16, "y": 0.0, "z": 0.3},
    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  }
}
```

**Response**: `200 OK`

### 3.5 Delete Sensor

**Endpoint**: `DELETE /sensors/{sensor_id}`

**Response**: `204 No Content`

---

## Fusion Configuration API

### 4.1 Create Fusion Configuration

**Endpoint**: `POST /fusion/configs`

**Request**:
```json
{
  "config_id": "multi_sensor_fusion_v1",
  "description": "Multi-sensor fusion for object detection",
  "algorithm": {
    "type": "kalman_filter",
    "variant": "extended_kalman_filter",
    "parameters": {
      "process_noise": 0.01,
      "measurement_noise": 0.1,
      "initial_covariance": 1.0
    }
  },
  "sensors": [
    {
      "sensor_id": "camera_front",
      "enabled": true,
      "weight": 0.4,
      "update_rate_hz": 30.0,
      "timeout_ms": 100
    },
    {
      "sensor_id": "lidar_top",
      "enabled": true,
      "weight": 0.6,
      "update_rate_hz": 10.0,
      "timeout_ms": 150
    }
  ],
  "synchronization": {
    "method": "nearest_timestamp",
    "max_time_diff_ms": 50,
    "interpolation": "linear"
  },
  "output": {
    "fused_objects": true,
    "fused_state": true,
    "occupancy_grid": false,
    "publish_rate_hz": 20.0
  }
}
```

**Response**: `201 Created`
```json
{
  "config_id": "multi_sensor_fusion_v1",
  "status": "created",
  "created_at": "2025-01-15T10:30:00Z",
  "endpoints": {
    "activate": "/fusion/configs/multi_sensor_fusion_v1/activate",
    "status": "/fusion/configs/multi_sensor_fusion_v1/status"
  }
}
```

### 4.2 Get Fusion Configuration

**Endpoint**: `GET /fusion/configs/{config_id}`

**Response**: `200 OK`

### 4.3 Update Fusion Configuration

**Endpoint**: `PATCH /fusion/configs/{config_id}`

**Request**:
```json
{
  "sensors": [
    {
      "sensor_id": "camera_front",
      "weight": 0.5
    }
  ]
}
```

**Response**: `200 OK`

### 4.4 Activate/Deactivate Fusion

**Endpoint**: `POST /fusion/configs/{config_id}/activate`

**Request**:
```json
{
  "active": true
}
```

**Response**: `200 OK`
```json
{
  "config_id": "multi_sensor_fusion_v1",
  "status": "active",
  "started_at": "2025-01-15T10:30:30Z"
}
```

### 4.5 List Fusion Configurations

**Endpoint**: `GET /fusion/configs`

**Response**: `200 OK`
```json
{
  "configs": [
    {
      "config_id": "multi_sensor_fusion_v1",
      "status": "active",
      "created_at": "2025-01-15T10:30:00Z"
    }
  ],
  "total": 1
}
```

---

## Data Streaming API

### 5.1 WebSocket Connection

Connect to sensor data stream.

**Endpoint**: `ws://api.wia.live/sensors/{sensor_id}/stream`

**Connection**:
```javascript
const ws = new WebSocket('ws://api.wia.live/sensors/camera_front/stream');
ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'subscribe',
    filters: {
      min_confidence: 0.8
    }
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Received:', data);
};
```

**Message Format**:
```json
{
  "type": "sensor_data",
  "sensor_id": "camera_front",
  "timestamp": 1704067200.123456789,
  "data": {
    "detections": [
      {
        "class_id": "person",
        "confidence": 0.92,
        "bbox": {"x": 450, "y": 300, "width": 180, "height": 420}
      }
    ]
  }
}
```

### 5.2 Subscribe to Fused Output

**Endpoint**: `ws://api.wia.live/fusion/{config_id}/stream`

**Message Format**:
```json
{
  "type": "fused_objects",
  "timestamp": 1704067200.123456789,
  "objects": [
    {
      "object_id": "obj_001",
      "class": "person",
      "confidence": 0.94,
      "pose": {
        "position": {"x": 2.8, "y": 0.5, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
      }
    }
  ]
}
```

---

## Fused Output Query API

### 6.1 Query Fused Objects

**Endpoint**: `GET /fusion/{config_id}/objects`

**Query Parameters**:
- `start_time`: Start timestamp (Unix time)
- `end_time`: End timestamp (Unix time)
- `class`: Filter by object class
- `min_confidence`: Minimum confidence threshold
- `limit`: Number of results

**Response**: `200 OK`
```json
{
  "objects": [
    {
      "object_id": "obj_001",
      "timestamp": 1704067200.123456789,
      "class": "person",
      "confidence": 0.94,
      "pose": {
        "position": {"x": 2.8, "y": 0.5, "z": 0.0}
      }
    }
  ],
  "total": 15,
  "limit": 50
}
```

### 6.2 Get Object Trajectory

**Endpoint**: `GET /fusion/{config_id}/objects/{object_id}/trajectory`

**Query Parameters**:
- `start_time`: Start timestamp
- `end_time`: End timestamp
- `interpolate`: Boolean, whether to interpolate missing points

**Response**: `200 OK`
```json
{
  "object_id": "obj_001",
  "trajectory": [
    {
      "timestamp": 1704067200.000,
      "position": {"x": 2.5, "y": 0.3, "z": 0.0},
      "velocity": {"x": 0.5, "y": 0.1, "z": 0.0}
    },
    {
      "timestamp": 1704067200.050,
      "position": {"x": 2.525, "y": 0.305, "z": 0.0},
      "velocity": {"x": 0.5, "y": 0.1, "z": 0.0}
    }
  ]
}
```

### 6.3 Query Fused State

**Endpoint**: `GET /fusion/{config_id}/state`

**Query Parameters**:
- `timestamp`: Specific timestamp (Unix time)
- `latest`: Boolean, get latest state

**Response**: `200 OK`
```json
{
  "timestamp": 1704067200.123456789,
  "pose": {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  },
  "velocity": {
    "linear": {"x": 1.2, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
  }
}
```

---

## Calibration API

### 7.1 Upload Calibration Data

**Endpoint**: `POST /calibration/extrinsics`

**Request**:
```json
{
  "calibration_id": "extrinsics_v1",
  "reference_frame": "base_link",
  "transforms": [
    {
      "from_frame": "base_link",
      "to_frame": "camera_front_optical",
      "translation": {"x": 0.15, "y": 0.0, "z": 0.3},
      "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  ]
}
```

**Response**: `201 Created`

### 7.2 Get Calibration Data

**Endpoint**: `GET /calibration/extrinsics/{calibration_id}`

**Response**: `200 OK`

### 7.3 Get Transform

**Endpoint**: `GET /calibration/transform`

**Query Parameters**:
- `from_frame`: Source frame ID
- `to_frame`: Target frame ID
- `timestamp`: Optional timestamp

**Response**: `200 OK`
```json
{
  "from_frame": "base_link",
  "to_frame": "camera_front_optical",
  "translation": {"x": 0.15, "y": 0.0, "z": 0.3},
  "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
  "timestamp": 1704067200.000000000
}
```

---

## Status and Health API

### 8.1 System Health

**Endpoint**: `GET /health`

**Response**: `200 OK`
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "uptime_seconds": 86400,
  "components": {
    "sensor_manager": "healthy",
    "fusion_engine": "healthy",
    "database": "healthy",
    "message_queue": "healthy"
  }
}
```

### 8.2 Fusion Status

**Endpoint**: `GET /fusion/{config_id}/status`

**Response**: `200 OK`
```json
{
  "config_id": "multi_sensor_fusion_v1",
  "status": "active",
  "uptime_seconds": 3600,
  "performance": {
    "fusion_rate_hz": 19.8,
    "latency_ms": 25.3,
    "cpu_usage_percent": 45.2,
    "memory_mb": 512.5
  },
  "sensors": [
    {
      "sensor_id": "camera_front",
      "status": "active",
      "data_rate_hz": 29.9,
      "latency_ms": 15.2
    },
    {
      "sensor_id": "lidar_top",
      "status": "active",
      "data_rate_hz": 10.0,
      "latency_ms": 8.5
    }
  ]
}
```

### 8.3 Metrics

**Endpoint**: `GET /fusion/{config_id}/metrics`

**Query Parameters**:
- `start_time`: Start timestamp
- `end_time`: End timestamp
- `metric`: Specific metric name

**Response**: `200 OK`
```json
{
  "metrics": {
    "fusion_rate_hz": {
      "average": 19.8,
      "min": 18.5,
      "max": 20.1,
      "stddev": 0.5
    },
    "latency_ms": {
      "average": 25.3,
      "min": 20.1,
      "max": 35.8,
      "stddev": 3.2
    }
  }
}
```

---

## TypeScript SDK

```typescript
import { WIASensorFusionClient } from '@wia/sensor-fusion';

const client = new WIASensorFusionClient({
  baseUrl: 'https://api.wia.live/sensor-fusion/v1',
  apiKey: 'your-api-key'
});

// Register sensor
await client.sensors.register({
  sensor_id: 'camera_front',
  sensor_type: 'camera',
  frame_id: 'camera_front_optical',
  capabilities: {
    resolution: { width: 1920, height: 1080 },
    frame_rate_hz: 30
  }
});

// Create fusion configuration
const config = await client.fusion.createConfig({
  config_id: 'my_fusion',
  algorithm: {
    type: 'kalman_filter',
    variant: 'extended_kalman_filter'
  },
  sensors: [
    { sensor_id: 'camera_front', weight: 0.5 },
    { sensor_id: 'lidar_top', weight: 0.5 }
  ]
});

// Activate fusion
await client.fusion.activate('my_fusion');

// Subscribe to fused objects
const stream = client.fusion.streamObjects('my_fusion');
stream.on('data', (objects) => {
  console.log('Fused objects:', objects);
});

// Query objects
const objects = await client.fusion.queryObjects('my_fusion', {
  start_time: Date.now() - 60000,
  end_time: Date.now(),
  min_confidence: 0.8
});
```

---

## Python SDK

```python
from wia_sensor_fusion import SensorFusionClient

# Initialize client
client = SensorFusionClient(
    base_url='https://api.wia.live/sensor-fusion/v1',
    api_key='your-api-key'
)

# Register sensor
client.sensors.register(
    sensor_id='camera_front',
    sensor_type='camera',
    frame_id='camera_front_optical',
    capabilities={
        'resolution': {'width': 1920, 'height': 1080},
        'frame_rate_hz': 30
    }
)

# Create fusion configuration
config = client.fusion.create_config(
    config_id='my_fusion',
    algorithm={
        'type': 'kalman_filter',
        'variant': 'extended_kalman_filter'
    },
    sensors=[
        {'sensor_id': 'camera_front', 'weight': 0.5},
        {'sensor_id': 'lidar_top', 'weight': 0.5}
    ]
)

# Activate fusion
client.fusion.activate('my_fusion')

# Subscribe to fused objects
async for objects in client.fusion.stream_objects('my_fusion'):
    print('Fused objects:', objects)

# Query objects
objects = client.fusion.query_objects(
    'my_fusion',
    start_time=time.time() - 60,
    end_time=time.time(),
    min_confidence=0.8
)
```

---

## Error Codes

| Code | Status | Description |
|------|--------|-------------|
| 1000 | 400 | Invalid request format |
| 1001 | 400 | Missing required field |
| 1002 | 400 | Invalid parameter value |
| 2000 | 404 | Sensor not found |
| 2001 | 404 | Fusion config not found |
| 2002 | 404 | Object not found |
| 3000 | 409 | Sensor already registered |
| 3001 | 409 | Config already exists |
| 4000 | 500 | Internal server error |
| 4001 | 500 | Fusion engine error |
| 4002 | 503 | Service unavailable |
| 5000 | 401 | Unauthorized |
| 5001 | 403 | Forbidden |
| 5002 | 429 | Rate limit exceeded |

**Error Response Format**:
```json
{
  "error": {
    "code": 2000,
    "message": "Sensor not found",
    "details": "Sensor 'camera_front' does not exist",
    "timestamp": "2025-01-15T10:30:00Z",
    "request_id": "req-123456"
  }
}
```

---

## Related Specifications

- [PHASE-1-DATA-FORMAT.md](./PHASE-1-DATA-FORMAT.md) - Data format schemas
- [PHASE-3-PROTOCOL.md](./PHASE-3-PROTOCOL.md) - Communication protocol
- [PHASE-4-INTEGRATION.md](./PHASE-4-INTEGRATION.md) - System integration

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01
**Author**: WIA AI Sensor Fusion Working Group

---

弘益人間 - *Benefit All Humanity*
