# WIA-SEMI-013 Phase 2: API Interface Specification

**Version:** 1.0  
**Date:** 2025-01-15  
**Status:** Published  

## Overview

Phase 2 defines programmatic interfaces (APIs) for 3D sensor control, data acquisition, and processing. The specification provides RESTful APIs for network access and native SDKs for TypeScript/JavaScript, Python, C++, and ROS integration.

## 1. Device Management API

### Device Discovery

**Endpoint:** `GET /api/v1/devices`

**Response:**
```json
{
  "devices": [
    {
      "id": "sensor-001",
      "name": "Sony IMX556PLR",
      "technology": "iToF",
      "status": "available",
      "firmware_version": "1.2.3"
    }
  ]
}
```

### Device Information

**Endpoint:** `GET /api/v1/devices/{device_id}`

**Response:**
```json
{
  "id": "sensor-001",
  "specification": {
    /* Full sensor spec per Phase 1 */
  },
  "capabilities": ["depth_streaming", "point_cloud", "rgb_sync"],
  "current_config": {
    "resolution": {"width": 640, "height": 480},
    "frameRate": 30,
    "depthQuality": "balanced"
  }
}
```

## 2. Configuration API

### Set Configuration

**Endpoint:** `PUT /api/v1/devices/{device_id}/config`

**Request:**
```json
{
  "resolution": {"width": 640, "height": 480},
  "frameRate": 30,
  "depthQuality": "high",
  "filters": {
    "temporal": {"enabled": true, "frames": 3},
    "spatial": {"enabled": true, "sigma": 1.5},
    "edge_preserving": true
  },
  "powerMode": "balanced"
}
```

**Response:**
```json
{
  "status": "success",
  "applied_config": { /* echo of applied configuration */ }
}
```

### Preset Modes

**Endpoint:** `PUT /api/v1/devices/{device_id}/preset`

**Request:**
```json
{
  "preset": "face_authentication"
}
```

**Presets:**
- `face_authentication`: High accuracy, short range
- `gesture_recognition`: High frame rate, medium accuracy
- `navigation`: Wide FOV, balanced accuracy/speed
- `inspection`: Maximum accuracy, controlled environment
- `long_range`: Extended range, lower resolution

## 3. Data Streaming API

### Start Streaming

**Endpoint:** `POST /api/v1/devices/{device_id}/stream/start`

**Request:**
```json
{
  "streams": ["depth", "rgb", "confidence"],
  "format": "binary",
  "transport": "websocket"
}
```

**Response:**
```json
{
  "stream_id": "stream-12345",
  "websocket_url": "ws://localhost:8080/stream/stream-12345"
}
```

### WebSocket Frame Format

**Binary Message Structure:**
```
[Header: 32 bytes]
  - frame_type: uint8 (0=depth, 1=rgb, 2=confidence)
  - sequence: uint32
  - timestamp_us: uint64
  - width: uint16
  - height: uint16
  - encoding: uint8
  - reserved: 14 bytes
[Data: variable length based on encoding]
```

### Stop Streaming

**Endpoint:** `POST /api/v1/devices/{device_id}/stream/stop`

**Request:**
```json
{
  "stream_id": "stream-12345"
}
```

## 4. Calibration API

### Get Calibration

**Endpoint:** `GET /api/v1/devices/{device_id}/calibration`

**Response:**
```json
{
  "intrinsic": { /* Per Phase 1 format */ },
  "extrinsic": { /* If multi-sensor system */ },
  "temperature_compensation": { /* If available */ }
}
```

### Set User Calibration

**Endpoint:** `PUT /api/v1/devices/{device_id}/calibration/user`

**Request:**
```json
{
  "intrinsic_override": {
    "camera_matrix": { /* User-calibrated values */ }
  }
}
```

## 5. Processing API

### Depth to Point Cloud

**Endpoint:** `POST /api/v1/processing/depth-to-pointcloud`

**Request:**
```json
{
  "depth_data": "base64_encoded_depth_map",
  "calibration_id": "sensor-001",
  "options": {
    "remove_invalid": true,
    "downsample": {"voxel_size": 0.01},
    "output_format": "pcd"
  }
}
```

**Response:**
```json
{
  "point_cloud": "base64_encoded_pcd_data",
  "point_count": 150000,
  "processing_time_ms": 45
}
```

### Filter Point Cloud

**Endpoint:** `POST /api/v1/processing/filter-pointcloud`

**Request:**
```json
{
  "point_cloud": "base64_encoded_pcd_data",
  "filters": [
    {"type": "statistical_outlier", "params": {"k": 50, "std_dev": 1.0}},
    {"type": "voxel_downsample", "params": {"voxel_size": 0.01}}
  ]
}
```

## 6. TypeScript SDK

### Installation

```bash
npm install @wia/semi-013-sdk
```

### Basic Usage

```typescript
import { WIA3DImageSensor, DepthFrame } from '@wia/semi-013-sdk';

// Initialize sensor
const sensor = new WIA3DImageSensor({
  deviceId: 'sensor-001',
  resolution: { width: 640, height: 480 },
  frameRate: 30
});

await sensor.start();

// Stream depth frames
sensor.on('depth-frame', (frame: DepthFrame) => {
  console.log(`Frame ${frame.sequenceNumber}`);
  
  // Get depth at pixel
  const depth = frame.getDepth(320, 240);
  
  // Convert to point cloud
  const pointCloud = frame.toPointCloud({
    removeInvalid: true,
    voxelSize: 0.01
  });
  
  // Export
  const pcdData = pointCloud.exportPCD();
});

// Configure
await sensor.configure({
  depthQuality: 'high',
  filters: {
    temporal: { enabled: true, frames: 3 }
  }
});

// Stop
await sensor.stop();
```

### Advanced Features

```typescript
// Multi-sensor fusion
const fusion = new WIASensorFusion({
  sensors: [sensor1, sensor2],
  method: 'weighted_average'
});

// SLAM integration
const slam = new WIASLAM({
  sensor: sensor,
  mapResolution: 0.05,
  loopClosureEnabled: true
});

slam.on('pose-update', (pose) => {
  console.log('Camera pose:', pose);
});
```

## 7. Python SDK

### Installation

```bash
pip install wia-semi-013
```

### Basic Usage

```python
from wia_semi_013 import WIA3DImageSensor

# Initialize
sensor = WIA3DImageSensor(
    device_id='sensor-001',
    resolution=(640, 480),
    frame_rate=30
)

sensor.start()

# Callback
def on_depth_frame(frame):
    depth_map = frame.get_depth_map()
    point_cloud = frame.to_point_cloud(remove_invalid=True)
    
    # Process with NumPy
    import numpy as np
    points = np.array(point_cloud.points)
    
    print(f"Points: {len(points)}")

sensor.on('depth-frame', on_depth_frame)

# Wait
import time
time.sleep(10)

sensor.stop()
```

## 8. C++ SDK

### CMake Integration

```cmake
find_package(WIA_SEMI_013 REQUIRED)
target_link_libraries(my_app WIA::SEMI_013)
```

### Basic Usage

```cpp
#include <wia/semi_013/sensor.hpp>
#include <wia/semi_013/point_cloud.hpp>

int main() {
    wia::semi_013::Sensor sensor("sensor-001");
    sensor.setResolution(640, 480);
    sensor.setFrameRate(30);
    
    sensor.start();
    
    sensor.onDepthFrame([](const wia::semi_013::DepthFrame& frame) {
        auto pointCloud = frame.toPointCloud();
        pointCloud.removeOutliers();
        pointCloud.downsample(0.01);
        
        std::cout << "Points: " << pointCloud.size() << std::endl;
    });
    
    std::this_thread::sleep_for(std::chrono::seconds(10));
    
    sensor.stop();
    return 0;
}
```

## 9. ROS Integration

### ROS2 Package

```bash
sudo apt install ros-humble-wia-semi-013
```

### Launch File

```xml
<launch>
  <node pkg="wia_semi_013" exec="sensor_node" name="depth_sensor">
    <param name="device_id" value="sensor-001"/>
    <param name="frame_rate" value="30"/>
    <param name="depth_quality" value="high"/>
  </node>
</launch>
```

### Topics Published

- `/wia/depth/image` (sensor_msgs/Image): Depth image
- `/wia/depth/camera_info` (sensor_msgs/CameraInfo): Calibration
- `/wia/points` (sensor_msgs/PointCloud2): Point cloud
- `/wia/rgb/image` (sensor_msgs/Image): RGB image (if available)

## 7. API Standards Conformance

### 7.1 Transport and Encoding

All REST endpoints conform to:

- **HTTP/1.1** per RFC 9110 (semantics) and RFC 9112 (HTTP/1.1 over TCP).
- **HTTP/2** per RFC 9113 with required ALPN negotiation `h2`.
- **TLS 1.3** per RFC 8446; TLS 1.2 (RFC 5246) is the minimum acceptable for backward compatibility, but new deployments SHOULD prefer 1.3.
- **JSON request/response bodies** per RFC 8259, with media type `application/json`.
- **CBOR alternate encoding** per RFC 8949 for bandwidth-constrained edge deployments. Negotiated via `Accept: application/cbor`.

### 7.2 OpenAPI Description

The full machine-readable interface description is published as an OpenAPI 3.1.0 document at `/api/v1/openapi.json`. Schema validation uses JSON Schema draft 2020-12 as embedded in OpenAPI 3.1.

```http
GET /api/v1/openapi.json HTTP/1.1
Host: api.example.com
Accept: application/json
```

Responses include the `OpenAPI: 3.1.0` declaration and bind every endpoint to a request schema, response schema, and at least one of the security schemes declared in §7.4.

### 7.3 Error Responses

All HTTP error responses (4xx, 5xx) MUST follow RFC 9457 *Problem Details for HTTP APIs*:

```json
{
  "type": "https://api.example.com/errors/sensor-offline",
  "title": "Sensor Offline",
  "status": 503,
  "detail": "Sensor sensor-001 has not reported telemetry for 45s.",
  "instance": "/api/v1/devices/sensor-001"
}
```

The `Content-Type` header on error responses is `application/problem+json` per RFC 9457 §3.

### 7.4 Authentication and Authorization

Three security schemes are defined:

| Scheme | Reference | Use case |
|--------|-----------|----------|
| `bearerAuth` | RFC 6750 (Bearer Token Usage) | API gateway tokens |
| `oauth2` | RFC 6749 (OAuth 2.0 Authorization Framework) | Federated client identity |
| `mutualTLS` | RFC 8705 (OAuth 2.0 Mutual-TLS Client Authentication) | Sensor-to-server attestation |

Bearer tokens follow JWT encoding per RFC 7519 with required claims `iss`, `sub`, `exp`, `iat`. Optional claims `wia:device_class` and `wia:permissions` carry sensor-specific authorization.

OAuth 2.0 flows: deployments MUST support Authorization Code with PKCE (RFC 7636) and Client Credentials (RFC 6749 §4.4). The Resource Owner Password Credentials grant is forbidden for new deployments.

### 7.5 Rate Limiting

Rate-limit headers follow IETF draft conventions used by major clouds and codified for WIA as:

```
RateLimit-Limit: 1000, 1000;w=60
RateLimit-Remaining: 947
RateLimit-Reset: 53
```

Throttled requests return HTTP 429 with `Retry-After` per RFC 9110 §10.2.3.

### 7.6 Pagination and Filtering

List endpoints use cursor-based pagination per the schema:

```http
GET /api/v1/devices?cursor=eyJpZCI6Im...&limit=100 HTTP/1.1
```

Responses include the next cursor in a `Link` header per RFC 8288 *Web Linking*:

```
Link: <https://api.example.com/api/v1/devices?cursor=eyJpZCI6Im...>; rel="next"
```

### 7.7 WebSocket Real-Time API

For live streams, the WebSocket transport per RFC 6455 is used. Subprotocol negotiation declares `Sec-WebSocket-Protocol: wia.sensor.v1`. Message framing uses JSON text frames or CBOR binary frames matching the negotiated content type.

### 7.8 Conformance Statement

An implementation is conformant with Phase 2 when:

1. Every endpoint published in OpenAPI 3.1 description validates against its declared schema.
2. All error responses use RFC 9457 problem details.
3. All authenticated endpoints accept at least the `bearerAuth` scheme.
4. Rate-limit headers are emitted on all endpoints subject to throttling.

## 8. Implementation Notes

### 8.1 OpenAPI tooling

Reference tools used during conformance testing:

- **swagger-cli** for syntactic validation of the published `openapi.json`.
- **Spectral** (Stoplight) with the WIA ruleset (`spectral.yaml`) for stylistic conformance.
- **Schemathesis** for property-based fuzzing of every endpoint against its declared schema.
- **k6** for rate-limit conformance load testing.

### 8.2 Versioning policy

API path versioning (`/api/v1`, `/api/v2`) follows semantic-versioning principles aligned with the registry of breaking changes maintained at the published OpenAPI document URL. Minor versions (`v1.1`, `v1.2`) MUST remain wire-compatible with the prior minor of the same major. Major bumps (`v2`) coexist with `v1` for at least 12 months before deprecation.

### 8.3 Header registry

Custom WIA headers are prefixed `X-Wia-` and registered in the appendix of this Phase. Implementations MUST NOT introduce additional `X-Wia-` headers without coordinating with the WIA registry custodian.

---

**© 2025 SmileStory Inc. / WIA - World Certification Industry Association**  
**弘益人間 · Benefit All Humanity**
