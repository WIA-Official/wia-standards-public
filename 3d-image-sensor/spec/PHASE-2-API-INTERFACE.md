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

---

**© 2025 SmileStory Inc. / WIA - World Certification Industry Association**  
**弘益人間 · Benefit All Humanity**
