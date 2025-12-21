# WIA AI Sensor Fusion Data Format Specification

**Phase 1: Data Format Standard**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Primary Color**: #14B8A6 (Teal)

---

## Overview

### 1.1 Purpose

WIA AI Sensor Fusion Data Format defines standard structures for multi-modal sensor data integration, fusion configurations, and fused output representations. This standard enables interoperability between diverse sensor systems and fusion algorithms in AI-powered robotic and autonomous systems.

### 1.2 Scope

- **In Scope**:
  - Multi-modal sensor data (camera, LIDAR, IMU, force-torque, radar, ultrasonic)
  - Fusion configurations and parameters
  - Fused output data structures
  - Time synchronization metadata
  - Uncertainty quantification
  - Coordinate transformations
  - Sensor calibration data

- **Out of Scope** (Later Phases):
  - Real-time streaming protocols (Phase 3)
  - Fusion algorithm implementations (Phase 2)
  - Integration with external systems (Phase 4)

### 1.3 Design Principles

1. **Multi-modal**: Support diverse sensor types
2. **Timestamped**: Precise temporal alignment
3. **Uncertainty-aware**: Quantify confidence levels
4. **Coordinate-consistent**: Unified reference frames
5. **Extensible**: Add new sensor modalities easily

---

## Data Structure

### 2.1 Top-Level Structure

```
wia-sensor-fusion/
├── config.json              # Fusion configuration
├── sensors/
│   ├── camera_0.json        # Camera sensor data
│   ├── lidar_0.json         # LIDAR sensor data
│   ├── imu_0.json           # IMU sensor data
│   ├── force_torque_0.json  # Force-torque sensor data
│   └── radar_0.json         # Radar sensor data
├── calibration/
│   ├── extrinsics.json      # Sensor-to-sensor transforms
│   └── intrinsics.json      # Sensor-specific calibration
├── fusion/
│   ├── objects.json         # Fused object detections
│   ├── state.json           # Fused state estimate
│   └── occupancy.json       # Fused occupancy grid
└── metadata.json            # Session metadata
```

---

## Sensor Data Schemas

### 3.1 SensorFrame (Base Schema)

All sensor data inherits from this base structure:

```json
{
  "$schema": "https://wia.live/schemas/sensor-fusion/sensor-frame.schema.json",
  "sensor_id": "camera_0",
  "sensor_type": "camera",
  "timestamp": 1704067200.123456789,
  "frame_id": "sensor_frame",
  "sequence": 12345,
  "data": {},
  "quality": {
    "status": "good",
    "confidence": 0.95,
    "latency_ms": 12.5
  }
}
```

**Fields**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `sensor_id` | string | Yes | Unique sensor identifier |
| `sensor_type` | enum | Yes | Sensor modality type |
| `timestamp` | number | Yes | Unix timestamp (seconds, nanosecond precision) |
| `frame_id` | string | Yes | Coordinate frame reference |
| `sequence` | integer | Yes | Monotonic sequence number |
| `data` | object | Yes | Sensor-specific data payload |
| `quality` | Quality | Yes | Data quality metrics |

### 3.2 Camera Data

```json
{
  "$schema": "https://wia.live/schemas/sensor-fusion/camera.schema.json",
  "sensor_id": "camera_front",
  "sensor_type": "camera",
  "timestamp": 1704067200.123456789,
  "frame_id": "camera_front_optical",
  "sequence": 100,
  "data": {
    "image": {
      "encoding": "rgb8",
      "width": 1920,
      "height": 1080,
      "data_uri": "data:image/jpeg;base64,..."
    },
    "detections": [
      {
        "class_id": "person",
        "confidence": 0.92,
        "bbox": {
          "x": 450,
          "y": 300,
          "width": 180,
          "height": 420
        },
        "keypoints": [],
        "features": []
      }
    ]
  },
  "quality": {
    "status": "good",
    "confidence": 0.95,
    "latency_ms": 15.2,
    "brightness": 128,
    "contrast": 0.75
  }
}
```

### 3.3 LIDAR Data

```json
{
  "$schema": "https://wia.live/schemas/sensor-fusion/lidar.schema.json",
  "sensor_id": "lidar_top",
  "sensor_type": "lidar",
  "timestamp": 1704067200.123456789,
  "frame_id": "lidar_top",
  "sequence": 100,
  "data": {
    "point_cloud": {
      "format": "xyzi",
      "num_points": 65536,
      "points": [
        {"x": 2.5, "y": 1.2, "z": 0.3, "intensity": 0.8},
        {"x": 3.1, "y": -0.5, "z": 0.2, "intensity": 0.6}
      ]
    },
    "clusters": [
      {
        "cluster_id": 1,
        "num_points": 150,
        "centroid": {"x": 2.8, "y": 0.5, "z": 0.3},
        "bbox": {
          "min": {"x": 2.5, "y": 0.2, "z": 0.0},
          "max": {"x": 3.1, "y": 0.8, "z": 1.8}
        }
      }
    ]
  },
  "quality": {
    "status": "good",
    "confidence": 0.98,
    "latency_ms": 8.3,
    "return_rate": 0.95
  }
}
```

### 3.4 IMU Data

```json
{
  "$schema": "https://wia.live/schemas/sensor-fusion/imu.schema.json",
  "sensor_id": "imu_base",
  "sensor_type": "imu",
  "timestamp": 1704067200.123456789,
  "frame_id": "imu_link",
  "sequence": 1000,
  "data": {
    "linear_acceleration": {
      "x": 0.05,
      "y": -0.02,
      "z": -9.81
    },
    "angular_velocity": {
      "x": 0.001,
      "y": -0.002,
      "z": 0.015
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.707,
      "w": 0.707
    },
    "magnetometer": {
      "x": 25.3,
      "y": -12.1,
      "z": 45.8
    }
  },
  "quality": {
    "status": "good",
    "confidence": 0.99,
    "latency_ms": 2.1,
    "temperature_c": 45.2
  }
}
```

### 3.5 Force-Torque Sensor Data

```json
{
  "$schema": "https://wia.live/schemas/sensor-fusion/force-torque.schema.json",
  "sensor_id": "ft_wrist",
  "sensor_type": "force_torque",
  "timestamp": 1704067200.123456789,
  "frame_id": "wrist_ft_link",
  "sequence": 500,
  "data": {
    "force": {
      "x": 2.5,
      "y": -1.3,
      "z": 12.8
    },
    "torque": {
      "x": 0.15,
      "y": -0.08,
      "z": 0.22
    },
    "wrench_validity": true
  },
  "quality": {
    "status": "good",
    "confidence": 0.97,
    "latency_ms": 3.2,
    "noise_level": 0.05
  }
}
```

### 3.6 Radar Data

```json
{
  "$schema": "https://wia.live/schemas/sensor-fusion/radar.schema.json",
  "sensor_id": "radar_front",
  "sensor_type": "radar",
  "timestamp": 1704067200.123456789,
  "frame_id": "radar_front",
  "sequence": 200,
  "data": {
    "targets": [
      {
        "target_id": 1,
        "range": 15.2,
        "azimuth": 0.15,
        "elevation": 0.02,
        "range_rate": -2.5,
        "rcs": 12.3,
        "snr": 25.8
      }
    ]
  },
  "quality": {
    "status": "good",
    "confidence": 0.88,
    "latency_ms": 10.5
  }
}
```

---

## Fusion Configuration

### 4.1 FusionConfig Schema

```json
{
  "$schema": "https://wia.live/schemas/sensor-fusion/fusion-config.schema.json",
  "fusion_id": "multi_sensor_fusion_v1",
  "version": "1.0.0",
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
      "weight": 0.5,
      "update_rate_hz": 10.0,
      "timeout_ms": 150
    },
    {
      "sensor_id": "imu_base",
      "enabled": true,
      "weight": 0.1,
      "update_rate_hz": 100.0,
      "timeout_ms": 20
    }
  ],
  "synchronization": {
    "method": "nearest_timestamp",
    "max_time_diff_ms": 50,
    "interpolation": "linear"
  },
  "coordinate_frames": {
    "world_frame": "world",
    "base_frame": "base_link",
    "gravity_direction": {"x": 0, "y": 0, "z": -9.81}
  },
  "output": {
    "fused_objects": true,
    "fused_state": true,
    "occupancy_grid": false,
    "publish_rate_hz": 20.0
  }
}
```

---

## Fused Output Data

### 5.1 FusedObject Schema

```json
{
  "$schema": "https://wia.live/schemas/sensor-fusion/fused-object.schema.json",
  "object_id": "obj_001",
  "timestamp": 1704067200.123456789,
  "frame_id": "world",
  "class": "person",
  "confidence": 0.94,
  "pose": {
    "position": {"x": 2.8, "y": 0.5, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707},
    "covariance": [
      0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.001
    ]
  },
  "velocity": {
    "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
  },
  "dimensions": {
    "length": 0.6,
    "width": 0.4,
    "height": 1.8
  },
  "sources": [
    {"sensor_id": "camera_front", "weight": 0.4},
    {"sensor_id": "lidar_top", "weight": 0.6}
  ],
  "tracking": {
    "track_id": 15,
    "age_frames": 120,
    "lost_frames": 0
  }
}
```

### 5.2 FusedState Schema

```json
{
  "$schema": "https://wia.live/schemas/sensor-fusion/fused-state.schema.json",
  "timestamp": 1704067200.123456789,
  "frame_id": "world",
  "pose": {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "covariance": []
  },
  "velocity": {
    "linear": {"x": 1.2, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
  },
  "acceleration": {
    "linear": {"x": 0.05, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.01}
  },
  "sources": [
    {"sensor_id": "imu_base", "weight": 0.6},
    {"sensor_id": "lidar_top", "weight": 0.4}
  ]
}
```

---

## Calibration Data

### 6.1 Extrinsic Calibration

```json
{
  "$schema": "https://wia.live/schemas/sensor-fusion/extrinsics.schema.json",
  "calibration_id": "extrinsics_v1",
  "reference_frame": "base_link",
  "timestamp": 1704067200.000000000,
  "transforms": [
    {
      "from_frame": "base_link",
      "to_frame": "camera_front_optical",
      "translation": {"x": 0.15, "y": 0.0, "z": 0.3},
      "rotation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      },
      "uncertainty": {
        "translation_m": 0.001,
        "rotation_rad": 0.01
      }
    },
    {
      "from_frame": "base_link",
      "to_frame": "lidar_top",
      "translation": {"x": 0.0, "y": 0.0, "z": 0.5},
      "rotation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      },
      "uncertainty": {
        "translation_m": 0.0005,
        "rotation_rad": 0.005
      }
    }
  ]
}
```

---

## TypeScript Interfaces

```typescript
// Sensor Frame Base
interface SensorFrame {
  sensor_id: string;
  sensor_type: SensorType;
  timestamp: number;
  frame_id: string;
  sequence: number;
  data: any;
  quality: QualityMetrics;
}

enum SensorType {
  CAMERA = 'camera',
  LIDAR = 'lidar',
  IMU = 'imu',
  FORCE_TORQUE = 'force_torque',
  RADAR = 'radar',
  ULTRASONIC = 'ultrasonic'
}

interface QualityMetrics {
  status: 'good' | 'degraded' | 'poor' | 'failed';
  confidence: number;  // 0.0 - 1.0
  latency_ms: number;
  [key: string]: any;  // Additional sensor-specific metrics
}

// Fused Object
interface FusedObject {
  object_id: string;
  timestamp: number;
  frame_id: string;
  class: string;
  confidence: number;
  pose: Pose;
  velocity: Twist;
  dimensions: Dimensions;
  sources: SensorSource[];
  tracking: TrackingInfo;
}

interface Pose {
  position: Vector3;
  orientation: Quaternion;
  covariance: number[];  // 6x6 covariance matrix
}

interface Vector3 {
  x: number;
  y: number;
  z: number;
}

interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

interface SensorSource {
  sensor_id: string;
  weight: number;  // 0.0 - 1.0
}
```

---

## Python Data Classes

```python
from dataclasses import dataclass
from typing import List, Dict, Any
from enum import Enum

class SensorType(Enum):
    CAMERA = 'camera'
    LIDAR = 'lidar'
    IMU = 'imu'
    FORCE_TORQUE = 'force_torque'
    RADAR = 'radar'
    ULTRASONIC = 'ultrasonic'

@dataclass
class Vector3:
    x: float
    y: float
    z: float

@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float

@dataclass
class QualityMetrics:
    status: str  # 'good', 'degraded', 'poor', 'failed'
    confidence: float  # 0.0 - 1.0
    latency_ms: float
    extra: Dict[str, Any] = None

@dataclass
class SensorFrame:
    sensor_id: str
    sensor_type: SensorType
    timestamp: float
    frame_id: str
    sequence: int
    data: Dict[str, Any]
    quality: QualityMetrics

@dataclass
class Pose:
    position: Vector3
    orientation: Quaternion
    covariance: List[float]  # 6x6 = 36 elements

@dataclass
class FusedObject:
    object_id: str
    timestamp: float
    frame_id: str
    class_name: str
    confidence: float
    pose: Pose
    velocity: Dict[str, Vector3]
    dimensions: Dict[str, float]
    sources: List[Dict[str, Any]]
    tracking: Dict[str, Any]
```

---

## Validation Rules

### 8.1 Timestamp Validation

1. Timestamps MUST be Unix time in seconds with nanosecond precision
2. Timestamps MUST be monotonically increasing within a sensor stream
3. Timestamps MUST be synchronized to a common time source (UTC, GPS, PTP)

### 8.2 Coordinate Frame Validation

1. All coordinate frames MUST follow right-handed convention
2. Frame IDs MUST be unique within a system
3. Transformations MUST be valid (rotation quaternions normalized)

### 8.3 Data Quality

1. Confidence values MUST be in range [0.0, 1.0]
2. Sensor data with confidence < 0.3 SHOULD be rejected
3. Latency SHOULD be monitored and logged if > 100ms

---

## Related Specifications

- [PHASE-2-API-INTERFACE.md](./PHASE-2-API-INTERFACE.md) - API for sensor fusion
- [PHASE-3-PROTOCOL.md](./PHASE-3-PROTOCOL.md) - Communication protocol
- [PHASE-4-INTEGRATION.md](./PHASE-4-INTEGRATION.md) - System integration

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01
**Author**: WIA AI Sensor Fusion Working Group

---

弘益人間 - *Benefit All Humanity*
