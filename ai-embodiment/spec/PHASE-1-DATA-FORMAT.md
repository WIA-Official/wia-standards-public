# WIA AI Embodiment Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #3B82F6 (Blue)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Structure](#base-structure)
4. [Data Schema](#data-schema)
5. [Field Specifications](#field-specifications)
6. [Data Types](#data-types)
7. [Validation Rules](#validation-rules)
8. [Examples](#examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA AI Embodiment Data Format Standard defines a comprehensive data structure for integrating artificial intelligence systems with physical embodiments such as robots, prosthetics, and autonomous systems. This standard ensures interoperability between AI decision-making systems and physical actuators.

**Core Objectives**:
- Define standardized data formats for AI-to-physical system communication
- Establish sensor input and actuator output data structures
- Enable real-time coordination between AI cognition and physical actions
- Provide safety-critical data validation for embodied AI systems
- Support multi-modal sensory integration

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Sensor Integration | Standardized formats for multi-modal sensor data |
| Motor Control | Data structures for actuator commands |
| State Management | Robot/prosthetic state representation |
| Safety Constraints | Physical safety limit specifications |
| Coordination | Multi-joint and multi-limb synchronization |

### 1.3 Design Principles

1. **Real-Time Priority**: Optimized for low-latency physical control
2. **Safety-First**: Built-in safety constraint specifications
3. **Modularity**: Component-based architecture for various embodiments
4. **Extensibility**: Support for custom actuator types
5. **Interoperability**: Compatible with ROS2 and industry standards

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Embodiment** | Physical system controlled by AI (robot, prosthetic, vehicle) |
| **Actuator** | Motor, servo, or mechanism producing physical movement |
| **Sensor Fusion** | Integration of multiple sensor inputs |
| **Control Loop** | Feedback cycle between sensing and actuation |
| **DOF** | Degrees of Freedom - axes of movement |
| **Kinematic Chain** | Series of connected joints and links |
| **End Effector** | Terminal component (hand, gripper, tool) |
| **Proprioception** | Internal state sensing (position, force, velocity) |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"left_arm_joint_1"` |
| `position` | 3D coordinates (meters) | `{"x": 0.5, "y": 0.2, "z": 0.8}` |
| `orientation` | Quaternion rotation | `{"w": 1.0, "qx": 0, "qy": 0, "qz": 0}` |
| `velocity` | Speed (m/s or rad/s) | `1.5` |
| `torque` | Force moment (Nm) | `12.5` |
| `timestamp` | Unix milliseconds | `1704067200000` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Embodiment State Format

Every AI Embodiment message follows this base structure:

```json
{
  "$schema": "https://wia.live/ai-embodiment/v1/schema.json",
  "version": "1.0.0",
  "timestamp": {
    "unix_ms": 1704067200000,
    "iso": "2025-01-01T00:00:00.000Z"
  },
  "embodiment_id": "embodiment-uuid-here",
  "embodiment_type": "humanoid_robot",
  "state": {},
  "sensors": {},
  "actuators": {},
  "safety": {},
  "meta": {}
}
```

### 3.2 Top-Level Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `$schema` | string | Y | JSON Schema URL |
| `version` | string | Y | Standard version (SemVer) |
| `timestamp` | object | Y | Message timestamp |
| `embodiment_id` | string | Y | Unique embodiment identifier (UUID v4) |
| `embodiment_type` | string | Y | Embodiment category |
| `state` | object | Y | Current embodiment state |
| `sensors` | object | N | Sensor data readings |
| `actuators` | object | N | Actuator commands/status |
| `safety` | object | Y | Safety constraints and status |
| `meta` | object | N | Additional metadata |

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/ai-embodiment/v1/schema.json",
  "title": "WIA AI Embodiment Data Format",
  "type": "object",
  "required": ["$schema", "version", "timestamp", "embodiment_id", "embodiment_type", "state", "safety"],
  "properties": {
    "$schema": {
      "type": "string",
      "format": "uri"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "timestamp": {
      "type": "object",
      "required": ["unix_ms"],
      "properties": {
        "unix_ms": { "type": "integer", "minimum": 0 },
        "iso": { "type": "string", "format": "date-time" }
      }
    },
    "embodiment_id": {
      "type": "string",
      "format": "uuid"
    },
    "embodiment_type": {
      "type": "string",
      "enum": ["humanoid_robot", "robotic_arm", "mobile_robot", "prosthetic", "exoskeleton", "autonomous_vehicle", "drone", "custom"]
    },
    "state": {
      "$ref": "#/definitions/EmbodimentState"
    },
    "sensors": {
      "$ref": "#/definitions/SensorData"
    },
    "actuators": {
      "$ref": "#/definitions/ActuatorData"
    },
    "safety": {
      "$ref": "#/definitions/SafetyData"
    },
    "meta": {
      "type": "object"
    }
  },
  "definitions": {
    "EmbodimentState": {
      "type": "object",
      "properties": {
        "operational_mode": {
          "type": "string",
          "enum": ["idle", "active", "emergency_stop", "calibrating", "maintenance"]
        },
        "power_status": {
          "type": "object",
          "properties": {
            "battery_level": { "type": "number", "minimum": 0, "maximum": 100 },
            "voltage": { "type": "number" },
            "current": { "type": "number" },
            "charging": { "type": "boolean" }
          }
        },
        "base_position": { "$ref": "#/definitions/Pose3D" },
        "joints": {
          "type": "array",
          "items": { "$ref": "#/definitions/JointState" }
        },
        "end_effectors": {
          "type": "array",
          "items": { "$ref": "#/definitions/EndEffectorState" }
        }
      }
    },
    "Pose3D": {
      "type": "object",
      "properties": {
        "position": {
          "type": "object",
          "properties": {
            "x": { "type": "number" },
            "y": { "type": "number" },
            "z": { "type": "number" }
          }
        },
        "orientation": {
          "type": "object",
          "properties": {
            "w": { "type": "number" },
            "qx": { "type": "number" },
            "qy": { "type": "number" },
            "qz": { "type": "number" }
          }
        }
      }
    },
    "JointState": {
      "type": "object",
      "required": ["joint_id", "position"],
      "properties": {
        "joint_id": { "type": "string" },
        "joint_type": { "type": "string", "enum": ["revolute", "prismatic", "continuous", "fixed"] },
        "position": { "type": "number" },
        "velocity": { "type": "number" },
        "acceleration": { "type": "number" },
        "torque": { "type": "number" },
        "temperature": { "type": "number" }
      }
    },
    "EndEffectorState": {
      "type": "object",
      "properties": {
        "effector_id": { "type": "string" },
        "effector_type": { "type": "string" },
        "pose": { "$ref": "#/definitions/Pose3D" },
        "grip_force": { "type": "number" },
        "grip_state": { "type": "string", "enum": ["open", "closed", "grasping"] }
      }
    },
    "SensorData": {
      "type": "object",
      "properties": {
        "imu": { "$ref": "#/definitions/IMUData" },
        "force_torque": { "type": "array", "items": { "$ref": "#/definitions/ForceTorqueData" } },
        "proximity": { "type": "array", "items": { "$ref": "#/definitions/ProximityData" } },
        "tactile": { "type": "array", "items": { "$ref": "#/definitions/TactileData" } },
        "vision": { "type": "array", "items": { "$ref": "#/definitions/VisionData" } }
      }
    },
    "IMUData": {
      "type": "object",
      "properties": {
        "acceleration": { "type": "object", "properties": { "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"} } },
        "angular_velocity": { "type": "object", "properties": { "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"} } },
        "orientation": { "$ref": "#/definitions/Pose3D/properties/orientation" }
      }
    },
    "ForceTorqueData": {
      "type": "object",
      "properties": {
        "sensor_id": { "type": "string" },
        "force": { "type": "object", "properties": { "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"} } },
        "torque": { "type": "object", "properties": { "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"} } }
      }
    },
    "ProximityData": {
      "type": "object",
      "properties": {
        "sensor_id": { "type": "string" },
        "distance": { "type": "number" },
        "object_detected": { "type": "boolean" }
      }
    },
    "TactileData": {
      "type": "object",
      "properties": {
        "sensor_id": { "type": "string" },
        "pressure_map": { "type": "array", "items": { "type": "array", "items": { "type": "number" } } },
        "contact_detected": { "type": "boolean" }
      }
    },
    "VisionData": {
      "type": "object",
      "properties": {
        "camera_id": { "type": "string" },
        "frame_id": { "type": "integer" },
        "detected_objects": { "type": "array", "items": { "$ref": "#/definitions/DetectedObject" } }
      }
    },
    "DetectedObject": {
      "type": "object",
      "properties": {
        "object_id": { "type": "string" },
        "class": { "type": "string" },
        "confidence": { "type": "number", "minimum": 0, "maximum": 1 },
        "bounding_box": { "type": "object" },
        "pose": { "$ref": "#/definitions/Pose3D" }
      }
    },
    "ActuatorData": {
      "type": "object",
      "properties": {
        "commands": { "type": "array", "items": { "$ref": "#/definitions/ActuatorCommand" } },
        "status": { "type": "array", "items": { "$ref": "#/definitions/ActuatorStatus" } }
      }
    },
    "ActuatorCommand": {
      "type": "object",
      "required": ["actuator_id", "command_type"],
      "properties": {
        "actuator_id": { "type": "string" },
        "command_type": { "type": "string", "enum": ["position", "velocity", "torque", "impedance"] },
        "target_value": { "type": "number" },
        "duration_ms": { "type": "integer" },
        "priority": { "type": "integer", "minimum": 0, "maximum": 10 }
      }
    },
    "ActuatorStatus": {
      "type": "object",
      "properties": {
        "actuator_id": { "type": "string" },
        "state": { "type": "string", "enum": ["idle", "moving", "stalled", "error"] },
        "error_code": { "type": "integer" },
        "temperature": { "type": "number" }
      }
    },
    "SafetyData": {
      "type": "object",
      "required": ["safety_mode", "constraints"],
      "properties": {
        "safety_mode": { "type": "string", "enum": ["normal", "reduced", "collaborative", "emergency"] },
        "emergency_stop_active": { "type": "boolean" },
        "constraints": { "$ref": "#/definitions/SafetyConstraints" },
        "violations": { "type": "array", "items": { "$ref": "#/definitions/SafetyViolation" } }
      }
    },
    "SafetyConstraints": {
      "type": "object",
      "properties": {
        "max_velocity": { "type": "number" },
        "max_force": { "type": "number" },
        "workspace_limits": { "type": "object" },
        "restricted_zones": { "type": "array" },
        "human_proximity_threshold": { "type": "number" }
      }
    },
    "SafetyViolation": {
      "type": "object",
      "properties": {
        "violation_type": { "type": "string" },
        "severity": { "type": "string", "enum": ["warning", "critical", "emergency"] },
        "timestamp": { "type": "integer" },
        "details": { "type": "string" }
      }
    }
  }
}
```

---

## Field Specifications

### 5.1 Embodiment Type Definitions

| Type | Description | Typical DOF |
|------|-------------|-------------|
| `humanoid_robot` | Full humanoid form factor | 20-50+ |
| `robotic_arm` | Industrial/collaborative arm | 6-7 |
| `mobile_robot` | Wheeled/tracked platforms | 2-4 |
| `prosthetic` | Limb replacement devices | 1-20 |
| `exoskeleton` | Wearable powered assistance | 4-20 |
| `autonomous_vehicle` | Self-driving vehicles | 2-6 |
| `drone` | Unmanned aerial vehicles | 4-8 |
| `custom` | User-defined embodiment | Variable |

### 5.2 Joint Type Specifications

| Joint Type | Motion | Position Unit | Velocity Unit |
|------------|--------|---------------|---------------|
| `revolute` | Rotation (limited) | radians | rad/s |
| `prismatic` | Linear (limited) | meters | m/s |
| `continuous` | Rotation (unlimited) | radians | rad/s |
| `fixed` | No motion | N/A | N/A |

### 5.3 Safety Mode Specifications

| Mode | Max Velocity | Max Force | Human Interaction |
|------|-------------|-----------|-------------------|
| `normal` | 100% | 100% | Not allowed |
| `reduced` | 50% | 50% | Limited |
| `collaborative` | 25% | 25% | Full interaction |
| `emergency` | 0% | 0% | Immediate stop |

---

## Data Types

### 6.1 Coordinate Systems

All spatial data uses right-handed coordinate systems:
- **X-axis**: Forward
- **Y-axis**: Left
- **Z-axis**: Up

Units follow SI conventions:
- Position: meters (m)
- Rotation: radians (rad)
- Force: Newtons (N)
- Torque: Newton-meters (Nm)

### 6.2 Quaternion Representation

Orientation uses quaternion format (w, x, y, z):
```json
{
  "w": 1.0,
  "qx": 0.0,
  "qy": 0.0,
  "qz": 0.0
}
```

Normalization constraint: w² + x² + y² + z² = 1

---

## Validation Rules

### 7.1 Required Field Validation

| Rule | Condition | Error Code |
|------|-----------|------------|
| Schema presence | `$schema` must be valid URL | VR001 |
| Version format | Must match SemVer pattern | VR002 |
| Timestamp validity | `unix_ms` > 0 | VR003 |
| UUID format | `embodiment_id` must be valid UUID v4 | VR004 |
| Type validity | `embodiment_type` must be in enum | VR005 |

### 7.2 Physical Constraints

| Constraint | Valid Range | Error Code |
|------------|-------------|------------|
| Joint position | Within joint limits | PC001 |
| Velocity | 0 to max_velocity | PC002 |
| Torque | 0 to max_torque | PC003 |
| Temperature | -40°C to 85°C | PC004 |
| Battery level | 0% to 100% | PC005 |

### 7.3 Safety Validation

| Check | Description | Severity |
|-------|-------------|----------|
| Workspace bounds | Position within defined workspace | Critical |
| Collision proximity | Distance to obstacles | Warning/Critical |
| Force limits | Contact forces within safe range | Critical |
| Thermal limits | Component temperatures | Warning |

---

## Examples

### 8.1 Humanoid Robot State

```json
{
  "$schema": "https://wia.live/ai-embodiment/v1/schema.json",
  "version": "1.0.0",
  "timestamp": {
    "unix_ms": 1704067200000,
    "iso": "2025-01-01T00:00:00.000Z"
  },
  "embodiment_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "embodiment_type": "humanoid_robot",
  "state": {
    "operational_mode": "active",
    "power_status": {
      "battery_level": 85.5,
      "voltage": 48.2,
      "current": 12.5,
      "charging": false
    },
    "base_position": {
      "position": { "x": 1.5, "y": 2.0, "z": 0.0 },
      "orientation": { "w": 0.707, "qx": 0.0, "qy": 0.0, "qz": 0.707 }
    },
    "joints": [
      {
        "joint_id": "right_shoulder_pitch",
        "joint_type": "revolute",
        "position": 0.5,
        "velocity": 0.1,
        "torque": 15.0,
        "temperature": 42.0
      },
      {
        "joint_id": "right_elbow_pitch",
        "joint_type": "revolute",
        "position": 1.2,
        "velocity": 0.05,
        "torque": 8.0,
        "temperature": 38.0
      }
    ],
    "end_effectors": [
      {
        "effector_id": "right_hand",
        "effector_type": "gripper",
        "pose": {
          "position": { "x": 0.8, "y": -0.3, "z": 1.0 },
          "orientation": { "w": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0 }
        },
        "grip_force": 25.0,
        "grip_state": "grasping"
      }
    ]
  },
  "sensors": {
    "imu": {
      "acceleration": { "x": 0.1, "y": -0.05, "z": 9.81 },
      "angular_velocity": { "x": 0.01, "y": 0.02, "z": 0.0 },
      "orientation": { "w": 0.999, "qx": 0.01, "qy": 0.02, "qz": 0.0 }
    },
    "force_torque": [
      {
        "sensor_id": "right_wrist_ft",
        "force": { "x": 2.5, "y": 0.5, "z": -15.0 },
        "torque": { "x": 0.1, "y": 0.2, "z": 0.05 }
      }
    ]
  },
  "safety": {
    "safety_mode": "collaborative",
    "emergency_stop_active": false,
    "constraints": {
      "max_velocity": 0.5,
      "max_force": 150.0,
      "human_proximity_threshold": 0.3
    },
    "violations": []
  },
  "meta": {
    "manufacturer": "WIA Robotics",
    "model": "HumanoidX-1",
    "firmware_version": "2.1.0"
  }
}
```

### 8.2 Prosthetic Arm State

```json
{
  "$schema": "https://wia.live/ai-embodiment/v1/schema.json",
  "version": "1.0.0",
  "timestamp": {
    "unix_ms": 1704067200100,
    "iso": "2025-01-01T00:00:00.100Z"
  },
  "embodiment_id": "b2c3d4e5-f6a7-8901-bcde-f23456789012",
  "embodiment_type": "prosthetic",
  "state": {
    "operational_mode": "active",
    "power_status": {
      "battery_level": 72.0,
      "voltage": 7.4,
      "charging": false
    },
    "joints": [
      { "joint_id": "wrist_rotation", "joint_type": "continuous", "position": 0.0, "velocity": 0.0 },
      { "joint_id": "thumb_flex", "joint_type": "revolute", "position": 0.8, "velocity": 0.0 },
      { "joint_id": "index_flex", "joint_type": "revolute", "position": 1.0, "velocity": 0.0 },
      { "joint_id": "middle_flex", "joint_type": "revolute", "position": 1.0, "velocity": 0.0 },
      { "joint_id": "ring_flex", "joint_type": "revolute", "position": 1.0, "velocity": 0.0 },
      { "joint_id": "pinky_flex", "joint_type": "revolute", "position": 1.0, "velocity": 0.0 }
    ],
    "end_effectors": [
      {
        "effector_id": "hand",
        "effector_type": "prosthetic_hand",
        "grip_force": 35.0,
        "grip_state": "grasping"
      }
    ]
  },
  "sensors": {
    "tactile": [
      {
        "sensor_id": "thumb_tip",
        "pressure_map": [[0.5, 0.6], [0.4, 0.7]],
        "contact_detected": true
      }
    ]
  },
  "safety": {
    "safety_mode": "normal",
    "emergency_stop_active": false,
    "constraints": {
      "max_velocity": 2.0,
      "max_force": 50.0
    },
    "violations": []
  }
}
```

### 8.3 Mobile Robot State

```json
{
  "$schema": "https://wia.live/ai-embodiment/v1/schema.json",
  "version": "1.0.0",
  "timestamp": {
    "unix_ms": 1704067200200,
    "iso": "2025-01-01T00:00:00.200Z"
  },
  "embodiment_id": "c3d4e5f6-a7b8-9012-cdef-345678901234",
  "embodiment_type": "mobile_robot",
  "state": {
    "operational_mode": "active",
    "power_status": {
      "battery_level": 95.0,
      "voltage": 24.0,
      "charging": false
    },
    "base_position": {
      "position": { "x": 5.2, "y": 3.8, "z": 0.0 },
      "orientation": { "w": 0.866, "qx": 0.0, "qy": 0.0, "qz": 0.5 }
    }
  },
  "sensors": {
    "proximity": [
      { "sensor_id": "lidar_front", "distance": 2.5, "object_detected": true },
      { "sensor_id": "lidar_rear", "distance": 5.0, "object_detected": false }
    ],
    "vision": [
      {
        "camera_id": "rgb_front",
        "frame_id": 12345,
        "detected_objects": [
          {
            "object_id": "obj_001",
            "class": "person",
            "confidence": 0.95,
            "pose": {
              "position": { "x": 3.0, "y": 1.5, "z": 0.0 }
            }
          }
        ]
      }
    ]
  },
  "actuators": {
    "commands": [
      { "actuator_id": "left_wheel", "command_type": "velocity", "target_value": 0.5 },
      { "actuator_id": "right_wheel", "command_type": "velocity", "target_value": 0.5 }
    ]
  },
  "safety": {
    "safety_mode": "collaborative",
    "emergency_stop_active": false,
    "constraints": {
      "max_velocity": 1.0,
      "human_proximity_threshold": 0.5
    },
    "violations": []
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA AI Embodiment Data Format Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
