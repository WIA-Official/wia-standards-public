# WIA Assistive Wearable Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Final  
**Last Updated:** 2025-12-25  
**Author:** WIA Technical Committee  
**Standard ID:** WIA-AAC-015

---

## Table of Contents

1. [Introduction](#introduction)
2. [Device Data Model](#device-data-model)
3. [Sensor Data Formats](#sensor-data-formats)
4. [Event and Telemetry Formats](#event-and-telemetry-formats)
5. [Multi-Sensor Fusion](#multi-sensor-fusion)
6. [JSON Schema Definitions](#json-schema-definitions)
7. [Validation Rules](#validation-rules)
8. [Example Payloads](#example-payloads)

---

## 1. Introduction

This specification defines the standardized data format for representing assistive wearable device information in the WIA ecosystem. The format is designed to be:

- **Universal**: Support all wearable device types and sensors
- **Extensible**: Allow for future enhancements and custom extensions
- **Efficient**: Optimize for transmission over various protocols
- **Multi-Modal**: Support diverse interaction modalities (voice, haptic, visual)
- **Privacy-Preserving**: Enable local processing and encrypted transmission

### 1.1 Design Principles

1. **Interoperability**: Compatible with existing accessibility standards (WCAG, ARIA, etc.)
2. **Accessibility**: Human-readable JSON with binary optimization options
3. **Versioning**: Support multiple format versions simultaneously
4. **Error Resilience**: Graceful degradation and comprehensive error reporting
5. **User Sovereignty**: User controls data collection, storage, and sharing

### 1.2 Scope

This specification covers:
- Device metadata and capabilities
- Sensor data formats (camera, GPS, IMU, health sensors, etc.)
- Event schemas (navigation, emergency, accessibility events)
- Telemetry structures (battery, connectivity, processing status)
- Multi-sensor fusion formats
- Validation and error handling

---

## 2. Device Data Model

### 2.1 Core Device Object

Every WIA-compliant assistive wearable must expose a standard device object:

```json
{
  "wia_version": "1.0.0",
  "device": {
    "id": "uuid-unique-device-identifier",
    "manufacturer": "Company Name",
    "model": "Product Model XYZ",
    "category": "smart-glasses",
    "serial_number": "SN123456789",
    "firmware_version": "2.1.3",
    "hardware_revision": "Rev C"
  },
  "capabilities": {
    "sensors": [
      "camera",
      "gps",
      "imu",
      "microphone",
      "speaker"
    ],
    "processing": {
      "edge_ai": true,
      "cloud_available": true,
      "models": [
        "object-detection",
        "ocr",
        "scene-understanding"
      ]
    },
    "connectivity": [
      "bluetooth-le",
      "wifi",
      "nfc"
    ],
    "battery": {
      "removable": false,
      "typical_life_hours": 6,
      "charging_type": "usb-c"
    },
    "output_modalities": [
      "voice",
      "haptic",
      "visual"
    ]
  },
  "compliance": {
    "level": "standard",
    "certification_date": "2025-01-15",
    "certification_id": "WIA-AAC-015-STD-00123",
    "phases": [1, 2, 3, 4]
  }
}
```

**Field Descriptions**:

- **wia_version**: Semantic version of WIA standard (MAJOR.MINOR.PATCH)
- **device.id**: UUID uniquely identifying this device instance
- **device.category**: One of: `smart-glasses`, `haptic-vest`, `nav-band`, `health-watch`, `emergency-pendant`, `voice-ring`, `bone-earpiece`
- **capabilities.sensors**: Array of available sensors using standard names
- **compliance.level**: `basic`, `standard`, or `medical`

### 2.2 State Object

Real-time device state information:

```json
{
  "timestamp": "2025-01-15T14:32:18.234Z",
  "power": {
    "battery_percent": 67,
    "charging": false,
    "estimated_hours": 4.2,
    "power_mode": "balanced"
  },
  "connectivity": {
    "bluetooth": {
      "connected": true,
      "paired_device": "iPhone 15 Pro",
      "rssi": -45
    },
    "wifi": {
      "connected": true,
      "ssid": "HomeNetwork",
      "signal_strength": -38,
      "ip_address": "192.168.1.100"
    },
    "cellular": {
      "connected": false
    }
  },
  "sensors_active": ["camera", "gps", "microphone"],
  "processing_mode": "edge",
  "privacy_mode": false,
  "emergency_ready": true
}
```

---

## 3. Sensor Data Formats

### 3.1 Camera Data

```json
{
  "sensor": "camera",
  "timestamp": "2025-01-15T14:32:18.456Z",
  "data": {
    "frame_id": 12345,
    "resolution": {
      "width": 1920,
      "height": 1080
    },
    "format": "rgb",
    "fps": 30,
    "exposure": {
      "iso": 200,
      "shutter_speed": "1/60"
    },
    "focus": {
      "mode": "auto",
      "distance_meters": 2.5
    }
  },
  "ai_results": {
    "objects": [
      {
        "class": "person",
        "confidence": 0.95,
        "bounding_box": {
          "x": 120,
          "y": 80,
          "width": 200,
          "height": 400
        },
        "attributes": {
          "age_estimate": "adult",
          "pose": "standing",
          "distance_meters": 3.2
        }
      }
    ],
    "text_detected": [
      {
        "text": "EXIT",
        "confidence": 0.92,
        "language": "en",
        "bounding_box": {
          "x": 520,
          "y": 150,
          "width": 100,
          "height": 40
        }
      }
    ],
    "scene_description": "Indoor office corridor with one person walking. Exit sign visible on door ahead."
  }
}
```

### 3.2 GPS Data

```json
{
  "sensor": "gps",
  "timestamp": "2025-01-15T14:32:18.789Z",
  "data": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitude_meters": 15.3,
    "accuracy_meters": 4.5,
    "heading_degrees": 270,
    "speed_mps": 1.2,
    "satellites": 12,
    "fix_quality": "3d"
  },
  "location_context": {
    "indoor": false,
    "address": "123 Main St, San Francisco, CA 94102",
    "place_name": "Downtown Coffee Shop",
    "place_category": "cafe",
    "accessibility_features": [
      "curb_cut",
      "automatic_door",
      "accessible_restroom"
    ]
  }
}
```

### 3.3 IMU (Inertial Measurement Unit) Data

```json
{
  "sensor": "imu",
  "timestamp": "2025-01-15T14:32:18.890Z",
  "data": {
    "accelerometer": {
      "x": 0.12,
      "y": -0.05,
      "z": 9.81,
      "unit": "m/s^2"
    },
    "gyroscope": {
      "x": 0.01,
      "y": -0.02,
      "z": 0.00,
      "unit": "rad/s"
    },
    "magnetometer": {
      "x": 25.3,
      "y": -10.2,
      "z": 45.1,
      "unit": "uT"
    }
  },
  "derived": {
    "orientation": {
      "pitch": 2.3,
      "roll": -1.1,
      "yaw": 270.5
    },
    "activity": "walking",
    "step_count": 1234,
    "cadence_steps_per_minute": 115
  }
}
```

### 3.4 Health Sensors

```json
{
  "sensor": "health_monitor",
  "timestamp": "2025-01-15T14:32:19.000Z",
  "data": {
    "heart_rate": {
      "bpm": 72,
      "confidence": 0.98,
      "rhythm": "regular",
      "hrv_ms": 45
    },
    "spo2": {
      "percent": 98,
      "confidence": 0.95
    },
    "temperature": {
      "celsius": 36.7,
      "location": "wrist",
      "confidence": 0.90
    },
    "respiratory_rate": {
      "breaths_per_minute": 16,
      "method": "accelerometer"
    }
  },
  "alerts": [
    {
      "type": "heart_rate_high",
      "severity": "warning",
      "message": "Heart rate elevated for 10 minutes",
      "threshold": 90,
      "current": 95,
      "action_required": false
    }
  ]
}
```

---

## 4. Event and Telemetry Formats

### 4.1 Navigation Events

```json
{
  "event": "navigation_instruction",
  "timestamp": "2025-01-15T14:32:20.123Z",
  "data": {
    "type": "turn",
    "direction": "left",
    "distance_meters": 50,
    "instruction": "In 50 meters, turn left onto Market Street",
    "accessibility": {
      "curb_cut_available": true,
      "audible_signal": true,
      "tactile_paving": true,
      "crossing_type": "marked_crosswalk"
    },
    "hazards": [
      {
        "type": "construction",
        "description": "Sidewalk partially blocked",
        "alternative": "Use opposite sidewalk"
      }
    ]
  },
  "output_modalities": {
    "voice": true,
    "haptic": true,
    "visual": false
  }
}
```

### 4.2 Emergency Events

```json
{
  "event": "emergency_alert",
  "timestamp": "2025-01-15T14:32:21.456Z",
  "data": {
    "type": "fall_detected",
    "severity": "critical",
    "location": {
      "latitude": 37.7749,
      "longitude": -122.4194,
      "altitude": 15.3,
      "accuracy": 4.5,
      "address": "123 Main St, San Francisco, CA 94102"
    },
    "vitals": {
      "heart_rate": 110,
      "responsive": false,
      "motion_detected": false
    },
    "user_profile": {
      "name": "John Doe",
      "age": 75,
      "medical_conditions": ["diabetes", "hypertension"],
      "medications": ["metformin", "lisinopril"],
      "allergies": ["penicillin"],
      "emergency_contacts": [
        {
          "name": "Jane Doe",
          "relationship": "daughter",
          "phone": "+1-555-0123"
        }
      ]
    },
    "actions_taken": {
      "alert_sent": true,
      "call_initiated": true,
      "contacts_notified": true,
      "location_shared": true
    }
  }
}
```

---

## 5. Multi-Sensor Fusion

### 5.1 Obstacle Detection

```json
{
  "fusion": "obstacle_detection",
  "timestamp": "2025-01-15T14:32:23.012Z",
  "inputs": {
    "camera": {
      "objects": ["pole", "bench"],
      "depth_map_available": true
    },
    "ultrasonic": {
      "distance_cm": 78,
      "angle": 15
    },
    "gps": {
      "latitude": 37.7749,
      "longitude": -122.4194
    },
    "imu": {
      "heading": 270,
      "walking_speed": 1.2
    }
  },
  "result": {
    "obstacle": {
      "type": "pole",
      "distance_meters": 0.8,
      "direction": "ahead_right",
      "height_meters": 2.5,
      "width_meters": 0.3
    },
    "recommendation": {
      "action": "adjust_path",
      "direction": "left",
      "urgency": "low",
      "time_to_collision_seconds": 0.7
    },
    "output": {
      "voice": "Pole ahead on the right, 0.8 meters",
      "haptic": {
        "pattern": "increasing_pulse",
        "location": "right_shoulder",
        "intensity": 40
      }
    }
  }
}
```

---

## 6. JSON Schema Definitions

All WIA data formats are validated against formal JSON schemas:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA Device Object",
  "type": "object",
  "required": ["wia_version", "device", "capabilities", "compliance"],
  "properties": {
    "wia_version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "device": {
      "type": "object",
      "required": ["id", "manufacturer", "model", "category"],
      "properties": {
        "id": {
          "type": "string",
          "format": "uuid"
        },
        "category": {
          "enum": [
            "smart-glasses",
            "haptic-vest",
            "nav-band",
            "health-watch",
            "emergency-pendant",
            "voice-ring",
            "bone-earpiece"
          ]
        }
      }
    }
  }
}
```

---

## 7. Validation Rules

- **Timestamps**: ISO 8601 format with timezone (UTC recommended)
- **Numeric Ranges**: All values within physically plausible ranges
- **Required Fields**: Core fields must always be present
- **Type Safety**: Strict type checking per JSON schema
- **Encoding**: UTF-8 for text, Base64 for binary data

---

## 8. Example Payloads

See chapter 4 of the official WIA Assistive Wearable ebook for comprehensive examples.

---

**弘益人間 (Hongik Ingan)** - Benefit All Humanity

© 2025 WIA - MIT License
