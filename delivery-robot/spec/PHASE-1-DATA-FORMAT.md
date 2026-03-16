# WIA ROB-010 Delivery Robot Standard - Phase 1: Data Format Specification

> **Version**: 1.0.0  
> **Status**: Stable  
> **Last Updated**: 2025-12-26

---

## 1. Overview

Phase 1 defines the standardized JSON schema for representing delivery robot data. This specification ensures interoperability across platforms, vendors, and applications by establishing a common language for robot state, package information, route data, and sensor telemetry exchange.

### 1.1 Design Principles

- **Interoperability**: Universal format compatible with any delivery robot system  
- **Extensibility**: Support for additional robot classes and capabilities  
- **Precision**: Fine-grained state representation with timestamps and confidence values  
- **Completeness**: Support for navigation, packages, telemetry, and fleet operations  

---

## 2. Core Schema

### 2.1 RobotState Root Schema

```json
{
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "$id": "https://wiastandards.com/schemas/delivery-robot/v1/robot-state.json",
    "title": "WIA Delivery Robot State",
    "description": "Standardized robot state data format",
    "type": "object",
    "required": ["robot_id", "timestamp", "version", "position", "battery_level"],
    "properties": {
        "robot_id": {
            "type": "string",
            "format": "uuid",
            "description": "Unique identifier for the robot (UUID v4)"
        },
        "timestamp": {
            "type": "string",
            "format": "date-time",
            "description": "ISO 8601 timestamp of state capture"
        },
        "version": {
            "type": "string",
            "pattern": "^\\d+\\.\\d+\\.\\d+$",
            "description": "Schema version (SemVer format)"
        },
        "position": {
            "$ref": "#/$defs/Position"
        },
        "heading": {
            "type": "number",
            "minimum": 0,
            "maximum": 360,
            "description": "Heading in degrees (0 = North, 90 = East)"
        },
        "speed": {
            "type": "number",
            "minimum": 0,
            "description": "Current speed in meters per second"
        },
        "battery_level": {
            "type": "number",
            "minimum": 0,
            "maximum": 100,
            "description": "Battery charge percentage"
        },
        "mode": {
            "type": "string",
            "enum": ["autonomous", "manual", "idle", "charging", "error"],
            "description": "Current operational mode"
        },
        "cargo": {
            "$ref": "#/$defs/CargoStatus"
        },
        "route": {
            "$ref": "#/$defs/ActiveRoute"
        },
        "sensors": {
            "$ref": "#/$defs/SensorReadings"
        },
        "errors": {
            "type": "array",
            "items": {"$ref": "#/$defs/Error"}
        }
    }
}
```

### 2.2 Position Schema

```json
{
    "$defs": {
        "Position": {
            "type": "object",
            "required": ["latitude", "longitude"],
            "properties": {
                "latitude": {
                    "type": "number",
                    "minimum": -90,
                    "maximum": 90
                },
                "longitude": {
                    "type": "number",
                    "minimum": -180,
                    "maximum": 180
                },
                "altitude": {
                    "type": "number",
                    "description": "Altitude in meters above sea level"
                },
                "accuracy": {
                    "type": "number",
                    "minimum": 0,
                    "description": "Position accuracy in meters"
                },
                "source": {
                    "type": "string",
                    "enum": ["gps", "rtk-gps", "slam", "visual-odometry", "fused"],
                    "description": "Primary localization source"
                }
            }
        }
    }
}
```

---

## 3. Package Data Model

### 3.1 Package Schema

```json
{
    "$id": "https://wiastandards.com/schemas/delivery-robot/v1/package.json",
    "title": "Delivery Package",
    "type": "object",
    "required": ["package_id", "recipient", "pickup_location", "dropoff_location"],
    "properties": {
        "package_id": {
            "type": "string",
            "format": "uuid"
        },
        "tracking_number": {
            "type": "string",
            "description": "External tracking identifier"
        },
        "sender": {
            "$ref": "#/$defs/Contact"
        },
        "recipient": {
            "$ref": "#/$defs/Contact"
        },
        "pickup_location": {
            "$ref": "#/$defs/Location"
        },
        "dropoff_location": {
            "$ref": "#/$defs/Location"
        },
        "package_details": {
            "type": "object",
            "properties": {
                "weight_kg": {"type": "number", "minimum": 0},
                "dimensions_cm": {
                    "type": "object",
                    "properties": {
                        "length": {"type": "number", "minimum": 0},
                        "width": {"type": "number", "minimum": 0},
                        "height": {"type": "number", "minimum": 0}
                    }
                },
                "category": {
                    "type": "string",
                    "enum": ["food", "retail", "medical", "document", "other"]
                },
                "fragile": {"type": "boolean"},
                "temperature_controlled": {"type": "boolean"},
                "temperature_range_celsius": {
                    "type": "object",
                    "properties": {
                        "min": {"type": "number"},
                        "max": {"type": "number"}
                    }
                }
            }
        },
        "access_control": {
            "type": "object",
            "properties": {
                "access_code": {"type": "string", "pattern": "^[0-9]{4,8}$"},
                "qr_code": {"type": "string"},
                "biometric_required": {"type": "boolean"},
                "signature_required": {"type": "boolean"}
            }
        },
        "status": {
            "type": "string",
            "enum": ["pending", "assigned", "picked_up", "in_transit", "arrived", "delivered", "failed", "returned"]
        },
        "priority": {
            "type": "string",
            "enum": ["normal", "express", "urgent"],
            "default": "normal"
        },
        "estimated_delivery": {
            "type": "string",
            "format": "date-time"
        },
        "delivered_at": {
            "type": "string",
            "format": "date-time"
        }
    }
}
```

---

## 4. Route Data Model

### 4.1 Route Schema

```json
{
    "$id": "https://wiastandards.com/schemas/delivery-robot/v1/route.json",
    "title": "Navigation Route",
    "type": "object",
    "required": ["route_id", "waypoints"],
    "properties": {
        "route_id": {
            "type": "string",
            "format": "uuid"
        },
        "waypoints": {
            "type": "array",
            "minItems": 2,
            "items": {
                "type": "object",
                "required": ["position", "waypoint_type"],
                "properties": {
                    "position": {"$ref": "#/$defs/Position"},
                    "waypoint_type": {
                        "type": "string",
                        "enum": ["start", "intermediate", "destination", "charging_station"]
                    },
                    "arrival_time": {"type": "string", "format": "date-time"},
                    "dwell_time_seconds": {"type": "number", "minimum": 0}
                }
            }
        },
        "total_distance_meters": {
            "type": "number",
            "minimum": 0
        },
        "estimated_duration_seconds": {
            "type": "number",
            "minimum": 0
        },
        "route_preference": {
            "type": "string",
            "enum": ["fastest", "shortest", "safest", "energy_efficient"]
        },
        "navigation_constraints": {
            "type": "object",
            "properties": {
                "max_speed_ms": {"type": "number"},
                "avoid_roads": {"type": "boolean"},
                "sidewalk_only": {"type": "boolean"},
                "elevator_allowed": {"type": "boolean"}
            }
        }
    }
}
```

---

## 5. Sensor Data Schema

### 5.1 Sensor Readings

```json
{
    "$defs": {
        "SensorReadings": {
            "type": "object",
            "properties": {
                "lidar": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "sensor_id": {"type": "string"},
                            "timestamp": {"type": "string", "format": "date-time"},
                            "scan_rate_hz": {"type": "number"},
                            "points": {
                                "type": "array",
                                "items": {
                                    "type": "object",
                                    "properties": {
                                        "x": {"type": "number"},
                                        "y": {"type": "number"},
                                        "z": {"type": "number"},
                                        "intensity": {"type": "number"}
                                    }
                                }
                            }
                        }
                    }
                },
                "cameras": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "sensor_id": {"type": "string"},
                            "timestamp": {"type": "string", "format": "date-time"},
                            "image_url": {"type": "string", "format": "uri"},
                            "resolution": {
                                "type": "object",
                                "properties": {
                                    "width": {"type": "integer"},
                                    "height": {"type": "integer"}
                                }
                            },
                            "detections": {
                                "type": "array",
                                "items": {
                                    "type": "object",
                                    "properties": {
                                        "class": {"type": "string"},
                                        "confidence": {"type": "number", "minimum": 0, "maximum": 1},
                                        "bounding_box": {
                                            "type": "object",
                                            "properties": {
                                                "x": {"type": "integer"},
                                                "y": {"type": "integer"},
                                                "width": {"type": "integer"},
                                                "height": {"type": "integer"}
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                },
                "ultrasonic": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "sensor_id": {"type": "string"},
                            "distance_cm": {"type": "number", "minimum": 0}
                        }
                    }
                },
                "imu": {
                    "type": "object",
                    "properties": {
                        "acceleration": {
                            "type": "object",
                            "properties": {
                                "x": {"type": "number"},
                                "y": {"type": "number"},
                                "z": {"type": "number"}
                            }
                        },
                        "gyroscope": {
                            "type": "object",
                            "properties": {
                                "roll": {"type": "number"},
                                "pitch": {"type": "number"},
                                "yaw": {"type": "number"}
                            }
                        }
                    }
                }
            }
        }
    }
}
```

---

## 6. Best Practices

### 6.1 Data Validation

- Use JSON Schema validation on all inputs and outputs  
- Implement strict type checking  
- Validate ranges and enums  
- Log validation errors for debugging  

### 6.2 Versioning

- Follow semantic versioning (major.minor.patch)  
- Maintain backward compatibility for minor/patch updates  
- Include version in all message payloads  
- Document breaking changes in major versions  

### 6.3 Performance

- Minimize payload size (compress if necessary)  
- Use efficient data encoding (Protocol Buffers for high-frequency telemetry)  
- Implement delta updates for frequently changing data  
- Cache static information  

---

**Copyright 2025 WIA / SmileStory Inc.**  
**License**: MIT
