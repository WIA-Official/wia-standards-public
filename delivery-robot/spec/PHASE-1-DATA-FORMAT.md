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


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.
