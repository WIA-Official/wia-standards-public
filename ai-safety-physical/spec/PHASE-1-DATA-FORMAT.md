# WIA AI Safety Physical - Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01
**Primary Color:** #EF4444 (Red - Safety)

## 1. Overview

This specification defines standard data formats for AI systems operating in physical environments. It establishes JSON schemas for safety zones, force/velocity limits, collision detection, emergency states, and real-time safety monitoring in accordance with ISO 10218 (Robotics Safety) and ISO 13849 (Safety of Machinery).

### 1.1 Safety Integrity Levels (SIL)

| SIL Level | PFH (Probability of Failure per Hour) | Application |
|-----------|--------------------------------------|-------------|
| SIL 1 | 10⁻⁵ to 10⁻⁶ | Low risk environments |
| SIL 2 | 10⁻⁶ to 10⁻⁷ | Moderate risk, human proximity |
| SIL 3 | 10⁻⁷ to 10⁻⁸ | High risk, collaborative robotics |
| SIL 4 | 10⁻⁸ to 10⁻⁹ | Critical safety applications |

### 1.2 Coordinate System

All spatial data uses the right-handed Cartesian coordinate system (ISO 9787):
- **X-axis**: Forward (positive) / Backward (negative)
- **Y-axis**: Left (positive) / Right (negative)
- **Z-axis**: Up (positive) / Down (negative)
- **Units**: Meters (m) for position, meters/second (m/s) for velocity

---

## 2. Safety Zone Definition

### 2.1 SafetyZone Schema

Safety zones define spatial boundaries where different safety constraints apply.

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "required": ["id", "type", "geometry", "safetyLevel", "constraints"],
  "properties": {
    "id": {
      "type": "string",
      "description": "Unique zone identifier",
      "pattern": "^zone-[a-z0-9-]+$"
    },
    "type": {
      "type": "string",
      "enum": ["protective", "warning", "collaborative", "restricted", "prohibited"],
      "description": "Zone classification per ISO 10218-2"
    },
    "geometry": {
      "type": "object",
      "oneOf": [
        { "$ref": "#/definitions/Box" },
        { "$ref": "#/definitions/Sphere" },
        { "$ref": "#/definitions/Cylinder" },
        { "$ref": "#/definitions/Polygon" }
      ]
    },
    "safetyLevel": {
      "type": "string",
      "enum": ["SIL1", "SIL2", "SIL3", "SIL4"]
    },
    "constraints": {
      "$ref": "#/definitions/ZoneConstraints"
    },
    "humanPresence": {
      "type": "boolean",
      "description": "Human presence detected in zone"
    },
    "monitoring": {
      "$ref": "#/definitions/MonitoringConfig"
    }
  }
}
```

### 2.2 Geometry Definitions

#### Box Geometry
```json
{
  "Box": {
    "type": "object",
    "required": ["type", "center", "dimensions", "orientation"],
    "properties": {
      "type": { "const": "box" },
      "center": { "$ref": "#/definitions/Point3D" },
      "dimensions": {
        "type": "object",
        "required": ["length", "width", "height"],
        "properties": {
          "length": { "type": "number", "minimum": 0 },
          "width": { "type": "number", "minimum": 0 },
          "height": { "type": "number", "minimum": 0 }
        }
      },
      "orientation": { "$ref": "#/definitions/Quaternion" }
    }
  }
}
```

#### Sphere Geometry
```json
{
  "Sphere": {
    "type": "object",
    "required": ["type", "center", "radius"],
    "properties": {
      "type": { "const": "sphere" },
      "center": { "$ref": "#/definitions/Point3D" },
      "radius": { "type": "number", "minimum": 0, "maximum": 100 }
    }
  }
}
```

#### Cylinder Geometry
```json
{
  "Cylinder": {
    "type": "object",
    "required": ["type", "center", "radius", "height", "axis"],
    "properties": {
      "type": { "const": "cylinder" },
      "center": { "$ref": "#/definitions/Point3D" },
      "radius": { "type": "number", "minimum": 0 },
      "height": { "type": "number", "minimum": 0 },
      "axis": { "$ref": "#/definitions/Vector3D" }
    }
  }
}
```

### 2.3 Zone Constraints

```json
{
  "ZoneConstraints": {
    "type": "object",
    "properties": {
      "maxVelocity": {
        "type": "object",
        "properties": {
          "linear": { "type": "number", "minimum": 0, "maximum": 10, "unit": "m/s" },
          "angular": { "type": "number", "minimum": 0, "maximum": 6.28, "unit": "rad/s" }
        }
      },
      "maxForce": { "type": "number", "minimum": 0, "maximum": 1000, "unit": "N" },
      "maxPower": { "type": "number", "minimum": 0, "unit": "W" },
      "maxPressure": { "type": "number", "minimum": 0, "maximum": 150, "unit": "N/cm²" },
      "requiresHumanApproval": { "type": "boolean" },
      "emergencyStopRequired": { "type": "boolean" },
      "minSeparationDistance": { "type": "number", "minimum": 0, "unit": "m" }
    }
  }
}
```

### 2.4 Complete Example

```json
{
  "id": "zone-collaborative-assembly",
  "type": "collaborative",
  "geometry": {
    "type": "box",
    "center": { "x": 2.0, "y": 0.0, "z": 1.0 },
    "dimensions": { "length": 4.0, "width": 3.0, "height": 2.5 },
    "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 }
  },
  "safetyLevel": "SIL3",
  "constraints": {
    "maxVelocity": { "linear": 0.25, "angular": 0.52 },
    "maxForce": 150,
    "maxPower": 80,
    "maxPressure": 25,
    "requiresHumanApproval": false,
    "emergencyStopRequired": true,
    "minSeparationDistance": 0.5
  },
  "humanPresence": true,
  "monitoring": {
    "frequency": 100,
    "sensors": ["lidar", "camera", "force_torque"],
    "alertThreshold": 0.8
  }
}
```

---

## 3. Force and Velocity Limits

### 3.1 LimitProfile Schema

```json
{
  "type": "object",
  "required": ["id", "name", "limits", "validFrom"],
  "properties": {
    "id": { "type": "string" },
    "name": { "type": "string" },
    "description": { "type": "string" },
    "limits": {
      "type": "object",
      "properties": {
        "velocity": { "$ref": "#/definitions/VelocityLimits" },
        "force": { "$ref": "#/definitions/ForceLimits" },
        "acceleration": { "$ref": "#/definitions/AccelerationLimits" }
      }
    },
    "validFrom": { "type": "string", "format": "date-time" },
    "validUntil": { "type": "string", "format": "date-time" },
    "compliance": { "type": "array", "items": { "type": "string" } }
  }
}
```

### 3.2 Velocity Limits (ISO 10218-1)

```json
{
  "VelocityLimits": {
    "type": "object",
    "properties": {
      "linear": {
        "type": "object",
        "properties": {
          "x": { "type": "number", "minimum": 0, "maximum": 10 },
          "y": { "type": "number", "minimum": 0, "maximum": 10 },
          "z": { "type": "number", "minimum": 0, "maximum": 10 },
          "magnitude": { "type": "number", "minimum": 0, "maximum": 10 }
        }
      },
      "angular": {
        "type": "object",
        "properties": {
          "roll": { "type": "number", "minimum": 0, "maximum": 6.28 },
          "pitch": { "type": "number", "minimum": 0, "maximum": 6.28 },
          "yaw": { "type": "number", "minimum": 0, "maximum": 6.28 }
        }
      },
      "contextual": {
        "type": "object",
        "properties": {
          "noHuman": { "type": "number", "description": "Max velocity with no human present" },
          "humanDetected": { "type": "number", "description": "Reduced velocity when human detected" },
          "collaborative": { "type": "number", "description": "Velocity during human collaboration" },
          "emergency": { "type": "number", "description": "Emergency deceleration rate" }
        }
      }
    }
  }
}
```

### 3.3 Force Limits (ISO/TS 15066)

```json
{
  "ForceLimits": {
    "type": "object",
    "properties": {
      "transient": {
        "type": "object",
        "description": "Transient contact limits per body region",
        "properties": {
          "skull": { "type": "number", "maximum": 130, "unit": "N" },
          "forehead": { "type": "number", "maximum": 130, "unit": "N" },
          "face": { "type": "number", "maximum": 65, "unit": "N" },
          "neck": { "type": "number", "maximum": 150, "unit": "N" },
          "back": { "type": "number", "maximum": 210, "unit": "N" },
          "chest": { "type": "number", "maximum": 140, "unit": "N" },
          "abdomen": { "type": "number", "maximum": 110, "unit": "N" },
          "pelvis": { "type": "number", "maximum": 180, "unit": "N" },
          "upperArm": { "type": "number", "maximum": 150, "unit": "N" },
          "lowerArm": { "type": "number", "maximum": 160, "unit": "N" },
          "hand": { "type": "number", "maximum": 140, "unit": "N" },
          "thigh": { "type": "number", "maximum": 220, "unit": "N" },
          "lowerLeg": { "type": "number", "maximum": 220, "unit": "N" },
          "foot": { "type": "number", "maximum": 220, "unit": "N" }
        }
      },
      "quasiStatic": {
        "type": "object",
        "description": "Quasi-static contact limits (clamping)",
        "properties": {
          "chest": { "type": "number", "maximum": 140, "unit": "N" },
          "abdomen": { "type": "number", "maximum": 110, "unit": "N" },
          "hand": { "type": "number", "maximum": 95, "unit": "N" }
        }
      },
      "gripForce": { "type": "number", "maximum": 500, "unit": "N" },
      "actuatorTorque": {
        "type": "array",
        "items": {
          "type": "object",
          "properties": {
            "jointId": { "type": "string" },
            "maxTorque": { "type": "number", "unit": "Nm" },
            "maxCurrent": { "type": "number", "unit": "A" }
          }
        }
      }
    }
  }
}
```

---

## 4. Collision Detection

### 4.1 CollisionEvent Schema

```json
{
  "type": "object",
  "required": ["id", "timestamp", "severity", "location", "response"],
  "properties": {
    "id": { "type": "string", "pattern": "^collision-[0-9a-f-]+$" },
    "timestamp": { "type": "integer", "description": "Unix timestamp in milliseconds" },
    "severity": {
      "type": "string",
      "enum": ["info", "warning", "critical", "emergency"]
    },
    "detectionMethod": {
      "type": "string",
      "enum": ["force_torque", "vision", "proximity", "tactile", "model_based"]
    },
    "location": {
      "type": "object",
      "properties": {
        "point": { "$ref": "#/definitions/Point3D" },
        "bodyPart": { "type": "string" },
        "surfaceNormal": { "$ref": "#/definitions/Vector3D" }
      }
    },
    "impact": {
      "type": "object",
      "properties": {
        "force": { "type": "number", "unit": "N" },
        "pressure": { "type": "number", "unit": "N/cm²" },
        "velocity": { "type": "number", "unit": "m/s" },
        "energy": { "type": "number", "unit": "J" }
      }
    },
    "response": { "$ref": "#/definitions/CollisionResponse" },
    "prediction": {
      "type": "object",
      "properties": {
        "timeToCollision": { "type": "number", "unit": "s" },
        "probability": { "type": "number", "minimum": 0, "maximum": 1 },
        "trajectory": { "type": "array", "items": { "$ref": "#/definitions/Point3D" } }
      }
    }
  }
}
```

### 4.2 Collision Response

```json
{
  "CollisionResponse": {
    "type": "object",
    "required": ["action", "executedAt"],
    "properties": {
      "action": {
        "type": "string",
        "enum": [
          "emergency_stop",
          "protective_stop",
          "speed_reduction",
          "retreat",
          "compliance_mode",
          "alert_only"
        ]
      },
      "executedAt": { "type": "integer", "description": "Response timestamp in ms" },
      "responseTime": { "type": "number", "description": "Detection to response time in ms" },
      "stoppingDistance": { "type": "number", "unit": "m" },
      "recoveryPlan": {
        "type": "object",
        "properties": {
          "automatic": { "type": "boolean" },
          "manualApprovalRequired": { "type": "boolean" },
          "steps": { "type": "array", "items": { "type": "string" } }
        }
      }
    }
  }
}
```

### 4.3 Example Collision Event

```json
{
  "id": "collision-7f8e9d2c-1a3b-4c5d-8e9f-0a1b2c3d4e5f",
  "timestamp": 1704067200000,
  "severity": "critical",
  "detectionMethod": "force_torque",
  "location": {
    "point": { "x": 1.2, "y": 0.5, "z": 0.8 },
    "bodyPart": "upperArm",
    "surfaceNormal": { "x": 0.0, "y": 1.0, "z": 0.0 }
  },
  "impact": {
    "force": 142.5,
    "pressure": 18.3,
    "velocity": 0.18,
    "energy": 2.3
  },
  "response": {
    "action": "protective_stop",
    "executedAt": 1704067200045,
    "responseTime": 45,
    "stoppingDistance": 0.012,
    "recoveryPlan": {
      "automatic": false,
      "manualApprovalRequired": true,
      "steps": [
        "verify_human_safety",
        "assess_damage",
        "reset_safety_state",
        "resume_operation"
      ]
    }
  },
  "prediction": {
    "timeToCollision": 0.25,
    "probability": 0.92,
    "trajectory": [
      { "x": 1.0, "y": 0.4, "z": 0.8 },
      { "x": 1.1, "y": 0.45, "z": 0.8 },
      { "x": 1.2, "y": 0.5, "z": 0.8 }
    ]
  }
}
```

---

## 5. Emergency States

### 5.1 EmergencyState Schema

```json
{
  "type": "object",
  "required": ["state", "timestamp", "trigger", "category"],
  "properties": {
    "state": {
      "type": "string",
      "enum": [
        "normal",
        "warning",
        "protective_stop",
        "emergency_stop_category_0",
        "emergency_stop_category_1",
        "emergency_stop_category_2",
        "safe_state",
        "recovery"
      ]
    },
    "timestamp": { "type": "integer" },
    "trigger": {
      "type": "object",
      "properties": {
        "source": {
          "type": "string",
          "enum": ["manual", "automatic", "sensor", "system", "external"]
        },
        "reason": { "type": "string" },
        "triggerId": { "type": "string" }
      }
    },
    "category": {
      "type": "string",
      "enum": ["cat_0", "cat_1", "cat_2"],
      "description": "ISO 13850 emergency stop categories"
    },
    "systemStatus": {
      "type": "object",
      "properties": {
        "motorsEnabled": { "type": "boolean" },
        "powerAvailable": { "type": "boolean" },
        "brakesEngaged": { "type": "boolean" },
        "sensorsActive": { "type": "boolean" },
        "communicationActive": { "type": "boolean" }
      }
    },
    "recovery": {
      "type": "object",
      "properties": {
        "automatic": { "type": "boolean" },
        "requiresReset": { "type": "boolean" },
        "authorizationRequired": { "type": "boolean" },
        "estimatedTime": { "type": "number", "unit": "s" }
      }
    }
  }
}
```

### 5.2 Emergency Stop Categories (ISO 13850)

| Category | Description | Power Cut | Controlled Stop |
|----------|-------------|-----------|-----------------|
| Cat 0 | Uncontrolled stop | Immediate | No |
| Cat 1 | Controlled stop, then power cut | After stop | Yes |
| Cat 2 | Controlled stop, power maintained | No | Yes |

### 5.3 State Transition Example

```json
{
  "state": "emergency_stop_category_1",
  "timestamp": 1704067200123,
  "trigger": {
    "source": "manual",
    "reason": "E-Stop button pressed by operator",
    "triggerId": "estop-button-01"
  },
  "category": "cat_1",
  "systemStatus": {
    "motorsEnabled": false,
    "powerAvailable": true,
    "brakesEngaged": true,
    "sensorsActive": true,
    "communicationActive": true
  },
  "recovery": {
    "automatic": false,
    "requiresReset": true,
    "authorizationRequired": true,
    "estimatedTime": 30
  }
}
```

---

## 6. Safety Monitoring Data

### 6.1 SafetyMetrics Schema

```json
{
  "type": "object",
  "required": ["timestamp", "metrics", "status"],
  "properties": {
    "timestamp": { "type": "integer" },
    "metrics": {
      "type": "object",
      "properties": {
        "currentVelocity": {
          "type": "object",
          "properties": {
            "linear": { "$ref": "#/definitions/Vector3D" },
            "angular": { "$ref": "#/definitions/Vector3D" }
          }
        },
        "currentForce": {
          "type": "object",
          "properties": {
            "applied": { "$ref": "#/definitions/Vector3D" },
            "magnitude": { "type": "number" }
          }
        },
        "separationDistance": {
          "type": "object",
          "properties": {
            "minimum": { "type": "number", "unit": "m" },
            "location": { "$ref": "#/definitions/Point3D" },
            "human": { "type": "boolean" }
          }
        },
        "performanceLimits": {
          "type": "object",
          "properties": {
            "velocityUtilization": { "type": "number", "minimum": 0, "maximum": 1 },
            "forceUtilization": { "type": "number", "minimum": 0, "maximum": 1 },
            "powerUtilization": { "type": "number", "minimum": 0, "maximum": 1 }
          }
        }
      }
    },
    "status": {
      "type": "string",
      "enum": ["safe", "warning", "alarm", "fault"]
    },
    "violations": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "type": { "type": "string" },
          "severity": { "type": "string" },
          "message": { "type": "string" },
          "value": { "type": "number" },
          "limit": { "type": "number" }
        }
      }
    }
  }
}
```

---

## 7. Risk Assessment Data

### 7.1 RiskAssessment Schema

```json
{
  "type": "object",
  "required": ["taskId", "timestamp", "riskLevel", "factors"],
  "properties": {
    "taskId": { "type": "string" },
    "timestamp": { "type": "integer" },
    "riskLevel": {
      "type": "string",
      "enum": ["negligible", "low", "medium", "high", "very_high"]
    },
    "riskScore": { "type": "number", "minimum": 0, "maximum": 100 },
    "factors": {
      "type": "object",
      "properties": {
        "severity": { "type": "integer", "minimum": 1, "maximum": 4 },
        "frequency": { "type": "integer", "minimum": 1, "maximum": 5 },
        "probability": { "type": "integer", "minimum": 1, "maximum": 5 },
        "avoidability": { "type": "integer", "minimum": 1, "maximum": 3 }
      }
    },
    "mitigations": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "measure": { "type": "string" },
          "effectiveness": { "type": "number", "minimum": 0, "maximum": 1 },
          "implemented": { "type": "boolean" }
        }
      }
    }
  }
}
```

---

## 8. Common Type Definitions

### 8.1 Geometric Types

```json
{
  "Point3D": {
    "type": "object",
    "required": ["x", "y", "z"],
    "properties": {
      "x": { "type": "number" },
      "y": { "type": "number" },
      "z": { "type": "number" }
    }
  },
  "Vector3D": {
    "type": "object",
    "required": ["x", "y", "z"],
    "properties": {
      "x": { "type": "number" },
      "y": { "type": "number" },
      "z": { "type": "number" }
    }
  },
  "Quaternion": {
    "type": "object",
    "required": ["x", "y", "z", "w"],
    "properties": {
      "x": { "type": "number" },
      "y": { "type": "number" },
      "z": { "type": "number" },
      "w": { "type": "number" }
    }
  }
}
```

---

## 9. Validation Rules

1. All timestamps must be Unix time in milliseconds
2. Force values must not exceed ISO/TS 15066 body region limits
3. Velocity limits must align with ISO 10218-1 collaborative speed requirements
4. Emergency stop response time must be < 100ms for SIL 3/4 systems
5. Safety zone geometry must not overlap with prohibited zones
6. Collision detection frequency must be ≥ 100Hz for collaborative operations
7. All safety-critical data must include checksums or digital signatures
8. State transitions must follow ISO 10218-1 state machine

---

## 10. Compliance Matrix

| Standard | Requirement | Data Format Element |
|----------|-------------|---------------------|
| ISO 10218-1 | Collaborative operation limits | VelocityLimits, ForceLimits |
| ISO 10218-2 | Safety zones | SafetyZone schema |
| ISO/TS 15066 | Power and force limiting | ForceLimits.transient/quasiStatic |
| ISO 13849-1 | Safety integrity | SIL levels, emergency states |
| ISO 13850 | Emergency stop | EmergencyState categories |
| IEC 61508 | Functional safety | Safety monitoring metrics |

---

## 11. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-01 | WIA | Initial specification |

---

<div align="center" style="margin-top: 2em; padding-top: 1em; border-top: 2px solid #EF4444;">

**弘益人間**
*Broadly benefiting humanity through safe AI-physical interaction*

</div>
