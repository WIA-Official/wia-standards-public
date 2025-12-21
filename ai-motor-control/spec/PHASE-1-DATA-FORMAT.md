# WIA AI Motor Control - Phase 1: Data Format Specification

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Primary Color**: #F59E0B (Amber)

---

## 1. Overview

### 1.1 Purpose

This specification defines standard data formats for AI-controlled motor systems in robotics, including motor commands, feedback, trajectories, control parameters, and safety constraints. The standard enables interoperability between AI control systems and motor hardware across different manufacturers and platforms.

### 1.2 Scope

- Motor command data structures
- Real-time feedback formats
- Trajectory planning data
- Control mode definitions
- PID and impedance parameters
- Multi-axis coordination
- Safety limits and constraints
- Motor health monitoring

### 1.3 Design Principles

| Principle | Description |
|-----------|-------------|
| **Real-time Ready** | Sub-millisecond parsing and serialization |
| **Safety First** | Built-in constraint validation |
| **Multi-axis Support** | Coordinated control of multiple motors |
| **Extensible** | Support for new control modes and parameters |
| **Hardware Agnostic** | Works with any motor controller |

---

## 2. Control Modes

### 2.1 Control Mode Enumeration

```typescript
type ControlMode =
  | 'position'      // Position control (degrees/radians)
  | 'velocity'      // Velocity control (deg/s, rad/s)
  | 'torque'        // Torque/force control (Nm, N)
  | 'impedance'     // Impedance control
  | 'admittance'    // Admittance control
  | 'trajectory'    // Trajectory following
  | 'gravity_comp'  // Gravity compensation
  | 'force_ctrl'    // Force control
  | 'hybrid'        // Hybrid position/force
  | 'disabled';     // Motor disabled
```

### 2.2 Control Mode Configuration

```json
{
  "mode": "position",
  "parameters": {
    "kp": 50.0,
    "ki": 0.1,
    "kd": 2.5,
    "feedforward": 0.0,
    "deadband": 0.01
  },
  "limits": {
    "max_error": 5.0,
    "max_velocity": 180.0,
    "max_acceleration": 360.0,
    "max_jerk": 1000.0
  }
}
```

---

## 3. Motor Command Schema

### 3.1 Single Motor Command

Complete specification for commanding a single motor or actuator.

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "MotorCommand",
  "type": "object",
  "required": ["motor_id", "timestamp", "control_mode", "setpoint"],
  "properties": {
    "motor_id": {
      "type": "string",
      "description": "Unique motor identifier",
      "pattern": "^[a-zA-Z0-9_-]+$"
    },
    "timestamp": {
      "type": "number",
      "description": "Command timestamp (Unix milliseconds)",
      "minimum": 0
    },
    "control_mode": {
      "type": "string",
      "enum": ["position", "velocity", "torque", "impedance", "admittance", "trajectory", "gravity_comp", "force_ctrl", "hybrid", "disabled"]
    },
    "setpoint": {
      "type": "number",
      "description": "Target value in mode-specific units"
    },
    "velocity_limit": {
      "type": "number",
      "description": "Maximum velocity (deg/s or rad/s)",
      "minimum": 0
    },
    "acceleration_limit": {
      "type": "number",
      "description": "Maximum acceleration",
      "minimum": 0
    },
    "torque_limit": {
      "type": "number",
      "description": "Maximum torque (Nm)",
      "minimum": 0
    },
    "pid_gains": {
      "type": "object",
      "properties": {
        "kp": {"type": "number", "minimum": 0},
        "ki": {"type": "number", "minimum": 0},
        "kd": {"type": "number", "minimum": 0},
        "ki_limit": {"type": "number"},
        "feedforward": {"type": "number"}
      }
    },
    "impedance_params": {
      "type": "object",
      "properties": {
        "stiffness": {"type": "number", "minimum": 0},
        "damping": {"type": "number", "minimum": 0},
        "inertia": {"type": "number", "minimum": 0}
      }
    },
    "filter_config": {
      "type": "object",
      "properties": {
        "type": {"enum": ["none", "lowpass", "notch", "butterworth"]},
        "cutoff_frequency": {"type": "number"},
        "order": {"type": "integer", "minimum": 1}
      }
    },
    "safety_limits": {
      "type": "object",
      "properties": {
        "min_position": {"type": "number"},
        "max_position": {"type": "number"},
        "max_velocity": {"type": "number"},
        "max_torque": {"type": "number"},
        "emergency_stop": {"type": "boolean"}
      }
    },
    "priority": {
      "type": "integer",
      "minimum": 0,
      "maximum": 100,
      "description": "Command priority (0=lowest, 100=highest)"
    },
    "metadata": {
      "type": "object",
      "description": "Additional metadata"
    }
  }
}
```

### 3.2 Multi-Axis Command

Coordinated control of multiple motors for synchronized motion.

```json
{
  "command_id": "cmd_1234567890",
  "timestamp": 1704067200000,
  "sync_mode": "simultaneous",
  "motors": [
    {
      "motor_id": "shoulder_pitch",
      "control_mode": "position",
      "setpoint": 45.0,
      "velocity_limit": 60.0,
      "pid_gains": {"kp": 50, "ki": 0.1, "kd": 2.5}
    },
    {
      "motor_id": "shoulder_roll",
      "control_mode": "position",
      "setpoint": 15.0,
      "velocity_limit": 60.0,
      "pid_gains": {"kp": 45, "ki": 0.1, "kd": 2.0}
    },
    {
      "motor_id": "elbow",
      "control_mode": "position",
      "setpoint": 90.0,
      "velocity_limit": 90.0,
      "pid_gains": {"kp": 40, "ki": 0.05, "kd": 1.8}
    }
  ],
  "coordination": {
    "type": "linear_interpolation",
    "total_duration_ms": 2000,
    "acceleration_profile": "trapezoidal"
  }
}
```

---

## 4. Motor Feedback Schema

### 4.1 Real-time Feedback

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "MotorFeedback",
  "type": "object",
  "required": ["motor_id", "timestamp", "position", "velocity", "torque", "status"],
  "properties": {
    "motor_id": {
      "type": "string",
      "description": "Motor identifier"
    },
    "timestamp": {
      "type": "number",
      "description": "Feedback timestamp (Unix milliseconds)"
    },
    "position": {
      "type": "number",
      "description": "Current position (deg or rad)"
    },
    "velocity": {
      "type": "number",
      "description": "Current velocity (deg/s or rad/s)"
    },
    "acceleration": {
      "type": "number",
      "description": "Current acceleration"
    },
    "torque": {
      "type": "number",
      "description": "Current torque (Nm)"
    },
    "current": {
      "type": "number",
      "description": "Motor current (A)"
    },
    "voltage": {
      "type": "number",
      "description": "Motor voltage (V)"
    },
    "temperature": {
      "type": "number",
      "description": "Motor temperature (°C)"
    },
    "power": {
      "type": "number",
      "description": "Power consumption (W)"
    },
    "position_error": {
      "type": "number",
      "description": "Position tracking error"
    },
    "velocity_error": {
      "type": "number",
      "description": "Velocity tracking error"
    },
    "control_effort": {
      "type": "number",
      "description": "Current control effort (%)"
    },
    "status": {
      "type": "object",
      "properties": {
        "enabled": {"type": "boolean"},
        "fault": {"type": "boolean"},
        "warning": {"type": "boolean"},
        "calibrated": {"type": "boolean"},
        "homed": {"type": "boolean"},
        "in_motion": {"type": "boolean"},
        "at_target": {"type": "boolean"},
        "over_temperature": {"type": "boolean"},
        "over_current": {"type": "boolean"},
        "position_limit": {"type": "boolean"},
        "encoder_error": {"type": "boolean"}
      }
    },
    "health": {
      "type": "object",
      "properties": {
        "overall_score": {"type": "number", "minimum": 0, "maximum": 100},
        "hours_operated": {"type": "number"},
        "cycles_completed": {"type": "integer"},
        "maintenance_due": {"type": "boolean"},
        "estimated_lifetime": {"type": "number"}
      }
    }
  }
}
```

### 4.2 Feedback Example

```json
{
  "motor_id": "shoulder_pitch",
  "timestamp": 1704067200150,
  "position": 44.95,
  "velocity": 12.3,
  "acceleration": 5.2,
  "torque": 8.5,
  "current": 2.1,
  "voltage": 48.0,
  "temperature": 42.5,
  "power": 100.8,
  "position_error": 0.05,
  "velocity_error": 0.3,
  "control_effort": 45.2,
  "status": {
    "enabled": true,
    "fault": false,
    "warning": false,
    "calibrated": true,
    "homed": true,
    "in_motion": true,
    "at_target": false,
    "over_temperature": false,
    "over_current": false,
    "position_limit": false,
    "encoder_error": false
  },
  "health": {
    "overall_score": 95.5,
    "hours_operated": 1250.5,
    "cycles_completed": 458920,
    "maintenance_due": false,
    "estimated_lifetime": 8500.0
  }
}
```

---

## 5. Trajectory Schema

### 5.1 Trajectory Definition

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Trajectory",
  "type": "object",
  "required": ["trajectory_id", "motor_id", "waypoints", "timing"],
  "properties": {
    "trajectory_id": {
      "type": "string",
      "description": "Unique trajectory identifier"
    },
    "motor_id": {
      "type": "string",
      "description": "Target motor or motor group"
    },
    "type": {
      "enum": ["point_to_point", "continuous", "periodic", "spline"],
      "description": "Trajectory type"
    },
    "waypoints": {
      "type": "array",
      "minItems": 2,
      "items": {
        "type": "object",
        "required": ["position", "time"],
        "properties": {
          "position": {"type": "number"},
          "velocity": {"type": "number"},
          "acceleration": {"type": "number"},
          "time": {"type": "number"},
          "torque": {"type": "number"}
        }
      }
    },
    "timing": {
      "type": "object",
      "properties": {
        "start_time": {"type": "number"},
        "total_duration": {"type": "number"},
        "interpolation": {"enum": ["linear", "cubic", "quintic", "trapezoidal"]}
      }
    },
    "constraints": {
      "type": "object",
      "properties": {
        "max_velocity": {"type": "number"},
        "max_acceleration": {"type": "number"},
        "max_jerk": {"type": "number"},
        "max_torque": {"type": "number"}
      }
    },
    "loop": {
      "type": "object",
      "properties": {
        "enabled": {"type": "boolean"},
        "count": {"type": "integer", "minimum": -1},
        "reverse_on_loop": {"type": "boolean"}
      }
    }
  }
}
```

### 5.2 Trajectory Example

```json
{
  "trajectory_id": "traj_pick_and_place_001",
  "motor_id": "gripper_axis",
  "type": "spline",
  "waypoints": [
    {"position": 0.0, "velocity": 0.0, "time": 0.0},
    {"position": 45.0, "velocity": 30.0, "time": 1.5},
    {"position": 90.0, "velocity": 15.0, "time": 3.0},
    {"position": 90.0, "velocity": 0.0, "time": 4.0},
    {"position": 0.0, "velocity": 0.0, "time": 6.0}
  ],
  "timing": {
    "start_time": 1704067200000,
    "total_duration": 6000,
    "interpolation": "quintic"
  },
  "constraints": {
    "max_velocity": 60.0,
    "max_acceleration": 120.0,
    "max_jerk": 500.0,
    "max_torque": 15.0
  },
  "loop": {
    "enabled": false,
    "count": 1,
    "reverse_on_loop": false
  }
}
```

---

## 6. PID Controller Parameters

### 6.1 PID Configuration Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "PIDConfig",
  "type": "object",
  "required": ["motor_id", "control_mode", "gains"],
  "properties": {
    "motor_id": {"type": "string"},
    "control_mode": {
      "enum": ["position", "velocity", "torque"]
    },
    "gains": {
      "type": "object",
      "required": ["kp", "ki", "kd"],
      "properties": {
        "kp": {
          "type": "number",
          "minimum": 0,
          "description": "Proportional gain"
        },
        "ki": {
          "type": "number",
          "minimum": 0,
          "description": "Integral gain"
        },
        "kd": {
          "type": "number",
          "minimum": 0,
          "description": "Derivative gain"
        },
        "ki_limit": {
          "type": "number",
          "description": "Integral windup limit"
        },
        "feedforward": {
          "type": "number",
          "description": "Feedforward gain"
        },
        "derivative_filter": {
          "type": "number",
          "description": "Derivative filter coefficient"
        }
      }
    },
    "output_limits": {
      "type": "object",
      "properties": {
        "min": {"type": "number"},
        "max": {"type": "number"}
      }
    },
    "setpoint_filter": {
      "type": "object",
      "properties": {
        "enabled": {"type": "boolean"},
        "time_constant": {"type": "number"}
      }
    }
  }
}
```

---

## 7. Safety Limits Schema

### 7.1 Safety Configuration

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "SafetyLimits",
  "type": "object",
  "required": ["motor_id", "position_limits", "velocity_limits", "torque_limits"],
  "properties": {
    "motor_id": {"type": "string"},
    "position_limits": {
      "type": "object",
      "required": ["min", "max"],
      "properties": {
        "min": {"type": "number"},
        "max": {"type": "number"},
        "soft_min": {"type": "number"},
        "soft_max": {"type": "number"},
        "home_position": {"type": "number"}
      }
    },
    "velocity_limits": {
      "type": "object",
      "properties": {
        "max_velocity": {"type": "number", "minimum": 0},
        "emergency_decel": {"type": "number", "minimum": 0}
      }
    },
    "torque_limits": {
      "type": "object",
      "properties": {
        "continuous_torque": {"type": "number"},
        "peak_torque": {"type": "number"},
        "peak_duration_ms": {"type": "number"}
      }
    },
    "thermal_limits": {
      "type": "object",
      "properties": {
        "max_temperature": {"type": "number"},
        "warning_temperature": {"type": "number"},
        "thermal_derating": {"type": "boolean"}
      }
    },
    "current_limits": {
      "type": "object",
      "properties": {
        "continuous_current": {"type": "number"},
        "peak_current": {"type": "number"}
      }
    },
    "collision_detection": {
      "type": "object",
      "properties": {
        "enabled": {"type": "boolean"},
        "sensitivity": {"type": "number", "minimum": 0, "maximum": 100},
        "threshold": {"type": "number"}
      }
    }
  }
}
```

---

## 8. TypeScript Interfaces

### 8.1 Core Types

```typescript
// Motor Command
interface MotorCommand {
  motor_id: string;
  timestamp: number;
  control_mode: ControlMode;
  setpoint: number;
  velocity_limit?: number;
  acceleration_limit?: number;
  torque_limit?: number;
  pid_gains?: PIDGains;
  impedance_params?: ImpedanceParams;
  filter_config?: FilterConfig;
  safety_limits?: SafetyLimits;
  priority?: number;
  metadata?: Record<string, unknown>;
}

// PID Gains
interface PIDGains {
  kp: number;
  ki: number;
  kd: number;
  ki_limit?: number;
  feedforward?: number;
}

// Impedance Parameters
interface ImpedanceParams {
  stiffness: number;
  damping: number;
  inertia: number;
}

// Motor Feedback
interface MotorFeedback {
  motor_id: string;
  timestamp: number;
  position: number;
  velocity: number;
  acceleration?: number;
  torque: number;
  current: number;
  voltage: number;
  temperature: number;
  power?: number;
  position_error?: number;
  velocity_error?: number;
  control_effort?: number;
  status: MotorStatus;
  health?: MotorHealth;
}

// Motor Status
interface MotorStatus {
  enabled: boolean;
  fault: boolean;
  warning: boolean;
  calibrated: boolean;
  homed: boolean;
  in_motion: boolean;
  at_target: boolean;
  over_temperature: boolean;
  over_current: boolean;
  position_limit: boolean;
  encoder_error: boolean;
}

// Trajectory
interface Trajectory {
  trajectory_id: string;
  motor_id: string;
  type: 'point_to_point' | 'continuous' | 'periodic' | 'spline';
  waypoints: Waypoint[];
  timing: TrajectoryTiming;
  constraints?: TrajectoryConstraints;
  loop?: LoopConfig;
}

interface Waypoint {
  position: number;
  velocity?: number;
  acceleration?: number;
  time: number;
  torque?: number;
}
```

---

## 9. Python Data Classes

### 9.1 Motor Command

```python
from dataclasses import dataclass, field
from typing import Optional, Dict, Any
from enum import Enum

class ControlMode(Enum):
    POSITION = "position"
    VELOCITY = "velocity"
    TORQUE = "torque"
    IMPEDANCE = "impedance"
    TRAJECTORY = "trajectory"
    DISABLED = "disabled"

@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float
    ki_limit: Optional[float] = None
    feedforward: Optional[float] = 0.0

@dataclass
class ImpedanceParams:
    stiffness: float
    damping: float
    inertia: float

@dataclass
class MotorCommand:
    motor_id: str
    timestamp: int
    control_mode: ControlMode
    setpoint: float
    velocity_limit: Optional[float] = None
    acceleration_limit: Optional[float] = None
    torque_limit: Optional[float] = None
    pid_gains: Optional[PIDGains] = None
    impedance_params: Optional[ImpedanceParams] = None
    priority: int = 50
    metadata: Dict[str, Any] = field(default_factory=dict)

@dataclass
class MotorFeedback:
    motor_id: str
    timestamp: int
    position: float
    velocity: float
    torque: float
    current: float
    voltage: float
    temperature: float
    position_error: float = 0.0
    velocity_error: float = 0.0
    enabled: bool = True
    fault: bool = False
    warning: bool = False

@dataclass
class Waypoint:
    position: float
    time: float
    velocity: Optional[float] = None
    acceleration: Optional[float] = None
    torque: Optional[float] = None

@dataclass
class Trajectory:
    trajectory_id: str
    motor_id: str
    waypoints: list[Waypoint]
    total_duration: float
    interpolation: str = "quintic"
    max_velocity: Optional[float] = None
    max_acceleration: Optional[float] = None
```

---

## 10. Validation Rules

### 10.1 Data Validation

1. **Timestamp Validation**
   - Must be positive integer (Unix milliseconds)
   - Future timestamps allowed for planned commands
   - Maximum 5-minute clock skew tolerance

2. **Control Mode Validation**
   - Must be valid enum value
   - Mode transitions must be explicitly handled
   - Some modes require specific parameters

3. **Setpoint Validation**
   - Must be within configured safety limits
   - Position: min_position ≤ setpoint ≤ max_position
   - Velocity: |setpoint| ≤ max_velocity
   - Torque: |setpoint| ≤ max_torque

4. **PID Gains Validation**
   - All gains must be non-negative
   - kp > 0 for stable control
   - ki_limit must be positive if specified

5. **Trajectory Validation**
   - Minimum 2 waypoints required
   - Waypoint times must be monotonically increasing
   - Must satisfy velocity/acceleration constraints
   - Continuity requirements for spline trajectories

---

## 11. Binary Format (Optional)

For high-frequency communication, binary format can be used:

### 11.1 Compact Motor Command (24 bytes)

```
Offset  Size  Field
------  ----  -----
0       4     motor_id (uint32 hash)
4       8     timestamp (uint64)
12      1     control_mode (uint8)
13      1     flags (uint8)
14      2     reserved
16      4     setpoint (float32)
20      2     velocity_limit (uint16, scaled)
22      2     torque_limit (uint16, scaled)
```

---

## 12. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial specification |

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01
**Author**: WIA AI Motor Control Working Group

---

弘益人間 - *Benefit All Humanity*
