# WIA-IND-024: Manufacturing Automation Specification v1.0

> **Standard ID:** WIA-IND-024
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Industry Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Production Line Automation](#2-production-line-automation)
3. [PLC and SCADA Integration](#3-plc-and-scada-integration)
4. [Robot Arm Control](#4-robot-arm-control)
5. [Conveyor Systems](#5-conveyor-systems)
6. [Assembly Automation](#6-assembly-automation)
7. [Quality Inspection Automation](#7-quality-inspection-automation)
8. [Material Handling](#8-material-handling)
9. [Production Scheduling](#9-production-scheduling)
10. [OEE Monitoring](#10-oee-monitoring)
11. [Safety Interlocks](#11-safety-interlocks)
12. [Communication Protocols](#12-communication-protocols)
13. [Data Models](#13-data-models)
14. [Security and Access Control](#14-security-and-access-control)
15. [Implementation Guidelines](#15-implementation-guidelines)
16. [References](#16-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive standard for manufacturing automation systems, enabling interoperability between production equipment, control systems, and enterprise applications across diverse manufacturing environments.

### 1.2 Scope

The standard covers:
- Production line control and orchestration
- PLC/SCADA system integration protocols
- Industrial robot programming and coordination
- Conveyor and material transport systems
- Automated assembly processes
- Quality inspection and testing automation
- Warehouse and material handling systems
- AI-optimized production scheduling
- Real-time OEE (Overall Equipment Effectiveness) monitoring
- Comprehensive safety interlock systems

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard democratizes advanced manufacturing automation, making Industry 4.0 technologies accessible to factories of all sizes while promoting worker safety, environmental sustainability, and economic prosperity.

### 1.4 Terminology

- **PLC**: Programmable Logic Controller
- **SCADA**: Supervisory Control and Data Acquisition
- **MES**: Manufacturing Execution System
- **ERP**: Enterprise Resource Planning
- **OEE**: Overall Equipment Effectiveness
- **AGV**: Automated Guided Vehicle
- **AS/RS**: Automated Storage and Retrieval System
- **HMI**: Human-Machine Interface
- **IoT**: Internet of Things
- **IIoT**: Industrial Internet of Things
- **TSN**: Time-Sensitive Networking
- **OPC UA**: Open Platform Communications Unified Architecture
- **MQTT**: Message Queuing Telemetry Transport
- **Digital Twin**: Virtual representation of physical manufacturing system
- **Predictive Maintenance**: AI-based maintenance scheduling
- **Lights-Out Manufacturing**: Fully autonomous operation without human presence

### 1.5 Design Principles

1. **Interoperability**: Vendor-neutral protocols and data formats
2. **Modularity**: Plug-and-play components and subsystems
3. **Scalability**: From single machine to multi-factory deployment
4. **Real-Time**: Deterministic communication for time-critical operations
5. **Safety-First**: Comprehensive safety systems with redundancy
6. **Intelligence**: AI/ML optimization and predictive analytics
7. **Openness**: Open-source reference implementations
8. **Sustainability**: Energy efficiency and environmental monitoring

---

## 2. Production Line Automation

### 2.1 Production Line Architecture

#### 2.1.1 Hierarchical Control Structure

```
Level 5: Enterprise (ERP)
    ↓
Level 4: Manufacturing Operations (MES)
    ↓
Level 3: Production Supervision (SCADA)
    ↓
Level 2: Cell Control (PLC/PAC)
    ↓
Level 1: Field Devices (Sensors, Actuators)
    ↓
Level 0: Physical Process (Machines, Robots)
```

#### 2.1.2 Production Line Components

**Workstation Types:**
1. **Assembly Stations**: Part joining, fastening, welding
2. **Machining Stations**: CNC, drilling, milling, turning
3. **Processing Stations**: Heat treatment, coating, cleaning
4. **Inspection Stations**: Measurement, testing, quality control
5. **Packaging Stations**: Boxing, labeling, palletizing

**Transport Systems:**
- Belt conveyors
- Roller conveyors
- Chain conveyors
- Overhead conveyors
- AGVs (Automated Guided Vehicles)
- AMRs (Autonomous Mobile Robots)

**Buffer Systems:**
- FIFO buffers
- LIFO buffers
- Random access buffers
- Accumulation zones

### 2.2 Line Control Strategies

#### 2.2.1 Push vs Pull Systems

**Push System:**
```
Production based on forecast → MRP → Schedule → Execute
Advantages: High utilization, economies of scale
Disadvantages: Inventory buildup, inflexibility
```

**Pull System (Kanban/JIT):**
```
Customer order → Signal upstream → Produce only what's needed
Advantages: Low inventory, flexible, quality focus
Disadvantages: Vulnerable to disruptions
```

#### 2.2.2 Line Balancing

**Objective:** Minimize cycle time while balancing workload

```
Line Efficiency = (Sum of Task Times) / (Number of Stations × Cycle Time) × 100%

Balancing Loss = 1 - Line Efficiency

Example:
Total task time: 180 seconds
Stations: 6
Cycle time: 35 seconds
Line Efficiency = 180 / (6 × 35) = 85.7%
```

### 2.3 Production Line Data Model

```json
{
  "productionLine": {
    "lineId": "LINE-A",
    "facilityId": "FAC-001",
    "name": "Assembly Line A",
    "type": "mixed-model",
    "status": "running",
    "workstations": [
      {
        "stationId": "WS-01",
        "type": "assembly",
        "position": 1,
        "equipment": [
          {
            "equipmentId": "ROBOT-A1",
            "type": "robot-arm",
            "model": "UR10e",
            "status": "operational",
            "currentTask": "pick-and-place",
            "cycleTime": 12.5,
            "uptime": 0.987
          }
        ],
        "cycleTime": 45,
        "targetOutput": 80,
        "currentOutput": 78,
        "efficiency": 0.975
      }
    ],
    "conveyors": [
      {
        "conveyorId": "CONV-001",
        "type": "belt",
        "length": 15.0,
        "speed": 0.5,
        "direction": "forward",
        "loadStatus": "75%"
      }
    ],
    "metrics": {
      "oee": 0.842,
      "availability": 0.912,
      "performance": 0.954,
      "quality": 0.967,
      "targetCycleTime": 45,
      "actualCycleTime": 47.2,
      "unitsProduced": 1247,
      "defectiveUnits": 41
    }
  }
}
```

### 2.4 Line Control Commands

#### 2.4.1 Basic Operations

```typescript
// Start production line
startLine(lineId: string, configuration: LineConfig): Promise<void>

// Stop production line
stopLine(lineId: string, reason: string): Promise<void>

// Emergency stop
emergencyStop(lineId: string): Promise<void>

// Change production speed
setLineSpeed(lineId: string, speedPercent: number): Promise<void>

// Switch product model
changeModel(lineId: string, modelId: string): Promise<void>
```

#### 2.4.2 Advanced Operations

```typescript
// Dynamic line balancing
rebalanceLine(lineId: string, optimization: OptimizationStrategy): Promise<BalanceResult>

// Predictive maintenance scheduling
scheduleMaintenance(lineId: string, strategy: MaintenanceStrategy): Promise<Schedule>

// Energy optimization
optimizeEnergy(lineId: string, constraints: EnergyConstraints): Promise<EnergyProfile>
```

---

## 3. PLC and SCADA Integration

### 3.1 PLC Communication Protocols

#### 3.1.1 Supported Protocols

**Siemens:**
- S7 Protocol (S7-300, S7-400, S7-1200, S7-1500)
- PROFINET
- Industrial Ethernet

**Allen-Bradley (Rockwell):**
- EtherNet/IP
- ControlNet
- DeviceNet
- CIP (Common Industrial Protocol)

**Schneider Electric:**
- Modbus TCP/IP
- Modbus RTU
- CANopen

**Mitsubishi:**
- MELSEC Protocol
- CC-Link
- CC-Link IE

**Omron:**
- FINS Protocol
- EtherNet/IP

#### 3.1.2 OPC UA Integration

**Connection Configuration:**
```json
{
  "opcua": {
    "endpoint": "opc.tcp://192.168.1.100:4840",
    "securityMode": "SignAndEncrypt",
    "securityPolicy": "Basic256Sha256",
    "authentication": {
      "type": "username",
      "username": "factory_admin",
      "password": "encrypted_password"
    },
    "namespaces": [
      {
        "uri": "urn:factory:line-a",
        "index": 2
      }
    ],
    "nodes": [
      {
        "nodeId": "ns=2;s=Station.WS01.Temperature",
        "dataType": "Float",
        "accessLevel": "CurrentRead"
      },
      {
        "nodeId": "ns=2;s=Station.WS01.Production.Count",
        "dataType": "UInt32",
        "accessLevel": "CurrentReadOrWrite"
      }
    ]
  }
}
```

### 3.2 PLC Data Structures

#### 3.2.1 Memory Areas (Siemens S7)

```
Inputs (I):      Digital/analog inputs from field devices
Outputs (Q):     Digital/analog outputs to actuators
Flags (M):       Internal memory (bits, bytes, words)
Data Blocks (DB): Structured data storage
Timers (T):      Timer functions
Counters (C):    Counter functions
```

**Data Block Structure:**
```
DB1 - Production Data
  DBW0:   Current production count (WORD)
  DBW2:   Target production count (WORD)
  DBW4:   Cycle time in ms (WORD)
  DBD6:   Total runtime in seconds (DWORD)
  DBX10.0: Line running (BOOL)
  DBX10.1: Emergency stop active (BOOL)
  DBX10.2: Maintenance required (BOOL)
  DBX10.3: Quality alarm (BOOL)
```

#### 3.2.2 Address Mapping

```typescript
interface PLCAddress {
  // Siemens S7
  area: 'I' | 'Q' | 'M' | 'DB';
  dbNumber?: number;      // For DB area
  byteOffset: number;     // Byte address
  bitOffset?: number;     // Bit address (0-7)
  dataType: 'BOOL' | 'BYTE' | 'WORD' | 'DWORD' | 'REAL' | 'STRING';

  // Full address examples:
  // "I0.0"     - Input bit 0.0
  // "Q2.5"     - Output bit 2.5
  // "DB1.DBW0" - Data block 1, word 0
  // "DB1.DBD6" - Data block 1, double word 6
}
```

### 3.3 SCADA System Integration

#### 3.3.1 SCADA Architecture

```
┌─────────────────────────────────────────────┐
│  SCADA Server (Master Station)             │
│  - Data historian                           │
│  - Alarm management                         │
│  - Trend analysis                           │
│  - Report generation                        │
└─────────────────┬───────────────────────────┘
                  │
         ┌────────┴────────┐
         │                 │
    ┌────▼────┐      ┌────▼────┐
    │  HMI 1  │      │  HMI 2  │
    │(Operator│      │(Engineer│
    │ Station)│      │ Station)│
    └────┬────┘      └────┬────┘
         │                │
    ┌────┴────────────────┴────┐
    │   Industrial Network     │
    │   (Ethernet, PROFINET)   │
    └────┬────────────────┬────┘
         │                │
    ┌────▼────┐      ┌───▼─────┐
    │ PLC 1   │      │ PLC 2   │
    │(Line A) │      │(Line B) │
    └─────────┘      └─────────┘
```

#### 3.3.2 Real-Time Data Acquisition

**Tag Configuration:**
```json
{
  "tags": [
    {
      "tagName": "LINE_A.WS01.TEMP",
      "description": "Workstation 1 Temperature",
      "dataSource": "PLC1.DB1.DBW10",
      "dataType": "REAL",
      "unit": "°C",
      "scanRate": 1000,
      "deadband": 0.5,
      "alarmLimits": {
        "highHigh": 85.0,
        "high": 80.0,
        "low": 10.0,
        "lowLow": 5.0
      },
      "trending": {
        "enabled": true,
        "retentionDays": 365
      }
    },
    {
      "tagName": "LINE_A.PRODUCTION.COUNT",
      "description": "Production Counter",
      "dataSource": "PLC1.DB1.DBW0",
      "dataType": "UINT",
      "scanRate": 500,
      "trending": {
        "enabled": true,
        "retentionDays": 730
      }
    }
  ]
}
```

### 3.4 Industrial Communication Protocols

#### 3.4.1 Modbus Protocol

**Function Codes:**
```
01: Read Coils (digital outputs)
02: Read Discrete Inputs
03: Read Holding Registers (16-bit)
04: Read Input Registers
05: Write Single Coil
06: Write Single Register
15: Write Multiple Coils
16: Write Multiple Registers
```

**Modbus TCP Frame:**
```
Transaction ID (2 bytes)
Protocol ID (2 bytes) = 0x0000
Length (2 bytes)
Unit ID (1 byte)
Function Code (1 byte)
Data (N bytes)
```

**Example: Read Holding Registers**
```javascript
// Read 10 registers starting at address 100
{
  transactionId: 0x0001,
  protocolId: 0x0000,
  length: 6,
  unitId: 1,
  functionCode: 0x03,
  startAddress: 100,
  quantity: 10
}
```

#### 3.4.2 MQTT for IIoT

**Topic Structure:**
```
factory/{facilityId}/line/{lineId}/workstation/{wsId}/metric/{metricName}

Examples:
factory/FAC001/line/LINE-A/workstation/WS01/metric/temperature
factory/FAC001/line/LINE-A/workstation/WS01/metric/production-count
factory/FAC001/line/LINE-A/workstation/WS01/status/running
factory/FAC001/line/LINE-A/alarm/emergency-stop
```

**Message Format:**
```json
{
  "timestamp": "2025-12-27T10:30:45.123Z",
  "facilityId": "FAC-001",
  "lineId": "LINE-A",
  "workstationId": "WS-01",
  "metricName": "temperature",
  "value": 72.5,
  "unit": "°C",
  "quality": "good",
  "source": "PLC1.DB1.DBW10"
}
```

---

## 4. Robot Arm Control

### 4.1 Robot Types and Specifications

#### 4.1.1 Industrial Robot Classifications

**By Configuration:**
1. **Articulated (6-axis)**: Most versatile, automotive assembly
2. **SCARA (4-axis)**: Fast pick-and-place, electronics assembly
3. **Delta/Parallel**: Ultra-high speed, food packaging
4. **Cartesian/Gantry**: Large workspace, CNC loading
5. **Collaborative (Cobot)**: Safe human interaction, light assembly

**By Payload:**
- Micro: <1 kg (electronics)
- Small: 1-5 kg (assembly)
- Medium: 5-25 kg (material handling)
- Large: 25-100 kg (palletizing)
- Heavy: >100 kg (automotive)

#### 4.1.2 Robot Specifications

```json
{
  "robot": {
    "robotId": "ROBOT-A1",
    "manufacturer": "Universal Robots",
    "model": "UR10e",
    "type": "collaborative-6axis",
    "specifications": {
      "reach": 1300,
      "payload": 12.5,
      "repeatability": 0.05,
      "axes": 6,
      "maxSpeed": {
        "joint": 180,
        "tcp": 1.0
      },
      "weight": 33.5
    },
    "units": {
      "reach": "mm",
      "payload": "kg",
      "repeatability": "mm",
      "maxSpeed": "deg/s and m/s",
      "weight": "kg"
    }
  }
}
```

### 4.2 Robot Programming

#### 4.2.1 Motion Commands

**Joint Space Movement:**
```typescript
interface JointPosition {
  j1: number;  // Base rotation (deg)
  j2: number;  // Shoulder (deg)
  j3: number;  // Elbow (deg)
  j4: number;  // Wrist 1 (deg)
  j5: number;  // Wrist 2 (deg)
  j6: number;  // Wrist 3 (deg)
}

// Move to joint position
moveJ(position: JointPosition, velocity: number, acceleration: number): Promise<void>
```

**Cartesian Space Movement:**
```typescript
interface CartesianPose {
  x: number;      // mm
  y: number;      // mm
  z: number;      // mm
  rx: number;     // Roll (deg)
  ry: number;     // Pitch (deg)
  rz: number;     // Yaw (deg)
}

// Linear movement
moveL(pose: CartesianPose, velocity: number, acceleration: number): Promise<void>

// Circular movement
moveC(via: CartesianPose, target: CartesianPose, velocity: number): Promise<void>
```

#### 4.2.2 End Effector Control

**Gripper Commands:**
```typescript
interface GripperControl {
  // Open/close gripper
  setGripperPosition(position: number): Promise<void>;  // 0-100%

  // Set grip force
  setGripperForce(force: number): Promise<void>;  // N

  // Detect object
  detectObject(): Promise<boolean>;

  // Get gripper status
  getGripperStatus(): Promise<GripperStatus>;
}

interface GripperStatus {
  position: number;      // Current position (0-100%)
  force: number;         // Applied force (N)
  objectDetected: boolean;
  error: string | null;
}
```

**Tool Center Point (TCP) Configuration:**
```typescript
interface TCPConfiguration {
  // Tool offset from robot flange
  offset: {
    x: number;  // mm
    y: number;  // mm
    z: number;  // mm
    rx: number; // deg
    ry: number; // deg
    rz: number; // deg
  };

  // Tool weight for dynamics compensation
  mass: number;  // kg

  // Center of gravity
  cog: {
    x: number;  // mm
    y: number;  // mm
    z: number;  // mm
  };
}
```

### 4.3 Robot Coordination

#### 4.3.1 Multi-Robot Coordination

**Collision Avoidance:**
```typescript
interface WorkspaceZone {
  zoneId: string;
  type: 'exclusive' | 'shared' | 'handoff';
  bounds: {
    xMin: number;
    xMax: number;
    yMin: number;
    yMax: number;
    zMin: number;
    zMax: number;
  };
  assignedRobots: string[];
  priority: number;
}

// Request workspace access
requestWorkspace(robotId: string, zoneId: string): Promise<boolean>

// Release workspace
releaseWorkspace(robotId: string, zoneId: string): Promise<void>
```

#### 4.3.2 Synchronized Motion

```typescript
// Synchronize two robots for cooperative handling
interface CooperativeTask {
  robots: string[];
  syncPoints: CartesianPose[];
  payload: {
    mass: number;
    dimensions: { length: number; width: number; height: number };
    graspPoints: CartesianPose[];
  };
}

executeSynchronized(task: CooperativeTask): Promise<void>
```

### 4.4 Vision-Guided Robotics

#### 4.4.1 Vision System Integration

```json
{
  "visionSystem": {
    "systemId": "VISION-01",
    "cameras": [
      {
        "cameraId": "CAM-01",
        "type": "2D-area-scan",
        "resolution": [2048, 2048],
        "fps": 60,
        "mountPosition": "eye-in-hand",
        "calibration": {
          "intrinsic": "matrix_3x3",
          "extrinsic": "matrix_4x4",
          "distortion": "coefficients"
        }
      }
    ],
    "processing": {
      "patternMatching": true,
      "blobAnalysis": true,
      "OCR": true,
      "barcodeReading": true,
      "dimensionMeasurement": true
    }
  }
}
```

#### 4.4.2 Pick-and-Place with Vision

```typescript
interface VisionGuidedPick {
  // Capture image
  image: CaptureImage(): Promise<Image>;

  // Detect parts
  parts: DetectParts(image: Image, template: Template): Promise<PartLocation[]>;

  // Calculate pick pose
  pose: CalculatePickPose(part: PartLocation, tcpOffset: TCPConfiguration): Promise<CartesianPose>;

  // Execute pick
  success: ExecutePick(robot: RobotArm, pose: CartesianPose, gripForce: number): Promise<boolean>;
}
```

---

## 5. Conveyor Systems

### 5.1 Conveyor Types

#### 5.1.1 Belt Conveyors

**Specifications:**
```json
{
  "conveyorId": "CONV-001",
  "type": "belt",
  "specifications": {
    "length": 15000,
    "width": 600,
    "speed": {
      "min": 0.1,
      "max": 2.0,
      "current": 0.5
    },
    "loadCapacity": 100,
    "beltMaterial": "PVC",
    "motorPower": 1.5
  },
  "units": {
    "length": "mm",
    "width": "mm",
    "speed": "m/s",
    "loadCapacity": "kg/m",
    "motorPower": "kW"
  }
}
```

#### 5.1.2 Roller Conveyors

**Powered Roller:**
- Individual motor control
- Accumulation capability
- Zero-pressure accumulation (ZPA)
- Dynamic zone control

**Gravity Roller:**
- No power required
- Downhill flow
- Manual push sections

### 5.2 Conveyor Control

#### 5.2.1 Speed Control

```typescript
interface ConveyorControl {
  // Set conveyor speed
  setSpeed(conveyorId: string, speed: number): Promise<void>;

  // Start/stop conveyor
  start(conveyorId: string): Promise<void>;
  stop(conveyorId: string): Promise<void>;

  // Reverse direction
  reverse(conveyorId: string): Promise<void>;

  // Emergency stop
  emergencyStop(conveyorId: string): Promise<void>;
}
```

#### 5.2.2 Zone Control

```typescript
interface ConveyorZone {
  zoneId: string;
  conveyorId: string;
  startPosition: number;  // mm from origin
  endPosition: number;    // mm from origin
  sensors: {
    entryBlockedSensor: string;
    exitBlockedSensor: string;
    productPresentSensor: string;
  };
  mode: 'singulation' | 'accumulation' | 'merging';
}

// Zone control logic
controlZone(zone: ConveyorZone, state: ZoneState): Promise<void>
```

### 5.3 Part Tracking

#### 5.3.1 Tracking Methods

**Barcode/QR Code:**
```typescript
interface BarcodeScanner {
  scannerId: string;
  position: number;  // Position on conveyor (mm)
  symbologies: string[];  // Supported barcode types
  readRate: number;  // Reads per second
}

// Scan event
interface ScanEvent {
  timestamp: Date;
  scannerId: string;
  barcode: string;
  symbology: string;
  quality: number;  // 0-100
}
```

**RFID Tracking:**
```typescript
interface RFIDReader {
  readerId: string;
  position: number;
  frequency: number;  // MHz (13.56, 433, 915)
  readRange: number;  // mm
  multiRead: boolean;
}

interface RFIDTag {
  epc: string;  // Electronic Product Code
  tid: string;  // Tag ID
  userData: string;
  rssi: number;  // Signal strength
}
```

#### 5.3.2 Position Tracking

```typescript
interface PartPosition {
  partId: string;
  conveyorId: string;
  position: number;  // mm from origin
  velocity: number;  // m/s
  entryTime: Date;
  estimatedExitTime: Date;
}

// Update position based on encoder
updatePosition(partId: string, encoderPulses: number): PartPosition
```

---

## 6. Assembly Automation

### 6.1 Assembly Processes

#### 6.1.1 Fastening Operations

**Screw Driving:**
```typescript
interface ScrewDrivingStation {
  stationId: string;
  screwdriver: {
    type: 'electric' | 'pneumatic';
    torque: {
      min: number;  // Nm
      max: number;  // Nm
      target: number;
    };
    speed: number;  // RPM
    angleControl: boolean;
  };
  feedSystem: {
    type: 'bowl-feeder' | 'strip-feed' | 'bulk-feed';
    capacity: number;
  };
}

// Execute screw driving
interface ScrewDrivingResult {
  success: boolean;
  finalTorque: number;  // Nm
  finalAngle: number;   // degrees
  time: number;         // seconds
  curve: TorqueAngleCurve;
}

executeScrewDriving(params: ScrewDrivingParams): Promise<ScrewDrivingResult>
```

**Riveting:**
```typescript
interface RivetingStation {
  stationId: string;
  rivetGun: {
    type: 'pneumatic' | 'hydraulic' | 'servo';
    force: number;  // kN
    stroke: number; // mm
  };
}
```

#### 6.1.2 Welding Operations

**Spot Welding:**
```json
{
  "spotWeldingStation": {
    "stationId": "WELD-01",
    "welder": {
      "type": "resistance-spot",
      "electrodeForce": 4500,
      "weldCurrent": 12000,
      "weldTime": 300,
      "holdTime": 150,
      "units": {
        "force": "N",
        "current": "A",
        "time": "ms"
      }
    },
    "robot": "ROBOT-W1",
    "weldGun": {
      "electrodeType": "Class 2 Copper",
      "electrodeLife": 5000,
      "currentCount": 3247
    }
  }
}
```

**Arc Welding:**
```typescript
interface ArcWeldingParams {
  weldType: 'MIG' | 'TIG' | 'Laser';
  voltage: number;  // V
  current: number;  // A
  wireSpeed: number;  // mm/s
  travelSpeed: number;  // mm/s
  shieldingGas: string;
  gasFlowRate: number;  // L/min
}
```

### 6.2 Part Feeding Systems

#### 6.2.1 Vibratory Bowl Feeders

```typescript
interface BowlFeeder {
  feederId: string;
  diameter: number;  // mm
  capacity: number;  // parts
  feedRate: number;  // parts/min
  partOrientation: string;
  sensors: {
    levelSensor: boolean;
    jamSensor: boolean;
    outputSensor: boolean;
  };
}
```

#### 6.2.2 Flexible Feeders

**Vision-Based Bin Picking:**
```typescript
interface BinPickingSystem {
  systemId: string;
  camera: {
    type: '3D-structured-light' | '3D-stereo' | '2D-area';
    resolution: [number, number];
  };
  algorithm: {
    segmentation: string;
    poseEstimation: string;
    graspPlanning: string;
  };
  successRate: number;  // %
  cycleTime: number;    // seconds
}
```

### 6.3 Assembly Verification

#### 6.3.1 Presence Detection

```typescript
interface PresenceCheck {
  checkId: string;
  method: 'vision' | 'laser' | 'ultrasonic' | 'force';
  location: CartesianPose;
  tolerance: number;  // mm
  result: 'present' | 'absent' | 'misaligned';
}
```

#### 6.3.2 Force/Torque Monitoring

```typescript
interface ForceTorqueSensor {
  sensorId: string;
  capacity: {
    force: [number, number, number];  // Fx, Fy, Fz (N)
    torque: [number, number, number]; // Tx, Ty, Tz (Nm)
  };
  resolution: number;
  sampleRate: number;  // Hz
}

// Monitor assembly force
interface AssemblyForceProfile {
  timestamps: number[];
  forces: number[][];
  torques: number[][];
  anomalyDetected: boolean;
}
```

---

## 7. Quality Inspection Automation

### 7.1 Vision Inspection

#### 7.1.1 2D Vision Inspection

**Defect Detection:**
```typescript
interface VisionInspection {
  inspectionId: string;
  partId: string;
  defectTypes: string[];
  results: {
    defectType: string;
    location: { x: number; y: number };
    severity: 'critical' | 'major' | 'minor';
    confidence: number;  // 0-1
  }[];
  overallResult: 'pass' | 'fail' | 'review';
}

// Inspection criteria
interface InspectionCriteria {
  scratches: { maxLength: number; maxWidth: number; maxCount: number };
  dents: { maxDepth: number; maxDiameter: number; maxCount: number };
  contamination: { maxArea: number };
  colorVariation: { maxDeltaE: number };
}
```

#### 7.1.2 3D Vision Inspection

**Dimensional Measurement:**
```typescript
interface DimensionalInspection {
  features: {
    featureName: string;
    type: 'length' | 'width' | 'height' | 'diameter' | 'angle' | 'flatness';
    nominal: number;
    upperTolerance: number;
    lowerTolerance: number;
    measured: number;
    deviation: number;
    result: 'pass' | 'fail';
  }[];
}

// 3D scanner configuration
interface Scanner3D {
  scannerId: string;
  technology: 'laser-triangulation' | 'structured-light' | 'photogrammetry';
  accuracy: number;  // mm
  resolution: number;  // mm
  scanTime: number;  // seconds
  pointCloudDensity: number;  // points/mm²
}
```

### 7.2 Non-Destructive Testing (NDT)

#### 7.2.1 X-Ray Inspection

```json
{
  "xraySystem": {
    "systemId": "XRAY-01",
    "type": "2D-radiography",
    "voltage": 160,
    "current": 5.0,
    "detectorType": "flat-panel",
    "resolution": 0.1,
    "inspectionCapabilities": [
      "void-detection",
      "crack-detection",
      "foreign-object-detection",
      "solder-joint-inspection"
    ],
    "units": {
      "voltage": "kV",
      "current": "mA",
      "resolution": "mm"
    }
  }
}
```

#### 7.2.2 Ultrasonic Testing

```typescript
interface UltrasonicTesting {
  testId: string;
  method: 'pulse-echo' | 'through-transmission';
  frequency: number;  // MHz
  results: {
    depth: number;  // mm
    amplitude: number;  // %
    defectType: 'void' | 'delamination' | 'inclusion';
    severity: number;
  }[];
}
```

### 7.3 Functional Testing

#### 7.3.1 Electrical Testing

```typescript
interface ElectricalTest {
  testId: string;
  testType: 'continuity' | 'resistance' | 'voltage' | 'current' | 'functional';
  testPoints: {
    pointId: string;
    expected: number;
    measured: number;
    tolerance: number;
    unit: string;
    result: 'pass' | 'fail';
  }[];
}
```

#### 7.3.2 Leak Testing

```typescript
interface LeakTest {
  testId: string;
  method: 'pressure-decay' | 'vacuum-decay' | 'mass-flow';
  testPressure: number;  // kPa
  testDuration: number;  // seconds
  maxLeakRate: number;   // ml/min
  measuredLeakRate: number;
  result: 'pass' | 'fail';
}
```

---

## 8. Material Handling

### 8.1 Automated Guided Vehicles (AGV)

#### 8.1.1 AGV Types

**Wire-Guided AGV:**
```json
{
  "agvId": "AGV-001",
  "type": "wire-guided",
  "navigation": {
    "method": "magnetic-wire",
    "accuracy": 10
  },
  "specifications": {
    "payload": 1000,
    "speed": 1.5,
    "batteryCapacity": 48,
    "batteryVoltage": 48,
    "chargingTime": 4
  },
  "units": {
    "accuracy": "mm",
    "payload": "kg",
    "speed": "m/s",
    "batteryCapacity": "Ah",
    "batteryVoltage": "V",
    "chargingTime": "hours"
  }
}
```

**Autonomous Mobile Robot (AMR):**
```typescript
interface AMR {
  amrId: string;
  navigation: {
    method: 'SLAM' | 'vision' | 'lidar';
    sensors: {
      lidar: { range: number; resolution: number };
      cameras: { count: number; resolution: [number, number] };
      imu: boolean;
      wheelEncoders: boolean;
    };
  };
  pathPlanning: {
    algorithm: 'A*' | 'RRT' | 'DWA';
    obstacleAvoidance: boolean;
    dynamicReplanning: boolean;
  };
}
```

### 8.2 Automated Storage and Retrieval Systems (AS/RS)

#### 8.2.1 AS/RS Configuration

```typescript
interface ASRSConfiguration {
  systemId: string;
  type: 'unit-load' | 'mini-load' | 'shuttle-system';
  dimensions: {
    aisles: number;
    levels: number;
    columns: number;
    height: number;
    length: number;
  };
  capacity: {
    totalLocations: number;
    palletSize: { length: number; width: number; height: number };
    maxWeight: number;
  };
  crane: {
    horizontalSpeed: number;  // m/s
    verticalSpeed: number;    // m/s
    acceleration: number;     // m/s²
    cycleTime: number;        // seconds (average)
  };
}
```

#### 8.2.2 Warehouse Management

```typescript
interface WarehouseOperation {
  // Store item
  store(item: Item, strategy: 'nearest' | 'ABC' | 'random'): Promise<Location>;

  // Retrieve item
  retrieve(itemId: string): Promise<Item>;

  // Inventory count
  getInventory(filter?: InventoryFilter): Promise<InventoryReport>;

  // Optimize storage
  optimizeLayout(strategy: OptimizationStrategy): Promise<LayoutPlan>;
}

interface Item {
  itemId: string;
  sku: string;
  quantity: number;
  dimensions: { length: number; width: number; height: number };
  weight: number;
  classification: 'A' | 'B' | 'C';  // ABC analysis
  expiryDate?: Date;
}
```

### 8.3 Palletizing and Depalletizing

#### 8.3.1 Palletizing Patterns

```typescript
interface PalletizingPattern {
  patternId: string;
  palletSize: { length: number; width: number };
  boxSize: { length: number; width: number; height: number };
  layers: {
    layerNumber: number;
    boxes: {
      position: { x: number; y: number; z: number };
      orientation: number;  // degrees
    }[];
  }[];
  totalBoxes: number;
  stability: number;  // 0-1
}

// Generate optimal pattern
generatePattern(pallet: PalletSize, box: BoxSize, maxHeight: number): PalletizingPattern
```

#### 8.3.2 Robotic Palletizing

```typescript
interface PalletizingRobot {
  robotId: string;
  endEffector: 'vacuum' | 'clamp' | 'fork';
  capacity: number;  // kg
  cycleTime: number;  // seconds per box
  throughput: number;  // boxes per hour
}

// Execute palletizing
palletize(boxes: Box[], pattern: PalletizingPattern, robot: PalletizingRobot): Promise<void>
```

---

## 9. Production Scheduling

### 9.1 Scheduling Algorithms

#### 9.1.1 Traditional Methods

**First-Come-First-Served (FCFS):**
```
Simple queue-based scheduling
Advantages: Simple, fair
Disadvantages: May not optimize throughput
```

**Shortest Processing Time (SPT):**
```
Priority to jobs with shortest processing time
Advantages: Minimizes average completion time
Disadvantages: Long jobs may starve
```

**Earliest Due Date (EDD):**
```
Priority to jobs with earliest due date
Advantages: Minimizes maximum lateness
Disadvantages: May ignore processing time
```

#### 9.1.2 AI-Optimized Scheduling

```typescript
interface SchedulingProblem {
  jobs: {
    jobId: string;
    product: string;
    quantity: number;
    dueDate: Date;
    priority: number;
    setupTime: number;  // seconds
    processingTime: number;  // seconds per unit
  }[];

  resources: {
    resourceId: string;
    type: 'machine' | 'labor' | 'material';
    capacity: number;
    availability: TimeWindow[];
  }[];

  constraints: {
    precedence: { jobA: string; jobB: string }[];
    capacityLimits: { resourceId: string; limit: number }[];
    timeWindows: { jobId: string; window: TimeWindow }[];
  };

  objectives: {
    minimizeMakespan: number;      // 0-1 weight
    minimizeLateness: number;      // 0-1 weight
    minimizeSetupTime: number;     // 0-1 weight
    maximizeUtilization: number;   // 0-1 weight
  };
}

// AI-based scheduler
interface SchedulingResult {
  schedule: {
    jobId: string;
    resourceId: string;
    startTime: Date;
    endTime: Date;
    setupStart: Date;
    processingStart: Date;
  }[];

  metrics: {
    makespan: number;          // Total time (seconds)
    averageLateness: number;   // Average delay (seconds)
    totalSetupTime: number;    // Total setup (seconds)
    resourceUtilization: { [resourceId: string]: number };  // %
  };

  ganttChart: GanttData;
}

optimizeSchedule(problem: SchedulingProblem): Promise<SchedulingResult>
```

### 9.2 Capacity Planning

#### 9.2.1 Production Capacity Calculation

```typescript
// Calculate line capacity
interface CapacityCalculation {
  // Theoretical capacity
  theoreticalCapacity = (availableTime / cycleTime);

  // Effective capacity
  effectiveCapacity = theoreticalCapacity * efficiency;

  // Actual capacity (considering quality)
  actualCapacity = effectiveCapacity * qualityRate;
}

// Example
const capacity = calculateCapacity({
  availableTime: 8 * 3600,  // 8 hours in seconds
  cycleTime: 45,            // seconds per unit
  efficiency: 0.85,         // 85% efficiency
  qualityRate: 0.95         // 95% good parts
});

// Result:
// theoreticalCapacity = 640 units
// effectiveCapacity = 544 units
// actualCapacity = 517 units
```

### 9.3 Dynamic Scheduling

#### 9.3.1 Real-Time Rescheduling

```typescript
interface DynamicScheduler {
  // Monitor production status
  monitorProduction(): Promise<ProductionStatus>;

  // Detect deviations
  detectDeviations(status: ProductionStatus, schedule: Schedule): Deviation[];

  // Reschedule on disruptions
  reschedule(
    currentSchedule: Schedule,
    disruption: Disruption,
    strategy: 'reactive' | 'proactive'
  ): Promise<Schedule>;
}

interface Disruption {
  type: 'machine-breakdown' | 'quality-issue' | 'material-shortage' | 'rush-order';
  affectedResource: string;
  estimatedDuration: number;  // seconds
  severity: 'low' | 'medium' | 'high';
}
```

---

## 10. OEE Monitoring

### 10.1 OEE Calculation

#### 10.1.1 OEE Formula

```
OEE = Availability × Performance × Quality

Where:
Availability = Operating Time / Planned Production Time
Performance = (Ideal Cycle Time × Total Count) / Operating Time
Quality = Good Count / Total Count
```

#### 10.1.2 Detailed Calculations

**Availability:**
```typescript
interface AvailabilityCalculation {
  plannedProductionTime: number;  // seconds
  downtime: {
    breakdowns: number;
    setupChangeover: number;
    plannedMaintenance: number;
    unplannedStops: number;
  };

  // Operating Time = Planned Production Time - Downtime
  operatingTime = plannedProductionTime - totalDowntime;

  // Availability
  availability = operatingTime / plannedProductionTime;
}
```

**Performance:**
```typescript
interface PerformanceCalculation {
  idealCycleTime: number;  // seconds per unit
  totalCount: number;      // units produced
  operatingTime: number;   // seconds

  // Theoretical output
  theoreticalOutput = operatingTime / idealCycleTime;

  // Performance
  performance = totalCount / theoreticalOutput;

  // Alternative calculation
  performanceAlt = (idealCycleTime * totalCount) / operatingTime;
}
```

**Quality:**
```typescript
interface QualityCalculation {
  totalCount: number;     // Total units produced
  defectCount: number;    // Defective units
  reworkCount: number;    // Units requiring rework

  // Good count
  goodCount = totalCount - defectCount - reworkCount;

  // Quality rate
  quality = goodCount / totalCount;
}
```

### 10.2 Six Big Losses

#### 10.2.1 Loss Categories

**Availability Losses:**
1. **Equipment Failure**: Unplanned breakdowns
2. **Setup and Adjustments**: Changeover time

**Performance Losses:**
3. **Idling and Minor Stops**: Short stops (<5 minutes)
4. **Reduced Speed**: Running below ideal speed

**Quality Losses:**
5. **Process Defects**: Scrap and rework during stable production
6. **Reduced Yield**: Startup losses, warm-up defects

#### 10.2.2 Loss Tracking

```typescript
interface LossTracking {
  lineId: string;
  date: Date;
  shift: string;

  losses: {
    equipmentFailure: {
      incidents: { startTime: Date; duration: number; reason: string }[];
      totalTime: number;
      impactOnOEE: number;
    };

    setupAdjustment: {
      changeovers: { startTime: Date; duration: number; fromProduct: string; toProduct: string }[];
      totalTime: number;
      impactOnOEE: number;
    };

    minorStops: {
      stops: { time: Date; duration: number; reason: string }[];
      count: number;
      totalTime: number;
      impactOnOEE: number;
    };

    reducedSpeed: {
      duration: number;
      speedReduction: number;  // %
      unitsLost: number;
      impactOnOEE: number;
    };

    processDefects: {
      defectCount: number;
      defectTypes: { type: string; count: number }[];
      impactOnOEE: number;
    };

    reducedYield: {
      startupUnits: number;
      startupRejects: number;
      impactOnOEE: number;
    };
  };
}
```

### 10.3 Real-Time OEE Monitoring

#### 10.3.1 Data Collection

```typescript
interface OEEDataCollector {
  // Collect production data
  collectData(lineId: string, interval: number): Promise<ProductionData>;

  // Calculate real-time OEE
  calculateRealtimeOEE(data: ProductionData): OEEMetrics;

  // Generate alerts
  checkThresholds(oee: OEEMetrics, thresholds: OEEThresholds): Alert[];
}

interface ProductionData {
  timestamp: Date;
  lineId: string;
  actualOutput: number;
  targetOutput: number;
  goodUnits: number;
  defectiveUnits: number;
  downtime: number;
  cycleTime: number;
  idealCycleTime: number;
}

interface OEEMetrics {
  overall: number;
  availability: number;
  performance: number;
  quality: number;
  losses: LossBreakdown;
}
```

#### 10.3.2 OEE Dashboard

```json
{
  "oeeDashboard": {
    "facilityId": "FAC-001",
    "timestamp": "2025-12-27T10:30:00Z",
    "lines": [
      {
        "lineId": "LINE-A",
        "status": "running",
        "currentOEE": 84.2,
        "availability": 91.2,
        "performance": 95.4,
        "quality": 96.7,
        "trend": "improving",
        "alerts": []
      },
      {
        "lineId": "LINE-B",
        "status": "maintenance",
        "currentOEE": 0,
        "availability": 0,
        "performance": 0,
        "quality": 0,
        "trend": "stable",
        "alerts": [
          {
            "severity": "info",
            "message": "Planned maintenance in progress"
          }
        ]
      }
    ],
    "facilityOEE": 72.8,
    "worldClassTarget": 85.0
  }
}
```

---

## 11. Safety Interlocks

### 11.1 Safety System Architecture

#### 11.1.1 Safety Integrity Levels (SIL)

**IEC 61508 SIL Levels:**
```
SIL 4: 10⁻⁵ to 10⁻⁴ (highest safety, rarely used in manufacturing)
SIL 3: 10⁻⁴ to 10⁻³ (high safety, critical machinery)
SIL 2: 10⁻³ to 10⁻² (medium safety, standard industrial)
SIL 1: 10⁻² to 10⁻¹ (basic safety)
```

**ISO 13849-1 Performance Levels:**
```
PLe: Highest (equivalent to SIL 3)
PLd: High (equivalent to SIL 2)
PLc: Medium
PLb: Low
PLa: Lowest
```

#### 11.1.2 Safety Architecture

```
┌─────────────────────────────────────┐
│  Safety PLC (Dual-Channel)          │
│  - Category 4, PLe                  │
│  - SIL 3                             │
│  - Redundant processors              │
│  - Self-monitoring                   │
└───────────┬─────────────────────────┘
            │
    ┌───────┴───────┐
    │               │
┌───▼────┐    ┌────▼────┐
│Safety  │    │Safety   │
│Input   │    │Output   │
│Module  │    │Module   │
└───┬────┘    └────┬────┘
    │              │
    │              │
┌───▼──────────────▼────────┐
│  Safety Devices            │
│  - E-stops                 │
│  - Light curtains          │
│  - Safety gates            │
│  - Safety mats             │
│  - Safety relays           │
└────────────────────────────┘
```

### 11.2 Safety Devices

#### 11.2.1 Emergency Stop (E-Stop)

```typescript
interface EmergencyStop {
  deviceId: string;
  type: 'mushroom-button' | 'rope-pull' | 'wireless';
  category: 0 | 1 | 2;  // EN ISO 13850
  color: 'red';
  actuationMethod: 'push-pull' | 'twist-release';
  contacts: {
    normallyOpen: number;
    normallyClosed: number;
    redundant: boolean;
  };

  // E-stop action
  action: {
    stopCategory: 0 | 1;  // 0=uncontrolled, 1=controlled
    powerRemoval: boolean;
    brakingRequired: boolean;
    resetRequired: boolean;
  };
}
```

#### 11.2.2 Safety Light Curtains

```typescript
interface SafetyLightCurtain {
  deviceId: string;
  type: 'type-2' | 'type-4';  // IEC 61496
  resolution: number;  // mm
  protectedHeight: number;  // mm
  range: number;  // mm
  responseTime: number;  // ms

  // Detection capability
  detectionCapability: {
    fingerDetection: boolean;  // 14mm
    handDetection: boolean;    // 30mm
    bodyDetection: boolean;    // 70mm
  };

  // Safety functions
  muting: {
    enabled: boolean;
    sensors: string[];
    conditions: string[];
  };

  blanking: {
    enabled: boolean;
    zones: { start: number; end: number }[];
  };
}
```

#### 11.2.3 Safety Mats

```typescript
interface SafetyMat {
  deviceId: string;
  dimensions: { length: number; width: number };  // mm
  switchingForce: number;  // kg
  responseTime: number;  // ms
  connections: 'series' | 'parallel';

  // Placement
  location: {
    zone: string;
    position: { x: number; y: number };
  };
}
```

### 11.3 Safety Zones

#### 11.3.1 Zone Classification

```typescript
interface SafetyZone {
  zoneId: string;
  type: 'restricted' | 'limited' | 'collaborative';

  // Access control
  accessControl: {
    method: 'interlock-gate' | 'light-curtain' | 'safety-mat';
    devices: string[];
  };

  // Zone boundaries
  boundaries: {
    vertices: { x: number; y: number; z: number }[];
  };

  // Response actions
  onViolation: {
    action: 'immediate-stop' | 'controlled-stop' | 'speed-reduction';
    stopTime: number;  // ms
    notification: boolean;
    alarm: boolean;
  };

  // Collaborative mode
  collaborative: {
    enabled: boolean;
    maxSpeed: number;  // mm/s
    maxForce: number;  // N
    powerAndForceMonitoring: boolean;
  };
}
```

#### 11.3.2 Safety Distance Calculation

```typescript
// Calculate minimum safety distance for light curtain
function calculateSafetyDistance(params: {
  responseTime: number;  // ms (system + device)
  approachSpeed: number;  // mm/s (1600 mm/s typical hand speed)
  resolution: number;     // mm (light curtain resolution)
  additionalDistance: number;  // mm (penetration depth)
}): number {
  // ISO 13855 formula
  const K = params.approachSpeed;  // mm/s
  const T = params.responseTime / 1000;  // convert to seconds
  const C = params.additionalDistance;
  const D_pf = params.resolution - 14;  // Detection capability factor

  const safetyDistance = (K * T) + C + D_pf;

  return Math.max(safetyDistance, 100);  // Minimum 100mm
}

// Example
const distance = calculateSafetyDistance({
  responseTime: 50,      // 50ms
  approachSpeed: 1600,   // 1600 mm/s
  resolution: 30,        // 30mm light curtain
  additionalDistance: 850  // 850mm penetration depth
});
// Result: ~1000mm minimum safety distance
```

### 11.4 Safety Functions

#### 11.4.1 Safe Torque Off (STO)

```typescript
interface SafeTorqueOff {
  deviceId: string;
  drives: string[];  // Associated motor drives
  responseTime: number;  // ms
  safetyIntegrityLevel: 'SIL2' | 'SIL3';
  performanceLevel: 'PLd' | 'PLe';

  // Activation sources
  triggers: {
    emergencyStop: boolean;
    safetyGate: boolean;
    lightCurtain: boolean;
    safetyPLC: boolean;
  };
}
```

#### 11.4.2 Safe Limited Speed (SLS)

```typescript
interface SafeLimitedSpeed {
  deviceId: string;
  robot: string;
  speedLimits: {
    normal: number;      // mm/s
    collaborative: number;  // mm/s (typically 250 mm/s)
    maintenance: number;  // mm/s (typically 50 mm/s)
  };
  monitoring: {
    encoders: string[];
    redundancy: 'dual-channel';
    crossCheck: boolean;
  };
}
```

#### 11.4.3 Safe Standstill (SS1, SS2)

```typescript
interface SafeStandstill {
  deviceId: string;
  type: 'SS1' | 'SS2';
  // SS1: Controlled stop, then STO
  // SS2: Controlled stop, then safe operating stop

  monitoringMethod: {
    encoder: boolean;
    proximity: boolean;
    redundant: boolean;
  };

  standstillTolerance: number;  // mm or degrees
  responseTime: number;  // ms
}
```

### 11.5 Safety Risk Assessment

#### 11.5.1 Risk Evaluation

```typescript
interface RiskAssessment {
  hazardId: string;
  description: string;

  // Risk parameters
  severity: 1 | 2 | 3 | 4;  // Death/serious injury to slight injury
  frequency: 1 | 2 | 3 | 4 | 5;  // Rare to continuous
  probability: 1 | 2 | 3 | 4 | 5;  // Negligible to very high

  // Risk calculation (ISO 12100)
  riskLevel = severity * frequency * probability;

  // Required safety measures
  requiredPL: 'PLa' | 'PLb' | 'PLc' | 'PLd' | 'PLe';
  requiredSIL: 'SIL1' | 'SIL2' | 'SIL3';

  // Mitigation measures
  safeguards: {
    type: 'elimination' | 'substitution' | 'engineering' | 'administrative' | 'PPE';
    description: string;
    implemented: boolean;
  }[];

  // Residual risk
  residualRisk: {
    severity: number;
    frequency: number;
    probability: number;
    level: number;
    acceptable: boolean;
  };
}
```

---

## 12. Communication Protocols

### 12.1 Industrial Ethernet

#### 12.1.1 PROFINET

**Configuration:**
```json
{
  "profinet": {
    "networkType": "PROFINET-IO",
    "ioController": "192.168.1.100",
    "devices": [
      {
        "deviceName": "PLC-S7-1500",
        "ipAddress": "192.168.1.101",
        "macAddress": "00:1B:1B:7A:2C:3D",
        "deviceType": "controller",
        "cycleTime": 1
      },
      {
        "deviceName": "ROBOT-01",
        "ipAddress": "192.168.1.102",
        "macAddress": "00:1B:1B:7A:2C:4E",
        "deviceType": "io-device",
        "cycleTime": 4
      }
    ],
    "realTime": {
      "enabled": true,
      "class": "RT-IRT",
      "jitter": 1
    }
  }
}
```

#### 12.1.2 EtherNet/IP

**CIP Object Model:**
```typescript
interface CIPObject {
  classId: number;
  instanceId: number;
  attributes: {
    attributeId: number;
    dataType: string;
    value: any;
  }[];
  services: {
    serviceCode: number;
    serviceName: string;
  }[];
}

// Example: Motor Control Object (Class 0x25)
const motorControl: CIPObject = {
  classId: 0x25,
  instanceId: 1,
  attributes: [
    { attributeId: 1, dataType: 'UINT', value: 3000 },  // RPM
    { attributeId: 2, dataType: 'REAL', value: 75.5 },  // Torque
    { attributeId: 3, dataType: 'BOOL', value: true }   // Running
  ],
  services: [
    { serviceCode: 0x0E, serviceName: 'Get_Attribute_Single' },
    { serviceCode: 0x10, serviceName: 'Set_Attribute_Single' }
  ]
};
```

### 12.2 Fieldbus Protocols

#### 12.2.1 CAN/CANopen

**CAN Message:**
```typescript
interface CANMessage {
  cobId: number;  // 11-bit or 29-bit identifier
  rtr: boolean;   // Remote Transmission Request
  data: Uint8Array;  // 0-8 bytes
  dlc: number;    // Data Length Code
}

// CANopen PDO (Process Data Object)
interface PDO {
  cobId: number;  // 0x180-0x57F (TPDO), 0x200-0x4FF (RPDO)
  transmissionType: 0 | 1 | 254 | 255;  // Async, Sync, Event, Device
  inhibitTime: number;  // microseconds
  eventTimer: number;   // milliseconds
  mapping: {
    objectIndex: number;
    subIndex: number;
    length: number;  // bits
  }[];
}
```

### 12.3 Time-Sensitive Networking (TSN)

#### 12.3.1 TSN Configuration

```typescript
interface TSNConfiguration {
  // IEEE 802.1AS - Time synchronization
  timeSynchronization: {
    enabled: true;
    grandmaster: '192.168.1.1';
    accuracy: 1;  // microseconds
  };

  // IEEE 802.1Qbv - Time-Aware Shaper
  timeAwareShaper: {
    enabled: true;
    gateControlList: {
      gateStates: number[];  // Binary: 0=closed, 1=open per priority
      timeInterval: number;  // nanoseconds
    }[];
    cycleTime: 1000000;  // nanoseconds (1ms)
  };

  // IEEE 802.1Qbu - Frame Preemption
  framePreemption: {
    enabled: true;
    expressQueues: [7, 6];  // High-priority queues
    preemptableQueues: [5, 4, 3, 2, 1, 0];
  };

  // IEEE 802.1CB - Frame Replication and Elimination
  seamlessRedundancy: {
    enabled: true;
    paths: ['path-A', 'path-B'];
  };
}
```

---

## 13. Data Models

### 13.1 Production Data Model

```typescript
interface ProductionOrder {
  orderId: string;
  product: {
    productId: string;
    name: string;
    version: string;
    bom: BillOfMaterials;
    routing: ProductionRouting;
  };
  quantity: number;
  priority: number;
  dueDate: Date;
  status: 'planned' | 'released' | 'in-progress' | 'completed' | 'cancelled';

  // Execution details
  execution: {
    lineId: string;
    startTime: Date;
    endTime?: Date;
    actualQuantity: number;
    goodQuantity: number;
    scrapQuantity: number;
    reworkQuantity: number;
  };

  // Material consumption
  materialUsage: {
    materialId: string;
    plannedQuantity: number;
    actualQuantity: number;
    unit: string;
  }[];

  // Quality records
  qualityChecks: {
    checkId: string;
    timestamp: Date;
    result: 'pass' | 'fail';
    measurements: any;
  }[];
}

interface BillOfMaterials {
  items: {
    itemId: string;
    description: string;
    quantity: number;
    unit: string;
    level: number;  // BOM level (0=top)
    childItems: string[];
  }[];
}

interface ProductionRouting {
  operations: {
    operationId: string;
    sequence: number;
    workstation: string;
    description: string;
    standardTime: number;  // seconds
    setupTime: number;     // seconds
    resources: {
      resourceId: string;
      quantity: number;
    }[];
  }[];
}
```

### 13.2 Equipment Data Model

```typescript
interface Equipment {
  equipmentId: string;
  name: string;
  type: string;
  manufacturer: string;
  model: string;
  serialNumber: string;
  commissionDate: Date;

  // Location
  location: {
    facilityId: string;
    lineId: string;
    workstationId: string;
    position: { x: number; y: number; z: number };
  };

  // Status
  status: {
    operational: 'running' | 'idle' | 'maintenance' | 'error' | 'offline';
    alarms: Alarm[];
    runtime: number;  // seconds
    cycleCount: number;
  };

  // Maintenance
  maintenance: {
    lastPM: Date;
    nextPM: Date;
    pmInterval: number;  // hours
    mtbf: number;  // hours (Mean Time Between Failures)
    mttr: number;  // hours (Mean Time To Repair)
    history: MaintenanceRecord[];
  };

  // Performance
  performance: {
    oee: number;
    availability: number;
    performance: number;
    quality: number;
    targetCycleTime: number;
    actualCycleTime: number;
  };

  // Sensors
  sensors: {
    sensorId: string;
    type: string;
    value: number;
    unit: string;
    timestamp: Date;
  }[];
}

interface MaintenanceRecord {
  recordId: string;
  date: Date;
  type: 'preventive' | 'corrective' | 'predictive';
  description: string;
  partsReplaced: string[];
  labor: {
    technician: string;
    hours: number;
  }[];
  cost: number;
  downtime: number;  // seconds
}
```

---

## 14. Security and Access Control

### 14.1 Industrial Cybersecurity

#### 14.1.1 Defense-in-Depth

```
Layer 1: Physical Security
  - Locked cabinets
  - Facility access control

Layer 2: Network Segmentation
  - Firewall between OT and IT
  - VLANs for production zones
  - DMZ for HMI/SCADA

Layer 3: Access Control
  - Role-based access (RBAC)
  - Multi-factor authentication
  - Privileged access management

Layer 4: Application Security
  - Secure coding practices
  - Input validation
  - Encryption

Layer 5: Monitoring and Response
  - Intrusion detection (IDS)
  - Security information and event management (SIEM)
  - Incident response plan
```

#### 14.1.2 Access Control

```typescript
interface AccessControl {
  // User authentication
  authenticate(username: string, password: string, mfa?: string): Promise<AuthToken>;

  // Authorization
  authorize(user: User, resource: string, action: string): Promise<boolean>;

  // Audit logging
  logAccess(user: User, resource: string, action: string, result: boolean): Promise<void>;
}

interface User {
  userId: string;
  username: string;
  roles: Role[];
  permissions: Permission[];
}

interface Role {
  roleId: string;
  name: string;
  permissions: Permission[];
}

interface Permission {
  resource: string;  // e.g., "production-line"
  actions: ('read' | 'write' | 'execute' | 'delete')[];
}
```

---

## 15. Implementation Guidelines

### 15.1 Deployment Architecture

```
┌─────────────────────────────────────────┐
│  Enterprise Layer (Cloud/On-Prem)      │
│  - ERP                                  │
│  - Data Analytics                       │
│  - Machine Learning                     │
└──────────────┬──────────────────────────┘
               │ HTTPS/REST
┌──────────────▼──────────────────────────┐
│  Manufacturing Execution (MES)          │
│  - Production scheduling                │
│  - Quality management                   │
│  - Inventory management                 │
└──────────────┬──────────────────────────┘
               │ OPC UA / MQTT
┌──────────────▼──────────────────────────┐
│  Supervisory Control (SCADA)            │
│  - Real-time monitoring                 │
│  - Alarm management                     │
│  - Historian                            │
└──────────────┬──────────────────────────┘
               │ PROFINET / EtherNet/IP
┌──────────────▼──────────────────────────┐
│  Control Layer (PLC/PAC)                │
│  - Line control                         │
│  - Safety interlocks                    │
│  - Device coordination                  │
└──────────────┬──────────────────────────┘
               │ I/O, Fieldbus
┌──────────────▼──────────────────────────┐
│  Field Devices                          │
│  - Robots, conveyors, sensors           │
└─────────────────────────────────────────┘
```

### 15.2 Best Practices

#### 15.2.1 Design Principles

1. **Modularity**: Design systems with interchangeable components
2. **Standardization**: Use industry standards for protocols and interfaces
3. **Scalability**: Plan for future expansion
4. **Resilience**: Implement redundancy for critical systems
5. **Maintainability**: Design for easy troubleshooting and repair
6. **Documentation**: Comprehensive documentation from day one
7. **Testing**: Rigorous testing before production deployment
8. **Training**: Thorough operator and maintenance training

#### 15.2.2 Performance Optimization

```typescript
// Example: Optimize conveyor speed for throughput
function optimizeConveyorSpeed(params: {
  lineLength: number;        // mm
  partLength: number;        // mm
  partSpacing: number;       // mm
  targetThroughput: number;  // parts/hour
  maxSpeed: number;          // m/s
}): number {
  const pitchLength = params.partLength + params.partSpacing;
  const partsPerSecond = params.targetThroughput / 3600;
  const requiredSpeed = (pitchLength * partsPerSecond) / 1000;  // m/s

  return Math.min(requiredSpeed, params.maxSpeed);
}
```

---

## 16. References

### 16.1 Standards and Specifications

**Industrial Automation:**
- IEC 61131: Programmable controllers
- IEC 61499: Function blocks for industrial process control
- IEC 62264: Enterprise-control system integration (ANSI/ISA-95)
- IEC 62541: OPC Unified Architecture

**Safety:**
- ISO 12100: Safety of machinery - General principles
- ISO 13849-1: Safety-related parts of control systems
- ISO 13850: Emergency stop function
- IEC 61508: Functional safety
- IEC 62061: Safety of machinery - Functional safety

**Robotics:**
- ISO 10218: Robots and robotic devices - Safety requirements
- ISO/TS 15066: Collaborative robots
- ISO 9283: Manipulating industrial robots - Performance criteria

**Communication:**
- IEC 61158: Fieldbus for industrial control systems
- IEC 61784: Industrial communication networks
- IEEE 802.1: Time-Sensitive Networking (TSN)
- OPC Foundation: OPC UA specifications

**Quality:**
- ISO 9001: Quality management systems
- ISO/TS 16949: Automotive quality management (now IATF 16949)
- Six Sigma methodologies

### 16.2 Further Reading

- MESA International: Manufacturing Operations Management
- VDMA: German Engineering Federation publications
- Rockwell Automation: The Connected Enterprise
- Siemens: Totally Integrated Automation
- Industry 4.0 Working Group: Reference Architecture Model

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
