# WIA-FARMING_ROBOT: PHASE 1 - Data Format Specification

**Version:** 1.0
**Status:** Active
**Last Updated:** 2026-01-12

---

## Overview

The WIA-FARMING_ROBOT standard defines data formats for agricultural robotics systems, enabling interoperability between autonomous tractors, harvesting robots, precision agriculture equipment, and farm management systems.

**Market Context:** Global agricultural robotics market projected at $26.35B by 2032, with autonomous systems becoming core elements of farm operations.

---

## Core Data Structures

### 1. Robot Identification

```typescript
interface RobotIdentification {
  robotId: string;              // Unique identifier (UUID v4)
  manufacturer: string;          // Equipment manufacturer
  model: string;                // Robot model
  type: RobotType;              // TRACTOR | HARVESTER | SPRAYER | SEEDER | SCOUT
  serialNumber: string;         // Manufacturing serial number
  certifications: string[];      // ISO 18497, FIRA compliance
  firmwareVersion: string;      // Current firmware version
  deploymentDate: ISO8601Date;  // Initial deployment date
}

enum RobotType {
  AUTONOMOUS_TRACTOR = "AUTONOMOUS_TRACTOR",
  HARVESTING_ROBOT = "HARVESTING_ROBOT",
  PRECISION_SPRAYER = "PRECISION_SPRAYER",
  SEEDING_ROBOT = "SEEDING_ROBOT",
  CROP_SCOUT = "CROP_SCOUT",
  WEEDING_ROBOT = "WEEDING_ROBOT",
  LIVESTOCK_MONITORING = "LIVESTOCK_MONITORING"
}
```

### 2. Positioning Data

```typescript
interface PositioningData {
  timestamp: ISO8601DateTime;
  gnss: GNSSData;               // Global Navigation Satellite System
  rtk: RTKData;                 // Real-Time Kinematic (±2.5cm accuracy)
  heading: number;              // Degrees (0-360)
  speed: number;                // Meters per second
  altitude: number;             // Meters above sea level
  accuracy: AccuracyMetrics;
}

interface GNSSData {
  latitude: number;             // Decimal degrees
  longitude: number;            // Decimal degrees
  satellites: number;           // Number of satellites in view
  hdop: number;                 // Horizontal Dilution of Precision
  quality: GNSSQuality;         // NO_FIX | 2D_FIX | 3D_FIX | RTK_FLOAT | RTK_FIXED
}

interface RTKData {
  enabled: boolean;
  baseStationId: string;
  correctionAge: number;        // Seconds since last correction
  positionAccuracy: number;     // Centimeters (target: ±2.5cm)
}

interface AccuracyMetrics {
  horizontal: number;           // Centimeters
  vertical: number;             // Centimeters
  confidence: number;           // Percentage (0-100)
}
```

### 3. Sensor Fusion Data

```typescript
interface SensorFusionData {
  timestamp: ISO8601DateTime;
  lidar: LiDARData;
  cameras: CameraData[];
  radar: RadarData;
  imu: IMUData;                 // Inertial Measurement Unit
  environmentalSensors: EnvironmentalData;
}

interface LiDARData {
  scanId: string;
  pointCloud: PointCloud3D;     // 3D mapping data
  obstacleDetection: Obstacle[];
  scanRate: number;             // Hz
  range: number;                // Meters
  resolution: number;           // Points per degree
}

interface PointCloud3D {
  points: Point3D[];
  format: "XYZ" | "XYZRGB" | "XYZI";  // Intensity or color
  coordinateSystem: "ROBOT" | "WORLD" | "FIELD";
}

interface Point3D {
  x: number;
  y: number;
  z: number;
  intensity?: number;           // Reflectivity (0-255)
  rgb?: [number, number, number];
}

interface Obstacle {
  id: string;
  type: ObstacleType;
  position: Point3D;
  boundingBox: BoundingBox3D;
  velocity: Vector3D;           // If moving
  confidence: number;           // 0-1
}

enum ObstacleType {
  STATIC = "STATIC",
  DYNAMIC = "DYNAMIC",
  HUMAN = "HUMAN",
  ANIMAL = "ANIMAL",
  VEHICLE = "VEHICLE",
  TREE = "TREE",
  ROCK = "ROCK",
  UNKNOWN = "UNKNOWN"
}

interface CameraData {
  cameraId: string;
  position: CameraPosition;     // FRONT | REAR | LEFT | RIGHT | TOP
  image: ImageData;
  detections: ObjectDetection[];
  cropAnalysis: CropAnalysis;
}

interface ObjectDetection {
  class: string;
  confidence: number;
  boundingBox: BoundingBox2D;
  segmentationMask?: Polygon[];
}

interface CropAnalysis {
  healthIndex: number;          // 0-100
  maturityLevel: number;        // 0-100
  diseaseDetection: DiseaseMarker[];
  weedDetection: WeedMarker[];
}
```

### 4. Operational Data

```typescript
interface OperationalData {
  timestamp: ISO8601DateTime;
  taskId: string;
  taskType: TaskType;
  status: OperationalStatus;
  area: GeoPolygon;             // Working area
  coverage: CoverageMetrics;
  performance: PerformanceMetrics;
}

enum TaskType {
  PLOWING = "PLOWING",
  SEEDING = "SEEDING",
  SPRAYING = "SPRAYING",
  HARVESTING = "HARVESTING",
  MONITORING = "MONITORING",
  WEEDING = "WEEDING",
  MAINTENANCE = "MAINTENANCE"
}

enum OperationalStatus {
  IDLE = "IDLE",
  NAVIGATING = "NAVIGATING",
  WORKING = "WORKING",
  PAUSED = "PAUSED",
  ERROR = "ERROR",
  CHARGING = "CHARGING",
  MAINTENANCE = "MAINTENANCE"
}

interface CoverageMetrics {
  totalArea: number;            // Square meters
  coveredArea: number;          // Square meters
  completionPercentage: number; // 0-100
  efficiency: number;           // Actual vs. planned speed
}

interface PerformanceMetrics {
  workRate: number;             // Hectares per hour
  fuelConsumption: number;      // Liters per hectare
  energyConsumption: number;    // kWh (for electric)
  seedingRate?: number;         // Seeds per square meter
  sprayingRate?: number;        // Liters per hectare
  harvestYield?: number;        // Kg per hectare
}
```

### 5. Path Planning Data

```typescript
interface PathPlanningData {
  planId: string;
  field: FieldDefinition;
  path: PathSegment[];
  waypoints: Waypoint[];
  constraints: PathConstraints;
}

interface FieldDefinition {
  fieldId: string;
  boundary: GeoPolygon;
  obstacles: GeoPolygon[];      // No-go zones
  entryPoints: GeoPoint[];
  exitPoints: GeoPoint[];
  slope: SlopeData;
  soilType: string;
}

interface PathSegment {
  segmentId: string;
  startPoint: GeoPoint;
  endPoint: GeoPoint;
  segmentType: "STRAIGHT" | "CURVE" | "TURN";
  speed: number;                // m/s
  heading: number;              // Degrees
  width: number;                // Working width in meters
}

interface Waypoint {
  position: GeoPoint;
  action: WaypointAction;
  timestamp: ISO8601DateTime;
  tolerance: number;            // Meters
}

enum WaypointAction {
  PASS = "PASS",
  STOP = "STOP",
  TURN = "TURN",
  START_WORK = "START_WORK",
  STOP_WORK = "STOP_WORK",
  ADJUST_SPEED = "ADJUST_SPEED"
}
```

---

## Data Exchange Formats

### Primary Format: JSON

```json
{
  "messageType": "ROBOT_TELEMETRY",
  "version": "1.0",
  "timestamp": "2026-01-12T10:30:00Z",
  "robot": {
    "robotId": "550e8400-e29b-41d4-a716-446655440000",
    "type": "AUTONOMOUS_TRACTOR"
  },
  "positioning": {
    "latitude": 42.3601,
    "longitude": -71.0589,
    "accuracy": 2.3
  },
  "status": "WORKING",
  "task": {
    "taskId": "task-2026-001",
    "type": "SEEDING",
    "completion": 67.5
  }
}
```

### Secondary Format: Protocol Buffers

For high-frequency, low-latency communication (sensor streams, real-time control).

### Tertiary Format: CSV

For bulk data export, historical records, and analytics.

---

## Data Quality Requirements

| Metric | Requirement |
|--------|-------------|
| **Positioning Accuracy** | ±2.5cm with RTK |
| **Update Rate** | 10 Hz minimum |
| **Timestamp Precision** | Millisecond resolution |
| **Data Completeness** | >99% of required fields |
| **Latency** | <100ms for critical data |

---

## References

- ISO 18497: Agricultural machinery and tractors - Safety of highly automated agricultural machines
- FIRA Standards (International Forum of Agricultural Robotics)
- GNSS RTK Positioning Standards
- LiDAR Point Cloud Data Format (ASTM E3125)

---

**弘益人間 (Benefit All Humanity)**
© 2026 WIA (World Industry Association)
