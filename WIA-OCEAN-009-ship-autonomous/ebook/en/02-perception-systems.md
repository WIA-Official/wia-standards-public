# Chapter 2: Perception Systems - The Eyes and Ears of Autonomous Ships

## Building Environmental Awareness

An autonomous ship must perceive its environment with greater reliability than human watchkeepers. While a human navigator uses eyes, ears, and intuition, autonomous vessels rely on multiple sensor systems that work together to create a comprehensive understanding of surrounding conditions.

### The Sensor Suite

Modern autonomous ships integrate six primary sensor categories:

#### Marine Radar Systems

Radar (Radio Detection and Ranging) remains the backbone of maritime perception, having proven reliable since the 1940s.

**X-Band Radar (9-10 GHz):**
- **Primary use:** High-resolution target detection
- **Range:** Up to 48 nautical miles
- **Accuracy:** 10-20 meters at close range
- **Update rate:** 2-3 seconds per rotation
- **Weather:** Affected by rain/fog (higher attenuation)
- **Target detection:** Small boats, buoys, coastline detail

**S-Band Radar (2-4 GHz):**
- **Primary use:** Long-range surveillance
- **Range:** 96+ nautical miles
- **Accuracy:** 50-100 meters
- **Update rate:** 2-4 seconds per rotation
- **Weather:** Better rain/fog penetration
- **Target detection:** Large ships, weather systems, land masses

**Modern Solid-State Radar:**
- No magnetron (traditional component requiring warmup)
- Instant-on capability
- 50,000+ hour lifespan (vs. 3,000 for magnetron)
- Lower power consumption
- Multiple simultaneous beams
- Better clutter suppression

**Radar Processing:**
```typescript
interface RadarSystem {
  scan(): RadarSweep;
  trackTargets(sweep: RadarSweep, history: RadarTrack[]): RadarTrack[];
  calculateCPA(track: RadarTrack, ownShip: ShipState): CPACalculation;
  assessCollisionRisk(cpa: CPACalculation): CollisionRisk;
}

interface RadarSweep {
  timestamp: Date;
  targets: RadarTarget[];
  clutter: ClutterMap;
  performanceMetrics: {
    antennaRotation: number;    // RPM
    transmitPower: number;       // Kilowatts
    receiverSensitivity: number; // dBm
  };
}

interface RadarTarget {
  range: number;           // Nautical miles
  bearing: number;         // Degrees true
  radialVelocity: number;  // Knots (Doppler)
  strength: number;        // dBm (signal strength)
  size: {
    length: number;        // Meters (estimated)
    width: number;
  };
  classification: "ship" | "land" | "weather" | "clutter" | "unknown";
}

interface RadarTrack {
  id: string;
  position: GeoPosition;
  course: number;          // Degrees true
  speed: number;           // Knots
  history: TrackPoint[];
  confidence: number;      // 0-1
  aisCorrelation?: AISTarget;  // If matched with AIS
}

interface CPACalculation {
  closestPointOfApproach: number;  // Nautical miles
  timeToClosestApproach: number;   // Minutes
  bearingAtCPA: number;            // Degrees relative
  actionRequired: boolean;
  colregsApplicable: "overtaking" | "head_on" | "crossing" | "none";
}
```

**ARPA (Automatic Radar Plotting Aid):**
Modern radar includes ARPA functionality:
- Tracks 200+ targets simultaneously
- Calculates CPA (Closest Point of Approach) for each
- Predicts collision risks
- Displays tracks overlaid on radar image
- Alerts when safe distance violated

**Autonomous Enhancement:**
```typescript
class AutonomousARPA {
  private readonly SAFE_DISTANCE_MILES = 2.0;
  private readonly CPA_ALERT_THRESHOLD = 1.0;  // Nautical miles
  private readonly TCPA_ALERT_THRESHOLD = 20;  // Minutes

  assessAllTargets(tracks: RadarTrack[], ownShip: ShipState): ThreatAssessment[] {
    return tracks
      .map(track => this.assessTarget(track, ownShip))
      .filter(assessment => assessment.threatLevel !== "none")
      .sort((a, b) => b.priority - a.priority);
  }

  private assessTarget(track: RadarTrack, ownShip: ShipState): ThreatAssessment {
    const cpa = this.calculateCPA(track, ownShip);

    // Determine threat level
    let threatLevel: "none" | "monitor" | "caution" | "warning" | "critical";

    if (cpa.closestPointOfApproach < 0.5 && cpa.timeToClosestApproach < 10) {
      threatLevel = "critical";
    } else if (cpa.closestPointOfApproach < 1.0 && cpa.timeToClosestApproach < 20) {
      threatLevel = "warning";
    } else if (cpa.closestPointOfApproach < 2.0 && cpa.timeToClosestApproach < 30) {
      threatLevel = "caution";
    } else if (cpa.closestPointOfApproach < 5.0) {
      threatLevel = "monitor";
    } else {
      threatLevel = "none";
    }

    return {
      targetId: track.id,
      threatLevel,
      cpa,
      colregsStatus: this.determineColregsStatus(track, ownShip),
      recommendedAction: this.recommendAction(threatLevel, cpa, track, ownShip),
      priority: this.calculatePriority(threatLevel, cpa)
    };
  }
}
```

#### Lidar (Light Detection and Ranging)

Lidar provides high-resolution 3D mapping of the ship's surroundings using laser pulses.

**Marine Lidar Specifications:**
- **Range:** 200-500 meters typical for marine applications
- **Accuracy:** ±2-5 centimeters
- **Update rate:** 10-20 Hz (full 360° scans)
- **Wavelength:** 905nm or 1550nm (infrared)
- **Points per second:** 300,000 - 1,000,000
- **Weather performance:** Reduced in heavy fog/rain

**Advantages over Radar:**
- Much higher resolution (centimeters vs. meters)
- Precise 3D geometry of obstacles
- Accurate distance to nearby objects
- Detects small objects (debris, small boats, buoys)
- Works well for harbor/port operations

**Lidar Data Processing:**
```typescript
interface LidarSystem {
  scan(): PointCloud;
  segmentObjects(cloud: PointCloud): SegmentedObject[];
  classifyObjects(segments: SegmentedObject[]): ClassifiedObject[];
  trackObjects(objects: ClassifiedObject[], history: ObjectTrack[]): ObjectTrack[];
}

interface PointCloud {
  timestamp: Date;
  points: LidarPoint[];
  sensorPosition: Position3D;
  sensorOrientation: Orientation3D;
}

interface LidarPoint {
  x: number;          // Meters from sensor
  y: number;
  z: number;
  intensity: number;  // Return signal strength
  distance: number;   // Meters
  azimuth: number;    // Degrees
  elevation: number;  // Degrees
}

interface SegmentedObject {
  points: LidarPoint[];
  centroid: Position3D;
  boundingBox: BoundingBox3D;
  volume: number;       // Cubic meters
  density: number;      // Points per cubic meter
}

interface ClassifiedObject {
  type: "vessel" | "buoy" | "pier" | "container" | "debris" | "unknown";
  confidence: number;
  position: Position3D;
  dimensions: { length: number; width: number; height: number };
  velocity?: Velocity3D;
  distance: number;
  bearing: number;
}
```

**Obstacle Detection Algorithm:**
```typescript
class LidarObstacleDetector {
  private readonly MIN_OBJECT_POINTS = 50;
  private readonly MAX_OBJECT_DISTANCE = 300;  // Meters

  detectObstacles(pointCloud: PointCloud): Obstacle[] {
    // 1. Remove ground plane (sea surface)
    const seaLevel = this.estimateSeaLevel(pointCloud);
    const filteredPoints = pointCloud.points.filter(
      p => p.z > seaLevel + 0.5  // 50cm above water
    );

    // 2. Cluster points into objects
    const clusters = this.dbscanClustering(filteredPoints, {
      epsilon: 0.5,      // 50cm max distance between points
      minPoints: this.MIN_OBJECT_POINTS
    });

    // 3. Classify each cluster
    const obstacles = clusters.map(cluster => {
      const bbox = this.calculateBoundingBox(cluster);
      const type = this.classifyCluster(cluster, bbox);

      return {
        type,
        position: this.calculateCentroid(cluster),
        boundingBox: bbox,
        distance: this.calculateDistance(cluster),
        threat: this.assessThreat(bbox, type)
      };
    });

    return obstacles.filter(o => o.distance < this.MAX_OBJECT_DISTANCE);
  }

  private classifyCluster(points: LidarPoint[], bbox: BoundingBox3D): string {
    const volume = bbox.length * bbox.width * bbox.height;
    const aspectRatio = bbox.length / bbox.width;

    // Ship: Large, elongated
    if (volume > 1000 && aspectRatio > 2.0) {
      return "vessel";
    }

    // Buoy: Small, roughly spherical
    if (volume < 10 && aspectRatio < 2.0 && bbox.height > 1.0) {
      return "buoy";
    }

    // Pier/dock: Very long, low
    if (bbox.length > 50 && bbox.height < 5) {
      return "pier";
    }

    return "unknown";
  }
}
```

#### Optical Camera Systems

Cameras provide visual confirmation and enable AI-powered object recognition.

**Camera Array Configuration:**
- **360° coverage:** 6-10 cameras around ship
- **Resolution:** 4K (3840×2160) minimum per camera
- **Frame rate:** 30 fps standard, 60 fps for critical areas
- **Field of view:** 80-120° per camera with 20% overlap
- **Dynamic range:** HDR for handling sun glare and shadows

**Camera Types:**

**Visible Light (RGB) Cameras:**
- Daytime primary perception
- Color information for flag identification, light colors
- Reads navigation light patterns (red/green/white)
- Captures AIS transponder displays on other ships

**Infrared (Thermal) Cameras:**
- Night vision without active illumination
- Detects heat signatures from ships, people
- Sees through light fog
- Range: 5-10 km for ship-sized targets

**Low-Light Cameras:**
- Starlight sensitivity (0.001 lux)
- Bridges gap between day and night cameras
- Color information in low light

**Computer Vision Processing:**
```typescript
interface VisionSystem {
  captureFrames(): CameraFrame[];
  detectObjects(frames: CameraFrame[]): VisualDetection[];
  recognizeNavigationLights(frame: CameraFrame): NavigationLight[];
  readNavigationMarks(frame: CameraFrame): NavigationMark[];
  assessVisibility(frames: CameraFrame[]): VisibilityConditions;
}

interface VisualDetection {
  camera: string;
  object: {
    type: "ship" | "buoy" | "marker" | "debris" | "bird" | "unknown";
    boundingBox: Rectangle;
    confidence: number;
    color?: string;
    text?: string;  // OCR from vessel names, buoy markings
  };
  estimated3DPosition?: Position3D;  // If fused with radar/lidar
}

interface NavigationLight {
  color: "red" | "green" | "white" | "yellow";
  pattern: "fixed" | "flashing" | "quick_flashing" | "group_flashing";
  bearing: number;
  elevation: number;
  flashPeriod?: number;  // Seconds
  vesselAspect?: "port" | "starboard" | "stern" | "masthead";
}
```

**AI Object Detection:**
```typescript
class MarineObjectDetector {
  private model: NeuralNetwork;  // YOLO v8 or similar

  async detectShips(frame: CameraFrame): Promise<ShipDetection[]> {
    // Run neural network inference
    const detections = await this.model.detect(frame.image);

    // Filter for ship detections
    const ships = detections
      .filter(d => d.class === "ship" || d.class === "vessel")
      .filter(d => d.confidence > 0.7);

    // Classify ship type using secondary model
    const classified = await Promise.all(
      ships.map(async ship => {
        const cropped = this.cropImage(frame.image, ship.boundingBox);
        const vesselType = await this.classifyVesselType(cropped);

        return {
          ...ship,
          vesselType,
          estimatedLength: this.estimateLengthFromImage(ship, frame.cameraParams),
          navigationStatus: this.analyzeNavigationStatus(cropped)
        };
      })
    );

    return classified;
  }

  private async classifyVesselType(image: ImageData): Promise<VesselType> {
    // Classify based on visual features
    const features = await this.extractFeatures(image);

    // Container ship: Stacked containers visible
    if (features.hasContainers && features.height > features.length * 0.3) {
      return { type: "container", confidence: 0.9 };
    }

    // Tanker: Low profile, cylindrical tanks
    if (features.hasCylindricalStructures && features.lowProfile) {
      return { type: "tanker", confidence: 0.85 };
    }

    // Fishing vessel: Outriggers, nets visible
    if (features.hasOutriggers || features.hasNets) {
      return { type: "fishing", confidence: 0.8 };
    }

    return { type: "unknown", confidence: 0.5 };
  }
}
```

#### AIS (Automatic Identification System)

AIS broadcasts ship identity, position, course, and speed via VHF radio.

**AIS Class A (Large Ships):**
- Updates every 2-10 seconds when moving
- Range: 20-40 nautical miles
- Mandatory for ships >300 GT
- Transmit power: 12.5W

**AIS Class B (Small Vessels):**
- Updates every 30 seconds
- Range: 5-10 nautical miles
- Lower transmit power: 2W
- Used by recreational boats, fishing vessels

**AIS Data Structure:**
```typescript
interface AISMessage {
  mmsi: number;              // Unique ship identifier (9 digits)
  messageType: number;       // 1-27 (different message types)
  timestamp: Date;

  // Position report (Messages 1, 2, 3)
  position?: {
    latitude: number;
    longitude: number;
    positionAccuracy: "high" | "low";  // <10m or >10m
    course: number;          // Degrees true (0-359)
    speed: number;           // Knots (0-102.2)
    heading: number;         // Degrees true (0-359)
    rateOfTurn?: number;     // Degrees/minute
    navigationStatus: NavigationStatus;
  };

  // Static data (Message 5)
  staticData?: {
    vesselName: string;
    callSign: string;
    imo: number;             // IMO ship number
    vesselType: VesselType;
    dimensions: {
      length: number;        // Meters
      width: number;         // Meters
      draft: number;         // Meters
      positionRefA: number;  // Dist from GPS to bow
      positionRefB: number;  // Dist from GPS to stern
      positionRefC: number;  // Dist from GPS to port
      positionRefD: number;  // Dist from GPS to starboard
    };
    destination?: string;
    eta?: Date;
  };
}

type NavigationStatus =
  | "under_way_engine"
  | "at_anchor"
  | "not_under_command"
  | "restricted_maneuverability"
  | "constrained_by_draft"
  | "moored"
  | "aground"
  | "fishing"
  | "under_way_sailing";

type VesselType =
  | "cargo"
  | "tanker"
  | "passenger"
  | "high_speed_craft"
  | "tug"
  | "pilot"
  | "search_rescue"
  | "fishing"
  | "sailing"
  | "pleasure_craft"
  | "other";
```

**AIS Integration:**
```typescript
class AISProcessor {
  private aisTargets: Map<number, AISTarget> = new Map();

  processMessage(message: AISMessage): void {
    const mmsi = message.mmsi;

    // Update or create target
    let target = this.aisTargets.get(mmsi);
    if (!target) {
      target = this.createNewTarget(message);
      this.aisTargets.set(mmsi, target);
    } else {
      target = this.updateTarget(target, message);
    }

    // Correlate with radar
    const radarTrack = this.findMatchingRadarTrack(target);
    if (radarTrack) {
      target.radarCorrelated = true;
      target.radarTrackId = radarTrack.id;
      // Use AIS for identity, radar for precise position
    }
  }

  fuseWithRadar(aisTarget: AISTarget, radarTrack: RadarTrack): FusedTarget {
    return {
      id: aisTarget.mmsi.toString(),
      identity: aisTarget.staticData,

      // Use radar for precise position (often more accurate)
      position: radarTrack.position,

      // Use AIS for course/speed (ship's own calculation)
      course: aisTarget.position.course,
      speed: aisTarget.position.speed,

      // Combined confidence
      confidence: Math.min(aisTarget.confidence, radarTrack.confidence),

      dataSource: "ais+radar",
      navigationStatus: aisTarget.position.navigationStatus
    };
  }
}
```

#### GPS/GNSS Navigation

Precise positioning using Global Navigation Satellite Systems.

**Multi-Constellation GNSS:**
- **GPS (USA):** 31 satellites, global coverage
- **GLONASS (Russia):** 24 satellites
- **Galileo (EU):** 26 satellites
- **BeiDou (China):** 35 satellites

**Positioning Accuracy:**
- Standard GPS: 5-10 meters
- Dual-frequency GPS: 1-2 meters
- SBAS (WAAS/EGNOS): 1-3 meters
- RTK (Real-Time Kinematic): 1-5 centimeters

**Autonomous Navigation Requirements:**
```typescript
interface GNSSSystem {
  getPosition(): GNSSPosition;
  validatePosition(position: GNSSPosition): PositionValidity;
  detectSpoofing(): SpoofingDetection;
  fuseWithINS(): IntegratedPosition;  // Inertial Navigation System
}

interface GNSSPosition {
  latitude: number;
  longitude: number;
  altitude: number;
  timestamp: Date;

  accuracy: {
    horizontal: number;  // Meters (95% confidence)
    vertical: number;
    velocity: number;
  };

  satellites: {
    gps: number;
    glonass: number;
    galileo: number;
    beidou: number;
    total: number;
  };

  dop: {
    hdop: number;  // Horizontal dilution of precision
    vdop: number;  // Vertical
    pdop: number;  // Position
  };

  fixQuality: "no_fix" | "gps" | "dgps" | "rtk_fixed" | "rtk_float";
}
```

**GPS Spoofing Detection:**
```typescript
class GNSSSpoofingDetector {
  detectSpoofing(gnss: GNSSPosition, ins: INSPosition, ais: AISPosition): SpoofingDetection {
    const checks: SpoofingCheck[] = [];

    // 1. Compare GNSS with inertial navigation
    const insDeviation = this.calculateDeviation(gnss, ins);
    if (insDeviation > 100) {  // 100 meters
      checks.push({
        type: "ins_mismatch",
        severity: "high",
        deviation: insDeviation
      });
    }

    // 2. Check for impossible speed changes
    const previousPosition = this.getLastPosition();
    const speed = this.calculateSpeed(previousPosition, gnss);
    if (speed > 50) {  // 50 knots impossible for cargo ship
      checks.push({
        type: "impossible_speed",
        severity: "critical",
        speed
      });
    }

    // 3. Validate against AIS position of nearby ships
    const aisInconsistency = this.checkAISConsistency(gnss, ais);
    if (aisInconsistency) {
      checks.push(aisInconsistency);
    }

    // 4. Check satellite geometry (spoofing often has unusual)
    if (gnss.dop.hdop > 5.0 && gnss.satellites.total > 12) {
      checks.push({
        type: "unusual_dop",
        severity: "medium",
        hdop: gnss.dop.hdop
      });
    }

    return {
      spoofed: checks.some(c => c.severity === "critical"),
      checks,
      confidence: this.calculateSpoofingConfidence(checks)
    };
  }
}
```

### Sensor Fusion

Individual sensors have limitations. Sensor fusion combines data from all sources to create robust environmental awareness.

**Fusion Architecture:**
```typescript
interface SensorFusionEngine {
  fuse(
    radar: RadarTrack[],
    lidar: ClassifiedObject[],
    vision: VisualDetection[],
    ais: AISTarget[],
    gnss: GNSSPosition
  ): FusedEnvironmentModel;
}

interface FusedEnvironmentModel {
  ownShip: {
    position: GNSSPosition;
    heading: number;
    speed: number;
    rateOfTurn: number;
  };

  vessels: FusedVessel[];
  obstacles: FusedObstacle[];
  navigationAids: NavigationAid[];

  environmentalConditions: {
    visibility: number;      // Meters
    seaState: number;        // 0-9 scale
    weather: WeatherCondition;
  };

  confidence: number;        // Overall model confidence
  timestamp: Date;
}

interface FusedVessel {
  id: string;

  // Identity (from AIS if available)
  identity?: {
    name: string;
    mmsi: number;
    type: VesselType;
    dimensions: Dimensions;
  };

  // Position (fused from all sources)
  position: GeoPosition;
  course: number;
  speed: number;

  // Data sources that contributed
  sources: {
    radar?: RadarTrack;
    lidar?: ClassifiedObject;
    vision?: VisualDetection;
    ais?: AISTarget;
  };

  // Fused confidence
  confidence: number;

  // Collision assessment
  cpa: CPACalculation;
  threat: ThreatLevel;
}
```

**Kalman Filter for Track Fusion:**
```typescript
class VesselTrackFusion {
  private kalmanFilters: Map<string, KalmanFilter> = new Map();

  fuseTarget(
    radar: RadarTrack,
    ais: AISTarget,
    vision: VisualDetection
  ): FusedVessel {
    const id = ais?.mmsi?.toString() || radar.id;

    // Get or create Kalman filter for this target
    let kf = this.kalmanFilters.get(id);
    if (!kf) {
      kf = new KalmanFilter({
        stateVariables: ["lat", "lon", "speed", "course"],
        processNoise: 0.1,
        measurementNoise: 1.0
      });
      this.kalmanFilters.set(id, kf);
    }

    // Prepare measurements from each sensor
    const measurements: Measurement[] = [];

    if (radar) {
      measurements.push({
        value: radar.position,
        uncertainty: 50,  // Radar ~50m accuracy
        weight: 0.4
      });
    }

    if (ais) {
      measurements.push({
        value: ais.position,
        uncertainty: 10,  // AIS ~10m accuracy
        weight: 0.5
      });
    }

    if (vision) {
      // Vision provides bearing but uncertain range
      measurements.push({
        value: this.calculatePositionFromBearing(vision),
        uncertainty: 200,
        weight: 0.1
      });
    }

    // Fuse measurements
    const fusedState = kf.update(measurements);

    return {
      id,
      identity: ais?.staticData,
      position: {
        latitude: fusedState.lat,
        longitude: fusedState.lon
      },
      course: fusedState.course,
      speed: fusedState.speed,
      sources: { radar, ais, vision },
      confidence: this.calculateConfidence(measurements, fusedState.residuals)
    };
  }
}
```

### Real-World Performance

**Yara Birkeland Sensor Suite:**
- 7× Furuno X-band radars
- 2× Furuno S-band radars
- 4× Velodyne lidar sensors
- 6× HD visual cameras
- 2× Thermal cameras
- 2× AIS receivers
- 3× GNSS receivers (triple-redundant)
- Weather sensors (wind, temp, pressure)

**Detection Performance:**
- Large ships (>100m): Detected at 20+ nautical miles (radar)
- Small boats (<10m): Detected at 2-5 nautical miles (radar + lidar)
- Buoys: Detected at 500 meters (lidar + vision)
- Debris: Detected at 100 meters (lidar)
- Docked vessels: Full 3D mapping (lidar)
- **Reliability:** 99.9% detection rate in testing

### Philosophy: 弘益人間

Comprehensive perception systems embody 弘益人間 by:
- Detecting hazards more reliably than human watchkeepers
- Never succumbing to fatigue or distraction
- Providing equal safety day and night, good weather and bad
- Enabling ships to operate in conditions unsafe for human crew
- Sharing perception data to improve safety for all vessels

---

**Next Chapter:** AI Decision-Making and Path Planning - how autonomous systems process sensor data to make navigation decisions.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
