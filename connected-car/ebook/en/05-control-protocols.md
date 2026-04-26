# Chapter 5: Connected Car Control Protocols

## V2X Communication and Vehicle Control Systems

This chapter details the communication protocols that enable vehicle-to-everything (V2X) communication and remote vehicle control, forming the foundation for intelligent transportation systems.

---

## V2X Protocol Stack

### Protocol Architecture Overview

```typescript
// WIA Connected Car V2X Protocol Implementation
// Complete V2X Communication Stack

/**
 * V2X Protocol Stack Architecture
 * Layered approach following OSI model adaptations
 */
interface V2XProtocolStack {
  application: ApplicationLayer;
  facilities: FacilitiesLayer;
  networking: NetworkingLayer;
  access: AccessLayer;
  physical: PhysicalLayer;
  security: SecurityServices;
  management: ManagementServices;
}

/**
 * Application Layer - Safety and Non-Safety Applications
 */
interface ApplicationLayer {
  safetyApplications: SafetyApplication[];
  mobilityApplications: MobilityApplication[];
  informationApplications: InfoApplication[];
  commercialApplications: CommercialApplication[];
}

interface SafetyApplication {
  id: string;
  name: string;
  type: SafetyAppType;
  priority: ApplicationPriority;
  latencyRequirement: number;  // milliseconds
  reliabilityTarget: number;   // percentage
  messageTypes: string[];
}

type SafetyAppType =
  | "FORWARD_COLLISION_WARNING"
  | "INTERSECTION_MOVEMENT_ASSIST"
  | "BLIND_SPOT_WARNING"
  | "LANE_CHANGE_WARNING"
  | "DO_NOT_PASS_WARNING"
  | "EMERGENCY_VEHICLE_ALERT"
  | "QUEUE_WARNING"
  | "REDUCED_SPEED_ZONE_WARNING"
  | "CURVE_SPEED_WARNING"
  | "ROAD_HAZARD_WARNING";

type ApplicationPriority = "CRITICAL" | "HIGH" | "MEDIUM" | "LOW";

interface MobilityApplication {
  id: string;
  name: string;
  type: MobilityAppType;
  requirements: ApplicationRequirements;
}

type MobilityAppType =
  | "TRAFFIC_SIGNAL_PRIORITY"
  | "GREEN_LIGHT_OPTIMAL_SPEED"
  | "INTELLIGENT_INTERSECTION"
  | "COOPERATIVE_ADAPTIVE_CRUISE"
  | "PLATOONING"
  | "COOPERATIVE_MERGE"
  | "SPEED_HARMONIZATION";

/**
 * Facilities Layer - Message Handling and Processing
 */
interface FacilitiesLayer {
  messageHandler: MessageHandler;
  vehicleDataProvider: VehicleDataProvider;
  positionProvider: PositionProvider;
  timeProvider: TimeProvider;
  localDynamicMap: LocalDynamicMap;
}

interface MessageHandler {
  encoders: MessageEncoder[];
  decoders: MessageDecoder[];
  validators: MessageValidator[];
  routing: MessageRouter;
}

interface MessageEncoder {
  messageType: string;
  encoding: EncodingType;
  encode(message: any): Buffer;
}

interface MessageDecoder {
  messageType: string;
  encoding: EncodingType;
  decode(buffer: Buffer): any;
}

type EncodingType = "UPER" | "XER" | "JER" | "OER";

/**
 * Networking and Transport Layer
 */
interface NetworkingLayer {
  geoNetworking: GeoNetworking;
  btp: BasicTransportProtocol;
  ipv6: IPv6Adaptation;
}

interface GeoNetworking {
  type: "ETSI_ITS_GN";
  version: string;
  capabilities: GNCapabilities;
  configuration: GNConfiguration;
}

interface GNCapabilities {
  geoUnicast: boolean;
  geoBroadcast: boolean;
  geoAnycast: boolean;
  topoBroadcast: boolean;
  singleHop: boolean;
  locationService: boolean;
}

interface GNConfiguration {
  stationType: StationType;
  gnAddress: GNAddress;
  locationTable: LocationTableConfig;
  packetBuffer: PacketBufferConfig;
}

type StationType =
  | "UNKNOWN" | "PEDESTRIAN" | "CYCLIST" | "MOPED"
  | "MOTORCYCLE" | "PASSENGER_CAR" | "BUS" | "LIGHT_TRUCK"
  | "HEAVY_TRUCK" | "TRAILER" | "SPECIAL_VEHICLE" | "TRAM"
  | "ROAD_SIDE_UNIT";

interface GNAddress {
  manual: boolean;
  stationType: number;
  stationCountryCode: number;
  mid: Buffer;  // 6 bytes MAC-derived
}

/**
 * Access Layer - DSRC and C-V2X
 */
interface AccessLayer {
  dsrc: DSRCAccess;
  cV2X: CV2XAccess;
  multiAccess: MultiAccessCoordination;
}

interface DSRCAccess {
  standard: "IEEE_802_11P" | "ETSI_ITS_G5";
  channels: DSRCChannel[];
  txPower: number;  // dBm
  dataRate: number; // Mbps
  channelSwitching: ChannelSwitchingMode;
}

interface DSRCChannel {
  number: number;
  frequency: number;     // MHz
  bandwidth: number;     // MHz
  usage: ChannelUsage;
  priority: number;
}

type ChannelUsage =
  | "CONTROL" | "SERVICE" | "SAFETY" | "NON_SAFETY";

interface CV2XAccess {
  mode: "PC5" | "UU" | "DUAL";
  release: "REL14" | "REL15" | "REL16" | "REL17";
  configuration: CV2XConfiguration;
}

interface CV2XConfiguration {
  resourceAllocation: ResourceAllocationMode;
  slBandwidth: number;
  txPool: TransmissionPool;
  rxPool: ReceptionPool;
  syncConfig: SyncConfiguration;
}

type ResourceAllocationMode = "MODE_1" | "MODE_2" | "MODE_3" | "MODE_4";

/**
 * Physical Layer Configuration
 */
interface PhysicalLayer {
  antennaConfig: AntennaConfiguration;
  radioConfig: RadioConfiguration;
  gnssSensor: GNSSSensorConfig;
  inertialSensor: InertialSensorConfig;
}

interface AntennaConfiguration {
  type: AntennaType;
  gain: number;        // dBi
  placement: AntennaPlacement;
  diversity: boolean;
}

type AntennaType = "OMNIDIRECTIONAL" | "DIRECTIONAL" | "MIMO" | "PHASED_ARRAY";

interface AntennaPlacement {
  location: "ROOF" | "SIDE_MIRROR" | "BUMPER" | "INTERNAL";
  height: number;  // meters above ground
  orientation: number;  // degrees
}

/**
 * V2X Message Implementation - SAE J2735
 */
class V2XMessageService {
  private encoder: BSMEncoder;
  private decoder: BSMDecoder;
  private securityService: V2XSecurityService;
  private transmitter: V2XTransmitter;

  constructor(
    private config: V2XServiceConfig,
    private vehicleDataProvider: VehicleDataProvider
  ) {
    this.encoder = new BSMEncoder();
    this.decoder = new BSMDecoder();
    this.securityService = new V2XSecurityService(config.security);
    this.transmitter = new V2XTransmitter(config.transmit);
  }

  /**
   * Generate and transmit Basic Safety Message
   */
  async transmitBSM(): Promise<void> {
    // Collect vehicle data
    const vehicleData = await this.vehicleDataProvider.getCurrentState();

    // Build BSM Part I (Core Data)
    const bsmPart1: BSMCoreData = {
      msgCnt: this.getNextMessageCount(),
      id: this.config.temporaryId,
      secMark: this.getDSecond(),
      lat: this.encodeLat(vehicleData.position.latitude),
      long: this.encodeLong(vehicleData.position.longitude),
      elev: this.encodeElev(vehicleData.position.altitude),
      accuracy: this.encodeAccuracy(vehicleData.position.accuracy),
      transmission: this.encodeTransmission(vehicleData.transmission),
      speed: this.encodeSpeed(vehicleData.speed),
      heading: this.encodeHeading(vehicleData.heading),
      angle: this.encodeSteeringAngle(vehicleData.steeringAngle),
      accelSet: this.encodeAcceleration(vehicleData.acceleration),
      brakes: this.encodeBrakes(vehicleData.brakes),
      size: this.encodeSize(vehicleData.dimensions)
    };

    // Build BSM Part II (Optional Extensions)
    const bsmPart2 = this.buildBSMPart2(vehicleData);

    // Encode to UPER
    const encoded = this.encoder.encode({ coreData: bsmPart1, partII: bsmPart2 });

    // Sign message
    const signed = await this.securityService.signMessage(encoded);

    // Transmit
    await this.transmitter.broadcast(signed, {
      channel: this.config.transmitChannel,
      priority: "HIGH",
      rate: this.config.transmitRate
    });
  }

  /**
   * Process received V2X message
   */
  async processReceivedMessage(
    rawMessage: Buffer,
    metadata: ReceiveMetadata
  ): Promise<ProcessedV2XMessage> {
    // Verify signature
    const verified = await this.securityService.verifyMessage(rawMessage);
    if (!verified.valid) {
      throw new SecurityError("Message signature verification failed");
    }

    // Decode message
    const decoded = this.decoder.decode(verified.payload);

    // Validate plausibility
    const plausibility = this.validatePlausibility(decoded, metadata);
    if (!plausibility.valid) {
      console.warn("Plausibility check failed:", plausibility.reasons);
    }

    // Convert to internal representation
    return {
      type: decoded.messageType,
      content: this.convertToInternalFormat(decoded),
      source: {
        id: decoded.coreData?.id,
        certificate: verified.certificate,
        rssi: metadata.rssi,
        timestamp: metadata.timestamp
      },
      plausibility
    };
  }

  private encodeLat(lat: number): number {
    // Convert to 1/10 micro-degrees
    return Math.round(lat * 10000000);
  }

  private encodeLong(long: number): number {
    return Math.round(long * 10000000);
  }

  private encodeElev(altitude: number): number {
    // Convert to decimeters, offset by 4096
    return Math.round(altitude * 10) + 4096;
  }

  private encodeSpeed(speedKmh: number): number {
    // Convert km/h to 0.02 m/s units
    const speedMs = speedKmh / 3.6;
    return Math.round(speedMs / 0.02);
  }

  private encodeHeading(heading: number): number {
    // Convert degrees to 0.0125 degree units
    return Math.round(heading / 0.0125);
  }

  private getNextMessageCount(): number {
    this.messageCount = (this.messageCount + 1) % 128;
    return this.messageCount;
  }

  private getDSecond(): number {
    const now = new Date();
    return (now.getSeconds() * 1000) + now.getMilliseconds();
  }

  private messageCount = 0;

  // Stub methods for encoding
  private encodeAccuracy(accuracy: any): any { return accuracy; }
  private encodeTransmission(transmission: any): any { return transmission; }
  private encodeSteeringAngle(angle: number): any { return { value: angle }; }
  private encodeAcceleration(accel: any): any { return accel; }
  private encodeBrakes(brakes: any): any { return brakes; }
  private encodeSize(dimensions: any): any { return dimensions; }
  private buildBSMPart2(data: any): any { return null; }
  private validatePlausibility(msg: any, meta: any): any { return { valid: true }; }
  private convertToInternalFormat(msg: any): any { return msg; }
}

interface BSMCoreData {
  msgCnt: number;
  id: Buffer;
  secMark: number;
  lat: number;
  long: number;
  elev: number;
  accuracy: any;
  transmission: any;
  speed: number;
  heading: number;
  angle: any;
  accelSet: any;
  brakes: any;
  size: any;
}

interface V2XServiceConfig {
  temporaryId: Buffer;
  transmitChannel: number;
  transmitRate: number;
  security: SecurityConfig;
  transmit: TransmitConfig;
}

interface ReceiveMetadata {
  rssi: number;
  timestamp: Date;
  channel: number;
  dataRate: number;
}

interface ProcessedV2XMessage {
  type: string;
  content: any;
  source: MessageSource;
  plausibility: PlausibilityResult;
}

interface MessageSource {
  id: Buffer | undefined;
  certificate: any;
  rssi: number;
  timestamp: Date;
}

interface PlausibilityResult {
  valid: boolean;
  reasons?: string[];
}

class SecurityError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "SecurityError";
  }
}
```

---

## Signal Phase and Timing (SPaT)

```typescript
/**
 * SPaT Message Processing for Traffic Signal Communication
 * SAE J2735 Signal Phase and Timing Message
 */
interface SPaTMessage {
  timeStamp?: number;  // MinuteOfTheYear
  name?: string;       // Descriptive name
  intersections: IntersectionState[];
}

interface IntersectionState {
  name?: string;
  id: IntersectionReferenceID;
  revision: number;
  status: IntersectionStatusObject;
  moy?: number;           // MinuteOfTheYear
  timeStamp?: number;     // DSecond
  enabledLanes?: LaneID[];
  states: MovementState[];
  maneuverAssistList?: ConnectionManeuverAssist[];
}

interface IntersectionReferenceID {
  region?: number;   // 0-65535
  id: number;        // 0-65535
}

interface IntersectionStatusObject {
  manualControlIsEnabled: boolean;
  stopTimeIsActivated: boolean;
  failureFlash: boolean;
  preemptIsActive: boolean;
  signalPriorityIsActive: boolean;
  fixedTimeOperation: boolean;
  trafficDependentOperation: boolean;
  standbyOperation: boolean;
  failureMode: boolean;
  off: boolean;
  recentMAPmessageUpdate: boolean;
  recentChangeInMAPassignedLanesIDsUsed: boolean;
  noValidMAPisAvailableAtThisTime: boolean;
  noValidSPaTisAvailableAtThisTime: boolean;
}

interface MovementState {
  movementName?: string;
  signalGroup: number;    // 0-255
  stateTimeSpeed: MovementEvent[];
  maneuverAssistList?: ConnectionManeuverAssist[];
}

interface MovementEvent {
  eventState: MovementPhaseState;
  timing?: TimeChangeDetails;
  speeds?: AdvisorySpeed[];
}

enum MovementPhaseState {
  unavailable = 0,
  dark = 1,
  stopThenProceed = 2,
  stopAndRemain = 3,
  preMovement = 4,
  permissiveMovementAllowed = 5,
  protectedMovementAllowed = 6,
  permissiveClearance = 7,
  protectedClearance = 8,
  cautionConflictingTraffic = 9
}

interface TimeChangeDetails {
  startTime?: number;       // TimeMark
  minEndTime: number;       // TimeMark
  maxEndTime?: number;      // TimeMark
  likelyTime?: number;      // TimeMark
  confidence?: number;      // 0-15
  nextTime?: number;        // TimeMark
}

interface AdvisorySpeed {
  type: SpeedAdvisoryType;
  speed?: number;           // 0-500 (0.1 m/s units)
  confidence?: SpeedConfidence;
  distance?: number;        // meters
  class?: number;           // vehicle class
}

enum SpeedAdvisoryType {
  none = 0,
  greenwave = 1,
  ecoDrive = 2,
  transit = 3
}

enum SpeedConfidence {
  unavailable = 0,
  prec100ms = 1,
  prec10ms = 2,
  prec5ms = 3,
  prec1ms = 4,
  prec01ms = 5,
  prec005ms = 6,
  prec001ms = 7
}

type LaneID = number;  // 0-255

interface ConnectionManeuverAssist {
  connectionID: number;
  queueLength?: number;
  availableStorageLength?: number;
  waitOnStop?: boolean;
  pedBicycleDetect?: boolean;
}

/**
 * SPaT Processing and GLOSA (Green Light Optimal Speed Advisory)
 */
class SPaTProcessor {
  private currentSPaT: Map<string, SPaTMessage> = new Map();
  private approachingIntersections: IntersectionApproach[] = [];

  /**
   * Process received SPaT message
   */
  processSPaT(spat: SPaTMessage, source: V2XMessageSource): void {
    for (const intersection of spat.intersections) {
      const key = this.getIntersectionKey(intersection.id);
      this.currentSPaT.set(key, spat);

      // Update approaching intersection list
      this.updateApproachingIntersection(intersection);
    }
  }

  /**
   * Calculate Green Light Optimal Speed Advisory
   */
  calculateGLOSA(
    vehiclePosition: Position,
    vehicleSpeed: number,
    vehicleHeading: number,
    targetIntersection: IntersectionReferenceID,
    targetSignalGroup: number
  ): GLOSAResult {
    const key = this.getIntersectionKey(targetIntersection);
    const spat = this.currentSPaT.get(key);

    if (!spat) {
      return { available: false, reason: "No SPaT data available" };
    }

    const intersection = spat.intersections.find(
      i => i.id.id === targetIntersection.id
    );

    if (!intersection) {
      return { available: false, reason: "Intersection not found in SPaT" };
    }

    const movement = intersection.states.find(
      s => s.signalGroup === targetSignalGroup
    );

    if (!movement) {
      return { available: false, reason: "Signal group not found" };
    }

    const currentEvent = movement.stateTimeSpeed[0];
    if (!currentEvent || !currentEvent.timing) {
      return { available: false, reason: "No timing information" };
    }

    // Calculate distance to intersection
    const distance = this.calculateDistanceToStopBar(
      vehiclePosition,
      targetIntersection
    );

    // Calculate time to reach intersection at various speeds
    const currentState = currentEvent.eventState;
    const timing = currentEvent.timing;

    // Get time remaining in current phase
    const now = this.getCurrentTimeMark();
    const minTimeRemaining = Math.max(0, timing.minEndTime - now);
    const maxTimeRemaining = timing.maxEndTime
      ? Math.max(0, timing.maxEndTime - now)
      : minTimeRemaining;
    const likelyTimeRemaining = timing.likelyTime
      ? Math.max(0, timing.likelyTime - now)
      : (minTimeRemaining + maxTimeRemaining) / 2;

    // Calculate optimal speed
    return this.computeOptimalSpeed(
      distance,
      vehicleSpeed,
      currentState,
      {
        min: minTimeRemaining / 10,  // Convert to seconds
        max: maxTimeRemaining / 10,
        likely: likelyTimeRemaining / 10
      },
      {
        minSpeed: 20,   // km/h
        maxSpeed: 60,   // km/h (speed limit)
        targetAccel: 1.5,  // m/s²
        maxDecel: 3.0      // m/s²
      }
    );
  }

  private computeOptimalSpeed(
    distance: number,
    currentSpeed: number,
    signalState: MovementPhaseState,
    timeRemaining: { min: number; max: number; likely: number },
    constraints: SpeedConstraints
  ): GLOSAResult {
    const isGreen = signalState === MovementPhaseState.permissiveMovementAllowed ||
                    signalState === MovementPhaseState.protectedMovementAllowed;
    const isRed = signalState === MovementPhaseState.stopAndRemain;

    if (isGreen) {
      // Calculate speed to pass on green
      const speedToPass = (distance / timeRemaining.min) * 3.6;  // m/s to km/h

      if (speedToPass <= constraints.maxSpeed && speedToPass >= constraints.minSpeed) {
        return {
          available: true,
          advisorySpeed: Math.round(speedToPass),
          speedType: "PROCEED_GREEN",
          confidence: this.calculateConfidence(timeRemaining),
          timeToGreen: 0,
          distance
        };
      } else if (speedToPass < constraints.minSpeed) {
        // Too slow to make it, calculate speed for next green
        return this.calculateSpeedForNextGreen(
          distance,
          timeRemaining,
          constraints
        );
      } else {
        // Would need to exceed speed limit
        return {
          available: true,
          advisorySpeed: constraints.maxSpeed,
          speedType: "PROCEED_CAUTION",
          confidence: 0.7,
          distance
        };
      }
    } else if (isRed) {
      // Calculate when to arrive at green
      return this.calculateSpeedForNextGreen(
        distance,
        timeRemaining,
        constraints
      );
    }

    return { available: false, reason: "Unknown signal state" };
  }

  private calculateSpeedForNextGreen(
    distance: number,
    currentPhaseRemaining: { min: number; max: number; likely: number },
    constraints: SpeedConstraints
  ): GLOSAResult {
    // Assume next phase timing (would need full cycle data for accuracy)
    const estimatedNextGreen = currentPhaseRemaining.likely + 5; // 5s yellow + buffer

    const speedToArriveAtGreen = (distance / estimatedNextGreen) * 3.6;

    if (speedToArriveAtGreen >= constraints.minSpeed &&
        speedToArriveAtGreen <= constraints.maxSpeed) {
      return {
        available: true,
        advisorySpeed: Math.round(speedToArriveAtGreen),
        speedType: "ECO_APPROACH",
        confidence: 0.6,
        timeToGreen: estimatedNextGreen,
        distance
      };
    }

    // Speed needed is outside constraints - advise stopping
    return {
      available: true,
      advisorySpeed: 0,
      speedType: "PREPARE_TO_STOP",
      confidence: 0.8,
      timeToGreen: estimatedNextGreen,
      distance
    };
  }

  private calculateDistanceToStopBar(
    position: Position,
    intersection: IntersectionReferenceID
  ): number {
    // Would use MAP message data for accurate stop bar location
    // Placeholder implementation
    return 200;  // meters
  }

  private getCurrentTimeMark(): number {
    const now = new Date();
    return now.getSeconds() * 10 + Math.floor(now.getMilliseconds() / 100);
  }

  private getIntersectionKey(id: IntersectionReferenceID): string {
    return `${id.region || 0}-${id.id}`;
  }

  private calculateConfidence(timing: { min: number; max: number }): number {
    const spread = timing.max - timing.min;
    if (spread < 2) return 0.95;
    if (spread < 5) return 0.85;
    if (spread < 10) return 0.70;
    return 0.50;
  }

  private updateApproachingIntersection(intersection: IntersectionState): void {
    // Implementation for tracking approaching intersections
  }
}

interface GLOSAResult {
  available: boolean;
  reason?: string;
  advisorySpeed?: number;
  speedType?: SpeedAdvisoryTypeString;
  confidence?: number;
  timeToGreen?: number;
  distance?: number;
}

type SpeedAdvisoryTypeString =
  | "PROCEED_GREEN"
  | "PROCEED_CAUTION"
  | "ECO_APPROACH"
  | "PREPARE_TO_STOP";

interface SpeedConstraints {
  minSpeed: number;
  maxSpeed: number;
  targetAccel: number;
  maxDecel: number;
}

interface Position {
  latitude: number;
  longitude: number;
  altitude?: number;
}

interface V2XMessageSource {
  id: string;
  rssi: number;
  channel: number;
}

interface IntersectionApproach {
  intersection: IntersectionReferenceID;
  distance: number;
  eta: number;
  signalGroup: number;
}
```

---

## Remote Vehicle Control Protocol

```typescript
/**
 * Remote Vehicle Control Protocol
 * Secure command execution framework
 */
interface RemoteControlProtocol {
  authentication: AuthenticationLayer;
  authorization: AuthorizationLayer;
  commandExecution: CommandExecutionLayer;
  feedback: FeedbackLayer;
}

/**
 * Command Types and Definitions
 */
enum VehicleCommandType {
  // Security commands
  LOCK = "LOCK",
  UNLOCK = "UNLOCK",
  ARM_ALARM = "ARM_ALARM",
  DISARM_ALARM = "DISARM_ALARM",

  // Powertrain commands
  REMOTE_START = "REMOTE_START",
  REMOTE_STOP = "REMOTE_STOP",

  // Climate commands
  CLIMATE_ON = "CLIMATE_ON",
  CLIMATE_OFF = "CLIMATE_OFF",
  SET_TEMPERATURE = "SET_TEMPERATURE",
  DEFROST_ON = "DEFROST_ON",
  DEFROST_OFF = "DEFROST_OFF",
  SEAT_HEATER = "SEAT_HEATER",
  SEAT_COOLER = "SEAT_COOLER",
  STEERING_HEATER = "STEERING_HEATER",

  // Charging commands (EVs)
  START_CHARGING = "START_CHARGING",
  STOP_CHARGING = "STOP_CHARGING",
  SET_CHARGE_LIMIT = "SET_CHARGE_LIMIT",
  SET_CHARGE_SCHEDULE = "SET_CHARGE_SCHEDULE",
  OPEN_CHARGE_PORT = "OPEN_CHARGE_PORT",
  CLOSE_CHARGE_PORT = "CLOSE_CHARGE_PORT",

  // Access commands
  OPEN_TRUNK = "OPEN_TRUNK",
  CLOSE_TRUNK = "CLOSE_TRUNK",
  OPEN_FRUNK = "OPEN_FRUNK",
  OPEN_WINDOWS = "OPEN_WINDOWS",
  CLOSE_WINDOWS = "CLOSE_WINDOWS",
  VENT_WINDOWS = "VENT_WINDOWS",

  // Location commands
  FLASH_LIGHTS = "FLASH_LIGHTS",
  HONK_HORN = "HONK_HORN",
  LOCATE_VEHICLE = "LOCATE_VEHICLE",

  // Advanced commands
  SUMMON = "SUMMON",
  PARK_ASSIST = "PARK_ASSIST",
  SOFTWARE_UPDATE = "SOFTWARE_UPDATE"
}

interface VehicleCommand {
  id: string;
  type: VehicleCommandType;
  vehicleId: string;
  userId: string;
  timestamp: Date;
  parameters?: CommandParameters;
  timeout: number;
  priority: CommandPriority;
  requiresConfirmation: boolean;
  expiresAt: Date;
}

type CommandParameters = Record<string, any>;

enum CommandPriority {
  CRITICAL = 0,   // Safety-related commands
  HIGH = 1,       // Security commands
  NORMAL = 2,     // Standard commands
  LOW = 3         // Non-urgent commands
}

interface CommandResult {
  commandId: string;
  status: CommandStatus;
  startedAt?: Date;
  completedAt?: Date;
  progress?: number;
  result?: any;
  error?: CommandError;
}

enum CommandStatus {
  QUEUED = "QUEUED",
  PENDING = "PENDING",
  SENT = "SENT",
  ACKNOWLEDGED = "ACKNOWLEDGED",
  IN_PROGRESS = "IN_PROGRESS",
  COMPLETED = "COMPLETED",
  FAILED = "FAILED",
  TIMEOUT = "TIMEOUT",
  CANCELLED = "CANCELLED",
  REJECTED = "REJECTED"
}

interface CommandError {
  code: string;
  message: string;
  retryable: boolean;
  details?: any;
}

/**
 * Remote Control Service Implementation
 */
class RemoteControlService {
  private commandQueue: PriorityQueue<VehicleCommand>;
  private activeCommands: Map<string, CommandExecution> = new Map();
  private vehicleConnection: VehicleConnectionManager;

  constructor(
    private config: RemoteControlConfig,
    private authService: AuthenticationService,
    private authzService: AuthorizationService
  ) {
    this.commandQueue = new PriorityQueue((a, b) => a.priority - b.priority);
    this.vehicleConnection = new VehicleConnectionManager(config.connection);
  }

  /**
   * Submit command for execution
   */
  async submitCommand(
    command: Omit<VehicleCommand, "id" | "timestamp" | "expiresAt">
  ): Promise<CommandResult> {
    // Generate command ID
    const commandId = crypto.randomUUID();

    // Validate user authentication
    const authResult = await this.authService.validateSession(command.userId);
    if (!authResult.valid) {
      throw new AuthenticationError("Invalid session");
    }

    // Check authorization
    const authzResult = await this.authzService.checkPermission({
      userId: command.userId,
      vehicleId: command.vehicleId,
      action: command.type,
      context: { parameters: command.parameters }
    });

    if (!authzResult.allowed) {
      throw new AuthorizationError(
        `Not authorized for ${command.type}`,
        authzResult.reason
      );
    }

    // Validate command parameters
    this.validateCommandParameters(command.type, command.parameters);

    // Check vehicle connectivity
    const connectivity = await this.vehicleConnection.checkConnectivity(
      command.vehicleId
    );

    if (!connectivity.online) {
      // Queue command for later delivery
      return this.queueOfflineCommand(command, commandId);
    }

    // Execute command
    return this.executeCommand({
      ...command,
      id: commandId,
      timestamp: new Date(),
      expiresAt: new Date(Date.now() + command.timeout * 1000)
    });
  }

  /**
   * Execute command on vehicle
   */
  private async executeCommand(command: VehicleCommand): Promise<CommandResult> {
    const execution = new CommandExecution(command);
    this.activeCommands.set(command.id, execution);

    try {
      // Update status to pending
      execution.updateStatus(CommandStatus.PENDING);

      // Build secure command payload
      const payload = await this.buildSecurePayload(command);

      // Send to vehicle
      execution.updateStatus(CommandStatus.SENT);
      const response = await this.vehicleConnection.sendCommand(
        command.vehicleId,
        payload,
        { timeout: command.timeout * 1000 }
      );

      // Process response
      if (response.acknowledged) {
        execution.updateStatus(CommandStatus.ACKNOWLEDGED);
      }

      // Wait for completion
      const result = await this.waitForCompletion(execution, response);

      return result;
    } catch (error) {
      execution.updateStatus(CommandStatus.FAILED, {
        error: this.normalizeError(error)
      });
      throw error;
    } finally {
      this.activeCommands.delete(command.id);
    }
  }

  /**
   * Build secure command payload
   */
  private async buildSecurePayload(command: VehicleCommand): Promise<Buffer> {
    const payload: CommandPayload = {
      header: {
        version: 1,
        commandId: command.id,
        timestamp: command.timestamp.getTime(),
        userId: command.userId,
        nonce: crypto.randomBytes(16)
      },
      body: {
        type: command.type,
        parameters: command.parameters || {},
        timeout: command.timeout
      }
    };

    // Serialize payload
    const serialized = this.serializePayload(payload);

    // Encrypt with vehicle's public key
    const encrypted = await this.encryptForVehicle(
      command.vehicleId,
      serialized
    );

    // Sign with user's private key
    const signed = await this.signCommand(command.userId, encrypted);

    return signed;
  }

  /**
   * Wait for command completion
   */
  private async waitForCompletion(
    execution: CommandExecution,
    initialResponse: CommandResponse
  ): Promise<CommandResult> {
    return new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        execution.updateStatus(CommandStatus.TIMEOUT);
        reject(new TimeoutError("Command execution timed out"));
      }, execution.command.timeout * 1000);

      // Subscribe to status updates
      const unsubscribe = execution.onStatusChange((status, data) => {
        if (status === CommandStatus.COMPLETED) {
          clearTimeout(timeout);
          unsubscribe();
          resolve(execution.getResult());
        } else if (status === CommandStatus.FAILED) {
          clearTimeout(timeout);
          unsubscribe();
          reject(new CommandExecutionError(data?.error));
        }
      });

      // If initial response indicates immediate completion
      if (initialResponse.completed) {
        execution.updateStatus(CommandStatus.COMPLETED, {
          result: initialResponse.result
        });
      }
    });
  }

  /**
   * Queue command for offline vehicle
   */
  private async queueOfflineCommand(
    command: any,
    commandId: string
  ): Promise<CommandResult> {
    const fullCommand: VehicleCommand = {
      ...command,
      id: commandId,
      timestamp: new Date(),
      expiresAt: new Date(Date.now() + command.timeout * 1000)
    };

    this.commandQueue.enqueue(fullCommand);

    // Schedule retry when vehicle comes online
    this.vehicleConnection.onVehicleOnline(command.vehicleId, async () => {
      const pendingCommand = this.commandQueue.dequeue();
      if (pendingCommand && pendingCommand.id === commandId) {
        await this.executeCommand(pendingCommand);
      }
    });

    return {
      commandId,
      status: CommandStatus.QUEUED
    };
  }

  /**
   * Validate command parameters
   */
  private validateCommandParameters(
    type: VehicleCommandType,
    parameters?: CommandParameters
  ): void {
    const rules = COMMAND_VALIDATION_RULES[type];
    if (!rules) return;

    for (const rule of rules) {
      if (rule.required && !parameters?.[rule.field]) {
        throw new ValidationError(`Missing required parameter: ${rule.field}`);
      }

      if (parameters?.[rule.field] !== undefined) {
        if (rule.min !== undefined && parameters[rule.field] < rule.min) {
          throw new ValidationError(
            `Parameter ${rule.field} below minimum: ${rule.min}`
          );
        }
        if (rule.max !== undefined && parameters[rule.field] > rule.max) {
          throw new ValidationError(
            `Parameter ${rule.field} above maximum: ${rule.max}`
          );
        }
        if (rule.enum && !rule.enum.includes(parameters[rule.field])) {
          throw new ValidationError(
            `Invalid value for ${rule.field}: ${parameters[rule.field]}`
          );
        }
      }
    }
  }

  private serializePayload(payload: CommandPayload): Buffer {
    return Buffer.from(JSON.stringify(payload));
  }

  private async encryptForVehicle(vehicleId: string, data: Buffer): Promise<Buffer> {
    return data; // Placeholder
  }

  private async signCommand(userId: string, data: Buffer): Promise<Buffer> {
    return data; // Placeholder
  }

  private normalizeError(error: any): CommandError {
    return {
      code: error.code || "UNKNOWN",
      message: error.message,
      retryable: error.retryable ?? false
    };
  }
}

// Validation rules for command parameters
const COMMAND_VALIDATION_RULES: Record<string, ValidationRule[]> = {
  [VehicleCommandType.SET_TEMPERATURE]: [
    { field: "temperature", required: true, min: 15, max: 28 },
    { field: "zone", required: false, enum: ["DRIVER", "PASSENGER", "ALL"] }
  ],
  [VehicleCommandType.SET_CHARGE_LIMIT]: [
    { field: "limit", required: true, min: 50, max: 100 }
  ],
  [VehicleCommandType.SEAT_HEATER]: [
    { field: "seat", required: true, enum: ["DRIVER", "PASSENGER", "REAR_LEFT", "REAR_RIGHT"] },
    { field: "level", required: true, min: 0, max: 3 }
  ],
  [VehicleCommandType.SUMMON]: [
    { field: "direction", required: true, enum: ["FORWARD", "REVERSE"] },
    { field: "distance", required: false, min: 0, max: 60 }
  ]
};

interface ValidationRule {
  field: string;
  required: boolean;
  min?: number;
  max?: number;
  enum?: any[];
}

interface CommandPayload {
  header: {
    version: number;
    commandId: string;
    timestamp: number;
    userId: string;
    nonce: Buffer;
  };
  body: {
    type: VehicleCommandType;
    parameters: CommandParameters;
    timeout: number;
  };
}

interface CommandResponse {
  acknowledged: boolean;
  completed: boolean;
  result?: any;
  error?: CommandError;
}

interface RemoteControlConfig {
  connection: ConnectionConfig;
}

interface ConnectionConfig {
  timeout: number;
  retries: number;
}

// Supporting classes (stubs)
class CommandExecution {
  constructor(public command: VehicleCommand) {}
  updateStatus(status: CommandStatus, data?: any): void {}
  onStatusChange(callback: (status: CommandStatus, data?: any) => void): () => void { return () => {}; }
  getResult(): CommandResult { return { commandId: this.command.id, status: CommandStatus.COMPLETED }; }
}

class PriorityQueue<T> {
  private items: T[] = [];
  constructor(private comparator: (a: T, b: T) => number) {}
  enqueue(item: T): void { this.items.push(item); this.items.sort(this.comparator); }
  dequeue(): T | undefined { return this.items.shift(); }
}

class VehicleConnectionManager {
  constructor(config: ConnectionConfig) {}
  async checkConnectivity(vehicleId: string): Promise<{ online: boolean }> { return { online: true }; }
  async sendCommand(vehicleId: string, payload: Buffer, options: any): Promise<CommandResponse> {
    return { acknowledged: true, completed: true };
  }
  onVehicleOnline(vehicleId: string, callback: () => void): void {}
}

class AuthenticationService {
  async validateSession(userId: string): Promise<{ valid: boolean }> { return { valid: true }; }
}

class AuthorizationService {
  async checkPermission(request: any): Promise<{ allowed: boolean; reason?: string }> {
    return { allowed: true };
  }
}

// Error classes
class AuthenticationError extends Error { constructor(message: string) { super(message); this.name = "AuthenticationError"; } }
class AuthorizationError extends Error { constructor(message: string, public reason?: string) { super(message); this.name = "AuthorizationError"; } }
class ValidationError extends Error { constructor(message: string) { super(message); this.name = "ValidationError"; } }
class TimeoutError extends Error { constructor(message: string) { super(message); this.name = "TimeoutError"; } }
class CommandExecutionError extends Error { constructor(public error?: CommandError) { super(error?.message); this.name = "CommandExecutionError"; } }
```

---

## CAN Bus Protocol Bridge

```typescript
/**
 * CAN Bus Protocol Bridge
 * Translating cloud commands to vehicle CAN messages
 */
interface CANBridge {
  translator: CommandTranslator;
  gateway: CANGateway;
  security: CANSecurity;
}

interface CANMessage {
  id: number;           // CAN ID (11-bit standard or 29-bit extended)
  extended: boolean;    // Extended frame flag
  data: Buffer;         // Up to 8 bytes (CAN 2.0) or 64 bytes (CAN FD)
  dlc: number;          // Data length code
  timestamp?: number;   // Message timestamp
}

interface CANSignal {
  name: string;
  startBit: number;
  length: number;
  byteOrder: "LITTLE_ENDIAN" | "BIG_ENDIAN";
  valueType: "SIGNED" | "UNSIGNED";
  factor: number;
  offset: number;
  min: number;
  max: number;
  unit: string;
}

/**
 * CAN Database (DBC) Parser and Handler
 */
class CANDatabase {
  private messages: Map<number, CANMessageDefinition> = new Map();
  private signalsByName: Map<string, { messageId: number; signal: CANSignal }> = new Map();

  /**
   * Load DBC file
   */
  loadDBC(dbcContent: string): void {
    const lines = dbcContent.split("\n");
    let currentMessage: CANMessageDefinition | null = null;

    for (const line of lines) {
      const trimmed = line.trim();

      // Parse message definition
      const messageMatch = trimmed.match(/^BO_ (\d+) (\w+): (\d+)/);
      if (messageMatch) {
        currentMessage = {
          id: parseInt(messageMatch[1]),
          name: messageMatch[2],
          length: parseInt(messageMatch[3]),
          signals: []
        };
        this.messages.set(currentMessage.id, currentMessage);
        continue;
      }

      // Parse signal definition
      const signalMatch = trimmed.match(
        /^SG_ (\w+) : (\d+)\|(\d+)@([01])([+-]) \(([^,]+),([^)]+)\) \[([^|]+)\|([^\]]+)\] "([^"]*)" (.*)/
      );
      if (signalMatch && currentMessage) {
        const signal: CANSignal = {
          name: signalMatch[1],
          startBit: parseInt(signalMatch[2]),
          length: parseInt(signalMatch[3]),
          byteOrder: signalMatch[4] === "1" ? "LITTLE_ENDIAN" : "BIG_ENDIAN",
          valueType: signalMatch[5] === "+" ? "UNSIGNED" : "SIGNED",
          factor: parseFloat(signalMatch[6]),
          offset: parseFloat(signalMatch[7]),
          min: parseFloat(signalMatch[8]),
          max: parseFloat(signalMatch[9]),
          unit: signalMatch[10]
        };
        currentMessage.signals.push(signal);
        this.signalsByName.set(signal.name, {
          messageId: currentMessage.id,
          signal
        });
      }
    }
  }

  /**
   * Decode CAN message to physical values
   */
  decode(message: CANMessage): DecodedCANMessage | null {
    const definition = this.messages.get(message.id);
    if (!definition) return null;

    const signals: Record<string, number> = {};

    for (const signalDef of definition.signals) {
      const rawValue = this.extractSignalValue(message.data, signalDef);
      const physicalValue = rawValue * signalDef.factor + signalDef.offset;
      signals[signalDef.name] = physicalValue;
    }

    return {
      id: message.id,
      name: definition.name,
      signals,
      timestamp: message.timestamp
    };
  }

  /**
   * Encode physical values to CAN message
   */
  encode(messageName: string, signals: Record<string, number>): CANMessage {
    const definition = Array.from(this.messages.values()).find(
      m => m.name === messageName
    );

    if (!definition) {
      throw new Error(`Message ${messageName} not found in database`);
    }

    const data = Buffer.alloc(definition.length);

    for (const [signalName, physicalValue] of Object.entries(signals)) {
      const signalDef = definition.signals.find(s => s.name === signalName);
      if (!signalDef) continue;

      const rawValue = Math.round((physicalValue - signalDef.offset) / signalDef.factor);
      this.packSignalValue(data, signalDef, rawValue);
    }

    return {
      id: definition.id,
      extended: false,
      data,
      dlc: definition.length
    };
  }

  private extractSignalValue(data: Buffer, signal: CANSignal): number {
    let value = 0n;
    const startByte = Math.floor(signal.startBit / 8);
    const startBitInByte = signal.startBit % 8;

    if (signal.byteOrder === "LITTLE_ENDIAN") {
      for (let i = 0; i < Math.ceil(signal.length / 8); i++) {
        const byteIndex = startByte + i;
        if (byteIndex < data.length) {
          value |= BigInt(data[byteIndex]) << BigInt(i * 8);
        }
      }
      value = (value >> BigInt(startBitInByte)) & ((1n << BigInt(signal.length)) - 1n);
    } else {
      // Big endian extraction
      let bitsRead = 0;
      let currentByte = startByte;
      let currentBit = startBitInByte;

      while (bitsRead < signal.length) {
        const bitsInThisByte = Math.min(8 - currentBit, signal.length - bitsRead);
        const mask = (1 << bitsInThisByte) - 1;
        const shift = 8 - currentBit - bitsInThisByte;
        const byteValue = (data[currentByte] >> shift) & mask;
        value = (value << BigInt(bitsInThisByte)) | BigInt(byteValue);
        bitsRead += bitsInThisByte;
        currentByte++;
        currentBit = 0;
      }
    }

    // Handle signed values
    if (signal.valueType === "SIGNED") {
      const signBit = 1n << BigInt(signal.length - 1);
      if (value & signBit) {
        value -= 1n << BigInt(signal.length);
      }
    }

    return Number(value);
  }

  private packSignalValue(data: Buffer, signal: CANSignal, value: number): void {
    let bigValue = BigInt(value);

    // Handle negative values for signed signals
    if (signal.valueType === "SIGNED" && value < 0) {
      bigValue = (1n << BigInt(signal.length)) + bigValue;
    }

    const startByte = Math.floor(signal.startBit / 8);
    const startBitInByte = signal.startBit % 8;

    if (signal.byteOrder === "LITTLE_ENDIAN") {
      bigValue <<= BigInt(startBitInByte);
      for (let i = 0; i < Math.ceil((signal.length + startBitInByte) / 8); i++) {
        const byteIndex = startByte + i;
        if (byteIndex < data.length) {
          data[byteIndex] |= Number((bigValue >> BigInt(i * 8)) & 0xFFn);
        }
      }
    } else {
      // Big endian packing
      let bitsWritten = 0;
      let currentByte = startByte;
      let currentBit = startBitInByte;

      while (bitsWritten < signal.length) {
        const bitsInThisByte = Math.min(8 - currentBit, signal.length - bitsWritten);
        const shift = signal.length - bitsWritten - bitsInThisByte;
        const mask = (1n << BigInt(bitsInThisByte)) - 1n;
        const byteValue = Number((bigValue >> BigInt(shift)) & mask);
        const targetShift = 8 - currentBit - bitsInThisByte;
        data[currentByte] |= byteValue << targetShift;
        bitsWritten += bitsInThisByte;
        currentByte++;
        currentBit = 0;
      }
    }
  }
}

interface CANMessageDefinition {
  id: number;
  name: string;
  length: number;
  signals: CANSignal[];
}

interface DecodedCANMessage {
  id: number;
  name: string;
  signals: Record<string, number>;
  timestamp?: number;
}

/**
 * Command to CAN Translator
 */
class CommandToCANTranslator {
  private canDb: CANDatabase;
  private commandMappings: Map<VehicleCommandType, CommandCANMapping>;

  constructor(canDb: CANDatabase) {
    this.canDb = canDb;
    this.commandMappings = this.initializeMappings();
  }

  /**
   * Translate cloud command to CAN messages
   */
  translate(
    command: VehicleCommandType,
    parameters: CommandParameters
  ): CANMessage[] {
    const mapping = this.commandMappings.get(command);
    if (!mapping) {
      throw new Error(`No CAN mapping for command: ${command}`);
    }

    return mapping.generator(parameters, this.canDb);
  }

  private initializeMappings(): Map<VehicleCommandType, CommandCANMapping> {
    const mappings = new Map<VehicleCommandType, CommandCANMapping>();

    // Door lock command
    mappings.set(VehicleCommandType.LOCK, {
      generator: (params, db) => [
        db.encode("BCM_DoorControl", {
          DoorLockRequest: 1,
          AllDoors: 1,
          RequestSource: 2  // Remote
        })
      ]
    });

    // Door unlock command
    mappings.set(VehicleCommandType.UNLOCK, {
      generator: (params, db) => [
        db.encode("BCM_DoorControl", {
          DoorLockRequest: 0,
          AllDoors: 1,
          RequestSource: 2
        })
      ]
    });

    // Climate control
    mappings.set(VehicleCommandType.CLIMATE_ON, {
      generator: (params, db) => {
        const messages: CANMessage[] = [];

        // HVAC power on
        messages.push(db.encode("HVAC_Control", {
          HVACPowerRequest: 1,
          ACRequest: 1,
          RecirculationMode: 0
        }));

        // Set temperature if provided
        if (params.temperature) {
          messages.push(db.encode("HVAC_TempControl", {
            TargetTemperature: params.temperature,
            Zone: params.zone === "DRIVER" ? 0 : params.zone === "PASSENGER" ? 1 : 2
          }));
        }

        return messages;
      }
    });

    // Remote start
    mappings.set(VehicleCommandType.REMOTE_START, {
      generator: (params, db) => {
        // Remote start requires specific sequence
        return [
          // Wake up ECU
          db.encode("BCM_WakeUp", { WakeUpRequest: 1 }),
          // Authenticate (simplified - real implementation uses challenge-response)
          db.encode("Gateway_Auth", { AuthSequence: 1 }),
          // Send start command
          db.encode("EMS_RemoteStart", {
            RemoteStartRequest: 1,
            Duration: params.duration || 10  // minutes
          })
        ];
      }
    });

    // Charging control
    mappings.set(VehicleCommandType.START_CHARGING, {
      generator: (params, db) => [
        db.encode("BMS_ChargeControl", {
          ChargeRequest: 1,
          ChargeMode: params.mode || 0,  // 0=Normal, 1=Fast
          TargetSOC: params.targetSoC || 80
        })
      ]
    });

    return mappings;
  }
}

interface CommandCANMapping {
  generator: (params: CommandParameters, db: CANDatabase) => CANMessage[];
}
```

---

## Summary

| Protocol | Use Case | Latency | Range | Security |
|----------|----------|---------|-------|----------|
| **BSM (SAE J2735)** | V2V Safety | <100ms | 300m | IEEE 1609.2 |
| **SPaT/MAP** | V2I Traffic | <200ms | 500m | IEEE 1609.2 |
| **C-V2X PC5** | Direct V2X | <20ms | 500m | 3GPP Security |
| **Remote Command** | Vehicle Control | <30s | Global | TLS + mTLS |
| **CAN Bus** | Internal ECU | <10ms | Vehicle | SecOC |

---

**Next Chapter:** [Chapter 6: Integration](./06-integration.md) - OEM and ecosystem integration patterns.

---

© 2025 World Industry Association (WIA). All rights reserved.
