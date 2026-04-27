/**
 * WIA-COMM-003: V2X Communication SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for Vehicle-to-Everything (V2X) communication including:
 * - V2V (Vehicle-to-Vehicle) messaging
 * - V2I (Vehicle-to-Infrastructure) integration
 * - V2P (Vehicle-to-Pedestrian) safety
 * - V2N (Vehicle-to-Network) connectivity
 * - Collision detection and warnings
 * - Platooning coordination
 */

import {
  V2XConfig,
  V2VMessage,
  V2IMessage,
  BSM,
  CAM,
  DENM,
  SPaTMessage,
  PSM,
  CollisionWarning,
  PedestrianDetection,
  V2NTelemetry,
  Platoon,
  PlatoonJoinRequest,
  PlatoonJoinResponse,
  PlatoonManeuver,
  Position,
  Acceleration,
  MessageType,
  V2XTechnology,
  SecurityLevel,
  V2XError,
  V2XErrorCode,
  V2X_CONSTANTS,
  V2XEventHandler,
  MessageFilter,
  RSUInfo
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMM-003 V2X Communication SDK
 */
export class V2XCommunication {
  private version = '1.0.0';
  private initialized = false;
  private config: V2XConfig;
  private msgCount = 0;
  private eventHandlers: Map<string, Set<V2XEventHandler>> = new Map();
  private receivedMessages: Map<string, any[]> = new Map();

  constructor(config: V2XConfig) {
    this.config = {
      frequency: V2X_CONSTANTS.DSRC_FREQUENCY,
      transmitPower: 20, // dBm
      securityLevel: 'standard',
      privacyLevel: 'medium',
      messageRates: {
        bsm: 10,
        cam: 5,
        denm: 1
      },
      maxRange: config.technology === 'DSRC'
        ? V2X_CONSTANTS.MAX_RANGE_DSRC
        : V2X_CONSTANTS.MAX_RANGE_CV2X,
      ...config
    };
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Initialize V2X communication system
   */
  async initialize(): Promise<void> {
    if (this.initialized) {
      throw new V2XError(
        V2XErrorCode.INITIALIZATION_FAILED,
        'V2X already initialized'
      );
    }

    // Initialize radio
    await this.initializeRadio();

    // Initialize security module
    await this.initializeSecurity();

    // Initialize GNSS
    await this.initializeGNSS();

    this.initialized = true;
  }

  /**
   * Shutdown V2X communication
   */
  async shutdown(): Promise<void> {
    this.initialized = false;
    this.eventHandlers.clear();
    this.receivedMessages.clear();
  }

  // ============================================================================
  // V2V Communication
  // ============================================================================

  /**
   * Send V2V message (broadcast)
   */
  async sendV2V(message: V2VMessage): Promise<void> {
    this.ensureInitialized();

    // Sign message if security enabled
    if (this.config.securityLevel !== 'none') {
      // In real implementation, this would sign the message
      console.log('Signing V2V message...');
    }

    // Broadcast message
    await this.broadcastMessage(message);

    // Emit event
    this.emit('v2v:sent', message);
  }

  /**
   * Broadcast vehicle position (BSM/CAM)
   */
  async broadcastPosition(params: {
    lat: number;
    lon: number;
    speed: number;
    heading: number;
    acceleration?: Acceleration;
  }): Promise<void> {
    const bsm = this.generateBSM(params);
    await this.sendV2V({
      type: MessageType.BSM,
      senderId: this.config.vehicleId,
      position: { latitude: params.lat, longitude: params.lon },
      speed: params.speed,
      heading: params.heading,
      acceleration: params.acceleration || { longitudinal: 0, lateral: 0, vertical: 0 },
      timestamp: Date.now(),
      data: bsm
    });
  }

  /**
   * Register V2V message handler
   */
  onV2V(handler: V2XEventHandler<V2VMessage>): void {
    this.on('v2v:received', handler);
  }

  /**
   * Detect collision risk with another vehicle
   */
  detectCollisionRisk(targetMessage: V2VMessage): CollisionWarning | null {
    // Get own vehicle state (simulated)
    const egoVehicle = this.getEgoVehicleState();

    // Calculate distance
    const distance = this.calculateDistance(
      egoVehicle.position,
      targetMessage.position
    );

    // Check if vehicles are on collision course
    const headingDiff = Math.abs(egoVehicle.heading - targetMessage.heading);

    // Only warn if heading in similar direction (within 45 degrees)
    if (headingDiff > 45 && headingDiff < 315) {
      return null; // Not on collision course
    }

    // Calculate relative velocity
    const relativeVelocity = egoVehicle.speed - targetMessage.speed;

    // If target is moving away or same speed, no risk
    if (relativeVelocity <= 0) {
      return null;
    }

    // Calculate time to collision (TTC)
    const ttc = distance / (relativeVelocity / 3.6); // Convert km/h to m/s

    // Determine warning level
    let level: CollisionWarning['level'];
    let action: CollisionWarning['action'];
    let requiredDeceleration = 0;

    if (ttc < V2X_CONSTANTS.CRITICAL_COLLISION_TTC) {
      level = 'CRITICAL';
      action = 'EMERGENCY_BRAKE';
      requiredDeceleration = Math.pow(relativeVelocity / 3.6, 2) / (2 * distance);
    } else if (ttc < V2X_CONSTANTS.COLLISION_WARNING_TTC) {
      level = 'HIGH';
      action = 'BRAKE';
      requiredDeceleration = Math.pow(relativeVelocity / 3.6, 2) / (2 * distance);
    } else if (ttc < 4.0) {
      level = 'MEDIUM';
      action = 'WARN';
    } else if (ttc < 6.0) {
      level = 'LOW';
      action = 'MONITOR';
    } else {
      return null; // No warning needed
    }

    return {
      level,
      targetVehicleId: targetMessage.senderId,
      ttc,
      distance,
      action,
      requiredDeceleration: requiredDeceleration > 0 ? requiredDeceleration : undefined
    };
  }

  /**
   * Send hazard warning (DENM)
   */
  async sendHazardWarning(params: {
    type: 'accident' | 'breakdown' | 'hazard' | 'weather';
    severity: 'low' | 'medium' | 'high' | 'danger';
    description?: string;
  }): Promise<void> {
    const egoState = this.getEgoVehicleState();

    const denm: DENM = {
      messageType: MessageType.DENM,
      protocolVersion: 1,
      stationID: this.parseStationId(this.config.vehicleId),
      management: {
        actionID: {
          originatingStationID: this.parseStationId(this.config.vehicleId),
          sequenceNumber: this.msgCount++
        },
        detectionTime: Date.now(),
        referenceTime: Date.now(),
        eventPosition: egoState.position,
        relevanceDistance: 'lessThan500m',
        relevanceTrafficDirection: 'allTrafficDirections',
        validityDuration: 600, // 10 minutes
        transmissionInterval: 1000 // 1 Hz
      },
      situation: {
        eventType: {
          causeCode: this.mapHazardTypeToCauseCode(params.type)
        },
        severity: params.severity
      }
    };

    await this.broadcastMessage(denm);
    this.emit('hazard:sent', denm);
  }

  /**
   * Register hazard warning handler
   */
  onHazardWarning(handler: V2XEventHandler<DENM>): void {
    this.on('hazard:received', handler);
  }

  // ============================================================================
  // V2I Communication
  // ============================================================================

  /**
   * Subscribe to infrastructure updates (RSU)
   */
  subscribeToInfrastructure(rsuId: string, handler: V2XEventHandler<SPaTMessage>): void {
    this.on(`rsu:${rsuId}`, handler);
  }

  /**
   * Unsubscribe from infrastructure
   */
  unsubscribeFromInfrastructure(rsuId: string): void {
    this.off(`rsu:${rsuId}`);
  }

  /**
   * Get traffic signal information
   */
  async getTrafficSignalInfo(intersectionId: number): Promise<SPaTMessage | null> {
    // Query for SPaT messages for this intersection
    const messages = this.receivedMessages.get('spat') || [];

    const spatMessage = messages.find(
      (msg: SPaTMessage) => msg.intersectionID === intersectionId
    );

    return spatMessage || null;
  }

  /**
   * Request signal priority (emergency vehicles)
   */
  async requestSignalPriority(params: {
    intersectionId: number;
    approach: 'north' | 'south' | 'east' | 'west';
    eta: number; // seconds
    duration: number; // requested green time in seconds
  }): Promise<boolean> {
    // Send Signal Request Message (SRM)
    const srm = {
      messageType: MessageType.SRM,
      requestID: Math.floor(Math.random() * 100000),
      vehicleID: this.config.vehicleId,
      vehicleType: 'emergency',
      priority: 'emergency',
      route: {
        approach: params.approach,
        departure: 'south' // Opposite or specified
      },
      eta: params.eta,
      duration: params.duration
    };

    await this.sendMessage(srm);

    // In real implementation, wait for SSM (Signal Status Message)
    return true;
  }

  // ============================================================================
  // V2P Communication
  // ============================================================================

  /**
   * Detect pedestrians in vicinity
   */
  detectPedestrians(): PedestrianDetection[] {
    const psmMessages = this.receivedMessages.get('psm') || [];
    const egoState = this.getEgoVehicleState();
    const detections: PedestrianDetection[] = [];

    for (const psm of psmMessages as PSM[]) {
      const distance = this.calculateDistance(egoState.position, psm.position);

      // Only include pedestrians within range
      if (distance > 100) continue;

      // Assess risk
      const risk = this.assessPedestrianRisk(egoState, psm, distance);

      detections.push({
        pedestrianId: psm.deviceID,
        position: psm.position,
        speed: psm.speed,
        heading: psm.heading,
        distance,
        risk: risk.level,
        ttc: risk.ttc,
        action: risk.action
      });
    }

    return detections;
  }

  /**
   * Register pedestrian detection handler
   */
  onPedestrianDetected(handler: V2XEventHandler<PedestrianDetection[]>): void {
    this.on('pedestrian:detected', handler);
  }

  // ============================================================================
  // V2N Communication
  // ============================================================================

  /**
   * Send telemetry to cloud
   */
  async sendTelemetry(telemetry: V2NTelemetry): Promise<void> {
    // In real implementation, send to cloud via cellular/WiFi
    console.log('Sending telemetry to cloud:', telemetry);

    this.emit('telemetry:sent', telemetry);
  }

  /**
   * Subscribe to traffic updates from cloud
   */
  onTrafficUpdate(handler: V2XEventHandler<any>): void {
    this.on('traffic:update', handler);
  }

  // ============================================================================
  // Platooning
  // ============================================================================

  /**
   * Create platoon (become leader)
   */
  createPlatoon(params: {
    maxMembers?: number;
    targetSpeed: number;
    spacing?: number;
  }): Platoon {
    const platoon: Platoon = {
      platoonId: this.generateUUID(),
      leader: this.config.vehicleId,
      members: [],
      formation: 'line',
      targetSpeed: params.targetSpeed,
      spacing: params.spacing || 10, // meters
      maxMembers: params.maxMembers || 10,
      status: 'FORMING'
    };

    this.emit('platoon:created', platoon);

    return platoon;
  }

  /**
   * Request to join platoon
   */
  async requestJoinPlatoon(platoonId: string): Promise<PlatoonJoinResponse> {
    const request: PlatoonJoinRequest = {
      vehicleId: this.config.vehicleId,
      platoonId,
      capabilities: {
        cacc: true,
        v2v: true,
        maxSpeed: 120, // km/h
        vehicleType: 'passengerCar' as any
      }
    };

    // Send join request
    await this.sendMessage({
      type: 'PLATOON_JOIN_REQUEST',
      data: request
    });

    // In real implementation, wait for response
    return {
      status: 'APPROVED',
      position: 1,
      targetSpacing: 10,
      targetSpeed: 100
    };
  }

  /**
   * Leave platoon
   */
  async leavePlatoon(platoonId: string): Promise<void> {
    await this.sendMessage({
      type: 'PLATOON_LEAVE',
      platoonId,
      vehicleId: this.config.vehicleId
    });

    this.emit('platoon:left', { platoonId });
  }

  /**
   * Send platoon maneuver command (leader only)
   */
  async sendPlatoonManeuver(maneuver: PlatoonManeuver): Promise<void> {
    await this.broadcastMessage(maneuver);
    this.emit('platoon:maneuver', maneuver);
  }

  /**
   * Register platoon event handler
   */
  onPlatoonEvent(event: string, handler: V2XEventHandler): void {
    this.on(`platoon:${event}`, handler);
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Measure communication latency to target
   */
  async measureLatency(targetVehicleId: string, count: number = 10): Promise<{
    min: number;
    max: number;
    avg: number;
    samples: number[];
  }> {
    const samples: number[] = [];

    for (let i = 0; i < count; i++) {
      const startTime = Date.now();

      // Send ping
      await this.sendMessage({
        type: 'PING',
        targetId: targetVehicleId,
        timestamp: startTime
      });

      // Wait for pong (simulated)
      await new Promise(resolve => setTimeout(resolve, 5 + Math.random() * 10));

      const latency = Date.now() - startTime;
      samples.push(latency);
    }

    return {
      min: Math.min(...samples),
      max: Math.max(...samples),
      avg: samples.reduce((a, b) => a + b, 0) / samples.length,
      samples
    };
  }

  /**
   * Get nearby vehicles
   */
  getNearbyVehicles(maxDistance: number = 500): V2VMessage[] {
    const v2vMessages = this.receivedMessages.get('v2v') || [];
    const egoState = this.getEgoVehicleState();

    return (v2vMessages as V2VMessage[]).filter(msg => {
      const distance = this.calculateDistance(egoState.position, msg.position);
      return distance <= maxDistance;
    });
  }

  /**
   * Get connected RSUs
   */
  getConnectedRSUs(): RSUInfo[] {
    // In real implementation, return list of RSUs in range
    return [];
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  private ensureInitialized(): void {
    if (!this.initialized) {
      throw new V2XError(
        V2XErrorCode.INITIALIZATION_FAILED,
        'V2X not initialized. Call initialize() first.'
      );
    }
  }

  private async initializeRadio(): Promise<void> {
    // Initialize V2X radio hardware
    console.log(`Initializing ${this.config.technology} radio at ${this.config.frequency} MHz`);
  }

  private async initializeSecurity(): Promise<void> {
    // Load certificates and initialize security module
    console.log(`Initializing security (level: ${this.config.securityLevel})`);
  }

  private async initializeGNSS(): Promise<void> {
    // Initialize GNSS receiver
    console.log('Initializing GNSS receiver');
  }

  private generateBSM(params: {
    lat: number;
    lon: number;
    speed: number;
    heading: number;
    acceleration?: Acceleration;
  }): BSM {
    this.msgCount = (this.msgCount + 1) % 128;

    return {
      messageType: MessageType.BSM,
      msgCount: this.msgCount,
      id: this.config.vehicleId,
      timestamp: Date.now() % 65536,
      position: {
        latitude: params.lat,
        longitude: params.lon,
        altitude: 0,
        accuracy: {
          semiMajor: 12,
          semiMinor: 10,
          orientation: 0
        }
      },
      speed: params.speed,
      heading: params.heading,
      acceleration: params.acceleration || { longitudinal: 0, lateral: 0, vertical: 0 },
      brakeStatus: {
        wheelBrakes: '00000',
        tractionControl: 'off',
        abs: 'off',
        stabilityControl: 'off',
        brakeBoost: 'off',
        auxBrakes: 'off'
      },
      vehicleSize: {
        width: 2.0,
        length: 4.5
      }
    };
  }

  private getEgoVehicleState(): {
    position: Position;
    speed: number;
    heading: number;
    acceleration: Acceleration;
  } {
    // In real implementation, get from vehicle CAN bus or sensors
    return {
      position: { latitude: 37.7749, longitude: -122.4194, altitude: 10 },
      speed: 65, // km/h
      heading: 270,
      acceleration: { longitudinal: 0, lateral: 0, vertical: 0 }
    };
  }

  private calculateDistance(pos1: Position, pos2: Position): number {
    // Haversine formula for distance calculation
    const R = 6371e3; // Earth radius in meters
    const φ1 = pos1.latitude * Math.PI / 180;
    const φ2 = pos2.latitude * Math.PI / 180;
    const Δφ = (pos2.latitude - pos1.latitude) * Math.PI / 180;
    const Δλ = (pos2.longitude - pos1.longitude) * Math.PI / 180;

    const a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
      Math.cos(φ1) * Math.cos(φ2) *
      Math.sin(Δλ / 2) * Math.sin(Δλ / 2);

    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c; // Distance in meters
  }

  private assessPedestrianRisk(
    egoState: any,
    psm: PSM,
    distance: number
  ): {
    level: 'NONE' | 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
    ttc?: number;
    action?: 'MONITOR' | 'WARN_AND_SLOW' | 'EMERGENCY_BRAKE';
  } {
    // Simple risk assessment based on distance and speed
    if (distance > 50) {
      return { level: 'NONE' };
    }

    const ttc = distance / (egoState.speed / 3.6); // Convert km/h to m/s

    if (ttc < 2.0) {
      return { level: 'CRITICAL', ttc, action: 'EMERGENCY_BRAKE' };
    } else if (ttc < 4.0) {
      return { level: 'HIGH', ttc, action: 'WARN_AND_SLOW' };
    } else if (ttc < 6.0) {
      return { level: 'MEDIUM', ttc, action: 'MONITOR' };
    } else {
      return { level: 'LOW', ttc };
    }
  }

  private mapHazardTypeToCauseCode(type: string): any {
    const mapping: Record<string, any> = {
      'accident': 2,
      'breakdown': 15,
      'hazard': 9,
      'weather': 6
    };
    return mapping[type] || 99;
  }

  private parseStationId(vehicleId: string): number {
    // Convert vehicle ID to station ID
    return parseInt(vehicleId.replace(/\D/g, '').slice(0, 8)) || 123456;
  }

  private generateUUID(): string {
    return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
      const r = Math.random() * 16 | 0;
      const v = c === 'x' ? r : (r & 0x3 | 0x8);
      return v.toString(16);
    });
  }

  private async broadcastMessage(message: any): Promise<void> {
    // In real implementation, broadcast via V2X radio
    console.log('Broadcasting message:', message.messageType || message.type);
  }

  private async sendMessage(message: any): Promise<void> {
    // In real implementation, send via V2X radio
    console.log('Sending message:', message.type);
  }

  private on(event: string, handler: V2XEventHandler): void {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, new Set());
    }
    this.eventHandlers.get(event)!.add(handler);
  }

  private off(event: string): void {
    this.eventHandlers.delete(event);
  }

  private emit(event: string, data: any): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      handlers.forEach(handler => handler(data));
    }
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate distance between two positions
 */
export function calculateDistance(pos1: Position, pos2: Position): number {
  const R = 6371e3; // Earth radius in meters
  const φ1 = pos1.latitude * Math.PI / 180;
  const φ2 = pos2.latitude * Math.PI / 180;
  const Δφ = (pos2.latitude - pos1.latitude) * Math.PI / 180;
  const Δλ = (pos2.longitude - pos1.longitude) * Math.PI / 180;

  const a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
    Math.cos(φ1) * Math.cos(φ2) *
    Math.sin(Δλ / 2) * Math.sin(Δλ / 2);

  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

  return R * c; // Distance in meters
}

/**
 * Calculate bearing between two positions
 */
export function calculateBearing(pos1: Position, pos2: Position): number {
  const φ1 = pos1.latitude * Math.PI / 180;
  const φ2 = pos2.latitude * Math.PI / 180;
  const Δλ = (pos2.longitude - pos1.longitude) * Math.PI / 180;

  const y = Math.sin(Δλ) * Math.cos(φ2);
  const x = Math.cos(φ1) * Math.sin(φ2) -
    Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);

  const θ = Math.atan2(y, x);

  return (θ * 180 / Math.PI + 360) % 360; // Bearing in degrees
}

/**
 * Calculate time to collision
 */
export function calculateTTC(
  distance: number,
  egoSpeed: number,
  targetSpeed: number
): number | null {
  const relativeSpeed = (egoSpeed - targetSpeed) / 3.6; // Convert km/h to m/s

  if (relativeSpeed <= 0) {
    return null; // Target moving away or same speed
  }

  return distance / relativeSpeed;
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { V2XCommunication };
export default V2XCommunication;
