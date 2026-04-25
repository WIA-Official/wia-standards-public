/**
 * WIA-AUTO-002: ADAS - Advanced Driver Assistance System SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Safety Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for Advanced Driver Assistance Systems including:
 * - Sensor fusion
 * - Object detection and tracking
 * - Lane detection and keeping
 * - Collision avoidance
 * - Adaptive cruise control
 */

import {
  ADASConfig,
  ADASState,
  ADASProcessingResult,
  SensorData,
  DetectedObject,
  ObjectList,
  LaneInfo,
  CollisionAvoidanceResult,
  CollisionRisk,
  LaneDepartureStatus,
  ACCStatus,
  ACCState,
  ACCConfig,
  LKAStatus,
  LKAState,
  SafeDistanceParams,
  SafeDistanceResult,
  RoadCondition,
  SAELevel,
  ValidationResult,
  TTCResult,
  SensorFusionResult,
  ADAS_CONSTANTS,
  ADASErrorCode,
  ADASError,
  Vector3,
  ObjectClass,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUTO-002 ADAS System SDK
 */
export class ADASSystem {
  private version = '1.0.0';
  private config: ADASConfig;
  private state: ADASState;
  private lastProcessingTime: number = 0;

  constructor(config: ADASConfig) {
    this.config = config;
    this.state = {
      initialized: false,
      health: 'healthy',
      activeWarnings: [],
      features: {},
      timestamp: Date.now() * 1000,
    };

    this.initialize();
  }

  /**
   * Initialize ADAS system
   */
  private initialize(): void {
    // Validate configuration
    const validation = this.validateConfiguration();
    if (!validation.isValid) {
      throw new ADASError(
        ADASErrorCode.INVALID_INPUT,
        'Invalid ADAS configuration',
        { errors: validation.errors }
      );
    }

    // Initialize feature states
    if (this.config.features?.acc) {
      this.state.features.acc = {
        state: ACCState.OFF,
        egoVelocity: 0,
        commandedAcceleration: 0,
        timestamp: Date.now() * 1000,
      };
    }

    if (this.config.features?.lka) {
      this.state.features.lka = {
        state: LKAState.OFF,
        commandedSteering: 0,
        lateralOffset: 0,
        headingError: 0,
        laneConfidence: 0,
        timestamp: Date.now() * 1000,
      };
    }

    this.state.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Get system state
   */
  getState(): ADASState {
    return { ...this.state };
  }

  /**
   * Validate system configuration
   */
  validateConfiguration(): ValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];

    // Check sensor configuration
    const sensorCount = [
      this.config.sensors.lidar?.enabled,
      this.config.sensors.radar?.enabled,
      this.config.sensors.camera?.enabled,
    ].filter(Boolean).length;

    if (sensorCount < 2) {
      errors.push('At least 2 sensor types required for redundancy');
    }

    // Check SAE level requirements
    if (this.config.safetyLevel >= SAELevel.LEVEL_2) {
      if (!this.config.sensors.camera?.enabled) {
        errors.push('Camera required for SAE Level 2+');
      }
      if (sensorCount < 3) {
        warnings.push('3+ sensors recommended for SAE Level 2+');
      }
    }

    if (this.config.safetyLevel >= SAELevel.LEVEL_3) {
      if (sensorCount < 3) {
        errors.push('At least 3 sensor types required for SAE Level 3+');
      }
      recommendations.push('Consider redundant processing units for SAE Level 3+');
    }

    // Check detection parameters
    const maxRange = this.config.detection?.maxRange || 200;
    if (maxRange > 200) {
      warnings.push('Detection range > 200m may have reduced accuracy');
    }

    const minConfidence = this.config.detection?.minConfidence || 0.7;
    if (minConfidence < 0.7) {
      warnings.push('Confidence threshold < 0.7 may increase false positives');
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      sensorHealth: {
        lidar: this.config.sensors.lidar?.enabled ? 'healthy' : undefined,
        radar: this.config.sensors.radar?.enabled ? 'healthy' : undefined,
        camera: this.config.sensors.camera?.enabled ? 'healthy' : undefined,
        ultrasonic: this.config.sensors.ultrasonic?.enabled ? 'healthy' : undefined,
      },
      recommendations,
    };
  }

  /**
   * Process sensor data through ADAS pipeline
   */
  processSensorData(sensorData: SensorData): ADASProcessingResult {
    const startTime = performance.now();

    if (!this.state.initialized) {
      throw new ADASError(
        ADASErrorCode.SYSTEM_OVERLOAD,
        'System not initialized'
      );
    }

    // Detect objects from sensor data
    const objects = this.detectObjects(sensorData);

    // Detect lanes
    const lanes = this.detectLanes(sensorData);

    // Check collision risk
    const collision = this.checkCollisionRisk({
      objects: objects.objects,
      egoVelocity: 25, // TODO: Get from vehicle state
      roadCondition: RoadCondition.DRY,
    });

    // Check lane departure
    const laneDeparture = this.checkLaneDeparture(lanes);

    // Update ACC if enabled
    let acc: ACCStatus | undefined;
    if (this.config.features?.acc && this.state.features.acc) {
      acc = this.updateACC(objects.objects, this.state.features.acc);
    }

    // Update LKA if enabled
    let lka: LKAStatus | undefined;
    if (this.config.features?.lka && this.state.features.lka) {
      lka = this.updateLKA(lanes, this.state.features.lka);
    }

    const processingTime = performance.now() - startTime;
    this.lastProcessingTime = processingTime;

    return {
      objects: objects.objects,
      lanes,
      collision,
      laneDeparture,
      acc,
      lka,
      systemState: this.state,
      timestamp: sensorData.timestamp,
    };
  }

  /**
   * Detect objects from sensor data
   */
  detectObjects(sensorData: SensorData): ObjectList {
    const objects: DetectedObject[] = [];

    // Simplified object detection (in real implementation, use sensor fusion)
    // This is a placeholder that would integrate LiDAR, Radar, and Camera data

    if (sensorData.radar) {
      // Convert radar targets to detected objects
      sensorData.radar.targets.forEach((target, idx) => {
        const x = target.range * Math.cos(target.azimuth);
        const y = target.range * Math.sin(target.azimuth);

        objects.push({
          id: 1000 + idx,
          class: ObjectClass.CAR, // Simplified classification
          classConfidence: 0.8,
          position: { x, y, z: 0 },
          velocity: { x: target.rangeRate, y: 0, z: 0 },
          acceleration: { x: 0, y: 0, z: 0 },
          dimensions: { length: 4.5, width: 1.8, height: 1.5 },
          heading: 0,
          trackingAge: 1.0,
        });
      });
    }

    return {
      objects,
      timestamp: sensorData.timestamp,
      frameId: 'base_link',
    };
  }

  /**
   * Detect lanes from camera data
   */
  detectLanes(sensorData: SensorData): LaneInfo {
    // Simplified lane detection (placeholder)
    // In real implementation, use camera image processing

    return {
      egoLeft: {
        coefficients: [1.75, 0.001, -0.00001, 0],
        type: 'dashed' as const,
        color: 'white' as const,
        confidence: 0.9,
        validRange: { yMin: 0, yMax: 80 },
      },
      egoRight: {
        coefficients: [-1.75, 0.001, -0.00001, 0],
        type: 'solid' as const,
        color: 'white' as const,
        confidence: 0.9,
        validRange: { yMin: 0, yMax: 80 },
      },
      egoPosition: {
        lateralOffset: 0,
        headingAngle: 0,
      },
      timestamp: sensorData.timestamp,
    };
  }

  /**
   * Check collision risk
   */
  checkCollisionRisk(params: {
    objects: DetectedObject[];
    egoVelocity: number;
    roadCondition: RoadCondition;
  }): CollisionAvoidanceResult {
    const { objects, egoVelocity, roadCondition } = params;

    // Find closest object in ego lane
    const inPathObjects = objects.filter(
      (obj) => obj.position.x > 0 && Math.abs(obj.position.y) < 2.0
    );

    if (inPathObjects.length === 0) {
      return {
        shouldBrake: false,
        brakingForce: 0,
        ttc: {
          ttc: Infinity,
          distance: Infinity,
          relativeVelocity: 0,
          objectId: -1,
          risk: CollisionRisk.NONE,
        },
        safeDistance: this.calculateSafeDistance({
          velocity: egoVelocity,
          roadCondition,
        }).totalDistance,
        warningLevel: 'none',
      };
    }

    // Get closest object
    const closest = inPathObjects.reduce((prev, curr) =>
      curr.position.x < prev.position.x ? curr : prev
    );

    // Calculate TTC
    const relativeVelocity = egoVelocity - closest.velocity.x;
    const safeDistResult = this.calculateSafeDistance({
      velocity: egoVelocity,
      roadCondition,
    });

    const ttc = this.calculateTTC({
      distance: closest.position.x,
      egoVelocity,
      targetVelocity: closest.velocity.x,
      safeDistance: safeDistResult.totalDistance,
    });

    // Determine braking action
    let shouldBrake = false;
    let brakingForce = 0;
    let warningLevel: CollisionAvoidanceResult['warningLevel'] = 'none';

    if (ttc.risk === CollisionRisk.CRITICAL) {
      shouldBrake = true;
      brakingForce = 1.0;
      warningLevel = 'emergency';
    } else if (ttc.risk === CollisionRisk.HIGH) {
      shouldBrake = true;
      brakingForce = 0.6;
      warningLevel = 'haptic';
    } else if (ttc.risk === CollisionRisk.MEDIUM) {
      brakingForce = 0.3;
      warningLevel = 'audible';
    } else if (ttc.risk === CollisionRisk.LOW) {
      warningLevel = 'visual';
    }

    return {
      shouldBrake,
      brakingForce,
      ttc,
      safeDistance: safeDistResult.totalDistance,
      warningLevel,
    };
  }

  /**
   * Calculate Time-to-Collision
   */
  calculateTTC(params: {
    distance: number;
    egoVelocity: number;
    targetVelocity: number;
    safeDistance: number;
  }): TTCResult {
    const { distance, egoVelocity, targetVelocity, safeDistance } = params;

    const relativeVelocity = egoVelocity - targetVelocity;

    // Calculate TTC
    let ttc: number;
    if (relativeVelocity <= 0) {
      // Not approaching or target moving away
      ttc = Infinity;
    } else {
      ttc = (distance - safeDistance) / relativeVelocity;
    }

    // Determine risk level
    let risk: CollisionRisk;
    if (ttc < 0 || distance < safeDistance) {
      risk = CollisionRisk.CRITICAL;
    } else if (ttc < ADAS_CONSTANTS.TTC_THRESHOLDS.BRAKE) {
      risk = CollisionRisk.CRITICAL;
    } else if (ttc < ADAS_CONSTANTS.TTC_THRESHOLDS.ALERT) {
      risk = CollisionRisk.HIGH;
    } else if (ttc < ADAS_CONSTANTS.TTC_THRESHOLDS.WARN) {
      risk = CollisionRisk.MEDIUM;
    } else if (ttc < ADAS_CONSTANTS.TTC_THRESHOLDS.MONITOR) {
      risk = CollisionRisk.LOW;
    } else {
      risk = CollisionRisk.NONE;
    }

    return {
      ttc: Math.max(0, ttc),
      distance,
      relativeVelocity,
      objectId: -1, // TODO: Pass object ID
      risk,
    };
  }

  /**
   * Calculate safe following distance
   */
  calculateSafeDistance(params: SafeDistanceParams): SafeDistanceResult {
    const {
      velocity,
      roadCondition,
      reactionTime = ADAS_CONSTANTS.REACTION_TIME,
      safetyMargin = 1.2,
    } = params;

    // Get friction coefficient
    let frictionCoefficient: number;
    switch (roadCondition) {
      case RoadCondition.DRY:
        frictionCoefficient = ADAS_CONSTANTS.FRICTION.DRY_ASPHALT;
        break;
      case RoadCondition.WET:
        frictionCoefficient = ADAS_CONSTANTS.FRICTION.WET_ASPHALT;
        break;
      case RoadCondition.SNOW:
        frictionCoefficient = ADAS_CONSTANTS.FRICTION.SNOW;
        break;
      case RoadCondition.ICE:
        frictionCoefficient = ADAS_CONSTANTS.FRICTION.ICE;
        break;
      default:
        frictionCoefficient = ADAS_CONSTANTS.FRICTION.WET_ASPHALT; // Conservative
    }

    // Calculate reaction distance
    const reactionDistance = velocity * reactionTime;

    // Calculate maximum deceleration
    const maxDeceleration = frictionCoefficient * ADAS_CONSTANTS.GRAVITY;

    // Calculate braking distance
    const brakingDistance = (velocity * velocity) / (2 * maxDeceleration);

    // Total safe distance with safety margin
    const totalDistance =
      (reactionDistance + brakingDistance) * safetyMargin +
      ADAS_CONSTANTS.SAFETY.MIN_DISTANCE;

    return {
      totalDistance,
      reactionDistance,
      brakingDistance,
      frictionCoefficient,
      maxDeceleration,
    };
  }

  /**
   * Check lane departure
   */
  checkLaneDeparture(lanes: LaneInfo): LaneDepartureStatus {
    const { lateralOffset, headingAngle } = lanes.egoPosition;

    // Estimate lane width (distance between left and right boundaries at y=10m)
    const y = 10;
    let laneWidth = 3.5; // Default

    if (lanes.egoLeft && lanes.egoRight) {
      const leftX = this.evaluatePolynomial(lanes.egoLeft.coefficients, y);
      const rightX = this.evaluatePolynomial(lanes.egoRight.coefficients, y);
      laneWidth = Math.abs(leftX - rightX);
    }

    // Calculate distance to lane edges
    const distToLeftEdge = laneWidth / 2 - lateralOffset;
    const distToRightEdge = laneWidth / 2 + lateralOffset;

    // Estimate vehicle velocity (placeholder - should come from vehicle state)
    const velocity = 25; // m/s

    // Calculate TLC (Time to Lane Crossing)
    let tlcLeft = Infinity;
    let tlcRight = Infinity;

    if (headingAngle > 0) {
      // Drifting left
      const lateralVelocity = velocity * Math.sin(headingAngle);
      if (lateralVelocity > 0.01) {
        tlcLeft = distToLeftEdge / lateralVelocity;
      }
    } else if (headingAngle < 0) {
      // Drifting right
      const lateralVelocity = velocity * Math.abs(Math.sin(headingAngle));
      if (lateralVelocity > 0.01) {
        tlcRight = distToRightEdge / lateralVelocity;
      }
    }

    const tlc = Math.min(tlcLeft, tlcRight);
    const direction =
      tlcLeft < tlcRight ? 'left' : tlcRight < tlcLeft ? 'right' : 'none';

    // Determine warning level
    let warningLevel: LaneDepartureStatus['warningLevel'] = 'none';
    let isDeparting = false;

    if (tlc < 0.5) {
      warningLevel = 'audible';
      isDeparting = true;
    } else if (tlc < 1.0) {
      warningLevel = 'haptic';
      isDeparting = true;
    } else if (tlc < 2.0) {
      warningLevel = 'visual';
      isDeparting = true;
    }

    return {
      isDeparting,
      tlc,
      direction: direction as 'left' | 'right' | 'none',
      warningLevel,
    };
  }

  /**
   * Update Adaptive Cruise Control
   */
  private updateACC(objects: DetectedObject[], currentStatus: ACCStatus): ACCStatus {
    // Find target vehicle in ego lane
    const inPathObjects = objects.filter(
      (obj) => obj.position.x > 0 && Math.abs(obj.position.y) < 2.0
    );

    const timestamp = Date.now() * 1000;

    if (inPathObjects.length === 0) {
      // No target - cruise mode
      return {
        ...currentStatus,
        state: ACCState.CRUISE,
        targetObject: undefined,
        timestamp,
      };
    }

    // Get closest object
    const target = inPathObjects.reduce((prev, curr) =>
      curr.position.x < prev.position.x ? curr : prev
    );

    return {
      ...currentStatus,
      state: ACCState.FOLLOWING,
      targetObject: {
        id: target.id,
        distance: target.position.x,
        relativeVelocity: target.velocity.x - currentStatus.egoVelocity,
      },
      timestamp,
    };
  }

  /**
   * Update Lane Keeping Assist
   */
  private updateLKA(lanes: LaneInfo, currentStatus: LKAStatus): LKAStatus {
    const { lateralOffset, headingAngle } = lanes.egoPosition;

    // Simple PID control (proportional only for this example)
    const Kp = 0.5;
    const Kd = 0.2;

    const commandedSteering = -(Kp * lateralOffset + Kd * headingAngle);

    return {
      ...currentStatus,
      state: LKAState.ACTIVE,
      commandedSteering,
      lateralOffset,
      headingError: headingAngle,
      laneConfidence:
        Math.min(
          lanes.egoLeft?.confidence || 0,
          lanes.egoRight?.confidence || 0
        ) || 0,
      timestamp: Date.now() * 1000,
    };
  }

  /**
   * Activate emergency braking
   */
  activateEmergencyBraking(): void {
    // In real implementation, this would send command to vehicle actuators
    this.state.activeWarnings.push('EMERGENCY_BRAKING_ACTIVE');
  }

  /**
   * Evaluate polynomial at given y value
   */
  private evaluatePolynomial(coefficients: number[], y: number): number {
    let result = 0;
    for (let i = 0; i < coefficients.length; i++) {
      result += coefficients[i] * Math.pow(y, i);
    }
    return result;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate safe following distance (standalone function)
 */
export function calculateSafeDistance(params: SafeDistanceParams): SafeDistanceResult {
  const system = new ADASSystem({
    sensors: {},
    safetyLevel: SAELevel.LEVEL_0,
  });
  return system.calculateSafeDistance(params);
}

/**
 * Calculate Time-to-Collision (standalone function)
 */
export function calculateTTC(params: {
  distance: number;
  egoVelocity: number;
  targetVelocity: number;
  safeDistance: number;
}): TTCResult {
  const system = new ADASSystem({
    sensors: {},
    safetyLevel: SAELevel.LEVEL_0,
  });
  return system.calculateTTC(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { ADASSystem };
export default ADASSystem;
