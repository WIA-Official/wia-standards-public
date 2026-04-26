/**
 * WIA-DEF-003: Military Robot SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for military robotics systems including:
 * - Robot configuration and management
 * - Mission planning and validation
 * - Efficiency calculations
 * - Safety checks and diagnostics
 */

import {
  MilitaryRobot,
  RobotType,
  WeightClass,
  LocomotionType,
  AutonomyLevel,
  RobotStatus,
  MobilitySpecs,
  SensorSuite,
  PowerSystem,
  CommunicationSystem,
  Manipulator,
  MissionParams,
  MissionType,
  MissionStatus,
  MissionValidation,
  SafetyCheck,
  MissionProgress,
  MissionEvent,
  Waypoint,
  GeoCoordinate,
  RobotPosition,
  EfficiencyParams,
  EfficiencyResult,
  RobotTelemetry,
  DiagnosticReport,
  ComponentStatus,
  DefenseErrorCode,
  MilitaryRobotError,
  DEFENSE_CONSTANTS,
  RulesOfEngagement,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-003 Military Robot SDK
 */
export class MilitaryRobotSDK {
  private version = '1.0.0';
  private robots: Map<string, MilitaryRobot> = new Map();
  private missions: Map<string, MissionParams> = new Map();

  constructor() {
    // SDK initialized
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ==========================================================================
  // Robot Management
  // ==========================================================================

  /**
   * Create a new robot configuration
   *
   * @param params - Robot creation parameters
   * @returns Configured military robot
   */
  createRobot(params: {
    type: RobotType | string;
    weight?: number;
    autonomyLevel?: AutonomyLevel | number;
    payload?: number;
    range?: number;
  }): MilitaryRobot {
    const type = this.parseRobotType(params.type);
    const autonomyLevel = params.autonomyLevel ?? this.getDefaultAutonomy(type);

    // Generate robot ID
    const id = this.generateRobotId(type);

    // Determine weight class
    const weight = params.weight ?? this.getDefaultWeight(type);
    const weightClass = this.determineWeightClass(weight);

    // Create robot based on type
    const robot: MilitaryRobot = {
      id,
      type,
      weightClass,
      weight,
      autonomyLevel:
        typeof autonomyLevel === 'number'
          ? autonomyLevel
          : this.parseAutonomyLevel(autonomyLevel),
      mobility: this.createMobilitySpecs(type, weight),
      sensors: this.createSensorSuite(type),
      power: this.createPowerSystem(type, weight),
      communication: this.createCommunicationSystem(type),
      dimensions: this.estimateDimensions(weight),
      status: RobotStatus.Idle,
      batteryLevel: 100,
    };

    // Add type-specific features
    if (type === RobotType.EOD) {
      robot.manipulator = this.createManipulator('heavy-eod');
    } else if (type === RobotType.Logistics) {
      robot.payloadCapacity = params.payload ?? 250;
    }

    // Store robot
    this.robots.set(id, robot);

    return robot;
  }

  /**
   * Get robot by ID
   */
  getRobot(id: string): MilitaryRobot | undefined {
    return this.robots.get(id);
  }

  /**
   * Update robot status
   */
  updateRobotStatus(
    robotId: string,
    status: RobotStatus,
    position?: RobotPosition,
    batteryLevel?: number
  ): void {
    const robot = this.robots.get(robotId);
    if (!robot) {
      throw new MilitaryRobotError(
        DefenseErrorCode.INVALID_ROBOT,
        `Robot ${robotId} not found`
      );
    }

    robot.status = status;
    if (position) robot.position = position;
    if (batteryLevel !== undefined) robot.batteryLevel = batteryLevel;
  }

  // ==========================================================================
  // Mission Planning & Validation
  // ==========================================================================

  /**
   * Validate a mission before execution
   *
   * @param mission - Mission parameters
   * @returns Validation result
   */
  validateMission(mission: MissionParams): MissionValidation {
    const errors: string[] = [];
    const warnings: string[] = [];
    const safetyChecks: SafetyCheck[] = [];
    const recommendations: string[] = [];

    // Get robot
    const robot = this.robots.get(mission.robotId);
    if (!robot) {
      errors.push(`Robot ${mission.robotId} not found`);
      return {
        isValid: false,
        errors,
        warnings,
        safetyChecks,
        riskLevel: 'extreme',
        recommendations,
      };
    }

    // Check autonomy level
    safetyChecks.push({
      name: 'Autonomy Level',
      status: robot.autonomyLevel >= mission.requiredAutonomy ? 'pass' : 'fail',
      description: 'Verify robot autonomy meets mission requirements',
      value: robot.autonomyLevel,
      expected: mission.requiredAutonomy,
      correctiveAction: 'Upgrade robot autonomy or assign different robot',
    });

    if (robot.autonomyLevel < mission.requiredAutonomy) {
      errors.push(
        `Robot autonomy level (${robot.autonomyLevel}) insufficient for mission (requires ${mission.requiredAutonomy})`
      );
    }

    // Check robot type compatibility
    const typeCompatible = this.checkTypeCompatibility(robot.type, mission.type);
    safetyChecks.push({
      name: 'Robot Type Compatibility',
      status: typeCompatible ? 'pass' : 'warning',
      description: 'Check if robot type is suitable for mission',
      value: robot.type,
      expected: mission.type,
      correctiveAction: 'Consider using a robot designed for this mission type',
    });

    if (!typeCompatible) {
      warnings.push(
        `Robot type (${robot.type}) may not be optimal for ${mission.type} mission`
      );
    }

    // Calculate mission distance and energy
    const distance = this.calculateMissionDistance(mission.waypoints);
    const estimatedEnergy = this.estimateEnergyConsumption(robot, distance);
    const estimatedDuration = this.estimateMissionDuration(robot, mission.waypoints);

    // Check battery
    const currentBattery = robot.batteryLevel ?? 100;
    const availableEnergy = (robot.power.capacity * currentBattery) / 100;
    const energyMargin = availableEnergy - estimatedEnergy;
    const batterySufficient = energyMargin > robot.power.capacity * 0.2; // 20% margin

    safetyChecks.push({
      name: 'Battery Capacity',
      status: batterySufficient ? 'pass' : 'fail',
      description: 'Ensure sufficient battery for mission + 20% reserve',
      value: availableEnergy,
      expected: estimatedEnergy * 1.2,
      correctiveAction: 'Charge battery or reduce mission scope',
    });

    if (!batterySufficient) {
      errors.push(
        `Insufficient battery: need ${(estimatedEnergy * 1.2).toFixed(1)} Wh, have ${availableEnergy.toFixed(1)} Wh`
      );
    } else if (energyMargin < robot.power.capacity * 0.3) {
      warnings.push('Battery margin is low (<30%). Consider charging before mission.');
    }

    // Check communication range
    const maxDistance = Math.max(
      ...mission.waypoints.map((wp) => this.calculateDistance(wp.location, mission.waypoints[0].location))
    );

    const withinRange = maxDistance <= robot.communication.maxRange;
    safetyChecks.push({
      name: 'Communication Range',
      status: withinRange ? 'pass' : 'fail',
      description: 'Verify all waypoints within communication range',
      value: maxDistance,
      expected: robot.communication.maxRange,
      correctiveAction: 'Reduce mission range or use mesh networking',
    });

    if (!withinRange) {
      errors.push(
        `Waypoint exceeds communication range: ${maxDistance.toFixed(0)}m > ${robot.communication.maxRange}m`
      );
    }

    // Check mission duration
    const maxDuration = mission.maxDuration ?? DEFENSE_CONSTANTS.MAX_MISSION_DURATION * 3600;
    safetyChecks.push({
      name: 'Mission Duration',
      status: estimatedDuration <= maxDuration ? 'pass' : 'warning',
      description: 'Check mission duration is reasonable',
      value: estimatedDuration,
      expected: maxDuration,
      correctiveAction: 'Split mission into multiple phases',
    });

    if (estimatedDuration > maxDuration) {
      warnings.push(
        `Mission duration (${(estimatedDuration / 3600).toFixed(1)}h) exceeds recommended maximum`
      );
    }

    // Check ROE compliance
    if (mission.roe) {
      const roeCheck = this.validateRulesOfEngagement(robot, mission.roe);
      safetyChecks.push(roeCheck);
      if (roeCheck.status === 'fail') {
        errors.push(`ROE validation failed: ${roeCheck.description}`);
      }
    }

    // Determine risk level
    let riskLevel: MissionValidation['riskLevel'];
    if (errors.length > 0) {
      riskLevel = 'extreme';
    } else if (warnings.length >= 3) {
      riskLevel = 'high';
    } else if (warnings.length > 0) {
      riskLevel = 'medium';
    } else {
      riskLevel = 'low';
    }

    // Generate recommendations
    if (mission.type === MissionType.EOD && robot.autonomyLevel > AutonomyLevel.Teleoperation) {
      recommendations.push('EOD missions should use teleoperation (autonomy level 1) for safety');
    }

    if (distance > 10000 && robot.type !== RobotType.Logistics) {
      recommendations.push('Long-range mission: consider using a logistics robot');
    }

    if (mission.waypoints.length > 10) {
      recommendations.push('Complex mission: verify all waypoints are necessary');
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      safetyChecks,
      estimatedDuration,
      estimatedEnergy,
      batterySufficient,
      riskLevel,
      recommendations,
    };
  }

  /**
   * Create a new mission
   */
  createMission(params: {
    robotId: string;
    type: MissionType;
    waypoints: Waypoint[];
    requiredAutonomy?: AutonomyLevel;
    operatorId: string;
    roe?: RulesOfEngagement;
  }): MissionParams {
    const mission: MissionParams = {
      id: this.generateMissionId(),
      type: params.type,
      robotId: params.robotId,
      waypoints: params.waypoints,
      requiredAutonomy: params.requiredAutonomy ?? AutonomyLevel.SupervisedAutonomy,
      operatorId: params.operatorId,
      roe: params.roe,
      status: MissionStatus.Pending,
    };

    this.missions.set(mission.id, mission);
    return mission;
  }

  /**
   * Approve a mission
   */
  approveMission(missionId: string): void {
    const mission = this.missions.get(missionId);
    if (!mission) {
      throw new MilitaryRobotError(
        DefenseErrorCode.MISSION_VALIDATION_FAILED,
        `Mission ${missionId} not found`
      );
    }

    const validation = this.validateMission(mission);
    if (!validation.isValid) {
      throw new MilitaryRobotError(
        DefenseErrorCode.MISSION_VALIDATION_FAILED,
        `Mission validation failed: ${validation.errors.join('; ')}`
      );
    }

    mission.status = MissionStatus.Approved;
    mission.startTime = new Date();
  }

  // ==========================================================================
  // Efficiency Calculations
  // ==========================================================================

  /**
   * Calculate efficiency metrics for a robot operation
   *
   * @param params - Efficiency calculation parameters
   * @returns Efficiency metrics
   */
  calculateEfficiency(params: EfficiencyParams): EfficiencyResult {
    const { robotType, payload, distance, terrain, speed, manipulatorUsage } = params;

    // Base energy consumption (Wh/km)
    const baseConsumption = this.getBaseConsumption(robotType);

    // Terrain factor
    const terrainFactor = this.getTerrainFactor(terrain);

    // Payload factor (increases with payload)
    const payloadFactor = 1 + payload / 500; // 500 kg reference

    // Speed factor
    const avgSpeed = speed ?? this.getDefaultSpeed(robotType);
    const speedFactor = avgSpeed / 5; // 5 m/s reference

    // Manipulator factor
    const manipFactor = manipulatorUsage ? 1 + manipulatorUsage / 100 : 1;

    // Total energy consumption (Wh)
    const energyConsumption =
      (baseConsumption * (distance / 1000) * terrainFactor * payloadFactor * speedFactor * manipFactor);

    // Mission duration (seconds)
    const duration = distance / avgSpeed;

    // Efficiency metric (payload-km per Wh)
    const efficiency = (payload * distance) / 1000 / energyConsumption;

    // Battery cycles required
    const robotPower = this.getTypicalBatteryCapacity(robotType);
    const batteryCycles = Math.ceil(energyConsumption / robotPower);

    // Human equivalent
    const humanEquivalent = this.calculateHumanEquivalent(
      robotType,
      payload,
      distance,
      duration
    );

    return {
      energyConsumption,
      duration,
      efficiency,
      batteryCycles,
      humanEquivalent,
    };
  }

  // ==========================================================================
  // Telemetry & Diagnostics
  // ==========================================================================

  /**
   * Generate diagnostic report for a robot
   */
  generateDiagnosticReport(robotId: string): DiagnosticReport {
    const robot = this.robots.get(robotId);
    if (!robot) {
      throw new MilitaryRobotError(
        DefenseErrorCode.INVALID_ROBOT,
        `Robot ${robotId} not found`
      );
    }

    // Calculate overall health
    const batteryHealth = robot.batteryLevel ?? 100;
    const overallHealth = batteryHealth; // Simplified

    // Component statuses
    const components = {
      mobility: {
        name: 'Mobility System',
        status: 'operational' as const,
        health: 95,
      },
      sensors: {
        name: 'Sensor Suite',
        status: 'operational' as const,
        health: 98,
      },
      communication: {
        name: 'Communication System',
        status: 'operational' as const,
        health: 100,
      },
      power: {
        name: 'Power System',
        status: batteryHealth > 30 ? ('operational' as const) : ('degraded' as const),
        health: batteryHealth,
      },
    };

    if (robot.manipulator) {
      components['manipulator'] = {
        name: 'Manipulator',
        status: 'operational',
        health: 97,
      };
    }

    const faults = [];
    const maintenance = [];

    // Check for low battery
    if (batteryHealth < DEFENSE_CONSTANTS.LOW_BATTERY_THRESHOLD) {
      faults.push({
        code: 'PWR-001',
        severity: 'warning' as const,
        description: 'Battery level low',
        firstOccurrence: new Date(),
        count: 1,
        action: 'Charge battery',
        cleared: false,
      });
      maintenance.push('Charge battery to >80%');
    }

    // Determine system status
    let systemStatus: DiagnosticReport['systemStatus'];
    if (faults.some((f) => f.severity === 'critical')) {
      systemStatus = 'critical';
    } else if (faults.some((f) => f.severity === 'error')) {
      systemStatus = 'failed';
    } else if (faults.length > 0) {
      systemStatus = 'degraded';
    } else {
      systemStatus = 'healthy';
    }

    return {
      robotId,
      timestamp: new Date(),
      health: overallHealth,
      systemStatus,
      components,
      faults,
      maintenance,
    };
  }

  // ==========================================================================
  // Helper Methods
  // ==========================================================================

  private parseRobotType(type: RobotType | string): RobotType {
    if (Object.values(RobotType).includes(type as RobotType)) {
      return type as RobotType;
    }

    // Try to match common aliases
    const lowerType = type.toLowerCase();
    if (lowerType.includes('recon') || lowerType.includes('scout')) {
      return RobotType.Reconnaissance;
    }
    if (lowerType.includes('eod') || lowerType.includes('bomb')) {
      return RobotType.EOD;
    }
    if (lowerType.includes('logistic') || lowerType.includes('transport')) {
      return RobotType.Logistics;
    }
    if (lowerType.includes('support') || lowerType.includes('combat')) {
      return RobotType.CombatSupport;
    }

    throw new MilitaryRobotError(
      DefenseErrorCode.INVALID_ROBOT,
      `Unknown robot type: ${type}`
    );
  }

  private parseAutonomyLevel(level: AutonomyLevel | number): AutonomyLevel {
    if (typeof level === 'number') {
      return level as AutonomyLevel;
    }
    return level;
  }

  private getDefaultAutonomy(type: RobotType): AutonomyLevel {
    switch (type) {
      case RobotType.Reconnaissance:
        return AutonomyLevel.SupervisedAutonomy;
      case RobotType.EOD:
        return AutonomyLevel.Teleoperation;
      case RobotType.Logistics:
        return AutonomyLevel.HighAutonomy;
      case RobotType.CombatSupport:
        return AutonomyLevel.SupervisedAutonomy;
    }
  }

  private getDefaultWeight(type: RobotType): number {
    switch (type) {
      case RobotType.Reconnaissance:
        return 25;
      case RobotType.EOD:
        return 150;
      case RobotType.Logistics:
        return 300;
      case RobotType.CombatSupport:
        return 100;
    }
  }

  private determineWeightClass(weight: number): WeightClass {
    if (weight < 5) return WeightClass.Micro;
    if (weight < 25) return WeightClass.Light;
    if (weight < 150) return WeightClass.Medium;
    if (weight < 500) return WeightClass.Heavy;
    return WeightClass.SuperHeavy;
  }

  private createMobilitySpecs(type: RobotType, weight: number): MobilitySpecs {
    // Simplified mobility specs based on type and weight
    const locomotion =
      type === RobotType.EOD ? LocomotionType.Tracked : LocomotionType.Wheeled;

    return {
      type: locomotion,
      maxSpeed: weight < 100 ? 5 : 3,
      maxSlope: locomotion === LocomotionType.Tracked ? 40 : 25,
      maxObstacleHeight: weight / 200, // Rough estimate
      canClimbStairs: locomotion === LocomotionType.Tracked,
      turningRadius: weight / 100,
    };
  }

  private createSensorSuite(type: RobotType): SensorSuite {
    const base: SensorSuite = {
      cameras: ['RGB', 'thermal'],
      hasLIDAR: true,
      navigation: {
        gps: true,
        imu: true,
      },
    };

    if (type === RobotType.EOD) {
      base.cameras.push('night-vision');
      base.hasXRay = true;
      base.environmental = {
        chemical: true,
        radiation: true,
        explosive: true,
      };
    } else if (type === RobotType.Reconnaissance) {
      base.cameras.push('360-panoramic', 'night-vision');
    }

    return base;
  }

  private createPowerSystem(type: RobotType, weight: number): PowerSystem {
    // Energy density: ~150 Wh/kg for batteries
    const batteryWeight = weight * 0.15; // 15% of robot weight
    const capacity = batteryWeight * 150;

    const consumption = {
      idle: weight * 0.5,
      cruising: weight * 3,
      peak: weight * 10,
    };

    const endurance = capacity / consumption.cruising;

    return {
      batteryType: 'Li-ion',
      capacity,
      voltage: 48,
      endurance,
      chargingTime: 2,
      hotSwap: weight > 100,
      consumption,
    };
  }

  private createCommunicationSystem(type: RobotType): CommunicationSystem {
    return {
      frequencies: [0.9, 2.4, 5.8],
      maxRange: type === RobotType.Logistics ? 10000 : 2000,
      bandwidth: 50,
      encryption: 'AES-256',
      meshCapable: true,
      maxLatency: 100,
    };
  }

  private createManipulator(variant: string): Manipulator {
    if (variant === 'heavy-eod') {
      return {
        dof: 7,
        reach: 1.5,
        payload: 50,
        hasForceFeedback: true,
        endEffector: 'gripper',
        tools: ['disruptor', 'xray', 'cutter', 'hook'],
        accuracy: 2,
      };
    }

    return {
      dof: 6,
      reach: 1.0,
      payload: 20,
      hasForceFeedback: false,
      endEffector: 'gripper',
    };
  }

  private estimateDimensions(weight: number): { length: number; width: number; height: number } {
    // Rough estimate based on weight
    const volume = weight / 200; // Assume 200 kg/m³ average density
    const side = Math.pow(volume, 1 / 3);

    return {
      length: side * 1.5,
      width: side,
      height: side * 0.8,
    };
  }

  private generateRobotId(type: RobotType): string {
    const prefix = type.substring(0, 3).toUpperCase();
    const timestamp = Date.now().toString(36);
    const random = Math.random().toString(36).substring(2, 6);
    return `${prefix}-${timestamp}-${random}`;
  }

  private generateMissionId(): string {
    const date = new Date().toISOString().split('T')[0].replace(/-/g, '');
    const random = Math.random().toString(36).substring(2, 8).toUpperCase();
    return `MISSION-${date}-${random}`;
  }

  private checkTypeCompatibility(robotType: RobotType, missionType: MissionType): boolean {
    const compatibility: Record<RobotType, MissionType[]> = {
      [RobotType.Reconnaissance]: [MissionType.Reconnaissance, MissionType.Patrol, MissionType.Search],
      [RobotType.EOD]: [MissionType.EOD],
      [RobotType.Logistics]: [MissionType.Logistics, MissionType.Transport],
      [RobotType.CombatSupport]: [MissionType.Support, MissionType.Patrol],
    };

    return compatibility[robotType]?.includes(missionType) ?? false;
  }

  private calculateMissionDistance(waypoints: Waypoint[]): number {
    let total = 0;
    for (let i = 1; i < waypoints.length; i++) {
      total += this.calculateDistance(waypoints[i - 1].location, waypoints[i].location);
    }
    return total;
  }

  private calculateDistance(a: GeoCoordinate, b: GeoCoordinate): number {
    // Haversine formula for great-circle distance
    const R = 6371000; // Earth radius in meters
    const φ1 = (a.latitude * Math.PI) / 180;
    const φ2 = (b.latitude * Math.PI) / 180;
    const Δφ = ((b.latitude - a.latitude) * Math.PI) / 180;
    const Δλ = ((b.longitude - a.longitude) * Math.PI) / 180;

    const x =
      Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
      Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
    const c = 2 * Math.atan2(Math.sqrt(x), Math.sqrt(1 - x));

    return R * c;
  }

  private estimateEnergyConsumption(robot: MilitaryRobot, distance: number): number {
    // Simple estimate: cruising power × time
    const speed = robot.mobility.maxSpeed * 0.7; // 70% of max speed
    const time = distance / speed; // seconds
    const power = robot.power.consumption.cruising; // watts
    return (power * time) / 3600; // Wh
  }

  private estimateMissionDuration(robot: MilitaryRobot, waypoints: Waypoint[]): number {
    const distance = this.calculateMissionDistance(waypoints);
    const speed = robot.mobility.maxSpeed * 0.6; // 60% of max speed (conservative)
    const travelTime = distance / speed;

    // Add time for waypoint actions
    const actionTime = waypoints.reduce((sum, wp) => sum + (wp.duration ?? 0), 0);

    return travelTime + actionTime;
  }

  private validateRulesOfEngagement(
    robot: MilitaryRobot,
    roe: RulesOfEngagement
  ): SafetyCheck {
    // Check if robot autonomy is compatible with ROE
    if (roe.requireHumanAuth && robot.autonomyLevel >= AutonomyLevel.FullAutonomy) {
      return {
        name: 'Rules of Engagement',
        status: 'fail',
        description: 'ROE requires human authorization but robot autonomy is too high',
        value: robot.autonomyLevel,
        expected: AutonomyLevel.HighAutonomy,
        correctiveAction: 'Reduce robot autonomy level or modify ROE',
      };
    }

    return {
      name: 'Rules of Engagement',
      status: 'pass',
      description: 'Robot configuration complies with ROE',
    };
  }

  private getBaseConsumption(type: RobotType): number {
    // Base energy consumption in Wh/km
    switch (type) {
      case RobotType.Reconnaissance:
        return 50;
      case RobotType.EOD:
        return 150;
      case RobotType.Logistics:
        return 200;
      case RobotType.CombatSupport:
        return 100;
    }
  }

  private getTerrainFactor(terrain: string): number {
    const factors: Record<string, number> = {
      paved: 1.0,
      dirt: 1.2,
      sand: 1.8,
      mud: 2.0,
      snow: 1.5,
      rubble: 2.5,
    };
    return factors[terrain] ?? 1.5;
  }

  private getDefaultSpeed(type: RobotType): number {
    switch (type) {
      case RobotType.Reconnaissance:
        return 5;
      case RobotType.EOD:
        return 2;
      case RobotType.Logistics:
        return 4;
      case RobotType.CombatSupport:
        return 3;
    }
  }

  private getTypicalBatteryCapacity(type: RobotType): number {
    switch (type) {
      case RobotType.Reconnaissance:
        return 500;
      case RobotType.EOD:
        return 2000;
      case RobotType.Logistics:
        return 5000;
      case RobotType.CombatSupport:
        return 1500;
    }
  }

  private calculateHumanEquivalent(
    type: RobotType,
    payload: number,
    distance: number,
    duration: number
  ): EfficiencyResult['humanEquivalent'] {
    // Rough estimates
    let personnel: number;
    let timeRatio: number;
    let riskReduction: number;

    switch (type) {
      case RobotType.EOD:
        personnel = 3; // EOD team
        timeRatio = 0.8; // Robot slightly faster
        riskReduction = 99; // Huge risk reduction
        break;
      case RobotType.Logistics:
        personnel = Math.ceil(payload / 50); // 50 kg per person
        timeRatio = 0.5; // Robot much faster
        riskReduction = 70; // Significant risk reduction
        break;
      case RobotType.Reconnaissance:
        personnel = 2; // Scout team
        timeRatio = 0.6;
        riskReduction = 80;
        break;
      default:
        personnel = 2;
        timeRatio = 0.7;
        riskReduction = 60;
    }

    return { personnel, timeRatio, riskReduction };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create a robot (standalone function)
 */
export function createRobot(params: {
  type: RobotType | string;
  weight?: number;
  autonomyLevel?: AutonomyLevel | number;
  payload?: number;
}): MilitaryRobot {
  const sdk = new MilitaryRobotSDK();
  return sdk.createRobot(params);
}

/**
 * Validate a mission (standalone function)
 */
export function validateMission(mission: MissionParams): MissionValidation {
  const sdk = new MilitaryRobotSDK();
  return sdk.validateMission(mission);
}

/**
 * Calculate efficiency (standalone function)
 */
export function calculateEfficiency(params: EfficiencyParams): EfficiencyResult {
  const sdk = new MilitaryRobotSDK();
  return sdk.calculateEfficiency(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { MilitaryRobotSDK };
export default MilitaryRobotSDK;
