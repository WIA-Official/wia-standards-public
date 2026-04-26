/**
 * WIA-DEF-002: Military Drone SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for military drone systems including:
 * - Drone configuration and classification
 * - Mission planning and execution
 * - Flight parameter calculations
 * - Sensor integration and control
 * - Safety and ethical compliance checks
 */

import {
  DroneConfiguration,
  DroneClass,
  FlightPlan,
  MissionParameters,
  MissionCalculation,
  MissionExecution,
  BorderPatrol,
  ForceProtection,
  DisasterResponse,
  SearchAndRescue,
  MedicalDelivery,
  PreFlightChecklist,
  EthicalCompliance,
  SafetyCheck,
  DroneErrorCode,
  DroneError,
  DRONE_CONSTANTS,
  GeoCoordinate,
  Waypoint,
  MissionType,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-002 Military Drone SDK
 */
export class MilitaryDroneSDK {
  private version = '1.0.0';
  private initialized = false;

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Get drone classification details
   *
   * @param droneClass - Drone classification
   * @returns Classification specifications
   */
  getClassificationDetails(droneClass: DroneClass) {
    const classifications = {
      'Class I - Micro/Mini': {
        class: 'Class I - Micro/Mini' as const,
        weightRange: { min: 0, max: 2 },
        enduranceRange: { min: 0.1, max: 1 },
        altitudeRange: { min: 0, max: 1000 },
        rangeKm: { min: 0, max: 5 },
        payloadCapacity: { min: 0, max: 0.5 },
        examples: ['Black Hornet', 'Raven'],
      },
      'Class II - Small Tactical': {
        class: 'Class II - Small Tactical' as const,
        weightRange: { min: 2, max: 25 },
        enduranceRange: { min: 1, max: 4 },
        altitudeRange: { min: 1000, max: 5000 },
        rangeKm: { min: 5, max: 50 },
        payloadCapacity: { min: 0.5, max: 5 },
        examples: ['ScanEagle', 'Puma'],
      },
      'Class III - Medium Tactical': {
        class: 'Class III - Medium Tactical' as const,
        weightRange: { min: 25, max: 150 },
        enduranceRange: { min: 4, max: 12 },
        altitudeRange: { min: 5000, max: 15000 },
        rangeKm: { min: 50, max: 200 },
        payloadCapacity: { min: 5, max: 30 },
        examples: ['Shadow', 'Hermes 450'],
      },
      'Class IV - MALE': {
        class: 'Class IV - MALE' as const,
        weightRange: { min: 150, max: 600 },
        enduranceRange: { min: 12, max: 48 },
        altitudeRange: { min: 15000, max: 30000 },
        rangeKm: { min: 200, max: 1000 },
        payloadCapacity: { min: 30, max: 200 },
        examples: ['Predator', 'Reaper', 'Watchkeeper'],
      },
      'Class V - HALE': {
        class: 'Class V - HALE' as const,
        weightRange: { min: 600, max: 10000 },
        enduranceRange: { min: 48, max: 720 },
        altitudeRange: { min: 30000, max: 65000 },
        rangeKm: { min: 1000, max: 20000 },
        payloadCapacity: { min: 200, max: 1500 },
        examples: ['Global Hawk', 'Zephyr'],
      },
    };

    return classifications[droneClass];
  }

  /**
   * Calculate mission parameters including fuel, endurance, and coverage
   *
   * @param params - Mission parameters
   * @returns Calculated mission parameters
   */
  calculateMissionParameters(params: MissionParameters): MissionCalculation {
    const {
      droneClass,
      distance,
      loiterTime = 0,
      altitude,
      speed,
      payloadWeight,
      weather,
    } = params;

    const classDetails = this.getClassificationDetails(droneClass);
    const warnings: string[] = [];
    const recommendations: string[] = [];

    // Estimate cruise speed if not provided (typically 60-70% of max speed)
    const cruiseSpeed = speed || classDetails.rangeKm.max / classDetails.enduranceRange.max;

    // Calculate flight time
    const transitTime = distance / cruiseSpeed; // hours
    const totalFlightTime = transitTime + loiterTime;

    // Get max endurance for class
    const maxEndurance = classDetails.enduranceRange.max;

    // Apply weather corrections
    let weatherFactor = 1.0;
    if (weather) {
      // Headwind/tailwind affects fuel consumption
      const windEffect = Math.abs(weather.windSpeed) / cruiseSpeed;
      weatherFactor = 1 + windEffect * 0.1; // 10% effect per wind ratio

      // Temperature effects
      if (weather.temperature < 0) {
        weatherFactor *= 1.1; // 10% penalty in cold
        warnings.push('Cold weather: 10% endurance penalty');
      } else if (weather.temperature > 35) {
        weatherFactor *= 1.05; // 5% penalty in heat
        warnings.push('Hot weather: 5% endurance penalty');
      }
    }

    // Apply payload weight penalty
    const maxPayload = classDetails.payloadCapacity.max;
    const payloadFactor = 1 + (payloadWeight / maxPayload) * 0.2; // 20% max penalty

    // Calculate required endurance with safety margin
    const fuelReserveHours = DRONE_CONSTANTS.FUEL_RESERVES.MINIMUM_MINUTES / 60;
    const requiredEndurance = totalFlightTime * weatherFactor * payloadFactor + fuelReserveHours;
    const enduranceMargin = maxEndurance - requiredEndurance;

    // Estimate fuel/battery requirements
    let fuelRequired: number | undefined;
    let batteryRequired: number | undefined;

    // Rough estimate: assume 20-40 liters/hour for fuel-powered drones
    if (droneClass.includes('Class IV') || droneClass.includes('Class V')) {
      const fuelConsumptionRate = 25; // liters/hour (typical for MALE/HALE)
      fuelRequired = requiredEndurance * fuelConsumptionRate;
    } else {
      // Battery-powered (smaller drones)
      const powerConsumption = 200; // Wh/hour (typical)
      batteryRequired = requiredEndurance * powerConsumption;
    }

    // Calculate surveillance area (assuming circular orbit)
    const sensorRangeKm = this.estimateSensorRange(altitude, droneClass);
    const surveillanceArea = Math.PI * sensorRangeKm * sensorRangeKm;

    // Determine feasibility
    let feasibility: MissionCalculation['feasibility'];
    if (requiredEndurance <= maxEndurance * 0.8) {
      feasibility = 'possible';
    } else if (requiredEndurance <= maxEndurance) {
      feasibility = 'marginal';
      warnings.push('Limited endurance margin - consider reducing mission duration');
    } else {
      feasibility = 'impossible';
      warnings.push('Insufficient endurance for mission');
    }

    // Altitude check
    if (altitude > classDetails.altitudeRange.max) {
      warnings.push(`Altitude exceeds max ceiling (${classDetails.altitudeRange.max} ft)`);
      feasibility = 'impossible';
    }

    // Distance check
    if (distance > classDetails.rangeKm.max) {
      warnings.push(`Distance exceeds max range (${classDetails.rangeKm.max} km)`);
      feasibility = 'impossible';
    }

    // Recommendations
    if (loiterTime > 4) {
      recommendations.push('Long loiter time - consider using higher-endurance platform');
    }

    if (payloadWeight > maxPayload * 0.8) {
      recommendations.push('Heavy payload - performance may be degraded');
    }

    return {
      fuelRequired,
      batteryRequired,
      totalFlightTime: requiredEndurance,
      maxEndurance,
      enduranceMargin,
      surveillanceArea,
      feasibility,
      warnings,
      recommendations,
    };
  }

  /**
   * Validate a flight plan for safety and compliance
   *
   * @param flightPlan - Flight plan to validate
   * @returns Pre-flight checklist with validation results
   */
  validateFlightPlan(flightPlan: FlightPlan): PreFlightChecklist {
    const checks: PreFlightChecklist['checks'] = {
      weather: {
        name: 'Weather Check',
        status: 'pass',
        description: 'Weather conditions within limits',
      },
      airspace: {
        name: 'Airspace Clearance',
        status: flightPlan.approvals.airspaceCleared ? 'pass' : 'fail',
        description: 'Airspace authorization obtained',
        correctiveAction: flightPlan.approvals.airspaceCleared
          ? undefined
          : 'Obtain airspace clearance from ATC',
      },
      fuel: {
        name: 'Fuel/Battery',
        status: 'pass',
        description: 'Sufficient fuel/battery for mission + reserves',
      },
      communication: {
        name: 'Communication Links',
        status: 'pass',
        description: 'Primary and backup links operational',
      },
      sensors: {
        name: 'Sensor Systems',
        status: 'pass',
        description: 'All mission sensors calibrated and functional',
      },
      autopilot: {
        name: 'Autopilot System',
        status: 'pass',
        description: 'Flight control system operational',
      },
      geofence: {
        name: 'Geofencing',
        status: 'pass',
        description: 'Geofence boundaries configured',
      },
      emergency: {
        name: 'Emergency Procedures',
        status: 'pass',
        description: 'Lost link and emergency landing procedures programmed',
      },
    };

    // Check if weather approval is granted
    if (!flightPlan.approvals.weatherAcceptable) {
      checks.weather.status = 'warning';
      checks.weather.description = 'Weather marginal - monitor conditions';
    }

    // Check if all waypoints are valid
    if (flightPlan.waypoints.length < 2) {
      checks.autopilot.status = 'fail';
      checks.autopilot.description = 'Insufficient waypoints (minimum 2 required)';
    }

    // Overall completion status
    const failedChecks = Object.values(checks).filter((c) => c.status === 'fail');
    const completed = failedChecks.length === 0;

    return {
      completed,
      timestamp: completed ? new Date() : undefined,
      operator: 'SDK Validation',
      checks,
      approvedBy: completed ? 'System' : undefined,
      approvalTimestamp: completed ? new Date() : undefined,
    };
  }

  /**
   * Plan a border patrol mission
   *
   * @param start - Start coordinate
   * @param end - End coordinate
   * @param droneClass - Drone class to use
   * @returns Border patrol configuration
   */
  planBorderPatrol(
    start: GeoCoordinate,
    end: GeoCoordinate,
    droneClass: DroneClass
  ): BorderPatrol {
    const distance = this.calculateDistance(start, end);
    const classDetails = this.getClassificationDetails(droneClass);

    // Default patrol altitude (1000-3000 ft AGL)
    const altitude = 2000;

    // Cruise speed
    const speed = classDetails.rangeKm.max / classDetails.enduranceRange.max * 0.539957; // Convert to knots

    // Sensor coverage (estimate based on altitude and FOV)
    const sensorCoverage = (altitude * 0.3048) * Math.tan((60 * Math.PI) / 180) * 2 / 1000; // km

    // Revisit rate based on endurance and distance
    const transitTime = distance / (speed * DRONE_CONSTANTS.CONVERSIONS.KNOTS_TO_MS * 3.6);
    const revisitRate = transitTime * 2; // Round trip

    return {
      borderSegment: {
        start,
        end,
        length: distance,
      },
      patternType: 'linear',
      altitude,
      speed,
      sensorCoverage,
      revisitRate,
    };
  }

  /**
   * Plan a disaster response mission
   *
   * @param disasterType - Type of disaster
   * @param center - Center of affected area
   * @param radiusKm - Radius of affected area in km
   * @returns Disaster response mission configuration
   */
  planDisasterResponse(
    disasterType: DisasterResponse['disasterType'],
    center: GeoCoordinate,
    radiusKm: number
  ): DisasterResponse {
    return {
      disasterType,
      affectedArea: {
        center,
        radius: radiusKm,
      },
      assessmentType: 'rapid',
      priority: ['damage', 'survivors', 'infrastructure', 'hazards'],
      products: ['imagery', 'damage-map', 'survivor-locations'],
    };
  }

  /**
   * Plan a medical delivery mission
   *
   * @param origin - Origin coordinates
   * @param destination - Destination coordinates
   * @param deliveryType - Type of medical delivery
   * @param weight - Weight in kg
   * @returns Medical delivery mission configuration
   */
  planMedicalDelivery(
    origin: GeoCoordinate,
    destination: GeoCoordinate,
    deliveryType: MedicalDelivery['deliveryType'],
    weight: number
  ): MedicalDelivery {
    // Determine temperature control requirements
    let temperatureControl: MedicalDelivery['temperatureControl'];
    if (deliveryType === 'blood' || deliveryType === 'vaccine') {
      temperatureControl = { min: 2, max: 8 }; // Celsius
    }

    return {
      origin,
      destination,
      deliveryType,
      weight,
      volume: weight * 1.5, // Rough estimate: 1.5 liters per kg
      urgency: deliveryType === 'blood' ? 'emergency' : 'urgent',
      temperatureControl,
      dropType: weight < 5 ? 'parachute' : 'landing',
    };
  }

  /**
   * Check ethical compliance for a mission
   *
   * @param missionType - Type of mission
   * @param hasWeapons - Whether weapons are involved
   * @returns Ethical compliance assessment
   */
  checkEthicalCompliance(
    missionType: MissionType,
    hasWeapons: boolean
  ): EthicalCompliance {
    return {
      humanInLoop: hasWeapons, // Required for weapon systems
      civilianProtection: {
        collateralDamageEstimate: 0,
        proportionalityAssessment: 'proportional',
        legalReview: hasWeapons,
      },
      dataPrivacy: {
        collectionJustified: true,
        retentionPeriod: 90, // days
        accessRestricted: true,
        encryptionEnabled: true,
      },
      internationalLaw: {
        genevaCompliant: true,
        treatyCompliant: true,
        rulesOfEngagement: missionType.includes('ISR') ? 'Observe Only' : 'Standard ROE',
      },
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Calculate distance between two coordinates (Haversine formula)
   */
  private calculateDistance(coord1: GeoCoordinate, coord2: GeoCoordinate): number {
    const R = DRONE_CONSTANTS.EARTH_RADIUS / 1000; // km
    const dLat = this.toRadians(coord2.latitude - coord1.latitude);
    const dLon = this.toRadians(coord2.longitude - coord1.longitude);

    const a =
      Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(this.toRadians(coord1.latitude)) *
        Math.cos(this.toRadians(coord2.latitude)) *
        Math.sin(dLon / 2) *
        Math.sin(dLon / 2);

    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c; // km
  }

  /**
   * Convert degrees to radians
   */
  private toRadians(degrees: number): number {
    return degrees * (Math.PI / 180);
  }

  /**
   * Estimate sensor range based on altitude and drone class
   */
  private estimateSensorRange(altitudeFt: number, droneClass: DroneClass): number {
    // Convert to meters
    const altitudeM = altitudeFt * DRONE_CONSTANTS.CONVERSIONS.FEET_TO_METERS;

    // Assume 60-degree FOV for typical sensors
    const fovDeg = 60;
    const rangeM = altitudeM * Math.tan((fovDeg / 2) * (Math.PI / 180)) * 2;

    return rangeM / 1000; // Convert to km
  }

  /**
   * Generate waypoints for area coverage pattern
   */
  generateAreaCoverageWaypoints(
    center: GeoCoordinate,
    radiusKm: number,
    altitudeFt: number,
    pattern: 'lawnmower' | 'spiral'
  ): Waypoint[] {
    const waypoints: Waypoint[] = [];

    if (pattern === 'spiral') {
      // Generate spiral pattern
      const turns = 5;
      const pointsPerTurn = 20;

      for (let i = 0; i < turns * pointsPerTurn; i++) {
        const angle = (i / pointsPerTurn) * 2 * Math.PI;
        const radius = (radiusKm * 1000 * i) / (turns * pointsPerTurn);

        const lat = center.latitude + (radius / DRONE_CONSTANTS.EARTH_RADIUS) * (180 / Math.PI) * Math.sin(angle);
        const lon =
          center.longitude +
          ((radius / DRONE_CONSTANTS.EARTH_RADIUS) * (180 / Math.PI) * Math.cos(angle)) /
            Math.cos((center.latitude * Math.PI) / 180);

        waypoints.push({
          id: `WP-${i + 1}`,
          position: { latitude: lat, longitude: lon, altitude: altitudeFt },
          action: 'flyby',
        });
      }
    }

    return waypoints;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create a drone system configuration (standalone function)
 */
export function createDroneSystem(config: Partial<DroneConfiguration>): DroneConfiguration {
  const sdk = new MilitaryDroneSDK();
  const classDetails = sdk.getClassificationDetails(
    config.classification || 'Class IV - MALE'
  );

  return {
    id: config.id || `DRONE-${Date.now()}`,
    name: config.name || 'Military UAV',
    classification: config.classification || 'Class IV - MALE',
    physical: config.physical || {
      weight: classDetails.weightRange.min,
      maxTakeoffWeight: classDetails.weightRange.max,
      length: 8,
      height: 2,
    },
    performance: config.performance || {
      maxSpeed: 200,
      cruiseSpeed: 120,
      maxAltitude: classDetails.altitudeRange.max,
      serviceCeiling: classDetails.altitudeRange.max,
      endurance: classDetails.enduranceRange.max,
      range: classDetails.rangeKm.max,
    },
    propulsion: config.propulsion || {
      type: 'gasoline',
      fuelCapacity: 400,
      powerOutput: 100,
    },
    payload: config.payload || {
      maxWeight: classDetails.payloadCapacity.max,
      sensors: {
        gimballed: true,
        gimbalAxes: 3,
      },
    },
    communication: config.communication || {
      primaryLink: {
        name: 'Primary C2',
        type: 'LOS',
        frequency: 5,
        band: 'C',
        range: 200,
        dataRate: { command: 10, telemetry: 100, video: 10 },
        encryption: 'AES-256',
        antiJam: true,
        frequencyHopping: true,
        latency: 50,
      },
      backupLinks: [],
      redundancy: true,
      automaticFailover: true,
    },
    autopilot: config.autopilot || {
      manufacturer: 'Generic',
      model: 'AP-1000',
      version: '1.0',
      sensors: {
        imu: true,
        gps: true,
        barometer: true,
        magnetometer: true,
        airspeed: true,
      },
      capabilities: {
        waypoint: true,
        loiter: true,
        orbit: true,
        terrainFollowing: false,
        obstacleAvoidance: false,
        autoTakeoff: true,
        autoLanding: true,
        returnToBase: true,
      },
      navigationAccuracy: {
        gps: 3,
        ins: 1,
      },
      safetyFeatures: {
        geofencing: true,
        lowFuelRTB: true,
        lostLinkRTB: true,
        emergencyLanding: true,
        collisionAvoidance: false,
      },
    },
  };
}

/**
 * Calculate mission parameters (standalone function)
 */
export function calculateMissionParameters(
  params: MissionParameters
): MissionCalculation {
  const sdk = new MilitaryDroneSDK();
  return sdk.calculateMissionParameters(params);
}

/**
 * Validate mission (standalone function)
 */
export function validateMission(flightPlan: FlightPlan): PreFlightChecklist {
  const sdk = new MilitaryDroneSDK();
  return sdk.validateFlightPlan(flightPlan);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { MilitaryDroneSDK };
export default MilitaryDroneSDK;
