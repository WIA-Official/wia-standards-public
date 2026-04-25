/**
 * WIA-DEF-019: Underwater Weapon SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for underwater weapon systems including:
 * - Torpedo configuration and attack calculations
 * - Naval mine warfare planning
 * - UUV mission planning and execution
 * - Sonar range and acoustic propagation
 * - Mine clearance operations
 * - Countermeasure effectiveness analysis
 */

import {
  TorpedoConfiguration,
  TorpedoAttackParams,
  TorpedoInterceptResult,
  MineConfiguration,
  MinefieldLayout,
  MinefieldDesign,
  UUVConfiguration,
  UUVMission,
  UUVMissionExecution,
  SonarConfiguration,
  AcousticPropagation,
  SonarRangeResult,
  MineClearanceOperation,
  MineClearanceResult,
  SoundSpeedParams,
  SoundSpeedResult,
  AcousticAbsorption,
  TargetStrengthParams,
  TargetStrengthResult,
  CountermeasureConfiguration,
  EnvironmentalConditions,
  GeoCoordinate,
  UNDERWATER_CONSTANTS,
  UnderwaterWeaponError,
  UnderwaterWeaponErrorCode,
  TorpedoClass,
  MineType,
  UUVType,
  SonarType,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-019 Underwater Weapon SDK
 */
export class UnderwaterWeaponSDK {
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

  // ==========================================================================
  // Torpedo Systems
  // ==========================================================================

  /**
   * Create a torpedo weapon system
   *
   * @param config - Partial torpedo configuration
   * @returns Complete torpedo configuration
   */
  createTorpedo(
    config: Partial<TorpedoConfiguration>
  ): TorpedoConfiguration {
    const defaults = this.getTorpedoDefaults(
      config.class || 'Heavy Torpedo (533mm)'
    );

    return {
      ...defaults,
      ...config,
      guidance: { ...defaults.guidance, ...config.guidance },
      warhead: { ...defaults.warhead, ...config.warhead },
      speed: { ...defaults.speed, ...config.speed },
    };
  }

  /**
   * Get default torpedo specifications by class
   */
  private getTorpedoDefaults(
    torpedoClass: TorpedoClass
  ): TorpedoConfiguration {
    const defaults: Record<TorpedoClass, TorpedoConfiguration> = {
      'Heavy Torpedo (533mm)': {
        class: 'Heavy Torpedo (533mm)',
        diameter: 533,
        length: 6.5,
        weight: 1800,
        propulsion: 'Electric',
        speed: { cruise: 40, sprint: 50 },
        range: 50000,
        maxDepth: 1000,
        guidance: {
          primary: 'Wire-Guided',
          secondary: 'Active Acoustic Homing',
          ai: true,
          wireLength: 30000,
        },
        warhead: { type: 'Shaped Charge', weight: 250, fuzing: 'Combined' },
        endurance: 30,
      },
      'Heavy Torpedo (650mm)': {
        class: 'Heavy Torpedo (650mm)',
        diameter: 650,
        length: 8.0,
        weight: 2700,
        propulsion: 'Thermal (Otto Fuel II)',
        speed: { cruise: 45, sprint: 55 },
        range: 100000,
        maxDepth: 1200,
        guidance: {
          primary: 'Wire-Guided',
          secondary: 'Wake Homing',
          ai: true,
          wireLength: 50000,
        },
        warhead: { type: 'High Explosive (HE)', weight: 500, fuzing: 'Proximity' },
        endurance: 45,
      },
      'Light Torpedo (324mm)': {
        class: 'Light Torpedo (324mm)',
        diameter: 324,
        length: 2.75,
        weight: 280,
        propulsion: 'Electric',
        speed: { cruise: 40, sprint: 45 },
        range: 12000,
        maxDepth: 800,
        guidance: {
          primary: 'Active Acoustic Homing',
          secondary: 'Passive Acoustic Homing',
          ai: false,
        },
        warhead: { type: 'High Explosive (HE)', weight: 60, fuzing: 'Contact' },
        endurance: 10,
      },
      'Supercavitating Torpedo': {
        class: 'Supercavitating Torpedo',
        diameter: 533,
        length: 8.0,
        weight: 2900,
        propulsion: 'Rocket',
        speed: { cruise: 200, sprint: 230 },
        range: 12000,
        maxDepth: 400,
        guidance: { primary: 'Inertial', ai: false },
        warhead: { type: 'Shaped Charge', weight: 210, fuzing: 'Contact' },
        endurance: 3,
      },
    };

    return defaults[torpedoClass];
  }

  /**
   * Calculate torpedo intercept parameters
   *
   * @param params - Attack parameters
   * @returns Intercept calculation result
   */
  calculateTorpedoIntercept(
    params: TorpedoAttackParams
  ): TorpedoInterceptResult {
    const { torpedoSpeed, targetSpeed, targetBearing, range, depth } = params;

    // Convert speeds to m/s
    const vTorpedo = torpedoSpeed * UNDERWATER_CONSTANTS.KNOTS_TO_MS;
    const vTarget = targetSpeed * UNDERWATER_CONSTANTS.KNOTS_TO_MS;

    // Calculate relative velocity components
    const bearingRad = (targetBearing * Math.PI) / 180;
    const vTargetX = vTarget * Math.sin(bearingRad);
    const vTargetY = vTarget * Math.cos(bearingRad);

    // Solve intercept triangle
    const a = vTargetX ** 2 + vTargetY ** 2 - vTorpedo ** 2;
    const b = 2 * range * vTargetY;
    const c = range ** 2;

    const discriminant = b ** 2 - 4 * a * c;

    if (discriminant < 0) {
      throw new UnderwaterWeaponError(
        UnderwaterWeaponErrorCode.INVALID_TARGET,
        'Target cannot be intercepted (torpedo too slow or target out of range)',
        { torpedoSpeed, targetSpeed, range }
      );
    }

    const timeToImpact = (-b - Math.sqrt(discriminant)) / (2 * a);

    // Calculate lead angle
    const interceptX = vTargetX * timeToImpact;
    const interceptY = range + vTargetY * timeToImpact;
    const leadAngle = (Math.atan2(interceptX, interceptY) * 180) / Math.PI;

    // Calculate fuel requirement (simplified model)
    const distance = Math.sqrt(interceptX ** 2 + interceptY ** 2);
    const fuelRequired = (distance / (range * 0.8)) * 100; // Percentage

    // Estimate probability of hit
    const depthFactor = Math.exp(-Math.abs(depth - (params.targetDepth || depth)) / 100);
    const speedFactor = Math.min(1, vTorpedo / (vTarget + 10));
    const probabilityOfHit = depthFactor * speedFactor * 0.85;

    // Select guidance mode
    const guidanceMode =
      range > 10000 ? 'Wire-Guided' : 'Active Acoustic Homing';

    return {
      timeToImpact,
      leadAngle,
      fuelRequired,
      probabilityOfHit,
      guidanceMode,
    };
  }

  // ==========================================================================
  // Naval Mine Warfare
  // ==========================================================================

  /**
   * Design a minefield layout
   *
   * @param layout - Minefield layout parameters
   * @returns Minefield design
   */
  designMinefield(layout: MinefieldLayout): MinefieldDesign {
    const { area, density, pattern, waterDepth } = layout;

    // Calculate mine spacing based on density
    const spacingMap = {
      Low: 200,
      Medium: 100,
      High: 50,
      'Very High': 25,
    };
    const mineSpacing = spacingMap[density];

    // Calculate total mines based on pattern
    let totalMines: number;
    let coverage: number;

    if (pattern === 'Linear (Barrier)') {
      const rows = Math.ceil(area.width / mineSpacing);
      const cols = Math.ceil(area.length / mineSpacing);
      totalMines = rows * cols;
      coverage = 0.8; // 80% coverage for linear barrier
    } else if (pattern === 'Grid') {
      totalMines = Math.ceil((area.width / mineSpacing) * (area.length / mineSpacing));
      coverage = 0.9;
    } else if (pattern === 'Staggered') {
      const rows = Math.ceil(area.width / mineSpacing);
      const cols = Math.ceil(area.length / (mineSpacing * 1.5));
      totalMines = rows * cols;
      coverage = 0.85;
    } else if (pattern === 'Concentric (Defensive)') {
      const radius = Math.min(area.width, area.length) / 2;
      const rings = Math.ceil(radius / mineSpacing);
      totalMines = rings * 8 * rings; // Increasing mines per ring
      coverage = 0.95;
    } else {
      // Random
      const areaKm2 = (area.width * area.length) / 1e6;
      totalMines = Math.ceil(areaKm2 * (100 / (mineSpacing / 10)));
      coverage = 0.7;
    }

    // Calculate deployment time (simplified)
    const deploymentRatePerHour = 10; // mines per hour (depends on method)
    const deploymentTime = totalMines / deploymentRatePerHour;

    // Determine deployment method based on depth and purpose
    const deploymentMethod =
      waterDepth < 50
        ? 'Surface Vessel'
        : waterDepth < 200
        ? 'Submarine'
        : 'Aircraft';

    // Estimate effectiveness
    const depthFactor = waterDepth < 100 ? 1.0 : 0.85;
    const patternFactor =
      pattern === 'Concentric (Defensive)' ? 1.0 : pattern === 'Grid' ? 0.9 : 0.8;
    const estimatedEffectiveness =
      coverage * depthFactor * patternFactor * 0.8;

    return {
      totalMines,
      mineSpacing,
      deploymentTime,
      deploymentMethod,
      coverage,
      estimatedEffectiveness,
    };
  }

  // ==========================================================================
  // Unmanned Underwater Vehicles (UUVs)
  // ==========================================================================

  /**
   * Plan a UUV mission
   *
   * @param mission - UUV mission parameters
   * @returns Mission execution result
   */
  planUUVMission(mission: UUVMission): UUVMissionExecution {
    const { area, depth, duration, surveyPattern } = mission;

    // Calculate estimated distance based on survey pattern
    let estimatedDistance: number;
    const radius = area.radius;

    if (surveyPattern === 'Grid' || surveyPattern === 'Lawn Mower') {
      const swathWidth = 100; // meters (typical sonar swath)
      const lines = Math.ceil((radius * 2) / swathWidth);
      estimatedDistance = (lines * radius * 2) / 1000; // km
    } else if (surveyPattern === 'Spiral') {
      estimatedDistance = (Math.PI * radius ** 2) / (100 * 1000); // Simplified
    } else {
      // Random or waypoint-based
      estimatedDistance = (radius * Math.PI) / 1000; // Circumference estimate
    }

    // Calculate estimated time based on typical UUV speed
    const typicalSpeed = 3; // knots
    const speedKmH = typicalSpeed * UNDERWATER_CONSTANTS.KNOTS_TO_KMH;
    const estimatedTime = estimatedDistance / speedKmH;

    // Calculate battery requirement
    const batteryRequired = (estimatedTime / duration) * 100;

    // Validate mission
    const warnings: string[] = [];
    let missionValid = true;

    if (batteryRequired > 100) {
      warnings.push('Mission duration exceeds UUV endurance capability');
      missionValid = false;
    }

    if (depth.operating > 3000) {
      warnings.push('Operating depth exceeds typical UUV limits');
    }

    if (estimatedDistance > 100) {
      warnings.push('Mission distance is very long; consider multiple UUVs');
    }

    // Calculate coverage area
    const coverageArea = Math.PI * (radius / 1000) ** 2; // km²

    return {
      missionValid,
      estimatedDistance,
      estimatedTime,
      batteryRequired,
      coverageArea,
      warnings: warnings.length > 0 ? warnings : undefined,
    };
  }

  // ==========================================================================
  // Sonar & Acoustics
  // ==========================================================================

  /**
   * Calculate sound speed in water
   *
   * @param params - Environmental parameters
   * @returns Sound speed result
   */
  calculateSoundSpeed(params: SoundSpeedParams): SoundSpeedResult {
    const { temperature, salinity, depth } = params;

    // Mackenzie equation for sound speed
    const T = temperature;
    const S = salinity;
    const D = depth;

    const c =
      1448.96 +
      4.591 * T -
      0.05304 * T ** 2 +
      0.0002374 * T ** 3 +
      0.016 * D +
      (1.34 - 0.01 * T) * (S - 35) +
      1.675e-7 * D ** 2 -
      7.139e-13 * T * D ** 3;

    // Calculate gradient (simplified: assume linear with depth)
    const c0 = this.calculateSoundSpeed({ temperature, salinity, depth: 0 })
      .soundSpeed;
    const c1000 = this.calculateSoundSpeed({ temperature, salinity, depth: 1000 })
      .soundSpeed;
    const gradient = (c1000 - c0) / 1000;

    return {
      soundSpeed: c,
      gradient,
    };
  }

  /**
   * Calculate acoustic absorption
   *
   * @param frequency - Frequency in Hz
   * @returns Absorption characteristics
   */
  calculateAcousticAbsorption(frequency: number): AcousticAbsorption {
    // Francois-Garrison equation (simplified)
    const f = frequency / 1000; // Convert to kHz

    // Absorption coefficient in dB/km
    const alpha =
      0.11 * (f ** 2 / (1 + f ** 2)) + 44 * (f ** 2 / (4100 + f ** 2)) + 0.0003 * f ** 2;

    const rangeFor3dBLoss = 3000 / alpha;
    const rangeFor10dBLoss = 10000 / alpha;

    return {
      frequency,
      absorptionCoefficient: alpha,
      rangeFor3dBLoss,
      rangeFor10dBLoss,
    };
  }

  /**
   * Calculate sonar detection range
   *
   * @param propagation - Acoustic propagation parameters
   * @returns Sonar range result
   */
  calculateSonarRange(propagation: AcousticPropagation): SonarRangeResult {
    const { frequency, power, environment } = propagation;

    // Calculate transmission loss (simplified spherical spreading + absorption)
    const absorption = this.calculateAcousticAbsorption(frequency);

    // Sonar equation: SE = SL - 2*TL + TS - NL - DT
    // Simplified: maxRange when SE = 0

    const sourceLevel = power; // dB
    const detectionThreshold = UNDERWATER_CONSTANTS.DETECTION_THRESHOLD; // dB
    const ambientNoise = 50 + environment.seaState * 5; // Simplified noise model
    const targetStrength = 20; // Assume medium target (dB)

    // TL = 20*log10(r) + α*r (spherical spreading + absorption)
    // Solve for r when SE = 0

    const allowedLoss =
      (sourceLevel + targetStrength - ambientNoise - detectionThreshold) / 2;

    // Iterative solution (simplified)
    let maxRange = 1000; // Initial guess (meters)
    for (let i = 0; i < 10; i++) {
      const TL =
        20 * Math.log10(maxRange) +
        (absorption.absorptionCoefficient * maxRange) / 1000;
      const error = TL - allowedLoss;
      maxRange -= error * 50; // Adjust range
      if (Math.abs(error) < 0.1) break;
    }

    // Determine propagation mode based on depth and range
    const waterDepth = propagation.bathymetry?.depth || 1000;
    let propagationMode: SonarRangeResult['propagationMode'] = 'Direct Path';
    const convergenceZones: number[] = [];

    if (waterDepth > 1000 && maxRange > 30000) {
      propagationMode = 'Convergence Zone';
      // Convergence zones at ~30-60 km intervals
      for (let i = 30000; i < maxRange; i += 30000) {
        convergenceZones.push(i);
      }
    } else if (environment.depth < 200 && waterDepth < 500) {
      propagationMode = 'Surface Duct';
    } else if (waterDepth > 1000) {
      propagationMode = 'Deep Sound Channel';
    }

    return {
      maxRange,
      convergenceZones: convergenceZones.length > 0 ? convergenceZones : undefined,
      propagationMode,
      confidenceLevel: 0.75,
    };
  }

  /**
   * Calculate target strength
   *
   * @param params - Target parameters
   * @returns Target strength result
   */
  calculateTargetStrength(params: TargetStrengthParams): TargetStrengthResult {
    const { targetType, aspect, frequency } = params;

    // Simplified target strength models
    const baseStrength: Record<string, number> = {
      'Large Submarine': 25,
      'Small Submarine': 15,
      'Surface Ship': 25,
      Mine: 0,
      UUV: -5,
      'Fish School': 5,
    };

    let targetStrength = baseStrength[targetType] || 0;

    // Aspect dependency
    if (aspect === 'Beam') {
      targetStrength += 5; // Higher cross-section
    } else if (aspect === 'Bow' || aspect === 'Stern') {
      targetStrength -= 5; // Lower cross-section
    }

    // Frequency dependency (simplified)
    if (frequency > 10000) {
      targetStrength += 3; // Higher frequency, better resolution
    }

    return {
      targetStrength,
      aspectDependency: true,
      frequencyDependency: frequency > 10000,
      range: { min: targetStrength - 5, max: targetStrength + 5 },
    };
  }

  // ==========================================================================
  // Mine Clearance Operations
  // ==========================================================================

  /**
   * Plan mine clearance operation
   *
   * @param operation - Mine clearance parameters
   * @returns Clearance operation result
   */
  planMineClearance(operation: MineClearanceOperation): MineClearanceResult {
    const { area, depth, method, safety, purpose, estimatedMineCount } = operation;

    // Calculate coverage area
    const coverageArea = Math.PI * (area.radius / 1000) ** 2; // km²

    // Estimate resources based on method
    const resourcesRequired: MineClearanceResult['resourcesRequired'] = {};

    if (method === 'UUV Hunting') {
      resourcesRequired.uuvs = Math.ceil(coverageArea / 5); // 1 UUV per 5 km²
      resourcesRequired.supportVessels = Math.ceil(resourcesRequired.uuvs / 3);
    } else if (method === 'ROV Disposal') {
      resourcesRequired.rovs = Math.ceil((estimatedMineCount || 10) / 5);
      resourcesRequired.supportVessels = Math.ceil(resourcesRequired.rovs / 2);
    } else if (method === 'Diver Clearance') {
      resourcesRequired.divers = Math.ceil((estimatedMineCount || 10) * 2); // 2 divers per mine
      resourcesRequired.supportVessels = 2;
    } else {
      // Sweeping methods
      resourcesRequired.supportVessels = Math.ceil(coverageArea / 10);
    }

    // Estimate duration
    const ratePerDay = method === 'UUV Hunting' ? 10 : method === 'ROV Disposal' ? 5 : 2; // km² per day
    const estimatedDuration = coverageArea / ratePerDay;

    // Clearance confidence
    const methodConfidence: Record<string, number> = {
      'UUV Hunting': 0.9,
      'ROV Disposal': 0.95,
      'Diver Clearance': 0.98,
      'Mechanical Sweeping': 0.7,
      'Influence Sweeping': 0.75,
      'Controlled Detonation': 0.85,
    };
    let clearanceConfidence = methodConfidence[method] || 0.8;

    // Adjust for safety level
    if (safety === 'Maximum') {
      clearanceConfidence *= 0.95; // Slower but more thorough
    }

    // Safety rating
    const safetyRating: MineClearanceResult['safetyRating'] =
      depth.max < 50 && method !== 'Diver Clearance'
        ? 'Low Risk'
        : method === 'Diver Clearance'
        ? 'High Risk'
        : 'Medium Risk';

    // Recommendations
    const recommendations: string[] = [];
    if (purpose === 'Humanitarian') {
      recommendations.push('Prioritize complete clearance over speed');
      recommendations.push('Engage local communities for historical minefield data');
    }
    if (depth.max > 200) {
      recommendations.push('Use UUV/ROV methods; avoid diver operations');
    }
    if (estimatedMineCount && estimatedMineCount > 50) {
      recommendations.push('Consider multiple teams for parallel operations');
    }

    return {
      operationValid: true,
      estimatedDuration,
      resourcesRequired,
      coverageArea,
      clearanceConfidence,
      safetyRating,
      recommendations: recommendations.length > 0 ? recommendations : undefined,
    };
  }

  // ==========================================================================
  // Utility Functions
  // ==========================================================================

  /**
   * Calculate distance between two geographic coordinates
   *
   * @param coord1 - First coordinate
   * @param coord2 - Second coordinate
   * @returns Distance in meters
   */
  calculateDistance(coord1: GeoCoordinate, coord2: GeoCoordinate): number {
    const lat1 = (coord1.latitude * Math.PI) / 180;
    const lat2 = (coord2.latitude * Math.PI) / 180;
    const dLat = lat2 - lat1;
    const dLon = ((coord2.longitude - coord1.longitude) * Math.PI) / 180;

    const a =
      Math.sin(dLat / 2) ** 2 +
      Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return UNDERWATER_CONSTANTS.EARTH_RADIUS * c;
  }

  /**
   * Convert knots to meters per second
   */
  knotsToMetersPerSecond(knots: number): number {
    return knots * UNDERWATER_CONSTANTS.KNOTS_TO_MS;
  }

  /**
   * Convert meters per second to knots
   */
  metersPerSecondToKnots(ms: number): number {
    return ms / UNDERWATER_CONSTANTS.KNOTS_TO_MS;
  }
}

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * Create an underwater weapon system (torpedo)
 */
export function createUnderwaterWeapon(
  config: Partial<TorpedoConfiguration>
): TorpedoConfiguration {
  const sdk = new UnderwaterWeaponSDK();
  return sdk.createTorpedo(config);
}

/**
 * Calculate acoustic propagation range
 */
export function calculateAcousticRange(
  propagation: AcousticPropagation
): SonarRangeResult {
  const sdk = new UnderwaterWeaponSDK();
  return sdk.calculateSonarRange(propagation);
}

/**
 * Plan mine clearance operation
 */
export function planMineClearance(
  operation: MineClearanceOperation
): MineClearanceResult {
  const sdk = new UnderwaterWeaponSDK();
  return sdk.planMineClearance(operation);
}

// ============================================================================
// Re-export Types
// ============================================================================

export * from './types';
export { UnderwaterWeaponSDK };
