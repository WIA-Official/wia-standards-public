/**
 * WIA-DEF-007: Laser Weapon Standard - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Systems Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for directed energy laser weapon systems,
 * focusing on defensive applications such as C-RAM.
 */

import {
  Vector3,
  LaserSystemConfig,
  BeamParameters,
  BeamPropagation,
  AtmosphericConditions,
  AtmosphericAttenuation,
  ThermalBlooming,
  Target,
  EngagementParams,
  EngagementSolution,
  ValidationResult,
  SafetyCheck,
  ThermalState,
  ThermalManagementPlan,
  CRAMThreat,
  SimulationResult,
  PHYSICS_CONSTANTS,
  LaserErrorCode,
  LaserWeaponError,
  Result,
} from './types';

// ============================================================================
// Constants
// ============================================================================

const { SPEED_OF_LIGHT, PLANCK_CONSTANT, AIR_SPECIFIC_HEAT, AIR_DENSITY_STP } =
  PHYSICS_CONSTANTS;

const PI = Math.PI;

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate vector magnitude
 */
function vectorMagnitude(v: Vector3): number {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * Calculate distance between two points
 */
function distance(a: Vector3, b: Vector3): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const dz = b.z - a.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * Convert wavelength to photon energy
 */
function wavelengthToEnergy(wavelength: number): number {
  return (PLANCK_CONSTANT * SPEED_OF_LIGHT) / wavelength;
}

// ============================================================================
// Beam Calculations
// ============================================================================

/**
 * Calculate beam intensity at given power and radius
 *
 * I = P / (π × r²)
 *
 * @param power - Laser power in watts
 * @param beamRadius - Beam radius in meters
 * @returns Intensity in W/m²
 */
export function calculateBeamIntensity(
  power: number,
  beamRadius: number
): number {
  if (beamRadius <= 0) {
    throw new LaserWeaponError(
      LaserErrorCode.INVALID_PARAMETERS,
      'Beam radius must be positive'
    );
  }
  return power / (PI * beamRadius * beamRadius);
}

/**
 * Calculate beam divergence
 *
 * θ = M² × λ / (π × w₀)
 *
 * @param wavelength - Wavelength in meters
 * @param beamWaist - Beam waist radius in meters
 * @param beamQuality - M² beam quality factor
 * @returns Divergence angle in radians
 */
export function calculateBeamDivergence(
  wavelength: number,
  beamWaist: number,
  beamQuality: number = 1.0
): number {
  return (beamQuality * wavelength) / (PI * beamWaist);
}

/**
 * Calculate beam radius at given range
 *
 * r(z) = r₀ × √(1 + (z × θ / r₀)²)
 *
 * @param initialRadius - Initial beam radius in meters
 * @param range - Propagation distance in meters
 * @param divergence - Beam divergence in radians
 * @returns Beam radius at range in meters
 */
export function calculateBeamRadiusAtRange(
  initialRadius: number,
  range: number,
  divergence: number
): number {
  const ratio = (range * divergence) / initialRadius;
  return initialRadius * Math.sqrt(1 + ratio * ratio);
}

/**
 * Calculate Rayleigh range
 *
 * z_R = π × w₀² / λ
 *
 * @param beamWaist - Beam waist radius in meters
 * @param wavelength - Wavelength in meters
 * @returns Rayleigh range in meters
 */
export function calculateRayleighRange(
  beamWaist: number,
  wavelength: number
): number {
  return (PI * beamWaist * beamWaist) / wavelength;
}

/**
 * Calculate complete beam parameters at target range
 */
export function calculateBeamAtRange(params: {
  power: number;
  wavelength: number;
  beamQuality: number;
  initialRadius: number;
  range: number;
}): BeamParameters {
  const { power, wavelength, beamQuality, initialRadius, range } = params;

  const divergence = calculateBeamDivergence(
    wavelength,
    initialRadius,
    beamQuality
  );
  const radius = calculateBeamRadiusAtRange(initialRadius, range, divergence);
  const intensity = calculateBeamIntensity(power, radius);
  const rayleighRange = calculateRayleighRange(initialRadius, wavelength);

  return {
    radius,
    divergence,
    intensity,
    power,
    beamQuality,
    rayleighRange,
    waistRadius: initialRadius,
  };
}

// ============================================================================
// Atmospheric Calculations
// ============================================================================

/**
 * Calculate atmospheric attenuation coefficient
 *
 * Based on visibility and weather conditions
 *
 * @param conditions - Atmospheric conditions
 * @param wavelength - Laser wavelength in meters
 * @returns Attenuation coefficient in km⁻¹
 */
export function calculateAtmosphericAttenuation(
  conditions: AtmosphericConditions,
  wavelength: number = 1.064e-6
): number {
  const { visibility, humidity, precipitation = 0 } = conditions;

  // Base attenuation from visibility (aerosol scattering)
  // α ≈ 3.91 / V (where V is visibility in km)
  const visibilityKm = visibility / 1000;
  const aerosol = 3.91 / Math.max(visibilityKm, 0.1);

  // Molecular absorption (minimal at 1 μm)
  const molecular = 0.01;

  // Additional attenuation from precipitation
  let precipitation_attn = 0;
  if (precipitation > 0) {
    // Rain: approximately 5-10 km⁻¹ per mm/hr
    precipitation_attn = precipitation * 7;
  }

  // Humidity correction (minor at IR wavelengths)
  const humidity_factor = 1 + (humidity / 100) * 0.1;

  const total =
    (aerosol + molecular) * humidity_factor + precipitation_attn;

  return total;
}

/**
 * Calculate atmospheric transmission over distance
 *
 * T = exp(-α × L)
 *
 * @param conditions - Atmospheric conditions
 * @param distance - Propagation distance in meters
 * @param wavelength - Wavelength in meters
 * @returns Atmospheric attenuation object
 */
export function calculateAtmosphericTransmission(
  conditions: AtmosphericConditions,
  distance: number,
  wavelength: number = 1.064e-6
): AtmosphericAttenuation {
  const coefficient = calculateAtmosphericAttenuation(conditions, wavelength);
  const distanceKm = distance / 1000;
  const transmission = Math.exp(-coefficient * distanceKm);

  return {
    coefficient,
    aerosol: coefficient * 0.9, // Approximate breakdown
    molecular: coefficient * 0.05,
    scattering: coefficient * 0.05,
    transmission,
    distance,
  };
}

/**
 * Calculate thermal blooming effect
 *
 * @param power - Laser power in watts
 * @param range - Propagation distance in meters
 * @param beamRadius - Beam radius in meters
 * @param conditions - Atmospheric conditions
 * @returns Thermal blooming result
 */
export function calculateThermalBlooming(
  power: number,
  range: number,
  beamRadius: number,
  conditions: AtmosphericConditions
): ThermalBlooming {
  const { windSpeed = 5 } = conditions; // Default 5 m/s wind

  // Absorption coefficient (approximately 0.1 km⁻¹ for clear air)
  const k = 0.0001; // m⁻¹

  // Thermal expansion coefficient of air
  const alpha = 0.00367; // K⁻¹

  // Air density
  const rho = AIR_DENSITY_STP;

  // Thermal distortion parameter
  // N_TD = (k × α × P × L) / (ρ × C_p × v × r⁴)
  const r4 = Math.pow(beamRadius, 4);
  const numerator = k * alpha * power * range;
  const denominator = rho * AIR_SPECIFIC_HEAT * windSpeed * r4;
  const distortionParameter = numerator / denominator;

  // Beam deflection (simplified model)
  const deflection = distortionParameter * 0.001; // radians

  // Intensity reduction factor
  let reductionFactor = 1.0;
  let severity: ThermalBlooming['severity'] = 'negligible';

  if (distortionParameter < 0.1) {
    severity = 'negligible';
    reductionFactor = 0.95;
  } else if (distortionParameter < 0.3) {
    severity = 'minor';
    reductionFactor = 0.8;
  } else if (distortionParameter < 0.5) {
    severity = 'moderate';
    reductionFactor = 0.6;
  } else if (distortionParameter < 1.0) {
    severity = 'severe';
    reductionFactor = 0.3;
  } else {
    severity = 'critical';
    reductionFactor = 0.1;
  }

  return {
    distortionParameter,
    deflection,
    reductionFactor,
    severity,
  };
}

// ============================================================================
// Beam Propagation
// ============================================================================

/**
 * Simulate beam propagation through atmosphere
 */
export function simulateBeamPropagation(params: {
  power: number;
  wavelength: number;
  beamQuality: number;
  initialRadius: number;
  range: number;
  atmospheric: AtmosphericConditions;
}): BeamPropagation {
  const { power, wavelength, beamQuality, initialRadius, range, atmospheric } =
    params;

  // Calculate initial beam parameters
  const initial = calculateBeamAtRange({
    power,
    wavelength,
    beamQuality,
    initialRadius,
    range: 0,
  });

  // Calculate atmospheric transmission
  const attenuation = calculateAtmosphericTransmission(
    atmospheric,
    range,
    wavelength
  );

  // Calculate beam at target range (diffraction only)
  const divergence = calculateBeamDivergence(
    wavelength,
    initialRadius,
    beamQuality
  );
  const targetRadius = calculateBeamRadiusAtRange(
    initialRadius,
    range,
    divergence
  );

  // Calculate thermal blooming
  const blooming = calculateThermalBlooming(
    power,
    range,
    targetRadius,
    atmospheric
  );

  // Apply atmospheric transmission and thermal blooming
  const deliveredPower =
    power * attenuation.transmission * blooming.reductionFactor;
  const peakIntensity = calculateBeamIntensity(deliveredPower, targetRadius);

  const final: BeamParameters = {
    radius: targetRadius,
    divergence,
    intensity: peakIntensity,
    power: deliveredPower,
    beamQuality,
    rayleighRange: calculateRayleighRange(initialRadius, wavelength),
    waistRadius: initialRadius,
  };

  return {
    initial,
    final,
    distance: range,
    transmission: attenuation.transmission * blooming.reductionFactor,
    deliveredPower,
    spotSize: targetRadius * 2,
    peakIntensity,
  };
}

// ============================================================================
// Engagement Calculations
// ============================================================================

/**
 * Calculate time to kill for target
 *
 * TTK = (m × C_p × ΔT) / (I × A × η)
 *
 * @param target - Target characteristics
 * @param intensity - Beam intensity on target in W/m²
 * @returns Time to kill in seconds
 */
export function calculateTimeToKill(
  target: Target,
  intensity: number
): number {
  // Target-specific parameters
  let mass = 1.0; // kg (mass in beam)
  let specificHeat = 900; // J/(kg·K) for aluminum
  let tempRise = 300; // K to structural failure
  let absorption = 0.6; // Absorption efficiency

  // Adjust based on target type
  switch (target.type) {
    case 'mortar':
      mass = 0.5;
      specificHeat = 900;
      tempRise = 250;
      break;
    case 'rocket':
      mass = 1.0;
      specificHeat = 900;
      tempRise = 300;
      break;
    case 'artillery':
      mass = 2.0;
      specificHeat = 460; // steel
      tempRise = 400;
      break;
    case 'uav':
      mass = 0.3;
      specificHeat = 1000; // composite
      tempRise = 200;
      break;
    default:
      mass = 1.0;
      specificHeat = 900;
      tempRise = 300;
  }

  // Illuminated area (approximately target cross-section)
  const area = PI * Math.pow(target.size / 2, 2);

  // Energy required
  const energyRequired = mass * specificHeat * tempRise;

  // Power absorbed
  const powerAbsorbed = intensity * area * absorption;

  // Time to kill
  const ttk = energyRequired / Math.max(powerAbsorbed, 1);

  return ttk;
}

/**
 * Evaluate engagement feasibility
 */
export function evaluateEngagement(
  params: EngagementParams
): EngagementSolution {
  const { target, system, atmospheric } = params;

  const range = target.range;

  // Simulate beam propagation
  const propagation = simulateBeamPropagation({
    power: system.power,
    wavelength: system.wavelength,
    beamQuality: system.beamQuality,
    initialRadius: 0.1, // 10 cm aperture assumption
    range,
    atmospheric,
  });

  // Calculate time to kill
  const timeToKill = calculateTimeToKill(
    target,
    propagation.final.intensity
  );

  // Check if target will be in range long enough
  const targetVelocity = vectorMagnitude(target.velocity);
  const timeInRange = target.timeToImpact || 30; // seconds

  const feasible = timeToKill < timeInRange && timeToKill < 15; // Max 15s TTK

  // Calculate kill probability
  let killProbability = 0;
  if (feasible) {
    // Simple model: probability increases with margin
    const margin = timeInRange / timeToKill;
    killProbability = Math.min(0.95, 0.5 + margin * 0.2);
  }

  // Assess risk
  let risk: EngagementSolution['risk'] = 'low';
  if (timeToKill > 10) risk = 'medium';
  if (timeToKill > 15) risk = 'high';
  if (!feasible) risk = 'extreme';

  // Warnings
  const warnings: string[] = [];
  if (propagation.transmission < 0.5) {
    warnings.push('Atmospheric transmission < 50%');
  }
  if (timeToKill > 10) {
    warnings.push('Time to kill > 10 seconds');
  }
  if (range > 5000) {
    warnings.push('Target beyond optimal range');
  }

  // Limitations
  const limitations: string[] = [];
  if (!feasible) {
    if (timeToKill > timeInRange) {
      limitations.push('Insufficient dwell time');
    }
    if (propagation.deliveredPower < system.power * 0.1) {
      limitations.push('Excessive atmospheric attenuation');
    }
  }

  const now = new Date();
  const windowStart = new Date(now.getTime() + 2000); // 2s response time
  const windowEnd = new Date(
    now.getTime() + (target.timeToImpact || 30) * 1000
  );

  return {
    feasible,
    range,
    timeToKill,
    dwellTime: timeToKill,
    killProbability,
    beamAtTarget: propagation.final,
    energyRequired: propagation.deliveredPower * timeToKill,
    windowStart,
    windowEnd,
    risk,
    limitations: feasible ? undefined : limitations,
    warnings,
  };
}

/**
 * Validate engagement for safety and feasibility
 */
export function validateEngagement(params: EngagementParams): ValidationResult {
  const { target, system, atmospheric } = params;

  const errors: string[] = [];
  const warnings: string[] = [];
  const info: string[] = [];
  const safetyChecks: SafetyCheck[] = [];

  // System status check
  safetyChecks.push({
    name: 'System Status',
    passed: system.status === 'ready' || system.status === 'active',
    status: system.status === 'ready' ? 'pass' : 'fail',
    description: `Laser system is ${system.status}`,
    value: system.status,
    severity: 'critical',
  });

  if (system.status !== 'ready' && system.status !== 'active') {
    errors.push('Laser system not ready');
  }

  // Power check
  const minPower = 30000; // 30 kW minimum
  safetyChecks.push({
    name: 'Power Level',
    passed: system.power >= minPower,
    status: system.power >= minPower ? 'pass' : 'fail',
    description: 'Sufficient laser power',
    value: system.power,
    threshold: minPower,
    severity: 'error',
  });

  if (system.power < minPower) {
    errors.push('Insufficient laser power for engagement');
  }

  // Range check
  const maxRange = 5000; // 5 km max
  safetyChecks.push({
    name: 'Range Limit',
    passed: target.range <= maxRange,
    status: target.range <= maxRange ? 'pass' : 'warning',
    description: 'Target within effective range',
    value: target.range,
    threshold: maxRange,
    severity: 'warning',
  });

  if (target.range > maxRange) {
    warnings.push('Target beyond recommended range');
  }

  // IFF check
  const iffStatus =
    target.iff === 'hostile'
      ? 'cleared'
      : target.iff === 'friendly'
        ? 'friendly'
        : target.iff === 'neutral'
          ? 'unknown'
          : 'unknown';

  safetyChecks.push({
    name: 'IFF Status',
    passed: iffStatus === 'cleared',
    status: iffStatus === 'cleared' ? 'pass' : 'fail',
    description: `IFF: ${iffStatus}`,
    value: iffStatus,
    severity: 'critical',
  });

  if (iffStatus !== 'cleared') {
    errors.push('IFF not cleared - target not confirmed hostile');
  }

  // Atmospheric check
  const attenuation = calculateAtmosphericTransmission(
    atmospheric,
    target.range,
    system.wavelength
  );

  safetyChecks.push({
    name: 'Atmospheric Transmission',
    passed: attenuation.transmission > 0.3,
    status: attenuation.transmission > 0.5 ? 'pass' : 'warning',
    description: 'Atmospheric conditions acceptable',
    value: attenuation.transmission,
    threshold: 0.5,
    severity: 'warning',
  });

  if (attenuation.transmission < 0.3) {
    warnings.push('Poor atmospheric transmission');
  }

  // Background safety (simplified)
  const backgroundSafe = true; // Would check beam termination point
  safetyChecks.push({
    name: 'Background Safety',
    passed: backgroundSafe,
    status: 'pass',
    description: 'Safe beam termination',
    severity: 'critical',
  });

  // Collateral damage assessment
  const collateralDamage: ValidationResult['collateralDamage'] =
    target.range > 1000 ? 'none' : 'low';

  // Overall validation
  const isValid = errors.length === 0;

  // Recommendation
  let recommendation: ValidationResult['recommendation'];
  if (!isValid) {
    recommendation = 'do-not-engage';
  } else if (warnings.length > 2) {
    recommendation = 'engage-with-caution';
  } else {
    recommendation = 'engage';
  }

  return {
    isValid,
    errors,
    warnings,
    info,
    safetyChecks,
    iffStatus,
    collateralDamage,
    backgroundSafe,
    recommendation,
  };
}

// ============================================================================
// Thermal Management
// ============================================================================

/**
 * Calculate thermal state
 */
export function calculateThermalState(
  system: LaserSystemConfig,
  currentPower: number,
  dutyCycle: number
): ThermalState {
  const wasteHeat = currentPower * (1 - system.efficiency);
  const avgWasteHeat = wasteHeat * dutyCycle;

  const thermalMargin = system.coolingCapacity - avgWasteHeat;

  let status: ThermalState['status'];
  if (thermalMargin > system.coolingCapacity * 0.3) {
    status = 'normal';
  } else if (thermalMargin > 0) {
    status = 'warm';
  } else if (thermalMargin > -system.coolingCapacity * 0.2) {
    status = 'hot';
  } else {
    status = 'overtemp';
  }

  const maxDutyCycle = system.coolingCapacity / wasteHeat;

  return {
    wasteHeat,
    coolingCapacity: system.coolingCapacity,
    flowRate: 20, // L/min (example)
    inletTemp: 20,
    outletTemp: 35,
    temperatures: {
      laserHead: 40,
      pumpDiodes: 35,
      optics: 25,
      coolant: 30,
    },
    dutyCycle,
    maxDutyCycle: Math.min(maxDutyCycle, 1.0),
    thermalMargin,
    status,
  };
}

/**
 * Generate thermal management plan
 */
export function generateThermalPlan(
  power: number,
  efficiency: number,
  coolingCapacity: number,
  desiredDutyCycle: number
): ThermalManagementPlan {
  const wasteHeat = power * (1 - efficiency);
  const requiredCooling = wasteHeat * desiredDutyCycle;

  const feasible = requiredCooling <= coolingCapacity;

  const maxContinuousOperation = feasible
    ? 3600 // 1 hour if feasible
    : (coolingCapacity / wasteHeat) * 3600;

  const warnings: string[] = [];
  if (!feasible) {
    warnings.push('Requested duty cycle exceeds cooling capacity');
  }

  const coolDownPeriods = [];
  if (desiredDutyCycle > 0.5) {
    coolDownPeriods.push({
      afterSeconds: 1800, // After 30 min
      durationSeconds: 300, // 5 min cool-down
    });
  }

  return {
    operatingPower: power,
    plannedDutyCycle: desiredDutyCycle,
    requiredCooling,
    coolDownPeriods,
    maxContinuousOperation,
    feasible,
    warnings,
  };
}

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * Laser Weapon SDK
 *
 * Main interface for laser weapon calculations and simulations
 */
export class LaserWeaponSDK {
  private version = '1.0.0';

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Create a laser system configuration
   */
  createLaserSystem(
    config: Partial<LaserSystemConfig> & {
      type: LaserSystemConfig['type'];
      power: number;
    }
  ): LaserSystemConfig {
    const defaults: Partial<LaserSystemConfig> = {
      wavelength: 1.064e-6, // 1064 nm
      beamQuality: 1.5,
      efficiency: 0.3,
      coolingCapacity: config.power * 2.5,
      maxDutyCycle: 0.8,
      status: 'ready',
    };

    return {
      id: `laser-${Date.now()}`,
      ...defaults,
      ...config,
    } as LaserSystemConfig;
  }

  /**
   * Calculate beam intensity
   */
  calculateBeamIntensity = calculateBeamIntensity;

  /**
   * Calculate beam at range
   */
  calculateBeamAtRange = calculateBeamAtRange;

  /**
   * Simulate beam propagation
   */
  simulateBeamPropagation = simulateBeamPropagation;

  /**
   * Calculate atmospheric transmission
   */
  calculateAtmosphericTransmission = calculateAtmosphericTransmission;

  /**
   * Calculate thermal blooming
   */
  calculateThermalBlooming = calculateThermalBlooming;

  /**
   * Calculate time to kill
   */
  calculateTimeToKill = calculateTimeToKill;

  /**
   * Evaluate engagement
   */
  evaluateEngagement = evaluateEngagement;

  /**
   * Validate engagement
   */
  validateEngagement = validateEngagement;

  /**
   * Calculate thermal state
   */
  calculateThermalState = calculateThermalState;

  /**
   * Generate thermal management plan
   */
  generateThermalPlan = generateThermalPlan;

  /**
   * Run complete engagement simulation
   */
  simulateEngagement(params: EngagementParams): SimulationResult {
    const startTime = Date.now();

    try {
      const solution = this.evaluateEngagement(params);
      const validation = this.validateEngagement(params);

      const propagation = this.simulateBeamPropagation({
        power: params.system.power,
        wavelength: params.system.wavelength,
        beamQuality: params.system.beamQuality,
        initialRadius: 0.1,
        range: params.target.range,
        atmospheric: params.atmospheric,
      });

      const blooming = this.calculateThermalBlooming(
        params.system.power,
        params.target.range,
        propagation.final.radius,
        params.atmospheric
      );

      const duration = Date.now() - startTime;

      return {
        id: `sim-${Date.now()}`,
        parameters: params,
        solution,
        propagation,
        thermalBlooming: blooming,
        validation,
        duration,
        success: solution.feasible && validation.isValid,
      };
    } catch (error) {
      const duration = Date.now() - startTime;
      return {
        id: `sim-${Date.now()}`,
        parameters: params,
        solution: {} as EngagementSolution,
        propagation: {} as BeamPropagation,
        thermalBlooming: {} as ThermalBlooming,
        validation: {} as ValidationResult,
        duration,
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';

export {
  // Beam functions
  calculateBeamDivergence,
  calculateBeamRadiusAtRange,
  calculateRayleighRange,
  calculateBeamAtRange,
  calculateBeamIntensity,

  // Atmospheric functions
  calculateAtmosphericAttenuation,
  calculateAtmosphericTransmission,
  calculateThermalBlooming,

  // Propagation
  simulateBeamPropagation,

  // Engagement
  calculateTimeToKill,
  evaluateEngagement,
  validateEngagement,

  // Thermal
  calculateThermalState,
  generateThermalPlan,
};

// Default export
export default LaserWeaponSDK;
