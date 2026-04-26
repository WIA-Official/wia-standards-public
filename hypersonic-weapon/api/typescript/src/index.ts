/**
 * WIA-DEF-008: Hypersonic Weapon SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for hypersonic weapon systems including:
 * - Aerodynamic calculations
 * - Thermal protection analysis
 * - Trajectory optimization
 * - Guidance system modeling
 * - Detectability analysis
 */

import {
  HeatFluxParams,
  HeatFluxResult,
  MaterialType,
  TrajectoryParams,
  TrajectoryResult,
  TrajectoryPoint,
  DetectabilityParams,
  DetectabilityResult,
  SimulationParams,
  SimulationResult,
  FlightConditions,
  AerodynamicCoefficients,
  ScramjetParams,
  EnginePerformance,
  GuidanceParams,
  GuidanceResult,
  VehicleType,
  Vector3,
  HYPERSONIC_CONSTANTS,
  HypersonicErrorCode,
  HypersonicError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-008 Hypersonic Weapon SDK
 */
export class HypersonicSDK {
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
   * Calculate heat flux at a point on the vehicle
   *
   * @param params - Heat flux calculation parameters
   * @returns Heat flux result with temperature and material recommendations
   */
  calculateHeatFlux(params: HeatFluxParams): HeatFluxResult {
    const { velocity, altitude, noseRadius, position = 0 } = params;

    // Validate inputs
    if (velocity < HYPERSONIC_CONSTANTS.MACH_5_THRESHOLD) {
      throw new HypersonicError(
        HypersonicErrorCode.VELOCITY_TOO_LOW,
        `Velocity must be at least Mach 5 (${HYPERSONIC_CONSTANTS.MACH_5_THRESHOLD} m/s)`
      );
    }

    if (altitude < 0 || altitude > 100000) {
      throw new HypersonicError(
        HypersonicErrorCode.ALTITUDE_OUT_OF_RANGE,
        'Altitude must be between 0 and 100,000 meters'
      );
    }

    // Calculate atmospheric density (exponential model)
    const density = this.getAtmosphericDensity(altitude);
    const temperature = this.getAtmosphericTemperature(altitude);

    // Heat flux using Fay-Riddell equation (simplified)
    // Q̇ = 1.83 × 10⁻⁴ × √(ρ/R) × v³
    const heatFlux =
      1.83e-4 * Math.sqrt(density / noseRadius) * Math.pow(velocity, 3);

    // Adjust for position along vehicle (nose sees highest heating)
    const positionFactor = 1.0 - 0.7 * position; // 30% reduction at tail
    const adjustedHeatFlux = heatFlux * positionFactor;

    // Calculate stagnation temperature
    // T₀ = T∞ × (1 + (γ-1)/2 × M²)
    const machNumber = this.calculateMachNumber(velocity, altitude);
    const gamma = HYPERSONIC_CONSTANTS.GAMMA_AIR;
    const stagnationTemperature =
      temperature * (1 + ((gamma - 1) / 2) * machNumber * machNumber);

    // Estimate wall temperature (radiation equilibrium)
    // Q̇ = εσT⁴
    const emissivity = 0.8; // Typical for oxidized surfaces
    const sigma = HYPERSONIC_CONSTANTS.STEFAN_BOLTZMANN;
    const wallTemperature = Math.pow(
      adjustedHeatFlux / (emissivity * sigma),
      0.25
    );

    // Determine required material
    const recommendedMaterial = this.selectMaterial(wallTemperature);

    // Check if active cooling is needed
    const needsActiveCooling = adjustedHeatFlux > 5e6; // 5 MW/m²

    let coolingRequired: number | undefined;
    if (needsActiveCooling) {
      // Cooling requirement is the heat flux that exceeds material capacity
      const materialCapacity = this.getMaterialCapacity(recommendedMaterial);
      coolingRequired = (adjustedHeatFlux - materialCapacity) / 1000; // Convert to kW/m²
    }

    return {
      heatFlux: adjustedHeatFlux,
      stagnationTemperature,
      wallTemperature,
      recommendedMaterial,
      coolingRequired,
      needsActiveCooling,
    };
  }

  /**
   * Optimize trajectory for maximum range or specific target
   *
   * @param params - Trajectory optimization parameters
   * @returns Optimized trajectory with flight profile
   */
  optimizeTrajectory(params: TrajectoryParams): TrajectoryResult {
    const {
      launchAngle,
      initialVelocity,
      targetRange,
      vehicleType,
      liftToDragRatio = 5,
      constraints,
    } = params;

    // Validate inputs
    if (initialVelocity < HYPERSONIC_CONSTANTS.MACH_5_THRESHOLD) {
      throw new HypersonicError(
        HypersonicErrorCode.VELOCITY_TOO_LOW,
        'Initial velocity must be hypersonic (Mach 5+)'
      );
    }

    const g = HYPERSONIC_CONSTANTS.GRAVITY;
    const angleRad = (launchAngle * Math.PI) / 180;

    // Calculate ballistic range (no lift)
    const ballisticRange =
      (initialVelocity * initialVelocity * Math.sin(2 * angleRad)) / g;

    // Apply skip-glide enhancement for HGV
    let skipFactor = 0;
    let skipCount = 0;

    if (vehicleType === 'hgv') {
      // Skip-glide adds 30-50% range
      skipFactor = 0.3 + 0.2 * (liftToDragRatio / 10); // Higher L/D = better skip
      skipCount = Math.floor(2 + liftToDragRatio / 3); // More skips with higher L/D
    } else if (vehicleType === 'hcm') {
      // Cruise missile with powered flight
      skipFactor = 0.1; // Modest glide benefit
      skipCount = 1;
    }

    const range = ballisticRange * (1 + skipFactor);

    // Flight time estimation
    const averageVelocity = initialVelocity * 0.7; // Account for drag
    const flightTime = range / averageVelocity;

    // Maximum altitude (boost phase apex)
    const maxAltitude =
      (initialVelocity * initialVelocity * Math.sin(angleRad) * Math.sin(angleRad)) /
      (2 * g);

    // Generate trajectory profile
    const profile = this.generateTrajectoryProfile(
      initialVelocity,
      angleRad,
      flightTime,
      maxAltitude,
      skipCount
    );

    // Terminal conditions
    const terminalVelocity = initialVelocity * 0.6; // Energy loss during flight
    const impactAngle = 60 + Math.random() * 20; // 60-80 degrees (steep)

    return {
      range,
      skipCount,
      flightTime,
      maxAltitude,
      averageVelocity,
      profile,
      terminalVelocity,
      impactAngle,
    };
  }

  /**
   * Analyze detectability of hypersonic vehicle
   *
   * @param params - Detectability analysis parameters
   * @returns Detection ranges and vulnerability assessment
   */
  analyzeDetectability(params: DetectabilityParams): DetectabilityResult {
    const {
      altitude,
      velocity,
      radarCrossSection,
      surfaceTemperature = 1500,
      enginePlumeVisible = false,
      radarType = 'x-band',
    } = params;

    // Radar detection range (simplified radar equation)
    // R = (PtGtGrλ²σ / ((4π)³Smin))^(1/4)
    const radarPowerFactor = radarType === 'oth' ? 100 : radarType === 'x-band' ? 50 : 30;
    const radarDetectionRange =
      radarPowerFactor * 1000 * Math.pow(radarCrossSection / 0.01, 0.25);

    // Infrared detection based on temperature and area
    // I = εσT⁴A
    const emissivity = 0.8;
    const sigma = HYPERSONIC_CONSTANTS.STEFAN_BOLTZMANN;
    const estimatedArea = 2.0; // m²
    const radiantIntensity =
      emissivity *
      sigma *
      Math.pow(surfaceTemperature, 4) *
      estimatedArea;

    // IR detection range (simplified)
    const baseIRRange = 80000; // 80 km baseline
    const temperatureFactor = Math.pow(surfaceTemperature / 1000, 2);
    const infraredDetectionRange =
      baseIRRange * temperatureFactor * (enginePlumeVisible ? 1.5 : 1.0);

    // Stealth rating (0=easily detected, 1=very stealthy)
    const stealthRating =
      1.0 -
      (radarCrossSection / 1.0) * 0.5 -
      (temperatureFactor - 1) * 0.3 -
      (enginePlumeVisible ? 0.2 : 0);
    const clampedStealthRating = Math.max(0, Math.min(1, stealthRating));

    // Detection time before impact
    const effectiveDetectionRange = Math.max(
      radarDetectionRange,
      infraredDetectionRange
    );
    const detectionTime = effectiveDetectionRange / velocity;

    // Countermeasures
    const countermeasures: string[] = [];
    if (radarCrossSection > 0.01)
      countermeasures.push('Reduce radar cross-section with stealth shaping');
    if (surfaceTemperature > 1200)
      countermeasures.push('Apply low-emissivity coating');
    if (enginePlumeVisible) countermeasures.push('Shield engine plume');
    if (altitude < 30000) countermeasures.push('Fly at higher altitude');
    countermeasures.push('Use evasive maneuvers');
    countermeasures.push('Deploy electronic countermeasures');

    // Vulnerability assessment
    let vulnerability: 'low' | 'medium' | 'high';
    if (detectionTime > 300) vulnerability = 'low'; // >5 minutes
    else if (detectionTime > 120) vulnerability = 'medium'; // 2-5 minutes
    else vulnerability = 'high'; // <2 minutes

    return {
      radarDetectionRange,
      infraredDetectionRange,
      stealthRating: clampedStealthRating,
      detectionTime,
      countermeasures,
      vulnerability,
    };
  }

  /**
   * Simulate complete hypersonic flight
   *
   * @param params - Simulation parameters
   * @returns Complete simulation result
   */
  simulateHypersonicFlight(params: SimulationParams): SimulationResult {
    const startTime = Date.now();

    try {
      // Extract parameters
      const { vehicle, flight, environment } = params;

      // Calculate trajectory
      const trajectory = this.optimizeTrajectory({
        launchAngle: flight.launchAngle,
        initialVelocity: flight.initialVelocity,
        targetRange: flight.targetRange,
        vehicleType: vehicle.type,
        liftToDragRatio: 5, // Default from waverider design
      });

      // Analyze thermal loads
      const maxHeatPoint = trajectory.profile.reduce((max, point) =>
        point.heatFlux > max.heatFlux ? point : max
      );

      const thermalAnalysis = this.calculateHeatFlux({
        velocity: maxHeatPoint.velocity.x,
        altitude: maxHeatPoint.altitude,
        noseRadius: vehicle.geometry.noseRadius,
      });

      // Check TPS adequacy
      const materialMaxTemp = this.getMaterialMaxTemp(vehicle.tps);
      const tpsAdequacy = thermalAnalysis.wallTemperature < materialMaxTemp;

      // Analyze detectability
      const detectability = this.analyzeDetectability({
        altitude: 30000, // Representative cruise altitude
        velocity: flight.initialVelocity * 0.8,
        radarCrossSection: 0.01, // Small RCS for stealth design
        surfaceTemperature: thermalAnalysis.wallTemperature,
        enginePlumeVisible: vehicle.propulsion.includes('scramjet'),
      });

      // Calculate accuracy (CEP)
      const baseCEP = 50; // meters (baseline INS)
      const terrainMatchingBonus = vehicle.type === 'hcm' ? 0.5 : 0.8;
      const cep = baseCEP * terrainMatchingBonus;

      // Defensive rating
      const defensiveRating = this.calculateDefensiveRating(
        detectability.stealthRating,
        trajectory.range,
        cep
      );

      const duration = Date.now() - startTime;

      return {
        id: `SIM-${Date.now()}-${Math.random().toString(36).substr(2, 6)}`,
        success: true,
        trajectory,
        thermal: {
          maxHeatFlux: maxHeatPoint.heatFlux,
          maxTemperature: thermalAnalysis.wallTemperature,
          totalHeatLoad: maxHeatPoint.heatFlux * trajectory.flightTime * 1e-6, // MJ
          tpsAdequacy,
        },
        performance: {
          range: trajectory.range,
          accuracy: cep,
          flightTime: trajectory.flightTime,
          terminalVelocity: trajectory.terminalVelocity,
        },
        detectability,
        defensiveRating,
      };
    } catch (error) {
      return {
        id: `SIM-${Date.now()}-ERROR`,
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
        trajectory: {} as any,
        thermal: {} as any,
        performance: {} as any,
        detectability: {} as any,
        defensiveRating: {} as any,
      };
    }
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Get atmospheric density at altitude (exponential model)
   */
  private getAtmosphericDensity(altitude: number): number {
    const rho0 = HYPERSONIC_CONSTANTS.STANDARD_DENSITY;
    const scaleHeight = 8500; // meters
    return rho0 * Math.exp(-altitude / scaleHeight);
  }

  /**
   * Get atmospheric temperature at altitude
   */
  private getAtmosphericTemperature(altitude: number): number {
    const T0 = HYPERSONIC_CONSTANTS.STANDARD_TEMPERATURE;
    if (altitude < 11000) {
      // Troposphere
      return T0 - 0.0065 * altitude;
    } else if (altitude < 25000) {
      // Lower stratosphere
      return 216.65;
    } else {
      // Upper stratosphere
      return 216.65 + 0.003 * (altitude - 25000);
    }
  }

  /**
   * Calculate Mach number from velocity and altitude
   */
  private calculateMachNumber(velocity: number, altitude: number): number {
    const temperature = this.getAtmosphericTemperature(altitude);
    const gamma = HYPERSONIC_CONSTANTS.GAMMA_AIR;
    const R = HYPERSONIC_CONSTANTS.GAS_CONSTANT_AIR;
    const speedOfSound = Math.sqrt(gamma * R * temperature);
    return velocity / speedOfSound;
  }

  /**
   * Select appropriate material based on temperature
   */
  private selectMaterial(temperature: number): MaterialType {
    if (temperature > 3200) return 'uhtc';
    if (temperature > 2500) return 'carbon-carbon';
    if (temperature > 2000) return 'rcc';
    if (temperature > 1500) return 'ceramic-composite';
    if (temperature > 1000) return 'tungsten';
    return 'ablative';
  }

  /**
   * Get material heat capacity
   */
  private getMaterialCapacity(material: MaterialType): number {
    const capacities: Record<MaterialType, number> = {
      'carbon-carbon': 8e6,
      rcc: 6e6,
      uhtc: 10e6,
      ablative: 4e6,
      tungsten: 9e6,
      'ceramic-composite': 5e6,
    };
    return capacities[material];
  }

  /**
   * Get material maximum temperature
   */
  private getMaterialMaxTemp(material: MaterialType): number {
    const temps: Record<MaterialType, number> = {
      'carbon-carbon': 3000 + 273,
      rcc: 2200 + 273,
      uhtc: 3500 + 273,
      ablative: 2200 + 273,
      tungsten: 3400 + 273,
      'ceramic-composite': 1500 + 273,
    };
    return temps[material];
  }

  /**
   * Generate trajectory profile points
   */
  private generateTrajectoryProfile(
    initialVelocity: number,
    angleRad: number,
    flightTime: number,
    maxAltitude: number,
    skipCount: number
  ): TrajectoryPoint[] {
    const points: TrajectoryPoint[] = [];
    const steps = 100;
    const dt = flightTime / steps;

    for (let i = 0; i <= steps; i++) {
      const t = i * dt;
      const progress = t / flightTime;

      // Sinusoidal altitude profile for skip-glide
      let altitude = maxAltitude * Math.sin(Math.PI * progress);
      if (skipCount > 0) {
        altitude *= 1 + 0.3 * Math.sin(skipCount * Math.PI * progress);
      }

      const velocity = initialVelocity * (1 - 0.4 * progress); // Velocity decay
      const machNumber = this.calculateMachNumber(velocity, altitude);

      const heatFlux = this.calculateHeatFlux({
        velocity,
        altitude,
        noseRadius: 0.15,
      }).heatFlux;

      const acceleration = 15 * (1 - progress); // Deceleration in g's

      points.push({
        time: t,
        position: {
          x: velocity * t * Math.cos(angleRad),
          y: altitude,
          z: 0,
        },
        velocity: {
          x: velocity * Math.cos(angleRad),
          y: velocity * Math.sin(angleRad),
          z: 0,
        },
        altitude,
        machNumber,
        heatFlux,
        acceleration,
      });
    }

    return points;
  }

  /**
   * Calculate defensive rating
   */
  private calculateDefensiveRating(
    stealthRating: number,
    range: number,
    cep: number
  ) {
    const overall = 0.4 * stealthRating + 0.3 * (range / 5000000) + 0.3 * (1 - cep / 100);

    let deterrenceValue: 'low' | 'medium' | 'high' | 'strategic';
    if (overall > 0.8) deterrenceValue = 'strategic';
    else if (overall > 0.6) deterrenceValue = 'high';
    else if (overall > 0.4) deterrenceValue = 'medium';
    else deterrenceValue = 'low';

    const recommendedRole =
      deterrenceValue === 'strategic'
        ? 'Strategic deterrence and defense'
        : deterrenceValue === 'high'
          ? 'Regional deterrence'
          : deterrenceValue === 'medium'
            ? 'Tactical strike'
            : 'Research and development';

    return {
      overall: Math.max(0, Math.min(1, overall)),
      deterrenceValue,
      recommendedRole,
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate heat flux (standalone function)
 */
export function calculateHeatFlux(params: HeatFluxParams): HeatFluxResult {
  const sdk = new HypersonicSDK();
  return sdk.calculateHeatFlux(params);
}

/**
 * Optimize trajectory (standalone function)
 */
export function optimizeTrajectory(params: TrajectoryParams): TrajectoryResult {
  const sdk = new HypersonicSDK();
  return sdk.optimizeTrajectory(params);
}

/**
 * Analyze detectability (standalone function)
 */
export function analyzeDetectability(params: DetectabilityParams): DetectabilityResult {
  const sdk = new HypersonicSDK();
  return sdk.analyzeDetectability(params);
}

/**
 * Simulate hypersonic flight (standalone function)
 */
export function simulateHypersonicFlight(params: SimulationParams): SimulationResult {
  const sdk = new HypersonicSDK();
  return sdk.simulateHypersonicFlight(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { HypersonicSDK };
export default HypersonicSDK;
