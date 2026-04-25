/**
 * WIA-AUTO-019: Hyperloop Transportation SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Mobility Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for hyperloop systems including:
 * - Aerodynamic drag calculations
 * - Magnetic levitation force computations
 * - Linear motor thrust analysis
 * - Energy consumption modeling
 * - Journey simulation
 * - Pod design validation
 */

import {
  DragCalculation,
  DragResult,
  LevitationParams,
  LevitationResult,
  LinearMotorParams,
  ThrustResult,
  JourneyParams,
  EnergyResult,
  PodDesign,
  ValidationResult,
  SimulationParams,
  SimulationResult,
  Route,
  JourneyPhase,
  PHYSICS_CONSTANTS,
  DESIGN_CONSTANTS,
  HyperloopErrorCode,
  HyperloopError,
  AerodynamicProfile,
  SafetyCheck,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUTO-019 Hyperloop Transportation SDK
 */
export class HyperloopSDK {
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
   * Calculate drag force in low-pressure environment
   *
   * @param params - Drag calculation parameters
   * @returns Drag force and power requirements
   */
  calculateDragForce(params: DragCalculation): DragResult {
    const {
      pressure,
      velocity,
      dragCoefficient,
      crossSectionArea,
      temperature = PHYSICS_CONSTANTS.STANDARD_TEMPERATURE,
    } = params;

    // Validate inputs
    if (pressure <= 0) {
      throw new HyperloopError(
        HyperloopErrorCode.INVALID_PARAMETERS,
        'Pressure must be positive'
      );
    }

    if (velocity < 0) {
      throw new HyperloopError(
        HyperloopErrorCode.INVALID_PARAMETERS,
        'Velocity must be non-negative'
      );
    }

    // Calculate air density using ideal gas law: ρ = (P × M) / (R × T)
    const airDensity =
      (pressure * PHYSICS_CONSTANTS.MOLAR_MASS_AIR) /
      (PHYSICS_CONSTANTS.GAS_CONSTANT * temperature);

    // Calculate drag force: F = 0.5 × ρ × v² × Cd × A
    const force =
      0.5 * airDensity * velocity * velocity * dragCoefficient * crossSectionArea;

    // Calculate power: P = F × v
    const power = force * velocity;

    // Calculate energy per 100 km in kWh
    const energyPer100km = (power * (100000 / velocity)) / 3600000; // Convert J to kWh

    // Calculate Reynolds number
    const dynamicViscosity = 1.81e-5; // Pa·s at 20°C
    const characteristicLength = Math.sqrt((4 * crossSectionArea) / Math.PI); // Equivalent diameter
    const reynoldsNumber =
      (airDensity * velocity * characteristicLength) / dynamicViscosity;

    return {
      force,
      power,
      energyPer100km,
      airDensity,
      reynoldsNumber,
    };
  }

  /**
   * Calculate magnetic levitation force
   *
   * @param params - Levitation parameters
   * @returns Levitation force and stability analysis
   */
  calculateLevitationForce(params: LevitationParams): LevitationResult {
    const {
      magneticFieldStrength,
      effectiveArea,
      podMass,
      levitationGap,
      numberOfMagnets = 8,
    } = params;

    // Validate inputs
    if (magneticFieldStrength <= 0) {
      throw new HyperloopError(
        HyperloopErrorCode.INVALID_PARAMETERS,
        'Magnetic field strength must be positive'
      );
    }

    if (podMass <= 0) {
      throw new HyperloopError(
        HyperloopErrorCode.INVALID_PARAMETERS,
        'Pod mass must be positive'
      );
    }

    // Calculate required force to levitate: F = m × g
    const requiredForce = podMass * PHYSICS_CONSTANTS.GRAVITY;

    // Calculate levitation force per magnet: F = (B² × A) / (2μ₀)
    const forcePerMagnet =
      (magneticFieldStrength * magneticFieldStrength * effectiveArea) /
      (2 * PHYSICS_CONSTANTS.MAGNETIC_PERMEABILITY);

    // Total force from all magnets
    const force = forcePerMagnet * numberOfMagnets;

    // Calculate safety margin
    const safetyMargin = force / requiredForce;

    // Power consumption estimation (based on empirical data)
    // Typical: 5-10 kW per magnet at operating conditions
    const powerPerMagnet = 7.5; // kW (average)
    const powerRequired = powerPerMagnet * numberOfMagnets;

    // Gap stability factor (simplified model)
    // Stability decreases with larger gaps
    const nominalGap = DESIGN_CONSTANTS.LEVITATION_GAP; // mm
    const gapDeviation = Math.abs(levitationGap - nominalGap) / nominalGap;
    const gapStability = Math.max(0, 1 - gapDeviation * 2);

    // System is stable if force exceeds requirement with margin and gap is stable
    const isStable = safetyMargin >= 1.5 && gapStability >= 0.8;

    return {
      force,
      requiredForce,
      powerRequired,
      gapStability,
      isStable,
      safetyMargin,
    };
  }

  /**
   * Calculate linear induction motor thrust
   *
   * @param params - Linear motor parameters
   * @returns Thrust force and motor characteristics
   */
  calculateLinearMotorThrust(params: LinearMotorParams): ThrustResult {
    const {
      magneticField,
      current,
      conductorLength,
      numberOfPhases,
      powerFactorAngle,
      frequency,
      polePitch,
    } = params;

    // Validate inputs
    if (magneticField <= 0 || current <= 0) {
      throw new HyperloopError(
        HyperloopErrorCode.INVALID_PARAMETERS,
        'Magnetic field and current must be positive'
      );
    }

    // Calculate thrust force: F = B × I × L × n × cos(φ)
    const force =
      magneticField *
      current *
      conductorLength *
      numberOfPhases *
      Math.cos(powerFactorAngle);

    // Calculate synchronous speed: v_sync = 2 × τ × f
    const synchronousSpeed = 2 * polePitch * frequency;

    // Calculate motor power (apparent): P = √3 × V × I
    // Simplified estimation: V ≈ B × L × v_sync
    const voltage = magneticField * conductorLength * synchronousSpeed;
    const power = (Math.sqrt(3) * voltage * current * Math.cos(powerFactorAngle)) / 1000; // kW

    // Motor efficiency (typical for LIM: 85-90%)
    const efficiency = 0.87;

    // Calculate slip (simplified, actual slip depends on load)
    const slip = 0.05; // Typical 5% at rated load

    return {
      force,
      synchronousSpeed,
      power,
      efficiency,
      slip,
    };
  }

  /**
   * Calculate energy consumption for a journey
   *
   * @param params - Journey parameters
   * @returns Energy consumption breakdown
   */
  calculateEnergyConsumption(params: JourneyParams): EnergyResult {
    const {
      distance,
      podMass,
      maxSpeed,
      passengers,
      cargo = 0,
      elevationChange = 0,
      passengerMass = 75,
    } = params;

    // Validate inputs
    if (distance <= 0) {
      throw new HyperloopError(
        HyperloopErrorCode.INVALID_PARAMETERS,
        'Distance must be positive'
      );
    }

    if (maxSpeed > PHYSICS_CONSTANTS.MAX_SPEED) {
      throw new HyperloopError(
        HyperloopErrorCode.SPEED_LIMIT_EXCEEDED,
        `Speed ${maxSpeed} m/s exceeds maximum ${PHYSICS_CONSTANTS.MAX_SPEED} m/s`
      );
    }

    // Calculate total mass
    const totalMass = podMass + passengers * passengerMass + cargo;

    // 1. Acceleration Energy: E = 0.5 × m × v²
    const kineticEnergy = 0.5 * totalMass * maxSpeed * maxSpeed;
    const accelerationEnergy = kineticEnergy / (3600000 * 0.85); // Convert to kWh with motor efficiency

    // 2. Cruising Energy (drag): E = F_drag × distance
    const dragCalc = this.calculateDragForce({
      pressure: PHYSICS_CONSTANTS.TARGET_PRESSURE,
      velocity: maxSpeed,
      dragCoefficient: DESIGN_CONSTANTS.DRAG_COEFFICIENT,
      crossSectionArea: Math.PI * Math.pow(DESIGN_CONSTANTS.POD_DIAMETER / 2, 2),
    });

    const cruisingEnergy = (dragCalc.force * distance) / 3600000; // Convert J to kWh

    // 3. Braking Energy (regenerative recovery)
    const regenerativeEfficiency = 0.75; // 75% recovery
    const brakingEnergy = -accelerationEnergy * regenerativeEfficiency;

    // 4. Elevation Energy: E = m × g × h
    const elevationEnergy =
      (totalMass * PHYSICS_CONSTANTS.GRAVITY * elevationChange) / 3600000; // kWh

    // 5. Levitation Energy: Power × Time
    const travelTime = distance / maxSpeed; // seconds
    const levitationPower = 75; // kW (typical for 15,000 kg pod)
    const levitationEnergy = (levitationPower * travelTime) / 3600; // kWh

    // 6. Auxiliary Systems: HVAC, lights, communications, etc.
    const auxiliaryPower = 25; // kW
    const auxiliaryEnergy = (auxiliaryPower * travelTime) / 3600; // kWh

    // Total energy
    const total =
      accelerationEnergy +
      cruisingEnergy +
      brakingEnergy +
      elevationEnergy +
      levitationEnergy +
      auxiliaryEnergy;

    // Per passenger energy
    const perPassenger = total / passengers;

    // Efficiency (kWh per 100 km)
    const efficiency = (total / distance) * 100000;

    // Recovery rate
    const energyRecovered = Math.abs(brakingEnergy);
    const energyUsed = accelerationEnergy + cruisingEnergy + elevationEnergy + levitationEnergy + auxiliaryEnergy;
    const recoveryRate = energyRecovered / energyUsed;

    return {
      acceleration: accelerationEnergy,
      cruising: cruisingEnergy,
      braking: brakingEnergy,
      auxiliary: auxiliaryEnergy,
      levitation: levitationEnergy,
      total,
      perPassenger,
      efficiency,
      recoveryRate,
    };
  }

  /**
   * Validate pod design
   *
   * @param design - Pod design specifications
   * @returns Validation result with errors and recommendations
   */
  validatePodDesign(design: PodDesign): ValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];

    // Calculate tube area
    const tubeArea = Math.PI * Math.pow(DESIGN_CONSTANTS.TUBE_DIAMETER / 2, 2);
    const podArea = Math.PI * Math.pow(design.diameter / 2, 2);
    const tubeAreaRatio = podArea / tubeArea;

    // Check tube area ratio (should be < 0.7 for Kantrowitz limit)
    if (tubeAreaRatio >= 0.7) {
      errors.push(
        `Tube area ratio ${tubeAreaRatio.toFixed(3)} exceeds maximum 0.7 (choking risk)`
      );
    } else if (tubeAreaRatio >= 0.6) {
      warnings.push(
        `Tube area ratio ${tubeAreaRatio.toFixed(3)} is high. Consider smaller diameter.`
      );
    }

    // Calculate Kantrowitz limit
    const kantrowitzLimit =
      PHYSICS_CONSTANTS.SPEED_OF_SOUND * Math.sqrt(1 / tubeAreaRatio - 1);

    if (design.maxSpeed > kantrowitzLimit) {
      errors.push(
        `Max speed ${design.maxSpeed} m/s exceeds Kantrowitz limit ${kantrowitzLimit.toFixed(1)} m/s`
      );
      recommendations.push(
        'Add compressor fan, increase tube diameter, or reduce pod diameter'
      );
    }

    // Check diameter constraint
    if (design.diameter >= DESIGN_CONSTANTS.TUBE_DIAMETER) {
      errors.push('Pod diameter must be less than tube diameter');
    }

    // Check length-to-diameter ratio
    const lengthToDiameter = design.length / design.diameter;
    if (lengthToDiameter < 3) {
      warnings.push(
        `Length-to-diameter ratio ${lengthToDiameter.toFixed(1)} is low. May have poor aerodynamics.`
      );
      recommendations.push('Increase pod length for better aerodynamic efficiency');
    }

    // Calculate power-to-weight ratio
    const totalPower =
      design.powerProfile.levitation +
      design.powerProfile.auxiliary +
      design.powerProfile.hvac +
      design.powerProfile.communications;
    const powerToWeight = (totalPower * 1000) / design.mass; // W/kg

    // Check mass constraints
    const massEfficiency =
      (design.passengerCapacity * 75 + design.cargoCapacity) / design.mass;

    if (massEfficiency < 0.3) {
      warnings.push(
        `Mass efficiency ${(massEfficiency * 100).toFixed(1)}% is low. Pod may be over-engineered.`
      );
      recommendations.push('Consider lightweight materials (carbon fiber, aluminum alloys)');
    }

    // Check acceleration/deceleration limits
    if (design.maxAcceleration > DESIGN_CONSTANTS.MAX_ACCELERATION) {
      warnings.push(
        `Max acceleration ${design.maxAcceleration} m/s² exceeds comfortable limit ${DESIGN_CONSTANTS.MAX_ACCELERATION} m/s²`
      );
      recommendations.push('Reduce max acceleration for passenger comfort');
    }

    if (design.maxDeceleration > DESIGN_CONSTANTS.MAX_DECELERATION) {
      errors.push(
        `Max deceleration ${design.maxDeceleration} m/s² exceeds safety limit ${DESIGN_CONSTANTS.MAX_DECELERATION} m/s²`
      );
    }

    // Check drag coefficient
    if (design.dragCoefficient > 0.25) {
      warnings.push(
        `Drag coefficient ${design.dragCoefficient} is high. Aerodynamic optimization recommended.`
      );
      recommendations.push(
        'Improve aerodynamic profile with streamlined nose and tail sections'
      );
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      metrics: {
        tubeAreaRatio,
        kantrowitzLimit,
        powerToWeight,
        massEfficiency,
      },
      recommendations,
    };
  }

  /**
   * Simulate a complete hyperloop journey
   *
   * @param params - Simulation parameters
   * @returns Complete journey simulation result
   */
  simulateJourney(params: SimulationParams): SimulationResult {
    const { origin, destination, podType, passengers, cargo = 0, departureTime } = params;

    // Create sample route (in production, this would query a database)
    const route: Route = this.createSampleRoute(origin, destination);

    // Create sample pod design (in production, this would query a database)
    const pod: PodDesign = this.createSamplePod(podType);

    // Validate pod design
    const validation = this.validatePodDesign(pod);
    if (!validation.isValid) {
      return {
        id: `SIM-${Date.now()}`,
        route,
        pod,
        duration: 0,
        distance: 0,
        maxSpeed: 0,
        averageSpeed: 0,
        energyConsumption: {
          acceleration: 0,
          cruising: 0,
          braking: 0,
          auxiliary: 0,
          levitation: 0,
          total: 0,
          perPassenger: 0,
          efficiency: 0,
          recoveryRate: 0,
        },
        timeline: [],
        environmental: {
          co2Savings: 0,
          energySavings: 0,
          comparison: [],
        },
        success: false,
        error: validation.errors.join('; '),
      };
    }

    // Calculate journey phases
    const timeline: JourneyPhase[] = [];
    let currentTime = 0;
    let currentDistance = 0;
    let currentSpeed = 0;

    // Phase 1: Boarding (2 minutes)
    timeline.push({
      phase: 'boarding',
      startTime: currentTime,
      endTime: currentTime + 120,
      distance: 0,
      speed: 0,
      energyConsumed: 0,
    });
    currentTime += 120;

    // Phase 2: Airlock Evacuation (4 minutes)
    timeline.push({
      phase: 'airlock-evacuation',
      startTime: currentTime,
      endTime: currentTime + 240,
      distance: 0,
      speed: 0,
      energyConsumed: 5, // kWh for pumps
    });
    currentTime += 240;

    // Phase 3: Acceleration
    const maxSpeed = Math.min(pod.maxSpeed, route.maxSpeed);
    const accelTime = maxSpeed / pod.maxAcceleration;
    const accelDistance = 0.5 * pod.maxAcceleration * accelTime * accelTime;

    timeline.push({
      phase: 'acceleration',
      startTime: currentTime,
      endTime: currentTime + accelTime,
      distance: accelDistance,
      speed: maxSpeed,
      energyConsumed: 0, // Will be calculated in energy analysis
    });
    currentTime += accelTime;
    currentDistance += accelDistance;
    currentSpeed = maxSpeed;

    // Phase 4: Cruising
    const decelTime = maxSpeed / pod.maxDeceleration;
    const decelDistance = 0.5 * pod.maxDeceleration * decelTime * decelTime;
    const cruiseDistance = route.distance - accelDistance - decelDistance;
    const cruiseTime = cruiseDistance / maxSpeed;

    timeline.push({
      phase: 'cruising',
      startTime: currentTime,
      endTime: currentTime + cruiseTime,
      distance: cruiseDistance,
      speed: maxSpeed,
      energyConsumed: 0, // Will be calculated
    });
    currentTime += cruiseTime;
    currentDistance += cruiseDistance;

    // Phase 5: Deceleration
    timeline.push({
      phase: 'deceleration',
      startTime: currentTime,
      endTime: currentTime + decelTime,
      distance: decelDistance,
      speed: 0,
      energyConsumed: 0, // Regenerative (negative)
    });
    currentTime += decelTime;
    currentDistance += decelDistance;
    currentSpeed = 0;

    // Phase 6: Airlock Pressurization (3 minutes)
    timeline.push({
      phase: 'airlock-pressurization',
      startTime: currentTime,
      endTime: currentTime + 180,
      distance: 0,
      speed: 0,
      energyConsumed: 0,
    });
    currentTime += 180;

    // Phase 7: Deboarding (2 minutes)
    timeline.push({
      phase: 'deboarding',
      startTime: currentTime,
      endTime: currentTime + 120,
      distance: 0,
      speed: 0,
      energyConsumed: 0,
    });
    currentTime += 120;

    // Calculate energy consumption
    const energyConsumption = this.calculateEnergyConsumption({
      distance: route.distance,
      podMass: pod.mass,
      maxSpeed: maxSpeed,
      passengers: passengers,
      cargo: cargo,
    });

    // Distribute energy to phases
    timeline[2].energyConsumed = energyConsumption.acceleration;
    timeline[3].energyConsumed = energyConsumption.cruising + energyConsumption.levitation;
    timeline[4].energyConsumed = energyConsumption.braking;

    // Calculate environmental impact
    const environmental = {
      co2Savings: this.calculateCO2Savings(route.distance, passengers, 'airplane'),
      energySavings: this.calculateEnergySavings(
        energyConsumption.total,
        route.distance,
        passengers,
        'car'
      ),
      comparison: [
        {
          mode: 'Airplane',
          timeDifference: 3600 - currentTime, // Assume 1 hour for airplane (+ airport time)
          energyDifference: 900 * passengers - energyConsumption.total,
          co2Difference: (900 * passengers * 0.5) - 0, // Hyperloop is zero emission
        },
        {
          mode: 'High-Speed Rail',
          timeDifference: (route.distance / (350 / 3.6)) - currentTime, // HSR at 350 km/h
          energyDifference: 20 * passengers - energyConsumption.total,
          co2Difference: (20 * passengers * 0.3) - 0,
        },
        {
          mode: 'Car',
          timeDifference: (route.distance / (120 / 3.6)) - currentTime, // Car at 120 km/h
          energyDifference: 200 * passengers - energyConsumption.total,
          co2Difference: (200 * passengers * 0.4) - 0,
        },
      ],
    };

    return {
      id: `SIM-${Date.now()}`,
      route,
      pod,
      duration: currentTime,
      distance: route.distance,
      maxSpeed: maxSpeed,
      averageSpeed: route.distance / (currentTime - 120 - 240 - 180 - 120), // Exclude station time
      energyConsumption,
      timeline,
      environmental,
      success: true,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Create sample route (placeholder)
   */
  private createSampleRoute(origin: string, destination: string): Route {
    // Sample routes (in production, query from database)
    const routes: Record<string, Route> = {
      'LA-SF': {
        id: 'ROUTE-LA-SF',
        name: 'Los Angeles to San Francisco',
        origin: 'LAX-CENTRAL',
        destination: 'SF-DOWNTOWN',
        distance: 600000, // 600 km
        waypoints: [],
        maxSpeed: PHYSICS_CONSTANTS.MAX_SPEED,
        averageGrade: 0.5,
        numberOfCurves: 12,
        estimatedTime: 2100, // 35 minutes
      },
    };

    const routeKey = `${origin}-${destination}`;
    return (
      routes[routeKey] || {
        id: `ROUTE-${origin}-${destination}`,
        name: `${origin} to ${destination}`,
        origin,
        destination,
        distance: 500000,
        waypoints: [],
        maxSpeed: PHYSICS_CONSTANTS.MAX_SPEED,
        averageGrade: 0,
        numberOfCurves: 10,
        estimatedTime: 1800,
      }
    );
  }

  /**
   * Create sample pod design (placeholder)
   */
  private createSamplePod(podType: string): PodDesign {
    return {
      id: `POD-${podType}`,
      name: `Hyperloop ${podType}`,
      length: DESIGN_CONSTANTS.POD_LENGTH,
      diameter: DESIGN_CONSTANTS.POD_DIAMETER,
      mass: 15000,
      dragCoefficient: DESIGN_CONSTANTS.DRAG_COEFFICIENT,
      passengerCapacity: 28,
      cargoCapacity: 2000,
      maxSpeed: PHYSICS_CONSTANTS.MAX_SPEED,
      maxAcceleration: DESIGN_CONSTANTS.MAX_ACCELERATION,
      maxDeceleration: DESIGN_CONSTANTS.MAX_DECELERATION,
      powerProfile: {
        levitation: 75,
        auxiliary: 15,
        hvac: 10,
        communications: 5,
      },
    };
  }

  /**
   * Calculate CO₂ savings compared to other transport modes
   */
  private calculateCO2Savings(
    distance: number,
    passengers: number,
    comparisonMode: string
  ): number {
    // CO₂ emissions in kg per passenger per km
    const emissionFactors: Record<string, number> = {
      airplane: 0.15, // kg CO₂ per passenger-km
      car: 0.12,
      bus: 0.05,
      train: 0.04,
    };

    const factor = emissionFactors[comparisonMode] || 0.15;
    const distanceKm = distance / 1000;

    // Hyperloop with renewable energy has zero direct emissions
    const hyperloopEmissions = 0;
    const comparisonEmissions = factor * distanceKm * passengers;

    return comparisonEmissions - hyperloopEmissions;
  }

  /**
   * Calculate energy savings compared to other transport modes
   */
  private calculateEnergySavings(
    hyperloopEnergy: number,
    distance: number,
    passengers: number,
    comparisonMode: string
  ): number {
    // Energy consumption in kWh per passenger per km
    const energyFactors: Record<string, number> = {
      airplane: 1.5,
      car: 0.3,
      bus: 0.1,
      train: 0.05,
    };

    const factor = energyFactors[comparisonMode] || 0.3;
    const distanceKm = distance / 1000;
    const comparisonEnergy = factor * distanceKm * passengers;

    return comparisonEnergy - hyperloopEnergy;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate drag force (standalone function)
 */
export function calculateDragForce(params: DragCalculation): DragResult {
  const sdk = new HyperloopSDK();
  return sdk.calculateDragForce(params);
}

/**
 * Calculate levitation force (standalone function)
 */
export function calculateLevitationForce(params: LevitationParams): LevitationResult {
  const sdk = new HyperloopSDK();
  return sdk.calculateLevitationForce(params);
}

/**
 * Calculate linear motor thrust (standalone function)
 */
export function calculateLinearMotorThrust(params: LinearMotorParams): ThrustResult {
  const sdk = new HyperloopSDK();
  return sdk.calculateLinearMotorThrust(params);
}

/**
 * Calculate energy consumption (standalone function)
 */
export function calculateEnergyConsumption(params: JourneyParams): EnergyResult {
  const sdk = new HyperloopSDK();
  return sdk.calculateEnergyConsumption(params);
}

/**
 * Validate pod design (standalone function)
 */
export function validatePodDesign(design: PodDesign): ValidationResult {
  const sdk = new HyperloopSDK();
  return sdk.validatePodDesign(design);
}

/**
 * Simulate journey (standalone function)
 */
export function simulateJourney(params: SimulationParams): SimulationResult {
  const sdk = new HyperloopSDK();
  return sdk.simulateJourney(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { HyperloopSDK };
export default HyperloopSDK;
