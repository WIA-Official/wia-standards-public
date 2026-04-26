/**
 * WIA-QUA-008: Plasma Technology SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Plasma Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for plasma technology including:
 * - Plasma parameter calculations
 * - Plasma generation simulation
 * - Fusion reactor modeling
 * - Plasma processing tools
 * - Medical plasma applications
 * - Plasma propulsion calculations
 */

import {
  PlasmaParameters,
  PlasmaProperties,
  PlasmaType,
  RFPlasmaConfig,
  DCPlasmaConfig,
  MicrowavePlasmaConfig,
  PlasmaGenerationResult,
  TokamakConfig,
  StellatorConfig,
  FusionPerformance,
  EtchingParams,
  EtchingResult,
  PECVDParams,
  PECVDResult,
  CAPParameters,
  MedicalPlasmaResult,
  IonThrusterConfig,
  HallThrusterConfig,
  ThrusterPerformance,
  LangmuirProbe,
  LangmuirProbeResult,
  OESData,
  OESResult,
  SimulationResult,
  PLASMA_CONSTANTS,
  GAS_PROPERTIES,
  PlasmaErrorCode,
  PlasmaError,
  ReactiveSpecies,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-QUA-008 Plasma Technology SDK
 */
export class PlasmaSDK {
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
   * Calculate comprehensive plasma parameters
   */
  calculatePlasmaParameters(params: PlasmaParameters): PlasmaProperties {
    const {
      electronDensity,
      ionDensity = electronDensity,
      neutralDensity = 0,
      electronTemperature,
      ionTemperature = electronTemperature,
      gasTemperature = 300,
    } = params;

    // Validate inputs
    if (electronDensity <= 0) {
      throw new PlasmaError(
        PlasmaErrorCode.DENSITY_TOO_LOW,
        'Electron density must be positive'
      );
    }

    if (electronTemperature <= 0) {
      throw new PlasmaError(
        PlasmaErrorCode.TEMPERATURE_OUT_OF_RANGE,
        'Electron temperature must be positive'
      );
    }

    const k = PLASMA_CONSTANTS.BOLTZMANN;
    const e = PLASMA_CONSTANTS.ELEMENTARY_CHARGE;
    const me = PLASMA_CONSTANTS.ELECTRON_MASS;
    const eps0 = PLASMA_CONSTANTS.EPSILON_0;

    // Calculate Debye length: λD = √(ε₀kT / ne²)
    const debyeLength = Math.sqrt(
      (eps0 * k * electronTemperature) / (electronDensity * e * e)
    );

    // Calculate plasma frequency: ωpe = √(ne² / ε₀me)
    const plasmaFrequency = Math.sqrt(
      (electronDensity * e * e) / (eps0 * me)
    );

    // Calculate plasma parameter (particles in Debye sphere)
    const plasmaParameter =
      electronDensity * (4 / 3) * Math.PI * Math.pow(debyeLength, 3);

    // Calculate ionization degree
    const totalDensity = electronDensity + neutralDensity;
    const ionizationDegree =
      totalDensity > 0 ? electronDensity / totalDensity : 0;

    // Classify plasma type
    let plasmaType: PlasmaType;
    const tempRatio = electronTemperature / gasTemperature;
    if (electronTemperature > 100e6) {
      plasmaType = 'fusion';
    } else if (tempRatio > 10) {
      plasmaType = 'non-thermal';
    } else if (tempRatio < 2) {
      plasmaType = 'thermal';
    } else if (electronTemperature < 10000) {
      plasmaType = 'low-temperature';
    } else {
      plasmaType = 'high-temperature';
    }

    // Check quasi-neutrality
    const isQuasiNeutral = Math.abs(electronDensity - ionDensity) / electronDensity < 0.01;

    // Estimate collision frequency (simplified)
    const collisionFrequency = this.calculateCollisionFrequency(
      electronDensity,
      electronTemperature,
      neutralDensity
    );

    return {
      electronDensity,
      ionDensity,
      neutralDensity,
      electronTemperature,
      ionTemperature,
      gasTemperature,
      debyeLength,
      plasmaFrequency,
      plasmaParameter,
      ionizationDegree,
      plasmaType,
      isQuasiNeutral,
      collisionFrequency,
    };
  }

  /**
   * Calculate Debye length
   */
  calculateDebyeLength(electronDensity: number, electronTemperature: number): number {
    const k = PLASMA_CONSTANTS.BOLTZMANN;
    const e = PLASMA_CONSTANTS.ELEMENTARY_CHARGE;
    const eps0 = PLASMA_CONSTANTS.EPSILON_0;

    return Math.sqrt((eps0 * k * electronTemperature) / (electronDensity * e * e));
  }

  /**
   * Calculate plasma frequency
   */
  calculatePlasmaFrequency(electronDensity: number): number {
    const e = PLASMA_CONSTANTS.ELEMENTARY_CHARGE;
    const me = PLASMA_CONSTANTS.ELECTRON_MASS;
    const eps0 = PLASMA_CONSTANTS.EPSILON_0;

    return Math.sqrt((electronDensity * e * e) / (eps0 * me));
  }

  /**
   * Generate RF plasma
   */
  generateRFPlasma(config: RFPlasmaConfig): PlasmaGenerationResult {
    const { frequency, power, pressure, gas, coupling } = config;

    // Calculate absorbed power (simplified model)
    const absorbedPower = power * 0.7; // Assume 70% coupling efficiency

    // Estimate plasma density based on power and pressure
    const electronDensity = this.estimateRFPlasmaDensity(
      absorbedPower,
      pressure,
      frequency,
      coupling
    );

    // Estimate electron temperature
    const electronTemperature = this.estimateElectronTemperature(pressure, power);

    const parameters = this.calculatePlasmaParameters({
      electronDensity,
      electronTemperature,
      neutralDensity: this.pressureToNeutralDensity(pressure, 300),
      gasTemperature: 300,
    });

    return {
      method: coupling === 'capacitive' ? 'rf-ccp' : 'rf-icp',
      parameters,
      absorbedPower,
      uniformity: 0.85,
      efficiency: 0.7,
      stable: true,
      warnings: [],
    };
  }

  /**
   * Generate DC plasma
   */
  generateDCPlasma(config: DCPlasmaConfig): PlasmaGenerationResult {
    const { voltage, current, pressure, gapDistance, dischargeType } = config;

    const absorbedPower = voltage * current;

    // Estimate plasma density
    const electronDensity = (current / PLASMA_CONSTANTS.ELEMENTARY_CHARGE) * 1e16;

    // Estimate electron temperature
    const electronTemperature = this.estimateElectronTemperature(pressure, absorbedPower);

    const parameters = this.calculatePlasmaParameters({
      electronDensity,
      electronTemperature,
      neutralDensity: this.pressureToNeutralDensity(pressure, 300),
    });

    return {
      method: dischargeType === 'glow' ? 'dc-glow' : 'dc-arc',
      parameters,
      absorbedPower,
      uniformity: 0.6,
      efficiency: 0.5,
      stable: voltage < 1000,
      warnings: voltage > 1000 ? ['High voltage - arc risk'] : [],
    };
  }

  /**
   * Simulate tokamak fusion reactor
   */
  simulateTokamak(config: TokamakConfig): FusionPerformance {
    const {
      majorRadius,
      minorRadius,
      plasmaCurrent,
      toroidalField,
      plasmaDensity,
      ionTemperature,
      elongation = 1.7,
      triangularity = 0.33,
    } = config;

    // Convert temperature to keV
    const T_keV = ionTemperature * PLASMA_CONSTANTS.BOLTZMANN / PLASMA_CONSTANTS.EV_TO_JOULES / 1000;

    // Calculate fusion reaction rate <σv>
    const sigmaV = this.calculateDTFusionRate(T_keV);

    // Fusion power density (D-T 50-50 mix)
    const n_D = plasmaDensity / 2;
    const n_T = plasmaDensity / 2;
    const E_fusion = 17.6e6 * PLASMA_CONSTANTS.EV_TO_JOULES; // 17.6 MeV
    const reactionRate = n_D * n_T * sigmaV;
    const volume = 2 * Math.PI * Math.PI * majorRadius * minorRadius * minorRadius * elongation;
    const fusionPower = reactionRate * E_fusion * volume / 4;

    // Energy confinement time (ITER-89P scaling)
    const aspectRatio = majorRadius / minorRadius;
    const heatingPower = 50e6; // Assume 50 MW heating
    const confinementTime =
      0.048 *
      Math.pow(plasmaCurrent / 1e6, 0.85) *
      Math.pow(majorRadius, 1.2) *
      Math.pow(minorRadius, 0.3) *
      Math.pow(elongation, 0.5) *
      Math.pow(toroidalField, 0.2) /
      (Math.pow(heatingPower / 1e6, 0.5) *
        Math.pow(plasmaDensity / 1e19, 0.1) *
        Math.pow(aspectRatio, 0.5));

    // Triple product
    const tripleProduct = plasmaDensity * T_keV * confinementTime;

    // Q factor
    const qFactor = fusionPower / heatingPower;

    // Safety factor
    const safetyFactor =
      (minorRadius * toroidalField) / (majorRadius * PLASMA_CONSTANTS.MU_0 * plasmaCurrent / (2 * Math.PI * minorRadius));

    // Beta
    const k = PLASMA_CONSTANTS.BOLTZMANN;
    const mu0 = PLASMA_CONSTANTS.MU_0;
    const pressure = 2 * plasmaDensity * k * ionTemperature;
    const beta = (2 * mu0 * pressure) / (toroidalField * toroidalField);

    // Check Lawson criterion (n·τ·T > 3×10²¹ keV·s/m³)
    const lawsonSatisfied = tripleProduct > 3e21;

    // Check ignition (Q > 5 typically considered ignition regime)
    const ignition = qFactor > 5;

    return {
      fusionPower,
      reactionRate,
      confinementTime,
      tripleProduct,
      qFactor,
      safetyFactor,
      beta,
      lawsonSatisfied,
      ignition,
    };
  }

  /**
   * Simulate stellarator
   */
  simulateStellator(config: StellatorConfig): FusionPerformance {
    const {
      majorRadius,
      minorRadius,
      plasmaDensity,
      ionTemperature,
      magneticField,
      rotationalTransform,
    } = config;

    const T_keV = ionTemperature * PLASMA_CONSTANTS.BOLTZMANN / PLASMA_CONSTANTS.EV_TO_JOULES / 1000;
    const sigmaV = this.calculateDTFusionRate(T_keV);

    const n_D = plasmaDensity / 2;
    const n_T = plasmaDensity / 2;
    const E_fusion = 17.6e6 * PLASMA_CONSTANTS.EV_TO_JOULES;
    const reactionRate = n_D * n_T * sigmaV;
    const volume = 2 * Math.PI * Math.PI * majorRadius * minorRadius * minorRadius;
    const fusionPower = reactionRate * E_fusion * volume / 4;

    // Stellarator confinement scaling (simplified ISS04)
    const aspectRatio = majorRadius / minorRadius;
    const heatingPower = 30e6;
    const confinementTime =
      0.134 *
      Math.pow(majorRadius, 0.64) *
      Math.pow(minorRadius * 2, 2.21) *
      Math.pow(magneticField, 0.83) *
      Math.pow(plasmaDensity / 1e19, 0.51) /
      Math.pow(heatingPower / 1e6, 0.61);

    const tripleProduct = plasmaDensity * T_keV * confinementTime;
    const qFactor = fusionPower / heatingPower;
    const safetyFactor = 1 / rotationalTransform;

    const k = PLASMA_CONSTANTS.BOLTZMANN;
    const mu0 = PLASMA_CONSTANTS.MU_0;
    const pressure = 2 * plasmaDensity * k * ionTemperature;
    const beta = (2 * mu0 * pressure) / (magneticField * magneticField);

    return {
      fusionPower,
      reactionRate,
      confinementTime,
      tripleProduct,
      qFactor,
      safetyFactor,
      beta,
      lawsonSatisfied: tripleProduct > 3e21,
      ignition: qFactor > 5,
    };
  }

  /**
   * Calculate ion thruster performance
   */
  calculateIonThruster(config: IonThrusterConfig): ThrusterPerformance {
    const { propellant, beamPower, accelerationVoltage, massFlowRate, beamCurrent } = config;

    const gasProps = GAS_PROPERTIES[propellant as keyof typeof GAS_PROPERTIES] || GAS_PROPERTIES.xenon;
    const ionMass = gasProps.mass * PLASMA_CONSTANTS.AMU;
    const e = PLASMA_CONSTANTS.ELEMENTARY_CHARGE;
    const g0 = PLASMA_CONSTANTS.STANDARD_GRAVITY;

    // Exhaust velocity: ve = √(2eV/m)
    const exhaustVelocity = Math.sqrt((2 * e * accelerationVoltage) / ionMass);

    // Thrust: F = ṁ × ve
    const massFlowRateKg = massFlowRate * 1e-6; // mg/s to kg/s
    const thrust = massFlowRateKg * exhaustVelocity;

    // Specific impulse: Isp = ve / g0
    const specificImpulse = exhaustVelocity / g0;

    // Efficiency
    const thrustPower = 0.5 * massFlowRateKg * exhaustVelocity * exhaustVelocity;
    const totalPower = beamPower + 100; // Add 100W for neutralizer, etc.
    const efficiency = thrustPower / totalPower;

    // Power-to-thrust ratio
    const powerToThrustRatio = totalPower / (thrust * 1000); // W/mN

    // Propellant consumption
    const propellantConsumption = massFlowRateKg * 86400; // kg/day

    return {
      thrust,
      specificImpulse,
      efficiency,
      exhaustVelocity,
      powerToThrustRatio,
      propellantConsumption,
    };
  }

  /**
   * Calculate Hall thruster performance
   */
  calculateHallThruster(config: HallThrusterConfig): ThrusterPerformance {
    const { propellant, dischargePower, dischargeVoltage, massFlowRate } = config;

    const gasProps = GAS_PROPERTIES[propellant as keyof typeof GAS_PROPERTIES] || GAS_PROPERTIES.xenon;
    const ionMass = gasProps.mass * PLASMA_CONSTANTS.AMU;
    const e = PLASMA_CONSTANTS.ELEMENTARY_CHARGE;
    const g0 = PLASMA_CONSTANTS.STANDARD_GRAVITY;

    // Estimate exhaust velocity (Hall thrusters have lower voltage than ion)
    const effectiveVoltage = dischargeVoltage * 0.8; // Account for voltage utilization
    const exhaustVelocity = Math.sqrt((2 * e * effectiveVoltage) / ionMass);

    const massFlowRateKg = massFlowRate * 1e-6;
    const thrust = massFlowRateKg * exhaustVelocity;
    const specificImpulse = exhaustVelocity / g0;

    const thrustPower = 0.5 * massFlowRateKg * exhaustVelocity * exhaustVelocity;
    const efficiency = thrustPower / dischargePower;

    const powerToThrustRatio = dischargePower / (thrust * 1000);
    const propellantConsumption = massFlowRateKg * 86400;

    return {
      thrust,
      specificImpulse,
      efficiency,
      exhaustVelocity,
      powerToThrustRatio,
      propellantConsumption,
    };
  }

  /**
   * Analyze Langmuir probe data
   */
  analyzeLangmuirProbe(probe: LangmuirProbe): LangmuirProbeResult {
    const { voltageData, currentData, probeArea } = probe;

    // Find floating potential (I ≈ 0)
    let floatingIndex = 0;
    let minCurrent = Math.abs(currentData[0]);
    for (let i = 1; i < currentData.length; i++) {
      if (Math.abs(currentData[i]) < minCurrent) {
        minCurrent = Math.abs(currentData[i]);
        floatingIndex = i;
      }
    }
    const floatingPotential = voltageData[floatingIndex];

    // Find ion saturation current (most negative voltage)
    const ionSaturationCurrent = Math.abs(currentData[0]);

    // Estimate electron temperature from exponential region slope
    // Te = e / [k × d(ln I)/dV]
    let electronTemperatureEV = 2.0; // Default 2 eV
    for (let i = floatingIndex + 5; i < currentData.length - 5 && i < floatingIndex + 20; i++) {
      if (currentData[i] > 0 && currentData[i + 1] > 0) {
        const dV = voltageData[i + 1] - voltageData[i];
        const dLnI = Math.log(currentData[i + 1]) - Math.log(currentData[i]);
        if (dLnI > 0 && dV > 0) {
          electronTemperatureEV = 1 / (dLnI / dV);
          break;
        }
      }
    }

    const electronTemperatureK = electronTemperatureEV * PLASMA_CONSTANTS.EV_TO_KELVIN;

    // Estimate plasma potential (where dI/dV is maximum)
    const plasmaPotential = floatingPotential + 2.5 * electronTemperatureEV;

    // Electron saturation current
    const electronSaturationCurrent = currentData[currentData.length - 1];

    // Calculate electron density from ion saturation current
    // ne = Ii,sat / (0.61 × e × A × √(kTe/mi))
    const mi = 131.29 * PLASMA_CONSTANTS.AMU; // Assume xenon
    const k = PLASMA_CONSTANTS.BOLTZMANN;
    const e = PLASMA_CONSTANTS.ELEMENTARY_CHARGE;
    const Te = electronTemperatureK;

    const electronDensity =
      ionSaturationCurrent / (0.61 * e * probeArea * Math.sqrt((k * Te) / mi));

    return {
      electronDensity,
      electronTemperatureEV,
      electronTemperatureK,
      plasmaPotential,
      floatingPotential,
      electronSaturationCurrent,
      ionSaturationCurrent,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Estimate collision frequency
   */
  private calculateCollisionFrequency(
    electronDensity: number,
    electronTemperature: number,
    neutralDensity: number
  ): number {
    // Simplified collision frequency
    const sigma = 1e-19; // Collision cross-section (m²)
    const k = PLASMA_CONSTANTS.BOLTZMANN;
    const me = PLASMA_CONSTANTS.ELECTRON_MASS;
    const v_thermal = Math.sqrt((8 * k * electronTemperature) / (Math.PI * me));

    return neutralDensity * sigma * v_thermal;
  }

  /**
   * Estimate RF plasma density
   */
  private estimateRFPlasmaDensity(
    power: number,
    pressure: number,
    frequency: number,
    coupling: 'capacitive' | 'inductive'
  ): number {
    // Simplified model: ne ∝ √P
    const baseDensity = coupling === 'inductive' ? 1e17 : 1e16;
    return baseDensity * Math.sqrt(power / 1000) * Math.sqrt(pressure / 10);
  }

  /**
   * Estimate electron temperature
   */
  private estimateElectronTemperature(pressure: number, power: number): number {
    // Simplified: Te decreases with pressure, increases with power
    return 30000 * Math.sqrt(power / 1000) / Math.sqrt(pressure / 10);
  }

  /**
   * Convert pressure to neutral density
   */
  private pressureToNeutralDensity(pressure: number, temperature: number): number {
    // Ideal gas law: n = P / (kT)
    const k = PLASMA_CONSTANTS.BOLTZMANN;
    return pressure / (k * temperature);
  }

  /**
   * Calculate D-T fusion reaction rate
   */
  private calculateDTFusionRate(T_keV: number): number {
    // Parameterized <σv> for D-T reaction (Bosch-Hale formula)
    // Simplified approximation valid for 10-100 keV
    if (T_keV < 1) return 0;
    if (T_keV < 10) return 1e-28 * Math.pow(T_keV, 2);
    if (T_keV > 100) return 1e-22;

    // Approximate fit
    const C1 = 1.17302e-24;
    const C2 = 1.51361e-2;
    const C3 = 7.51886e-2;
    const C4 = 4.60643e-3;
    const C5 = 1.35000e-2;
    const C6 = -1.06750e-4;
    const C7 = 1.36600e-5;

    const theta = T_keV / (1 - T_keV * (C2 + T_keV * (C4 + T_keV * C6)) / (1 + T_keV * (C3 + T_keV * (C5 + T_keV * C7))));

    return C1 * Math.pow(theta, -2 / 3) * Math.exp(-3 * Math.pow(theta, 1 / 3));
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate Debye length (standalone)
 */
export function calculateDebyeLength(electronDensity: number, electronTemperature: number): number {
  const sdk = new PlasmaSDK();
  return sdk.calculateDebyeLength(electronDensity, electronTemperature);
}

/**
 * Calculate plasma frequency (standalone)
 */
export function calculatePlasmaFrequency(electronDensity: number): number {
  const sdk = new PlasmaSDK();
  return sdk.calculatePlasmaFrequency(electronDensity);
}

/**
 * Calculate plasma parameters (standalone)
 */
export function calculatePlasmaParameters(params: PlasmaParameters): PlasmaProperties {
  const sdk = new PlasmaSDK();
  return sdk.calculatePlasmaParameters(params);
}

/**
 * Generate RF plasma (standalone)
 */
export function generateRFPlasma(config: RFPlasmaConfig): PlasmaGenerationResult {
  const sdk = new PlasmaSDK();
  return sdk.generateRFPlasma(config);
}

/**
 * Simulate tokamak (standalone)
 */
export function simulateTokamak(config: TokamakConfig): FusionPerformance {
  const sdk = new PlasmaSDK();
  return sdk.simulateTokamak(config);
}

/**
 * Calculate fusion power (standalone)
 */
export function calculateFusionPower(
  plasmaDensity: number,
  ionTemperature: number,
  volume: number
): number {
  const sdk = new PlasmaSDK();
  const T_keV = ionTemperature * PLASMA_CONSTANTS.BOLTZMANN / PLASMA_CONSTANTS.EV_TO_JOULES / 1000;
  const sigmaV = (sdk as any).calculateDTFusionRate(T_keV);
  const n_D = plasmaDensity / 2;
  const n_T = plasmaDensity / 2;
  const E_fusion = 17.6e6 * PLASMA_CONSTANTS.EV_TO_JOULES;
  const reactionRate = n_D * n_T * sigmaV;
  return reactionRate * E_fusion * volume / 4;
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { PlasmaSDK };
export default PlasmaSDK;

**弘익人間 (홍익인간) · Benefit All Humanity**
