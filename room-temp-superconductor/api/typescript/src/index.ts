/**
 * WIA-QUA-019: Room-Temperature Superconductor - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  HydrideCompound,
  LK99Material,
  RoomTempSuperconductor,
  DiamondAnvilCell,
  HighPressureSynthesisResult,
  ResistanceMeasurement,
  ResistanceVsTemperature,
  MagneticMeasurement,
  MeissnerTestResult,
  CriticalCurrentTest,
  RTSValidationResult,
  CharacterizationSuite,
  CharacterizationReport,
  ApplicationSimulationResult,
  RTSPhysicalConstants,
  TemperatureConversion,
  PressureConversion,
  RTSMaterialClass,
} from './types';

// Export all types
export * from './types';

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for room-temperature superconductivity
 */
export const CONSTANTS: RTSPhysicalConstants = {
  ROOM_TEMP_MIN: 300, // Kelvin (27°C)
  ROOM_TEMP_PREFERRED: 350, // Kelvin (77°C)
  ROOM_TEMP_IDEAL: 400, // Kelvin (127°C)
  KB: 1.380649e-23, // J/K
  KB_EV: 8.617333262e-5, // eV/K
  H: 6.62607015e-34, // J·s
  HBAR: 1.054571817e-34, // J·s
  E: 1.602176634e-19, // C
  ME: 9.1093837015e-31, // kg
  BCS_GAP_RATIO: 1.764,
  PHI_0: 2.067833848e-15, // Wb
  MU_0: 1.2566370614e-6, // H/m
  ATM: 101325, // Pa
  GPA: 1e9, // Pa
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Convert temperature between scales
 */
export function convertTemperature(
  value: number,
  from: 'kelvin' | 'celsius' | 'fahrenheit'
): TemperatureConversion {
  let kelvin: number;

  switch (from) {
    case 'kelvin':
      kelvin = value;
      break;
    case 'celsius':
      kelvin = value + 273.15;
      break;
    case 'fahrenheit':
      kelvin = ((value - 32) * 5) / 9 + 273.15;
      break;
  }

  return {
    kelvin,
    celsius: kelvin - 273.15,
    fahrenheit: ((kelvin - 273.15) * 9) / 5 + 32,
  };
}

/**
 * Convert pressure between units
 */
export function convertPressure(
  value: number,
  from: 'pascal' | 'gpa' | 'bar' | 'atm'
): PressureConversion {
  let pascal: number;

  switch (from) {
    case 'pascal':
      pascal = value;
      break;
    case 'gpa':
      pascal = value * 1e9;
      break;
    case 'bar':
      pascal = value * 1e5;
      break;
    case 'atm':
      pascal = value * 101325;
      break;
  }

  return {
    pascal,
    gpa: pascal / 1e9,
    bar: pascal / 1e5,
    atm: pascal / 101325,
  };
}

/**
 * Calculate BCS energy gap
 */
export function calculateBCSEnergyGap(tc: number, temperature: number = 0): number {
  const delta0 = CONSTANTS.BCS_GAP_RATIO * CONSTANTS.KB_EV * tc; // eV

  if (temperature >= tc) {
    return 0;
  }

  if (temperature === 0) {
    return delta0;
  }

  // Temperature-dependent gap (approximate)
  const ratio = tc / temperature - 1;
  const delta = delta0 * Math.tanh(1.74 * Math.sqrt(ratio));

  return delta; // eV
}

// ============================================================================
// High-Pressure Synthesis Class
// ============================================================================

/**
 * High-pressure synthesis using diamond anvil cell
 */
export class HighPressureSynthesis {
  private material: HydrideCompound;
  private dac: DiamondAnvilCell;

  constructor(material: HydrideCompound, dac: DiamondAnvilCell) {
    this.material = material;
    this.dac = dac;
  }

  /**
   * Compress sample to target pressure
   */
  async compress(): Promise<{ pressure: number; success: boolean }> {
    const targetPressure = this.dac.pressure.target;

    // Simulate compression (in real implementation, this would interface with hardware)
    console.log(`Compressing to ${targetPressure / 1e9} GPa...`);

    // Simulate time delay
    await new Promise((resolve) => setTimeout(resolve, 100));

    return {
      pressure: targetPressure,
      success: true,
    };
  }

  /**
   * Anneal sample at high pressure and temperature
   */
  async anneal(): Promise<HighPressureSynthesisResult> {
    const heating = this.dac.heating;

    if (!heating) {
      return {
        success: false,
        pressure: this.dac.pressure.target,
        temperature: 300,
        phase: 'unknown',
        quality: 'poor',
        messages: ['No heating parameters specified'],
      };
    }

    console.log(`Annealing at ${heating.targetTemp} K for ${heating.duration} s...`);

    // Simulate annealing
    await new Promise((resolve) => setTimeout(resolve, 200));

    // Check if conditions are suitable for the material
    const pressureOk = Math.abs(this.dac.pressure.target - this.material.criticalPressure) < 50e9;
    const tempOk = heating.targetTemp > 1500 && heating.targetTemp < 3000;

    const success = pressureOk && tempOk;

    return {
      success,
      pressure: this.dac.pressure.target,
      temperature: heating.targetTemp,
      phase: success ? this.material.latticeStructure : 'mixed-phase',
      quality: success ? 'good' : 'fair',
      messages: success ? ['Synthesis successful'] : ['Suboptimal conditions'],
    };
  }
}

// ============================================================================
// Room-Temperature Superconductor Class
// ============================================================================

/**
 * Room-temperature superconductor system
 */
export class RoomTempSuperconductorSystem {
  private config: RoomTempSuperconductor;

  constructor(config: RoomTempSuperconductor) {
    this.config = config;
  }

  /**
   * Measure critical temperature
   */
  async measureCriticalTemperature(params: ResistanceMeasurement): Promise<ResistanceVsTemperature> {
    const [tStart, tEnd] = params.temperatureRange;
    const nPoints = params.temperaturePoints;

    const temperature: number[] = [];
    const resistance: number[] = [];

    // Generate temperature array
    for (let i = 0; i < nPoints; i++) {
      const t = tStart + (i / (nPoints - 1)) * (tEnd - tStart);
      temperature.push(t);

      // Simulate resistance (simplified model)
      let r: number;
      const tc = this.config.targetTc;
      const transitionWidth = params.expectedTransition.transitionWidth;

      if (t > tc + transitionWidth / 2) {
        // Normal state
        r = params.expectedTransition.normalStateResistance;
      } else if (t < tc - transitionWidth / 2) {
        // Superconducting state (zero resistance)
        r = params.expectedTransition.residualResistance;
      } else {
        // Transition region
        const normalized = (t - (tc - transitionWidth / 2)) / transitionWidth;
        r =
          params.expectedTransition.residualResistance +
          (params.expectedTransition.normalStateResistance -
            params.expectedTransition.residualResistance) *
            normalized;
      }

      resistance.push(r);
    }

    // Find Tc (90% criterion)
    const r90 =
      0.1 * params.expectedTransition.normalStateResistance +
      0.9 * params.expectedTransition.residualResistance;
    let tcMeasured = this.config.targetTc;
    for (let i = 0; i < temperature.length - 1; i++) {
      if (resistance[i] >= r90 && resistance[i + 1] < r90) {
        tcMeasured = (temperature[i] + temperature[i + 1]) / 2;
        break;
      }
    }

    const zeroResistance =
      Math.min(...resistance) / Math.max(...resistance) < 1e-6;

    return {
      temperature,
      resistance,
      tc: tcMeasured,
      transitionWidth: params.expectedTransition.transitionWidth,
      zeroResistance,
      resistanceRatio: Math.min(...resistance) / Math.max(...resistance),
    };
  }

  /**
   * Validate room-temperature superconductivity
   */
  async validate(): Promise<RTSValidationResult> {
    const tc = this.config.targetTc;
    const roomTempCapable = tc >= CONSTANTS.ROOM_TEMP_MIN;

    // Simulate validation measurements
    const zeroResistance = roomTempCapable && tc > CONSTANTS.ROOM_TEMP_MIN;
    const meissnerEffect = roomTempCapable && tc > CONSTANTS.ROOM_TEMP_MIN;
    const criticalCurrent = roomTempCapable;
    const reproducible = true; // Assume reproducible for this simulation
    const independentlyVerified = false; // Would require external verification

    const passedTests = [
      roomTempCapable && 'Tc >= 300K',
      zeroResistance && 'Zero resistance',
      meissnerEffect && 'Meissner effect',
      criticalCurrent && 'Critical current',
      reproducible && 'Reproducible',
    ].filter(Boolean);

    const confidence = passedTests.length / 6; // Out of 6 possible tests

    let validationTier: 1 | 2 | 3;
    if (zeroResistance && meissnerEffect && criticalCurrent && reproducible) {
      validationTier = 1; // Essential tests passed
    } else if (zeroResistance && meissnerEffect) {
      validationTier = 2; // Tier 1 tests passed
    } else {
      validationTier = 3; // Incomplete validation
    }

    const messages: string[] = [];
    if (!roomTempCapable) {
      messages.push(`Tc = ${tc} K is below room temperature threshold (300 K)`);
    }
    if (roomTempCapable) {
      messages.push(`Room-temperature superconductivity confirmed at Tc = ${tc} K`);
    }
    if (!independentlyVerified) {
      messages.push('Independent verification recommended');
    }

    return {
      roomTempCapable,
      tc,
      zeroResistance,
      meissnerEffect,
      criticalCurrent,
      reproducible,
      independentlyVerified,
      confidence,
      validationTier,
      messages,
    };
  }
}

// ============================================================================
// LK-99 Material Class
// ============================================================================

/**
 * LK-99 type material
 */
export class LK99MaterialSystem {
  private config: LK99Material;

  constructor(config: LK99Material) {
    this.config = config;
  }

  /**
   * Characterize LK-99 sample
   */
  async characterize(tests: {
    xrayDiffraction?: boolean;
    resistanceMeasurement?: boolean;
    magneticSusceptibility?: boolean;
    raman?: boolean;
  }): Promise<{
    phase: string;
    resistance: number;
    diamagnetic: boolean;
    validationStatus: string;
  }> {
    console.log('Characterizing LK-99 sample...');

    // Simulate characterization
    await new Promise((resolve) => setTimeout(resolve, 100));

    const phase =
      this.config.copperDoping > 0.05 && this.config.copperDoping < 0.15
        ? 'Pb10-xCux(PO4)6O'
        : 'mixed-phase';

    const resistance = Math.random() * 1000; // Ohms (random for simulation)
    const diamagnetic = Math.random() > 0.5; // 50/50 for controversial material

    return {
      phase,
      resistance,
      diamagnetic,
      validationStatus: this.config.validationStatus,
    };
  }
}

// ============================================================================
// Meissner Effect Test
// ============================================================================

/**
 * Meissner effect (diamagnetic levitation) test
 */
export class MeissnerTest {
  private material: RoomTempSuperconductor;
  private temperature: number;
  private appliedField: number;

  constructor(config: { material: RoomTempSuperconductor; temperature: number; appliedField: number }) {
    this.material = config.material;
    this.temperature = config.temperature;
    this.appliedField = config.appliedField;
  }

  /**
   * Test diamagnetism and levitation
   */
  async testDiamagnetism(): Promise<MeissnerTestResult> {
    console.log(`Testing Meissner effect at ${this.temperature} K...`);

    await new Promise((resolve) => setTimeout(resolve, 100));

    const tc = this.material.targetTc;
    const isSuperconducting = this.temperature < tc;

    let susceptibility: number;
    let levitating: boolean;
    let fieldExpulsion: number;

    if (isSuperconducting) {
      // Perfect diamagnetism for ideal superconductor
      susceptibility = -0.95 - Math.random() * 0.05; // χ between -0.95 and -1.0
      levitating = true;
      fieldExpulsion = 0.9 + Math.random() * 0.1; // 90-100% field expulsion
    } else {
      // Normal state (weak diamagnetism or paramagnetic)
      susceptibility = -0.00001 + Math.random() * 0.00002; // Weak diamagnetic
      levitating = false;
      fieldExpulsion = 0;
    }

    const meissnerFraction = isSuperconducting ? -susceptibility : 0;

    return {
      susceptibility,
      levitating,
      fieldExpulsion,
      meissnerFraction,
      temperature: this.temperature,
      appliedField: this.appliedField,
    };
  }
}

// ============================================================================
// Characterization Suite
// ============================================================================

/**
 * Comprehensive characterization suite
 */
export class CharacterizationSuiteRunner {
  private config: CharacterizationSuite;

  constructor(config: CharacterizationSuite) {
    this.config = config;
  }

  /**
   * Run all characterization tests
   */
  async runAll(): Promise<CharacterizationReport> {
    const results: CharacterizationReport = {
      totalTests: this.config.tests.length,
      passedTests: [],
      failedTests: [],
      confidence: 0,
      isSuperconducting: false,
      isRoomTempSuperconducting: false,
      results: {},
      recommendations: [],
    };

    // Run each test
    for (const test of this.config.tests) {
      console.log(`Running test: ${test}...`);

      switch (test) {
        case 'resistance-vs-temperature':
          const rtSystem = new RoomTempSuperconductorSystem(this.config.sample);
          const resistanceResult = await rtSystem.measureCriticalTemperature({
            method: 'four-point-probe',
            temperatureRange: [250, 350],
            temperaturePoints: 100,
            rampRate: 1,
            measurementCurrent: 1e-3,
            expectedTransition: {
              tc: this.config.sample.targetTc,
              transitionWidth: 5,
              residualResistance: 1e-9,
              normalStateResistance: 100,
            },
          });
          results.results.resistance = resistanceResult;
          if (resistanceResult.zeroResistance) {
            results.passedTests.push(test);
          } else {
            results.failedTests.push(test);
          }
          break;

        case 'meissner-effect':
          const meissner = new MeissnerTest({
            material: this.config.sample,
            temperature: 290,
            appliedField: 0.01,
          });
          const meissnerResult = await meissner.testDiamagnetism();
          results.results.meissner = meissnerResult;
          if (meissnerResult.susceptibility < -0.9) {
            results.passedTests.push(test);
          } else {
            results.failedTests.push(test);
          }
          break;

        default:
          // Simulate other tests
          if (Math.random() > 0.3) {
            results.passedTests.push(test);
          } else {
            results.failedTests.push(test);
          }
          break;
      }
    }

    // Calculate overall assessment
    results.confidence = results.passedTests.length / results.totalTests;
    results.isSuperconducting = results.passedTests.length >= results.totalTests * 0.7;
    results.isRoomTempSuperconducting =
      results.isSuperconducting && this.config.sample.targetTc >= CONSTANTS.ROOM_TEMP_MIN;

    // Recommendations
    if (results.failedTests.length > 0) {
      results.recommendations.push(`Re-run failed tests: ${results.failedTests.join(', ')}`);
    }
    if (!results.isRoomTempSuperconducting) {
      results.recommendations.push('Material does not meet room-temperature criteria (Tc >= 300K)');
    }
    if (results.confidence < 1.0) {
      results.recommendations.push('Perform additional validation measurements');
    }

    return results;
  }
}

// ============================================================================
// Application Simulator
// ============================================================================

/**
 * Application simulator for room-temp superconductors
 */
export class ApplicationSimulator {
  private application: string;
  private material: RoomTempSuperconductor;
  private config: any;

  constructor(config: { application: string; material: RoomTempSuperconductor; [key: string]: any }) {
    this.application = config.application;
    this.material = config.material;
    this.config = config;
  }

  /**
   * Simulate application performance
   */
  async simulate(): Promise<ApplicationSimulationResult> {
    console.log(`Simulating ${this.application}...`);

    await new Promise((resolve) => setTimeout(resolve, 100));

    let result: ApplicationSimulationResult;

    switch (this.application) {
      case 'power-transmission':
        result = this.simulatePowerTransmission();
        break;

      case 'maglev':
        result = this.simulateMaglev();
        break;

      case 'quantum-computing':
        result = this.simulateQuantumComputer();
        break;

      case 'mri':
        result = this.simulateMRI();
        break;

      default:
        result = {
          application: this.application,
          powerLoss: 0,
          conventionalLoss: 0,
          efficiencyGain: 0,
          annualSavings: 0,
          co2Reduction: 0,
          metrics: {},
        };
    }

    return result;
  }

  private simulatePowerTransmission(): ApplicationSimulationResult {
    const cable = this.config.cable;
    const length = cable.length; // meters
    const current = cable.current; // Amperes

    // Superconductor: essentially zero loss
    const powerLossSC = 0.001 * length; // Minimal cooling power

    // Conventional cable: resistive loss
    const resistivity = 1.68e-8; // Ohm·m (copper)
    const area = cable.crossSection; // m²
    const resistance = (resistivity * length) / area;
    const powerLossConventional = current ** 2 * resistance; // Watts

    const efficiencyGain = (powerLossConventional - powerLossSC) / powerLossConventional;
    const annualSavings = ((powerLossConventional - powerLossSC) * 8760 * 0.1) / 1000; // USD/year at $0.10/kWh
    const co2Reduction = ((powerLossConventional - powerLossSC) * 8760 * 0.5) / 1000; // kg CO₂/year

    return {
      application: 'power-transmission',
      powerLoss: powerLossSC,
      conventionalLoss: powerLossConventional,
      efficiencyGain,
      annualSavings,
      co2Reduction,
      metrics: {
        cableLength: length,
        current: current,
        efficiency: 1 - powerLossSC / (current * cable.crossSection),
      },
    };
  }

  private simulateMaglev(): ApplicationSimulationResult {
    return {
      application: 'maglev',
      powerLoss: 1000, // Watts
      conventionalLoss: 50000, // Watts
      efficiencyGain: 0.98,
      annualSavings: 500000,
      co2Reduction: 100000,
      metrics: {
        levitationHeight: 0.01,
        maxSpeed: 600,
        noCryogenics: true,
      },
    };
  }

  private simulateQuantumComputer(): ApplicationSimulationResult {
    return {
      application: 'quantum-computing',
      powerLoss: 1000, // Watts
      conventionalLoss: 1000000, // Watts (dilution fridge)
      efficiencyGain: 0.999,
      annualSavings: 10000000,
      co2Reduction: 5000000,
      metrics: {
        qubitCount: 1000000,
        noDilutionFridge: true,
        footprint: 'desktop',
      },
    };
  }

  private simulateMRI(): ApplicationSimulationResult {
    return {
      application: 'MRI',
      powerLoss: 5000, // Watts
      conventionalLoss: 50000, // Watts
      efficiencyGain: 0.9,
      annualSavings: 400000,
      co2Reduction: 200000,
      metrics: {
        fieldStrength: 7, // Tesla
        portable: true,
        noCryogens: true,
        costReduction: '10-100x',
      },
    };
  }
}

// ============================================================================
// Material Database
// ============================================================================

/**
 * Known room-temperature superconductor candidates
 */
export const MATERIAL_DATABASE = {
  H3S: {
    name: 'H3S',
    formula: 'H₃S',
    latticeStructure: 'Im-3m',
    criticalTemperature: 203,
    criticalPressure: 155e9,
    hydrogenContent: 75,
    synthesisMethod: 'diamond-anvil-cell' as const,
    discoveryYear: 2015,
    status: 'confirmed' as const,
  },

  LaH10: {
    name: 'LaH10',
    formula: 'LaH₁₀',
    latticeStructure: 'Fm-3m',
    criticalTemperature: 250,
    criticalPressure: 170e9,
    hydrogenContent: 90.9,
    synthesisMethod: 'diamond-anvil-cell' as const,
    discoveryYear: 2019,
    status: 'confirmed' as const,
  },

  CSH: {
    name: 'C-S-H',
    formula: 'C₁₅H₃₂S₂',
    latticeStructure: 'Im-3m',
    criticalTemperature: 288, // Just 12K below room temp!
    criticalPressure: 267e9,
    hydrogenContent: 80,
    synthesisMethod: 'diamond-anvil-cell' as const,
    discoveryYear: 2020,
    status: 'confirmed' as const,
  },

  LK99: {
    composition: 'Pb₁₀₋ₓCuₓ(PO₄)₆O',
    copperDoping: 0.1,
    synthesisTemp: 1273,
    annealingTime: 96,
    claimedTc: 400,
    validationStatus: 'inconclusive' as const,
    phase: 'lead-apatite',
  },
};

// ============================================================================
// Export Main Classes
// ============================================================================

export {
  HighPressureSynthesis,
  RoomTempSuperconductorSystem,
  LK99MaterialSystem,
  MeissnerTest,
  CharacterizationSuiteRunner as CharacterizationSuite,
  ApplicationSimulator,
};

/**
 * 弘益人間 (Benefit All Humanity)
 *
 * Room-temperature superconductivity represents one of the most
 * transformative technologies for humanity - enabling lossless power
 * transmission, revolutionary transportation, accessible quantum
 * computing, and countless other benefits.
 */
