/**
 * WIA-SEMI-020: Semiconductor Packaging Standard - SDK Implementation
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

export * from './types';

import {
  PackageType,
  PackageDesign,
  ThermalProperties,
  ElectricalProperties,
  TSVSpecification,
  MicrobumpSpecification,
  HBMSpecification,
  UCIeConfiguration,
  AccelerationFactors,
  PHYSICAL_CONSTANTS,
} from './types';

// ============================================================================
// Thermal Analysis Functions
// ============================================================================

/**
 * Calculate junction-to-case thermal resistance
 */
export function calculateThermalResistance(
  power: number, // W
  junctionTemp: number, // °C
  caseTemp: number, // °C
): number {
  if (power <= 0) throw new Error('Power must be positive');
  const deltaT = junctionTemp - caseTemp;
  return deltaT / power; // °C/W
}

/**
 * Calculate junction temperature
 */
export function calculateJunctionTemperature(
  power: number, // W
  thetaJC: number, // °C/W
  ambientTemp: number, // °C
): number {
  return ambientTemp + power * thetaJC;
}

/**
 * Estimate thermal time constant
 */
export function estimateThermalTimeConstant(
  mass: number, // kg
  specificHeat: number, // J/kg·K
  thermalResistance: number, // °C/W
): number {
  // τ = R × C where C = mass × specific heat
  const thermalCapacitance = mass * specificHeat;
  return thermalResistance * thermalCapacitance; // seconds
}

/**
 * Calculate required heatsink thermal resistance
 */
export function calculateHeatsinkRequirement(
  power: number, // W
  maxJunctionTemp: number, // °C
  ambientTemp: number, // °C
  thetaJC: number, // °C/W
  timResistance: number, // °C/W
): number {
  const totalAllowedResistance = (maxJunctionTemp - ambientTemp) / power;
  const heatsinkResistance = totalAllowedResistance - thetaJC - timResistance;

  if (heatsinkResistance <= 0) {
    throw new Error('Required heatsink thermal resistance is negative - insufficient cooling possible');
  }

  return heatsinkResistance; // °C/W
}

// ============================================================================
// Electrical Analysis Functions
// ============================================================================

/**
 * Calculate interconnect resistance
 */
export function calculateInterconnectResistance(
  resistivity: number, // Ω·m
  length: number, // m
  crossSectionalArea: number, // m²
): number {
  return (resistivity * length) / crossSectionalArea; // Ω
}

/**
 * Calculate skin depth for AC signals
 */
export function calculateSkinDepth(
  frequency: number, // Hz
  resistivity: number, // Ω·m
  relativePermeability: number = 1,
): number {
  const mu0 = 4 * Math.PI * 1e-7; // H/m
  const mu = relativePermeability * mu0;
  return Math.sqrt(resistivity / (Math.PI * frequency * mu)); // m
}

/**
 * Calculate target PDN impedance
 */
export function calculateTargetImpedance(
  voltageRipple: number, // V (allowable ripple)
  transientCurrent: number, // A
): number {
  return voltageRipple / transientCurrent; // Ω
}

/**
 * Calculate UCIe bandwidth
 */
export function calculateUCIeBandwidth(
  laneCount: number,
  dataRatePerLane: number, // Gbps
): number {
  // Bandwidth in GB/s = (lanes × Gbps/lane) / 8
  return (laneCount * dataRatePerLane) / 8; // GB/s
}

/**
 * Calculate HBM bandwidth
 */
export function calculateHBMBandwidth(
  interfaceWidth: number, // bits
  dataRate: number, // Gbps
): number {
  // Bandwidth = (width × data rate) / 8
  return (interfaceWidth * dataRate) / 8; // GB/s
}

// ============================================================================
// TSV Analysis Functions
// ============================================================================

/**
 * Calculate TSV aspect ratio
 */
export function calculateTSVAspectRatio(
  depth: number, // μm
  diameter: number, // μm
): number {
  return depth / diameter;
}

/**
 * Calculate TSV resistance
 */
export function calculateTSVResistance(
  diameter: number, // μm
  depth: number, // μm
  copperResistivity: number = 1.7e-8, // Ω·m
): number {
  const diameterM = diameter * 1e-6;
  const depthM = depth * 1e-6;
  const area = Math.PI * (diameterM / 2) ** 2;
  const resistance = (copperResistivity * depthM) / area;
  return resistance * 1000; // mΩ
}

/**
 * Calculate TSV capacitance
 */
export function calculateTSVCapacitance(
  diameter: number, // μm
  depth: number, // μm
  oxideThickness: number, // μm
  epsilonR: number = 3.9, // SiO2
): number {
  const epsilon0 = 8.854e-12; // F/m
  const diameterM = diameter * 1e-6;
  const depthM = depth * 1e-6;
  const tOxM = oxideThickness * 1e-6;

  // Cylindrical capacitor approximation
  const outerRadius = (diameterM / 2) + tOxM;
  const innerRadius = diameterM / 2;

  const capacitance = (2 * Math.PI * epsilon0 * epsilonR * depthM) /
                       Math.log(outerRadius / innerRadius);

  return capacitance * 1e15; // fF
}

// ============================================================================
// Reliability Functions
// ============================================================================

/**
 * Calculate Arrhenius acceleration factor
 */
export function calculateArrheniusAF(
  activationEnergy: number, // eV
  useTemperature: number, // °C
  testTemperature: number, // °C
): number {
  const k = PHYSICAL_CONSTANTS.BOLTZMANN_CONSTANT; // eV/K
  const Tuse = useTemperature + 273.15; // K
  const Ttest = testTemperature + 273.15; // K

  const exponent = (activationEnergy / k) * ((1 / Tuse) - (1 / Ttest));
  return Math.exp(exponent);
}

/**
 * Calculate Coffin-Manson cycles to failure
 */
export function calculateCoffinMansonLifetime(
  deltaT: number, // °C (temperature excursion)
  frequency: number, // Hz
  exponent: number = 2.5,
  constant: number = 1e6,
): number {
  return constant / (Math.pow(deltaT, exponent) * Math.pow(frequency, -0.3));
}

/**
 * Calculate MTTF from failure rate
 */
export function calculateMTTF(failureRate: number): number {
  // FIT to hours: MTTF = 1e9 / FIT
  return 1e9 / failureRate;
}

/**
 * Calculate failure rate from MTTF
 */
export function calculateFailureRate(mttf: number): number {
  // Hours to FIT: FIT = 1e9 / MTTF
  return 1e9 / mttf;
}

/**
 * Convert years to hours
 */
export function yearsToHours(years: number): number {
  return years * 365.25 * 24;
}

// ============================================================================
// Package Design Validation
// ============================================================================

/**
 * Validate TSV specification
 */
export function validateTSVSpec(spec: TSVSpecification): string[] {
  const errors: string[] = [];

  if (spec.diameter < 1 || spec.diameter > 20) {
    errors.push('TSV diameter should be between 1-20 μm');
  }

  const aspectRatio = spec.depth / spec.diameter;
  if (aspectRatio < 5 || aspectRatio > 20) {
    errors.push('TSV aspect ratio should be between 5:1 and 20:1');
  }

  if (spec.pitch < spec.diameter * 2) {
    errors.push('TSV pitch should be at least 2× diameter');
  }

  if (spec.resistance > 50) {
    errors.push('TSV resistance exceeds 50 mΩ specification');
  }

  if (spec.capacitance > 10) {
    errors.push('TSV capacitance exceeds 10 fF specification');
  }

  return errors;
}

/**
 * Validate microbump specification
 */
export function validateMicrobumpSpec(spec: MicrobumpSpecification): string[] {
  const errors: string[] = [];

  if (spec.pitch < 20 || spec.pitch > 55) {
    errors.push('Microbump pitch should be between 20-55 μm for standard implementations');
  }

  if (spec.coplanarity > 5) {
    errors.push('Microbump coplanarity should be ≤5 μm');
  }

  if (spec.shearStrength < 20) {
    errors.push('Microbump shear strength should be ≥20 MPa');
  }

  return errors;
}

/**
 * Validate thermal design
 */
export function validateThermalDesign(
  thermal: ThermalProperties,
  power: number,
  ambientTemp: number = 25,
): string[] {
  const errors: string[] = [];

  const junctionTemp = calculateJunctionTemperature(
    power,
    thermal.junctionToCaseResistance,
    ambientTemp
  );

  if (junctionTemp > thermal.maxJunctionTemp) {
    errors.push(
      `Junction temperature (${junctionTemp.toFixed(1)}°C) exceeds maximum (${thermal.maxJunctionTemp}°C)`
    );
  }

  if (thermal.junctionToCaseResistance > 1.0) {
    errors.push('Junction-to-case thermal resistance >1°C/W may indicate inadequate thermal design');
  }

  return errors;
}

/**
 * Validate HBM specification
 */
export function validateHBMSpec(spec: HBMSpecification): string[] {
  const errors: string[] = [];

  const expectedBandwidth = calculateHBMBandwidth(spec.interfaceWidth, spec.dataRate);
  const tolerance = 0.05; // 5% tolerance

  if (Math.abs(expectedBandwidth - spec.bandwidth) / spec.bandwidth > tolerance) {
    errors.push(
      `Calculated bandwidth (${expectedBandwidth.toFixed(1)} GB/s) does not match specified bandwidth (${spec.bandwidth} GB/s)`
    );
  }

  if (spec.stackHeight < 4 || spec.stackHeight > 12) {
    errors.push('HBM stack height should be between 4-12 dies');
  }

  return errors;
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Convert temperature from Celsius to Kelvin
 */
export function celsiusToKelvin(celsius: number): number {
  return celsius + 273.15;
}

/**
 * Convert temperature from Kelvin to Celsius
 */
export function kelvinToCelsius(kelvin: number): number {
  return kelvin - 273.15;
}

/**
 * Calculate percentage difference
 */
export function percentageDifference(value1: number, value2: number): number {
  return ((value1 - value2) / value2) * 100;
}

/**
 * Check if value is within range
 */
export function isInRange(
  value: number,
  min: number,
  max: number,
  inclusive: boolean = true
): boolean {
  if (inclusive) {
    return value >= min && value <= max;
  }
  return value > min && value < max;
}

/**
 * Calculate power density
 */
export function calculatePowerDensity(
  power: number, // W
  area: number, // cm²
): number {
  return power / area; // W/cm²
}

/**
 * Estimate package cost multiplier based on complexity
 */
export function estimateCostMultiplier(
  packageType: PackageType,
  dieCount: number,
  hasHBM: boolean,
  hasTSV: boolean,
): number {
  let multiplier = 1.0;

  // Base cost by package type
  switch (packageType) {
    case PackageType.INTERPOSER_2_5D:
      multiplier = 3.0;
      break;
    case PackageType.STACKED_3D:
      multiplier = 4.0;
      break;
    case PackageType.FAN_OUT_WLP:
      multiplier = 2.0;
      break;
    case PackageType.CHIPLET_BASED:
      multiplier = 2.5;
      break;
    default:
      multiplier = 1.5;
  }

  // Add cost for die count
  multiplier *= (1 + (dieCount - 1) * 0.3);

  // Add cost for HBM
  if (hasHBM) {
    multiplier *= 1.5;
  }

  // Add cost for TSV
  if (hasTSV) {
    multiplier *= 1.2;
  }

  return multiplier;
}

// ============================================================================
// Package Builder
// ============================================================================

/**
 * PackageBuilder class for constructing package designs
 */
export class PackageBuilder {
  private design: Partial<PackageDesign> = {};

  setType(type: PackageType): this {
    this.design.type = type;
    return this;
  }

  setDimensions(length: number, width: number, height: number, tolerance: number = 0.1): this {
    this.design.dimensions = { length, width, height, tolerance };
    return this;
  }

  addChiplet(chiplet: any): this {
    if (!this.design.dies) {
      this.design.dies = [];
    }
    this.design.dies.push(chiplet);
    return this;
  }

  setThermalProperties(thermal: ThermalProperties): this {
    if (!this.design.thermal) {
      this.design.thermal = {} as any;
    }
    this.design.thermal.properties = thermal;
    return this;
  }

  build(): PackageDesign {
    if (!this.design.designId) {
      this.design.designId = `PKG-${Date.now()}`;
    }

    // Validate required fields
    if (!this.design.type) {
      throw new Error('Package type must be specified');
    }

    return this.design as PackageDesign;
  }
}

// ============================================================================
// Export Main SDK Class
// ============================================================================

/**
 * Main SDK class for WIA-SEMI-020 Semiconductor Packaging Standard
 */
export class WIASemiconductorPackaging {
  readonly version = '1.0.0';
  readonly standard = 'WIA-SEMI-020';

  // Thermal analysis
  thermal = {
    calculateResistance: calculateThermalResistance,
    calculateJunctionTemp: calculateJunctionTemperature,
    calculateTimeConstant: estimateThermalTimeConstant,
    calculateHeatsinkRequirement: calculateHeatsinkRequirement,
  };

  // Electrical analysis
  electrical = {
    calculateResistance: calculateInterconnectResistance,
    calculateSkinDepth: calculateSkinDepth,
    calculateTargetImpedance: calculateTargetImpedance,
    calculateUCIeBandwidth: calculateUCIeBandwidth,
    calculateHBMBandwidth: calculateHBMBandwidth,
  };

  // TSV analysis
  tsv = {
    calculateAspectRatio: calculateTSVAspectRatio,
    calculateResistance: calculateTSVResistance,
    calculateCapacitance: calculateTSVCapacitance,
    validate: validateTSVSpec,
  };

  // Reliability analysis
  reliability = {
    calculateArrheniusAF: calculateArrheniusAF,
    calculateCoffinMansonLifetime: calculateCoffinMansonLifetime,
    calculateMTTF: calculateMTTF,
    calculateFailureRate: calculateFailureRate,
    yearsToHours: yearsToHours,
  };

  // Validation
  validate = {
    tsvSpec: validateTSVSpec,
    microbumpSpec: validateMicrobumpSpec,
    thermalDesign: validateThermalDesign,
    hbmSpec: validateHBMSpec,
  };

  // Utilities
  utils = {
    celsiusToKelvin: celsiusToKelvin,
    kelvinToCelsius: kelvinToCelsius,
    percentageDifference: percentageDifference,
    isInRange: isInRange,
    calculatePowerDensity: calculatePowerDensity,
    estimateCostMultiplier: estimateCostMultiplier,
  };

  // Package builder
  createPackage(): PackageBuilder {
    return new PackageBuilder();
  }
}

// Create and export default instance
export const wiaSemiconductorPackaging = new WIASemiconductorPackaging();
export default wiaSemiconductorPackaging;
