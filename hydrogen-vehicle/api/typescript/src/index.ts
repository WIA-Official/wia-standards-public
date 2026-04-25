/**
 * WIA-AUTO-007: Hydrogen Vehicle SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for hydrogen fuel cell vehicles including:
 * - Fuel cell efficiency calculations
 * - Vehicle range estimation
 * - Hydrogen storage validation
 * - Refueling optimization
 * - System performance analysis
 */

import {
  FuelCellParams,
  EfficiencyResult,
  RangeParams,
  RangeResult,
  TankValidation,
  TankValidationResult,
  RefuelingParams,
  RefuelingPlan,
  EfficiencyParams,
  SystemEfficiency,
  HydrogenStorage,
  StorageCapacity,
  SafetyCheck,
  HYDROGEN_CONSTANTS,
  HydrogenVehicleErrorCode,
  HydrogenVehicleError,
  HydrogenVehicle,
  VehicleStatus,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUTO-007 Hydrogen Vehicle SDK
 */
export class HydrogenVehicleSDK {
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
   * Calculate fuel cell efficiency
   *
   * @param params - Fuel cell operating parameters
   * @returns Efficiency calculation result
   */
  calculateFuelCellEfficiency(params: FuelCellParams): EfficiencyResult {
    const {
      powerOutput,
      hydrogenFlowRate,
      stackVoltage,
      stackCurrent,
      temperature = 80,
      pressure = 2.5,
    } = params;

    // Validate inputs
    if (powerOutput <= 0) {
      throw new HydrogenVehicleError(
        HydrogenVehicleErrorCode.INVALID_PARAMETERS,
        'Power output must be positive'
      );
    }

    if (hydrogenFlowRate <= 0) {
      throw new HydrogenVehicleError(
        HydrogenVehicleErrorCode.INVALID_PARAMETERS,
        'Hydrogen flow rate must be positive'
      );
    }

    // Calculate power from stack parameters
    const stackPower = (stackVoltage * stackCurrent) / 1000; // kW

    // Calculate hydrogen energy input (kW)
    const h2EnergyRate =
      (hydrogenFlowRate * HYDROGEN_CONSTANTS.LHV_H2) / 3.6; // MJ/h to kW

    // Calculate electrical efficiency
    const efficiency = Math.min(1.0, powerOutput / h2EnergyRate);

    // Calculate voltage efficiency
    const voltageEfficiency =
      stackVoltage / (stackCurrent > 0 ? HYDROGEN_CONSTANTS.THERMONEUTRAL_VOLTAGE : 1);

    // Estimate stack volume (simplified model)
    // Assume ~4 kW/L power density
    const stackVolume = powerOutput / 4;
    const powerDensity = powerOutput / stackVolume;

    // Calculate current density (simplified, assuming 500 cm² per cell)
    const cellArea = 500; // cm²
    const currentDensity = stackCurrent / cellArea;

    // Calculate heat generation
    const heatGeneration = h2EnergyRate - powerOutput;

    // Assess feasibility
    let feasibility: EfficiencyResult['feasibility'];
    if (efficiency >= 0.55) {
      feasibility = 'optimal';
    } else if (efficiency >= 0.45) {
      feasibility = 'acceptable';
    } else if (efficiency >= 0.30) {
      feasibility = 'suboptimal';
    } else {
      feasibility = 'critical';
    }

    return {
      efficiency,
      powerDensity,
      currentDensity,
      voltageEfficiency,
      heatGeneration,
      feasibility,
    };
  }

  /**
   * Calculate vehicle range
   *
   * @param params - Range calculation parameters
   * @returns Range estimation result
   */
  calculateVehicleRange(params: RangeParams): RangeResult {
    const {
      hydrogenCapacity,
      fuelCellEfficiency,
      systemEfficiency,
      energyConsumption,
      includeBatteryAssist = false,
      batteryEfficiency = 0.90,
    } = params;

    // Validate inputs
    if (hydrogenCapacity <= 0) {
      throw new HydrogenVehicleError(
        HydrogenVehicleErrorCode.INVALID_PARAMETERS,
        'Hydrogen capacity must be positive'
      );
    }

    if (fuelCellEfficiency <= 0 || fuelCellEfficiency > 1) {
      throw new HydrogenVehicleError(
        HydrogenVehicleErrorCode.INVALID_PARAMETERS,
        'Fuel cell efficiency must be between 0 and 1'
      );
    }

    // Calculate total hydrogen energy (MJ)
    const hydrogenEnergy = hydrogenCapacity * HYDROGEN_CONSTANTS.LHV_H2;

    // Calculate fuel cell output energy
    const fuelCellEnergy = hydrogenEnergy * fuelCellEfficiency;

    // Apply system efficiency
    const usableEnergy = fuelCellEnergy * systemEfficiency;

    // Calculate battery contribution if applicable
    let batteryEnergy = 0;
    if (includeBatteryAssist) {
      // Assume 1.5 kWh battery buffer
      batteryEnergy = 1.5 * 3.6 * batteryEfficiency; // Convert kWh to MJ
    }

    // Total usable energy
    const totalUsableEnergy = usableEnergy + batteryEnergy;

    // Calculate range (km)
    const range = totalUsableEnergy / energyConsumption;

    // Reserve buffer (10% of range)
    const rangeBuffer = range * 0.10;

    return {
      range: range - rangeBuffer,
      energyAvailable: hydrogenEnergy + (includeBatteryAssist ? batteryEnergy / batteryEfficiency : 0),
      energyUsable: totalUsableEnergy,
      rangeBuffer,
      breakdown: {
        hydrogenEnergy,
        fuelCellEnergy,
        usableEnergy,
        batteryEnergy: includeBatteryAssist ? batteryEnergy : undefined,
      },
    };
  }

  /**
   * Validate hydrogen tank pressure and temperature
   *
   * @param validation - Tank validation parameters
   * @returns Validation result with safety checks
   */
  validateTankPressure(validation: TankValidation): TankValidationResult {
    const {
      pressure,
      temperature,
      tankType,
      standard,
      isRefueling = false,
    } = validation;

    const warnings: string[] = [];
    const errors: string[] = [];

    // Determine working pressure based on standard
    const workingPressure =
      standard === 'H70'
        ? HYDROGEN_CONSTANTS.H70_PRESSURE
        : HYDROGEN_CONSTANTS.H35_PRESSURE;

    // Maximum allowed pressure (working pressure)
    const maxAllowedPressure = workingPressure;

    // Test pressure (1.5× working pressure)
    const testPressure = workingPressure * 1.5;

    // Calculate safety margin
    const safetyMargin = (maxAllowedPressure - pressure) / maxAllowedPressure;

    // Pressure checks
    if (pressure > maxAllowedPressure) {
      errors.push(
        `Tank pressure (${pressure} bar) exceeds working pressure (${maxAllowedPressure} bar)`
      );
    } else if (pressure > maxAllowedPressure * 0.95) {
      warnings.push(
        `Tank pressure approaching maximum (${((pressure / maxAllowedPressure) * 100).toFixed(1)}%)`
      );
    }

    // Temperature checks
    let temperatureOk = true;
    const maxOperatingTemp = 85; // °C
    const minOperatingTemp = -40; // °C

    if (temperature > maxOperatingTemp) {
      errors.push(
        `Tank temperature (${temperature}°C) exceeds maximum (${maxOperatingTemp}°C)`
      );
      temperatureOk = false;
    } else if (temperature > maxOperatingTemp * 0.9) {
      warnings.push(
        `Tank temperature approaching maximum (${temperature}°C / ${maxOperatingTemp}°C)`
      );
    }

    if (temperature < minOperatingTemp) {
      errors.push(
        `Tank temperature (${temperature}°C) below minimum (${minOperatingTemp}°C)`
      );
      temperatureOk = false;
    }

    // Refueling-specific checks
    if (isRefueling) {
      const maxRefuelingTemp = 85; // °C
      const tempRiseLimit = 85; // °C max rise

      if (temperature > maxRefuelingTemp) {
        errors.push(
          `Refueling stopped: temperature limit reached (${temperature}°C)`
        );
      }
    }

    // Safety margin check
    if (safetyMargin < HYDROGEN_CONSTANTS.MIN_SAFETY_MARGIN) {
      warnings.push(
        `Low safety margin (${(safetyMargin * 100).toFixed(1)}%). Recommended: >${(HYDROGEN_CONSTANTS.MIN_SAFETY_MARGIN * 100).toFixed(0)}%`
      );
    }

    const isValid = errors.length === 0;

    return {
      isValid,
      warnings,
      errors,
      safetyMargin,
      maxAllowedPressure,
      temperatureOk,
    };
  }

  /**
   * Optimize hydrogen refueling parameters
   *
   * @param params - Refueling parameters
   * @returns Optimized refueling plan
   */
  optimizeRefueling(params: RefuelingParams): RefuelingPlan {
    const {
      targetPressure,
      ambientTemp,
      currentPressure,
      currentTemp,
      tankVolume,
      tankType = 'Type IV',
      standard = 'H70',
    } = params;

    const safetyChecks: SafetyCheck[] = [];

    // Determine pre-cooling temperature based on standard
    let preCoolTemp: number;
    if (standard === 'H70') {
      // H70: -40 to -33°C
      preCoolTemp = Math.max(-40, -40 + (ambientTemp + 10) / 5);
      preCoolTemp = Math.min(-33, preCoolTemp);
    } else {
      // H35: -20 to -10°C
      preCoolTemp = Math.max(-20, -20 + (ambientTemp + 5) / 3);
      preCoolTemp = Math.min(-10, preCoolTemp);
    }

    // Calculate hydrogen mass to dispense
    const R = HYDROGEN_CONSTANTS.GAS_CONSTANT;
    const MW = HYDROGEN_CONSTANTS.MOLECULAR_WEIGHT / 1000; // kg/mol
    const T_avg = ((currentTemp + 273.15) + (ambientTemp + 273.15)) / 2; // K

    // Simplified mass calculation (ignoring compressibility factor)
    const currentMass =
      (currentPressure * 1e5 * tankVolume / 1000 * MW) / (R * (currentTemp + 273.15));
    const targetMass =
      (targetPressure * 1e5 * tankVolume / 1000 * MW) / (R * T_avg);
    const hydrogenMass = Math.max(0, targetMass - currentMass);

    // Flow rate calculation (SAE J2601 recommends ~60 g/s)
    const flowRate = 0.060 * 60; // kg/min (3.6 kg/min)

    // Estimate refueling time
    const estimatedTime = hydrogenMass / flowRate; // minutes

    // Pressure ramp rate (bar/s)
    const pressureRampRate = (targetPressure - currentPressure) / (estimatedTime * 60);

    // Estimate final temperature
    // Temperature rise due to compression
    const tempRise = Math.min(
      85,
      (targetPressure - currentPressure) / 10 + ambientTemp / 2
    );
    const finalTemp = currentTemp + tempRise;

    // Safety checks
    safetyChecks.push({
      name: 'Pre-cooling Temperature',
      status: preCoolTemp >= -45 && preCoolTemp <= -10 ? 'pass' : 'fail',
      description: 'Verify pre-cooling meets standard requirements',
      value: preCoolTemp,
      threshold: standard === 'H70' ? -40 : -20,
    });

    safetyChecks.push({
      name: 'Final Temperature',
      status: finalTemp <= 85 ? 'pass' : 'fail',
      description: 'Ensure tank temperature stays within limits',
      value: finalTemp,
      threshold: 85,
      correctiveAction: 'Reduce flow rate or improve pre-cooling',
    });

    safetyChecks.push({
      name: 'Pressure Ramp Rate',
      status: pressureRampRate <= 5.0 ? 'pass' : 'warning',
      description: 'Check pressure increase rate',
      value: pressureRampRate,
      threshold: 5.0,
      correctiveAction: 'Reduce flow rate to lower ramp rate',
    });

    safetyChecks.push({
      name: 'Target Pressure',
      status: targetPressure <= (standard === 'H70' ? 700 : 350) ? 'pass' : 'fail',
      description: 'Verify target pressure matches standard',
      value: targetPressure,
      threshold: standard === 'H70' ? 700 : 350,
    });

    const isSafe = safetyChecks.every((check) => check.status !== 'fail');

    return {
      preCoolTemp,
      flowRate,
      estimatedTime,
      finalTemp,
      pressureRampRate,
      hydrogenMass,
      safetyChecks,
      isSafe,
    };
  }

  /**
   * Calculate overall system efficiency
   *
   * @param params - Efficiency parameters for all components
   * @returns System efficiency breakdown
   */
  calculateSystemEfficiency(params: EfficiencyParams): SystemEfficiency {
    const {
      fuelCellEfficiency,
      converterEfficiency,
      inverterEfficiency,
      motorEfficiency,
      transmissionEfficiency = 0.97,
      auxiliaryEfficiency = 0.93,
    } = params;

    // Validate all efficiencies are in range [0, 1]
    const efficiencies = [
      fuelCellEfficiency,
      converterEfficiency,
      inverterEfficiency,
      motorEfficiency,
      transmissionEfficiency,
      auxiliaryEfficiency,
    ];

    if (efficiencies.some((e) => e <= 0 || e > 1)) {
      throw new HydrogenVehicleError(
        HydrogenVehicleErrorCode.INVALID_PARAMETERS,
        'All efficiencies must be between 0 and 1'
      );
    }

    // Calculate powertrain efficiency (tank to wheel)
    const powertrainEfficiency =
      fuelCellEfficiency *
      converterEfficiency *
      inverterEfficiency *
      motorEfficiency *
      transmissionEfficiency;

    // Tank-to-wheel includes auxiliaries
    const tankToWheelEfficiency = powertrainEfficiency * auxiliaryEfficiency;

    // Calculate total losses
    const totalLosses = (1 - tankToWheelEfficiency) * 100;

    return {
      powertrainEfficiency,
      tankToWheelEfficiency,
      breakdown: {
        fuelCell: fuelCellEfficiency,
        converter: converterEfficiency,
        inverter: inverterEfficiency,
        motor: motorEfficiency,
        transmission: transmissionEfficiency,
        auxiliary: auxiliaryEfficiency,
      },
      totalLosses,
    };
  }

  /**
   * Calculate hydrogen storage capacity
   *
   * @param storage - Hydrogen storage system
   * @returns Storage capacity metrics
   */
  calculateStorageCapacity(storage: HydrogenStorage): StorageCapacity {
    const { pressure, totalVolume, tankType, standard } = storage;

    // Simplified mass calculation
    const R = HYDROGEN_CONSTANTS.GAS_CONSTANT;
    const MW = HYDROGEN_CONSTANTS.MOLECULAR_WEIGHT / 1000; // kg/mol
    const T = 288; // 15°C in K
    const Z = 1.06; // Compressibility factor at 700 bar

    // Mass calculation: m = (P × V × MW) / (Z × R × T)
    const mass =
      (pressure * 1e5 * (totalVolume / 1000) * MW) / (Z * R * T);

    // Estimate tank system weight based on type
    let tankWeight: number;
    if (tankType === 'Type IV') {
      // Type IV: ~20-25% gravimetric density
      tankWeight = mass / 0.055 - mass;
    } else {
      // Type III: ~15-18% gravimetric density
      tankWeight = mass / 0.045 - mass;
    }

    const gravimetricDensity = (mass / (mass + tankWeight)) * 100;

    // Volumetric density
    const volumetricDensity = mass / (totalVolume / 1000);

    // Energy stored
    const energyStored = mass * HYDROGEN_CONSTANTS.LHV_H2;
    const energyStoredKwh = energyStored / 3.6;

    return {
      mass,
      gravimetricDensity,
      volumetricDensity,
      energyStored,
      energyStoredKwh,
      tankWeight,
    };
  }

  /**
   * Get vehicle status summary
   *
   * @param vehicle - Complete vehicle data
   * @returns Vehicle status summary
   */
  getVehicleStatus(vehicle: HydrogenVehicle): VehicleStatus {
    const warnings: string[] = [];
    const errors: string[] = [];

    // Check fuel cell
    if (vehicle.fuelCell.status === 'fault') {
      errors.push('Fuel cell system fault detected');
    }
    if (vehicle.fuelCell.efficiency < 0.40) {
      warnings.push('Fuel cell efficiency below optimal range');
    }

    // Check hydrogen storage
    if (vehicle.hydrogenStorage.soc < 0.15) {
      warnings.push('Low hydrogen level - refueling recommended');
    }
    if (vehicle.hydrogenStorage.soc < 0.05) {
      errors.push('Critical hydrogen level - refuel immediately');
    }

    // Check temperature
    if (vehicle.hydrogenStorage.temperature > 75) {
      warnings.push('Tank temperature elevated');
    }

    // Check motor
    if (vehicle.motor.status === 'fault') {
      errors.push('Motor system fault detected');
    }

    // Determine if vehicle is ready
    const isReady =
      vehicle.fuelCell.status !== 'fault' &&
      vehicle.motor.status !== 'fault' &&
      vehicle.hydrogenStorage.soc > 0.05 &&
      errors.length === 0;

    return {
      vehicleId: vehicle.vehicleId,
      timestamp: vehicle.timestamp,
      isReady,
      fuelCell: {
        power: vehicle.fuelCell.powerOutput,
        efficiency: vehicle.fuelCell.efficiency,
        status: vehicle.fuelCell.status,
      },
      hydrogen: {
        massRemaining: vehicle.hydrogenStorage.massRemaining,
        soc: vehicle.hydrogenStorage.soc,
        pressure: vehicle.hydrogenStorage.pressure,
      },
      range: {
        remaining: vehicle.rangeRemaining,
        consumption: vehicle.energyConsumption,
      },
      warnings,
      errors,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Convert energy units
   */
  private convertEnergy(value: number, from: 'MJ' | 'kWh', to: 'MJ' | 'kWh'): number {
    if (from === to) return value;
    if (from === 'MJ' && to === 'kWh') return value / 3.6;
    return value * 3.6; // kWh to MJ
  }

  /**
   * Format energy in human-readable form
   */
  private formatEnergy(energy: number, unit: 'MJ' | 'kWh' = 'MJ'): string {
    if (unit === 'kWh') {
      return `${energy.toFixed(2)} kWh`;
    }
    if (energy < 1000) {
      return `${energy.toFixed(2)} MJ`;
    }
    return `${(energy / 1000).toFixed(2)} GJ`;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate fuel cell efficiency (standalone function)
 */
export function calculateFuelCellEfficiency(
  params: FuelCellParams
): EfficiencyResult {
  const sdk = new HydrogenVehicleSDK();
  return sdk.calculateFuelCellEfficiency(params);
}

/**
 * Calculate vehicle range (standalone function)
 */
export function calculateVehicleRange(params: RangeParams): RangeResult {
  const sdk = new HydrogenVehicleSDK();
  return sdk.calculateVehicleRange(params);
}

/**
 * Validate tank pressure (standalone function)
 */
export function validateTankPressure(
  validation: TankValidation
): TankValidationResult {
  const sdk = new HydrogenVehicleSDK();
  return sdk.validateTankPressure(validation);
}

/**
 * Optimize refueling (standalone function)
 */
export function optimizeRefueling(params: RefuelingParams): RefuelingPlan {
  const sdk = new HydrogenVehicleSDK();
  return sdk.optimizeRefueling(params);
}

/**
 * Calculate system efficiency (standalone function)
 */
export function calculateSystemEfficiency(
  params: EfficiencyParams
): SystemEfficiency {
  const sdk = new HydrogenVehicleSDK();
  return sdk.calculateSystemEfficiency(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { HydrogenVehicleSDK };
export default HydrogenVehicleSDK;
