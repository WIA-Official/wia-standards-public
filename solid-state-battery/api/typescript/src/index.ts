/**
 * WIA-AUTO-028: Solid-State Battery SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Energy Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for solid-state battery technology including:
 * - Energy density calculations
 * - Fast charging time estimation
 * - Thermal simulation
 * - Performance validation
 * - Cycle life prediction
 */

import {
  EnergyDensityParams,
  EnergyDensityResult,
  ChargingTimeParams,
  ChargingTimeResult,
  ThermalSimulationParams,
  ThermalSimulationResult,
  BatteryValidationParams,
  ValidationResult,
  CheckResult,
  ChargingLevel,
  MassBreakdown,
  StateOfHealth,
  CycleLifePredictionParams,
  CycleLifePredictionResult,
  BATTERY_CONSTANTS,
  BatteryErrorCode,
  SolidStateBatteryError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUTO-028 Solid-State Battery SDK
 */
export class SolidStateBatterySDK {
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
   * Calculate energy density for a solid-state battery
   *
   * @param params - Energy density parameters
   * @returns Energy density calculation results
   */
  calculateEnergyDensity(params: EnergyDensityParams): EnergyDensityResult {
    const { capacity, voltage, mass, volume, efficiency = 0.99 } = params;

    // Validate inputs
    if (capacity <= 0 || voltage <= 0 || mass <= 0) {
      throw new SolidStateBatteryError(
        BatteryErrorCode.INVALID_PARAMETERS,
        'Capacity, voltage, and mass must be positive'
      );
    }

    if (efficiency <= 0 || efficiency > 1) {
      throw new SolidStateBatteryError(
        BatteryErrorCode.INVALID_PARAMETERS,
        'Efficiency must be between 0 and 1'
      );
    }

    // Calculate total energy: E = C × V × η
    const totalEnergy = capacity * voltage * efficiency;

    // Calculate gravimetric energy density: E_g = E / m
    const gravimetric = totalEnergy / mass;

    // Calculate volumetric energy density if volume provided: E_v = E / V
    let volumetric: number | undefined;
    if (volume && volume > 0) {
      volumetric = totalEnergy / volume;
    }

    // Determine grade and level based on gravimetric energy density
    let grade: EnergyDensityResult['grade'];
    let level: EnergyDensityResult['level'];

    if (gravimetric >= BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.PREMIUM.gravimetric) {
      grade = 'Premium';
      level = 3;
    } else if (gravimetric >= BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.STANDARD.gravimetric) {
      grade = 'Standard';
      level = 2;
    } else if (gravimetric >= BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.BASIC.gravimetric) {
      grade = 'Basic';
      level = 1;
    } else {
      grade = 'Basic';
      level = 1;
    }

    // Calculate comparison to targets
    const targetGravimetric =
      level === 3
        ? BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.PREMIUM.gravimetric
        : level === 2
        ? BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.STANDARD.gravimetric
        : BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.BASIC.gravimetric;

    const targetVolumetric =
      level === 3
        ? BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.PREMIUM.volumetric
        : level === 2
        ? BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.STANDARD.volumetric
        : BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.BASIC.volumetric;

    return {
      gravimetric,
      volumetric,
      totalEnergy,
      grade,
      level,
      comparison: {
        gravimetricTarget: targetGravimetric,
        volumetricTarget: volumetric ? targetVolumetric : undefined,
        gravimetricAchievement: (gravimetric / targetGravimetric) * 100,
        volumetricAchievement: volumetric
          ? (volumetric / targetVolumetric) * 100
          : undefined,
      },
    };
  }

  /**
   * Estimate charging time for given parameters
   *
   * @param params - Charging time parameters
   * @returns Charging time estimation results
   */
  estimateChargingTime(params: ChargingTimeParams): ChargingTimeResult {
    const {
      capacity,
      currentSOC,
      targetSOC,
      chargingPower,
      temperature,
      coolingType = 'air',
      efficiency = 0.96,
    } = params;

    // Validate inputs
    if (capacity <= 0 || chargingPower <= 0) {
      throw new SolidStateBatteryError(
        BatteryErrorCode.INVALID_PARAMETERS,
        'Capacity and charging power must be positive'
      );
    }

    if (currentSOC < 0 || currentSOC > 100 || targetSOC < 0 || targetSOC > 100) {
      throw new SolidStateBatteryError(
        BatteryErrorCode.INVALID_PARAMETERS,
        'SOC values must be between 0 and 100'
      );
    }

    if (currentSOC >= targetSOC) {
      throw new SolidStateBatteryError(
        BatteryErrorCode.INVALID_PARAMETERS,
        'Current SOC must be less than target SOC'
      );
    }

    const warnings: string[] = [];

    // Calculate SOC change
    const socChange = targetSOC - currentSOC;

    // Calculate charge needed (Ah)
    const chargeNeeded = (capacity * socChange) / 100;

    // Estimate average voltage during charging (simplified)
    const avgVoltage = 3.7; // Typical for NMC-based cells

    // Calculate average current: I = P / V
    const avgCurrent = chargingPower * 1000 / avgVoltage;

    // Calculate C-rate
    const cRate = avgCurrent / capacity;

    // Calculate charging time: t = Q / (I × η) in hours
    const timeHours = chargeNeeded / (avgCurrent * efficiency);
    const minutes = timeHours * 60;

    // Determine charging level
    let level: ChargingLevel;
    if (cRate >= BATTERY_CONSTANTS.CHARGING.ULTRA_FAST_CRATE) {
      level = 'ultra-fast';
    } else if (cRate >= BATTERY_CONSTANTS.CHARGING.FAST_CRATE) {
      level = 'fast';
    } else if (cRate >= BATTERY_CONSTANTS.CHARGING.STANDARD_CRATE) {
      level = 'standard';
    } else {
      level = 'slow';
    }

    // Estimate temperature rise
    const powerLoss = chargingPower * (1 - efficiency);
    const timeSeconds = timeHours * 3600;
    const heatGenerated = powerLoss * timeSeconds; // kJ

    // Simplified thermal model
    const batteryMass = capacity * 0.006; // Rough estimate: 6 kg per 100 Ah
    const specificHeat = 1000; // J/(kg·K) approximate
    const tempRiseNocooling = (heatGenerated * 1000) / (batteryMass * specificHeat);

    // Apply cooling effectiveness
    let coolingEffectiveness: number;
    switch (coolingType) {
      case 'passive':
        coolingEffectiveness = 0.3;
        break;
      case 'air':
        coolingEffectiveness = 0.7;
        break;
      case 'liquid':
        coolingEffectiveness = 0.9;
        break;
      default:
        coolingEffectiveness = 0.5;
    }

    const tempRise = tempRiseNocooling * (1 - coolingEffectiveness);
    const maxTemperature = temperature + tempRise;

    // Check for warnings
    if (maxTemperature > BATTERY_CONSTANTS.TEMPERATURE.OPTIMAL_MAX) {
      warnings.push(
        `Temperature may exceed optimal range (${maxTemperature.toFixed(1)}°C > ${BATTERY_CONSTANTS.TEMPERATURE.OPTIMAL_MAX}°C)`
      );
    }

    if (maxTemperature > BATTERY_CONSTANTS.TEMPERATURE.SHUTDOWN) {
      warnings.push('CRITICAL: Temperature may trigger safety shutdown');
    }

    if (cRate > 6) {
      warnings.push('Very high C-rate may accelerate battery degradation');
    }

    const coolingRequired =
      maxTemperature > BATTERY_CONSTANTS.TEMPERATURE.OPTIMAL_MAX ||
      cRate >= BATTERY_CONSTANTS.CHARGING.FAST_CRATE;

    // Energy efficiency calculation
    const energyEfficiency = efficiency * 100;

    return {
      minutes: Math.round(minutes * 10) / 10,
      cRate: Math.round(cRate * 100) / 100,
      level,
      maxTemperature: Math.round(maxTemperature * 10) / 10,
      energyEfficiency: Math.round(energyEfficiency * 10) / 10,
      warnings,
      coolingRequired,
    };
  }

  /**
   * Simulate thermal behavior of battery
   *
   * @param params - Thermal simulation parameters
   * @returns Thermal simulation results
   */
  simulateThermal(params: ThermalSimulationParams): ThermalSimulationResult {
    const {
      ambientTemperature,
      power,
      coolingType,
      duration,
      initialTemperature = ambientTemperature,
      thermalProperties,
    } = params;

    // Validate inputs
    if (duration <= 0) {
      throw new SolidStateBatteryError(
        BatteryErrorCode.INVALID_PARAMETERS,
        'Duration must be positive'
      );
    }

    const warnings: string[] = [];

    // Use provided thermal properties or defaults
    const mass = thermalProperties?.mass || 30; // kg (typical for 100 kWh pack)
    const specificHeat = thermalProperties?.specificHeat || 1000; // J/(kg·K)
    const surfaceArea = thermalProperties?.surfaceArea || 2; // m²

    // Determine heat transfer coefficient based on cooling type
    let heatTransferCoeff: number; // W/(m²·K)
    switch (coolingType) {
      case 'passive':
        heatTransferCoeff = 7;
        break;
      case 'air':
        heatTransferCoeff = 50;
        break;
      case 'liquid':
        heatTransferCoeff = 200;
        break;
      case 'phase-change':
        heatTransferCoeff = 300;
        break;
      default:
        heatTransferCoeff = 50;
    }

    // Simulation parameters
    const timeStep = 10; // seconds
    const steps = Math.ceil(duration / timeStep);

    const temperatureProfile: number[] = [];
    const timeProfile: number[] = [];

    let currentTemp = initialTemperature;
    let maxTemp = initialTemperature;
    let minTemp = initialTemperature;
    let totalHeatGenerated = 0;
    let totalHeatDissipated = 0;

    // Thermal simulation loop
    for (let i = 0; i <= steps; i++) {
      const time = i * timeStep;
      timeProfile.push(time);
      temperatureProfile.push(currentTemp);

      if (i < steps) {
        // Heat generation: Q_gen = P × Δt
        const heatGen = power * timeStep; // J

        // Heat dissipation: Q_diss = h × A × (T - T_amb) × Δt
        const tempDiff = currentTemp - ambientTemperature;
        const heatDiss = heatTransferCoeff * surfaceArea * tempDiff * timeStep; // J

        // Net heat: Q_net = Q_gen - Q_diss
        const netHeat = heatGen - heatDiss;

        // Temperature change: ΔT = Q_net / (m × c_p)
        const tempChange = netHeat / (mass * specificHeat);

        // Update temperature
        currentTemp += tempChange;

        // Track totals
        totalHeatGenerated += heatGen;
        totalHeatDissipated += heatDiss;

        // Track extremes
        if (currentTemp > maxTemp) maxTemp = currentTemp;
        if (currentTemp < minTemp) minTemp = currentTemp;
      }
    }

    // Determine thermal runaway risk
    let thermalRunawayRisk: ThermalSimulationResult['thermalRunawayRisk'];
    if (maxTemp >= BATTERY_CONSTANTS.TEMPERATURE.THERMAL_RUNAWAY) {
      thermalRunawayRisk = 'high';
      warnings.push('CRITICAL: Thermal runaway temperature reached!');
    } else if (maxTemp >= BATTERY_CONSTANTS.TEMPERATURE.SHUTDOWN) {
      thermalRunawayRisk = 'medium';
      warnings.push('WARNING: Approaching shutdown temperature');
    } else if (maxTemp >= BATTERY_CONSTANTS.TEMPERATURE.OPTIMAL_MAX + 10) {
      thermalRunawayRisk = 'low';
      warnings.push('Temperature exceeds optimal range');
    } else {
      thermalRunawayRisk = 'none';
    }

    // Calculate required cooling power to maintain optimal temp
    const tempDiffAtEnd = temperatureProfile[temperatureProfile.length - 1] - ambientTemperature;
    const steadyStateHeatDissipation = heatTransferCoeff * surfaceArea * tempDiffAtEnd;
    const coolingPowerRequired = Math.max(0, power - steadyStateHeatDissipation / 1000);

    return {
      finalTemperature: Math.round(currentTemp * 10) / 10,
      maxTemperature: Math.round(maxTemp * 10) / 10,
      minTemperature: Math.round(minTemp * 10) / 10,
      temperatureProfile: {
        time: timeProfile,
        temperature: temperatureProfile.map((t) => Math.round(t * 10) / 10),
      },
      heatGenerated: totalHeatGenerated,
      heatDissipated: totalHeatDissipated,
      coolingPowerRequired: Math.round(coolingPowerRequired * 10) / 10,
      thermalRunawayRisk,
      warnings,
    };
  }

  /**
   * Validate battery performance against WIA-AUTO-028 standards
   *
   * @param params - Battery validation parameters
   * @returns Validation result with detailed checks
   */
  validateBatteryPerformance(params: BatteryValidationParams): ValidationResult {
    const {
      capacity,
      voltage,
      energyDensity,
      cycleLife,
      fastChargingTime,
      temperature,
      powerDensity,
      certifications = [],
    } = params;

    const errors: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];
    const comparisonToStandard: ValidationResult['comparisonToStandard'] = [];

    // Initialize check results
    const checks: ValidationResult['checks'] = {
      energyDensity: { status: 'not-tested' },
      cycleLife: { status: 'not-tested' },
      fastCharging: { status: 'not-tested' },
      temperature: { status: 'not-tested' },
      powerDensity: { status: 'not-tested' },
      safety: { status: 'not-tested' },
    };

    // Energy Density Check
    if (energyDensity >= BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.PREMIUM.gravimetric) {
      checks.energyDensity = {
        status: 'pass',
        measured: energyDensity,
        required: BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.PREMIUM.gravimetric,
        unit: 'Wh/kg',
        details: 'Premium grade',
      };
    } else if (energyDensity >= BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.STANDARD.gravimetric) {
      checks.energyDensity = {
        status: 'pass',
        measured: energyDensity,
        required: BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.STANDARD.gravimetric,
        unit: 'Wh/kg',
        details: 'Standard grade',
      };
    } else if (energyDensity >= BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.BASIC.gravimetric) {
      checks.energyDensity = {
        status: 'pass',
        measured: energyDensity,
        required: BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.BASIC.gravimetric,
        unit: 'Wh/kg',
        details: 'Basic grade',
      };
    } else {
      checks.energyDensity = {
        status: 'fail',
        measured: energyDensity,
        required: BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.BASIC.gravimetric,
        unit: 'Wh/kg',
        details: 'Below minimum standard',
      };
      errors.push(
        `Energy density ${energyDensity} Wh/kg is below minimum standard (${BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.BASIC.gravimetric} Wh/kg)`
      );
    }

    comparisonToStandard.push({
      parameter: 'Energy Density',
      required: BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.BASIC.gravimetric,
      actual: energyDensity,
      unit: 'Wh/kg',
      status:
        energyDensity >= BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.PREMIUM.gravimetric
          ? 'exceeds'
          : energyDensity >= BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.BASIC.gravimetric
          ? 'pass'
          : 'fail',
    });

    // Cycle Life Check
    if (cycleLife >= BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.PREMIUM) {
      checks.cycleLife = {
        status: 'pass',
        measured: cycleLife,
        required: BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.PREMIUM,
        unit: 'cycles',
        details: 'Premium grade',
      };
    } else if (cycleLife >= BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.STANDARD) {
      checks.cycleLife = {
        status: 'pass',
        measured: cycleLife,
        required: BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.STANDARD,
        unit: 'cycles',
        details: 'Standard grade',
      };
    } else if (cycleLife >= BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.BASIC) {
      checks.cycleLife = {
        status: 'pass',
        measured: cycleLife,
        required: BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.BASIC,
        unit: 'cycles',
        details: 'Basic grade',
      };
    } else {
      checks.cycleLife = {
        status: 'fail',
        measured: cycleLife,
        required: BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.BASIC,
        unit: 'cycles',
        details: 'Below minimum standard',
      };
      errors.push(
        `Cycle life ${cycleLife} cycles is below minimum standard (${BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.BASIC} cycles)`
      );
    }

    comparisonToStandard.push({
      parameter: 'Cycle Life',
      required: BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.BASIC,
      actual: cycleLife,
      unit: 'cycles',
      status:
        cycleLife >= BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.PREMIUM
          ? 'exceeds'
          : cycleLife >= BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.BASIC
          ? 'pass'
          : 'fail',
    });

    // Fast Charging Check
    if (fastChargingTime !== undefined) {
      if (fastChargingTime <= 10) {
        checks.fastCharging = {
          status: 'pass',
          measured: fastChargingTime,
          required: 10,
          unit: 'minutes',
          details: 'Ultra-fast charging',
        };
      } else if (fastChargingTime <= 15) {
        checks.fastCharging = {
          status: 'pass',
          measured: fastChargingTime,
          required: 15,
          unit: 'minutes',
          details: 'Fast charging',
        };
      } else if (fastChargingTime <= 30) {
        checks.fastCharging = {
          status: 'pass',
          measured: fastChargingTime,
          required: 30,
          unit: 'minutes',
          details: 'Standard charging',
        };
      } else {
        checks.fastCharging = {
          status: 'warning',
          measured: fastChargingTime,
          required: 30,
          unit: 'minutes',
          details: 'Slow charging',
        };
        warnings.push('Fast charging time exceeds standard target');
      }

      comparisonToStandard.push({
        parameter: 'Fast Charging (10-80%)',
        required: 15,
        actual: fastChargingTime,
        unit: 'minutes',
        status: fastChargingTime <= 10 ? 'exceeds' : fastChargingTime <= 15 ? 'pass' : 'fail',
      });
    }

    // Temperature Range Check
    if (temperature) {
      const tempRangeOk =
        temperature.min <= BATTERY_CONSTANTS.TEMPERATURE.OPERATING_MIN &&
        temperature.max >= BATTERY_CONSTANTS.TEMPERATURE.OPERATING_MAX;

      checks.temperature = {
        status: tempRangeOk ? 'pass' : 'warning',
        details: `Range: ${temperature.min}°C to ${temperature.max}°C`,
      };

      if (!tempRangeOk) {
        warnings.push(
          `Temperature range (${temperature.min}°C to ${temperature.max}°C) does not fully cover standard operating range (-30°C to 60°C)`
        );
      }
    }

    // Power Density Check
    if (powerDensity !== undefined) {
      if (powerDensity >= 1000) {
        checks.powerDensity = {
          status: 'pass',
          measured: powerDensity,
          required: 1000,
          unit: 'W/kg',
          details: 'Meets target',
        };
      } else {
        checks.powerDensity = {
          status: 'warning',
          measured: powerDensity,
          required: 1000,
          unit: 'W/kg',
          details: 'Below target',
        };
        warnings.push('Power density below recommended target (1000 W/kg)');
      }

      comparisonToStandard.push({
        parameter: 'Power Density',
        required: 1000,
        actual: powerDensity,
        unit: 'W/kg',
        status: powerDensity >= 1000 ? 'pass' : 'fail',
      });
    }

    // Safety Certifications Check
    const requiredCerts = ['UL2580', 'UN38.3'];
    const hasRequired = requiredCerts.every((cert) =>
      certifications.some((c) => c.includes(cert))
    );

    checks.safety = {
      status: hasRequired ? 'pass' : 'warning',
      details: `Certifications: ${certifications.join(', ') || 'None'}`,
    };

    if (!hasRequired) {
      warnings.push('Missing recommended safety certifications (UL2580, UN38.3)');
      recommendations.push('Obtain UL2580 and UN38.3 certifications for market compliance');
    }

    // Determine overall level and grade
    let level: ValidationResult['level'];
    let grade: ValidationResult['grade'];

    const passCount =
      (checks.energyDensity.status === 'pass' ? 1 : 0) +
      (checks.cycleLife.status === 'pass' ? 1 : 0) +
      (checks.fastCharging.status === 'pass' ? 1 : 0) +
      (checks.powerDensity.status === 'pass' ? 1 : 0);

    if (
      energyDensity >= BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.PREMIUM.gravimetric &&
      cycleLife >= BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.PREMIUM &&
      passCount >= 3
    ) {
      level = 3;
      grade = 'Premium';
    } else if (
      energyDensity >= BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.STANDARD.gravimetric &&
      cycleLife >= BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.STANDARD &&
      passCount >= 2
    ) {
      level = 2;
      grade = 'Standard';
    } else if (
      energyDensity >= BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.BASIC.gravimetric &&
      cycleLife >= BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.BASIC
    ) {
      level = 1;
      grade = 'Basic';
    } else {
      level = 1;
      grade = 'Below Standard';
    }

    // Add recommendations
    if (level < 3) {
      if (energyDensity < BATTERY_CONSTANTS.ENERGY_DENSITY_TARGETS.PREMIUM.gravimetric) {
        recommendations.push('Improve energy density through electrolyte optimization');
      }
      if (cycleLife < BATTERY_CONSTANTS.CYCLE_LIFE_TARGETS.PREMIUM) {
        recommendations.push('Enhance cycle life with better interface engineering');
      }
      if (fastChargingTime && fastChargingTime > 15) {
        recommendations.push('Optimize for faster charging with higher ionic conductivity');
      }
    }

    const isValid = errors.length === 0;

    return {
      isValid,
      level,
      grade,
      checks,
      warnings,
      errors,
      recommendations,
      comparisonToStandard,
    };
  }

  /**
   * Calculate mass breakdown of battery components
   *
   * @param totalMass - Total battery mass (kg)
   * @param chemistry - Battery chemistry type
   * @returns Mass breakdown
   */
  calculateMassBreakdown(totalMass: number, chemistry: string = 'NMC-LLZO'): MassBreakdown {
    // Typical distribution for solid-state batteries
    const anodeRatio = 0.12;
    const cathodeRatio = 0.40;
    const electrolyteRatio = 0.20;
    const currentCollectorsRatio = 0.10;
    const packagingRatio = 0.15;
    const otherRatio = 0.03;

    const breakdown: MassBreakdown = {
      anode: totalMass * anodeRatio,
      cathode: totalMass * cathodeRatio,
      electrolyte: totalMass * electrolyteRatio,
      currentCollectors: totalMass * currentCollectorsRatio,
      packaging: totalMass * packagingRatio,
      other: totalMass * otherRatio,
      total: totalMass,
      activeMaterialRatio: ((anodeRatio + cathodeRatio) * 100),
    };

    return breakdown;
  }

  /**
   * Predict cycle life based on current health metrics
   *
   * @param params - Cycle life prediction parameters
   * @returns Cycle life prediction
   */
  predictCycleLife(params: CycleLifePredictionParams): CycleLifePredictionResult {
    const {
      currentCycles,
      currentRetention,
      averageDOD,
      averageCRate,
      averageTemperature,
      eolCriterion = 80,
    } = params;

    // Calculate capacity fade so far
    const capacityFade = 100 - currentRetention;

    // Calculate degradation rate (% per 100 cycles)
    const degradationRate = currentCycles > 0 ? (capacityFade / currentCycles) * 100 : 0;

    // Adjust degradation rate based on usage conditions
    let adjustedRate = degradationRate;

    // Temperature factor (optimal is 25°C)
    const tempDeviation = Math.abs(averageTemperature - 25);
    const tempFactor = 1 + tempDeviation * 0.015; // 1.5% increase per degree
    adjustedRate *= tempFactor;

    // DOD factor (lower DOD is better)
    const dodFactor = 0.5 + (averageDOD / 100) * 0.5; // Range: 0.5 to 1.0
    adjustedRate *= dodFactor;

    // C-rate factor (lower C-rate is better)
    const crateFactor = 1 + Math.max(0, (averageCRate - 1) * 0.1);
    adjustedRate *= crateFactor;

    // Calculate remaining fade to EOL
    const remainingFade = currentRetention - eolCriterion;

    // Predict remaining cycles
    const remainingCycles =
      adjustedRate > 0 ? Math.round((remainingFade / adjustedRate) * 100) : Infinity;

    // Total predicted cycles
    const predictedCycles = currentCycles + remainingCycles;

    // Confidence interval (±20%)
    const confidence = {
      lower: Math.round(predictedCycles * 0.8),
      upper: Math.round(predictedCycles * 1.2),
    };

    // Determine factor impacts
    const factors = {
      temperature:
        averageTemperature >= 15 && averageTemperature <= 35
          ? 'positive'
          : averageTemperature < 15 || averageTemperature > 45
          ? 'negative'
          : ('neutral' as const),
      dod: averageDOD <= 80 ? 'positive' : averageDOD >= 95 ? 'negative' : ('neutral' as const),
      cRate:
        averageCRate <= 1 ? 'positive' : averageCRate >= 3 ? 'negative' : ('neutral' as const),
    };

    return {
      predictedCycles,
      remainingCycles,
      confidence,
      degradationRate: Math.round(adjustedRate * 100) / 100,
      factors,
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate energy density (standalone function)
 */
export function calculateEnergyDensity(
  params: EnergyDensityParams
): EnergyDensityResult {
  const sdk = new SolidStateBatterySDK();
  return sdk.calculateEnergyDensity(params);
}

/**
 * Estimate charging time (standalone function)
 */
export function estimateChargingTime(params: ChargingTimeParams): ChargingTimeResult {
  const sdk = new SolidStateBatterySDK();
  return sdk.estimateChargingTime(params);
}

/**
 * Simulate thermal behavior (standalone function)
 */
export function simulateThermal(
  params: ThermalSimulationParams
): ThermalSimulationResult {
  const sdk = new SolidStateBatterySDK();
  return sdk.simulateThermal(params);
}

/**
 * Validate battery performance (standalone function)
 */
export function validateBatteryPerformance(
  params: BatteryValidationParams
): ValidationResult {
  const sdk = new SolidStateBatterySDK();
  return sdk.validateBatteryPerformance(params);
}

/**
 * Calculate mass breakdown (standalone function)
 */
export function calculateMassBreakdown(
  totalMass: number,
  chemistry?: string
): MassBreakdown {
  const sdk = new SolidStateBatterySDK();
  return sdk.calculateMassBreakdown(totalMass, chemistry);
}

/**
 * Predict cycle life (standalone function)
 */
export function predictCycleLife(
  params: CycleLifePredictionParams
): CycleLifePredictionResult {
  const sdk = new SolidStateBatterySDK();
  return sdk.predictCycleLife(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { SolidStateBatterySDK };
export default SolidStateBatterySDK;
