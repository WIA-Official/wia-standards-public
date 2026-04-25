/**
 * WIA-TIME-007: Time Energy Source SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for time travel energy management including:
 * - Power requirement calculations
 * - Exotic matter generation
 * - Zero-point energy extraction
 * - Casimir effect harvesting
 * - Temporal flux capacitor management
 */

import {
  PowerRequirementParams,
  PowerRequirements,
  ExoticMatterConfig,
  ExoticMatterResult,
  ZeroPointEnergyParams,
  ZeroPointEnergyResult,
  CasimirConfig,
  CasimirResult,
  FluxCapacitorConfig,
  FluxCapacitor,
  FluxCapacitorBank,
  EnergyManifest,
  EnergyValidationParams,
  EnergyValidationResult,
  EnergySource,
  EnergyStorage,
  EnergySafetyCheck,
  ENERGY_CONSTANTS,
  EnergyErrorCode,
  EnergySystemError,
  TimeTravelMethod,
  EnergyComparison,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-TIME-007 Time Energy Source SDK
 */
export class TimeEnergySDK {
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
   * Calculate power requirements for temporal displacement
   *
   * @param params - Power requirement parameters
   * @returns Power requirements with breakdown
   */
  calculatePowerRequirements(params: PowerRequirementParams): PowerRequirements {
    const {
      mass,
      displacement,
      method,
      safetyMargin = 0.2,
      includeExoticMatter = true,
      includeFieldEnergy = true,
    } = params;

    // Validate inputs
    if (mass <= 0) {
      throw new EnergySystemError(
        EnergyErrorCode.INVALID_PARAMETERS,
        'Mass must be positive'
      );
    }

    const c = ENERGY_CONSTANTS.SPEED_OF_LIGHT;
    const G = ENERGY_CONSTANTS.GRAVITATIONAL_CONSTANT;

    // 1. Rest mass energy: E_rest = mc²
    const restMass = mass * c * c;

    // 2. Temporal displacement energy: E_temporal = E_rest × |Δt|
    const temporal = restMass * Math.abs(displacement);

    // 3. Exotic matter generation energy
    let exotic = 0;
    let exoticMatterRequired = 0;

    if (includeExoticMatter && (method === 'wormhole' || method === 'alcubierre')) {
      // Estimate exotic matter required based on method
      if (method === 'wormhole') {
        // For wormhole with 1m throat radius: m_exotic ≈ c⁴r/4G
        const throatRadius = 1.0; // meters
        exoticMatterRequired = (c ** 4 * throatRadius) / (4 * G);
        // Production efficiency factor ~10⁶
        exotic = exoticMatterRequired * c * c * 1e6;
      } else if (method === 'alcubierre') {
        // Alcubierre drive requires even more exotic matter
        const shipRadius = 10; // meters
        exoticMatterRequired = (c ** 4 * shipRadius) / G;
        exotic = exoticMatterRequired * c * c * 1e6;
      }
    }

    // 4. Field generation energy
    let field = 0;
    if (includeFieldEnergy) {
      const fieldRadius = 10; // meters
      const fieldVolume = (4 / 3) * Math.PI * fieldRadius ** 3;
      const fieldEnergyDensity = 1e25; // J/m³
      field = fieldVolume * fieldEnergyDensity * Math.log(1 + Math.abs(displacement));
    }

    // 5. Safety margin
    const safety = safetyMargin * (restMass + temporal + exotic + field);

    // Total energy
    const totalEnergy = restMass + temporal + exotic + field + safety;

    // Duration estimate (varies by method)
    let duration: number;
    switch (method) {
      case 'natural':
        duration = Math.abs(displacement); // Real-time
        break;
      case 'field':
        duration = 3600; // 1 hour
        break;
      case 'wormhole':
        duration = 60; // 1 minute
        break;
      case 'ctc':
        duration = 10; // 10 seconds
        break;
      case 'alcubierre':
        duration = 1; // 1 second
        break;
      default:
        duration = 3600;
    }

    const totalPower = totalEnergy / duration;

    // Feasibility assessment
    let feasibility: PowerRequirements['feasibility'];
    const worldAnnualEnergy = 6e20; // J
    const sunPowerPerSecond = 3.828e26; // W

    if (totalEnergy < worldAnnualEnergy) {
      feasibility = 'possible';
    } else if (totalEnergy < worldAnnualEnergy * 1e3) {
      feasibility = 'difficult';
    } else if (totalEnergy < sunPowerPerSecond * 86400) {
      feasibility = 'theoretical';
    } else {
      feasibility = 'impossible';
    }

    // Format energy
    const energyFormatted = this.formatEnergy(totalEnergy);

    // Generate comparisons
    const comparisons: EnergyComparison[] = [
      {
        source: 'World annual energy consumption',
        ratio: totalEnergy / worldAnnualEnergy,
        description: `${(totalEnergy / worldAnnualEnergy).toExponential(2)}× world annual`,
      },
      {
        source: 'Hiroshima atomic bomb',
        ratio: totalEnergy / 6.3e13,
        description: `${(totalEnergy / 6.3e13).toExponential(2)}× Hiroshima bomb`,
      },
      {
        source: 'Sun output per second',
        ratio: totalPower / sunPowerPerSecond,
        description: `${(totalPower / sunPowerPerSecond).toExponential(2)}× sun power`,
      },
      {
        source: 'Antimatter (1 kg)',
        ratio: totalEnergy / 9e16,
        description: `${(totalEnergy / 9e16).toFixed(2)} kg antimatter`,
      },
    ];

    return {
      totalEnergy,
      totalPower,
      duration,
      exoticMatterRequired,
      feasibility,
      breakdown: {
        restMass,
        temporal,
        exotic,
        field,
        safety,
      },
      energyFormatted,
      comparisons,
    };
  }

  /**
   * Generate exotic matter configuration
   *
   * @param config - Exotic matter configuration
   * @returns Exotic matter generation result
   */
  generateExoticMatter(config: ExoticMatterConfig): ExoticMatterResult {
    const { mass, density, stabilizationField, containmentType, productionMethod = 'casimir-cavity' } = config;

    // Validate inputs
    if (mass <= 0) {
      throw new EnergySystemError(
        EnergyErrorCode.INVALID_PARAMETERS,
        'Mass must be positive'
      );
    }

    if (density >= 0) {
      throw new EnergySystemError(
        EnergyErrorCode.INVALID_PARAMETERS,
        'Exotic matter density must be negative'
      );
    }

    // Calculate volume: V = m / ρ
    const volume = mass / Math.abs(density);

    // Calculate production rate based on method
    let productionRate: number;
    let energyPerKg: number;

    switch (productionMethod) {
      case 'casimir-cavity':
        productionRate = 1e-18; // kg/s
        energyPerKg = 1e20; // J/kg
        break;
      case 'vacuum-manipulation':
        productionRate = 1e-15; // kg/s
        energyPerKg = 1e22; // J/kg
        break;
      case 'quantum-squeezing':
        productionRate = 1e-12; // kg/s
        energyPerKg = 1e24; // J/kg
        break;
      default:
        productionRate = 1e-18;
        energyPerKg = 1e20;
    }

    // Total energy cost
    const energyCost = mass * energyPerKg;

    // Stability calculation (decreases with mass)
    const stability = Math.max(0.5, 1 - mass / 1e-6);

    // Expected lifetime (inversely proportional to mass)
    const lifetimeExpected = stability * 1e-3; // seconds

    // Generate unique ID
    const id = `EM-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    // Determine status
    let status: ExoticMatterResult['status'];
    if (stability > 0.9) {
      status = 'stable';
    } else if (stability > 0.7) {
      status = 'generating';
    } else if (stability > 0.5) {
      status = 'decaying';
    } else {
      status = 'failed';
    }

    return {
      id,
      mass,
      volume,
      density,
      stability,
      lifetimeExpected,
      energyCost,
      productionRate,
      status,
      created: new Date(),
      containmentField: {
        strength: stabilizationField,
        type: containmentType,
        stability: stability,
      },
    };
  }

  /**
   * Extract zero-point energy
   *
   * @param params - ZPE extraction parameters
   * @returns Zero-point energy extraction result
   */
  extractZeroPointEnergy(params: ZeroPointEnergyParams): ZeroPointEnergyResult {
    const { volume, frequency, efficiency, duration, qFactor = 1e9 } = params;

    // Validate inputs
    if (volume <= 0 || frequency <= 0 || efficiency <= 0 || efficiency > 1) {
      throw new EnergySystemError(
        EnergyErrorCode.INVALID_PARAMETERS,
        'Invalid ZPE extraction parameters'
      );
    }

    const h = ENERGY_CONSTANTS.REDUCED_PLANCK;
    const c = ENERGY_CONSTANTS.SPEED_OF_LIGHT;

    // Zero-point energy calculation
    // E_zpe = (ℏω/2) × V × ε × (modes)
    const omega = 2 * Math.PI * frequency;
    const modes = (8 * Math.PI * frequency ** 3 * volume) / c ** 3;
    const zpeSingleMode = (h * omega) / 2;

    // Total extractable energy (with efficiency)
    const total = zpeSingleMode * modes * efficiency * duration;

    // Energy density
    const energyDensity = total / volume;

    // Power calculation
    const power = total / duration;
    const rate = power; // Same as power for continuous extraction

    // Quantum efficiency (limited by Heisenberg uncertainty)
    const quantumEfficiency = Math.min(efficiency, 1e-3); // Realistic limit

    // Vacuum quality (affects extraction)
    const vacuumQuality = Math.min(1.0, qFactor / 1e9);

    // Status determination
    let status: ZeroPointEnergyResult['status'];
    const warnings: string[] = [];

    if (efficiency > 1e-3) {
      warnings.push('Efficiency exceeds theoretical limits');
      status = 'failed';
    } else if (energyDensity > 1e9) {
      warnings.push('Energy density may destabilize spacetime');
      status = 'failed';
    } else {
      status = 'extracting';
    }

    if (vacuumQuality < 0.9) {
      warnings.push('Vacuum quality below optimal threshold');
    }

    return {
      total,
      power,
      rate,
      energyDensity,
      quantumEfficiency,
      vacuumQuality,
      status,
      warnings,
    };
  }

  /**
   * Calculate Casimir effect energy
   *
   * @param config - Casimir configuration
   * @returns Casimir energy result
   */
  calculateCasimirEffect(config: CasimirConfig): CasimirResult {
    const { plateArea, separation, material, temperature, vacuumPressure } = config;

    // Validate inputs
    if (plateArea <= 0 || separation <= 0) {
      throw new EnergySystemError(
        EnergyErrorCode.INVALID_PARAMETERS,
        'Invalid Casimir configuration'
      );
    }

    const h = ENERGY_CONSTANTS.REDUCED_PLANCK;
    const c = ENERGY_CONSTANTS.SPEED_OF_LIGHT;

    // Casimir energy: E = -(π²ℏc A) / (720 d³)
    const energy = -(Math.PI ** 2 * h * c * plateArea) / (720 * separation ** 3);

    // Casimir force: F = (π²ℏc A) / (240 d⁴)
    const force = (Math.PI ** 2 * h * c * plateArea) / (240 * separation ** 4);

    // Energy density: ρ = -(π²ℏc) / (720 d⁴)
    const energyDensity = -(Math.PI ** 2 * h * c) / (720 * separation ** 4);

    // Work extractable (allow plates to approach from d₁ to d₂)
    const d2 = separation * 0.5; // Halve the distance
    const workExtractable =
      (Math.PI ** 2 * h * c * plateArea / 720) * (1 / d2 ** 3 - 1 / separation ** 3);

    // Practical power (assuming 1 Hz oscillation)
    const oscillationFrequency = 1; // Hz
    const power = Math.abs(workExtractable * oscillationFrequency);

    // Status checks
    let status: CasimirResult['status'];

    if (separation < 1e-9) {
      status = 'fault'; // Too close, quantum tunneling
    } else if (vacuumPressure > 1e-9) {
      status = 'fault'; // Vacuum not good enough
    } else if (temperature > 4) {
      status = 'calibrating'; // Temperature too high
    } else {
      status = 'operational';
    }

    return {
      energy,
      force,
      energyDensity,
      workExtractable,
      power,
      status,
    };
  }

  /**
   * Create temporal flux capacitor
   *
   * @param config - Flux capacitor configuration
   * @returns Flux capacitor instance
   */
  createFluxCapacitor(config: FluxCapacitorConfig): FluxCapacitor {
    const { capacity, chargeRate, voltage = 88, efficiency, class: capClass } = config;

    // Validate inputs
    if (capacity <= 0 || chargeRate <= 0 || efficiency <= 0 || efficiency > 1) {
      throw new EnergySystemError(
        EnergyErrorCode.INVALID_PARAMETERS,
        'Invalid flux capacitor configuration'
      );
    }

    if (capacity > ENERGY_CONSTANTS.MAX_SAFE_ENERGY) {
      throw new EnergySystemError(
        EnergyErrorCode.CAPACITOR_OVERLOAD,
        `Capacity exceeds maximum safe limit (${ENERGY_CONSTANTS.MAX_SAFE_ENERGY} J)`
      );
    }

    // Generate ID
    const id = `FC-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    // Initial state
    const currentCharge = 0;
    const chargePercent = 0;

    // Calculate charge time
    const timeToFull = capacity / (chargeRate * efficiency);

    // Operating temperature (cryogenic for superconductors)
    const temperature = 4; // Kelvin

    return {
      id,
      config,
      currentCharge,
      chargePercent,
      voltage,
      temperature,
      status: 'offline',
      timeToFull,
      timeToEmpty: 0,
      cycles: 0,
      health: 1.0,
      created: new Date(),
      updated: new Date(),
    };
  }

  /**
   * Validate energy system
   *
   * @param params - Validation parameters
   * @returns Validation result
   */
  validateEnergySystem(params: EnergyValidationParams): EnergyValidationResult {
    const {
      totalEnergy,
      requiredEnergy,
      sources,
      storage,
      safetyCheck = true,
      minStability = 0.95,
    } = params;

    const errors: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];
    const safetyChecks: EnergySafetyCheck[] = [];

    // Calculate available energy
    let availableEnergy = totalEnergy;
    for (const store of storage) {
      availableEnergy += store.stored;
    }

    const energyDelta = availableEnergy - requiredEnergy;

    // Energy availability check
    safetyChecks.push({
      name: 'Energy Availability',
      type: 'energy-level',
      status: energyDelta >= 0 ? 'pass' : 'fail',
      measuredValue: availableEnergy,
      expectedValue: requiredEnergy,
      threshold: requiredEnergy,
      thresholdType: 'minimum',
      description: 'Verify sufficient energy for operation',
      correctiveAction: 'Charge additional capacitors or activate backup sources',
      timestamp: new Date(),
    });

    if (energyDelta < 0) {
      errors.push(
        `Insufficient energy: need ${this.formatEnergy(requiredEnergy)}, have ${this.formatEnergy(availableEnergy)}`
      );
    } else if (energyDelta < requiredEnergy * 0.1) {
      warnings.push('Energy margin is low (<10%). Consider increasing reserves.');
    }

    // Maximum energy check
    safetyChecks.push({
      name: 'Maximum Safe Energy',
      type: 'energy-level',
      status: totalEnergy <= ENERGY_CONSTANTS.MAX_SAFE_ENERGY ? 'pass' : 'fail',
      measuredValue: totalEnergy,
      expectedValue: ENERGY_CONSTANTS.MAX_SAFE_ENERGY,
      threshold: ENERGY_CONSTANTS.MAX_SAFE_ENERGY,
      thresholdType: 'maximum',
      description: 'Ensure energy within safe limits',
      correctiveAction: 'Distribute energy across multiple storage systems',
      timestamp: new Date(),
    });

    if (totalEnergy > ENERGY_CONSTANTS.MAX_SAFE_ENERGY) {
      errors.push(`Total energy exceeds safe limit (${this.formatEnergy(ENERGY_CONSTANTS.MAX_SAFE_ENERGY)})`);
    }

    // Source stability checks
    if (safetyCheck) {
      for (const source of sources) {
        if (source.status === 'fault') {
          errors.push(`Energy source ${source.type} is in fault state`);
        } else if (source.status === 'depleted') {
          warnings.push(`Energy source ${source.type} is depleted`);
        }

        if (source.efficiency < minStability) {
          warnings.push(`Source ${source.type} efficiency (${source.efficiency}) below minimum (${minStability})`);
        }
      }

      // Storage safety checks
      for (const store of storage) {
        if (store.safetyStatus === 'critical') {
          errors.push(`Storage ${store.id} has critical safety status`);
        } else if (store.safetyStatus === 'warning') {
          warnings.push(`Storage ${store.id} has safety warning`);
        }
      }
    }

    // Risk assessment
    let risk: EnergyValidationResult['risk'];
    if (errors.length > 0) {
      risk = 'extreme';
    } else if (warnings.length > 2) {
      risk = 'high';
    } else if (warnings.length > 0) {
      risk = 'medium';
    } else {
      risk = 'low';
    }

    // Generate recommendations
    if (energyDelta < requiredEnergy * 0.2) {
      recommendations.push('Increase energy reserves to at least 20% above requirement');
    }

    if (sources.filter((s) => s.status === 'active').length < 2) {
      recommendations.push('Activate redundant energy sources for reliability');
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      availableEnergy,
      energyDelta,
      safetyChecks,
      recommendations,
      risk,
      timestamp: new Date(),
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Format energy in human-readable form
   */
  private formatEnergy(energy: number): string {
    if (energy < 1e3) return `${energy.toFixed(2)} J`;
    if (energy < 1e6) return `${(energy / 1e3).toFixed(2)} kJ`;
    if (energy < 1e9) return `${(energy / 1e6).toFixed(2)} MJ`;
    if (energy < 1e12) return `${(energy / 1e9).toFixed(2)} GJ`;
    if (energy < 1e15) return `${(energy / 1e12).toFixed(2)} TJ`;
    if (energy < 1e18) return `${(energy / 1e15).toFixed(2)} PJ`;
    if (energy < 1e21) return `${(energy / 1e18).toFixed(2)} EJ`;
    return `${energy.toExponential(2)} J`;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate power requirements (standalone function)
 */
export function calculatePowerRequirements(
  params: PowerRequirementParams
): PowerRequirements {
  const sdk = new TimeEnergySDK();
  return sdk.calculatePowerRequirements(params);
}

/**
 * Generate exotic matter (standalone function)
 */
export function generateExoticMatter(config: ExoticMatterConfig): ExoticMatterResult {
  const sdk = new TimeEnergySDK();
  return sdk.generateExoticMatter(config);
}

/**
 * Extract zero-point energy (standalone function)
 */
export function extractZeroPointEnergy(
  params: ZeroPointEnergyParams
): ZeroPointEnergyResult {
  const sdk = new TimeEnergySDK();
  return sdk.extractZeroPointEnergy(params);
}

/**
 * Calculate Casimir effect (standalone function)
 */
export function calculateCasimirEffect(config: CasimirConfig): CasimirResult {
  const sdk = new TimeEnergySDK();
  return sdk.calculateCasimirEffect(config);
}

/**
 * Create flux capacitor (standalone function)
 */
export function createFluxCapacitor(config: FluxCapacitorConfig): FluxCapacitor {
  const sdk = new TimeEnergySDK();
  return sdk.createFluxCapacitor(config);
}

/**
 * Validate energy system (standalone function)
 */
export function validateEnergySystem(
  params: EnergyValidationParams
): EnergyValidationResult {
  const sdk = new TimeEnergySDK();
  return sdk.validateEnergySystem(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { TimeEnergySDK };
export default TimeEnergySDK;
