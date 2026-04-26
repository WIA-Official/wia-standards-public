/**
 * WIA-AUTO-005: EV Charging SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for EV charging including:
 * - Charging time and cost calculations
 * - Session validation and compatibility checks
 * - Smart charging and load management
 * - V2G transaction handling
 * - OCPP protocol support
 */

import {
  ChargingTimeRequest,
  ChargingTimeResponse,
  ChargingCostRequest,
  ChargingCostResponse,
  SessionValidation,
  ValidationResult,
  ChargingSchedule,
  ChargingPeriod,
  TouRate,
  V2GConfig,
  V2GTransaction,
  LoadBalancing,
  ConnectorType,
  CHARGING_CONSTANTS,
  CONNECTOR_STANDARDS,
  ChargingErrorCode,
  ChargingError,
  CompatibilityCheck,
  VehicleInfo,
  ChargingStation,
  SafetyCheck,
  MonitoringData,
  BatteryChemistry,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUTO-005 EV Charging SDK
 */
export class EVChargingSDK {
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
   * Calculate charging time
   *
   * @param params - Charging time request parameters
   * @returns Charging time estimation
   */
  calculateChargingTime(params: ChargingTimeRequest): ChargingTimeResponse {
    const {
      batteryCapacity,
      currentSOC,
      targetSOC,
      chargingPower,
      efficiency = CHARGING_CONSTANTS.DEFAULT_EFFICIENCY,
      useChargingCurve = false,
      batteryChemistry,
    } = params;

    // Validate inputs
    if (batteryCapacity <= 0) {
      throw new ChargingError(
        ChargingErrorCode.INVALID_PARAMETERS,
        'Battery capacity must be positive'
      );
    }

    if (currentSOC < 0 || currentSOC > 1 || targetSOC < 0 || targetSOC > 1) {
      throw new ChargingError(
        ChargingErrorCode.INVALID_PARAMETERS,
        'SOC values must be between 0 and 1'
      );
    }

    if (targetSOC <= currentSOC) {
      throw new ChargingError(
        ChargingErrorCode.INVALID_PARAMETERS,
        'Target SOC must be greater than current SOC'
      );
    }

    if (chargingPower <= 0) {
      throw new ChargingError(
        ChargingErrorCode.INVALID_PARAMETERS,
        'Charging power must be positive'
      );
    }

    // Calculate energy needed
    const energyDelivered = batteryCapacity * (targetSOC - currentSOC);

    let hours: number;
    let averagePower: number;

    if (useChargingCurve && targetSOC > 0.5) {
      // Use charging curve taper for more accurate estimation
      const result = this.calculateWithChargingCurve(
        batteryCapacity,
        currentSOC,
        targetSOC,
        chargingPower,
        efficiency,
        batteryChemistry
      );
      hours = result.hours;
      averagePower = result.averagePower;
    } else {
      // Simple linear calculation
      averagePower = chargingPower * efficiency;
      hours = energyDelivered / averagePower;
    }

    const minutes = hours * 60;

    return {
      hours,
      minutes,
      energyDelivered,
      finalSOC: targetSOC,
      averagePower,
    };
  }

  /**
   * Calculate charging cost
   *
   * @param params - Charging cost request parameters
   * @returns Cost breakdown
   */
  calculateChargingCost(params: ChargingCostRequest): ChargingCostResponse {
    const {
      energy,
      pricing,
      duration = 0,
      idleTime = 0,
      parkingDuration = 0,
    } = params;

    const breakdown: { component: string; cost: number; unit?: string }[] = [];

    // Energy cost
    const energyCost = pricing.energyPrice ? energy * pricing.energyPrice : 0;
    if (energyCost > 0) {
      breakdown.push({
        component: 'Energy',
        cost: energyCost,
        unit: `${energy.toFixed(2)} kWh @ $${pricing.energyPrice?.toFixed(2)}/kWh`,
      });
    }

    // Time-based cost
    const timeCost = pricing.timePrice ? duration * pricing.timePrice : 0;
    if (timeCost > 0) {
      breakdown.push({
        component: 'Time',
        cost: timeCost,
        unit: `${duration} min @ $${pricing.timePrice?.toFixed(2)}/min`,
      });
    }

    // Session fee
    const sessionFee = pricing.sessionFee || 0;
    if (sessionFee > 0) {
      breakdown.push({
        component: 'Session Fee',
        cost: sessionFee,
      });
    }

    // Idle fee
    const idleFee = pricing.idleFee && idleTime > CHARGING_CONSTANTS.IDLE_GRACE_PERIOD
      ? (idleTime - CHARGING_CONSTANTS.IDLE_GRACE_PERIOD) * pricing.idleFee
      : 0;
    if (idleFee > 0) {
      breakdown.push({
        component: 'Idle Fee',
        cost: idleFee,
        unit: `${idleTime - CHARGING_CONSTANTS.IDLE_GRACE_PERIOD} min @ $${pricing.idleFee?.toFixed(2)}/min`,
      });
    }

    // Parking fee
    const parkingFee = pricing.parkingFee ? parkingDuration * pricing.parkingFee : 0;
    if (parkingFee > 0) {
      breakdown.push({
        component: 'Parking',
        cost: parkingFee,
        unit: `${parkingDuration} hr @ $${pricing.parkingFee?.toFixed(2)}/hr`,
      });
    }

    const totalCost = energyCost + timeCost + sessionFee + idleFee + parkingFee;

    return {
      energyCost,
      timeCost: timeCost > 0 ? timeCost : undefined,
      sessionFee: sessionFee > 0 ? sessionFee : undefined,
      idleFee: idleFee > 0 ? idleFee : undefined,
      parkingFee: parkingFee > 0 ? parkingFee : undefined,
      totalCost,
      breakdown,
    };
  }

  /**
   * Validate charging session
   *
   * @param validation - Session validation parameters
   * @returns Validation result
   */
  validateChargingSession(validation: SessionValidation): ValidationResult {
    const {
      connectorType,
      vehicleType,
      maxPower,
      batteryCapacity,
      currentSOC,
      vehicleConnectors,
    } = validation;

    const errors: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];

    // Check connector compatibility
    const connectorCompatible = vehicleConnectors
      ? vehicleConnectors.includes(connectorType)
      : true;

    const compatibility: CompatibilityCheck = {
      connector: connectorCompatible,
      power: true,
      communication: true,
      overall: true,
    };

    if (!connectorCompatible) {
      errors.push(`Connector type ${connectorType} not compatible with vehicle`);
      compatibility.overall = false;
    }

    // Get connector standard
    const connectorStandard = CONNECTOR_STANDARDS[connectorType];
    if (!connectorStandard) {
      errors.push(`Unknown connector type: ${connectorType}`);
      compatibility.connector = false;
      compatibility.overall = false;
    }

    // Check power compatibility
    if (connectorStandard && maxPower > connectorStandard.maxPower) {
      warnings.push(
        `Requested power (${maxPower} kW) exceeds connector maximum (${connectorStandard.maxPower} kW). Will charge at ${connectorStandard.maxPower} kW.`
      );
      compatibility.power = false;
    }

    // Estimate charging time if SOC provided
    let estimatedTime: ChargingTimeResponse | undefined;
    if (currentSOC !== undefined) {
      const targetSOC = vehicleType === 'BEV' ? 0.8 : 0.9;

      try {
        estimatedTime = this.calculateChargingTime({
          batteryCapacity,
          currentSOC,
          targetSOC,
          chargingPower: Math.min(
            maxPower,
            connectorStandard?.maxPower || maxPower
          ),
          useChargingCurve: true,
        });

        // Add recommendations based on charging time
        if (estimatedTime.hours < 0.5) {
          recommendations.push('Fast charging session. Consider battery health.');
        } else if (estimatedTime.hours > 8) {
          recommendations.push('Long charging session. Consider higher power charger.');
        }
      } catch (error) {
        warnings.push('Could not estimate charging time');
      }
    }

    // BEV-specific recommendations
    if (vehicleType === 'BEV') {
      recommendations.push('For battery longevity, charge to 80% for daily use.');
    }

    // PHEV-specific recommendations
    if (vehicleType === 'PHEV') {
      recommendations.push('Fully charge PHEV battery for maximum electric range.');
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      compatibility,
      estimatedTime,
      recommendations,
    };
  }

  /**
   * Create smart charging schedule
   *
   * @param vehicleId - Vehicle identifier
   * @param departureTime - Target departure time
   * @param currentSOC - Current state of charge (0-1)
   * @param targetSOC - Target state of charge (0-1)
   * @param batteryCapacity - Battery capacity in kWh
   * @param maxPower - Maximum charging power in kW
   * @param touRates - Time-of-use rate schedule
   * @returns Optimized charging schedule
   */
  createSmartChargingSchedule(
    vehicleId: string,
    departureTime: Date,
    currentSOC: number,
    targetSOC: number,
    batteryCapacity: number,
    maxPower: number,
    touRates?: TouRate[]
  ): ChargingSchedule {
    const energyNeeded = batteryCapacity * (targetSOC - currentSOC);
    const now = new Date();

    // If no TOU rates, create simple schedule
    if (!touRates || touRates.length === 0) {
      const hours = energyNeeded / (maxPower * CHARGING_CONSTANTS.DEFAULT_EFFICIENCY);
      const startTime = new Date(departureTime.getTime() - hours * 3600000);

      const period: ChargingPeriod = {
        start: startTime > now ? startTime : now,
        end: departureTime,
        power: maxPower,
        energy: energyNeeded,
        cost: 0, // Unknown without pricing
        rate: 0,
      };

      return {
        scheduleId: `SCH-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        vehicleId,
        departureTime,
        currentSOC,
        targetSOC,
        maxPower,
        chargingPeriods: [period],
        estimatedCost: 0,
        estimatedEnergy: energyNeeded,
      };
    }

    // Optimize for TOU rates
    const periods = this.optimizeChargingSchedule(
      now,
      departureTime,
      energyNeeded,
      maxPower,
      touRates
    );

    const estimatedCost = periods.reduce((sum, p) => sum + p.cost, 0);
    const estimatedEnergy = periods.reduce((sum, p) => sum + p.energy, 0);

    return {
      scheduleId: `SCH-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      vehicleId,
      departureTime,
      currentSOC,
      targetSOC,
      maxPower,
      touRates,
      chargingPeriods: periods,
      estimatedCost,
      estimatedEnergy,
    };
  }

  /**
   * Calculate V2G transaction revenue/cost
   *
   * @param config - V2G configuration
   * @param energyDischarged - Energy discharged from vehicle (kWh)
   * @param energyCharged - Energy charged to vehicle (kWh)
   * @param sellPrice - Price for selling energy ($/kWh)
   * @param buyPrice - Price for buying energy ($/kWh)
   * @returns V2G transaction details
   */
  calculateV2GTransaction(
    config: V2GConfig,
    energyDischarged: number,
    energyCharged: number,
    sellPrice: number,
    buyPrice: number
  ): V2GTransaction {
    const netEnergy = energyCharged - energyDischarged;
    const dischargeRevenue = energyDischarged * sellPrice;
    const chargeCost = energyCharged * buyPrice;
    const degradationCost = energyDischarged * config.degradationRate;

    const netRevenue = dischargeRevenue - chargeCost - degradationCost;

    // Calculate battery cycles consumed
    const cyclesConsumed = energyDischarged / 100; // Assuming 100 kWh = 1 full cycle

    return {
      transactionId: `V2G-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      sessionId: '',
      mode: config.mode,
      energyDischarged,
      energyCharged,
      netEnergy,
      dischargeRevenue,
      chargeCost,
      degradationCost,
      netRevenue,
      cyclesConsumed,
      timestamp: new Date(),
    };
  }

  /**
   * Calculate load balancing allocation
   *
   * @param totalPower - Total available power (kW)
   * @param buildingLoad - Current building load (kW)
   * @param activeSessions - Number of active charging sessions
   * @returns Load balancing configuration
   */
  calculateLoadBalancing(
    totalPower: number,
    buildingLoad: number,
    activeSessions: number
  ): LoadBalancing {
    const availablePower = totalPower - buildingLoad;

    if (availablePower <= 0 || activeSessions <= 0) {
      return {
        totalPower,
        buildingLoad,
        activeSessions,
        powerPerSession: 0,
      };
    }

    const powerPerSession = availablePower / activeSessions;

    return {
      totalPower,
      buildingLoad,
      activeSessions,
      powerPerSession,
    };
  }

  /**
   * Perform safety checks
   *
   * @param monitoringData - Current monitoring data
   * @returns Array of safety check results
   */
  performSafetyChecks(monitoringData: MonitoringData): SafetyCheck[] {
    const checks: SafetyCheck[] = [];
    const now = new Date();

    // Overvoltage check
    const maxVoltage = CHARGING_CONSTANTS.DC_VOLTAGE_MAX;
    checks.push({
      type: 'overvoltage',
      status: monitoringData.voltage <= maxVoltage ? 'pass' : 'fail',
      value: monitoringData.voltage,
      threshold: maxVoltage,
      message:
        monitoringData.voltage <= maxVoltage
          ? 'Voltage within safe limits'
          : `Overvoltage detected: ${monitoringData.voltage}V exceeds ${maxVoltage}V`,
      timestamp: now,
    });

    // Undervoltage check
    const minVoltage = CHARGING_CONSTANTS.DC_VOLTAGE_MIN * 0.9;
    checks.push({
      type: 'undervoltage',
      status: monitoringData.voltage >= minVoltage ? 'pass' : 'fail',
      value: monitoringData.voltage,
      threshold: minVoltage,
      message:
        monitoringData.voltage >= minVoltage
          ? 'Voltage within safe limits'
          : `Undervoltage detected: ${monitoringData.voltage}V below ${minVoltage}V`,
      timestamp: now,
    });

    // Overcurrent check
    const maxCurrent = CHARGING_CONSTANTS.DC_CURRENT_MAX;
    checks.push({
      type: 'overcurrent',
      status: monitoringData.current <= maxCurrent ? 'pass' : 'fail',
      value: monitoringData.current,
      threshold: maxCurrent,
      message:
        monitoringData.current <= maxCurrent
          ? 'Current within safe limits'
          : `Overcurrent detected: ${monitoringData.current}A exceeds ${maxCurrent}A`,
      timestamp: now,
    });

    // Temperature check
    const maxTemp = CHARGING_CONSTANTS.MAX_CABLE_TEMP;
    const warnTemp = maxTemp * 0.9;
    checks.push({
      type: 'overtemperature',
      status:
        monitoringData.temperature <= warnTemp
          ? 'pass'
          : monitoringData.temperature <= maxTemp
          ? 'warning'
          : 'fail',
      value: monitoringData.temperature,
      threshold: maxTemp,
      message:
        monitoringData.temperature <= warnTemp
          ? 'Temperature within safe limits'
          : monitoringData.temperature <= maxTemp
          ? `High temperature warning: ${monitoringData.temperature}°C`
          : `Overtemperature: ${monitoringData.temperature}°C exceeds ${maxTemp}°C`,
      timestamp: now,
    });

    return checks;
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Calculate charging time with charging curve consideration
   */
  private calculateWithChargingCurve(
    batteryCapacity: number,
    currentSOC: number,
    targetSOC: number,
    maxPower: number,
    efficiency: number,
    chemistry?: BatteryChemistry
  ): { hours: number; averagePower: number } {
    // Simplified charging curve model
    // Real implementation would use actual vehicle charging curves

    const energyNeeded = batteryCapacity * (targetSOC - currentSOC);
    let totalTime = 0;
    let totalEnergy = 0;
    let soc = currentSOC;
    const socStep = 0.01; // 1% increments

    while (soc < targetSOC) {
      // Power tapers after 50% SOC, especially after 80%
      let powerFactor = 1.0;

      if (soc > 0.8) {
        powerFactor = 0.5 - (soc - 0.8) * 2; // Rapid taper after 80%
      } else if (soc > 0.5) {
        powerFactor = 1.0 - (soc - 0.5) * 0.5; // Gradual taper 50-80%
      }

      // LFP batteries can maintain higher power longer
      if (chemistry === 'LFP' && soc < 0.9) {
        powerFactor = Math.max(powerFactor, 0.8);
      }

      const currentPower = maxPower * powerFactor * efficiency;
      const energyStep = batteryCapacity * socStep;
      const timeStep = energyStep / currentPower;

      totalTime += timeStep;
      totalEnergy += energyStep;
      soc += socStep;

      if (soc >= targetSOC) break;
    }

    const averagePower = totalEnergy / totalTime;

    return {
      hours: totalTime,
      averagePower,
    };
  }

  /**
   * Optimize charging schedule based on TOU rates
   */
  private optimizeChargingSchedule(
    startTime: Date,
    endTime: Date,
    energyNeeded: number,
    maxPower: number,
    touRates: TouRate[]
  ): ChargingPeriod[] {
    // Sort rates by price (cheapest first)
    const sortedRates = [...touRates].sort((a, b) => a.price - b.price);

    const periods: ChargingPeriod[] = [];
    let remainingEnergy = energyNeeded;

    for (const rate of sortedRates) {
      if (remainingEnergy <= 0) break;

      // Parse time strings (simplified)
      const [startHour, startMin] = rate.start.split(':').map(Number);
      const [endHour, endMin] = rate.end.split(':').map(Number);

      // Create period start/end times
      const periodStart = new Date(startTime);
      periodStart.setHours(startHour, startMin, 0, 0);

      const periodEnd = new Date(startTime);
      periodEnd.setHours(endHour, endMin, 0, 0);

      // Skip if period is outside available time window
      if (periodEnd > endTime || periodStart < startTime) {
        continue;
      }

      const durationHours = (periodEnd.getTime() - periodStart.getTime()) / 3600000;
      const maxEnergyInPeriod = maxPower * durationHours * CHARGING_CONSTANTS.DEFAULT_EFFICIENCY;
      const energyInPeriod = Math.min(remainingEnergy, maxEnergyInPeriod);

      if (energyInPeriod > 0) {
        periods.push({
          start: periodStart,
          end: new Date(periodStart.getTime() + (energyInPeriod / maxPower / CHARGING_CONSTANTS.DEFAULT_EFFICIENCY) * 3600000),
          power: maxPower,
          energy: energyInPeriod,
          cost: energyInPeriod * rate.price,
          rate: rate.price,
        });

        remainingEnergy -= energyInPeriod;
      }
    }

    // Sort periods by start time
    return periods.sort((a, b) => new Date(a.start).getTime() - new Date(b.start).getTime());
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate charging time (standalone function)
 */
export function calculateChargingTime(
  params: ChargingTimeRequest
): ChargingTimeResponse {
  const sdk = new EVChargingSDK();
  return sdk.calculateChargingTime(params);
}

/**
 * Calculate charging cost (standalone function)
 */
export function calculateChargingCost(
  params: ChargingCostRequest
): ChargingCostResponse {
  const sdk = new EVChargingSDK();
  return sdk.calculateChargingCost(params);
}

/**
 * Validate charging session (standalone function)
 */
export function validateChargingSession(
  validation: SessionValidation
): ValidationResult {
  const sdk = new EVChargingSDK();
  return sdk.validateChargingSession(validation);
}

/**
 * Create smart charging schedule (standalone function)
 */
export function createSmartChargingSchedule(
  vehicleId: string,
  departureTime: Date,
  currentSOC: number,
  targetSOC: number,
  batteryCapacity: number,
  maxPower: number,
  touRates?: TouRate[]
): ChargingSchedule {
  const sdk = new EVChargingSDK();
  return sdk.createSmartChargingSchedule(
    vehicleId,
    departureTime,
    currentSOC,
    targetSOC,
    batteryCapacity,
    maxPower,
    touRates
  );
}

/**
 * Get connector information
 */
export function getConnectorInfo(type: ConnectorType) {
  return CONNECTOR_STANDARDS[type];
}

/**
 * Get all connector standards
 */
export function getAllConnectors() {
  return CONNECTOR_STANDARDS;
}

/**
 * Get charging constants
 */
export function getChargingConstants() {
  return CHARGING_CONSTANTS;
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { EVChargingSDK };
export default EVChargingSDK;
