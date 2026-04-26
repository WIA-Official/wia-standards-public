/**
 * WIA-COMM-013: Data Center SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Data Center Infrastructure Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for data center infrastructure including:
 * - Tier classification and compliance validation
 * - Power capacity planning
 * - Cooling system design
 * - PUE calculations
 * - Rack layout optimization
 * - Cost estimation
 */

import {
  TierLevel,
  RedundancyConfig,
  TierRequirements,
  TierCompliance,
  PowerCapacityParams,
  PowerCapacityResult,
  PUEParams,
  PUEResult,
  CoolingCapacityParams,
  CoolingCapacityResult,
  RackLayoutParams,
  RackLayoutResult,
  CapExBreakdown,
  OpExBreakdown,
  TCOAnalysis,
  DATACENTER_CONSTANTS,
  TIER_STANDARDS,
  DCErrorCode,
  DataCenterError,
  CoolingType,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMM-013 Data Center SDK
 */
export class DataCenterSDK {
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
   * Calculate PUE (Power Usage Effectiveness)
   *
   * @param params - PUE calculation parameters
   * @returns PUE calculation results
   */
  calculatePUE(params: PUEParams): PUEResult {
    const { itLoad, coolingLoad, lightingLoad, upsLosses, otherLoads = 0 } = params;

    // Validate inputs
    if (itLoad <= 0) {
      throw new DataCenterError(
        DCErrorCode.INVALID_PARAMETERS,
        'IT load must be greater than 0'
      );
    }

    // Calculate total facility power
    const totalFacilityPower =
      itLoad + coolingLoad + lightingLoad + upsLosses + otherLoads;

    // Calculate PUE
    const pue = totalFacilityPower / itLoad;

    // Calculate DCiE (Data Center infrastructure Efficiency)
    const dcie = (100 / pue);

    // Determine rating
    let rating: 'excellent' | 'good' | 'fair' | 'poor';
    if (pue < 1.2) {
      rating = 'excellent';
    } else if (pue < 1.5) {
      rating = 'good';
    } else if (pue < 1.8) {
      rating = 'fair';
    } else {
      rating = 'poor';
    }

    // Breakdown by component (percentages)
    const breakdown = {
      it: (itLoad / totalFacilityPower) * 100,
      cooling: (coolingLoad / totalFacilityPower) * 100,
      lighting: (lightingLoad / totalFacilityPower) * 100,
      ups: (upsLosses / totalFacilityPower) * 100,
      other: (otherLoads / totalFacilityPower) * 100,
    };

    return {
      totalFacilityPower,
      itEquipmentPower: itLoad,
      pue: parseFloat(pue.toFixed(3)),
      dcie: parseFloat(dcie.toFixed(1)),
      rating,
      breakdown,
    };
  }

  /**
   * Validate tier compliance
   *
   * @param requirements - Tier requirements to validate
   * @returns Compliance validation result
   */
  validateTierCompliance(requirements: Partial<TierRequirements>): TierCompliance {
    const tier = requirements.tier || 'III';
    const standard = TIER_STANDARDS[tier];

    const checks: TierCompliance['checks'] = [];
    let passedChecks = 0;

    // Check power paths
    const powerPathCheck = {
      name: 'Power Paths',
      passed: (requirements.powerPaths || 0) >= standard.powerPaths,
      required: `≥${standard.powerPaths}`,
      actual: `${requirements.powerPaths || 0}`,
    };
    checks.push(powerPathCheck);
    if (powerPathCheck.passed) passedChecks++;

    // Check redundancy
    const redundancyValid = this.isRedundancyValid(
      requirements.redundancy || 'N',
      standard.redundancy
    );
    const redundancyCheck = {
      name: 'Redundancy',
      passed: redundancyValid,
      required: standard.redundancy,
      actual: requirements.redundancy || 'N',
    };
    checks.push(redundancyCheck);
    if (redundancyCheck.passed) passedChecks++;

    // Check concurrent maintenance (Tier III+)
    if (tier === 'III' || tier === 'IV') {
      const concurrentCheck = {
        name: 'Concurrent Maintenance',
        passed: requirements.concurrentMaintenance === true,
        required: 'Yes',
        actual: requirements.concurrentMaintenance ? 'Yes' : 'No',
      };
      checks.push(concurrentCheck);
      if (concurrentCheck.passed) passedChecks++;
    }

    // Check fault tolerance (Tier IV)
    if (tier === 'IV') {
      const faultTolerantCheck = {
        name: 'Fault Tolerant',
        passed: requirements.faultTolerant === true,
        required: 'Yes',
        actual: requirements.faultTolerant ? 'Yes' : 'No',
      };
      checks.push(faultTolerantCheck);
      if (faultTolerantCheck.passed) passedChecks++;
    }

    const isCompliant = checks.every((check) => check.passed);
    const score = (passedChecks / checks.length) * 100;

    const recommendations: string[] = [];
    if (!isCompliant) {
      checks.forEach((check) => {
        if (!check.passed) {
          recommendations.push(
            `Upgrade ${check.name}: ${check.actual} → ${check.required}`
          );
        }
      });
    }

    return {
      isCompliant,
      tier,
      checks,
      score: parseFloat(score.toFixed(1)),
      recommendations,
    };
  }

  /**
   * Calculate power capacity requirements
   *
   * @param params - Power capacity parameters
   * @returns Power capacity calculation results
   */
  calculatePowerCapacity(params: PowerCapacityParams): PowerCapacityResult {
    const {
      rackCount,
      avgPowerPerRack,
      redundancy,
      upsEfficiency,
      growthRate = 0.15,
      planningYears = 5,
    } = params;

    // Validate inputs
    if (rackCount <= 0 || avgPowerPerRack <= 0) {
      throw new DataCenterError(
        DCErrorCode.INVALID_PARAMETERS,
        'Rack count and power per rack must be greater than 0'
      );
    }

    if (upsEfficiency <= 0 || upsEfficiency > 1) {
      throw new DataCenterError(
        DCErrorCode.INVALID_PARAMETERS,
        'UPS efficiency must be between 0 and 1'
      );
    }

    // Calculate base IT load
    const itLoad = rackCount * avgPowerPerRack;

    // Calculate redundancy multiplier
    const redundancyMultiplier = this.getRedundancyMultiplier(redundancy);

    // Calculate total capacity with redundancy
    const totalCapacity = itLoad * redundancyMultiplier;

    // Calculate UPS capacity (kVA)
    // Assume power factor of 0.9
    const powerFactor = 0.9;
    const upsCapacity = totalCapacity / upsEfficiency / powerFactor;

    // Calculate generator capacity (typically 125% of total load)
    const generatorCapacity = totalCapacity * 1.25;

    // Calculate number of modules for N+1
    const upsModules = Math.ceil(totalCapacity / 500) + 1; // Assume 500kW modules
    const generators = Math.ceil(generatorCapacity / 1000) + 1; // Assume 1MW generators

    // Project future capacity
    const projectedCapacity = itLoad * Math.pow(1 + growthRate, planningYears);

    return {
      itLoad,
      totalCapacity,
      upsCapacity: parseFloat(upsCapacity.toFixed(1)),
      generatorCapacity: parseFloat(generatorCapacity.toFixed(1)),
      upsModules,
      generators,
      projectedCapacity: parseFloat(projectedCapacity.toFixed(1)),
    };
  }

  /**
   * Calculate cooling capacity requirements
   *
   * @param params - Cooling capacity parameters
   * @returns Cooling capacity calculation results
   */
  calculateCoolingCapacity(params: CoolingCapacityParams): CoolingCapacityResult {
    const { itLoad, targetPUE, redundancy } = params;

    // Validate inputs
    if (itLoad <= 0) {
      throw new DataCenterError(
        DCErrorCode.INVALID_PARAMETERS,
        'IT load must be greater than 0'
      );
    }

    if (targetPUE < 1.0) {
      throw new DataCenterError(
        DCErrorCode.INVALID_PARAMETERS,
        'PUE must be at least 1.0'
      );
    }

    // Calculate cooling load
    const coolingLoad = (targetPUE - 1) * itLoad;

    // Convert to tons (1 ton = 3.517 kW)
    const coolingTons = coolingLoad / DATACENTER_CONSTANTS.TON_TO_KW;

    // Calculate required airflow (CFM)
    const airflow = itLoad * DATACENTER_CONSTANTS.CFM_PER_KW;

    // Calculate number of cooling units (assume 30 tons per unit)
    const tonsPerUnit = 30;
    const coolingUnits = coolingTons / tonsPerUnit;

    // Apply redundancy
    const redundancyMultiplier = this.getRedundancyMultiplier(redundancy);
    const totalUnits = Math.ceil(coolingUnits * redundancyMultiplier);

    // Estimate COP based on cooling type
    const cop = this.estimateCOP(params.coolingType);

    return {
      coolingLoad: parseFloat(coolingLoad.toFixed(1)),
      coolingTons: parseFloat(coolingTons.toFixed(1)),
      airflow: parseFloat(airflow.toFixed(0)),
      coolingUnits: parseFloat(coolingUnits.toFixed(1)),
      totalUnits,
      cop,
    };
  }

  /**
   * Design rack layout
   *
   * @param params - Rack layout parameters
   * @returns Rack layout design results
   */
  designRackLayout(params: RackLayoutParams): RackLayoutResult {
    const {
      rackCount,
      avgPowerPerRack,
      floorArea,
      coldAisleWidth = 4,
      hotAisleWidth = 5,
    } = params;

    // Validate inputs
    if (rackCount <= 0 || floorArea <= 0) {
      throw new DataCenterError(
        DCErrorCode.INVALID_PARAMETERS,
        'Rack count and floor area must be greater than 0'
      );
    }

    // Calculate optimal layout
    const rows = Math.ceil(Math.sqrt(rackCount / 2));
    const racksPerRow = Math.ceil(rackCount / rows);

    // Calculate total IT load
    const totalITLoad = rackCount * avgPowerPerRack;

    // Calculate rack footprint (2ft x 4ft per rack)
    const rackFootprint = rackCount * 2 * 4;

    // Calculate floor utilization
    const floorUtilization = (rackFootprint / floorArea) * 100;

    // Calculate power density (kW per 1000 sq ft)
    const powerDensity = totalITLoad / (floorArea / 1000);

    // Determine cooling recommendation
    let coolingRecommendation: string;
    if (avgPowerPerRack <= 5) {
      coolingRecommendation = 'Standard air cooling (CRAC/CRAH)';
    } else if (avgPowerPerRack <= 15) {
      coolingRecommendation = 'Hot/cold aisle containment with CRAC/CRAH';
    } else if (avgPowerPerRack <= 25) {
      coolingRecommendation = 'In-row cooling units';
    } else {
      coolingRecommendation = 'Liquid cooling (direct-to-chip or immersion)';
    }

    return {
      rows,
      racksPerRow,
      totalITLoad,
      rackFootprint,
      floorUtilization: parseFloat(floorUtilization.toFixed(1)),
      powerDensity: parseFloat(powerDensity.toFixed(1)),
      coolingRecommendation,
    };
  }

  /**
   * Estimate data center costs
   *
   * @param itCapacity - IT capacity in kW
   * @param tier - Tier level
   * @param years - Analysis period in years
   * @returns TCO analysis
   */
  estimateCosts(
    itCapacity: number,
    tier: TierLevel = 'III',
    years: number = 5
  ): TCOAnalysis {
    // Validate inputs
    if (itCapacity <= 0) {
      throw new DataCenterError(
        DCErrorCode.INVALID_PARAMETERS,
        'IT capacity must be greater than 0'
      );
    }

    // CapEx cost per kW varies by tier
    const capexPerKW: Record<TierLevel, number> = {
      I: 2000,
      II: 3000,
      III: 4000,
      IV: 5000,
    };

    const costPerKW = capexPerKW[tier];
    const totalCapex = itCapacity * costPerKW;

    // CapEx breakdown
    const capex: CapExBreakdown = {
      electrical: totalCapex * 0.33,
      mechanical: totalCapex * 0.23,
      construction: totalCapex * 0.35,
      network: totalCapex * 0.06,
      security: totalCapex * 0.03,
      total: totalCapex,
      costPerKW,
    };

    // OpEx (annual)
    const opexPerKWYear = 1200;
    const totalOpex = itCapacity * opexPerKWYear;

    const opex: OpExBreakdown = {
      electricity: totalOpex * 0.7,
      staffing: totalOpex * 0.2,
      maintenance: totalOpex * 0.1,
      total: totalOpex,
      costPerKWYear: opexPerKWYear,
    };

    // Total cost over analysis period
    const totalCost = capex.total + opex.total * years;
    const costPerKW = totalCost / itCapacity;

    return {
      capex,
      opex,
      years,
      totalCost,
      costPerKW: parseFloat(costPerKW.toFixed(0)),
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Get redundancy multiplier
   */
  private getRedundancyMultiplier(redundancy: RedundancyConfig): number {
    const multipliers: Record<RedundancyConfig, number> = {
      N: 1.0,
      'N+1': 1.2,
      '2N': 2.0,
      '2N+1': 2.2,
    };
    return multipliers[redundancy];
  }

  /**
   * Check if redundancy configuration meets requirement
   */
  private isRedundancyValid(
    actual: RedundancyConfig,
    required: RedundancyConfig
  ): boolean {
    const hierarchy: RedundancyConfig[] = ['N', 'N+1', '2N', '2N+1'];
    const actualIndex = hierarchy.indexOf(actual);
    const requiredIndex = hierarchy.indexOf(required);
    return actualIndex >= requiredIndex;
  }

  /**
   * Estimate COP based on cooling type
   */
  private estimateCOP(coolingType: CoolingType): number {
    const copMap: Record<CoolingType, number> = {
      CRAC: 3.0,
      CRAH: 4.0,
      'in-row': 3.5,
      liquid: 5.0,
      immersion: 6.0,
    };
    return copMap[coolingType] || 3.5;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate PUE
 */
export function calculatePUE(params: PUEParams): PUEResult {
  const sdk = new DataCenterSDK();
  return sdk.calculatePUE(params);
}

/**
 * Validate tier compliance
 */
export function validateTierCompliance(
  requirements: Partial<TierRequirements>
): TierCompliance {
  const sdk = new DataCenterSDK();
  return sdk.validateTierCompliance(requirements);
}

/**
 * Calculate power capacity
 */
export function calculatePowerCapacity(
  params: PowerCapacityParams
): PowerCapacityResult {
  const sdk = new DataCenterSDK();
  return sdk.calculatePowerCapacity(params);
}

/**
 * Calculate cooling capacity
 */
export function calculateCoolingCapacity(
  params: CoolingCapacityParams
): CoolingCapacityResult {
  const sdk = new DataCenterSDK();
  return sdk.calculateCoolingCapacity(params);
}

/**
 * Design rack layout
 */
export function designRackLayout(params: RackLayoutParams): RackLayoutResult {
  const sdk = new DataCenterSDK();
  return sdk.designRackLayout(params);
}

/**
 * Estimate costs
 */
export function estimateCosts(
  itCapacity: number,
  tier: TierLevel = 'III',
  years: number = 5
): TCOAnalysis {
  const sdk = new DataCenterSDK();
  return sdk.estimateCosts(itCapacity, tier, years);
}

// ============================================================================
// Re-export Types
// ============================================================================

export * from './types';

// ============================================================================
// Default Export
// ============================================================================

export default DataCenterSDK;
