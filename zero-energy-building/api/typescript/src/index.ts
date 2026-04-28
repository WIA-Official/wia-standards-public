/**
 * WIA Zero Energy Building SDK
 *
 * Comprehensive SDK for monitoring, analyzing, and certifying
 * net-zero energy buildings according to WIA standards.
 *
 * @module @wia/zero-energy-building
 */

import {
  ZEBConfig,
  BuildingMetadata,
  EnergyProduction,
  EnergyConsumption,
  EnergyBalance,
  CertificationAssessment,
  CertificationLevel,
  CertificationCriteria,
  OptimizationRecommendation,
  ZEBEvent,
  ZEBEventType,
  ZEBEventHandler,
  SolarPVSystem,
  WindTurbineSystem,
  HVACSystem,
  InsulationSpec,
  EnergySourceType,
  ConsumptionCategory
} from './types';

export * from './types';

/**
 * Main Zero Energy Building SDK Class
 */
export class WIAZeroEnergyBuilding {
  private config: ZEBConfig;
  private building: BuildingMetadata | null = null;
  private productionData: Map<string, EnergyProduction[]> = new Map();
  private consumptionData: Map<string, EnergyConsumption[]> = new Map();
  private eventHandlers: Map<ZEBEventType, Set<ZEBEventHandler>> = new Map();
  private monitoringInterval: NodeJS.Timeout | null = null;

  constructor(config: ZEBConfig) {
    this.config = {
      monitoringInterval: 60,
      enableRealTimeMonitoring: false,
      certificationStandard: 'WIA-ZEB-v1.0',
      units: 'metric',
      ...config
    };
  }

  /**
   * Initialize building metadata
   */
  public async initializeBuilding(metadata: BuildingMetadata): Promise<void> {
    this.building = metadata;

    if (this.config.enableRealTimeMonitoring) {
      this.startMonitoring();
    }

    this.emitEvent({
      type: ZEBEventType.SYSTEM_MAINTENANCE,
      timestamp: new Date(),
      severity: 'info',
      message: `Building ${metadata.name} initialized`,
      data: { buildingId: metadata.id }
    });
  }

  /**
   * Get current building metadata
   */
  public getBuildingMetadata(): BuildingMetadata | null {
    return this.building;
  }

  /**
   * Track energy production
   */
  public async trackProduction(production: EnergyProduction): Promise<void> {
    const key = `${production.sourceType}_${new Date().toISOString().split('T')[0]}`;

    if (!this.productionData.has(key)) {
      this.productionData.set(key, []);
    }

    this.productionData.get(key)!.push(production);

    // Check for production anomalies
    if (production.efficiency < 50) {
      this.emitEvent({
        type: ZEBEventType.PRODUCTION_ANOMALY,
        timestamp: new Date(),
        severity: 'warning',
        message: `Low efficiency detected: ${production.sourceType} at ${production.efficiency}%`,
        data: { production }
      });
    }
  }

  /**
   * Track energy consumption
   */
  public async trackConsumption(consumption: EnergyConsumption): Promise<void> {
    const key = `${consumption.category}_${new Date().toISOString().split('T')[0]}`;

    if (!this.consumptionData.has(key)) {
      this.consumptionData.set(key, []);
    }

    this.consumptionData.get(key)!.push(consumption);

    // Check for consumption spikes
    if (consumption.currentPower > consumption.peakDemand * 1.2) {
      this.emitEvent({
        type: ZEBEventType.CONSUMPTION_SPIKE,
        timestamp: new Date(),
        severity: 'warning',
        message: `Consumption spike detected: ${consumption.category}`,
        data: { consumption }
      });
    }
  }

  /**
   * Calculate energy balance for a period
   */
  public async calculateEnergyBalance(
    period: 'daily' | 'monthly' | 'yearly',
    date?: Date
  ): Promise<EnergyBalance> {
    const targetDate = date || new Date();

    // Aggregate production
    let totalProduction = 0;
    this.productionData.forEach((productions) => {
      productions.forEach(p => {
        if (this.isInPeriod(p.timestamp, period, targetDate)) {
          totalProduction += this.getProductionForPeriod(p, period);
        }
      });
    });

    // Aggregate consumption
    let totalConsumption = 0;
    this.consumptionData.forEach((consumptions) => {
      consumptions.forEach(c => {
        if (this.isInPeriod(c.timestamp, period, targetDate)) {
          totalConsumption += this.getConsumptionForPeriod(c, period);
        }
      });
    });

    const netBalance = totalProduction - totalConsumption;
    const gridExport = Math.max(0, netBalance);
    const gridImport = Math.max(0, -netBalance);

    const selfConsumptionRate = totalProduction > 0
      ? ((totalProduction - gridExport) / totalProduction) * 100
      : 0;

    const selfSufficiencyRate = totalConsumption > 0
      ? ((totalConsumption - gridImport) / totalConsumption) * 100
      : 0;

    const balance: EnergyBalance = {
      totalProduction,
      totalConsumption,
      netBalance,
      gridExport,
      gridImport,
      selfConsumptionRate,
      selfSufficiencyRate,
      period,
      timestamp: targetDate
    };

    // Emit events based on balance
    if (netBalance > 0) {
      this.emitEvent({
        type: ZEBEventType.ENERGY_SURPLUS,
        timestamp: new Date(),
        severity: 'info',
        message: `Energy surplus of ${netBalance.toFixed(2)} kWh for ${period}`,
        data: { balance }
      });
    } else if (netBalance < 0) {
      this.emitEvent({
        type: ZEBEventType.ENERGY_DEFICIT,
        timestamp: new Date(),
        severity: 'warning',
        message: `Energy deficit of ${Math.abs(netBalance).toFixed(2)} kWh for ${period}`,
        data: { balance }
      });
    }

    return balance;
  }

  /**
   * Assess certification level
   */
  public async assessCertification(
    criteria?: CertificationCriteria
  ): Promise<CertificationAssessment> {
    const defaultCriteria: CertificationCriteria = {
      minimumSelfSufficiency: 80,
      maximumGridImport: 1000,
      minimumRenewablePercentage: 90,
      requiredInsulationStandard: 30,
      hvacEfficiencyMinimum: 15,
      airLeakageMaximum: 3.0,
      requiredMonitoringPeriod: 12
    };

    const activeCriteria = criteria || defaultCriteria;
    const yearlyBalance = await this.calculateEnergyBalance('yearly');

    // Calculate scores
    let score = 0;
    const recommendations: string[] = [];

    // Self-sufficiency check (40 points)
    if (yearlyBalance.selfSufficiencyRate >= activeCriteria.minimumSelfSufficiency) {
      score += 40;
    } else {
      score += (yearlyBalance.selfSufficiencyRate / activeCriteria.minimumSelfSufficiency) * 40;
      recommendations.push('Increase renewable energy production or reduce consumption to meet self-sufficiency target');
    }

    // Grid import check (30 points)
    if (yearlyBalance.gridImport <= activeCriteria.maximumGridImport) {
      score += 30;
    } else {
      score += Math.max(0, 30 - (yearlyBalance.gridImport - activeCriteria.maximumGridImport) / 100);
      recommendations.push('Reduce grid dependency by increasing on-site energy production');
    }

    // Renewable percentage (30 points)
    const renewablePercentage = this.calculateRenewablePercentage();
    if (renewablePercentage >= activeCriteria.minimumRenewablePercentage) {
      score += 30;
    } else {
      score += (renewablePercentage / activeCriteria.minimumRenewablePercentage) * 30;
      recommendations.push('Increase renewable energy sources proportion');
    }

    // Determine certification level
    let level: CertificationLevel;
    if (score >= 95) level = CertificationLevel.PLATINUM;
    else if (score >= 85) level = CertificationLevel.GOLD;
    else if (score >= 70) level = CertificationLevel.SILVER;
    else if (score >= 50) level = CertificationLevel.BRONZE;
    else level = CertificationLevel.NOT_CERTIFIED;

    const meetsMinimumRequirements = score >= 50;

    // Calculate energy intensity and carbon footprint
    const floorArea = this.building?.size.floorArea || 1;
    const energyIntensity = yearlyBalance.totalConsumption / floorArea;
    const carbonFootprint = yearlyBalance.gridImport * 0.5; // Simplified calculation

    const assessment: CertificationAssessment = {
      level,
      score,
      meetsMinimumRequirements,
      selfSufficiencyRate: yearlyBalance.selfSufficiencyRate,
      renewablePercentage,
      energyIntensity,
      carbonFootprint,
      recommendations,
      assessmentDate: new Date(),
      validUntil: new Date(Date.now() + 365 * 24 * 60 * 60 * 1000) // 1 year
    };

    this.emitEvent({
      type: ZEBEventType.CERTIFICATION_RENEWAL,
      timestamp: new Date(),
      severity: 'info',
      message: `Certification assessed: ${level} (${score.toFixed(1)}/100)`,
      data: { assessment }
    });

    return assessment;
  }

  /**
   * Get optimization recommendations
   */
  public async getOptimizationRecommendations(): Promise<OptimizationRecommendation[]> {
    const recommendations: OptimizationRecommendation[] = [];
    const balance = await this.calculateEnergyBalance('yearly');

    // Solar expansion recommendation
    if (balance.selfSufficiencyRate < 90) {
      recommendations.push({
        category: 'Energy Production',
        priority: 'high',
        title: 'Expand Solar PV Capacity',
        description: 'Install additional solar panels to increase renewable energy generation',
        estimatedSavings: 5000,
        estimatedCost: 15000,
        paybackPeriod: 3,
        implementationComplexity: 'moderate',
        carbonReduction: 2500
      });
    }

    // Energy storage recommendation
    if (balance.gridExport > balance.totalProduction * 0.3) {
      recommendations.push({
        category: 'Energy Storage',
        priority: 'medium',
        title: 'Add Battery Storage System',
        description: 'Store excess solar energy for use during peak demand or nighttime',
        estimatedSavings: 3000,
        estimatedCost: 12000,
        paybackPeriod: 4,
        implementationComplexity: 'moderate',
        carbonReduction: 1500
      });
    }

    // Insulation upgrade
    recommendations.push({
      category: 'Building Envelope',
      priority: 'high',
      title: 'Upgrade Insulation',
      description: 'Improve wall and roof insulation to reduce heating/cooling loads',
      estimatedSavings: 2000,
      estimatedCost: 8000,
      paybackPeriod: 4,
      implementationComplexity: 'moderate',
      carbonReduction: 1000
    });

    // Smart controls
    recommendations.push({
      category: 'Building Automation',
      priority: 'medium',
      title: 'Install Smart Energy Management System',
      description: 'Optimize energy use with AI-powered building automation',
      estimatedSavings: 1500,
      estimatedCost: 5000,
      paybackPeriod: 3.3,
      implementationComplexity: 'simple',
      carbonReduction: 750
    });

    this.emitEvent({
      type: ZEBEventType.OPTIMIZATION_AVAILABLE,
      timestamp: new Date(),
      severity: 'info',
      message: `${recommendations.length} optimization recommendations generated`,
      data: { count: recommendations.length }
    });

    return recommendations;
  }

  /**
   * Register event handler
   */
  public on(eventType: ZEBEventType, handler: ZEBEventHandler): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, new Set());
    }
    this.eventHandlers.get(eventType)!.add(handler);
  }

  /**
   * Unregister event handler
   */
  public off(eventType: ZEBEventType, handler: ZEBEventHandler): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      handlers.delete(handler);
    }
  }

  /**
   * Start real-time monitoring
   */
  public startMonitoring(): void {
    if (this.monitoringInterval) {
      return; // Already monitoring
    }

    this.monitoringInterval = setInterval(async () => {
      await this.performMonitoringCycle();
    }, (this.config.monitoringInterval || 60) * 1000);
  }

  /**
   * Stop real-time monitoring
   */
  public stopMonitoring(): void {
    if (this.monitoringInterval) {
      clearInterval(this.monitoringInterval);
      this.monitoringInterval = null;
    }
  }

  /**
   * Clean up resources
   */
  public destroy(): void {
    this.stopMonitoring();
    this.eventHandlers.clear();
    this.productionData.clear();
    this.consumptionData.clear();
  }

  // Private helper methods

  private emitEvent(event: ZEBEvent): void {
    const handlers = this.eventHandlers.get(event.type);
    if (handlers) {
      handlers.forEach(handler => {
        try {
          handler(event);
        } catch (error) {
          console.error('Error in event handler:', error);
        }
      });
    }
  }

  private async performMonitoringCycle(): Promise<void> {
    // Perform periodic checks and calculations
    const balance = await this.calculateEnergyBalance('daily');

    // Additional monitoring logic can be added here
  }

  private isInPeriod(timestamp: Date, period: 'daily' | 'monthly' | 'yearly', targetDate: Date): boolean {
    const t = new Date(timestamp);
    const target = new Date(targetDate);

    switch (period) {
      case 'daily':
        return t.toDateString() === target.toDateString();
      case 'monthly':
        return t.getMonth() === target.getMonth() && t.getFullYear() === target.getFullYear();
      case 'yearly':
        return t.getFullYear() === target.getFullYear();
      default:
        return false;
    }
  }

  private getProductionForPeriod(production: EnergyProduction, period: 'daily' | 'monthly' | 'yearly'): number {
    switch (period) {
      case 'daily':
        return production.dailyProduction;
      case 'monthly':
        return production.monthlyProduction;
      case 'yearly':
        return production.yearlyProduction;
      default:
        return 0;
    }
  }

  private getConsumptionForPeriod(consumption: EnergyConsumption, period: 'daily' | 'monthly' | 'yearly'): number {
    switch (period) {
      case 'daily':
        return consumption.dailyConsumption;
      case 'monthly':
        return consumption.monthlyConsumption;
      case 'yearly':
        return consumption.yearlyConsumption;
      default:
        return 0;
    }
  }

  private calculateRenewablePercentage(): number {
    let totalRenewable = 0;
    let totalProduction = 0;

    this.productionData.forEach((productions) => {
      productions.forEach(p => {
        totalProduction += p.yearlyProduction;
        if (p.sourceType !== EnergySourceType.GRID) {
          totalRenewable += p.yearlyProduction;
        }
      });
    });

    return totalProduction > 0 ? (totalRenewable / totalProduction) * 100 : 0;
  }
}

/**
 * Factory function to create a new Zero Energy Building instance
 */
export function createZeroEnergyBuilding(config: ZEBConfig): WIAZeroEnergyBuilding {
  return new WIAZeroEnergyBuilding(config);
}

/**
 * Default export
 */
export default WIAZeroEnergyBuilding;
