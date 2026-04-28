/**
 * WIA Water Quality Standard SDK
 * Comprehensive water quality monitoring and regulatory compliance
 *
 * @module @wia/water-quality
 * @version 1.0.0
 */

import {
  WaterSample,
  SampleLocation,
  SampleType,
  WaterSourceType,
  PhysicalParameters,
  ChemicalParameters,
  BiologicalParameters,
  MetalParameters,
  Contaminant,
  ContaminantCategory,
  ComplianceThreshold,
  ComplianceStandard,
  ComplianceResult,
  TreatmentProcess,
  TreatmentProcessType,
  MonitoringStation,
  WaterQualityEvent,
  WaterQualityEventType,
  WaterQualityEventHandler,
} from './types';

export * from './types';

/**
 * Main Water Quality Management Class
 */
export class WIAWaterQuality {
  private samples: Map<string, WaterSample> = new Map();
  private stations: Map<string, MonitoringStation> = new Map();
  private thresholds: Map<string, ComplianceThreshold> = new Map();
  private eventHandlers: Map<WaterQualityEventType, WaterQualityEventHandler[]> = new Map();

  constructor() {
    this.initializeDefaultThresholds();
  }

  // ============================================================================
  // Sample Recording
  // ============================================================================

  /**
   * Record a new water quality sample
   */
  recordSample(sample: WaterSample): void {
    // Validate sample data
    this.validateSample(sample);

    // Store sample
    this.samples.set(sample.sampleId, sample);

    // Emit event
    this.emit({
      type: 'sample-recorded',
      timestamp: new Date(),
      sampleId: sample.sampleId,
      stationId: sample.location.stationId,
      data: sample,
      severity: 'info',
    });

    // Check compliance automatically
    const complianceResults = this.checkCompliance(sample.sampleId);
    const violations = complianceResults.filter(r => !r.isCompliant);

    if (violations.length > 0) {
      this.emit({
        type: 'compliance-violation',
        timestamp: new Date(),
        sampleId: sample.sampleId,
        stationId: sample.location.stationId,
        data: { violations },
        severity: violations.some(v => v.severity === 'critical') ? 'critical' : 'warning',
      });
    }
  }

  /**
   * Validate sample data
   */
  private validateSample(sample: WaterSample): void {
    if (!sample.sampleId) {
      throw new Error('Sample ID is required');
    }
    if (this.samples.has(sample.sampleId)) {
      throw new Error(`Sample ${sample.sampleId} already exists`);
    }
    if (!sample.location || !sample.location.stationId) {
      throw new Error('Sample location and station ID are required');
    }
    if (!sample.physical || !sample.chemical) {
      throw new Error('Physical and chemical parameters are required');
    }
  }

  /**
   * Get sample by ID
   */
  getSample(sampleId: string): WaterSample | undefined {
    return this.samples.get(sampleId);
  }

  /**
   * Get all samples for a station
   */
  getSamplesByStation(stationId: string): WaterSample[] {
    return Array.from(this.samples.values()).filter(
      s => s.location.stationId === stationId
    );
  }

  /**
   * Get samples within a time range
   */
  getSamplesByTimeRange(startDate: Date, endDate: Date): WaterSample[] {
    return Array.from(this.samples.values()).filter(
      s => s.timestamp >= startDate && s.timestamp <= endDate
    );
  }

  // ============================================================================
  // Quality Analysis
  // ============================================================================

  /**
   * Calculate Water Quality Index (WQI)
   * Based on multiple parameters
   */
  calculateWQI(sampleId: string): number {
    const sample = this.samples.get(sampleId);
    if (!sample) {
      throw new Error(`Sample ${sampleId} not found`);
    }

    // Simplified WQI calculation (0-100 scale)
    // Higher is better
    const weights = {
      pH: 0.15,
      dissolvedOxygen: 0.20,
      turbidity: 0.15,
      totalColiform: 0.20,
      temperature: 0.10,
      BOD: 0.20,
    };

    let wqi = 0;
    let totalWeight = 0;

    // pH sub-index (optimal: 7.0)
    const pHScore = 100 - Math.abs(sample.chemical.pH - 7.0) * 10;
    wqi += pHScore * weights.pH;
    totalWeight += weights.pH;

    // Dissolved Oxygen sub-index (optimal: > 6 mg/L)
    const doScore = Math.min(100, (sample.chemical.dissolvedOxygen / 6) * 100);
    wqi += doScore * weights.dissolvedOxygen;
    totalWeight += weights.dissolvedOxygen;

    // Turbidity sub-index (optimal: < 5 NTU)
    const turbidityScore = Math.max(0, 100 - (sample.physical.turbidity / 5) * 100);
    wqi += turbidityScore * weights.turbidity;
    totalWeight += weights.turbidity;

    // Total Coliform sub-index (optimal: 0)
    if (sample.biological?.totalColiform !== undefined) {
      const coliformScore = Math.max(0, 100 - (sample.biological.totalColiform / 50) * 100);
      wqi += coliformScore * weights.totalColiform;
      totalWeight += weights.totalColiform;
    }

    // Temperature sub-index (optimal: 20°C)
    const tempScore = 100 - Math.abs(sample.physical.temperature - 20) * 2;
    wqi += tempScore * weights.temperature;
    totalWeight += weights.temperature;

    // BOD sub-index (optimal: < 3 mg/L)
    if (sample.chemical.BOD !== undefined) {
      const bodScore = Math.max(0, 100 - (sample.chemical.BOD / 3) * 100);
      wqi += bodScore * weights.BOD;
      totalWeight += weights.BOD;
    }

    return Math.max(0, Math.min(100, wqi / totalWeight));
  }

  /**
   * Analyze trends over time
   */
  analyzeTrend(stationId: string, parameter: string, days: number = 30): {
    trend: 'increasing' | 'decreasing' | 'stable';
    changeRate: number;
    samples: number;
  } {
    const cutoffDate = new Date();
    cutoffDate.setDate(cutoffDate.getDate() - days);

    const stationSamples = this.getSamplesByStation(stationId)
      .filter(s => s.timestamp >= cutoffDate)
      .sort((a, b) => a.timestamp.getTime() - b.timestamp.getTime());

    if (stationSamples.length < 2) {
      return { trend: 'stable', changeRate: 0, samples: stationSamples.length };
    }

    // Simple linear regression
    const values = stationSamples.map(s => this.getParameterValue(s, parameter));
    const n = values.length;
    const avgValue = values.reduce((a, b) => a + b, 0) / n;

    let slope = 0;
    let sumXY = 0;
    let sumX2 = 0;

    for (let i = 0; i < n; i++) {
      sumXY += i * values[i];
      sumX2 += i * i;
    }

    slope = (sumXY - n * (n - 1) / 2 * avgValue) / (sumX2 - n * (n - 1) * (n - 1) / 4);

    const changeRate = (slope / avgValue) * 100; // Percentage change

    let trend: 'increasing' | 'decreasing' | 'stable';
    if (Math.abs(changeRate) < 5) {
      trend = 'stable';
    } else if (changeRate > 0) {
      trend = 'increasing';
    } else {
      trend = 'decreasing';
    }

    return { trend, changeRate, samples: n };
  }

  /**
   * Get parameter value from sample
   */
  private getParameterValue(sample: WaterSample, parameter: string): number {
    // Check all parameter groups
    if (parameter in sample.physical) {
      return (sample.physical as any)[parameter];
    }
    if (parameter in sample.chemical) {
      return (sample.chemical as any)[parameter];
    }
    if (sample.biological && parameter in sample.biological) {
      return (sample.biological as any)[parameter];
    }
    if (sample.metals && parameter in sample.metals) {
      return (sample.metals as any)[parameter];
    }
    return 0;
  }

  // ============================================================================
  // Compliance Checking
  // ============================================================================

  /**
   * Initialize default WHO/EPA thresholds
   */
  private initializeDefaultThresholds(): void {
    // WHO Drinking Water Guidelines
    this.addThreshold({
      parameter: 'pH',
      minLevel: 6.5,
      maxLevel: 8.5,
      optimalRange: [7.0, 7.5],
      unit: 'pH units',
      standard: 'WHO',
    });

    this.addThreshold({
      parameter: 'turbidity',
      maxLevel: 5,
      unit: 'NTU',
      standard: 'WHO',
      actionLevel: 1,
    });

    this.addThreshold({
      parameter: 'dissolvedOxygen',
      minLevel: 5,
      optimalRange: [6, 8],
      unit: 'mg/L',
      standard: 'EPA',
    });

    this.addThreshold({
      parameter: 'totalColiform',
      maxLevel: 0,
      unit: 'CFU/100mL',
      standard: 'WHO',
      actionLevel: 10,
    });

    this.addThreshold({
      parameter: 'eColi',
      maxLevel: 0,
      unit: 'CFU/100mL',
      standard: 'WHO',
      actionLevel: 1,
    });

    this.addThreshold({
      parameter: 'lead',
      maxLevel: 0.01,
      unit: 'mg/L',
      standard: 'WHO',
      actionLevel: 0.015,
    });

    this.addThreshold({
      parameter: 'arsenic',
      maxLevel: 0.01,
      unit: 'mg/L',
      standard: 'WHO',
    });
  }

  /**
   * Add or update compliance threshold
   */
  addThreshold(threshold: ComplianceThreshold): void {
    this.thresholds.set(threshold.parameter, threshold);
  }

  /**
   * Check compliance for a sample
   */
  checkCompliance(sampleId: string): ComplianceResult[] {
    const sample = this.samples.get(sampleId);
    if (!sample) {
      throw new Error(`Sample ${sampleId} not found`);
    }

    const results: ComplianceResult[] = [];

    for (const [parameter, threshold] of this.thresholds.entries()) {
      const value = this.getParameterValue(sample, parameter);
      if (value === 0 && !sample.biological) continue; // Skip if parameter not measured

      const result = this.checkParameterCompliance(parameter, value, threshold);
      results.push(result);
    }

    return results;
  }

  /**
   * Check single parameter compliance
   */
  private checkParameterCompliance(
    parameter: string,
    value: number,
    threshold: ComplianceThreshold
  ): ComplianceResult {
    let isCompliant = true;
    let severity: 'minor' | 'major' | 'critical' | undefined;
    let action: string | undefined;

    // Check max level
    if (threshold.maxLevel !== undefined && value > threshold.maxLevel) {
      isCompliant = false;
      const exceedance = ((value - threshold.maxLevel) / threshold.maxLevel) * 100;

      if (exceedance > 50) {
        severity = 'critical';
        action = `Immediate action required: ${parameter} exceeds limit by ${exceedance.toFixed(1)}%`;
      } else if (exceedance > 20) {
        severity = 'major';
        action = `Corrective action needed: ${parameter} exceeds limit by ${exceedance.toFixed(1)}%`;
      } else {
        severity = 'minor';
        action = `Monitor closely: ${parameter} slightly exceeds limit`;
      }
    }

    // Check min level
    if (threshold.minLevel !== undefined && value < threshold.minLevel) {
      isCompliant = false;
      severity = 'major';
      action = `${parameter} below minimum acceptable level`;
    }

    return {
      parameter,
      measuredValue: value,
      threshold,
      isCompliant,
      severity,
      action,
    };
  }

  // ============================================================================
  // Contaminant Detection
  // ============================================================================

  /**
   * Detect contaminants in a sample
   */
  detectContaminants(sampleId: string): Contaminant[] {
    const sample = this.samples.get(sampleId);
    if (!sample) {
      throw new Error(`Sample ${sampleId} not found`);
    }

    const contaminants: Contaminant[] = [];

    // Check biological contaminants
    if (sample.biological) {
      if (sample.biological.totalColiform && sample.biological.totalColiform > 0) {
        contaminants.push({
          name: 'Total Coliform',
          category: 'microbiological',
          concentration: sample.biological.totalColiform,
          unit: 'CFU/100mL',
        });
      }
      if (sample.biological.eColi && sample.biological.eColi > 0) {
        contaminants.push({
          name: 'E. coli',
          category: 'microbiological',
          concentration: sample.biological.eColi,
          unit: 'CFU/100mL',
        });
      }
    }

    // Check heavy metals
    if (sample.metals) {
      const metalThresholds = {
        lead: 0.01,
        arsenic: 0.01,
        mercury: 0.001,
        cadmium: 0.003,
      };

      for (const [metal, threshold] of Object.entries(metalThresholds)) {
        const value = (sample.metals as any)[metal];
        if (value && value > threshold) {
          contaminants.push({
            name: metal.charAt(0).toUpperCase() + metal.slice(1),
            category: 'inorganic',
            concentration: value,
            unit: 'mg/L',
          });
        }
      }
    }

    if (contaminants.length > 0) {
      this.emit({
        type: 'contaminant-detected',
        timestamp: new Date(),
        sampleId,
        data: { contaminants },
        severity: 'warning',
      });
    }

    return contaminants;
  }

  // ============================================================================
  // Treatment Monitoring
  // ============================================================================

  /**
   * Monitor treatment process
   */
  monitorTreatment(process: TreatmentProcess): void {
    // Calculate efficiency for key parameters
    if (process.inlet.chemical && process.outlet.chemical) {
      const inletTurbidity = process.inlet.physical?.turbidity || 0;
      const outletTurbidity = process.outlet.physical?.turbidity || 0;

      if (inletTurbidity > 0) {
        process.efficiency = ((inletTurbidity - outletTurbidity) / inletTurbidity) * 100;
      }
    }

    this.emit({
      type: 'treatment-completed',
      timestamp: new Date(),
      data: process,
      severity: 'info',
    });
  }

  // ============================================================================
  // Station Management
  // ============================================================================

  /**
   * Register monitoring station
   */
  registerStation(station: MonitoringStation): void {
    this.stations.set(station.stationId, station);
  }

  /**
   * Get station by ID
   */
  getStation(stationId: string): MonitoringStation | undefined {
    return this.stations.get(stationId);
  }

  /**
   * Get all active stations
   */
  getActiveStations(): MonitoringStation[] {
    return Array.from(this.stations.values()).filter(s => s.isActive);
  }

  // ============================================================================
  // Reporting
  // ============================================================================

  /**
   * Generate water quality report
   */
  generateReport(stationId: string, startDate: Date, endDate: Date): {
    station: MonitoringStation | undefined;
    period: { start: Date; end: Date };
    sampleCount: number;
    averageWQI: number;
    complianceRate: number;
    violations: ComplianceResult[];
    contaminants: Contaminant[];
  } {
    const station = this.stations.get(stationId);
    const samples = this.getSamplesByStation(stationId).filter(
      s => s.timestamp >= startDate && s.timestamp <= endDate
    );

    let totalWQI = 0;
    let totalCompliance = 0;
    const allViolations: ComplianceResult[] = [];
    const allContaminants: Contaminant[] = [];

    for (const sample of samples) {
      const wqi = this.calculateWQI(sample.sampleId);
      totalWQI += wqi;

      const complianceResults = this.checkCompliance(sample.sampleId);
      const compliantCount = complianceResults.filter(r => r.isCompliant).length;
      totalCompliance += (compliantCount / complianceResults.length) * 100;

      const violations = complianceResults.filter(r => !r.isCompliant);
      allViolations.push(...violations);

      const contaminants = this.detectContaminants(sample.sampleId);
      allContaminants.push(...contaminants);
    }

    return {
      station,
      period: { start: startDate, end: endDate },
      sampleCount: samples.length,
      averageWQI: samples.length > 0 ? totalWQI / samples.length : 0,
      complianceRate: samples.length > 0 ? totalCompliance / samples.length : 0,
      violations: allViolations,
      contaminants: allContaminants,
    };
  }

  // ============================================================================
  // Event Handling
  // ============================================================================

  /**
   * Register event handler
   */
  on(eventType: WaterQualityEventType, handler: WaterQualityEventHandler): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, []);
    }
    this.eventHandlers.get(eventType)!.push(handler);
  }

  /**
   * Emit event
   */
  private emit(event: WaterQualityEvent): void {
    const handlers = this.eventHandlers.get(event.type);
    if (handlers) {
      handlers.forEach(handler => handler(event));
    }
  }
}

/**
 * Factory function to create WIAWaterQuality instance
 */
export function createWaterQuality(): WIAWaterQuality {
  return new WIAWaterQuality();
}

/**
 * Default export
 */
export default WIAWaterQuality;
