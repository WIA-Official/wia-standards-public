/**
 * WIA Air Quality Standard - SDK Implementation
 *
 * @packageDocumentation
 * @module wia-air-quality
 */

import { EventEmitter } from 'events';
import * as types from './types';

// Re-export all types
export * from './types';

/**
 * Main Air Quality Monitor class
 * Provides unified interface for air quality monitoring and AQI calculation
 *
 * @example
 * ```typescript
 * const monitor = new WIAAirQuality({
 *   aqiStandard: AQIStandard.US_EPA,
 *   refreshInterval: 300,
 *   alertsEnabled: true
 * });
 *
 * monitor.on('reading', (reading) => {
 *   const aqi = monitor.calculateAQI(reading);
 *   console.log(`AQI: ${aqi.aqi} (${aqi.category})`);
 * });
 *
 * await monitor.start();
 * ```
 */
export class WIAAirQuality extends EventEmitter {
  private config: types.AirQualityConfig;
  private stations: Map<string, types.MonitoringStation> = new Map();
  private sensors: Map<string, types.AQSensor> = new Map();
  private readings: types.AQReading[] = [];
  private alerts: Map<string, types.AQAlert> = new Map();
  private isMonitoring: boolean = false;

  /**
   * Create a new Air Quality Monitor instance
   * @param config - Configuration options
   */
  constructor(config: types.AirQualityConfig) {
    super();
    this.config = config;
  }

  /**
   * Start monitoring
   */
  async start(): Promise<void> {
    console.log('Starting air quality monitoring...');
    this.isMonitoring = true;
    this.emit('started');
  }

  /**
   * Stop monitoring
   */
  async stop(): Promise<void> {
    console.log('Stopping air quality monitoring...');
    this.isMonitoring = false;
    this.emit('stopped');
  }

  /**
   * Register a monitoring station
   * @param station - Station definition
   */
  registerStation(station: types.MonitoringStation): void {
    this.stations.set(station.id, station);
    for (const sensor of station.sensors) {
      this.sensors.set(sensor.id, sensor);
    }
    this.emit('station-registered', station);
  }

  /**
   * Get station by ID
   * @param stationId - Station ID
   */
  getStation(stationId: string): types.MonitoringStation | undefined {
    return this.stations.get(stationId);
  }

  /**
   * Get all stations
   */
  getAllStations(): types.MonitoringStation[] {
    return Array.from(this.stations.values());
  }

  /**
   * Get stations near location
   * @param location - Center location
   * @param radiusKm - Radius in kilometers
   */
  getStationsNear(location: types.GeoLocation, radiusKm: number): types.MonitoringStation[] {
    return Array.from(this.stations.values()).filter(station => {
      const distance = this.calculateDistance(location, station.location);
      return distance <= radiusKm;
    });
  }

  /**
   * Calculate distance between two points (Haversine)
   */
  private calculateDistance(loc1: types.GeoLocation, loc2: types.GeoLocation): number {
    const R = 6371; // Earth radius in km
    const dLat = this.toRad(loc2.latitude - loc1.latitude);
    const dLon = this.toRad(loc2.longitude - loc1.longitude);
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(this.toRad(loc1.latitude)) * Math.cos(this.toRad(loc2.latitude)) *
      Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
  }

  private toRad(deg: number): number {
    return deg * (Math.PI / 180);
  }

  /**
   * Submit a reading
   * @param reading - Air quality reading
   */
  async submitReading(reading: types.AQReading): Promise<void> {
    // Validate reading
    reading.valid = this.validateReading(reading);
    this.readings.push(reading);

    // Calculate AQI
    const aqi = this.calculateAQI(reading);

    // Check alerts
    if (this.config.alertsEnabled) {
      this.checkAlerts(reading, aqi);
    }

    this.emit('reading', { reading, aqi });
  }

  /**
   * Validate reading
   */
  private validateReading(reading: types.AQReading): boolean {
    const flags: string[] = [];

    for (const conc of reading.concentrations) {
      // Check for negative values
      if (conc.value < 0) {
        flags.push(`negative_${conc.pollutant}`);
      }
      // Check for unrealistic values
      if (conc.pollutant === types.Pollutant.PM25 && conc.value > 1000) {
        flags.push('pm25_out_of_range');
      }
    }

    reading.flags = flags;
    return flags.length === 0;
  }

  /**
   * Calculate AQI from reading
   * @param reading - Air quality reading
   */
  calculateAQI(reading: types.AQReading): types.AQIResult {
    const pollutantAQIs: types.PollutantAQI[] = [];
    let maxAQI = 0;
    let dominantPollutant = types.Pollutant.PM25;

    for (const conc of reading.concentrations) {
      const aqi = this.calculatePollutantAQI(conc.pollutant, conc.value);
      pollutantAQIs.push({
        pollutant: conc.pollutant,
        aqi,
        concentration: conc.value,
        unit: conc.unit
      });

      if (aqi > maxAQI) {
        maxAQI = aqi;
        dominantPollutant = conc.pollutant;
      }
    }

    const category = this.getAQICategory(maxAQI);

    return {
      aqi: Math.round(maxAQI),
      category,
      dominantPollutant,
      pollutantAQIs,
      standard: this.config.aqiStandard,
      timestamp: new Date()
    };
  }

  /**
   * Calculate AQI for a single pollutant
   */
  private calculatePollutantAQI(pollutant: types.Pollutant, concentration: number): number {
    const breakpoints = this.getBreakpoints(pollutant);

    for (const bp of breakpoints) {
      if (concentration >= bp.concLow && concentration <= bp.concHigh) {
        return ((bp.aqiHigh - bp.aqiLow) / (bp.concHigh - bp.concLow)) *
          (concentration - bp.concLow) + bp.aqiLow;
      }
    }

    // Above highest breakpoint
    return 500;
  }

  /**
   * Get breakpoints for US EPA standard
   */
  private getBreakpoints(pollutant: types.Pollutant): types.AQIBreakpoint[] {
    // US EPA breakpoints
    const breakpointTable: Record<types.Pollutant, types.AQIBreakpoint[]> = {
      [types.Pollutant.PM25]: [
        { pollutant, aqiLow: 0, aqiHigh: 50, concLow: 0, concHigh: 12 },
        { pollutant, aqiLow: 51, aqiHigh: 100, concLow: 12.1, concHigh: 35.4 },
        { pollutant, aqiLow: 101, aqiHigh: 150, concLow: 35.5, concHigh: 55.4 },
        { pollutant, aqiLow: 151, aqiHigh: 200, concLow: 55.5, concHigh: 150.4 },
        { pollutant, aqiLow: 201, aqiHigh: 300, concLow: 150.5, concHigh: 250.4 },
        { pollutant, aqiLow: 301, aqiHigh: 500, concLow: 250.5, concHigh: 500.4 }
      ],
      [types.Pollutant.PM10]: [
        { pollutant, aqiLow: 0, aqiHigh: 50, concLow: 0, concHigh: 54 },
        { pollutant, aqiLow: 51, aqiHigh: 100, concLow: 55, concHigh: 154 },
        { pollutant, aqiLow: 101, aqiHigh: 150, concLow: 155, concHigh: 254 },
        { pollutant, aqiLow: 151, aqiHigh: 200, concLow: 255, concHigh: 354 },
        { pollutant, aqiLow: 201, aqiHigh: 300, concLow: 355, concHigh: 424 },
        { pollutant, aqiLow: 301, aqiHigh: 500, concLow: 425, concHigh: 604 }
      ],
      [types.Pollutant.Ozone]: [
        { pollutant, aqiLow: 0, aqiHigh: 50, concLow: 0, concHigh: 54 },
        { pollutant, aqiLow: 51, aqiHigh: 100, concLow: 55, concHigh: 70 },
        { pollutant, aqiLow: 101, aqiHigh: 150, concLow: 71, concHigh: 85 },
        { pollutant, aqiLow: 151, aqiHigh: 200, concLow: 86, concHigh: 105 },
        { pollutant, aqiLow: 201, aqiHigh: 300, concLow: 106, concHigh: 200 }
      ],
      [types.Pollutant.NitrogenDioxide]: [
        { pollutant, aqiLow: 0, aqiHigh: 50, concLow: 0, concHigh: 53 },
        { pollutant, aqiLow: 51, aqiHigh: 100, concLow: 54, concHigh: 100 },
        { pollutant, aqiLow: 101, aqiHigh: 150, concLow: 101, concHigh: 360 },
        { pollutant, aqiLow: 151, aqiHigh: 200, concLow: 361, concHigh: 649 },
        { pollutant, aqiLow: 201, aqiHigh: 300, concLow: 650, concHigh: 1249 }
      ],
      [types.Pollutant.SulfurDioxide]: [
        { pollutant, aqiLow: 0, aqiHigh: 50, concLow: 0, concHigh: 35 },
        { pollutant, aqiLow: 51, aqiHigh: 100, concLow: 36, concHigh: 75 },
        { pollutant, aqiLow: 101, aqiHigh: 150, concLow: 76, concHigh: 185 },
        { pollutant, aqiLow: 151, aqiHigh: 200, concLow: 186, concHigh: 304 }
      ],
      [types.Pollutant.CarbonMonoxide]: [
        { pollutant, aqiLow: 0, aqiHigh: 50, concLow: 0, concHigh: 4.4 },
        { pollutant, aqiLow: 51, aqiHigh: 100, concLow: 4.5, concHigh: 9.4 },
        { pollutant, aqiLow: 101, aqiHigh: 150, concLow: 9.5, concHigh: 12.4 },
        { pollutant, aqiLow: 151, aqiHigh: 200, concLow: 12.5, concHigh: 15.4 }
      ],
      [types.Pollutant.Lead]: [],
      [types.Pollutant.VOC]: [],
      [types.Pollutant.Ammonia]: [],
      [types.Pollutant.Methane]: []
    };

    return breakpointTable[pollutant] || [];
  }

  /**
   * Get AQI category
   */
  private getAQICategory(aqi: number): types.AQICategory {
    if (aqi <= 50) return types.AQICategory.Good;
    if (aqi <= 100) return types.AQICategory.Moderate;
    if (aqi <= 150) return types.AQICategory.UnhealthySensitive;
    if (aqi <= 200) return types.AQICategory.Unhealthy;
    if (aqi <= 300) return types.AQICategory.VeryUnhealthy;
    return types.AQICategory.Hazardous;
  }

  /**
   * Get health recommendation
   * @param category - AQI category
   */
  getHealthRecommendation(category: types.AQICategory): types.HealthRecommendation {
    const recommendations: Record<types.AQICategory, types.HealthRecommendation> = {
      [types.AQICategory.Good]: {
        category,
        generalMessage: 'Air quality is satisfactory. Enjoy outdoor activities.',
        sensitiveGroupsMessage: 'Air quality is good. No precautions needed.',
        outdoorActivity: 'unrestricted',
        maskRecommendation: 'not_needed'
      },
      [types.AQICategory.Moderate]: {
        category,
        generalMessage: 'Air quality is acceptable. Unusually sensitive people should consider reducing prolonged outdoor exertion.',
        sensitiveGroupsMessage: 'Consider reducing prolonged or heavy outdoor exertion.',
        outdoorActivity: 'moderate',
        maskRecommendation: 'optional'
      },
      [types.AQICategory.UnhealthySensitive]: {
        category,
        generalMessage: 'Members of sensitive groups may experience health effects.',
        sensitiveGroupsMessage: 'Reduce prolonged or heavy outdoor exertion. Take more breaks.',
        outdoorActivity: 'reduce',
        maskRecommendation: 'recommended'
      },
      [types.AQICategory.Unhealthy]: {
        category,
        generalMessage: 'Everyone may begin to experience health effects.',
        sensitiveGroupsMessage: 'Avoid prolonged or heavy outdoor exertion. Move activities indoors.',
        outdoorActivity: 'avoid',
        maskRecommendation: 'recommended'
      },
      [types.AQICategory.VeryUnhealthy]: {
        category,
        generalMessage: 'Health alert: everyone may experience more serious health effects.',
        sensitiveGroupsMessage: 'Avoid all outdoor exertion. Remain indoors.',
        outdoorActivity: 'stay_indoor',
        maskRecommendation: 'required'
      },
      [types.AQICategory.Hazardous]: {
        category,
        generalMessage: 'Health warning of emergency conditions. Everyone is affected.',
        sensitiveGroupsMessage: 'Remain indoors. Keep windows closed. Use air purifier.',
        outdoorActivity: 'stay_indoor',
        maskRecommendation: 'required'
      }
    };

    return recommendations[category];
  }

  /**
   * Check and trigger alerts
   */
  private checkAlerts(reading: types.AQReading, aqi: types.AQIResult): void {
    if (!this.config.alertThresholds) return;

    for (const threshold of this.config.alertThresholds) {
      const pollutantAQI = aqi.pollutantAQIs.find(p => p.pollutant === threshold.pollutant);
      if (!pollutantAQI) continue;

      const conc = pollutantAQI.concentration;
      let severity: types.AQAlert['severity'] | null = null;

      if (conc >= threshold.emergencyThreshold) {
        severity = 'emergency';
      } else if (conc >= threshold.alertThreshold) {
        severity = 'alert';
      } else if (conc >= threshold.warningThreshold) {
        severity = 'warning';
      }

      if (severity) {
        const alert: types.AQAlert = {
          id: `alert-${Date.now()}-${threshold.pollutant}`,
          type: 'threshold',
          severity,
          pollutant: threshold.pollutant,
          threshold: threshold[`${severity}Threshold` as keyof types.AlertThreshold] as number,
          currentValue: conc,
          affectedArea: reading.location,
          radiusKm: 10,
          message: `${threshold.pollutant} level ${severity}: ${conc}`,
          createdAt: new Date(),
          expiresAt: new Date(Date.now() + 3600000),
          active: true
        };

        this.alerts.set(alert.id, alert);
        this.emit('alert', alert);
      }
    }
  }

  /**
   * Get active alerts
   */
  getActiveAlerts(): types.AQAlert[] {
    return Array.from(this.alerts.values()).filter(a => a.active);
  }

  /**
   * Get historical data
   * @param query - Query parameters
   */
  async getHistoricalData(query: types.HistoricalQuery): Promise<types.AQReading[]> {
    return this.readings.filter(r => {
      if (query.stationId && !r.sensorId.startsWith(query.stationId)) {
        return false;
      }
      if (r.timestamp < query.startTime || r.timestamp > query.endTime) {
        return false;
      }
      return true;
    });
  }

  /**
   * Calculate statistics
   * @param stationId - Station ID
   * @param pollutant - Pollutant
   * @param startTime - Start time
   * @param endTime - End time
   */
  calculateStatistics(
    stationId: string,
    pollutant: types.Pollutant,
    startTime: Date,
    endTime: Date
  ): types.AQStatistics {
    const relevantReadings = this.readings.filter(r =>
      r.sensorId.startsWith(stationId) &&
      r.timestamp >= startTime &&
      r.timestamp <= endTime
    );

    const values = relevantReadings
      .flatMap(r => r.concentrations)
      .filter(c => c.pollutant === pollutant)
      .map(c => c.value)
      .sort((a, b) => a - b);

    const n = values.length;
    if (n === 0) {
      return {
        stationId,
        pollutant,
        periodStart: startTime,
        periodEnd: endTime,
        mean: 0,
        median: 0,
        min: 0,
        max: 0,
        stdDev: 0,
        percentile95: 0,
        dataCompleteness: 0
      };
    }

    const sum = values.reduce((a, b) => a + b, 0);
    const mean = sum / n;
    const median = n % 2 === 0 ? (values[n / 2 - 1] + values[n / 2]) / 2 : values[Math.floor(n / 2)];
    const variance = values.reduce((a, b) => a + Math.pow(b - mean, 2), 0) / n;

    return {
      stationId,
      pollutant,
      periodStart: startTime,
      periodEnd: endTime,
      mean,
      median,
      min: values[0],
      max: values[n - 1],
      stdDev: Math.sqrt(variance),
      percentile95: values[Math.floor(n * 0.95)],
      dataCompleteness: 100
    };
  }

  /**
   * Check WIA compliance
   * @param targetLevel - Target certification level
   */
  checkCompliance(targetLevel: types.CertificationLevel): types.ComplianceReport {
    const tests: types.TestResult[] = [];

    // Test 1: Configuration
    tests.push({
      testName: 'Configuration Validation',
      passed: this.config.aqiStandard !== undefined,
      notes: 'AQI standard must be defined'
    });

    // Test 2: Stations
    tests.push({
      testName: 'Station Registration',
      passed: this.stations.size > 0,
      notes: 'At least one station required'
    });

    // Test 3: Data quality
    const hasRegulatoryQuality = Array.from(this.stations.values())
      .some(s => s.dataQuality === types.DataQuality.Regulatory);

    if (targetLevel !== types.CertificationLevel.Bronze) {
      tests.push({
        testName: 'Regulatory Data Quality',
        passed: hasRegulatoryQuality,
        notes: 'Regulatory-grade data required for Silver/Gold'
      });
    }

    // Test 4: Alert system for Gold
    if (targetLevel === types.CertificationLevel.Gold) {
      tests.push({
        testName: 'Alert System',
        passed: this.config.alertsEnabled && this.config.alertThresholds !== undefined,
        notes: 'Alert system required for Gold certification'
      });
    }

    const passed = tests.every(t => t.passed);

    return {
      standard: 'WIA-AIR-QUALITY',
      testDate: new Date().toISOString(),
      config: this.config,
      targetLevel,
      tests,
      passed,
      achievedLevel: passed ? targetLevel : undefined
    };
  }
}

/**
 * Default export for convenience
 */
export default {
  WIAAirQuality
};
