/**
 * WIA-SPACE-009: Space Radiation Protection Standard
 * TypeScript SDK Implementation
 *
 * @standard WIA-SPACE-009
 * @version 1.0.0
 * @organization WIA (World Certification Industry Association)
 */

import {
  AlertLevel,
  RadiationSource,
  MissionType,
  SolarActivity,
  RadiationMeasurement,
  SPEAlert,
  DosimetryReading,
  CareerLimit,
  MissionDoseParams,
  MissionDoseResult,
  StormShelter,
  APIResponse,
  MonitoringConfig,
  HistoricalDoseData,
  DoseStatistics
} from './types';

/**
 * Main SDK class for WIA-SPACE-009 Space Radiation Standard
 */
export class SpaceRadiationSDK {
  private baseURL: string;
  private apiKey: string;
  private monitoringInterval?: NodeJS.Timeout;

  /**
   * Initialize the SDK
   * @param baseURL - API base URL
   * @param apiKey - Authentication API key
   */
  constructor(baseURL: string, apiKey: string) {
    this.baseURL = baseURL.replace(/\/$/, '');
    this.apiKey = apiKey;
  }

  /**
   * Make authenticated API request
   */
  private async request<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<APIResponse<T>> {
    const url = `${this.baseURL}${endpoint}`;
    const headers = {
      'Authorization': `Bearer ${this.apiKey}`,
      'Content-Type': 'application/json',
      ...options.headers
    };

    try {
      const response = await fetch(url, { ...options, headers });
      const data = await response.json();

      return {
        status: response.ok ? 'success' : 'error',
        data: response.ok ? data : undefined,
        error: response.ok ? undefined : data.message || 'Request failed',
        timestamp: new Date().toISOString()
      };
    } catch (error) {
      return {
        status: 'error',
        error: error instanceof Error ? error.message : 'Unknown error',
        timestamp: new Date().toISOString()
      };
    }
  }

  /**
   * Get current radiation measurements
   */
  async getCurrentRadiation(): Promise<APIResponse<RadiationMeasurement>> {
    return this.request<RadiationMeasurement>('/api/v1/radiation/current');
  }

  /**
   * Get radiation measurement for specific location
   */
  async getRadiationByLocation(location: string): Promise<APIResponse<RadiationMeasurement>> {
    return this.request<RadiationMeasurement>(`/api/v1/radiation/location/${encodeURIComponent(location)}`);
  }

  /**
   * Get active SPE alerts
   */
  async getSPEAlerts(): Promise<APIResponse<SPEAlert[]>> {
    return this.request<SPEAlert[]>('/api/v1/alerts/spe');
  }

  /**
   * Get specific SPE alert by ID
   */
  async getSPEAlertById(alertId: string): Promise<APIResponse<SPEAlert>> {
    return this.request<SPEAlert>(`/api/v1/alerts/spe/${alertId}`);
  }

  /**
   * Submit dosimetry reading
   */
  async submitDosimetryReading(reading: DosimetryReading): Promise<APIResponse<{ accumulated_dose: number }>> {
    return this.request<{ accumulated_dose: number }>('/api/v1/dosimetry/reading', {
      method: 'POST',
      body: JSON.stringify(reading)
    });
  }

  /**
   * Get crew member's dosimetry history
   */
  async getDosimetryHistory(
    crewId: string,
    startDate?: string,
    endDate?: string
  ): Promise<APIResponse<HistoricalDoseData[]>> {
    const params = new URLSearchParams();
    if (startDate) params.append('start', startDate);
    if (endDate) params.append('end', endDate);

    const query = params.toString() ? `?${params.toString()}` : '';
    return this.request<HistoricalDoseData[]>(`/api/v1/dosimetry/history/${crewId}${query}`);
  }

  /**
   * Calculate career dose limit
   */
  async calculateCareerLimit(age: number, gender: 'male' | 'female', accumulatedDose: number): Promise<CareerLimit> {
    // NASA 3% REID (Risk of Exposure-Induced Death) limits
    const limits: Record<string, Record<number, number>> = {
      male: {
        25: 800, 30: 850, 35: 1000, 40: 1100, 45: 1200, 50: 1400, 55: 1600
      },
      female: {
        25: 600, 30: 700, 35: 900, 40: 1000, 45: 1100, 50: 1300, 55: 1500
      }
    };

    // Find closest age bracket
    const ages = Object.keys(limits[gender]).map(Number).sort((a, b) => a - b);
    const closestAge = ages.reduce((prev, curr) =>
      Math.abs(curr - age) < Math.abs(prev - age) ? curr : prev
    );

    const limit = limits[gender][closestAge];
    const remaining = limit - accumulatedDose;
    const percentage = (accumulatedDose / limit) * 100;

    return {
      age,
      gender,
      limit_mSv: limit,
      accumulated_mSv: accumulatedDose,
      remaining_mSv: remaining,
      percentage_used: parseFloat(percentage.toFixed(2))
    };
  }

  /**
   * Calculate mission dose estimate
   */
  async calculateMissionDose(params: MissionDoseParams): Promise<MissionDoseResult> {
    // Base dose rates by mission type and solar activity
    const baseRates: Record<MissionType, Record<SolarActivity, number>> = {
      [MissionType.LEO]: {
        [SolarActivity.MINIMUM]: 0.40,
        [SolarActivity.AVERAGE]: 0.45,
        [SolarActivity.MAXIMUM]: 0.50
      },
      [MissionType.LUNAR]: {
        [SolarActivity.MINIMUM]: 0.60,
        [SolarActivity.AVERAGE]: 0.80,
        [SolarActivity.MAXIMUM]: 1.00
      },
      [MissionType.MARS]: {
        [SolarActivity.MINIMUM]: 1.00,
        [SolarActivity.AVERAGE]: 1.20,
        [SolarActivity.MAXIMUM]: 1.40
      },
      [MissionType.DEEP_SPACE]: {
        [SolarActivity.MINIMUM]: 1.20,
        [SolarActivity.AVERAGE]: 1.50,
        [SolarActivity.MAXIMUM]: 1.80
      }
    };

    const baseDailyRate = baseRates[params.mission_type][params.solar_activity];
    const shieldingFactor = params.shielding_factor || 1.0;
    const dailyDoseRate = baseDailyRate * shieldingFactor;
    const totalDose = dailyDoseRate * params.duration_days;

    // Calculate confidence interval (±20%)
    const confidenceInterval: [number, number] = [
      totalDose * 0.8,
      totalDose * 1.2
    ];

    // Risk assessment
    let riskLevel: 'LOW' | 'MODERATE' | 'HIGH' | 'VERY_HIGH';
    if (totalDose < 250) riskLevel = 'LOW';
    else if (totalDose < 500) riskLevel = 'MODERATE';
    else if (totalDose < 1000) riskLevel = 'HIGH';
    else riskLevel = 'VERY_HIGH';

    // Generate recommendations
    const recommendations: string[] = [];
    if (totalDose > 500) {
      recommendations.push('Consider reducing mission duration or improving shielding');
    }
    if (params.solar_activity === SolarActivity.MAXIMUM) {
      recommendations.push('High solar activity - implement enhanced SPE monitoring');
    }
    if (params.mission_type === MissionType.MARS || params.mission_type === MissionType.DEEP_SPACE) {
      recommendations.push('Long-duration mission - ensure comprehensive medical countermeasures');
    }
    if (riskLevel === 'VERY_HIGH') {
      recommendations.push('⚠️ Dose exceeds recommended limits - mission redesign required');
    }

    return {
      daily_dose_rate_mSv: parseFloat(dailyDoseRate.toFixed(3)),
      total_dose_mSv: parseFloat(totalDose.toFixed(2)),
      confidence_interval: [
        parseFloat(confidenceInterval[0].toFixed(2)),
        parseFloat(confidenceInterval[1].toFixed(2))
      ],
      risk_level: riskLevel,
      recommendations
    };
  }

  /**
   * Get storm shelter status
   */
  async getStormShelters(): Promise<APIResponse<StormShelter[]>> {
    return this.request<StormShelter[]>('/api/v1/shelters');
  }

  /**
   * Activate storm shelter
   */
  async activateStormShelter(shelterId: string): Promise<APIResponse<{ status: string }>> {
    return this.request<{ status: string }>(`/api/v1/shelters/${shelterId}/activate`, {
      method: 'POST'
    });
  }

  /**
   * Get dose statistics for a time period
   */
  async getDoseStatistics(
    startDate: string,
    endDate: string,
    location?: string
  ): Promise<APIResponse<DoseStatistics>> {
    const params = new URLSearchParams({
      start: startDate,
      end: endDate
    });
    if (location) params.append('location', location);

    return this.request<DoseStatistics>(`/api/v1/statistics/dose?${params.toString()}`);
  }

  /**
   * Start real-time radiation monitoring
   */
  startMonitoring(
    config: MonitoringConfig,
    callback: (data: RadiationMeasurement) => void
  ): void {
    if (this.monitoringInterval) {
      this.stopMonitoring();
    }

    this.monitoringInterval = setInterval(async () => {
      const response = await this.getCurrentRadiation();
      if (response.status === 'success' && response.data) {
        callback(response.data);

        // Check thresholds and trigger alerts
        if (config.auto_alert) {
          if (response.data.radiation.dose_rate_mSv_day > config.thresholds.daily_dose_mSv) {
            console.warn('⚠️ Daily dose threshold exceeded:', response.data.radiation.dose_rate_mSv_day);
          }
        }
      }
    }, config.update_interval_sec * 1000);
  }

  /**
   * Stop real-time monitoring
   */
  stopMonitoring(): void {
    if (this.monitoringInterval) {
      clearInterval(this.monitoringInterval);
      this.monitoringInterval = undefined;
    }
  }

  /**
   * Validate measurement data
   */
  static validateMeasurement(measurement: RadiationMeasurement): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    // Validate dose rate range
    if (measurement.radiation.dose_rate_mSv_day < 0 || measurement.radiation.dose_rate_mSv_day > 10) {
      errors.push('Dose rate out of valid range (0-10 mSv/day)');
    }

    // Validate accumulated dose
    if (measurement.radiation.accumulated_dose_mSv < 0) {
      errors.push('Accumulated dose cannot be negative');
    }

    // Validate timestamp
    const timestamp = new Date(measurement.timestamp);
    if (isNaN(timestamp.getTime())) {
      errors.push('Invalid timestamp format');
    }

    // Validate alert level
    if (!Object.values(AlertLevel).includes(measurement.alerts.level)) {
      errors.push('Invalid alert level');
    }

    return {
      valid: errors.length === 0,
      errors
    };
  }

  /**
   * Get recommended action based on alert level
   */
  static getRecommendedActions(level: AlertLevel): string[] {
    const actions: Record<AlertLevel, string[]> = {
      [AlertLevel.GREEN]: [
        'Continue normal operations',
        'Maintain routine monitoring'
      ],
      [AlertLevel.YELLOW]: [
        'Increase dosimetry monitoring frequency',
        'Review EVA schedules',
        'Monitor space weather forecasts'
      ],
      [AlertLevel.ORANGE]: [
        'Abort all EVA activities',
        'Prepare storm shelter',
        'Brief crew on SPE protocols',
        'Activate enhanced monitoring'
      ],
      [AlertLevel.RED]: [
        '🚨 IMMEDIATE shelter evacuation',
        'Suspend all external activities',
        'Continuous crew dosimetry',
        'Maintain emergency communications',
        'Prepare medical countermeasures'
      ]
    };

    return actions[level];
  }
}

// Export all types
export * from './types';

// Default export
export default SpaceRadiationSDK;
