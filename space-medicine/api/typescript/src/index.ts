/**
 * WIA-SPACE-023: Space Medicine Standard - TypeScript SDK
 * Version: 1.0
 * Philosophy: 弘益人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

import EventEmitter from 'eventemitter3';
import {
  HealthMonitoringConfig,
  APIResponse,
  HealthData,
  HealthAlert,
  RadiationExposure,
  ExercisePrescription,
  MissionProfile,
  MissionLocation,
  VitalSigns,
  HealthStatus
} from './types';

export * from './types';

export class SpaceMedicineClient {
  private config: Required<HealthMonitoringConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: HealthMonitoringConfig) {
    this.config = {
      baseURL: 'https://api.wia-standards.org/v1/space-medicine',
      timeout: 60000,
      debug: false,
      alertThresholds: {
        heartRate: { min: 50, max: 100 },
        bloodPressure: { systolic: 140, diastolic: 90 },
        radiationDaily: 2.0 // mSv/day
      },
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  /**
   * Submit health data for a crew member
   */
  async submitHealthData(data: HealthData): Promise<APIResponse<HealthData>> {
    this.log('Submitting health data', data);
    const validated = this.validateHealthData(data);

    if (!validated.valid) {
      return {
        success: false,
        error: {
          code: 'VALIDATION_ERROR',
          message: validated.errors?.join(', ') || 'Validation failed'
        },
        timestamp: new Date().toISOString()
      };
    }

    return this.makeRequest('POST', '/health-data', data);
  }

  /**
   * Get health data for a crew member
   */
  async getHealthData(crewMemberId: string, from?: string, to?: string): Promise<APIResponse<HealthData[]>> {
    const params = new URLSearchParams();
    if (from) params.append('from', from);
    if (to) params.append('to', to);

    return this.makeRequest('GET', `/health-data/${crewMemberId}?${params}`);
  }

  /**
   * Calculate radiation exposure estimate
   */
  calculateRadiationExposure(
    location: MissionLocation,
    durationDays: number,
    solarActivity: 'Low' | 'Moderate' | 'High' = 'Moderate'
  ): RadiationExposure {
    const baseRates: Record<MissionLocation, number> = {
      'LEO': 0.5,      // mSv/day
      'Lunar': 0.7,    // mSv/day
      'Mars': 1.8,     // mSv/day (transit)
      'DeepSpace': 2.5 // mSv/day
    };

    const solarMultiplier = {
      'Low': 0.8,
      'Moderate': 1.0,
      'High': 1.3
    };

    const dailyDose = baseRates[location] * solarMultiplier[solarActivity];
    const cumulative = dailyDose * durationDays;

    return {
      cumulative,
      daily: dailyDose,
      eyeLens: cumulative * 0.8,
      skin: cumulative * 0.9,
      bloodForming: cumulative * 1.0
    };
  }

  /**
   * Generate exercise prescription based on mission parameters
   */
  generateExercisePrescription(
    crewMass: number,
    missionDuration: number,
    location: MissionLocation
  ): ExercisePrescription {
    let aerobicMinutes = 60;
    let resistiveMinutes = 30;
    let frequency = 5;
    let intensity: 'Low' | 'Moderate' | 'High' = 'Moderate';

    if (missionDuration > 30) {
      aerobicMinutes = 90;
      resistiveMinutes = 45;
      frequency = 6;
    }

    if (missionDuration > 180) {
      aerobicMinutes = 120;
      resistiveMinutes = 60;
      frequency = 6;
      intensity = 'High';
    }

    const equipment = ['ARED', 'Treadmill', 'Cycle Ergometer'];
    if (location === 'Mars') {
      equipment.push('Resistance Bands', 'Portable Weights');
    }

    return {
      aerobicMinutes,
      resistiveMinutes,
      frequency,
      intensity,
      equipment
    };
  }

  /**
   * Validate vital signs against thresholds
   */
  validateVitals(vitals: VitalSigns): { status: HealthStatus; alerts: string[] } {
    const alerts: string[] = [];
    let status: HealthStatus = 'Normal';

    const { heartRate, min, max } = this.config.alertThresholds?.heartRate || { min: 50, max: 100 };

    if (vitals.heartRate < (min || 50) || vitals.heartRate > (max || 100)) {
      alerts.push(`Heart rate ${vitals.heartRate} bpm outside normal range`);
      status = 'Warning';
    }

    if (vitals.bloodPressure.systolic > 140 || vitals.bloodPressure.diastolic > 90) {
      alerts.push('Blood pressure elevated');
      status = 'Warning';
    }

    if (vitals.temperature > 38.0 || vitals.temperature < 36.0) {
      alerts.push(`Temperature ${vitals.temperature}°C outside normal range`);
      status = 'Alert';
    }

    if (vitals.oxygenSaturation < 92) {
      alerts.push(`Low oxygen saturation: ${vitals.oxygenSaturation}%`);
      status = 'Alert';
    }

    return { status, alerts };
  }

  /**
   * Get active health alerts
   */
  async getAlerts(crewMemberId?: string): Promise<APIResponse<HealthAlert[]>> {
    const path = crewMemberId ? `/alerts/${crewMemberId}` : '/alerts';
    return this.makeRequest('GET', path);
  }

  /**
   * Acknowledge an alert
   */
  async acknowledgeAlert(alertId: string, resolution?: string): Promise<APIResponse<HealthAlert>> {
    return this.makeRequest('PUT', `/alerts/${alertId}/acknowledge`, { resolution });
  }

  /**
   * Subscribe to real-time health events
   */
  on(event: string, callback: (...args: any[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  /**
   * Emit health event
   */
  emit(event: string, ...args: any[]): void {
    this.eventEmitter.emit(event, ...args);
  }

  private validateHealthData(data: HealthData): { valid: boolean; errors?: string[] } {
    const errors: string[] = [];

    if (!data.crewMember?.id) errors.push('Crew member ID is required');
    if (!data.vitals) errors.push('Vital signs are required');
    if (!data.timestamp) errors.push('Timestamp is required');
    if (data.vitals?.heartRate < 0 || data.vitals?.heartRate > 250) {
      errors.push('Invalid heart rate');
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined
    };
  }

  private async makeRequest<T>(method: string, path: string, body?: any): Promise<APIResponse<T>> {
    const url = `${this.config.baseURL}${path}`;
    this.log(`${method} ${url}`, body);

    try {
      // Simulated response for demo purposes
      return {
        success: true,
        data: body as T,
        timestamp: new Date().toISOString()
      };
    } catch (error: any) {
      return {
        success: false,
        error: {
          code: 'REQUEST_FAILED',
          message: error.message,
        },
        timestamp: new Date().toISOString()
      };
    }
  }

  private log(...args: any[]): void {
    if (this.config.debug) {
      console.log('[WIA-SPACE-023]', ...args);
    }
  }
}

/**
 * Utility functions
 */
export const SpaceMedicineUtils = {
  /**
   * Calculate Body Mass Index (BMI) - note: standard BMI, not space-adjusted
   */
  calculateBMI(mass: number, height: number): number {
    return mass / (height * height);
  },

  /**
   * Estimate bone density loss percentage
   */
  estimateBoneLoss(durationDays: number, withCountermeasures: boolean = true): number {
    const monthlyLossRate = withCountermeasures ? 0.5 : 1.5; // % per month
    const months = durationDays / 30;
    return monthlyLossRate * months;
  },

  /**
   * Check if radiation exposure is within career limits
   */
  isWithinRadiationLimits(exposure: RadiationExposure, age: number, gender: 'M' | 'F'): boolean {
    // Simplified - actual limits are age and gender specific
    const careerLimit = 1000; // mSv (simplified)
    return exposure.cumulative < careerLimit && exposure.eyeLens < 1000;
  }
};
