/**
 * WIA Medical IoT Standard - SDK
 * Version: 1.0.0
 *
 * SDK for integrating medical IoT devices with healthcare systems
 */

import {
  MedicalIoTDevice,
  DeviceType,
  Measurement,
  MeasurementUnit,
  DeviceStatus,
  Alert,
  AlertLevel,
  VitalSigns,
  PatientReading,
  DeviceRegistration,
  DeviceConfiguration,
  DeviceCommand,
  DataStream,
  IntegrationConfig
} from './types';

export * from './types';

export class MedicalIoTSDK {
  private apiEndpoint: string;
  private apiKey: string;

  constructor(apiEndpoint: string, apiKey: string) {
    this.apiEndpoint = apiEndpoint;
    this.apiKey = apiKey;
  }

  /**
   * Create a medical IoT device reading
   */
  createDeviceReading(
    deviceId: string,
    deviceType: DeviceType,
    patientId: string,
    value: number,
    unit: MeasurementUnit,
    options?: {
      location?: string;
      firmwareVersion?: string;
      batteryLevel?: number;
      signalStrength?: number;
    }
  ): MedicalIoTDevice {
    return {
      format: 'WIA-MEDICAL-IOT-v1.0',
      timestamp: new Date().toISOString(),
      device_id: deviceId,
      device_type: deviceType,
      patient_id: patientId,
      measurement: {
        value,
        unit,
        timestamp: new Date().toISOString()
      },
      location: options?.location,
      metadata: {
        firmware_version: options?.firmwareVersion || '1.0.0',
        battery_level: options?.batteryLevel,
        signal_strength: options?.signalStrength
      }
    };
  }

  /**
   * Validate device reading
   */
  validateReading(reading: MedicalIoTDevice): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (reading.format !== 'WIA-MEDICAL-IOT-v1.0') {
      errors.push('Invalid format version');
    }

    if (!reading.device_id) {
      errors.push('Device ID is required');
    }

    if (!reading.patient_id) {
      errors.push('Patient ID is required');
    }

    if (!reading.measurement || typeof reading.measurement.value !== 'number') {
      errors.push('Invalid measurement value');
    }

    if (!this.isValidTimestamp(reading.timestamp)) {
      errors.push('Invalid timestamp format');
    }

    return {
      valid: errors.length === 0,
      errors
    };
  }

  /**
   * Create alert for abnormal readings
   */
  createAlert(
    level: AlertLevel,
    message: string,
    deviceId: string
  ): Alert {
    return {
      alert_id: `ALERT-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      level,
      message,
      timestamp: new Date().toISOString(),
      acknowledged: false,
      resolved: false
    };
  }

  /**
   * Check vital signs against normal ranges
   */
  checkVitalSigns(vitals: VitalSigns): Alert[] {
    const alerts: Alert[] = [];

    // Heart rate: normal 60-100 bpm
    if (vitals.heart_rate && vitals.heart_rate.unit === 'bpm') {
      if (vitals.heart_rate.value < 60) {
        alerts.push(this.createAlert('warning', 'Low heart rate detected', 'HR_MONITOR'));
      } else if (vitals.heart_rate.value > 100) {
        alerts.push(this.createAlert('warning', 'High heart rate detected', 'HR_MONITOR'));
      }
    }

    // SpO2: normal >= 95%
    if (vitals.spo2 && vitals.spo2.unit === '%') {
      if (vitals.spo2.value < 90) {
        alerts.push(this.createAlert('critical', 'Critical oxygen saturation', 'SPO2_MONITOR'));
      } else if (vitals.spo2.value < 95) {
        alerts.push(this.createAlert('warning', 'Low oxygen saturation', 'SPO2_MONITOR'));
      }
    }

    // Temperature: normal 36.5-37.5°C (97.7-99.5°F)
    if (vitals.temperature) {
      let tempC = vitals.temperature.value;
      if (vitals.temperature.unit === '°F') {
        tempC = (tempC - 32) * 5 / 9;
      }
      if (tempC < 36.5) {
        alerts.push(this.createAlert('warning', 'Low temperature detected', 'TEMP_MONITOR'));
      } else if (tempC > 37.5) {
        alerts.push(this.createAlert('warning', 'Fever detected', 'TEMP_MONITOR'));
      }
      if (tempC > 39.0) {
        alerts.push(this.createAlert('critical', 'High fever detected', 'TEMP_MONITOR'));
      }
    }

    // Blood pressure: normal <120/80 mmHg
    if (vitals.blood_pressure) {
      const systolic = vitals.blood_pressure.systolic.value;
      const diastolic = vitals.blood_pressure.diastolic.value;

      if (systolic >= 140 || diastolic >= 90) {
        alerts.push(this.createAlert('warning', 'High blood pressure detected', 'BP_MONITOR'));
      }
      if (systolic >= 180 || diastolic >= 120) {
        alerts.push(this.createAlert('critical', 'Hypertensive crisis', 'BP_MONITOR'));
      }
    }

    return alerts;
  }

  /**
   * Calculate BMI from weight and height
   */
  calculateBMI(weightKg: number, heightCm: number): {
    bmi: number;
    category: string;
  } {
    const heightM = heightCm / 100;
    const bmi = weightKg / (heightM * heightM);

    let category: string;
    if (bmi < 18.5) {
      category = 'Underweight';
    } else if (bmi < 25) {
      category = 'Normal weight';
    } else if (bmi < 30) {
      category = 'Overweight';
    } else {
      category = 'Obese';
    }

    return {
      bmi: Math.round(bmi * 10) / 10,
      category
    };
  }

  /**
   * Convert temperature units
   */
  convertTemperature(value: number, fromUnit: '°C' | '°F', toUnit: '°C' | '°F'): number {
    if (fromUnit === toUnit) return value;

    if (fromUnit === '°C' && toUnit === '°F') {
      return (value * 9 / 5) + 32;
    } else {
      return (value - 32) * 5 / 9;
    }
  }

  /**
   * Register a new device
   */
  async registerDevice(registration: DeviceRegistration): Promise<{ success: boolean; deviceId: string }> {
    try {
      const response = await fetch(`${this.apiEndpoint}/devices/register`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-API-Key': this.apiKey
        },
        body: JSON.stringify(registration)
      });

      const data = await response.json();
      return {
        success: response.ok,
        deviceId: data.device_id
      };
    } catch (error) {
      throw new Error(`Failed to register device: ${error}`);
    }
  }

  /**
   * Send device reading to server
   */
  async sendReading(reading: MedicalIoTDevice): Promise<{ success: boolean; readingId: string }> {
    const validation = this.validateReading(reading);
    if (!validation.valid) {
      throw new Error(`Invalid reading: ${validation.errors.join(', ')}`);
    }

    try {
      const response = await fetch(`${this.apiEndpoint}/readings`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-API-Key': this.apiKey
        },
        body: JSON.stringify(reading)
      });

      const data = await response.json();
      return {
        success: response.ok,
        readingId: data.reading_id
      };
    } catch (error) {
      throw new Error(`Failed to send reading: ${error}`);
    }
  }

  /**
   * Get device status
   */
  async getDeviceStatus(deviceId: string): Promise<{
    device_id: string;
    status: DeviceStatus;
    last_reading: string;
    battery_level?: number;
  }> {
    try {
      const response = await fetch(`${this.apiEndpoint}/devices/${deviceId}/status`, {
        headers: {
          'X-API-Key': this.apiKey
        }
      });

      return await response.json();
    } catch (error) {
      throw new Error(`Failed to get device status: ${error}`);
    }
  }

  /**
   * Get patient readings
   */
  async getPatientReadings(
    patientId: string,
    startDate?: string,
    endDate?: string
  ): Promise<MedicalIoTDevice[]> {
    try {
      const params = new URLSearchParams({
        patient_id: patientId,
        ...(startDate && { start_date: startDate }),
        ...(endDate && { end_date: endDate })
      });

      const response = await fetch(`${this.apiEndpoint}/readings?${params}`, {
        headers: {
          'X-API-Key': this.apiKey
        }
      });

      return await response.json();
    } catch (error) {
      throw new Error(`Failed to get patient readings: ${error}`);
    }
  }

  /**
   * Send device command
   */
  async sendCommand(command: DeviceCommand): Promise<{ success: boolean; result: any }> {
    try {
      const response = await fetch(`${this.apiEndpoint}/devices/${command.device_id}/command`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-API-Key': this.apiKey
        },
        body: JSON.stringify(command)
      });

      const data = await response.json();
      return {
        success: response.ok,
        result: data
      };
    } catch (error) {
      throw new Error(`Failed to send command: ${error}`);
    }
  }

  /**
   * Helper to check valid ISO 8601 timestamp
   */
  private isValidTimestamp(timestamp: string): boolean {
    return !isNaN(Date.parse(timestamp));
  }

  /**
   * Generate device ID
   */
  static generateDeviceId(deviceType: DeviceType): string {
    const prefix = deviceType.toUpperCase().replace(/_/g, '-');
    const random = Math.random().toString(36).substr(2, 9).toUpperCase();
    return `DEV-${prefix}-${random}`;
  }

  /**
   * Generate alert ID
   */
  static generateAlertId(): string {
    return `ALERT-${Date.now()}-${Math.random().toString(36).substr(2, 9).toUpperCase()}`;
  }
}

export default MedicalIoTSDK;
