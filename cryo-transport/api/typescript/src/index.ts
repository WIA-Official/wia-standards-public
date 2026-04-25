/**
 * WIA-CRYO-009 TypeScript SDK
 *
 * Official SDK for cryo transport standards compliance
 *
 * @packageDocumentation
 */

import axios, { AxiosInstance } from 'axios';
import { ethers } from 'ethers';
import * as Types from './types';

export * from './types';

/**
 * Main SDK class for WIA-CRYO-009 compliance
 */
export class CryoTransportSDK {
  private api: AxiosInstance;
  private config: Types.CryoTransportConfig;
  private blockchain?: ethers.Contract;

  constructor(config: Types.CryoTransportConfig) {
    this.config = config;

    // Initialize API client
    this.api = axios.create({
      baseURL: config.apiEndpoint,
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'CRYO-009-v2.0'
      }
    });

    // Initialize blockchain if configured
    if (config.blockchain?.enabled) {
      this.initBlockchain(config.blockchain);
    }
  }

  /**
   * Initialize blockchain connection
   */
  private async initBlockchain(config: Types.BlockchainConfig): Promise<void> {
    try {
      const provider = new ethers.JsonRpcProvider(config.rpcUrl);
      const signer = config.privateKey
        ? new ethers.Wallet(config.privateKey, provider)
        : undefined;

      const abi = [
        'function recordTransfer(string memory transportId, address from, address to, string memory metadata) public returns (bytes32)',
        'function getTransferHistory(string memory transportId) public view returns (tuple(address from, address to, uint256 timestamp, string metadata)[])',
        'event CustodyTransferred(string indexed transportId, address indexed from, address indexed to, uint256 timestamp)'
      ];

      this.blockchain = new ethers.Contract(
        config.contractAddress,
        abi,
        signer || provider
      );
    } catch (error) {
      console.error('Failed to initialize blockchain:', error);
    }
  }

  /**
   * Create a new transport plan
   */
  async createTransport(plan: Types.TransportPlan): Promise<Types.APIResponse<{ transportId: string }>> {
    try {
      const response = await this.api.post('/transport', plan);
      return {
        success: true,
        data: response.data,
        timestamp: new Date()
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Get current transport status
   */
  async getStatus(transportId: string): Promise<Types.APIResponse<Types.TransportStatus>> {
    try {
      const response = await this.api.get(`/transport/${transportId}/status`);
      return {
        success: true,
        data: response.data,
        timestamp: new Date()
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Submit sensor data package
   */
  async submitSensorData(data: Types.SensorDataPackage): Promise<Types.APIResponse<void>> {
    try {
      await this.api.post('/sensor-data', data);
      return {
        success: true,
        timestamp: new Date()
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Record a custody transfer
   */
  async recordCustodyTransfer(transfer: Types.CustodyTransfer): Promise<Types.APIResponse<{ txHash?: string }>> {
    try {
      // Submit to API
      const response = await this.api.post('/custody-transfer', transfer);

      // Record on blockchain if configured
      let txHash: string | undefined;
      if (this.blockchain) {
        const metadata = JSON.stringify({
          location: transfer.location,
          temperature: transfer.temperature.celsius,
          sealNumbers: transfer.containerSealNumbers
        });

        const tx = await this.blockchain.recordTransfer(
          transfer.id,
          transfer.fromPersonnel.biometricId || transfer.fromPersonnel.id,
          transfer.toPersonnel.biometricId || transfer.toPersonnel.id,
          metadata
        );

        const receipt = await tx.wait();
        txHash = receipt.hash;
      }

      return {
        success: true,
        data: { txHash },
        timestamp: new Date()
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Get custody transfer history
   */
  async getCustodyHistory(transportId: string): Promise<Types.APIResponse<Types.CustodyTransfer[]>> {
    try {
      const response = await this.api.get(`/transport/${transportId}/custody-history`);

      // Verify with blockchain if configured
      if (this.blockchain) {
        const chainHistory = await this.blockchain.getTransferHistory(transportId);
        // Could add verification logic here
      }

      return {
        success: true,
        data: response.data,
        timestamp: new Date()
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Report an event
   */
  async reportEvent(transportId: string, event: Types.TransportEvent): Promise<Types.APIResponse<void>> {
    try {
      await this.api.post(`/transport/${transportId}/event`, event);
      return {
        success: true,
        timestamp: new Date()
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Get temperature prediction (AI-powered)
   */
  async getTemperaturePrediction(transportId: string): Promise<Types.APIResponse<Types.TemperaturePrediction>> {
    try {
      const response = await this.api.get(`/transport/${transportId}/prediction/temperature`);
      return {
        success: true,
        data: response.data,
        timestamp: new Date()
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Optimize route
   */
  async optimizeRoute(
    origin: Types.GPSPosition,
    destination: Types.GPSPosition,
    options?: {
      mode?: Types.TransportMode;
      priority?: 'fastest' | 'safest' | 'cost' | 'balanced';
    }
  ): Promise<Types.APIResponse<Types.RouteOptimization>> {
    try {
      const response = await this.api.post('/route/optimize', {
        origin,
        destination,
        ...options
      });
      return {
        success: true,
        data: response.data,
        timestamp: new Date()
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Get transport summary (post-completion)
   */
  async getTransportSummary(transportId: string): Promise<Types.APIResponse<Types.TransportSummary>> {
    try {
      const response = await this.api.get(`/transport/${transportId}/summary`);
      return {
        success: true,
        data: response.data,
        timestamp: new Date()
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Validate temperature reading against standards
   */
  validateTemperature(temp: number): Types.TemperatureZone {
    if (temp >= -185 && temp <= -130) return Types.TemperatureZone.ACCEPTABLE;
    if (temp > -130) return Types.TemperatureZone.CRITICAL;
    if (temp < -196 || temp > -185) return Types.TemperatureZone.WARNING;
    return Types.TemperatureZone.OPTIMAL;
  }

  /**
   * Calculate required LN2 for transport
   */
  calculateLN2Requirements(
    duration: number,        // hours
    containerCapacity: number, // liters
    ambientTemp: number = 25   // °C
  ): {
    theoretical: number;
    recommended: number;
    safetyMargin: number;
  } {
    // Simplified boil-off calculation
    const boilOffRate = 0.8 + (ambientTemp / 50); // L/hour
    const theoretical = boilOffRate * duration;

    // Apply 3-5x safety factor (using 4x)
    const recommended = theoretical * 4;

    // Ensure doesn't exceed 80% of capacity
    const maxFill = containerCapacity * 0.8;
    const actual = Math.min(recommended, maxFill);

    return {
      theoretical,
      recommended: actual,
      safetyMargin: actual - theoretical
    };
  }

  /**
   * Check compliance level requirements
   */
  checkCompliance(
    plan: Types.TransportPlan,
    actualData: {
      temperatureAccuracy: number;
      gpsAccuracy: number;
      updateFrequency: number; // seconds
      hasBiometric: boolean;
      hasBlockchain: boolean;
      hasBackupSensors: boolean;
    }
  ): {
    level: Types.ComplianceLevel;
    meets: boolean;
    gaps: string[];
  } {
    const gaps: string[] = [];
    const required = plan.complianceLevel;

    // Level 1 requirements
    if (actualData.temperatureAccuracy > 0.5) {
      gaps.push('Temperature accuracy must be ±0.5°C or better');
    }

    // Level 2 requirements
    if (required === Types.ComplianceLevel.LEVEL_2_STANDARD ||
        required === Types.ComplianceLevel.LEVEL_3_PREMIUM) {
      if (actualData.temperatureAccuracy > 0.1) {
        gaps.push('Level 2+ requires temperature accuracy ±0.1°C');
      }
      if (actualData.gpsAccuracy > 5) {
        gaps.push('Level 2+ requires GPS accuracy ±5m');
      }
      if (actualData.updateFrequency > 60) {
        gaps.push('Level 2+ requires updates every 60 seconds or less');
      }
      if (!actualData.hasBiometric) {
        gaps.push('Level 2+ requires biometric authentication');
      }
    }

    // Level 3 requirements
    if (required === Types.ComplianceLevel.LEVEL_3_PREMIUM) {
      if (actualData.temperatureAccuracy > 0.05) {
        gaps.push('Level 3 requires temperature accuracy ±0.05°C');
      }
      if (actualData.gpsAccuracy > 2) {
        gaps.push('Level 3 requires GPS accuracy ±2m');
      }
      if (actualData.updateFrequency > 30) {
        gaps.push('Level 3 requires updates every 30 seconds or less');
      }
      if (!actualData.hasBackupSensors) {
        gaps.push('Level 3 requires redundant backup sensors');
      }
    }

    return {
      level: required,
      meets: gaps.length === 0,
      gaps
    };
  }

  /**
   * Error handler
   */
  private handleError(error: any): Types.APIResponse<never> {
    return {
      success: false,
      error: {
        code: error.response?.data?.code || 'UNKNOWN_ERROR',
        message: error.response?.data?.message || error.message,
        details: error.response?.data
      },
      timestamp: new Date()
    };
  }
}

/**
 * Helper function to create SDK instance
 */
export function createCryoTransportSDK(config: Types.CryoTransportConfig): CryoTransportSDK {
  return new CryoTransportSDK(config);
}

/**
 * Constants for WIA-CRYO-009 standard
 */
export const CONSTANTS = {
  TEMPERATURE: {
    OPTIMAL_MIN: -196,
    OPTIMAL_MAX: -185,
    ACCEPTABLE_MIN: -185,
    ACCEPTABLE_MAX: -140,
    WARNING_MIN: -140,
    WARNING_MAX: -130,
    CRITICAL_THRESHOLD: -130
  },
  G_FORCE: {
    CONTINUOUS_MAX: 0.5,
    PERIODIC_MAX: 2.0,
    SHOCK_MAX: 5.0,
    ABSOLUTE_MAX: 10.0
  },
  BOILING_POINT_LN2: -195.8,
  DEVITRIFICATION_TEMP: -135,
  STANDARD_VERSION: '2.0.0'
} as const;

/**
 * Utility functions
 */
export const utils = {
  /**
   * Convert Celsius to Kelvin
   */
  celsiusToKelvin(celsius: number): number {
    return celsius + 273.15;
  },

  /**
   * Calculate distance between two GPS points (Haversine formula)
   */
  calculateDistance(pos1: Types.GPSPosition, pos2: Types.GPSPosition): number {
    const R = 6371; // Earth's radius in km
    const dLat = (pos2.latitude - pos1.latitude) * Math.PI / 180;
    const dLon = (pos2.longitude - pos1.longitude) * Math.PI / 180;
    const a =
      Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(pos1.latitude * Math.PI / 180) * Math.cos(pos2.latitude * Math.PI / 180) *
      Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
  },

  /**
   * Check if position is within geofence
   */
  isWithinGeofence(position: Types.GPSPosition, geofence: Types.Geofence): boolean {
    if (geofence.radius) {
      // Circular geofence
      const center = geofence.coordinates[0];
      const distance = this.calculateDistance(position, center) * 1000; // Convert to meters
      return distance <= geofence.radius;
    } else {
      // Polygon geofence (point-in-polygon algorithm)
      // Simplified ray casting algorithm
      let inside = false;
      const coords = geofence.coordinates;
      for (let i = 0, j = coords.length - 1; i < coords.length; j = i++) {
        const xi = coords[i].latitude, yi = coords[i].longitude;
        const xj = coords[j].latitude, yj = coords[j].longitude;
        const intersect = ((yi > position.longitude) !== (yj > position.longitude))
          && (position.latitude < (xj - xi) * (position.longitude - yi) / (yj - yi) + xi);
        if (intersect) inside = !inside;
      }
      return inside;
    }
  },

  /**
   * Generate unique transport ID
   */
  generateTransportId(): string {
    const prefix = 'TRN';
    const timestamp = Date.now().toString(36).toUpperCase();
    const random = Math.random().toString(36).substr(2, 5).toUpperCase();
    return `${prefix}-${timestamp}-${random}`;
  }
};

// Default export
export default CryoTransportSDK;
