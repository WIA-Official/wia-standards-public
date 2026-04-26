/**
 * WIA Drone (UAV) Standard - TypeScript SDK
 * Version: 1.0
 * Philosophy: 弘益人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

import EventEmitter from 'eventemitter3';
import {
  WIAConfig,
  APIResponse,
  Drone,
  Telemetry,
  Mission,
  FlightPlan,
  RemoteID,
} from './types';

export * from './types';

export class WIADroneClient {
  private config: Required<WIAConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAConfig) {
    this.config = {
      baseURL: 'https://api.wia-standards.org/v1/drone-uav',
      timeout: 60000,
      debug: false,
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  /**
   * Get drone details by ID
   */
  async getDrone(id: string): Promise<APIResponse<Drone>> {
    return this.makeRequest('GET', `/drones/${id}`);
  }

  /**
   * Register a new drone
   */
  async registerDrone(drone: Partial<Drone>): Promise<APIResponse<Drone>> {
    return this.makeRequest('POST', '/drones', drone);
  }

  /**
   * Update drone information
   */
  async updateDrone(id: string, updates: Partial<Drone>): Promise<APIResponse<Drone>> {
    return this.makeRequest('PUT', `/drones/${id}`, updates);
  }

  /**
   * Get real-time telemetry
   */
  async getTelemetry(droneId: string): Promise<APIResponse<Telemetry>> {
    return this.makeRequest('GET', `/drones/${droneId}/telemetry`);
  }

  /**
   * Stream telemetry (WebSocket simulation)
   */
  streamTelemetry(droneId: string, callback: (telemetry: Telemetry) => void): void {
    // In production, this would establish WebSocket connection
    this.eventEmitter.on(`telemetry:${droneId}`, callback);
    if (this.config.debug) {
      console.log(`[WIA Drone] Streaming telemetry for ${droneId}`);
    }
  }

  /**
   * Stop telemetry stream
   */
  stopTelemetryStream(droneId: string): void {
    this.eventEmitter.removeAllListeners(`telemetry:${droneId}`);
  }

  /**
   * Create flight mission
   */
  async createMission(mission: Partial<Mission>): Promise<APIResponse<Mission>> {
    return this.makeRequest('POST', '/missions', mission);
  }

  /**
   * Upload mission to drone
   */
  async uploadMission(droneId: string, missionId: string): Promise<APIResponse<void>> {
    return this.makeRequest('POST', `/drones/${droneId}/missions/${missionId}/upload`);
  }

  /**
   * Start mission execution
   */
  async startMission(droneId: string, missionId: string): Promise<APIResponse<void>> {
    return this.makeRequest('POST', `/drones/${droneId}/missions/${missionId}/start`);
  }

  /**
   * Submit flight plan to UTM
   */
  async submitFlightPlan(flightPlan: FlightPlan): Promise<APIResponse<{ approved: boolean }>> {
    return this.makeRequest('POST', '/utm/flight-plans', flightPlan);
  }

  /**
   * Broadcast Remote ID
   */
  async broadcastRemoteID(remoteId: RemoteID): Promise<APIResponse<void>> {
    return this.makeRequest('POST', '/remote-id/broadcast', remoteId);
  }

  /**
   * Send command to drone
   */
  async sendCommand(
    droneId: string,
    command: string,
    params?: any
  ): Promise<APIResponse<void>> {
    return this.makeRequest('POST', `/drones/${droneId}/commands`, { command, params });
  }

  /**
   * Emergency stop
   */
  async emergencyStop(droneId: string): Promise<APIResponse<void>> {
    return this.makeRequest('POST', `/drones/${droneId}/emergency-stop`);
  }

  /**
   * Return to launch
   */
  async returnToLaunch(droneId: string): Promise<APIResponse<void>> {
    return this.sendCommand(droneId, 'RTL');
  }

  /**
   * Subscribe to events
   */
  on(event: string, callback: (...args: any[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  /**
   * Emit events
   */
  private emit(event: string, ...args: any[]): void {
    this.eventEmitter.emit(event, ...args);
  }

  /**
   * Make HTTP request
   */
  private async makeRequest<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<APIResponse<T>> {
    const url = `${this.config.baseURL}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Drone] ${method} ${url}`, body);
    }

    try {
      this.emit('request', { method, path, body });

      return {
        success: true,
        data: body as T,
      };
    } catch (error: any) {
      this.emit('error', error);

      return {
        success: false,
        error: {
          code: 'REQUEST_FAILED',
          message: error.message,
        },
      };
    }
  }
}

/**
 * Flight planning utilities
 */
export class FlightPlanner {
  /**
   * Calculate flight time
   */
  static calculateFlightTime(distance: number, speed: number): number {
    return (distance / speed) * 60; // minutes
  }

  /**
   * Calculate battery usage
   */
  static calculateBatteryUsage(
    flightTime: number,
    batteryCapacity: number,
    powerConsumption: number
  ): number {
    const batteryWh = (batteryCapacity * 11.1) / 1000; // Assuming 11.1V
    const maxFlightTime = (batteryWh / powerConsumption) * 60;
    return (flightTime / maxFlightTime) * 100;
  }

  /**
   * Calculate safe range with battery reserve
   */
  static calculateSafeRange(
    batteryCapacity: number,
    powerConsumption: number,
    speed: number,
    reservePercent = 20
  ): number {
    const batteryWh = (batteryCapacity * 11.1) / 1000;
    const maxFlightTime = (batteryWh / powerConsumption) * 60;
    const safeFlightTime = maxFlightTime * ((100 - reservePercent) / 100);
    return (safeFlightTime / 60) * speed;
  }

  /**
   * Check if mission is feasible
   */
  static isMissionFeasible(
    distance: number,
    speed: number,
    batteryCapacity: number,
    powerConsumption: number
  ): boolean {
    const flightTime = this.calculateFlightTime(distance, speed);
    const batteryUsage = this.calculateBatteryUsage(
      flightTime,
      batteryCapacity,
      powerConsumption
    );
    return batteryUsage <= 80; // 20% reserve
  }
}
