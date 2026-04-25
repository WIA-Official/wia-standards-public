/**
 * WIA-SEMI-019 Semiconductor Equipment Standard SDK
 * Version: 1.0.0
 *
 * 弘益人間 · Benefit All Humanity
 *
 * @packageDocumentation
 */

import axios, { AxiosInstance } from 'axios';
import WebSocket from 'ws';
import * as Types from './types';

export * from './types';

/**
 * WIA Semiconductor Equipment Client
 *
 * @example
 * ```typescript
 * const client = new WIAEquipmentClient({
 *   baseURL: 'https://equipment-001.fab.com',
 *   apiKey: 'your-api-key'
 * });
 *
 * const spec = await client.getEquipmentInfo();
 * console.log(spec.equipment.manufacturer);
 * ```
 */
export class WIAEquipmentClient {
  private client: AxiosInstance;
  private config: Types.WIAConfig;

  constructor(config: Types.WIAConfig) {
    this.config = config;
    this.client = axios.create({
      baseURL: `${config.baseURL}/api/v1`,
      timeout: config.timeout || 30000,
      headers: config.apiKey
        ? { Authorization: `Bearer ${config.apiKey}` }
        : {},
    });
  }

  // ==================== Equipment Information ====================

  /**
   * Get equipment specification (Phase 1)
   */
  async getEquipmentInfo(): Promise<Types.EquipmentSpecification> {
    const response = await this.client.get('/equipment/info');
    return response.data;
  }

  /**
   * Get current equipment status
   */
  async getEquipmentStatus(): Promise<Types.EquipmentStatus> {
    const response = await this.client.get('/equipment/status');
    return response.data;
  }

  // ==================== Parameters ====================

  /**
   * Get all current parameter values
   */
  async getParameters(): Promise<Types.Parameter[]> {
    const response = await this.client.get('/parameters');
    return response.data.parameters;
  }

  /**
   * Get specific parameter value
   */
  async getParameter(parameterId: string): Promise<Types.Parameter> {
    const response = await this.client.get(`/parameters/${parameterId}`);
    return response.data;
  }

  /**
   * Get parameter definition/metadata
   */
  async getParameterDefinition(
    parameterId: string
  ): Promise<Types.ParameterDefinition> {
    const response = await this.client.get(
      `/parameters/${parameterId}/definition`
    );
    return response.data;
  }

  // ==================== Commands ====================

  /**
   * Send command to equipment
   */
  async sendCommand(
    command: Types.Command
  ): Promise<Types.CommandResponse> {
    const response = await this.client.post('/command', command);
    return response.data;
  }

  /**
   * Get command status
   */
  async getCommandStatus(
    commandId: string
  ): Promise<Types.CommandResponse> {
    const response = await this.client.get(`/command/${commandId}/status`);
    return response.data;
  }

  // ==================== Alarms ====================

  /**
   * Get all active alarms
   */
  async getActiveAlarms(): Promise<Types.Alarm[]> {
    const response = await this.client.get('/alarms?status=active');
    return response.data.alarms;
  }

  /**
   * Get specific alarm
   */
  async getAlarm(alarmId: string): Promise<Types.Alarm> {
    const response = await this.client.get(`/alarms/${alarmId}`);
    return response.data;
  }

  /**
   * Acknowledge alarm
   */
  async acknowledgeAlarm(alarmId: string): Promise<void> {
    await this.client.post(`/alarms/${alarmId}/acknowledge`);
  }

  // ==================== Events ====================

  /**
   * Get events with optional filters
   */
  async getEvents(options?: {
    startTime?: string;
    endTime?: string;
    type?: string;
    limit?: number;
  }): Promise<Types.Event[]> {
    const params = new URLSearchParams();
    if (options?.startTime) params.append('start_time', options.startTime);
    if (options?.endTime) params.append('end_time', options.endTime);
    if (options?.type) params.append('type', options.type);
    if (options?.limit) params.append('limit', options.limit.toString());

    const response = await this.client.get(`/events?${params.toString()}`);
    return response.data.events;
  }

  // ==================== Recipes ====================

  /**
   * Get all recipes
   */
  async getRecipes(): Promise<Types.Recipe[]> {
    const response = await this.client.get('/recipes');
    return response.data.recipes;
  }

  /**
   * Get specific recipe
   */
  async getRecipe(recipeId: string): Promise<Types.Recipe> {
    const response = await this.client.get(`/recipes/${recipeId}`);
    return response.data;
  }

  /**
   * Upload new recipe
   */
  async uploadRecipe(recipe: Types.Recipe): Promise<{ recipe_id: string }> {
    const response = await this.client.post('/recipes', recipe);
    return response.data;
  }

  /**
   * Update existing recipe
   */
  async updateRecipe(
    recipeId: string,
    recipe: Types.Recipe
  ): Promise<void> {
    await this.client.put(`/recipes/${recipeId}`, recipe);
  }

  /**
   * Delete recipe
   */
  async deleteRecipe(recipeId: string): Promise<void> {
    await this.client.delete(`/recipes/${recipeId}`);
  }

  // ==================== WebSocket Streaming ====================

  /**
   * Create WebSocket connection for real-time parameter streaming
   *
   * @example
   * ```typescript
   * const ws = client.streamParameters(['temp', 'pressure'], 100, (data) => {
   *   console.log(data);
   * });
   * ```
   */
  streamParameters(
    parameters: string[],
    frequencyHz: number,
    onData: (data: Types.WebSocketDataMessage) => void,
    onError?: (error: Error) => void
  ): WebSocket {
    const wsUrl = this.config.baseURL.replace(/^http/, 'ws');
    const ws = new WebSocket(`${wsUrl}/api/v1/stream`);

    ws.on('open', () => {
      const subscribeMsg: Types.WebSocketSubscribeMessage = {
        action: 'subscribe',
        parameters,
        frequency_hz: frequencyHz,
        format: 'json',
      };
      ws.send(JSON.stringify(subscribeMsg));
    });

    ws.on('message', (data: string) => {
      try {
        const message = JSON.parse(data.toString());
        onData(message);
      } catch (error) {
        onError?.(error as Error);
      }
    });

    ws.on('error', (error) => {
      onError?.(error);
    });

    return ws;
  }
}

/**
 * Helper functions for common operations
 */
export class WIAHelpers {
  /**
   * Calculate throughput (WPH)
   */
  static calculateThroughput(
    processTimeSeconds: number,
    loadTimeSeconds: number,
    overheadTimeSeconds: number,
    uptimePercent: number
  ): {
    theoretical_wph: number;
    effective_wph: number;
  } {
    const totalCycleTime =
      processTimeSeconds + loadTimeSeconds + overheadTimeSeconds;
    const theoretical = 3600 / totalCycleTime;
    const effective = theoretical * (uptimePercent / 100);

    return {
      theoretical_wph: Number(theoretical.toFixed(2)),
      effective_wph: Number(effective.toFixed(2)),
    };
  }

  /**
   * Calculate Cpk (process capability index)
   */
  static calculateCpk(
    mean: number,
    stdDev: number,
    lsl: number,
    usl: number
  ): number {
    const cpkLower = (mean - lsl) / (3 * stdDev);
    const cpkUpper = (usl - mean) / (3 * stdDev);
    return Number(Math.min(cpkLower, cpkUpper).toFixed(2));
  }

  /**
   * Validate equipment specification against schema
   */
  static validateEquipmentSpec(
    spec: Types.EquipmentSpecification
  ): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (spec.standard !== 'WIA-SEMI-019') {
      errors.push('Invalid standard identifier');
    }

    if (!spec.equipment.manufacturer) {
      errors.push('Missing manufacturer');
    }

    if (!spec.equipment.model) {
      errors.push('Missing model');
    }

    if (!spec.equipment.type) {
      errors.push('Missing equipment type');
    }

    return {
      valid: errors.length === 0,
      errors,
    };
  }
}

/**
 * Constants
 */
export const WIA_CONSTANTS = {
  VERSION: '1.0.0',
  STANDARD: 'WIA-SEMI-019',
  CERTIFICATION_LEVELS: ['Bronze', 'Silver', 'Gold', 'Platinum'] as const,
  EQUIPMENT_STATES: [
    'IDLE',
    'SETUP',
    'READY',
    'EXECUTING',
    'PAUSED',
    'ALARM',
    'MAINTENANCE',
    'OFFLINE',
  ] as const,
  SVID_RANGES: {
    TEMPERATURE: { min: 4000, max: 4999 },
    PRESSURE: { min: 5000, max: 5999 },
    FLOW: { min: 6000, max: 6999 },
    POWER: { min: 7000, max: 7999 },
    MOTION: { min: 8000, max: 8999 },
    METROLOGY: { min: 9000, max: 9999 },
  },
};

/**
 * Default export
 */
export default WIAEquipmentClient;
