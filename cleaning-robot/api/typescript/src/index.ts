/**
 * WIA-ROB-011 Cleaning Robot Standard API
 * TypeScript Implementation
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-rob-011
 * @license MIT
 * @author WIA / SmileStory Inc.
 */

import * as types from './types';

// Re-export all types
export * from './types';

/**
 * Main API client for WIA-ROB-011 cleaning robots
 */
export class WiaCleaningRobot {
  private baseUrl: string;
  private token?: string;
  private timeout: number;

  constructor(options: types.ApiClientOptions) {
    const protocol = options.protocol || 'http';
    const port = options.port || 8080;
    this.baseUrl = `${protocol}://${options.host}:${port}/api/v1`;
    this.token = options.token;
    this.timeout = options.timeout || 5000;
  }

  /**
   * Connect to the robot
   */
  async connect(): Promise<boolean> {
    try {
      const response = await this.fetch('/health');
      return response.status === 'healthy';
    } catch (error) {
      throw new Error(`Failed to connect: ${error}`);
    }
  }

  /**
   * Get current robot status
   */
  async getStatus(): Promise<types.RobotState> {
    const response = await this.fetch('/robot/status');
    return response.state;
  }

  /**
   * Get robot identity information
   */
  async getIdentity(): Promise<types.RobotIdentity> {
    const response = await this.fetch('/robot/info');
    return response;
  }

  /**
   * Start cleaning
   */
  async startCleaning(params: types.StartCleaningParams): Promise<types.CommandResponse> {
    return await this.fetch('/robot/clean', {
      method: 'POST',
      body: JSON.stringify(params)
    });
  }

  /**
   * Stop cleaning
   */
  async stopCleaning(): Promise<types.CommandResponse> {
    return await this.fetch('/robot/stop', { method: 'POST' });
  }

  /**
   * Pause cleaning
   */
  async pauseCleaning(): Promise<types.CommandResponse> {
    return await this.fetch('/robot/pause', { method: 'POST' });
  }

  /**
   * Resume cleaning
   */
  async resumeCleaning(): Promise<types.CommandResponse> {
    return await this.fetch('/robot/resume', { method: 'POST' });
  }

  /**
   * Return to dock
   */
  async returnToDock(): Promise<types.CommandResponse> {
    return await this.fetch('/robot/dock', { method: 'POST' });
  }

  /**
   * Get current map
   */
  async getMap(floor?: number): Promise<types.OccupancyGrid> {
    const url = floor !== undefined ? `/robot/map?floor=${floor}` : '/robot/map';
    return await this.fetch(url);
  }

  /**
   * Get cleaning history
   */
  async getHistory(limit = 10, offset = 0): Promise<types.CleaningSession[]> {
    const response = await this.fetch(`/robot/history?limit=${limit}&offset=${offset}`);
    return response.sessions;
  }

  /**
   * Get cleaning schedule
   */
  async getSchedule(): Promise<types.CleaningSchedule> {
    return await this.fetch('/robot/schedule');
  }

  /**
   * Update cleaning schedule
   */
  async updateSchedule(schedule: types.CleaningSchedule): Promise<types.CommandResponse> {
    return await this.fetch('/robot/schedule', {
      method: 'PUT',
      body: JSON.stringify(schedule)
    });
  }

  /**
   * Get robot configuration
   */
  async getConfig(): Promise<types.RobotConfig> {
    return await this.fetch('/robot/config');
  }

  /**
   * Update robot configuration
   */
  async updateConfig(config: Partial<types.RobotConfig>): Promise<types.CommandResponse> {
    return await this.fetch('/robot/config', {
      method: 'PUT',
      body: JSON.stringify(config)
    });
  }

  /**
   * Get room information
   */
  async getRooms(): Promise<types.Room[]> {
    const response = await this.fetch('/robot/map/rooms');
    return response.rooms;
  }

  /**
   * Update room information
   */
  async updateRoom(roomId: string, room: Partial<types.Room>): Promise<types.CommandResponse> {
    return await this.fetch(`/robot/map/rooms/${roomId}`, {
      method: 'PUT',
      body: JSON.stringify(room)
    });
  }

  /**
   * Subscribe to real-time events via WebSocket
   */
  subscribeToEvents(callback: types.EventCallback, options?: types.WebSocketOptions): () => void {
    const wsUrl = this.baseUrl.replace(/^http/, 'ws').replace('/api/v1', '/ws');
    const ws = new WebSocket(wsUrl);

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.type === 'event') {
          callback(data.data as types.RobotEvent);
        }
      } catch (error) {
        console.error('Failed to parse WebSocket message:', error);
      }
    };

    ws.onopen = () => {
      // Subscribe to all event channels
      ws.send(JSON.stringify({
        type: 'subscribe',
        channel: 'events'
      }));
    };

    if (options?.reconnect) {
      let reconnectAttempts = 0;
      ws.onclose = () => {
        if (reconnectAttempts < (options.maxReconnectAttempts || Infinity)) {
          setTimeout(() => {
            reconnectAttempts++;
            this.subscribeToEvents(callback, options);
          }, options.reconnectInterval || 5000);
        }
      };
    }

    // Return unsubscribe function
    return () => ws.close();
  }

  /**
   * Internal fetch wrapper with authentication
   */
  private async fetch(endpoint: string, options: RequestInit = {}): Promise<any> {
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
      ...options.headers
    };

    if (this.token) {
      headers['Authorization'] = `Bearer ${this.token}`;
    }

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.timeout);

    try {
      const response = await fetch(`${this.baseUrl}${endpoint}`, {
        ...options,
        headers,
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.error?.message || `HTTP ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      clearTimeout(timeoutId);
      throw error;
    }
  }
}

/**
 * Create a new cleaning robot client instance
 */
export function createRobot(options: types.ApiClientOptions): WiaCleaningRobot {
  return new WiaCleaningRobot(options);
}

/**
 * Version of the SDK
 */
export const VERSION = '1.0.0';

/**
 * Default export
 */
export default WiaCleaningRobot;
