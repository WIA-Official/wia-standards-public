/**
 * WIA-AMR Standard SDK
 * @module @wia/amr-sdk
 * @version 1.0.0
 * @description TypeScript SDK for WIA-AMR Standard compliance
 */

import Ajv, { ValidateFunction } from 'ajv';
import addFormats from 'ajv-formats';
import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import * as mqtt from 'mqtt';
import WebSocket from 'ws';

// Re-export types
export * from './types';

import {
  RobotState,
  Task,
  Command,
  CommandAck,
  Map,
  Telemetry,
  WiaEvent,
  ApiResponse,
  FleetSummary,
  Position,
  TaskStatus,
  CommandType,
} from './types';

// ============================================================================
// Schema Validation
// ============================================================================

/**
 * Schema validator for WIA-AMR data types
 */
export class SchemaValidator {
  private ajv: Ajv;
  private validators: Map<string, ValidateFunction> = new Map();

  constructor() {
    this.ajv = new Ajv({ allErrors: true, strict: false });
    addFormats(this.ajv);
  }

  /**
   * Register a JSON schema
   */
  registerSchema(name: string, schema: object): void {
    const validate = this.ajv.compile(schema);
    this.validators.set(name, validate);
  }

  /**
   * Validate data against a registered schema
   */
  validate(schemaName: string, data: unknown): { valid: boolean; errors?: string[] } {
    const validator = this.validators.get(schemaName);
    if (!validator) {
      return { valid: false, errors: [`Schema '${schemaName}' not found`] };
    }

    const valid = validator(data);
    if (!valid) {
      const errors = validator.errors?.map(
        (e) => `${e.instancePath} ${e.message}`
      );
      return { valid: false, errors };
    }

    return { valid: true };
  }

  /**
   * Validate RobotState
   */
  validateRobotState(data: unknown): data is RobotState {
    // Basic validation without full schema
    const state = data as RobotState;
    return (
      typeof state.robotId === 'string' &&
      typeof state.position?.x === 'number' &&
      typeof state.position?.y === 'number' &&
      typeof state.position?.theta === 'number' &&
      typeof state.operatingState === 'string' &&
      typeof state.timestamp === 'string'
    );
  }

  /**
   * Validate Task
   */
  validateTask(data: unknown): data is Task {
    const task = data as Task;
    return (
      typeof task.taskId === 'string' &&
      typeof task.taskType === 'string'
    );
  }
}

// ============================================================================
// REST API Client
// ============================================================================

/**
 * Configuration for WIA-AMR API Client
 */
export interface WiaAmrClientConfig {
  /** Base URL of the API */
  baseUrl: string;
  /** API key or OAuth token */
  authToken?: string;
  /** Request timeout in ms */
  timeout?: number;
  /** Custom headers */
  headers?: Record<string, string>;
}

/**
 * WIA-AMR REST API Client
 */
export class WiaAmrClient {
  private http: AxiosInstance;

  constructor(config: WiaAmrClientConfig) {
    this.http = axios.create({
      baseURL: config.baseUrl,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        ...(config.authToken && { Authorization: `Bearer ${config.authToken}` }),
        ...config.headers,
      },
    });
  }

  // Robot Management

  /**
   * List all robots
   */
  async listRobots(params?: {
    status?: string;
    limit?: number;
    offset?: number;
  }): Promise<ApiResponse<RobotState[]>> {
    const response = await this.http.get('/api/v1/robots', { params });
    return response.data;
  }

  /**
   * Get robot by ID
   */
  async getRobot(robotId: string): Promise<RobotState> {
    const response = await this.http.get(`/api/v1/robots/${robotId}`);
    return response.data;
  }

  /**
   * Get robot current state
   */
  async getRobotState(robotId: string): Promise<RobotState> {
    const response = await this.http.get(`/api/v1/robots/${robotId}/state`);
    return response.data;
  }

  /**
   * Send command to robot
   */
  async sendCommand(
    robotId: string,
    command: Omit<Command, 'commandId' | 'timestamp'>
  ): Promise<CommandAck> {
    const response = await this.http.post(`/api/v1/robots/${robotId}/commands`, {
      ...command,
      timestamp: new Date().toISOString(),
    });
    return response.data;
  }

  // Task Management

  /**
   * Create a new task
   */
  async createTask(task: Omit<Task, 'taskId' | 'status'>): Promise<Task> {
    const response = await this.http.post('/api/v1/tasks', task);
    return response.data;
  }

  /**
   * Get task by ID
   */
  async getTask(taskId: string): Promise<Task> {
    const response = await this.http.get(`/api/v1/tasks/${taskId}`);
    return response.data;
  }

  /**
   * List tasks
   */
  async listTasks(params?: {
    status?: TaskStatus | TaskStatus[];
    robotId?: string;
    limit?: number;
    offset?: number;
  }): Promise<ApiResponse<Task[]>> {
    const response = await this.http.get('/api/v1/tasks', { params });
    return response.data;
  }

  /**
   * Cancel a task
   */
  async cancelTask(taskId: string): Promise<Task> {
    const response = await this.http.delete(`/api/v1/tasks/${taskId}`);
    return response.data;
  }

  // Map Management

  /**
   * List maps
   */
  async listMaps(): Promise<ApiResponse<Map[]>> {
    const response = await this.http.get('/api/v1/maps');
    return response.data;
  }

  /**
   * Get map by ID
   */
  async getMap(mapId: string): Promise<Map> {
    const response = await this.http.get(`/api/v1/maps/${mapId}`);
    return response.data;
  }

  // Fleet Management

  /**
   * Get fleet summary
   */
  async getFleetSummary(): Promise<FleetSummary> {
    const response = await this.http.get('/api/v1/fleet/summary');
    return response.data;
  }
}

// ============================================================================
// MQTT Client
// ============================================================================

/**
 * MQTT client configuration
 */
export interface MqttClientConfig {
  /** Broker URL (mqtt:// or mqtts://) */
  brokerUrl: string;
  /** Client ID */
  clientId?: string;
  /** Username */
  username?: string;
  /** Password or token */
  password?: string;
  /** Keep alive interval in seconds */
  keepAlive?: number;
}

/**
 * MQTT message handler
 */
export type MqttMessageHandler<T = unknown> = (
  topic: string,
  message: T,
  packet: mqtt.IPublishPacket
) => void;

/**
 * WIA-AMR MQTT Client for real-time communication
 */
export class WiaAmrMqttClient {
  private client: mqtt.MqttClient | null = null;
  private config: MqttClientConfig;
  private handlers: Map<string, MqttMessageHandler[]> = new Map();

  constructor(config: MqttClientConfig) {
    this.config = config;
  }

  /**
   * Connect to MQTT broker
   */
  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.client = mqtt.connect(this.config.brokerUrl, {
        clientId: this.config.clientId || `wia-amr-${Date.now()}`,
        username: this.config.username,
        password: this.config.password,
        keepalive: this.config.keepAlive || 30,
        clean: false,
        will: {
          topic: '/wia/amr/v1/clients/disconnected',
          payload: JSON.stringify({ clientId: this.config.clientId }),
          qos: 1,
          retain: false,
        },
      });

      this.client.on('connect', () => {
        console.log('Connected to MQTT broker');
        resolve();
      });

      this.client.on('error', (error) => {
        console.error('MQTT error:', error);
        reject(error);
      });

      this.client.on('message', (topic, payload, packet) => {
        this.handleMessage(topic, payload, packet);
      });
    });
  }

  /**
   * Disconnect from broker
   */
  async disconnect(): Promise<void> {
    return new Promise((resolve) => {
      if (this.client) {
        this.client.end(false, {}, () => {
          resolve();
        });
      } else {
        resolve();
      }
    });
  }

  /**
   * Subscribe to a topic
   */
  subscribe(topic: string, handler: MqttMessageHandler, qos: 0 | 1 | 2 = 1): void {
    if (!this.client) throw new Error('Not connected');

    this.client.subscribe(topic, { qos });

    const handlers = this.handlers.get(topic) || [];
    handlers.push(handler);
    this.handlers.set(topic, handlers);
  }

  /**
   * Unsubscribe from a topic
   */
  unsubscribe(topic: string): void {
    if (!this.client) throw new Error('Not connected');

    this.client.unsubscribe(topic);
    this.handlers.delete(topic);
  }

  /**
   * Publish a message
   */
  publish(topic: string, message: unknown, qos: 0 | 1 | 2 = 1): void {
    if (!this.client) throw new Error('Not connected');

    const payload = typeof message === 'string' ? message : JSON.stringify(message);
    this.client.publish(topic, payload, { qos });
  }

  /**
   * Subscribe to robot state updates
   */
  subscribeRobotState(robotId: string, handler: MqttMessageHandler<RobotState>): void {
    const topic = `/wia/amr/v1/robots/${robotId}/state`;
    this.subscribe(topic, handler as MqttMessageHandler);
  }

  /**
   * Subscribe to all robot states
   */
  subscribeAllRobotStates(handler: MqttMessageHandler<RobotState>): void {
    const topic = '/wia/amr/v1/robots/+/state';
    this.subscribe(topic, handler as MqttMessageHandler);
  }

  /**
   * Publish command to robot
   */
  publishCommand(robotId: string, command: Command): void {
    const topic = `/wia/amr/v1/robots/${robotId}/commands`;
    this.publish(topic, command, 2);
  }

  private handleMessage(
    topic: string,
    payload: Buffer,
    packet: mqtt.IPublishPacket
  ): void {
    let message: unknown;
    try {
      message = JSON.parse(payload.toString());
    } catch {
      message = payload.toString();
    }

    // Check exact match
    let handlers = this.handlers.get(topic);

    // Check wildcard matches
    if (!handlers) {
      for (const [pattern, h] of this.handlers) {
        if (this.matchTopic(pattern, topic)) {
          handlers = h;
          break;
        }
      }
    }

    if (handlers) {
      handlers.forEach((handler) => handler(topic, message, packet));
    }
  }

  private matchTopic(pattern: string, topic: string): boolean {
    const patternParts = pattern.split('/');
    const topicParts = topic.split('/');

    for (let i = 0; i < patternParts.length; i++) {
      if (patternParts[i] === '#') return true;
      if (patternParts[i] === '+') continue;
      if (patternParts[i] !== topicParts[i]) return false;
    }

    return patternParts.length === topicParts.length;
  }
}

// ============================================================================
// WebSocket Client
// ============================================================================

/**
 * WebSocket client configuration
 */
export interface WebSocketClientConfig {
  /** WebSocket URL (ws:// or wss://) */
  url: string;
  /** Authentication token */
  authToken?: string;
  /** Reconnect on disconnect */
  autoReconnect?: boolean;
  /** Reconnect interval in ms */
  reconnectInterval?: number;
}

/**
 * WebSocket message handler
 */
export type WsMessageHandler<T = unknown> = (message: T) => void;

/**
 * WIA-AMR WebSocket Client for real-time dashboards
 */
export class WiaAmrWebSocketClient {
  private ws: WebSocket | null = null;
  private config: WebSocketClientConfig;
  private handlers: Map<string, WsMessageHandler[]> = new Map();
  private reconnecting = false;

  constructor(config: WebSocketClientConfig) {
    this.config = config;
  }

  /**
   * Connect to WebSocket server
   */
  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.ws = new WebSocket(this.config.url);

      this.ws.on('open', () => {
        if (this.config.authToken) {
          this.send({ type: 'AUTH', token: this.config.authToken });
        }
        resolve();
      });

      this.ws.on('error', (error) => {
        reject(error);
      });

      this.ws.on('message', (data) => {
        this.handleMessage(data.toString());
      });

      this.ws.on('close', () => {
        if (this.config.autoReconnect && !this.reconnecting) {
          this.reconnect();
        }
      });
    });
  }

  /**
   * Disconnect from server
   */
  disconnect(): void {
    this.reconnecting = false;
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  /**
   * Send a message
   */
  send(message: unknown): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket not connected');
    }
    this.ws.send(JSON.stringify(message));
  }

  /**
   * Subscribe to channels
   */
  subscribe(channels: string[]): void {
    this.send({ type: 'SUBSCRIBE', channels });
  }

  /**
   * Register message handler
   */
  on(messageType: string, handler: WsMessageHandler): void {
    const handlers = this.handlers.get(messageType) || [];
    handlers.push(handler);
    this.handlers.set(messageType, handlers);
  }

  private handleMessage(data: string): void {
    try {
      const message = JSON.parse(data);
      const handlers = this.handlers.get(message.type);
      if (handlers) {
        handlers.forEach((handler) => handler(message));
      }
    } catch (error) {
      console.error('Failed to parse WebSocket message:', error);
    }
  }

  private reconnect(): void {
    this.reconnecting = true;
    const interval = this.config.reconnectInterval || 5000;

    setTimeout(async () => {
      try {
        await this.connect();
        this.reconnecting = false;
      } catch {
        this.reconnect();
      }
    }, interval);
  }
}

// ============================================================================
// VDA 5050 Adapter
// ============================================================================

/**
 * Adapter for VDA 5050 compatibility
 */
export class VDA5050Adapter {
  /**
   * Convert WIA-AMR Task to VDA 5050 Order
   */
  taskToOrder(task: Task, config: { manufacturer: string; serialNumber: string }): object {
    return {
      headerId: Date.now(),
      timestamp: new Date().toISOString(),
      version: '2.0.0',
      manufacturer: config.manufacturer,
      serialNumber: config.serialNumber,
      orderId: task.taskId,
      orderUpdateId: 0,
      nodes: task.destination
        ? [
            {
              nodeId: 'destination',
              sequenceId: 0,
              released: true,
              nodePosition: {
                x: task.destination.x,
                y: task.destination.y,
                theta: task.destination.theta,
                mapId: task.destination.mapId,
              },
              actions: task.actions?.map((a, i) => ({
                actionId: a.actionId || `action-${i}`,
                actionType: a.actionType.toLowerCase(),
                blockingType: a.blockingType || 'HARD',
                actionParameters: Object.entries(a.parameters || {}).map(([key, value]) => ({
                  key,
                  value: String(value),
                })),
              })) || [],
            },
          ]
        : [],
      edges: [],
    };
  }

  /**
   * Convert VDA 5050 State to WIA-AMR RobotState
   */
  stateToRobotState(vdaState: any): RobotState {
    return {
      robotId: vdaState.serialNumber,
      manufacturer: vdaState.manufacturer,
      position: {
        x: vdaState.agvPosition?.x || 0,
        y: vdaState.agvPosition?.y || 0,
        theta: vdaState.agvPosition?.theta || 0,
        mapId: vdaState.agvPosition?.mapId,
        positionInitialized: vdaState.agvPosition?.positionInitialized,
        localizationScore: vdaState.agvPosition?.localizationScore,
      },
      velocity: {
        vx: vdaState.velocity?.vx,
        vy: vdaState.velocity?.vy,
        omega: vdaState.velocity?.omega,
      },
      battery: {
        level: vdaState.batteryState?.batteryCharge || 0,
        voltage: vdaState.batteryState?.batteryVoltage,
        charging: vdaState.batteryState?.charging,
        health: vdaState.batteryState?.batteryHealth,
      },
      operatingState: this.mapOperatingMode(vdaState.operatingMode),
      safetyState: this.mapSafetyState(vdaState.safetyState),
      driving: vdaState.driving,
      paused: vdaState.paused,
      currentTaskId: vdaState.orderId,
      errors: vdaState.errors?.map((e: any) => ({
        errorId: e.errorType,
        errorType: e.errorType,
        errorLevel: e.errorLevel || 'ERROR',
        errorDescription: e.errorDescription,
      })) || [],
      timestamp: vdaState.timestamp,
    };
  }

  private mapOperatingMode(mode: string): RobotState['operatingState'] {
    const mapping: Record<string, RobotState['operatingState']> = {
      AUTOMATIC: 'NAVIGATING',
      SEMIAUTOMATIC: 'NAVIGATING',
      MANUAL: 'MANUAL',
      SERVICE: 'PAUSED',
      TEACHIN: 'MANUAL',
    };
    return mapping[mode] || 'IDLE';
  }

  private mapSafetyState(state: any): RobotState['safetyState'] {
    if (state?.eStop === 'AUTOACK' || state?.eStop === 'MANUAL') {
      return 'EMERGENCY_STOP';
    }
    if (state?.fieldViolation) {
      return 'PROTECTIVE_STOP';
    }
    return 'SAFE';
  }
}

// ============================================================================
// Exports
// ============================================================================

export default {
  SchemaValidator,
  WiaAmrClient,
  WiaAmrMqttClient,
  WiaAmrWebSocketClient,
  VDA5050Adapter,
};
