/**
 * WIA-IND-027: Industrial IoT - TypeScript SDK Implementation
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import { EventEmitter } from 'events';
import * as mqtt from 'mqtt';
import axios, { AxiosInstance } from 'axios';
import type {
  IndustrialIoTConfig,
  OPCUAConfig,
  MQTTConfig,
  ModbusConfig,
  TimeSeriesPoint,
  TimeSeriesQueryOptions,
  TimeSeriesResult,
  AlertRule,
  AlertEvent,
  SensorReading,
  DeviceInfo,
  OEEMetrics,
  ProductionMetrics,
  DigitalTwinConfig,
  DigitalTwinState,
  EdgeGatewayConfig,
  ProductionOrder,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * Industrial IoT SDK
 *
 * Main entry point for interacting with industrial IoT devices,
 * time-series data, alerts, and manufacturing systems.
 */
export class IndustrialIoTSDK extends EventEmitter {
  private config: IndustrialIoTConfig;
  private apiClient: AxiosInstance;
  private connections: Map<string, any> = new Map();

  constructor(config: IndustrialIoTConfig) {
    super();
    this.config = {
      timeout: 30000,
      debug: false,
      retry: {
        maxAttempts: 3,
        backoff: 'exponential',
        initialDelay: 1000,
      },
      ...config,
    };

    this.apiClient = axios.create({
      baseURL: this.config.apiEndpoint || 'https://api.wiastandards.com',
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
      },
    });

    this.log('IndustrialIoTSDK initialized', this.config);
  }

  // ==========================================================================
  // OPC-UA Integration
  // ==========================================================================

  /**
   * Connect to OPC-UA server
   */
  async connectOPCUA(config: OPCUAConfig): Promise<OPCUAClient> {
    this.log('Connecting to OPC-UA server:', config.endpoint);

    const client = new OPCUAClient(config, this);
    await client.connect();

    this.connections.set(`opcua:${config.endpoint}`, client);
    return client;
  }

  /**
   * Disconnect from OPC-UA server
   */
  async disconnectOPCUA(endpoint: string): Promise<void> {
    const client = this.connections.get(`opcua:${endpoint}`) as OPCUAClient;
    if (client) {
      await client.disconnect();
      this.connections.delete(`opcua:${endpoint}`);
    }
  }

  // ==========================================================================
  // MQTT Integration
  // ==========================================================================

  /**
   * Connect to MQTT broker
   */
  async connectMQTT(config: MQTTConfig): Promise<MQTTClient> {
    this.log('Connecting to MQTT broker:', config.broker);

    const client = new MQTTClient(config, this);
    await client.connect();

    this.connections.set(`mqtt:${config.broker}`, client);
    return client;
  }

  /**
   * Disconnect from MQTT broker
   */
  async disconnectMQTT(broker: string): Promise<void> {
    const client = this.connections.get(`mqtt:${broker}`) as MQTTClient;
    if (client) {
      await client.disconnect();
      this.connections.delete(`mqtt:${broker}`);
    }
  }

  // ==========================================================================
  // Time-Series Data
  // ==========================================================================

  /**
   * Write time-series data point
   */
  async writeTimeSeries(point: TimeSeriesPoint): Promise<void> {
    this.log('Writing time-series data:', point.measurement);

    await this.apiClient.post('/timeseries/write', {
      factoryId: this.config.factoryId,
      ...point,
    });
  }

  /**
   * Write multiple time-series data points
   */
  async writeTimeSeriesBatch(points: TimeSeriesPoint[]): Promise<void> {
    this.log('Writing batch time-series data:', points.length, 'points');

    await this.apiClient.post('/timeseries/write-batch', {
      factoryId: this.config.factoryId,
      points,
    });
  }

  /**
   * Query time-series data
   */
  async queryTimeSeries(options: TimeSeriesQueryOptions): Promise<TimeSeriesResult[]> {
    this.log('Querying time-series data:', options.measurement);

    const response = await this.apiClient.post('/timeseries/query', {
      factoryId: this.config.factoryId,
      ...options,
    });

    return response.data.results;
  }

  /**
   * Delete time-series data
   */
  async deleteTimeSeries(
    measurement: string,
    start: Date,
    end: Date,
    tags?: Record<string, string>
  ): Promise<void> {
    this.log('Deleting time-series data:', measurement);

    await this.apiClient.post('/timeseries/delete', {
      factoryId: this.config.factoryId,
      measurement,
      start,
      end,
      tags,
    });
  }

  // ==========================================================================
  // Alert Management
  // ==========================================================================

  /**
   * Create alert rule
   */
  async createAlertRule(rule: Omit<AlertRule, 'ruleId'>): Promise<AlertRule> {
    this.log('Creating alert rule:', rule.name);

    const response = await this.apiClient.post('/alerts/rules', {
      factoryId: this.config.factoryId,
      ...rule,
    });

    return response.data.rule;
  }

  /**
   * Get alert rule
   */
  async getAlertRule(ruleId: string): Promise<AlertRule> {
    const response = await this.apiClient.get(`/alerts/rules/${ruleId}`, {
      params: { factoryId: this.config.factoryId },
    });

    return response.data.rule;
  }

  /**
   * Update alert rule
   */
  async updateAlertRule(ruleId: string, updates: Partial<AlertRule>): Promise<AlertRule> {
    this.log('Updating alert rule:', ruleId);

    const response = await this.apiClient.patch(`/alerts/rules/${ruleId}`, {
      factoryId: this.config.factoryId,
      ...updates,
    });

    return response.data.rule;
  }

  /**
   * Delete alert rule
   */
  async deleteAlertRule(ruleId: string): Promise<void> {
    this.log('Deleting alert rule:', ruleId);

    await this.apiClient.delete(`/alerts/rules/${ruleId}`, {
      params: { factoryId: this.config.factoryId },
    });
  }

  /**
   * List alert rules
   */
  async listAlertRules(): Promise<AlertRule[]> {
    const response = await this.apiClient.get('/alerts/rules', {
      params: { factoryId: this.config.factoryId },
    });

    return response.data.rules;
  }

  /**
   * Send alert
   */
  async sendAlert(alert: Omit<AlertEvent, 'alertId' | 'status'>): Promise<AlertEvent> {
    this.log('Sending alert:', alert.message);

    const response = await this.apiClient.post('/alerts/events', {
      factoryId: this.config.factoryId,
      ...alert,
    });

    this.emit('alert', response.data.alert);
    return response.data.alert;
  }

  /**
   * Get active alerts
   */
  async getActiveAlerts(): Promise<AlertEvent[]> {
    const response = await this.apiClient.get('/alerts/events', {
      params: {
        factoryId: this.config.factoryId,
        status: 'active',
      },
    });

    return response.data.alerts;
  }

  /**
   * Acknowledge alert
   */
  async acknowledgeAlert(alertId: string, acknowledgedBy: string): Promise<AlertEvent> {
    this.log('Acknowledging alert:', alertId);

    const response = await this.apiClient.post(`/alerts/events/${alertId}/acknowledge`, {
      factoryId: this.config.factoryId,
      acknowledgedBy,
    });

    return response.data.alert;
  }

  // ==========================================================================
  // Device Management
  // ==========================================================================

  /**
   * Register device
   */
  async registerDevice(device: Omit<DeviceInfo, 'status' | 'lastSeen'>): Promise<DeviceInfo> {
    this.log('Registering device:', device.deviceId);

    const response = await this.apiClient.post('/devices', {
      factoryId: this.config.factoryId,
      ...device,
    });

    return response.data.device;
  }

  /**
   * Get device information
   */
  async getDevice(deviceId: string): Promise<DeviceInfo> {
    const response = await this.apiClient.get(`/devices/${deviceId}`, {
      params: { factoryId: this.config.factoryId },
    });

    return response.data.device;
  }

  /**
   * List devices
   */
  async listDevices(filters?: { type?: string; protocol?: string; status?: string }): Promise<DeviceInfo[]> {
    const response = await this.apiClient.get('/devices', {
      params: {
        factoryId: this.config.factoryId,
        ...filters,
      },
    });

    return response.data.devices;
  }

  /**
   * Update device status
   */
  async updateDeviceStatus(deviceId: string, status: DeviceInfo['status']): Promise<void> {
    await this.apiClient.patch(`/devices/${deviceId}`, {
      factoryId: this.config.factoryId,
      status,
      lastSeen: new Date(),
    });
  }

  /**
   * Update device firmware
   */
  async updateDeviceFirmware(
    deviceId: string,
    firmwareVersion: string,
    firmwareUrl: string
  ): Promise<void> {
    this.log('Updating device firmware:', deviceId, 'to', firmwareVersion);

    await this.apiClient.post(`/devices/${deviceId}/firmware`, {
      factoryId: this.config.factoryId,
      firmwareVersion,
      firmwareUrl,
    });
  }

  // ==========================================================================
  // Digital Twin
  // ==========================================================================

  /**
   * Create digital twin
   */
  async createDigitalTwin(config: DigitalTwinConfig): Promise<DigitalTwin> {
    this.log('Creating digital twin:', config.assetId);

    const response = await this.apiClient.post('/digital-twins', {
      factoryId: this.config.factoryId,
      ...config,
    });

    const twin = new DigitalTwin(response.data.twin, this);
    return twin;
  }

  /**
   * Get digital twin
   */
  async getDigitalTwin(assetId: string): Promise<DigitalTwin> {
    const response = await this.apiClient.get(`/digital-twins/${assetId}`, {
      params: { factoryId: this.config.factoryId },
    });

    return new DigitalTwin(response.data.twin, this);
  }

  // ==========================================================================
  // Manufacturing Metrics
  // ==========================================================================

  /**
   * Calculate OEE metrics
   */
  calculateOEE(params: {
    plannedTime: number;
    downtime: number;
    targetOutput: number;
    actualOutput: number;
    goodUnits: number;
  }): OEEMetrics {
    const operatingTime = params.plannedTime - params.downtime;
    const availability = operatingTime / params.plannedTime;
    const performance = params.actualOutput / params.targetOutput;
    const quality = params.goodUnits / params.actualOutput;
    const oee = availability * performance * quality;

    return {
      availability: availability * 100,
      performance: performance * 100,
      quality: quality * 100,
      oee: oee * 100,
      plannedTime: params.plannedTime,
      operatingTime,
      downtime: params.downtime,
      targetOutput: params.targetOutput,
      actualOutput: params.actualOutput,
      goodUnits: params.goodUnits,
      defectiveUnits: params.actualOutput - params.goodUnits,
      timestamp: new Date(),
    };
  }

  /**
   * Get production metrics
   */
  async getProductionMetrics(lineId: string, timeRange?: { start: Date; end: Date }): Promise<ProductionMetrics> {
    const response = await this.apiClient.get(`/production/metrics/${lineId}`, {
      params: {
        factoryId: this.config.factoryId,
        ...timeRange,
      },
    });

    return response.data.metrics;
  }

  // ==========================================================================
  // Edge Gateway
  // ==========================================================================

  /**
   * Connect to edge gateway
   */
  connectEdgeGateway(config: EdgeGatewayConfig): EdgeGateway {
    this.log('Connecting to edge gateway:', config.gatewayId);

    const gateway = new EdgeGateway(config, this);
    this.connections.set(`edge:${config.gatewayId}`, gateway);

    return gateway;
  }

  // ==========================================================================
  // Production Orders (MES Integration)
  // ==========================================================================

  /**
   * Create production order
   */
  async createProductionOrder(order: Omit<ProductionOrder, 'orderId' | 'status'>): Promise<ProductionOrder> {
    this.log('Creating production order:', order.productCode);

    const response = await this.apiClient.post('/production/orders', {
      factoryId: this.config.factoryId,
      ...order,
      status: 'pending',
    });

    return response.data.order;
  }

  /**
   * Get production order
   */
  async getProductionOrder(orderId: string): Promise<ProductionOrder> {
    const response = await this.apiClient.get(`/production/orders/${orderId}`, {
      params: { factoryId: this.config.factoryId },
    });

    return response.data.order;
  }

  /**
   * Update production order status
   */
  async updateProductionOrderStatus(
    orderId: string,
    status: ProductionOrder['status']
  ): Promise<ProductionOrder> {
    const response = await this.apiClient.patch(`/production/orders/${orderId}`, {
      factoryId: this.config.factoryId,
      status,
    });

    return response.data.order;
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  /**
   * Close all connections
   */
  async close(): Promise<void> {
    this.log('Closing all connections');

    for (const [key, connection] of this.connections) {
      if (connection.disconnect) {
        await connection.disconnect();
      }
    }

    this.connections.clear();
  }

  /**
   * Internal logging
   */
  private log(...args: any[]): void {
    if (this.config.debug) {
      console.log('[WIA-IND-027]', ...args);
    }
  }
}

// ============================================================================
// OPC-UA Client Class
// ============================================================================

export class OPCUAClient extends EventEmitter {
  private config: OPCUAConfig;
  private sdk: IndustrialIoTSDK;
  private connected: boolean = false;
  private subscriptions: Map<string, any> = new Map();

  constructor(config: OPCUAConfig, sdk: IndustrialIoTSDK) {
    super();
    this.config = config;
    this.sdk = sdk;
  }

  async connect(): Promise<void> {
    // Simulated connection - in production, use node-opcua library
    this.connected = true;
    this.emit('connected');
  }

  async disconnect(): Promise<void> {
    this.connected = false;
    this.subscriptions.clear();
    this.emit('disconnected');
  }

  async readNode(params: { nodeId: string; interval?: number }): Promise<{ value: any; timestamp: Date }> {
    if (!this.connected) throw new Error('Not connected to OPC-UA server');

    // Simulated read - in production, use actual OPC-UA client
    return {
      value: Math.random() * 100,
      timestamp: new Date(),
    };
  }

  async writeNode(params: { nodeId: string; value: any }): Promise<void> {
    if (!this.connected) throw new Error('Not connected to OPC-UA server');

    // Simulated write
  }

  async subscribe(params: {
    nodeId: string;
    samplingInterval?: number;
    onChange: (data: { value: any; timestamp: Date }) => void;
  }): Promise<void> {
    if (!this.connected) throw new Error('Not connected to OPC-UA server');

    // Simulated subscription
    const interval = setInterval(() => {
      const data = {
        value: Math.random() * 100,
        timestamp: new Date(),
      };
      params.onChange(data);
    }, params.samplingInterval || 1000);

    this.subscriptions.set(params.nodeId, interval);
  }

  async unsubscribe(nodeId: string): Promise<void> {
    const interval = this.subscriptions.get(nodeId);
    if (interval) {
      clearInterval(interval);
      this.subscriptions.delete(nodeId);
    }
  }
}

// ============================================================================
// MQTT Client Class
// ============================================================================

export class MQTTClient extends EventEmitter {
  private config: MQTTConfig;
  private sdk: IndustrialIoTSDK;
  private client: mqtt.MqttClient | null = null;

  constructor(config: MQTTConfig, sdk: IndustrialIoTSDK) {
    super();
    this.config = config;
    this.sdk = sdk;
  }

  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.client = mqtt.connect(this.config.broker, {
        clientId: this.config.clientId,
        username: this.config.username,
        password: this.config.password,
        clean: this.config.cleanSession,
        keepalive: this.config.keepAlive || 60,
        reconnectPeriod: this.config.reconnectPeriod || 1000,
        will: this.config.will,
      });

      this.client.on('connect', () => {
        this.emit('connected');
        resolve();
      });

      this.client.on('error', (error) => {
        this.emit('error', error);
        reject(error);
      });

      this.client.on('message', (topic, message) => {
        this.emit('message', topic, message);
      });
    });
  }

  async disconnect(): Promise<void> {
    if (this.client) {
      this.client.end();
      this.client = null;
    }
  }

  async publish(topic: string, message: string | Buffer, options?: mqtt.IClientPublishOptions): Promise<void> {
    if (!this.client) throw new Error('Not connected to MQTT broker');

    return new Promise((resolve, reject) => {
      this.client!.publish(topic, message, options || {}, (error) => {
        if (error) reject(error);
        else resolve();
      });
    });
  }

  async subscribe(topic: string | string[], options?: mqtt.IClientSubscribeOptions): Promise<void> {
    if (!this.client) throw new Error('Not connected to MQTT broker');

    return new Promise((resolve, reject) => {
      this.client!.subscribe(topic, options || {}, (error) => {
        if (error) reject(error);
        else resolve();
      });
    });
  }

  async unsubscribe(topic: string | string[]): Promise<void> {
    if (!this.client) throw new Error('Not connected to MQTT broker');

    return new Promise((resolve, reject) => {
      this.client!.unsubscribe(topic, (error) => {
        if (error) reject(error);
        else resolve();
      });
    });
  }

  on(event: 'message', listener: (topic: string, payload: Buffer) => void): this;
  on(event: string, listener: (...args: any[]) => void): this {
    return super.on(event, listener);
  }
}

// ============================================================================
// Digital Twin Class
// ============================================================================

export class DigitalTwin extends EventEmitter {
  private config: DigitalTwinConfig;
  private sdk: IndustrialIoTSDK;
  private state: DigitalTwinState;
  private syncInterval: NodeJS.Timeout | null = null;

  constructor(config: DigitalTwinConfig, sdk: IndustrialIoTSDK) {
    super();
    this.config = config;
    this.sdk = sdk;
    this.state = {
      assetId: config.assetId,
      sensorData: {},
      calculatedProperties: {},
      health: { overall: 100, subsystems: {} },
      anomalies: [],
      lastUpdate: new Date(),
    };
  }

  async sync(options: { liveData?: boolean; updateInterval?: number; predictiveMaintenance?: boolean }): Promise<void> {
    if (options.liveData) {
      const interval = options.updateInterval || this.config.updateInterval || 5000;

      this.syncInterval = setInterval(async () => {
        await this.updateState();
        this.emit('updated', this.state);
      }, interval);
    } else {
      await this.updateState();
    }
  }

  async updateState(): Promise<void> {
    // Simulated sensor data update
    for (const sensor of this.config.sensors) {
      this.state.sensorData[sensor.id] = Math.random() * 100;
    }

    this.state.lastUpdate = new Date();
  }

  async predict(options: { metric: string; horizon: string }): Promise<{ value: number; confidence: number }> {
    // Simulated prediction
    return {
      value: Math.random() * 100,
      confidence: 0.85,
    };
  }

  getState(): DigitalTwinState {
    return { ...this.state };
  }

  stopSync(): void {
    if (this.syncInterval) {
      clearInterval(this.syncInterval);
      this.syncInterval = null;
    }
  }
}

// ============================================================================
// Edge Gateway Class
// ============================================================================

export class EdgeGateway extends EventEmitter {
  private config: EdgeGatewayConfig;
  private sdk: IndustrialIoTSDK;

  constructor(config: EdgeGatewayConfig, sdk: IndustrialIoTSDK) {
    super();
    this.config = config;
    this.sdk = sdk;
  }

  async monitorLine(params: {
    lineId: string;
    metrics: string[];
    interval: number;
  }): Promise<ProductionMonitor> {
    const monitor = new ProductionMonitor(params, this.sdk);
    await monitor.start();
    return monitor;
  }
}

// ============================================================================
// Production Monitor Class
// ============================================================================

export class ProductionMonitor extends EventEmitter {
  private params: { lineId: string; metrics: string[]; interval: number };
  private sdk: IndustrialIoTSDK;
  private monitorInterval: NodeJS.Timeout | null = null;

  constructor(params: { lineId: string; metrics: string[]; interval: number }, sdk: IndustrialIoTSDK) {
    super();
    this.params = params;
    this.sdk = sdk;
  }

  async start(): Promise<void> {
    this.monitorInterval = setInterval(async () => {
      const metrics = await this.collectMetrics();
      this.emit('data', metrics);
    }, this.params.interval);
  }

  async stop(): Promise<void> {
    if (this.monitorInterval) {
      clearInterval(this.monitorInterval);
      this.monitorInterval = null;
    }
  }

  private async collectMetrics(): Promise<Record<string, any>> {
    // Simulated metrics collection
    return {
      oee: 75 + Math.random() * 20,
      throughput: 100 + Math.random() * 50,
      quality: 90 + Math.random() * 10,
      downtime: Math.random() * 30,
    };
  }
}

// ============================================================================
// Export everything
// ============================================================================

export * from './types';

export default IndustrialIoTSDK;

/**
 * 弘益人間 (Benefit All Humanity)
 */
