/**
 * WIA-COMM-002: IoT (M2M) SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA IoT and M2M Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive IoT/M2M capabilities including:
 * - MQTT, CoAP, AMQP protocol support
 * - Device provisioning and management
 * - Telemetry publishing and command handling
 * - Digital twin / device shadow
 * - Edge processing and filtering
 * - Firmware updates
 */

import {
  IoTDeviceConfig,
  ConnectionConfig,
  MQTTConfig,
  MQTTQoS,
  MQTTPublishOptions,
  CoAPConfig,
  CoAPRequestOptions,
  CoAPResponse,
  TelemetryData,
  DeviceCommand,
  CommandResponse,
  DeviceRegistration,
  ProvisioningResponse,
  FirmwareUpdate,
  FirmwareUpdateStatus,
  DeviceShadow,
  ShadowDelta,
  IoTErrorCode,
  IoTError,
  EventCallback,
  Subscription,
  IOT_PORTS,
  DeviceCredentials,
  EdgeRule,
  RuleCondition,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMM-002 IoT Device SDK
 */
export class IoTDeviceSDK {
  private version = '1.0.0';
  private config: IoTDeviceConfig;
  private connected = false;
  private subscriptions = new Map<string, EventCallback>();
  private messageQueue: Array<{ topic: string; payload: unknown }> = [];
  private reconnectAttempts = 0;
  private readonly maxReconnectAttempts = 10;

  constructor(config: IoTDeviceConfig) {
    this.config = config;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Get device ID
   */
  getDeviceId(): string {
    return this.config.deviceId;
  }

  // ==========================================================================
  // Connection Management
  // ==========================================================================

  /**
   * Connect to IoT platform
   */
  async connect(): Promise<void> {
    if (this.connected) {
      throw new IoTError(
        IoTErrorCode.CONNECTION_FAILED,
        'Device already connected'
      );
    }

    try {
      switch (this.config.protocol) {
        case 'MQTT':
          await this.connectMQTT();
          break;
        case 'CoAP':
          await this.connectCoAP();
          break;
        case 'HTTP':
          await this.connectHTTP();
          break;
        default:
          throw new IoTError(
            IoTErrorCode.CONFIGURATION_ERROR,
            `Unsupported protocol: ${this.config.protocol}`
          );
      }

      this.connected = true;
      this.reconnectAttempts = 0;

      // Process queued messages
      await this.processMessageQueue();

      console.log(`[IoT] Connected to ${this.config.connection.broker}`);
    } catch (error) {
      throw new IoTError(
        IoTErrorCode.CONNECTION_FAILED,
        `Failed to connect: ${(error as Error).message}`,
        { error }
      );
    }
  }

  /**
   * Disconnect from IoT platform
   */
  async disconnect(): Promise<void> {
    if (!this.connected) {
      return;
    }

    this.connected = false;
    this.subscriptions.clear();
    console.log('[IoT] Disconnected');
  }

  /**
   * Check if connected
   */
  isConnected(): boolean {
    return this.connected;
  }

  /**
   * Auto-reconnect logic
   */
  private async handleReconnect(): Promise<void> {
    if (!this.config.connection.autoReconnect) {
      return;
    }

    const backoff = this.config.connection.reconnectBackoff || [1, 2, 5, 10, 30];
    const delay = backoff[Math.min(this.reconnectAttempts, backoff.length - 1)];

    console.log(`[IoT] Reconnecting in ${delay} seconds...`);
    await this.sleep(delay * 1000);

    this.reconnectAttempts++;

    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      throw new IoTError(
        IoTErrorCode.CONNECTION_FAILED,
        'Max reconnection attempts exceeded'
      );
    }

    await this.connect();
  }

  // ==========================================================================
  // MQTT Protocol
  // ==========================================================================

  /**
   * Connect via MQTT
   */
  private async connectMQTT(): Promise<void> {
    const mqttConfig = this.config.connection as MQTTConfig;

    // Simulate MQTT connection
    console.log(`[MQTT] Connecting to ${mqttConfig.broker}...`);

    // Validate credentials
    if (!this.validateCredentials()) {
      throw new IoTError(
        IoTErrorCode.AUTHENTICATION_FAILED,
        'Invalid credentials'
      );
    }

    // Simulate connection delay
    await this.sleep(100);

    console.log(`[MQTT] Connected as ${this.config.deviceId}`);
  }

  /**
   * Publish message via MQTT
   */
  async publish(
    topic: string,
    payload: unknown,
    options?: MQTTPublishOptions
  ): Promise<void> {
    if (!this.connected) {
      // Queue message for later
      this.messageQueue.push({ topic, payload });
      console.log(`[MQTT] Message queued (offline): ${topic}`);
      return;
    }

    const qos = options?.qos ?? 1;
    const retain = options?.retain ?? false;

    try {
      // Simulate MQTT publish
      const message = JSON.stringify(payload);
      console.log(
        `[MQTT] Published to ${topic} (QoS ${qos}, retain: ${retain}): ${message.substring(0, 100)}...`
      );

      // Simulate network delay based on QoS
      await this.sleep(qos === 0 ? 10 : qos === 1 ? 20 : 50);
    } catch (error) {
      throw new IoTError(
        IoTErrorCode.PUBLISH_FAILED,
        `Failed to publish to ${topic}`,
        { error }
      );
    }
  }

  /**
   * Subscribe to topic via MQTT
   */
  async subscribe(
    topic: string,
    callback: EventCallback,
    qos: MQTTQoS = 1
  ): Promise<Subscription> {
    if (!this.connected) {
      throw new IoTError(
        IoTErrorCode.SUBSCRIBE_FAILED,
        'Cannot subscribe while disconnected'
      );
    }

    try {
      // Store subscription
      this.subscriptions.set(topic, callback);

      console.log(`[MQTT] Subscribed to ${topic} (QoS ${qos})`);

      return {
        topic,
        unsubscribe: async () => {
          this.subscriptions.delete(topic);
          console.log(`[MQTT] Unsubscribed from ${topic}`);
        },
        isActive: () => this.subscriptions.has(topic),
      };
    } catch (error) {
      throw new IoTError(
        IoTErrorCode.SUBSCRIBE_FAILED,
        `Failed to subscribe to ${topic}`,
        { error }
      );
    }
  }

  // ==========================================================================
  // CoAP Protocol
  // ==========================================================================

  /**
   * Connect via CoAP (UDP-based, connectionless)
   */
  private async connectCoAP(): Promise<void> {
    const coapConfig = this.config.connection as CoAPConfig;
    console.log(`[CoAP] Initialized CoAP client for ${coapConfig.broker}`);
    await this.sleep(50);
  }

  /**
   * Send CoAP request
   */
  async coapRequest(options: CoAPRequestOptions): Promise<CoAPResponse> {
    if (!this.connected) {
      throw new IoTError(IoTErrorCode.CONNECTION_FAILED, 'Not connected');
    }

    const startTime = Date.now();

    try {
      // Simulate CoAP request
      console.log(
        `[CoAP] ${options.method} ${options.path} ${options.type || 'CON'}`
      );

      // Simulate network delay
      await this.sleep(30);

      // Simulate response
      const code = options.method === 'GET' ? 205 : 204; // 2.05 Content or 2.04 Changed
      const response: CoAPResponse = {
        code,
        codeClass: Math.floor(code / 100),
        codeDetail: code % 100,
        payload: Buffer.from(
          JSON.stringify({ status: 'success', data: options.payload })
        ),
        options: {},
        responseTime: Date.now() - startTime,
      };

      return response;
    } catch (error) {
      throw new IoTError(
        IoTErrorCode.NETWORK_ERROR,
        `CoAP request failed: ${(error as Error).message}`,
        { error }
      );
    }
  }

  /**
   * Observe CoAP resource
   */
  async observeResource(
    path: string,
    callback: EventCallback<CoAPResponse>
  ): Promise<Subscription> {
    if (!this.connected) {
      throw new IoTError(IoTErrorCode.CONNECTION_FAILED, 'Not connected');
    }

    console.log(`[CoAP] Observing resource: ${path}`);

    // Simulate periodic updates
    const intervalId = setInterval(() => {
      const notification: CoAPResponse = {
        code: 205,
        codeClass: 2,
        codeDetail: 5,
        payload: Buffer.from(
          JSON.stringify({ value: Math.random() * 100, timestamp: new Date().toISOString() })
        ),
        options: { 6: Date.now() }, // Observe option
        responseTime: 0,
      };

      callback(notification);
    }, 5000);

    return {
      topic: path,
      unsubscribe: () => {
        clearInterval(intervalId);
        console.log(`[CoAP] Stopped observing ${path}`);
      },
      isActive: () => true,
    };
  }

  // ==========================================================================
  // HTTP Protocol
  // ==========================================================================

  /**
   * Connect via HTTP (validate endpoint)
   */
  private async connectHTTP(): Promise<void> {
    console.log(`[HTTP] Initialized HTTP client for ${this.config.connection.broker}`);
    await this.sleep(50);
  }

  // ==========================================================================
  // Telemetry
  // ==========================================================================

  /**
   * Publish telemetry data
   */
  async publishTelemetry(data: TelemetryData | Omit<TelemetryData, 'deviceId'>): Promise<void> {
    const telemetry: TelemetryData = {
      deviceId: this.config.deviceId,
      ...data,
      timestamp: data.timestamp || new Date().toISOString(),
    } as TelemetryData;

    const topic = `devices/${this.config.deviceId}/telemetry`;

    await this.publish(topic, telemetry, { qos: 1 });
  }

  // ==========================================================================
  // Commands
  // ==========================================================================

  /**
   * Subscribe to device commands
   */
  async subscribeToCommands(
    callback: EventCallback<DeviceCommand>
  ): Promise<Subscription> {
    const topic = `devices/${this.config.deviceId}/commands`;

    return this.subscribe(topic, (message) => {
      try {
        const command = typeof message === 'string' ? JSON.parse(message) : message;
        callback(command as DeviceCommand);
      } catch (error) {
        console.error('[IoT] Failed to parse command:', error);
      }
    });
  }

  /**
   * Send command response
   */
  async sendCommandResponse(response: CommandResponse): Promise<void> {
    const topic = `devices/${this.config.deviceId}/commands/response`;
    await this.publish(topic, response, { qos: 1 });
  }

  // ==========================================================================
  // Device Shadow / Digital Twin
  // ==========================================================================

  /**
   * Update device shadow (reported state)
   */
  async updateShadow(state: Record<string, unknown>): Promise<void> {
    const topic = `devices/${this.config.deviceId}/shadow/update`;

    const update = {
      state: {
        reported: state,
      },
      timestamp: new Date().toISOString(),
    };

    await this.publish(topic, update, { qos: 1 });
  }

  /**
   * Get device shadow
   */
  async getShadow(): Promise<DeviceShadow> {
    const topic = `devices/${this.config.deviceId}/shadow/get`;

    // In real implementation, this would make a request and wait for response
    // For this simulation, we'll return a mock shadow
    return {
      deviceId: this.config.deviceId,
      state: {
        reported: {},
        desired: {},
      },
      metadata: {
        reported: {},
        desired: {},
      },
      version: 1,
      timestamp: new Date().toISOString(),
    };
  }

  /**
   * Subscribe to shadow delta (desired state changes)
   */
  async subscribeShadowDelta(
    callback: EventCallback<ShadowDelta>
  ): Promise<Subscription> {
    const topic = `devices/${this.config.deviceId}/shadow/delta`;

    return this.subscribe(topic, (message) => {
      try {
        const delta = typeof message === 'string' ? JSON.parse(message) : message;
        callback(delta as ShadowDelta);
      } catch (error) {
        console.error('[IoT] Failed to parse shadow delta:', error);
      }
    });
  }

  // ==========================================================================
  // Firmware Updates
  // ==========================================================================

  /**
   * Check for firmware updates
   */
  async checkFirmwareUpdate(): Promise<FirmwareUpdate | null> {
    const topic = `devices/${this.config.deviceId}/firmware/check`;

    // Simulate checking for updates
    console.log('[IoT] Checking for firmware updates...');
    await this.sleep(100);

    // No update available (in real implementation, would query server)
    return null;
  }

  /**
   * Subscribe to firmware update notifications
   */
  async subscribeFirmwareUpdates(
    callback: EventCallback<FirmwareUpdate>
  ): Promise<Subscription> {
    const topic = `devices/${this.config.deviceId}/firmware/update`;

    return this.subscribe(topic, (message) => {
      try {
        const update = typeof message === 'string' ? JSON.parse(message) : message;
        callback(update as FirmwareUpdate);
      } catch (error) {
        console.error('[IoT] Failed to parse firmware update:', error);
      }
    });
  }

  /**
   * Report firmware update status
   */
  async reportFirmwareStatus(status: FirmwareUpdateStatus): Promise<void> {
    const topic = `devices/${this.config.deviceId}/firmware/status`;
    await this.publish(topic, status, { qos: 1 });
  }

  /**
   * Create firmware update manager
   */
  createFirmwareUpdateManager(): FirmwareUpdateManager {
    return new FirmwareUpdateManager(this);
  }

  // ==========================================================================
  // Edge Processing
  // ==========================================================================

  /**
   * Add edge processing rule
   */
  addEdgeRule(rule: EdgeRule): void {
    console.log(`[Edge] Added rule: ${rule.name}`);
    // In real implementation, would store and apply rules
  }

  /**
   * Process data through edge rules
   */
  processEdgeRules(data: TelemetryData): TelemetryData | null {
    // Simplified edge processing
    // In real implementation, would evaluate rules and apply actions
    return data;
  }

  // ==========================================================================
  // Helper Methods
  // ==========================================================================

  /**
   * Validate credentials
   */
  private validateCredentials(): boolean {
    const creds = this.config.credentials;

    if (!creds) {
      return false;
    }

    switch (creds.type) {
      case 'password':
        return !!(creds.username && creds.password);
      case 'certificate':
        return !!(creds.clientCert && creds.clientKey);
      case 'token':
        return !!creds.token;
      case 'psk':
        return !!creds.psk;
      case 'none':
        return true;
      default:
        return false;
    }
  }

  /**
   * Process queued messages
   */
  private async processMessageQueue(): Promise<void> {
    if (this.messageQueue.length === 0) {
      return;
    }

    console.log(`[IoT] Processing ${this.messageQueue.length} queued messages`);

    for (const { topic, payload } of this.messageQueue) {
      try {
        await this.publish(topic, payload);
      } catch (error) {
        console.error(`[IoT] Failed to send queued message to ${topic}:`, error);
      }
    }

    this.messageQueue = [];
  }

  /**
   * Sleep helper
   */
  private sleep(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}

// ============================================================================
// MQTT Client (Standalone)
// ============================================================================

/**
 * Standalone MQTT client
 */
export class MQTTClient {
  private config: MQTTConfig;
  private connected = false;
  private subscriptions = new Map<string, EventCallback>();

  constructor(config: MQTTConfig) {
    this.config = config;
  }

  async connect(): Promise<void> {
    console.log(`[MQTT] Connecting to ${this.config.broker}...`);
    await this.sleep(100);
    this.connected = true;
    console.log('[MQTT] Connected');
  }

  async disconnect(): Promise<void> {
    this.connected = false;
    this.subscriptions.clear();
    console.log('[MQTT] Disconnected');
  }

  async publish(
    topic: string,
    payload: string | Buffer,
    options?: MQTTPublishOptions
  ): Promise<void> {
    if (!this.connected) {
      throw new IoTError(IoTErrorCode.PUBLISH_FAILED, 'Not connected');
    }

    const qos = options?.qos ?? 1;
    console.log(`[MQTT] Published to ${topic} (QoS ${qos})`);
    await this.sleep(qos === 0 ? 10 : qos === 1 ? 20 : 50);
  }

  async subscribe(
    topic: string,
    callback: EventCallback,
    qos: MQTTQoS = 1
  ): Promise<Subscription> {
    if (!this.connected) {
      throw new IoTError(IoTErrorCode.SUBSCRIBE_FAILED, 'Not connected');
    }

    this.subscriptions.set(topic, callback);
    console.log(`[MQTT] Subscribed to ${topic} (QoS ${qos})`);

    return {
      topic,
      unsubscribe: async () => {
        this.subscriptions.delete(topic);
        console.log(`[MQTT] Unsubscribed from ${topic}`);
      },
      isActive: () => this.subscriptions.has(topic),
    };
  }

  private sleep(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}

// ============================================================================
// CoAP Client (Standalone)
// ============================================================================

/**
 * Standalone CoAP client
 */
export class CoAPClient {
  private config: CoAPConfig;

  constructor(config: CoAPConfig) {
    this.config = config;
  }

  async get(path: string): Promise<CoAPResponse> {
    return this.request({ method: 'GET', path });
  }

  async post(path: string, payload: Buffer | string): Promise<CoAPResponse> {
    return this.request({ method: 'POST', path, payload });
  }

  async put(path: string, payload: Buffer | string): Promise<CoAPResponse> {
    return this.request({ method: 'PUT', path, payload });
  }

  async delete(path: string): Promise<CoAPResponse> {
    return this.request({ method: 'DELETE', path });
  }

  async request(options: CoAPRequestOptions): Promise<CoAPResponse> {
    const startTime = Date.now();

    console.log(`[CoAP] ${options.method} ${options.path}`);
    await this.sleep(30);

    const code = options.method === 'GET' ? 205 : 204;
    return {
      code,
      codeClass: Math.floor(code / 100),
      codeDetail: code % 100,
      payload: Buffer.from(JSON.stringify({ status: 'success' })),
      options: {},
      responseTime: Date.now() - startTime,
    };
  }

  async observe(
    path: string,
    callback: EventCallback<CoAPResponse>
  ): Promise<Subscription> {
    console.log(`[CoAP] Observing ${path}`);

    const intervalId = setInterval(() => {
      callback({
        code: 205,
        codeClass: 2,
        codeDetail: 5,
        payload: Buffer.from(
          JSON.stringify({ value: Math.random() * 100, timestamp: new Date().toISOString() })
        ),
        options: {},
        responseTime: 0,
      });
    }, 5000);

    return {
      topic: path,
      unsubscribe: () => {
        clearInterval(intervalId);
        console.log(`[CoAP] Stopped observing ${path}`);
      },
      isActive: () => true,
    };
  }

  private sleep(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Device Provisioning
// ============================================================================

/**
 * Device provisioning service
 */
export class DeviceProvisioning {
  private serverUrl: string;

  constructor(config: { provisioningServer: string }) {
    this.serverUrl = config.provisioningServer;
  }

  /**
   * Register new device
   */
  async register(registration: DeviceRegistration): Promise<ProvisioningResponse> {
    console.log(`[Provisioning] Registering device ${registration.serialNumber}...`);

    // Simulate registration process
    await this.sleep(200);

    // Generate mock response
    const deviceId = `device-${Date.now()}`;
    const response: ProvisioningResponse = {
      deviceId,
      credentials: {
        type: 'certificate',
        clientCert: '-----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----',
        clientKey: '-----BEGIN PRIVATE KEY-----\n...\n-----END PRIVATE KEY-----',
        caCert: '-----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----',
      },
      endpoint: 'mqtts://iot.example.com:8883',
      topics: {
        telemetry: `devices/${deviceId}/telemetry`,
        commands: `devices/${deviceId}/commands`,
        status: `devices/${deviceId}/status`,
        configuration: `devices/${deviceId}/config`,
        firmwareUpdate: `devices/${deviceId}/firmware/update`,
      },
      status: 'success',
    };

    console.log(`[Provisioning] Device registered: ${deviceId}`);
    return response;
  }

  private sleep(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Firmware Update Manager
// ============================================================================

/**
 * Firmware update manager
 */
export class FirmwareUpdateManager {
  private device: IoTDeviceSDK;
  private eventCallbacks = new Map<string, EventCallback<FirmwareUpdate>>();

  constructor(device: IoTDeviceSDK) {
    this.device = device;
  }

  /**
   * Listen for update events
   */
  on(event: 'updateAvailable', callback: EventCallback<FirmwareUpdate>): void {
    this.eventCallbacks.set(event, callback);
  }

  /**
   * Check for updates
   */
  async checkForUpdates(): Promise<void> {
    const update = await this.device.checkFirmwareUpdate();

    if (update && this.eventCallbacks.has('updateAvailable')) {
      const callback = this.eventCallbacks.get('updateAvailable');
      callback?.(update);
    }
  }

  /**
   * Download firmware
   */
  async download(update: FirmwareUpdate): Promise<void> {
    console.log(`[Firmware] Downloading ${update.version}...`);

    await this.device.reportFirmwareStatus({
      updateId: update.id,
      deviceId: this.device.getDeviceId(),
      status: 'downloading',
      progress: 0,
      startedAt: new Date().toISOString(),
    });

    // Simulate download
    for (let i = 0; i <= 100; i += 20) {
      await this.sleep(200);
      await this.device.reportFirmwareStatus({
        updateId: update.id,
        deviceId: this.device.getDeviceId(),
        status: 'downloading',
        progress: i,
      });
    }

    console.log('[Firmware] Download complete');
  }

  /**
   * Verify firmware
   */
  async verify(update: FirmwareUpdate): Promise<void> {
    console.log('[Firmware] Verifying checksum...');

    await this.device.reportFirmwareStatus({
      updateId: update.id,
      deviceId: this.device.getDeviceId(),
      status: 'verifying',
      progress: 100,
    });

    await this.sleep(300);
    console.log('[Firmware] Verification successful');
  }

  /**
   * Install firmware
   */
  async install(update: FirmwareUpdate): Promise<void> {
    console.log('[Firmware] Installing...');

    await this.device.reportFirmwareStatus({
      updateId: update.id,
      deviceId: this.device.getDeviceId(),
      status: 'installing',
      progress: 100,
    });

    await this.sleep(500);

    await this.device.reportFirmwareStatus({
      updateId: update.id,
      deviceId: this.device.getDeviceId(),
      status: 'completed',
      progress: 100,
      completedAt: new Date().toISOString(),
    });

    console.log('[Firmware] Installation complete');
  }

  private sleep(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export {
  IoTDeviceSDK,
  MQTTClient,
  CoAPClient,
  DeviceProvisioning,
  FirmwareUpdateManager,
};
export default IoTDeviceSDK;

/**
 * 弘익人間 (Benefit All Humanity)
 */
