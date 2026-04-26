/**
 * WIA-COMM-018: Low-Power Network - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides a unified interface for LPWAN technologies:
 * - LoRaWAN
 * - Sigfox
 * - NB-IoT
 * - LTE-M
 */

import {
  Technology,
  DeviceClass,
  Region,
  SpreadingFactor,
  ConnectionStatus,
  PowerMode,
  EventType,
  LPWANConfig,
  LoRaWANConfig,
  SigfoxConfig,
  NBIoTConfig,
  LTEMConfig,
  UplinkMessage,
  DownlinkMessage,
  JoinRequest,
  JoinAccept,
  DeviceStatus,
  ADRConfig,
  ADRAdjustment,
  SleepConfig,
  BatteryInfo,
  NetworkStats,
  LinkQuality,
  CoverageSurveyResult,
  EventData,
  EventHandler,
  MessageHandler,
  LPWANError,
  LPWANErrorCode,
  CayenneLPPData,
  CayenneLPPType,
  SensorReading
} from './types';

// ============================================================================
// LPWANDevice - Main SDK Class
// ============================================================================

/**
 * Main LPWAN device class
 * Provides unified interface across different LPWAN technologies
 */
export class LPWANDevice {
  private config: LPWANConfig;
  private status: DeviceStatus;
  private eventHandlers: Map<EventType, EventHandler[]>;
  private messageHandlers: MessageHandler[];
  private adrConfig?: ADRConfig;
  private snrHistory: number[] = [];

  constructor(config: LPWANConfig) {
    this.config = config;
    this.eventHandlers = new Map();
    this.messageHandlers = [];

    // Initialize status
    this.status = {
      connectionStatus: ConnectionStatus.DISCONNECTED,
      powerMode: PowerMode.IDLE,
      frameCounterUp: 0,
      frameCounterDown: 0,
      uptime: 0,
      messagesSentToday: 0,
      messagesReceivedToday: 0
    };

    // Enable ADR by default for LoRaWAN
    if (config.technology === Technology.LORAWAN && config.enableADR !== false) {
      this.adrConfig = {
        enabled: true,
        targetSNR: 10,
        minDataRate: 0,
        maxDataRate: 5,
        historySize: 20,
        snrMargin: 5
      };
    }
  }

  // ============================================================================
  // Connection Management
  // ============================================================================

  /**
   * Initialize device
   */
  async initialize(): Promise<void> {
    this.emit(EventType.CONNECTED, { message: 'Initializing device' });

    // Technology-specific initialization
    switch (this.config.technology) {
      case Technology.LORAWAN:
        await this.initLoRaWAN();
        break;
      case Technology.SIGFOX:
        await this.initSigfox();
        break;
      case Technology.NBIOT:
        await this.initNBIoT();
        break;
      case Technology.LTEM:
        await this.initLTEM();
        break;
      default:
        throw new LPWANError(
          LPWANErrorCode.INVALID_CONFIG,
          `Unsupported technology: ${this.config.technology}`
        );
    }

    this.status.connectionStatus = ConnectionStatus.CONNECTED;
  }

  /**
   * Join network (LoRaWAN OTAA)
   */
  async join(timeout: number = 30000): Promise<JoinAccept> {
    if (this.config.technology !== Technology.LORAWAN) {
      throw new LPWANError(
        LPWANErrorCode.NOT_SUPPORTED,
        'Join is only supported for LoRaWAN'
      );
    }

    const loraConfig = this.config as LoRaWANConfig;

    if (!loraConfig.appKey) {
      throw new LPWANError(
        LPWANErrorCode.INVALID_CONFIG,
        'AppKey required for OTAA join'
      );
    }

    this.status.connectionStatus = ConnectionStatus.JOINING;
    this.emit(EventType.CONNECTED, { message: 'Joining network via OTAA' });

    // Simulate join procedure
    const joinRequest: JoinRequest = {
      deviceEUI: loraConfig.deviceEUI,
      appEUI: loraConfig.appEUI,
      devNonce: Math.floor(Math.random() * 65536)
    };

    // In real implementation, this would send join request and wait for accept
    await this.delay(2000);

    const joinAccept: JoinAccept = {
      appNonce: Math.floor(Math.random() * 16777216),
      netID: 0x000013,
      devAddr: this.generateDevAddr(),
      success: true
    };

    this.status.connectionStatus = ConnectionStatus.CONNECTED;
    this.emit(EventType.JOIN_SUCCESS, { message: 'Successfully joined network' });

    return joinAccept;
  }

  /**
   * Connect to network
   */
  async connect(): Promise<void> {
    await this.initialize();

    // For LoRaWAN with OTAA, join the network
    if (this.config.technology === Technology.LORAWAN) {
      const loraConfig = this.config as LoRaWANConfig;
      if (loraConfig.appKey) {
        await this.join();
      }
    }
  }

  /**
   * Disconnect from network
   */
  async disconnect(): Promise<void> {
    this.status.connectionStatus = ConnectionStatus.DISCONNECTED;
    this.emit(EventType.DISCONNECTED, { message: 'Disconnected from network' });
  }

  // ============================================================================
  // Message Transmission
  // ============================================================================

  /**
   * Send uplink message
   */
  async send(message: UplinkMessage): Promise<void> {
    if (this.status.connectionStatus !== ConnectionStatus.CONNECTED) {
      throw new LPWANError(
        LPWANErrorCode.TRANSMISSION_FAILED,
        'Device not connected to network'
      );
    }

    // Check payload size
    const maxPayload = this.getMaxPayloadSize();
    if (message.payload.length > maxPayload) {
      throw new LPWANError(
        LPWANErrorCode.INVALID_PAYLOAD,
        `Payload size ${message.payload.length} exceeds maximum ${maxPayload}`
      );
    }

    // Check duty cycle (for LoRaWAN in EU)
    if (this.config.technology === Technology.LORAWAN) {
      const loraConfig = this.config as LoRaWANConfig;
      if (loraConfig.region === Region.EU868 && this.status.dutyCycleUsage && this.status.dutyCycleUsage > 99) {
        throw new LPWANError(
          LPWANErrorCode.DUTY_CYCLE_EXCEEDED,
          'Duty cycle limit exceeded'
        );
      }
    }

    // Emit event
    this.emit(EventType.UPLINK_SENT, {
      message: `Sending ${message.payload.length} bytes on port ${message.port}`,
      data: message
    });

    // Simulate transmission
    await this.delay(100);

    // Update counters
    this.status.frameCounterUp = (this.status.frameCounterUp || 0) + 1;
    this.status.messagesSentToday = (this.status.messagesSentToday || 0) + 1;

    // Simulate confirmation
    if (message.confirmed) {
      await this.delay(1000);
      this.emit(EventType.UPLINK_CONFIRMED, {
        message: 'Uplink confirmed by network'
      });
    }
  }

  /**
   * Send sensor data (auto-encoded)
   */
  async sendData(data: Record<string, number | object>): Promise<void> {
    const payload = this.encodeData(data);

    await this.send({
      port: 1,
      payload,
      confirmed: false
    });
  }

  /**
   * Send Cayenne LPP encoded data
   */
  async sendCayenneLPP(data: CayenneLPPData[]): Promise<void> {
    const payload = this.encodeCayenneLPP(data);

    await this.send({
      port: 1,
      payload,
      encoding: 'cayennelpp'
    });
  }

  // ============================================================================
  // Message Reception
  // ============================================================================

  /**
   * Register message handler
   */
  onDownlink(handler: MessageHandler): void {
    this.messageHandlers.push(handler);
  }

  /**
   * Remove message handler
   */
  offDownlink(handler: MessageHandler): void {
    const index = this.messageHandlers.indexOf(handler);
    if (index > -1) {
      this.messageHandlers.splice(index, 1);
    }
  }

  /**
   * Simulate receiving a downlink message
   */
  private async receiveDownlink(message: DownlinkMessage): Promise<void> {
    this.status.frameCounterDown = (this.status.frameCounterDown || 0) + 1;
    this.status.messagesReceivedToday = (this.status.messagesReceivedToday || 0) + 1;

    // Update signal quality
    if (message.rssi) this.status.rssi = message.rssi;
    if (message.snr) this.status.snr = message.snr;

    // Process ADR if enabled
    if (this.adrConfig?.enabled && message.snr) {
      this.snrHistory.push(message.snr);
      if (this.snrHistory.length > (this.adrConfig.historySize || 20)) {
        this.snrHistory.shift();
      }

      const adjustment = this.processADR();
      if (adjustment) {
        this.emit(EventType.ADR_ADJUSTED, {
          message: `ADR adjustment: ${adjustment.action}`,
          data: adjustment
        });
      }
    }

    // Emit event
    this.emit(EventType.DOWNLINK_RECEIVED, {
      message: `Received ${message.payload.length} bytes on port ${message.port}`,
      data: message
    });

    // Call handlers
    for (const handler of this.messageHandlers) {
      await handler(message);
    }
  }

  // ============================================================================
  // Adaptive Data Rate (ADR)
  // ============================================================================

  /**
   * Enable ADR with custom configuration
   */
  enableADR(config?: Partial<ADRConfig>): void {
    this.adrConfig = {
      enabled: true,
      targetSNR: config?.targetSNR || 10,
      minDataRate: config?.minDataRate || 0,
      maxDataRate: config?.maxDataRate || 5,
      minTxPower: config?.minTxPower || 2,
      maxTxPower: config?.maxTxPower || 14,
      historySize: config?.historySize || 20,
      snrMargin: config?.snrMargin || 5
    };
  }

  /**
   * Disable ADR
   */
  disableADR(): void {
    if (this.adrConfig) {
      this.adrConfig.enabled = false;
    }
  }

  /**
   * Process ADR adjustment
   */
  private processADR(): ADRAdjustment | null {
    if (!this.adrConfig?.enabled || this.snrHistory.length < 10) {
      return null;
    }

    const avgSNR = this.snrHistory.reduce((a, b) => a + b, 0) / this.snrHistory.length;
    const snrMargin = avgSNR - (this.adrConfig.targetSNR || 10);
    const margin = this.adrConfig.snrMargin || 5;

    const currentSF = this.status.spreadingFactor || SpreadingFactor.SF10;
    const currentPower = this.status.txPower || 14;

    // Too much margin - increase efficiency
    if (snrMargin > margin + 5) {
      if (currentSF > SpreadingFactor.SF7) {
        const newSF = currentSF - 1;
        this.status.spreadingFactor = newSF as SpreadingFactor;

        return {
          action: 'DECREASE_SF',
          newSpreadingFactor: newSF as SpreadingFactor,
          reason: `SNR ${avgSNR.toFixed(1)} dB is high, decreasing SF for better data rate`,
          averageSNR: avgSNR
        };
      } else if (currentPower > (this.adrConfig.minTxPower || 2)) {
        const newPower = Math.max(currentPower - 3, this.adrConfig.minTxPower || 2);
        this.status.txPower = newPower;

        return {
          action: 'REDUCE_POWER',
          newTxPower: newPower,
          reason: `SNR ${avgSNR.toFixed(1)} dB is high, reducing power to save battery`,
          averageSNR: avgSNR
        };
      }
    }

    // Not enough margin - increase robustness
    if (snrMargin < -margin) {
      if (currentPower < (this.adrConfig.maxTxPower || 14)) {
        const newPower = Math.min(currentPower + 3, this.adrConfig.maxTxPower || 14);
        this.status.txPower = newPower;

        return {
          action: 'INCREASE_POWER',
          newTxPower: newPower,
          reason: `SNR ${avgSNR.toFixed(1)} dB is low, increasing power`,
          averageSNR: avgSNR
        };
      } else if (currentSF < SpreadingFactor.SF12) {
        const newSF = currentSF + 1;
        this.status.spreadingFactor = newSF as SpreadingFactor;

        return {
          action: 'INCREASE_SF',
          newSpreadingFactor: newSF as SpreadingFactor,
          reason: `SNR ${avgSNR.toFixed(1)} dB is low, increasing SF for better range`,
          averageSNR: avgSNR
        };
      }
    }

    return null;
  }

  // ============================================================================
  // Power Management
  // ============================================================================

  /**
   * Enter low-power mode
   */
  async enterLowPowerMode(config: SleepConfig): Promise<void> {
    this.status.powerMode = config.mode;

    this.emit(EventType.CONNECTED, {
      message: `Entering ${config.mode} for ${config.duration}ms`
    });

    // In real implementation, this would put hardware to sleep
    await this.delay(config.duration);

    this.status.powerMode = PowerMode.ACTIVE;
  }

  /**
   * Deep sleep (alias for enterLowPowerMode)
   */
  async sleep(config: SleepConfig): Promise<void> {
    return this.enterLowPowerMode(config);
  }

  /**
   * Get battery information
   */
  getBatteryLevel(): number {
    return this.status.batteryLevel || 100;
  }

  /**
   * Get detailed battery information
   */
  getBatteryInfo(): BatteryInfo {
    return {
      level: this.status.batteryLevel || 100,
      voltage: this.status.batteryVoltage || 3.6,
      type: 'lithium',
      charging: false,
      health: 100
    };
  }

  /**
   * Calculate estimated battery life
   */
  calculateBatteryLife(params: {
    batteryCapacity: number;
    messagesPerDay: number;
    payloadSize: number;
  }): number {
    const { batteryCapacity, messagesPerDay, payloadSize } = params;

    // Simplified calculation
    const sleepCurrentUA = 2;
    const txCurrentMA = this.getTxCurrent();
    const rxCurrentMA = 15;

    const txTimeS = this.getTxTime(payloadSize);
    const rxTimeS = 0.02 * 2; // 2 RX windows

    const sleepTimeS = 86400 - (messagesPerDay * (txTimeS + rxTimeS));

    const dailyConsumption = (
      (sleepCurrentUA / 1000) * (sleepTimeS / 3600) +
      txCurrentMA * messagesPerDay * (txTimeS / 3600) +
      rxCurrentMA * messagesPerDay * (rxTimeS / 3600)
    );

    const batteryLifeDays = batteryCapacity / dailyConsumption;
    return batteryLifeDays / 365.25; // years
  }

  // ============================================================================
  // Device Status
  // ============================================================================

  /**
   * Get device status
   */
  getStatus(): DeviceStatus {
    return { ...this.status };
  }

  /**
   * Get link quality
   */
  getLinkQuality(): LinkQuality {
    const rssi = this.status.rssi || -100;
    const snr = this.status.snr || 0;

    let quality: LinkQuality['quality'];
    if (rssi > -80 && snr > 10) quality = 'excellent';
    else if (rssi > -100 && snr > 5) quality = 'good';
    else if (rssi > -115 && snr > 0) quality = 'fair';
    else if (rssi > -125 && snr > -5) quality = 'poor';
    else quality = 'critical';

    const lqi = Math.max(0, Math.min(100, (rssi + 140) * 2));

    return {
      rssi,
      snr,
      lqi,
      quality
    };
  }

  /**
   * Get network statistics
   */
  getNetworkStats(): NetworkStats {
    const totalUplinks = this.status.frameCounterUp || 0;
    const totalDownlinks = this.status.frameCounterDown || 0;

    return {
      totalUplinks,
      totalDownlinks,
      failedTransmissions: 0,
      averageRSSI: this.status.rssi || -100,
      averageSNR: this.status.snr || 0,
      packetDeliveryRatio: 99.5
    };
  }

  // ============================================================================
  // Coverage Testing
  // ============================================================================

  /**
   * Run coverage survey
   */
  async runCoverageSurvey(params: {
    duration: number;
    interval: number;
    location?: { latitude: number; longitude: number };
  }): Promise<CoverageSurveyResult> {
    const startTime = Date.now();
    const tests: { rssi: number; snr: number }[] = [];

    const numTests = Math.floor(params.duration / params.interval);

    for (let i = 0; i < numTests; i++) {
      // Simulate test transmission
      const rssi = -100 + Math.random() * 40;
      const snr = -5 + Math.random() * 20;

      tests.push({ rssi, snr });

      await this.delay(params.interval * 1000);
    }

    const successful = tests.filter(t => t.rssi > -120 && t.snr > -10).length;
    const successRate = (successful / tests.length) * 100;

    const avgRSSI = tests.reduce((sum, t) => sum + t.rssi, 0) / tests.length;
    const avgSNR = tests.reduce((sum, t) => sum + t.snr, 0) / tests.length;

    let coverageQuality: CoverageSurveyResult['coverageQuality'];
    if (successRate >= 95 && avgRSSI > -100) coverageQuality = 'excellent';
    else if (successRate >= 85 && avgRSSI > -110) coverageQuality = 'good';
    else if (successRate >= 70 && avgRSSI > -120) coverageQuality = 'marginal';
    else if (successRate >= 50) coverageQuality = 'poor';
    else coverageQuality = 'no_coverage';

    return {
      startTime,
      duration: params.duration,
      location: params.location,
      totalTests: tests.length,
      successfulTests: successful,
      successRate,
      averageRSSI: avgRSSI,
      averageSNR: avgSNR,
      rssiRange: {
        min: Math.min(...tests.map(t => t.rssi)),
        max: Math.max(...tests.map(t => t.rssi))
      },
      snrRange: {
        min: Math.min(...tests.map(t => t.snr)),
        max: Math.max(...tests.map(t => t.snr))
      },
      gateways: [],
      coverageQuality
    };
  }

  // ============================================================================
  // Event Management
  // ============================================================================

  /**
   * Register event handler
   */
  on(eventType: EventType, handler: EventHandler): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, []);
    }
    this.eventHandlers.get(eventType)!.push(handler);
  }

  /**
   * Remove event handler
   */
  off(eventType: EventType, handler: EventHandler): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      const index = handlers.indexOf(handler);
      if (index > -1) {
        handlers.splice(index, 1);
      }
    }
  }

  /**
   * Emit event
   */
  private emit(type: EventType, data: Partial<EventData>): void {
    const event: EventData = {
      type,
      timestamp: Date.now(),
      severity: 'info',
      ...data
    };

    const handlers = this.eventHandlers.get(type);
    if (handlers) {
      handlers.forEach(handler => handler(event));
    }
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Initialize LoRaWAN
   */
  private async initLoRaWAN(): Promise<void> {
    const config = this.config as LoRaWANConfig;

    this.status.spreadingFactor = config.spreadingFactor || SpreadingFactor.SF10;
    this.status.txPower = config.txPower || 14;
    this.status.dataRate = this.calculateDataRate(
      this.status.spreadingFactor,
      config.bandwidth || 125
    );
  }

  /**
   * Initialize Sigfox
   */
  private async initSigfox(): Promise<void> {
    this.status.dataRate = 100; // 100 bps
  }

  /**
   * Initialize NB-IoT
   */
  private async initNBIoT(): Promise<void> {
    const config = this.config as NBIoTConfig;

    if (config.psmEnabled) {
      this.status.powerMode = PowerMode.DEEP_SLEEP;
    }

    this.status.dataRate = 60000; // 60 kbps typical
  }

  /**
   * Initialize LTE-M
   */
  private async initLTEM(): Promise<void> {
    this.status.dataRate = 200000; // 200 kbps typical
  }

  /**
   * Get maximum payload size
   */
  private getMaxPayloadSize(): number {
    const maxPayloads: Record<Technology, number> = {
      [Technology.LORAWAN]: 242,
      [Technology.SIGFOX]: 12,
      [Technology.NBIOT]: 1600,
      [Technology.LTEM]: 1000
    };

    return maxPayloads[this.config.technology];
  }

  /**
   * Calculate LoRa data rate
   */
  private calculateDataRate(sf: SpreadingFactor, bw: number): number {
    const sfMap: Record<number, number> = {
      7: 5500, 8: 3100, 9: 1800, 10: 980, 11: 440, 12: 250
    };

    return sfMap[sf] || 980;
  }

  /**
   * Get TX current consumption
   */
  private getTxCurrent(): number {
    switch (this.config.technology) {
      case Technology.LORAWAN:
        return 120; // mA at 14 dBm
      case Technology.SIGFOX:
        return 50;  // mA
      case Technology.NBIOT:
        return 220; // mA at 23 dBm
      case Technology.LTEM:
        return 250; // mA at 23 dBm
      default:
        return 100;
    }
  }

  /**
   * Get TX time
   */
  private getTxTime(payloadSize: number): number {
    // Simplified - actual calculation depends on SF, BW, etc.
    switch (this.config.technology) {
      case Technology.LORAWAN:
        return 0.5; // seconds (average)
      case Technology.SIGFOX:
        return 6;   // seconds (3 transmissions)
      case Technology.NBIOT:
        return 1;   // second
      case Technology.LTEM:
        return 0.5; // seconds
      default:
        return 1;
    }
  }

  /**
   * Generate random DevAddr
   */
  private generateDevAddr(): string {
    const addr = Math.floor(Math.random() * 0xFFFFFFFF);
    return addr.toString(16).padStart(8, '0').toUpperCase();
  }

  /**
   * Encode data as binary payload
   */
  private encodeData(data: Record<string, number | object>): Buffer {
    // Simple encoding - in production use proper encoding
    const jsonStr = JSON.stringify(data);
    return Buffer.from(jsonStr);
  }

  /**
   * Encode Cayenne LPP data
   */
  private encodeCayenneLPP(data: CayenneLPPData[]): Buffer {
    const buffers: Buffer[] = [];

    for (const item of data) {
      buffers.push(Buffer.from([item.channel, item.type]));

      if (typeof item.value === 'number') {
        const value = Math.round(item.value * 100);
        buffers.push(Buffer.from([value >> 8, value & 0xFF]));
      } else if ('x' in item.value && 'y' in item.value && 'z' in item.value) {
        // Accelerometer/Gyrometer
        const x = Math.round(item.value.x * 1000);
        const y = Math.round(item.value.y * 1000);
        const z = Math.round(item.value.z * 1000);
        buffers.push(Buffer.from([
          x >> 8, x & 0xFF,
          y >> 8, y & 0xFF,
          z >> 8, z & 0xFF
        ]));
      } else if ('lat' in item.value && 'lon' in item.value) {
        // GPS
        const lat = Math.round(item.value.lat * 10000);
        const lon = Math.round(item.value.lon * 10000);
        const alt = Math.round((item.value.alt || 0) * 100);
        buffers.push(Buffer.from([
          lat >> 16, (lat >> 8) & 0xFF, lat & 0xFF,
          lon >> 16, (lon >> 8) & 0xFF, lon & 0xFF,
          alt >> 8, alt & 0xFF
        ]));
      }
    }

    return Buffer.concat(buffers);
  }

  /**
   * Delay helper
   */
  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';
export { LPWANDevice };

// Default export
export default LPWANDevice;
