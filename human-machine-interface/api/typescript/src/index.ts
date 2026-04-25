/**
 * WIA-AUG-014: Human-Machine Interface SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Interface Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for human-machine interface communication including:
 * - Signal encoding and decoding
 * - Intent interpretation
 * - Haptic feedback generation
 * - Device connection management
 * - Calibration procedures
 */

import {
  SignalType,
  SignalPacket,
  LatencyTier,
  LatencyMeasurement,
  LatencyComponents,
  Intent,
  IntentCategory,
  MotorCommand,
  CommandResult,
  HapticFeedback,
  HapticPattern,
  CalibrationSession,
  CalibrationType,
  CalibrationResult,
  DeviceAdvertisement,
  DeviceState,
  Connection,
  ConnectionConfig,
  UserProfile,
  QualityMetrics,
  Vector3,
  HMI_CONSTANTS,
  HMIErrorCode,
  HMIError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUG-014 Human-Machine Interface SDK
 */
export class HumanMachineInterfaceSDK {
  private version = '1.0.0';
  private activeConnection: Connection | null = null;
  private signalHandlers: ((signal: SignalPacket) => void)[] = [];

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Discover available HMI devices
   */
  async discover(): Promise<DeviceAdvertisement[]> {
    // In a real implementation, this would scan for devices
    // For now, return a mock device
    return [
      {
        deviceId: `DEV-${Date.now()}`,
        deviceType: 'prosthetic_arm',
        manufacturer: 'WIA Demo',
        model: 'Demo-HMI-001',
        version: '1.0.0',
        capabilities: {
          signalTypes: [SignalType.SURFACE_EMG, SignalType.ACCELEROMETER],
          feedbackTypes: ['haptic', 'thermal'],
          latencyTier: 2,
          channels: 8,
          sampleRates: [500, 1000, 2000],
        },
        protocols: {
          hmi: 'WIA-AUG-014/1.0',
          security: 'WIA-SEC/1.0',
          safety: 'WIA-AUG-013/1.0',
        },
        status: 'available',
      },
    ];
  }

  /**
   * Connect to an HMI device
   */
  async connect(config: ConnectionConfig): Promise<Connection> {
    const devices = await this.discover();
    const device = devices.find((d) => d.deviceId === config.deviceId);

    if (!device) {
      throw new HMIError(HMIErrorCode.CONNECTION_NOT_FOUND, `Device ${config.deviceId} not found`);
    }

    const connection: Connection = {
      id: `CONN-${Date.now()}`,
      device,
      config,
      status: 'connected',
      connectedAt: new Date(),
      latency: 15,
      signalQuality: 95,
    };

    this.activeConnection = connection;
    return connection;
  }

  /**
   * Disconnect from current device
   */
  async disconnect(): Promise<void> {
    if (this.activeConnection) {
      this.activeConnection.status = 'disconnected';
      this.activeConnection = null;
    }
  }

  /**
   * Encode a neural/biological signal for transmission
   */
  encodeSignal(
    signalType: SignalType,
    samples: number[],
    channelCount: number
  ): SignalPacket {
    const sampleCount = Math.floor(samples.length / channelCount);

    return {
      header: {
        version: HMI_CONSTANTS.PROTOCOL_VERSION,
        signalType,
        timestamp: BigInt(Date.now() * 1000),
        channelCount,
        sampleCount,
        resolution: 24,
      },
      metadata: {
        deviceId: this.activeConnection?.device.deviceId || 'unknown',
        sessionId: `SES-${Date.now()}`,
        sequenceNumber: 0,
        quality: 95,
      },
      payload: {
        samples: new Int32Array(samples),
        markers: [],
        checksum: this.calculateChecksum(samples),
      },
    };
  }

  /**
   * Decode an intent from signal data
   */
  interpretIntent(signals: SignalPacket[]): Intent {
    // In a real implementation, this would use trained ML models
    // For demo, return a sample intent
    const signal = signals[0];

    return {
      category: 'motor',
      action: 'grip_close',
      parameters: {
        intensity: 0.7,
        speed: 0.5,
        precision: 0.8,
      },
      confidence: 0.92,
      timestamp: signal?.header.timestamp || BigInt(Date.now() * 1000),
      source: signal?.metadata.deviceId || 'unknown',
    };
  }

  /**
   * Generate a motor command from intent
   */
  generateCommand(intent: Intent): MotorCommand {
    const joints = this.mapIntentToJoints(intent);

    return {
      deviceId: this.activeConnection?.device.deviceId || 'unknown',
      commandType: 'position',
      joints,
      synchronization: {
        timestamp: BigInt(Date.now() * 1000),
        deadline: BigInt((Date.now() + 50) * 1000), // 50ms deadline
        priority: 1,
      },
    };
  }

  /**
   * Send a motor command to the device
   */
  async sendCommand(command: MotorCommand): Promise<CommandResult> {
    if (!this.activeConnection) {
      throw new HMIError(HMIErrorCode.CONNECTION_NOT_FOUND, 'No active connection');
    }

    // Simulate command execution
    const startTime = performance.now();

    // In real implementation, send to device and wait for response
    await this.simulateDelay(10);

    const executionTime = performance.now() - startTime;

    return {
      success: true,
      executionTime,
      achieved: {
        joints: command.joints.map((j) => j.target),
        error: command.joints.map(() => Math.random() * 0.01),
      },
    };
  }

  /**
   * Send haptic feedback to user
   */
  async sendFeedback(feedback: HapticFeedback): Promise<void> {
    if (!this.activeConnection) {
      throw new HMIError(HMIErrorCode.CONNECTION_NOT_FOUND, 'No active connection');
    }

    // Validate feedback parameters
    if (feedback.intensity < 0 || feedback.intensity > 1) {
      throw new HMIError(HMIErrorCode.INVALID_COMMAND, 'Intensity must be 0-1');
    }

    // In real implementation, send to device
    await this.simulateDelay(5);
  }

  /**
   * Start calibration session
   */
  async startCalibration(type: CalibrationType): Promise<CalibrationSession> {
    const session: CalibrationSession = {
      sessionId: `CAL-${Date.now()}`,
      type,
      startTime: new Date(),
      status: 'in_progress',
      tasks: this.getCalibrationTasks(type),
    };

    return session;
  }

  /**
   * Measure latency
   */
  measureLatency(): LatencyMeasurement {
    const components: LatencyComponents = {
      acquisition: 2 + Math.random() * 1,
      processing: 1 + Math.random() * 0.5,
      encoding: 0.5 + Math.random() * 0.3,
      transmission: 3 + Math.random() * 2,
      decoding: 0.5 + Math.random() * 0.3,
      execution: 2 + Math.random() * 1,
    };

    const total = Object.values(components).reduce((sum, val) => sum + val, 0);

    // Determine tier compliance
    let tier: LatencyTier = 4;
    const thresholds = HMI_CONSTANTS.LATENCY_THRESHOLDS;
    if (total <= thresholds.TIER_1) tier = 1;
    else if (total <= thresholds.TIER_2) tier = 2;
    else if (total <= thresholds.TIER_3) tier = 3;

    return {
      measurementId: `LAT-${Date.now()}`,
      tier,
      components,
      total,
      jitter: Math.random() * 2,
      compliance: total <= thresholds[`TIER_${tier}` as keyof typeof thresholds],
      timestamp: new Date(),
    };
  }

  /**
   * Get current device state
   */
  getDeviceState(): DeviceState | null {
    if (!this.activeConnection) return null;

    return {
      deviceId: this.activeConnection.device.deviceId,
      timestamp: BigInt(Date.now() * 1000),
      power: {
        batteryLevel: 85,
        charging: false,
        estimatedRuntime: 480,
      },
      position: {
        joints: [0, 15, 30, 45, 60],
        endEffector: { x: 0.5, y: 0.2, z: 0.1 },
      },
      sensors: {
        force: [0.1, 0.2, 0.15, 0.3, 0.25],
        temperature: [36.5, 36.7, 36.6],
        contact: [false, true, true, false, false],
      },
      status: 'active',
    };
  }

  /**
   * Get quality metrics
   */
  getMetrics(): QualityMetrics {
    return {
      signalToNoise: 45 + Math.random() * 10,
      channelCorrelation: 0.1 + Math.random() * 0.1,
      baselineStability: 0.05 + Math.random() * 0.03,
      artifactRate: 2 + Math.random() * 2,
      decodingAccuracy: 92 + Math.random() * 5,
      latencyCompliance: 95 + Math.random() * 3,
      feedbackAccuracy: 88 + Math.random() * 8,
    };
  }

  /**
   * Register signal handler
   */
  onSignal(callback: (signal: SignalPacket) => void): void {
    this.signalHandlers.push(callback);
  }

  /**
   * Create standard haptic pattern
   */
  createHapticPattern(
    name: keyof typeof HMI_CONSTANTS.HAPTIC_PATTERNS
  ): HapticPattern {
    const duration = HMI_CONSTANTS.HAPTIC_PATTERNS[name];

    return {
      name,
      segments: [
        {
          intensity: 1.0,
          duration,
          rampUp: duration * 0.1,
          rampDown: duration * 0.2,
        },
      ],
      repeatCount: 1,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private calculateChecksum(data: number[]): number {
    let crc = 0xffffffff;
    for (const byte of data) {
      crc ^= byte;
      for (let i = 0; i < 8; i++) {
        crc = (crc >>> 1) ^ (crc & 1 ? 0xedb88320 : 0);
      }
    }
    return ~crc >>> 0;
  }

  private mapIntentToJoints(intent: Intent): MotorCommand['joints'] {
    // Map common intents to joint commands
    const baseJoint = {
      limits: { min: 0, max: 90, maxVelocity: 100, maxAcceleration: 200 },
      compliance: 0.3,
    };

    switch (intent.action) {
      case 'grip_close':
        return [
          { jointId: 0, target: 90 * intent.parameters.intensity, ...baseJoint },
          { jointId: 1, target: 90 * intent.parameters.intensity, ...baseJoint },
          { jointId: 2, target: 90 * intent.parameters.intensity, ...baseJoint },
        ];
      case 'grip_open':
        return [
          { jointId: 0, target: 0, ...baseJoint },
          { jointId: 1, target: 0, ...baseJoint },
          { jointId: 2, target: 0, ...baseJoint },
        ];
      default:
        return [{ jointId: 0, target: 45, ...baseJoint }];
    }
  }

  private getCalibrationTasks(type: CalibrationType) {
    const baseTasks = [
      {
        taskId: 'REST',
        name: 'Rest State',
        description: 'Remain relaxed with no movement',
        duration: 30,
        instructions: ['Relax your muscles', 'Stay still', 'Breathe normally'],
        requiredSignals: [SignalType.SURFACE_EMG] as SignalType[],
        repetitions: 1,
        restPeriod: 5,
      },
    ];

    if (type === 'initial' || type === 'recalibration') {
      baseTasks.push(
        {
          taskId: 'MVC',
          name: 'Maximum Voluntary Contraction',
          description: 'Contract muscles with maximum force',
          duration: 5,
          instructions: ['Contract as hard as possible', 'Hold for 5 seconds'],
          requiredSignals: [SignalType.SURFACE_EMG] as SignalType[],
          repetitions: 3,
          restPeriod: 30,
        },
        {
          taskId: 'GRADED',
          name: 'Graded Force',
          description: 'Contract at 25%, 50%, 75%, 100%',
          duration: 20,
          instructions: ['Follow the on-screen force target'],
          requiredSignals: [SignalType.SURFACE_EMG] as SignalType[],
          repetitions: 4,
          restPeriod: 10,
        }
      );
    }

    return baseTasks;
  }

  private simulateDelay(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create HMI connection (standalone)
 */
export async function createHMIConnection(
  config: ConnectionConfig
): Promise<Connection> {
  const sdk = new HumanMachineInterfaceSDK();
  return sdk.connect(config);
}

/**
 * Encode signal (standalone)
 */
export function encodeSignal(
  signalType: SignalType,
  samples: number[],
  channelCount: number
): SignalPacket {
  const sdk = new HumanMachineInterfaceSDK();
  return sdk.encodeSignal(signalType, samples, channelCount);
}

/**
 * Create haptic pattern (standalone)
 */
export function createHapticPattern(
  name: keyof typeof HMI_CONSTANTS.HAPTIC_PATTERNS
): HapticPattern {
  const sdk = new HumanMachineInterfaceSDK();
  return sdk.createHapticPattern(name);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { HumanMachineInterfaceSDK };
export default HumanMachineInterfaceSDK;
