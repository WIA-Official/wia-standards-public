/**
 * WIA Exoskeleton Intent Detection Implementation
 * @version 1.0.0
 */

import {
  UserIntent,
  IntentSource,
  IntentState,
  FusionMethod,
  ConfirmationMode,
  EMGData,
  EMGFeatures,
  GRFData,
  GRFFeatures,
  IMUData,
  IMUFeatures,
  ButtonData,
  SourceResult,
  FeatureVector,
  IntentDetection,
  IntentDetectorConfig,
  CalibrationData,
  CalibrationState,
  DEFAULT_INTENT_CONFIG,
  DEFAULT_THRESHOLDS,
} from './types';

// ============================================================================
// Utility Functions
// ============================================================================

function generateDetectionId(): string {
  return `intent-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
}

// ============================================================================
// Source Detectors
// ============================================================================

/** EMG-based intent detector */
export class EMGIntentDetector {
  private baseline: Map<number, number> = new Map();
  private mvcValues: Map<number, number> = new Map();

  constructor(private config: IntentDetectorConfig['emgConfig']) {}

  /**
   * Extract features from raw EMG signal
   */
  extractFeatures(signal: number[]): EMGFeatures {
    const n = signal.length;

    // Mean Absolute Value
    const mav = signal.reduce((sum, v) => sum + Math.abs(v), 0) / n;

    // Root Mean Square
    const rms = Math.sqrt(signal.reduce((sum, v) => sum + v * v, 0) / n);

    // Waveform Length
    let wl = 0;
    for (let i = 1; i < n; i++) {
      wl += Math.abs(signal[i] - signal[i - 1]);
    }

    // Zero Crossings
    let zc = 0;
    for (let i = 1; i < n; i++) {
      if ((signal[i] > 0 && signal[i - 1] < 0) ||
          (signal[i] < 0 && signal[i - 1] > 0)) {
        zc++;
      }
    }

    // Slope Sign Changes
    let ssc = 0;
    for (let i = 2; i < n; i++) {
      const diff1 = signal[i - 1] - signal[i - 2];
      const diff2 = signal[i] - signal[i - 1];
      if (diff1 * diff2 < 0) {
        ssc++;
      }
    }

    // Integrated EMG
    const iemg = signal.reduce((sum, v) => sum + Math.abs(v), 0);

    // Normalized activation (% MVC)
    const activation = mav / (this.mvcValues.get(0) ?? 1);

    return { mav, rms, wl, zc, ssc, iemg, activation };
  }

  /**
   * Detect intent from EMG data
   */
  detect(data: EMGData): SourceResult {
    const startTime = Date.now();

    // Default to IDLE
    let intent = UserIntent.IDLE;
    let confidence = 0;

    // Extract features from all channels
    const features: Record<string, EMGFeatures> = {};
    data.channels.forEach((feat, channelId) => {
      features[`ch${channelId}`] = feat;
    });

    // Check for stand up intent (high rectus femoris + gluteus maximus)
    const rectusActivation = features['ch1']?.activation ?? 0;
    const gluteusActivation = features['ch5']?.activation ?? 0;

    if (rectusActivation > 0.3 && gluteusActivation > 0.3) {
      intent = UserIntent.STAND_UP;
      confidence = Math.min(rectusActivation, gluteusActivation);
    }

    // Check for walk intent (tibialis anterior activation)
    const tibialisActivation = features['ch3']?.activation ?? 0;
    if (tibialisActivation > 0.25 && intent === UserIntent.IDLE) {
      intent = UserIntent.WALK_FORWARD;
      confidence = tibialisActivation;
    }

    return {
      source: IntentSource.EMG,
      intent,
      confidence,
      latency: Date.now() - startTime,
    };
  }

  /**
   * Calibrate MVC values
   */
  calibrateMVC(channelId: number, mvcValue: number): void {
    this.mvcValues.set(channelId, mvcValue);
  }

  /**
   * Set baseline values
   */
  setBaseline(channelId: number, baseline: number): void {
    this.baseline.set(channelId, baseline);
  }
}

/** GRF-based intent detector */
export class GRFIntentDetector {
  private previousData?: GRFData;
  private thresholds = DEFAULT_THRESHOLDS;

  constructor(private config: IntentDetectorConfig['grfConfig']) {}

  /**
   * Extract features from GRF data
   */
  extractFeatures(data: GRFData): GRFFeatures {
    const leftCop = data.left.cop;
    const rightCop = data.right.cop;

    // Center of pressure shift
    const avgCopX = (leftCop.x + rightCop.x) / 2;
    const avgCopY = (leftCop.y + rightCop.y) / 2;

    let copShiftX = 0;
    let copShiftY = 0;

    if (this.previousData) {
      const prevAvgCopX = (this.previousData.left.cop.x + this.previousData.right.cop.x) / 2;
      const prevAvgCopY = (this.previousData.left.cop.y + this.previousData.right.cop.y) / 2;
      copShiftX = (avgCopX - prevAvgCopX) * 100; // Convert to cm
      copShiftY = (avgCopY - prevAvgCopY) * 100;
    }

    // Vertical force
    const totalVerticalForce = data.left.fz + data.right.fz;
    let fzChange = 0;

    if (this.previousData) {
      const prevFz = this.previousData.left.fz + this.previousData.right.fz;
      fzChange = totalVerticalForce - prevFz;
    }

    // Asymmetry
    const leftFz = data.left.fz;
    const rightFz = data.right.fz;
    const asymmetry = Math.abs(leftFz - rightFz) / (leftFz + rightFz + 0.001);

    // Loading rate (simplified)
    const loadingRate = Math.abs(fzChange) * (this.config?.sampleRate ?? 200);

    this.previousData = data;

    return {
      copShiftX,
      copShiftY,
      fzChange,
      asymmetry,
      loadingRate,
      totalVerticalForce,
    };
  }

  /**
   * Detect intent from GRF data
   */
  detect(data: GRFData): SourceResult {
    const startTime = Date.now();
    const features = this.extractFeatures(data);

    let intent = UserIntent.IDLE;
    let confidence = 0;

    const forceThreshold = this.config?.forceThreshold ?? 20;

    // Step initiation detection
    if (features.copShiftX > 3 && features.fzChange < -forceThreshold) {
      intent = UserIntent.WALK_FORWARD;
      confidence = Math.min(1, Math.abs(features.copShiftX) / 5);
    }

    // Stand up detection
    if (features.copShiftX > 3 && features.fzChange > forceThreshold * 2) {
      intent = UserIntent.STAND_UP;
      confidence = Math.min(1, features.fzChange / 100);
    }

    // Sit down detection
    if (features.copShiftX < -3 && features.fzChange < -forceThreshold) {
      intent = UserIntent.SIT_DOWN;
      confidence = Math.min(1, Math.abs(features.copShiftX) / 5);
    }

    // Turn detection
    if (Math.abs(features.copShiftY) > 5) {
      intent = features.copShiftY > 0 ? UserIntent.TURN_LEFT : UserIntent.TURN_RIGHT;
      confidence = Math.min(1, Math.abs(features.copShiftY) / 10);
    }

    return {
      source: IntentSource.GRF,
      intent,
      confidence,
      latency: Date.now() - startTime,
      rawData: features,
    };
  }
}

/** IMU-based intent detector */
export class IMUIntentDetector {
  private previousData: Map<string, IMUData> = new Map();

  constructor(private config: IntentDetectorConfig['imuConfig']) {}

  /**
   * Extract features from IMU data
   */
  extractFeatures(data: IMUData[]): IMUFeatures {
    // Find pelvis IMU
    const pelvis = data.find(d => d.location === 'pelvis');

    if (!pelvis) {
      return {
        pelvisPitch: 0,
        pelvisRoll: 0,
        pelvisYaw: 0,
        pelvisYawRate: 0,
        trunkInclination: 0,
        accelerationMagnitude: 0,
        jerk: 0,
      };
    }

    const orientation = pelvis.orientation ?? { roll: 0, pitch: 0, yaw: 0 };

    // Calculate acceleration magnitude
    const acc = pelvis.accelerometer;
    const accelerationMagnitude = Math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2);

    // Calculate jerk (rate of change of acceleration)
    let jerk = 0;
    const prev = this.previousData.get('pelvis');
    if (prev) {
      const dt = (pelvis.timestamp - prev.timestamp) / 1000;
      if (dt > 0) {
        const prevAcc = prev.accelerometer;
        jerk = Math.sqrt(
          ((acc.x - prevAcc.x) / dt) ** 2 +
          ((acc.y - prevAcc.y) / dt) ** 2 +
          ((acc.z - prevAcc.z) / dt) ** 2
        );
      }
    }

    // Trunk inclination (combined pitch and roll)
    const trunkInclination = Math.sqrt(
      orientation.pitch ** 2 + orientation.roll ** 2
    );

    this.previousData.set('pelvis', pelvis);

    return {
      pelvisPitch: orientation.pitch,
      pelvisRoll: orientation.roll,
      pelvisYaw: orientation.yaw,
      pelvisYawRate: pelvis.gyroscope.z,
      trunkInclination,
      accelerationMagnitude,
      jerk,
    };
  }

  /**
   * Detect intent from IMU data
   */
  detect(data: IMUData[]): SourceResult {
    const startTime = Date.now();
    const features = this.extractFeatures(data);

    let intent = UserIntent.IDLE;
    let confidence = 0;

    const gyroDeadband = this.config?.gyroDeadband ?? 0.5;

    // Walk forward detection
    if (features.pelvisPitch > 5) {
      intent = UserIntent.WALK_FORWARD;
      confidence = Math.min(1, features.pelvisPitch / 15);
    }

    // Walk backward detection
    if (features.pelvisPitch < -5) {
      intent = UserIntent.WALK_BACKWARD;
      confidence = Math.min(1, Math.abs(features.pelvisPitch) / 15);
    }

    // Turn detection
    if (Math.abs(features.pelvisYawRate) > 10) {
      intent = features.pelvisYawRate < 0 ? UserIntent.TURN_LEFT : UserIntent.TURN_RIGHT;
      confidence = Math.min(1, Math.abs(features.pelvisYawRate) / 30);
    }

    // Stair detection (high pitch with step pattern)
    if (features.pelvisPitch > 10 && features.jerk > 5) {
      intent = UserIntent.STAIR_ASCEND;
      confidence = Math.min(1, features.pelvisPitch / 20);
    }

    if (features.pelvisPitch < -5 && features.trunkInclination > 10) {
      intent = UserIntent.STAIR_DESCEND;
      confidence = Math.min(1, Math.abs(features.pelvisPitch) / 15);
    }

    return {
      source: IntentSource.IMU,
      intent,
      confidence,
      latency: Date.now() - startTime,
      rawData: features,
    };
  }
}

/** Button-based intent detector */
export class ButtonIntentDetector {
  private buttonStates: Map<string, { pressed: boolean; startTime: number }> = new Map();

  constructor(private config: IntentDetectorConfig['buttonConfig']) {}

  /**
   * Detect intent from button input
   */
  detect(data: ButtonData): SourceResult {
    const startTime = Date.now();
    let intent = UserIntent.IDLE;
    let confidence = 1.0; // Buttons have 100% confidence

    // Find button mapping
    const mapping = this.config?.buttons.find(b => b.id === data.buttonId);

    if (mapping && data.pressed) {
      const holdRequired = mapping.holdTime ?? 0;

      if (holdRequired > 0) {
        // Check hold time
        const state = this.buttonStates.get(data.buttonId);
        if (state && state.pressed) {
          const holdDuration = Date.now() - state.startTime;
          if (holdDuration >= holdRequired) {
            intent = mapping.intent;
          }
        } else {
          this.buttonStates.set(data.buttonId, { pressed: true, startTime: Date.now() });
        }
      } else {
        intent = mapping.intent;
      }
    } else if (!data.pressed) {
      this.buttonStates.delete(data.buttonId);
    }

    return {
      source: IntentSource.BUTTON,
      intent,
      confidence,
      latency: Date.now() - startTime,
    };
  }
}

// ============================================================================
// Intent Detector (Fusion)
// ============================================================================

export interface IIntentDetector {
  configure(config: Partial<IntentDetectorConfig>): void;
  getConfig(): IntentDetectorConfig;

  // Data input
  processEMG(data: EMGData): void;
  processGRF(data: GRFData): void;
  processIMU(data: IMUData[]): void;
  processButton(data: ButtonData): void;

  // Detection
  detectIntent(): IntentDetection;
  getLastDetection(): IntentDetection | null;

  // Calibration
  startCalibration(userId: string): void;
  getCalibrationState(): CalibrationState | null;
  applyCalibration(data: CalibrationData): void;

  // Event handlers
  onIntentDetected(callback: (detection: IntentDetection) => void): void;
}

export class IntentDetector implements IIntentDetector {
  private config: IntentDetectorConfig;
  private emgDetector?: EMGIntentDetector;
  private grfDetector?: GRFIntentDetector;
  private imuDetector?: IMUIntentDetector;
  private buttonDetector?: ButtonIntentDetector;

  private sourceResults: Map<IntentSource, SourceResult> = new Map();
  private lastDetection: IntentDetection | null = null;
  private lastDetectionTime: number = 0;
  private callbacks: ((detection: IntentDetection) => void)[] = [];
  private calibrationState: CalibrationState | null = null;

  constructor(config?: Partial<IntentDetectorConfig>) {
    this.config = { ...DEFAULT_INTENT_CONFIG, ...config };
    this.initializeDetectors();
  }

  private initializeDetectors(): void {
    if (this.config.enabledSources.includes(IntentSource.EMG) && this.config.emgConfig) {
      this.emgDetector = new EMGIntentDetector(this.config.emgConfig);
    }
    if (this.config.enabledSources.includes(IntentSource.GRF) && this.config.grfConfig) {
      this.grfDetector = new GRFIntentDetector(this.config.grfConfig);
    }
    if (this.config.enabledSources.includes(IntentSource.IMU) && this.config.imuConfig) {
      this.imuDetector = new IMUIntentDetector(this.config.imuConfig);
    }
    if (this.config.enabledSources.includes(IntentSource.BUTTON) && this.config.buttonConfig) {
      this.buttonDetector = new ButtonIntentDetector(this.config.buttonConfig);
    }
  }

  configure(config: Partial<IntentDetectorConfig>): void {
    this.config = { ...this.config, ...config };
    this.initializeDetectors();
  }

  getConfig(): IntentDetectorConfig {
    return { ...this.config };
  }

  // --------------------------------------------------------------------------
  // Data Processing
  // --------------------------------------------------------------------------

  processEMG(data: EMGData): void {
    if (this.emgDetector) {
      const result = this.emgDetector.detect(data);
      this.sourceResults.set(IntentSource.EMG, result);
    }
  }

  processGRF(data: GRFData): void {
    if (this.grfDetector) {
      const result = this.grfDetector.detect(data);
      this.sourceResults.set(IntentSource.GRF, result);
    }
  }

  processIMU(data: IMUData[]): void {
    if (this.imuDetector) {
      const result = this.imuDetector.detect(data);
      this.sourceResults.set(IntentSource.IMU, result);
    }
  }

  processButton(data: ButtonData): void {
    if (this.buttonDetector) {
      const result = this.buttonDetector.detect(data);
      this.sourceResults.set(IntentSource.BUTTON, result);

      // Buttons trigger immediate detection
      if (result.intent !== UserIntent.IDLE) {
        this.triggerDetection(result);
      }
    }
  }

  // --------------------------------------------------------------------------
  // Intent Detection
  // --------------------------------------------------------------------------

  detectIntent(): IntentDetection {
    const now = Date.now();

    // Debounce check
    if (now - this.lastDetectionTime < this.config.debounceTime && this.lastDetection) {
      return this.lastDetection;
    }

    // Collect all source results
    const allSources = Array.from(this.sourceResults.values()).filter(
      r => r.latency <= this.config.maxLatency
    );

    if (allSources.length === 0) {
      return this.createIdleDetection();
    }

    // Fuse results based on method
    const fusedResult = this.fuseResults(allSources);

    // Check confidence threshold
    if (fusedResult.confidence < this.config.confidenceThreshold) {
      return this.createIdleDetection();
    }

    // Create detection
    const detection: IntentDetection = {
      id: generateDetectionId(),
      timestamp: now,
      intent: fusedResult.intent,
      confidence: fusedResult.confidence,
      source: fusedResult.source,
      allSources,
      state: IntentState.CONFIRMED,
    };

    this.lastDetection = detection;
    this.lastDetectionTime = now;

    // Notify callbacks
    this.callbacks.forEach(cb => cb(detection));

    return detection;
  }

  private fuseResults(results: SourceResult[]): SourceResult {
    switch (this.config.fusionMethod) {
      case FusionMethod.WEIGHTED:
        return this.weightedFusion(results);
      case FusionMethod.VOTING:
        return this.votingFusion(results);
      default:
        return this.weightedFusion(results);
    }
  }

  private weightedFusion(results: SourceResult[]): SourceResult {
    // Priority weights
    const weights: Record<IntentSource, number> = {
      [IntentSource.BUTTON]: 1.0,
      [IntentSource.GRF]: 0.9,
      [IntentSource.EMG]: 0.8,
      [IntentSource.IMU]: 0.7,
      [IntentSource.BCI]: 0.6,
      [IntentSource.VOICE]: 0.5,
    };

    // Group by intent
    const intentScores = new Map<UserIntent, { score: number; count: number; topSource: IntentSource }>();

    for (const result of results) {
      if (result.intent === UserIntent.IDLE) continue;

      const weight = weights[result.source] ?? 0.5;
      const score = result.confidence * weight;

      const existing = intentScores.get(result.intent);
      if (existing) {
        if (score > existing.score / existing.count) {
          existing.topSource = result.source;
        }
        existing.score += score;
        existing.count++;
      } else {
        intentScores.set(result.intent, { score, count: 1, topSource: result.source });
      }
    }

    // Find best intent
    let bestIntent = UserIntent.IDLE;
    let bestScore = 0;
    let bestSource = IntentSource.IMU;

    intentScores.forEach((value, intent) => {
      const avgScore = value.score / value.count;
      if (avgScore > bestScore) {
        bestScore = avgScore;
        bestIntent = intent;
        bestSource = value.topSource;
      }
    });

    return {
      source: bestSource,
      intent: bestIntent,
      confidence: bestScore,
      latency: Math.max(...results.map(r => r.latency)),
    };
  }

  private votingFusion(results: SourceResult[]): SourceResult {
    const votes = new Map<UserIntent, number>();

    for (const result of results) {
      if (result.intent === UserIntent.IDLE) continue;
      votes.set(result.intent, (votes.get(result.intent) ?? 0) + 1);
    }

    let bestIntent = UserIntent.IDLE;
    let maxVotes = 0;

    votes.forEach((count, intent) => {
      if (count > maxVotes) {
        maxVotes = count;
        bestIntent = intent;
      }
    });

    const confidence = maxVotes / results.length;
    const primaryResult = results.find(r => r.intent === bestIntent);

    return {
      source: primaryResult?.source ?? IntentSource.IMU,
      intent: bestIntent,
      confidence,
      latency: Math.max(...results.map(r => r.latency)),
    };
  }

  private createIdleDetection(): IntentDetection {
    return {
      id: generateDetectionId(),
      timestamp: Date.now(),
      intent: UserIntent.IDLE,
      confidence: 1.0,
      source: IntentSource.IMU,
      allSources: [],
      state: IntentState.IDLE,
    };
  }

  private triggerDetection(result: SourceResult): void {
    const detection: IntentDetection = {
      id: generateDetectionId(),
      timestamp: Date.now(),
      intent: result.intent,
      confidence: result.confidence,
      source: result.source,
      allSources: [result],
      state: IntentState.CONFIRMED,
    };

    this.lastDetection = detection;
    this.lastDetectionTime = Date.now();
    this.callbacks.forEach(cb => cb(detection));
  }

  getLastDetection(): IntentDetection | null {
    return this.lastDetection;
  }

  // --------------------------------------------------------------------------
  // Calibration
  // --------------------------------------------------------------------------

  startCalibration(userId: string): void {
    this.calibrationState = {
      step: 'rest',
      progress: 0,
      trials: 0,
    };
  }

  getCalibrationState(): CalibrationState | null {
    return this.calibrationState;
  }

  applyCalibration(data: CalibrationData): void {
    // Apply EMG calibration
    if (data.emgBaseline && this.emgDetector) {
      Object.entries(data.emgBaseline).forEach(([ch, value]) => {
        this.emgDetector!.setBaseline(parseInt(ch), value);
      });
    }

    if (data.emgMVC && this.emgDetector) {
      Object.entries(data.emgMVC).forEach(([ch, value]) => {
        this.emgDetector!.calibrateMVC(parseInt(ch), value);
      });
    }

    this.calibrationState = {
      step: 'complete',
      progress: 100,
      trials: 0,
      successRate: 100,
    };
  }

  // --------------------------------------------------------------------------
  // Event Handlers
  // --------------------------------------------------------------------------

  onIntentDetected(callback: (detection: IntentDetection) => void): void {
    this.callbacks.push(callback);
  }
}

// ============================================================================
// Factory Function
// ============================================================================

export function createIntentDetector(
  config?: Partial<IntentDetectorConfig>
): IntentDetector {
  return new IntentDetector(config);
}
