/**
 * WIA Eye Gaze Standard - BCI Integration
 *
 * Integration with Brain-Computer Interface (BCI) systems.
 * Enables multimodal gaze + EEG intent detection.
 *
 * 弘益人間 - 널리 인간을 이롭게
 *
 * @module @anthropics/wia-eye-gaze/integrations/bci
 */

import type { GazePoint, GazeEvent, Vector2D } from '../api/typescript/src/types';

/**
 * BCI Data from EEG device
 */
export interface BCIData {
  /** Timestamp (Unix ms) */
  timestamp: number;
  /** EEG channels data */
  channels: EEGChannel[];
  /** Processed mental states */
  mentalStates?: MentalStates;
  /** Event-related potentials */
  erp?: ERPData;
  /** Steady-state visually evoked potential */
  ssvep?: SSVEPData;
  /** Motor imagery classification */
  motorImagery?: MotorImageryData;
  /** Device information */
  device: BCIDeviceInfo;
}

/**
 * EEG Channel data
 */
export interface EEGChannel {
  /** Channel name (e.g., Fp1, Fp2, Cz) */
  name: string;
  /** Raw voltage values (µV) */
  values: number[];
  /** Sampling rate (Hz) */
  samplingRate: number;
  /** Impedance (kΩ) */
  impedance?: number;
}

/**
 * Mental states derived from EEG
 */
export interface MentalStates {
  /** Attention level (0-1) */
  attention: number;
  /** Relaxation level (0-1) */
  relaxation: number;
  /** Mental workload (0-1) */
  workload: number;
  /** Drowsiness level (0-1) */
  drowsiness: number;
  /** Engagement/interest (0-1) */
  engagement: number;
}

/**
 * Event-Related Potential data
 */
export interface ERPData {
  /** P300 detected */
  p300Detected: boolean;
  /** P300 amplitude (µV) */
  p300Amplitude?: number;
  /** P300 latency (ms) */
  p300Latency?: number;
  /** Target stimulus ID */
  targetId?: string;
}

/**
 * SSVEP (Steady-State Visually Evoked Potential) data
 */
export interface SSVEPData {
  /** Detected frequency (Hz) */
  frequency: number;
  /** Signal amplitude */
  amplitude: number;
  /** Confidence score */
  confidence: number;
  /** Associated target ID */
  targetId?: string;
}

/**
 * Motor Imagery classification
 */
export interface MotorImageryData {
  /** Classified action */
  action: 'left' | 'right' | 'both' | 'feet' | 'tongue' | 'rest';
  /** Classification confidence */
  confidence: number;
}

/**
 * BCI Device information
 */
export interface BCIDeviceInfo {
  /** Device name */
  name: string;
  /** Device vendor */
  vendor: string;
  /** Number of channels */
  channelCount: number;
  /** Sampling rate (Hz) */
  samplingRate: number;
  /** Connection type */
  connectionType: 'bluetooth' | 'usb' | 'wifi' | 'serial';
}

/**
 * Multimodal intent from combined gaze + BCI
 */
export interface MultimodalIntent {
  /** Intent type */
  type: IntentType;
  /** Confidence score (0-1) */
  confidence: number;
  /** Target element/position */
  target?: IntentTarget;
  /** Timestamp */
  timestamp: number;
  /** Contributing modalities */
  modalities: {
    gaze: GazeContribution;
    bci: BCIContribution;
  };
}

/**
 * Intent types
 */
export type IntentType =
  | 'select' // User wants to select something
  | 'reject' // User wants to reject/cancel
  | 'confirm' // User confirms action
  | 'scroll-up'
  | 'scroll-down'
  | 'scroll-left'
  | 'scroll-right'
  | 'zoom-in'
  | 'zoom-out'
  | 'navigate-back'
  | 'navigate-forward'
  | 'speak' // Activate speech output
  | 'attention' // User is focusing
  | 'idle' // No specific intent
  | 'custom';

/**
 * Intent target
 */
export interface IntentTarget {
  /** Target element ID */
  elementId?: string;
  /** Target position */
  position: Vector2D;
  /** Target label */
  label?: string;
  /** Fixation duration at target */
  fixationDuration?: number;
}

/**
 * Gaze contribution to intent
 */
export interface GazeContribution {
  /** Current gaze point */
  point: GazePoint;
  /** Fixation state */
  isFixation: boolean;
  /** Fixation duration (ms) */
  fixationDuration?: number;
  /** Confidence from gaze */
  confidence: number;
}

/**
 * BCI contribution to intent
 */
export interface BCIContribution {
  /** Mental states */
  mentalStates?: MentalStates;
  /** ERP detection */
  erpDetected?: boolean;
  /** SSVEP frequency */
  ssvepFrequency?: number;
  /** Motor imagery action */
  motorAction?: string;
  /** Confidence from BCI */
  confidence: number;
}

/**
 * Calibration point for BCI
 */
export interface CalibrationPoint {
  /** Screen position */
  position: Vector2D;
  /** Duration to fixate (ms) */
  duration: number;
  /** SSVEP frequency (if used) */
  ssvepFrequency?: number;
  /** Collected EEG data */
  eegData?: EEGChannel[];
}

/**
 * Calibration result
 */
export interface BCICalibrationResult {
  /** Success status */
  success: boolean;
  /** Calibration quality score (0-1) */
  quality: number;
  /** Individual point results */
  points: {
    position: Vector2D;
    quality: number;
    snr: number; // Signal-to-noise ratio
  }[];
  /** Recommended settings */
  recommendations?: {
    useSsvep: boolean;
    useErp: boolean;
    useMotorImagery: boolean;
  };
}

/**
 * Gaze to BCI Integration
 *
 * Combines gaze tracking with BCI data for enhanced intent detection.
 */
export class GazeToBCI {
  private lastGazePoint: GazePoint | null = null;
  private lastBciData: BCIData | null = null;
  private calibrationPoints: CalibrationPoint[] = [];
  private isCalibrating = false;

  /** Configuration */
  private config = {
    /** Minimum confidence for intent detection */
    confidenceThreshold: 0.7,
    /** Weight for gaze data (vs BCI) */
    gazeWeight: 0.5,
    /** Use P300 for selection */
    useP300: true,
    /** Use SSVEP for selection */
    useSsvep: false,
    /** Use motor imagery */
    useMotorImagery: false,
    /** Fusion method */
    fusionMethod: 'weighted' as 'weighted' | 'bayesian' | 'voting',
    /** Fixation threshold for attention (ms) */
    fixationThreshold: 200,
  };

  /** Intent callbacks */
  private intentCallbacks: ((intent: MultimodalIntent) => void)[] = [];

  constructor(options?: Partial<typeof GazeToBCI.prototype.config>) {
    if (options) {
      this.config = { ...this.config, ...options };
    }
  }

  /**
   * Combine gaze and EEG data for multimodal intent detection
   */
  combineWithEEG(gazeData: GazePoint, eegData: BCIData): MultimodalIntent {
    this.lastGazePoint = gazeData;
    this.lastBciData = eegData;

    // Analyze gaze contribution
    const gazeContribution = this.analyzeGaze(gazeData);

    // Analyze BCI contribution
    const bciContribution = this.analyzeBCI(eegData);

    // Fuse modalities
    const intent = this.fuseModalities(gazeContribution, bciContribution);

    // Notify callbacks if above threshold
    if (intent.confidence >= this.config.confidenceThreshold) {
      for (const callback of this.intentCallbacks) {
        callback(intent);
      }
    }

    return intent;
  }

  /**
   * Analyze gaze data contribution
   */
  private analyzeGaze(gaze: GazePoint): GazeContribution {
    const isFixation = gaze.fixation !== undefined;
    const fixationDuration = gaze.fixation?.duration;

    return {
      point: gaze,
      isFixation,
      fixationDuration,
      confidence: isFixation && fixationDuration && fixationDuration > this.config.fixationThreshold
        ? Math.min(fixationDuration / 1000, 1.0)
        : gaze.confidence,
    };
  }

  /**
   * Analyze BCI data contribution
   */
  private analyzeBCI(bci: BCIData): BCIContribution {
    let confidence = 0;
    const contribution: BCIContribution = {
      mentalStates: bci.mentalStates,
      confidence: 0,
    };

    // Check P300 ERP
    if (this.config.useP300 && bci.erp?.p300Detected) {
      contribution.erpDetected = true;
      confidence = Math.max(confidence, 0.8);
    }

    // Check SSVEP
    if (this.config.useSsvep && bci.ssvep && bci.ssvep.confidence > 0.7) {
      contribution.ssvepFrequency = bci.ssvep.frequency;
      confidence = Math.max(confidence, bci.ssvep.confidence);
    }

    // Check motor imagery
    if (this.config.useMotorImagery && bci.motorImagery) {
      contribution.motorAction = bci.motorImagery.action;
      confidence = Math.max(confidence, bci.motorImagery.confidence);
    }

    // Consider attention level
    if (bci.mentalStates) {
      const attentionBoost = bci.mentalStates.attention * 0.2;
      confidence = Math.min(confidence + attentionBoost, 1.0);
    }

    contribution.confidence = confidence;
    return contribution;
  }

  /**
   * Fuse gaze and BCI contributions
   */
  private fuseModalities(
    gaze: GazeContribution,
    bci: BCIContribution
  ): MultimodalIntent {
    let intentType: IntentType = 'idle';
    let confidence = 0;

    switch (this.config.fusionMethod) {
      case 'weighted':
        confidence =
          gaze.confidence * this.config.gazeWeight +
          bci.confidence * (1 - this.config.gazeWeight);
        break;

      case 'bayesian':
        // Simplified Bayesian fusion
        confidence =
          (gaze.confidence * bci.confidence) /
          (gaze.confidence * bci.confidence +
            (1 - gaze.confidence) * (1 - bci.confidence));
        break;

      case 'voting':
        const gazeVote = gaze.confidence > 0.5 ? 1 : 0;
        const bciVote = bci.confidence > 0.5 ? 1 : 0;
        confidence = (gazeVote + bciVote) / 2;
        break;
    }

    // Determine intent type
    if (bci.erpDetected && gaze.isFixation) {
      intentType = 'select';
    } else if (bci.motorAction === 'left') {
      intentType = 'scroll-left';
    } else if (bci.motorAction === 'right') {
      intentType = 'scroll-right';
    } else if (gaze.isFixation && gaze.fixationDuration && gaze.fixationDuration > 500) {
      intentType = 'attention';
    }

    // Build target from gaze position
    const target: IntentTarget | undefined = gaze.isFixation
      ? {
          position: { x: gaze.point.x, y: gaze.point.y },
          fixationDuration: gaze.fixationDuration,
        }
      : undefined;

    return {
      type: intentType,
      confidence,
      target,
      timestamp: Date.now(),
      modalities: { gaze, bci },
    };
  }

  /**
   * Set BCI calibration target using gaze
   */
  setBCICalibrationTarget(point: GazePoint): void {
    if (!this.isCalibrating) return;

    const calibrationPoint: CalibrationPoint = {
      position: { x: point.x, y: point.y },
      duration: 2000,
      eegData: this.lastBciData?.channels,
    };

    this.calibrationPoints.push(calibrationPoint);
    console.log(`Calibration point set at (${point.x.toFixed(3)}, ${point.y.toFixed(3)})`);
  }

  /**
   * Start BCI calibration
   */
  startCalibration(): void {
    this.isCalibrating = true;
    this.calibrationPoints = [];
    console.log('BCI calibration started');
  }

  /**
   * Complete BCI calibration
   */
  completeCalibration(): BCICalibrationResult {
    this.isCalibrating = false;

    // Analyze calibration quality (simplified)
    const quality =
      this.calibrationPoints.length >= 9
        ? 0.9
        : this.calibrationPoints.length / 9;

    const result: BCICalibrationResult = {
      success: quality > 0.7,
      quality,
      points: this.calibrationPoints.map((p) => ({
        position: p.position,
        quality: Math.random() * 0.3 + 0.7, // Simulated
        snr: Math.random() * 5 + 5, // Simulated SNR
      })),
      recommendations: {
        useSsvep: quality > 0.8,
        useErp: quality > 0.6,
        useMotorImagery: quality > 0.5,
      },
    };

    console.log('BCI calibration complete:', result);
    return result;
  }

  /**
   * Register intent callback
   */
  onIntent(callback: (intent: MultimodalIntent) => void): void {
    this.intentCallbacks.push(callback);
  }

  /**
   * Update configuration
   */
  configure(options: Partial<typeof GazeToBCI.prototype.config>): void {
    this.config = { ...this.config, ...options };
  }

  /**
   * Get current configuration
   */
  getConfig(): typeof GazeToBCI.prototype.config {
    return { ...this.config };
  }

  /**
   * Cleanup
   */
  dispose(): void {
    this.intentCallbacks = [];
    this.calibrationPoints = [];
    this.lastGazePoint = null;
    this.lastBciData = null;
  }
}

/**
 * Supported BCI devices
 */
export const SupportedBCIDevices = {
  EMOTIV_EPOC: 'emotiv-epoc',
  EMOTIV_INSIGHT: 'emotiv-insight',
  OPENBCI_CYTON: 'openbci-cyton',
  OPENBCI_GANGLION: 'openbci-ganglion',
  MUSE_2: 'muse-2',
  NEUROSKY_MINDWAVE: 'neurosky-mindwave',
  NEUROSITY_CROWN: 'neurosity-crown',
};

export default GazeToBCI;
