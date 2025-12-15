/**
 * WIA Myoelectric Calibration Module
 * @module calibration
 */

import { Observable, Subject, BehaviorSubject } from 'rxjs';
import {
  Gesture,
  FeatureVector,
  LabeledSample,
  TrainingDataset,
  DatasetMetadata,
  FeatureConfig,
  FeatureType,
} from '../classifier/types';
import { GestureClassifier } from '../classifier/classifier';

/**
 * Calibration quality levels
 */
export type CalibrationQuality = 'poor' | 'fair' | 'good' | 'excellent';

/**
 * Progress information for calibration
 */
export interface CalibrationProgress {
  /** Current gesture being calibrated */
  currentGesture: Gesture;
  /** Number of samples collected for current gesture */
  samplesCollected: number;
  /** Number of samples required per gesture */
  samplesRequired: number;
  /** Quality assessment of collected samples */
  quality: CalibrationQuality;
  /** Overall progress percentage */
  overallProgress: number;
  /** Gestures completed */
  completedGestures: Gesture[];
  /** Gestures remaining */
  remainingGestures: Gesture[];
}

/**
 * User-specific trained model
 */
export interface UserModel {
  /** User identifier */
  userId: string;
  /** Creation timestamp */
  createdAt: Date;
  /** Gestures included in the model */
  gestures: Gesture[];
  /** Model accuracy on calibration data */
  accuracy: number;
  /** Overall calibration quality */
  calibrationQuality: number;
  /** Serialized model data */
  modelData: string;
}

/**
 * Calibration session configuration
 */
export interface CalibrationConfig {
  /** Gestures to calibrate */
  gestures: Gesture[];
  /** Samples required per gesture */
  samplesPerGesture: number;
  /** Sample duration in milliseconds */
  sampleDurationMs: number;
  /** Rest period between samples in milliseconds */
  restPeriodMs: number;
  /** Minimum quality threshold */
  minQualityThreshold: CalibrationQuality;
}

/**
 * Default calibration configuration
 */
export const DEFAULT_CALIBRATION_CONFIG: CalibrationConfig = {
  gestures: [
    Gesture.REST,
    Gesture.HAND_OPEN,
    Gesture.HAND_CLOSE,
    Gesture.WRIST_FLEXION,
    Gesture.WRIST_EXTENSION,
  ],
  samplesPerGesture: 10,
  sampleDurationMs: 3000,
  restPeriodMs: 2000,
  minQualityThreshold: 'fair',
};

/**
 * Calibration session for collecting user-specific training data
 */
export class CalibrationSession {
  private config: CalibrationConfig;
  private classifier: GestureClassifier;
  private samples: Map<Gesture, LabeledSample[]> = new Map();
  private currentGestureIndex: number = 0;
  private isActive: boolean = false;
  private userId: string;

  private progressSubject = new BehaviorSubject<CalibrationProgress | null>(null);
  private stateSubject = new Subject<CalibrationState>();

  /**
   * Observable for progress updates
   */
  public progress$ = this.progressSubject.asObservable();

  /**
   * Observable for state changes
   */
  public state$ = this.stateSubject.asObservable();

  constructor(
    userId: string,
    classifier: GestureClassifier,
    config: Partial<CalibrationConfig> = {}
  ) {
    this.userId = userId;
    this.classifier = classifier;
    this.config = { ...DEFAULT_CALIBRATION_CONFIG, ...config };

    // Initialize sample storage
    for (const gesture of this.config.gestures) {
      this.samples.set(gesture, []);
    }
  }

  /**
   * Start the calibration session
   */
  start(): void {
    this.isActive = true;
    this.currentGestureIndex = 0;
    this.stateSubject.next({
      type: 'started',
      gesture: this.getCurrentGesture(),
    });
    this.updateProgress();
  }

  /**
   * Stop the calibration session
   */
  stop(): void {
    this.isActive = false;
    this.stateSubject.next({ type: 'stopped' });
  }

  /**
   * Collect a sample for the current gesture
   */
  async collectSample(features: FeatureVector): Promise<void> {
    if (!this.isActive) {
      throw new Error('Calibration session is not active');
    }

    const gesture = this.getCurrentGesture();
    const gestureSamples = this.samples.get(gesture)!;

    // Create labeled sample
    const sample: LabeledSample = {
      features,
      label: gesture,
      subjectId: this.userId,
    };

    gestureSamples.push(sample);

    // Check if we have enough samples for this gesture
    if (gestureSamples.length >= this.config.samplesPerGesture) {
      this.stateSubject.next({
        type: 'gesture_complete',
        gesture,
      });

      // Move to next gesture
      this.currentGestureIndex++;

      if (this.currentGestureIndex >= this.config.gestures.length) {
        this.stateSubject.next({ type: 'all_complete' });
      } else {
        this.stateSubject.next({
          type: 'next_gesture',
          gesture: this.getCurrentGesture(),
        });
      }
    }

    this.updateProgress();
  }

  /**
   * Get the current gesture being calibrated
   */
  getCurrentGesture(): Gesture {
    return this.config.gestures[this.currentGestureIndex];
  }

  /**
   * Get current progress
   */
  getProgress(): CalibrationProgress {
    const currentGesture = this.getCurrentGesture();
    const gestureSamples = this.samples.get(currentGesture) ?? [];
    const completedGestures = this.config.gestures.slice(
      0,
      this.currentGestureIndex
    );
    const remainingGestures = this.config.gestures.slice(
      this.currentGestureIndex + 1
    );

    // Calculate overall progress
    let totalSamples = 0;
    for (const samples of this.samples.values()) {
      totalSamples += samples.length;
    }
    const totalRequired =
      this.config.gestures.length * this.config.samplesPerGesture;
    const overallProgress = (totalSamples / totalRequired) * 100;

    return {
      currentGesture,
      samplesCollected: gestureSamples.length,
      samplesRequired: this.config.samplesPerGesture,
      quality: this.assessQuality(gestureSamples),
      overallProgress,
      completedGestures,
      remainingGestures,
    };
  }

  /**
   * Complete calibration and generate user model
   */
  async complete(): Promise<UserModel> {
    if (!this.isAllComplete()) {
      throw new Error('Calibration is not complete');
    }

    // Collect all samples into a dataset
    const allSamples: LabeledSample[] = [];
    for (const samples of this.samples.values()) {
      allSamples.push(...samples);
    }

    const dataset: TrainingDataset = {
      id: `calibration_${this.userId}_${Date.now()}`,
      name: `User ${this.userId} Calibration`,
      samplesPerGesture: new Map(
        this.config.gestures.map((g) => [
          g,
          this.samples.get(g)?.length ?? 0,
        ])
      ),
      samples: allSamples,
      metadata: {
        createdAt: new Date(),
        subjectCount: 1,
        totalSamples: allSamples.length,
        featureConfig: {
          windowSizeMs: 200,
          overlapPercent: 50,
          sampleRate: 1000,
          featureTypes: [
            FeatureType.MAV,
            FeatureType.RMS,
            FeatureType.WL,
            FeatureType.ZC,
            FeatureType.SSC,
          ],
        },
      },
    };

    // Train the classifier
    const result = await this.classifier.train(dataset);

    // Create user model
    const userModel: UserModel = {
      userId: this.userId,
      createdAt: new Date(),
      gestures: this.config.gestures,
      accuracy: result.accuracy,
      calibrationQuality: this.calculateOverallQuality(),
      modelData: '', // Would contain serialized model
    };

    this.isActive = false;
    this.stateSubject.next({ type: 'model_created', model: userModel });

    return userModel;
  }

  /**
   * Check if calibration is complete
   */
  isAllComplete(): boolean {
    for (const gesture of this.config.gestures) {
      const samples = this.samples.get(gesture) ?? [];
      if (samples.length < this.config.samplesPerGesture) {
        return false;
      }
    }
    return true;
  }

  /**
   * Assess quality of collected samples
   */
  private assessQuality(samples: LabeledSample[]): CalibrationQuality {
    if (samples.length === 0) return 'poor';

    // Calculate variance of features across samples
    const featureVariances: number[] = [];
    const numFeatures = samples[0].features.features.length;

    for (let i = 0; i < numFeatures; i++) {
      const values = samples.map((s) => s.features.features[i]);
      const mean = values.reduce((a, b) => a + b, 0) / values.length;
      const variance =
        values.reduce((sum, v) => sum + (v - mean) ** 2, 0) / values.length;
      featureVariances.push(variance);
    }

    const avgVariance =
      featureVariances.reduce((a, b) => a + b, 0) / featureVariances.length;

    // Lower variance = more consistent = better quality
    // These thresholds would need tuning based on actual data
    if (avgVariance < 0.01) return 'excellent';
    if (avgVariance < 0.05) return 'good';
    if (avgVariance < 0.1) return 'fair';
    return 'poor';
  }

  /**
   * Calculate overall calibration quality
   */
  private calculateOverallQuality(): number {
    const qualityScores: number[] = [];

    for (const gesture of this.config.gestures) {
      const samples = this.samples.get(gesture) ?? [];
      const quality = this.assessQuality(samples);
      const score =
        quality === 'excellent'
          ? 1.0
          : quality === 'good'
          ? 0.75
          : quality === 'fair'
          ? 0.5
          : 0.25;
      qualityScores.push(score);
    }

    return qualityScores.reduce((a, b) => a + b, 0) / qualityScores.length;
  }

  /**
   * Update progress observable
   */
  private updateProgress(): void {
    this.progressSubject.next(this.getProgress());
  }
}

/**
 * Calibration state events
 */
export type CalibrationState =
  | { type: 'started'; gesture: Gesture }
  | { type: 'stopped' }
  | { type: 'gesture_complete'; gesture: Gesture }
  | { type: 'next_gesture'; gesture: Gesture }
  | { type: 'all_complete' }
  | { type: 'model_created'; model: UserModel };
