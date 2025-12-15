/**
 * WIA Myoelectric Real-time Streaming Module
 * @module streaming
 */

import { Observable, Subject, BehaviorSubject, interval, Subscription } from 'rxjs';
import { map, filter, buffer, bufferTime, share, takeUntil } from 'rxjs/operators';
import { FeatureVector, ClassificationResult, Gesture } from '../classifier/types';
import { GestureClassifier } from '../classifier/classifier';

/**
 * EMG sample from hardware
 */
export interface EMGSample {
  /** Timestamp in milliseconds */
  timestamp: number;
  /** Channel data (raw ADC values) */
  channels: number[];
}

/**
 * Streaming configuration
 */
export interface StreamingConfig {
  /** Sample rate in Hz */
  sampleRate: number;
  /** Feature extraction window size in ms */
  windowSizeMs: number;
  /** Window overlap percentage */
  overlapPercent: number;
  /** Enable smoothing filter */
  enableSmoothing: boolean;
  /** Smoothing factor (0-1) */
  smoothingFactor: number;
}

/**
 * Default streaming configuration
 */
export const DEFAULT_STREAMING_CONFIG: StreamingConfig = {
  sampleRate: 1000,
  windowSizeMs: 200,
  overlapPercent: 50,
  enableSmoothing: true,
  smoothingFactor: 0.3,
};

/**
 * Connection state
 */
export type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error';

/**
 * Real-time EMG streaming pipeline
 */
export class EMGStreamingPipeline {
  private config: StreamingConfig;
  private classifier: GestureClassifier;

  private sampleBuffer: EMGSample[] = [];
  private windowSamples: number;
  private hopSamples: number;

  private rawSampleSubject = new Subject<EMGSample>();
  private featureSubject = new Subject<FeatureVector>();
  private classificationSubject = new Subject<ClassificationResult>();
  private connectionStateSubject = new BehaviorSubject<ConnectionState>('disconnected');

  private stopSubject = new Subject<void>();
  private subscriptions: Subscription[] = [];

  private lastGesture: Gesture = Gesture.REST;
  private smoothedConfidence: number = 0;

  /**
   * Observable for raw EMG samples
   */
  public rawSamples$ = this.rawSampleSubject.asObservable();

  /**
   * Observable for extracted features
   */
  public features$ = this.featureSubject.asObservable();

  /**
   * Observable for classification results
   */
  public classifications$ = this.classificationSubject.asObservable();

  /**
   * Observable for connection state
   */
  public connectionState$ = this.connectionStateSubject.asObservable();

  constructor(
    classifier: GestureClassifier,
    config: Partial<StreamingConfig> = {}
  ) {
    this.classifier = classifier;
    this.config = { ...DEFAULT_STREAMING_CONFIG, ...config };

    this.windowSamples = Math.round(
      (this.config.windowSizeMs / 1000) * this.config.sampleRate
    );
    this.hopSamples = Math.round(
      this.windowSamples * (1 - this.config.overlapPercent / 100)
    );

    this.setupPipeline();
  }

  /**
   * Set up the processing pipeline
   */
  private setupPipeline(): void {
    // Feature extraction from raw samples
    const featureSub = this.rawSamples$
      .pipe(takeUntil(this.stopSubject))
      .subscribe((sample) => {
        this.processSample(sample);
      });

    this.subscriptions.push(featureSub);

    // Classification from features
    const classifySub = this.features$
      .pipe(takeUntil(this.stopSubject))
      .subscribe((features) => {
        const result = this.classifier.classify(features);
        const smoothed = this.applySmoothing(result);
        this.classificationSubject.next(smoothed);
      });

    this.subscriptions.push(classifySub);
  }

  /**
   * Process a single EMG sample
   */
  private processSample(sample: EMGSample): void {
    this.sampleBuffer.push(sample);

    // Check if we have enough samples for a window
    if (this.sampleBuffer.length >= this.windowSamples) {
      const window = this.sampleBuffer.slice(0, this.windowSamples);
      const features = this.extractFeatures(window);
      this.featureSubject.next(features);

      // Slide window by hop size
      this.sampleBuffer = this.sampleBuffer.slice(this.hopSamples);
    }
  }

  /**
   * Extract features from a window of samples
   */
  private extractFeatures(window: EMGSample[]): FeatureVector {
    const numChannels = window[0].channels.length;
    const features: number[] = [];
    const featureNames: string[] = [];

    for (let ch = 0; ch < numChannels; ch++) {
      const channelData = window.map((s) => s.channels[ch]);

      // Mean Absolute Value
      const mav = channelData.reduce((sum, v) => sum + Math.abs(v), 0) / channelData.length;
      features.push(mav);
      featureNames.push(`ch${ch}_mav`);

      // Root Mean Square
      const rms = Math.sqrt(
        channelData.reduce((sum, v) => sum + v * v, 0) / channelData.length
      );
      features.push(rms);
      featureNames.push(`ch${ch}_rms`);

      // Waveform Length
      let wl = 0;
      for (let i = 1; i < channelData.length; i++) {
        wl += Math.abs(channelData[i] - channelData[i - 1]);
      }
      features.push(wl);
      featureNames.push(`ch${ch}_wl`);

      // Zero Crossings
      let zc = 0;
      const threshold = 0.01;
      for (let i = 1; i < channelData.length; i++) {
        if (
          Math.sign(channelData[i]) !== Math.sign(channelData[i - 1]) &&
          Math.abs(channelData[i] - channelData[i - 1]) > threshold
        ) {
          zc++;
        }
      }
      features.push(zc);
      featureNames.push(`ch${ch}_zc`);

      // Slope Sign Changes
      let ssc = 0;
      for (let i = 1; i < channelData.length - 1; i++) {
        const diff1 = channelData[i] - channelData[i - 1];
        const diff2 = channelData[i] - channelData[i + 1];
        if (diff1 * diff2 > 0.0001) {
          ssc++;
        }
      }
      features.push(ssc);
      featureNames.push(`ch${ch}_ssc`);
    }

    return {
      timestamp: window[window.length - 1].timestamp,
      channelCount: numChannels,
      featureNames,
      features,
    };
  }

  /**
   * Apply smoothing to classification results
   */
  private applySmoothing(result: ClassificationResult): ClassificationResult {
    if (!this.config.enableSmoothing) {
      return result;
    }

    const alpha = this.config.smoothingFactor;

    // Exponential moving average for confidence
    if (result.gesture === this.lastGesture) {
      this.smoothedConfidence =
        alpha * result.confidence + (1 - alpha) * this.smoothedConfidence;
    } else {
      // Reset on gesture change
      this.smoothedConfidence = result.confidence;
      this.lastGesture = result.gesture;
    }

    return {
      ...result,
      confidence: this.smoothedConfidence,
    };
  }

  /**
   * Feed a raw EMG sample into the pipeline
   */
  feedSample(sample: EMGSample): void {
    this.rawSampleSubject.next(sample);
  }

  /**
   * Feed multiple samples
   */
  feedSamples(samples: EMGSample[]): void {
    for (const sample of samples) {
      this.rawSampleSubject.next(sample);
    }
  }

  /**
   * Connect to an EMG device stream
   */
  connectToStream(source: Observable<EMGSample>): Subscription {
    this.connectionStateSubject.next('connecting');

    const sub = source.pipe(takeUntil(this.stopSubject)).subscribe({
      next: (sample) => {
        if (this.connectionStateSubject.value !== 'connected') {
          this.connectionStateSubject.next('connected');
        }
        this.feedSample(sample);
      },
      error: (err) => {
        console.error('Stream error:', err);
        this.connectionStateSubject.next('error');
      },
      complete: () => {
        this.connectionStateSubject.next('disconnected');
      },
    });

    this.subscriptions.push(sub);
    return sub;
  }

  /**
   * Stop the pipeline
   */
  stop(): void {
    this.stopSubject.next();
    this.stopSubject.complete();

    for (const sub of this.subscriptions) {
      sub.unsubscribe();
    }

    this.connectionStateSubject.next('disconnected');
  }

  /**
   * Reset the pipeline state
   */
  reset(): void {
    this.sampleBuffer = [];
    this.lastGesture = Gesture.REST;
    this.smoothedConfidence = 0;
  }

  /**
   * Get current configuration
   */
  getConfig(): StreamingConfig {
    return { ...this.config };
  }

  /**
   * Update configuration
   */
  updateConfig(config: Partial<StreamingConfig>): void {
    this.config = { ...this.config, ...config };
    this.windowSamples = Math.round(
      (this.config.windowSizeMs / 1000) * this.config.sampleRate
    );
    this.hopSamples = Math.round(
      this.windowSamples * (1 - this.config.overlapPercent / 100)
    );
  }
}

/**
 * Gesture event detector for prosthetic control
 */
export class GestureEventDetector {
  private lastGesture: Gesture = Gesture.REST;
  private gestureStartTime: number = 0;
  private minDurationMs: number;
  private onGestureStart?: (gesture: Gesture) => void;
  private onGestureEnd?: (gesture: Gesture, durationMs: number) => void;

  constructor(
    minDurationMs: number = 100,
    onGestureStart?: (gesture: Gesture) => void,
    onGestureEnd?: (gesture: Gesture, durationMs: number) => void
  ) {
    this.minDurationMs = minDurationMs;
    this.onGestureStart = onGestureStart;
    this.onGestureEnd = onGestureEnd;
  }

  /**
   * Process a classification result
   */
  process(result: ClassificationResult): GestureEvent | null {
    const { gesture, timestamp } = result;

    if (gesture !== this.lastGesture) {
      // Gesture changed
      const previousGesture = this.lastGesture;
      const duration = timestamp - this.gestureStartTime;

      // End previous gesture if it was long enough
      if (previousGesture !== Gesture.REST && duration >= this.minDurationMs) {
        this.onGestureEnd?.(previousGesture, duration);
      }

      // Start new gesture
      this.lastGesture = gesture;
      this.gestureStartTime = timestamp;

      if (gesture !== Gesture.REST) {
        this.onGestureStart?.(gesture);
        return {
          type: 'start',
          gesture,
          timestamp,
        };
      }

      return {
        type: 'end',
        gesture: previousGesture,
        timestamp,
        durationMs: duration,
      };
    }

    return null;
  }
}

/**
 * Gesture event types
 */
export type GestureEvent =
  | { type: 'start'; gesture: Gesture; timestamp: number }
  | { type: 'end'; gesture: Gesture; timestamp: number; durationMs: number };
