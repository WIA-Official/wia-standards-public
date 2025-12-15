/**
 * WIA Myoelectric Gesture Recognition - Type Definitions
 * @module classifier/types
 */

/**
 * Supported hand gestures for myoelectric control
 */
export enum Gesture {
  REST = 'rest',
  HAND_OPEN = 'hand_open',
  HAND_CLOSE = 'hand_close',
  WRIST_FLEXION = 'wrist_flexion',
  WRIST_EXTENSION = 'wrist_extension',
  WRIST_PRONATION = 'wrist_pronation',
  WRIST_SUPINATION = 'wrist_supination',
  PINCH_THUMB_INDEX = 'pinch_thumb_index',
  PINCH_THUMB_MIDDLE = 'pinch_thumb_middle',
  POINT_INDEX = 'point_index',
  THUMBS_UP = 'thumbs_up',
}

/**
 * Feature vector extracted from EMG signals
 */
export interface FeatureVector {
  /** Timestamp in milliseconds */
  timestamp: number;
  /** Number of EMG channels */
  channelCount: number;
  /** Feature names for each value */
  featureNames: string[];
  /** Flattened feature values */
  features: number[];
}

/**
 * Result of gesture classification
 */
export interface ClassificationResult {
  /** Primary classified gesture */
  gesture: Gesture;
  /** Confidence score (0.0 - 1.0) */
  confidence: number;
  /** Alternative classifications with lower confidence */
  alternatives: AlternativeClassification[];
  /** Classification latency in milliseconds */
  latencyMs: number;
  /** Timestamp of classification */
  timestamp: number;
}

/**
 * Alternative classification result
 */
export interface AlternativeClassification {
  gesture: Gesture;
  confidence: number;
}

/**
 * Training dataset structure
 */
export interface TrainingDataset {
  /** Unique dataset identifier */
  id: string;
  /** Dataset name */
  name: string;
  /** Number of samples per gesture */
  samplesPerGesture: Map<Gesture, number>;
  /** Feature vectors with labels */
  samples: LabeledSample[];
  /** Metadata */
  metadata: DatasetMetadata;
}

/**
 * Single labeled sample for training
 */
export interface LabeledSample {
  /** Feature vector */
  features: FeatureVector;
  /** Ground truth gesture label */
  label: Gesture;
  /** Optional subject ID */
  subjectId?: string;
  /** Optional session ID */
  sessionId?: string;
}

/**
 * Dataset metadata
 */
export interface DatasetMetadata {
  /** Creation date */
  createdAt: Date;
  /** Number of subjects */
  subjectCount: number;
  /** Total sample count */
  totalSamples: number;
  /** Feature extraction configuration */
  featureConfig: FeatureConfig;
}

/**
 * Feature extraction configuration
 */
export interface FeatureConfig {
  /** Window size in milliseconds */
  windowSizeMs: number;
  /** Window overlap percentage */
  overlapPercent: number;
  /** Sample rate in Hz */
  sampleRate: number;
  /** Feature types to extract */
  featureTypes: FeatureType[];
}

/**
 * Available feature types
 */
export enum FeatureType {
  MAV = 'mav',
  RMS = 'rms',
  WL = 'wl',
  ZC = 'zc',
  SSC = 'ssc',
  MNF = 'mnf',
  MDF = 'mdf',
  VAR = 'var',
  IEMG = 'iemg',
}

/**
 * Training result
 */
export interface TrainingResult {
  /** Whether training succeeded */
  success: boolean;
  /** Accuracy on validation set */
  accuracy: number;
  /** Per-gesture metrics */
  gestureMetrics: Map<Gesture, GestureMetrics>;
  /** Confusion matrix */
  confusionMatrix: number[][];
  /** Training duration in seconds */
  trainingDurationSec: number;
  /** Model size in bytes */
  modelSizeBytes: number;
}

/**
 * Per-gesture classification metrics
 */
export interface GestureMetrics {
  /** Precision */
  precision: number;
  /** Recall */
  recall: number;
  /** F1 score */
  f1Score: number;
  /** Number of samples */
  sampleCount: number;
}

/**
 * Classifier configuration
 */
export interface ClassifierConfig {
  /** Classification algorithm */
  algorithm: ClassificationAlgorithm;
  /** Confidence threshold for valid classification */
  confidenceThreshold: number;
  /** Number of consecutive frames for majority voting */
  majorityVotingFrames: number;
  /** Minimum gesture duration in milliseconds */
  minGestureDurationMs: number;
}

/**
 * Available classification algorithms
 */
export enum ClassificationAlgorithm {
  SVM = 'svm',
  LDA = 'lda',
  KNN = 'knn',
  RANDOM_FOREST = 'random_forest',
  CNN = 'cnn',
  LSTM = 'lstm',
  TCN = 'tcn',
  TRANSFORMER = 'transformer',
}

/**
 * Model information
 */
export interface ModelInfo {
  /** Model identifier */
  id: string;
  /** Model name */
  name: string;
  /** Algorithm used */
  algorithm: ClassificationAlgorithm;
  /** Supported gestures */
  gestures: Gesture[];
  /** Input feature dimension */
  inputDimension: number;
  /** Model accuracy */
  accuracy: number;
  /** Model version */
  version: string;
  /** Creation date */
  createdAt: Date;
}
