/**
 * WIA Exoskeleton Intent Detection API
 * @module @wia/exoskeleton-intent
 * @version 1.0.0
 */

// Types
export {
  UserIntent,
  IntentSource,
  IntentState,
  ConfirmationMode,
  FusionMethod,
  Vector2D,
  Vector3D,
  EMGChannel,
  EMGFeatures,
  EMGData,
  EMGConfig,
  GRFData,
  GRFFeatures,
  GRFConfig,
  IMUData,
  IMUFeatures,
  IMUConfig,
  ButtonData,
  ButtonConfig,
  SourceResult,
  FeatureVector,
  IntentDetection,
  IntentDetectorConfig,
  IntentThresholds,
  CalibrationData,
  CalibrationState,
  DEFAULT_INTENT_CONFIG,
  DEFAULT_THRESHOLDS,
} from './types';

// Detectors
export {
  EMGIntentDetector,
  GRFIntentDetector,
  IMUIntentDetector,
  ButtonIntentDetector,
  IIntentDetector,
  IntentDetector,
  createIntentDetector,
} from './detector';
