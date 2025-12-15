/**
 * WIA Exoskeleton API
 * @module @wia/exoskeleton
 * @version 1.0.0
 *
 * Standard API for rehabilitation exoskeletons including
 * control interfaces and intent detection.
 *
 * @example
 * ```typescript
 * import {
 *   createExoController,
 *   createIntentDetector,
 *   JointType,
 *   ControlMode,
 *   UserIntent,
 * } from '@wia/exoskeleton';
 *
 * // Create controller
 * const controller = createExoController({ initialAssistance: 50 });
 *
 * // Apply impedance control
 * await controller.impedanceControl(JointType.KNEE, 'right', {
 *   stiffness: 30,
 *   damping: 2,
 *   equilibriumAngle: 20,
 * });
 *
 * // Create intent detector
 * const detector = createIntentDetector();
 * detector.onIntentDetected((detection) => {
 *   if (detection.intent !== UserIntent.IDLE) {
 *     console.log('Detected:', detection.intent);
 *   }
 * });
 * ```
 */

// Controller exports
export {
  // Types
  JointType,
  Side,
  ControlMode,
  AssistanceMode,
  CommandPriority,
  ResponseStatus,
  AdaptiveAlgorithm,
  TrajectoryType,
  PositionControlParams,
  VelocityControlParams,
  TorqueControlParams,
  ImpedanceParams,
  AdmittanceParams,
  ZeroTorqueParams,
  ControlParams,
  ControlCommand,
  ControlResponse,
  ControlError,
  AdaptiveAssistanceConfig,
  AssistanceState,
  JointLimits,
  BilateralJointLimits,
  RangeLimit,
  TrajectoryConfig,
  TrajectoryPoint,
  TrajectoryWaypoint,
  ControllerState,
  JointAssistanceLevels,
  // Classes and interfaces
  IExoController,
  IAssistanceController,
  ExoController,
  // Factory functions
  createExoController,
  // Trajectory generators
  generateMinimumJerkTrajectory,
  generateCycloidTrajectory,
} from './controller';

// Intent exports
export {
  // Types
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
  // Classes and interfaces
  EMGIntentDetector,
  GRFIntentDetector,
  IMUIntentDetector,
  ButtonIntentDetector,
  IIntentDetector,
  IntentDetector,
  // Factory functions
  createIntentDetector,
} from './intent';

/** Package version */
export const VERSION = '1.0.0';
