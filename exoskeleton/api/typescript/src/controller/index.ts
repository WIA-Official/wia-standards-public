/**
 * WIA Exoskeleton Controller API
 * @module @wia/exoskeleton-controller
 * @version 1.0.0
 */

// Types
export {
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
} from './types';

// Controller
export {
  IExoController,
  IAssistanceController,
  ExoController,
  createExoController,
  generateMinimumJerkTrajectory,
  generateCycloidTrajectory,
} from './controller';
