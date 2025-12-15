/**
 * WIA Exoskeleton Controller Types
 * @version 1.0.0
 */

// ============================================================================
// Enumerations
// ============================================================================

/** Joint types supported by the exoskeleton */
export enum JointType {
  HIP = 'hip',
  KNEE = 'knee',
  ANKLE = 'ankle',
}

/** Body side */
export type Side = 'left' | 'right';

/** Control modes for the exoskeleton */
export enum ControlMode {
  /** Position tracking control */
  POSITION = 'position',
  /** Velocity tracking control */
  VELOCITY = 'velocity',
  /** Direct torque control */
  TORQUE = 'torque',
  /** Impedance control (virtual spring-damper) */
  IMPEDANCE = 'impedance',
  /** Admittance control (force to motion) */
  ADMITTANCE = 'admittance',
  /** Zero torque / transparent mode */
  ZERO_TORQUE = 'zero_torque',
}

/** Assistance modes */
export enum AssistanceMode {
  /** No motor resistance, free movement */
  PASSIVE = 'passive',
  /** Motor assists user movement */
  ACTIVE_ASSIST = 'active_assist',
  /** Motor resists user movement */
  ACTIVE_RESIST = 'active_resist',
  /** Minimal interference, friction/inertia compensation */
  TRANSPARENT = 'transparent',
  /** Adaptive resistance based on performance */
  CHALLENGE = 'challenge',
}

/** Command priority levels */
export enum CommandPriority {
  /** Emergency stop commands */
  EMERGENCY = 0,
  /** Safety-related commands */
  SAFETY = 1,
  /** Therapeutic/rehabilitation commands */
  THERAPEUTIC = 2,
  /** Normal control commands */
  NORMAL = 3,
  /** Background tasks */
  BACKGROUND = 4,
}

/** Command response status */
export enum ResponseStatus {
  ACCEPTED = 'accepted',
  EXECUTING = 'executing',
  COMPLETED = 'completed',
  REJECTED = 'rejected',
  TIMEOUT = 'timeout',
  ERROR = 'error',
}

/** Adaptive assistance algorithms */
export enum AdaptiveAlgorithm {
  /** Based on trajectory tracking error */
  ERROR_BASED = 'error_based',
  /** Based on EMG muscle activation */
  EMG_BASED = 'emg_based',
  /** Based on detected fatigue */
  FATIGUE_BASED = 'fatigue_based',
  /** Based on gait performance metrics */
  PERFORMANCE_BASED = 'performance_based',
}

/** Trajectory types */
export enum TrajectoryType {
  MINIMUM_JERK = 'minimum_jerk',
  CYCLOID = 'cycloid',
  GAIT_TEMPLATE = 'gait_template',
  CUSTOM = 'custom',
}

// ============================================================================
// Control Parameters
// ============================================================================

/** Position control parameters */
export interface PositionControlParams {
  targetAngle: number;        // degrees
  maxVelocity?: number;       // deg/s
  maxAcceleration?: number;   // deg/s²
  kp?: number;                // Proportional gain (Nm/deg)
  kd?: number;                // Derivative gain (Nm·s/deg)
}

/** Velocity control parameters */
export interface VelocityControlParams {
  targetVelocity: number;     // deg/s
  maxTorque?: number;         // Nm
  kv?: number;                // Velocity gain
  ki?: number;                // Integral gain
}

/** Torque control parameters */
export interface TorqueControlParams {
  targetTorque: number;       // Nm
  rampRate?: number;          // Nm/s
  feedforward?: number;       // Nm
}

/** Impedance control parameters */
export interface ImpedanceParams {
  stiffness: number;          // Nm/rad (K)
  damping: number;            // Nm·s/rad (B)
  equilibriumAngle: number;   // degrees (θ₀)
  inertia?: number;           // kg·m² (optional virtual inertia)
}

/** Admittance control parameters */
export interface AdmittanceParams {
  virtualMass: number;        // kg (M)
  virtualDamping: number;     // Ns/m (B)
  virtualStiffness: number;   // N/m (K)
  forceThreshold?: number;    // N (movement initiation threshold)
}

/** Zero torque mode parameters */
export interface ZeroTorqueParams {
  frictionCompensation: boolean;
  gravityCompensation: boolean;
  inertiaCompensation?: boolean;
}

/** Union type for all control parameters */
export type ControlParams =
  | PositionControlParams
  | VelocityControlParams
  | TorqueControlParams
  | ImpedanceParams
  | AdmittanceParams
  | ZeroTorqueParams;

// ============================================================================
// Control Commands
// ============================================================================

/** Control command structure */
export interface ControlCommand {
  id: string;
  timestamp: number;
  joint: JointType;
  side: Side;
  mode: ControlMode;
  params: ControlParams;
  priority: CommandPriority;
  timeout?: number;           // ms
}

/** Control error information */
export interface ControlError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

/** Control command response */
export interface ControlResponse {
  commandId: string;
  status: ResponseStatus;
  timestamp: number;
  actualValue?: number;
  error?: ControlError;
}

// ============================================================================
// Assistance Control
// ============================================================================

/** Joint-specific assistance levels */
export interface JointAssistanceLevels {
  hip: number;                // 0-100%
  knee: number;               // 0-100%
  ankle: number;              // 0-100%
}

/** Adaptive assistance configuration */
export interface AdaptiveAssistanceConfig {
  enabled: boolean;
  algorithm: AdaptiveAlgorithm;
  minAssistance: number;      // 0-100%
  maxAssistance: number;      // 0-100%
  adaptationRate: number;     // %/s
  windowSize?: number;        // ms
  errorThresholdHigh?: number;
  errorThresholdLow?: number;
}

/** Assistance state */
export interface AssistanceState {
  globalLevel: number;        // 0-100%
  jointLevels: {
    left: JointAssistanceLevels;
    right: JointAssistanceLevels;
  };
  adaptiveConfig: AdaptiveAssistanceConfig;
  mode: AssistanceMode;
}

// ============================================================================
// Joint Limits
// ============================================================================

/** Range limit for a single joint */
export interface RangeLimit {
  min: number;                // degrees (extension limit)
  max: number;                // degrees (flexion limit)
  velocityLimit?: number;     // deg/s
  torqueLimit?: number;       // Nm
}

/** Joint limits for all joints */
export interface JointLimits {
  hip: RangeLimit;
  knee: RangeLimit;
  ankle: RangeLimit;
}

/** Full joint limits with bilateral support */
export interface BilateralJointLimits {
  left: JointLimits;
  right: JointLimits;
}

// ============================================================================
// Trajectory
// ============================================================================

/** Waypoint for custom trajectory */
export interface TrajectoryWaypoint {
  time: number;               // ms from start
  angle: number;              // degrees
  velocity?: number;          // deg/s
  acceleration?: number;      // deg/s²
}

/** Trajectory configuration */
export interface TrajectoryConfig {
  type: TrajectoryType;
  duration?: number;          // ms
  startAngle?: number;        // degrees
  endAngle?: number;          // degrees
  amplitude?: number;         // degrees (for cycloid)
  period?: number;            // ms (for cycloid)
  gaitType?: string;          // for gait_template
  speed?: number;             // m/s (for gait_template)
  waypoints?: TrajectoryWaypoint[]; // for custom
}

/** Generated trajectory point */
export interface TrajectoryPoint {
  time: number;
  angle: number;
  velocity: number;
  acceleration: number;
}

// ============================================================================
// Controller State
// ============================================================================

/** Current controller state */
export interface ControllerState {
  mode: ControlMode;
  assistanceMode: AssistanceMode;
  isActive: boolean;
  activeJoints: Array<{ joint: JointType; side: Side }>;
  currentCommand?: ControlCommand;
  lastResponse?: ControlResponse;
  errorState?: ControlError;
}
