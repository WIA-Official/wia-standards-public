/**
 * WIA Exoskeleton Controller Implementation
 * @version 1.0.0
 */

import {
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
  TrajectoryConfig,
  TrajectoryPoint,
  ControllerState,
  RangeLimit,
} from './types';

// ============================================================================
// Default Values
// ============================================================================

const DEFAULT_JOINT_LIMITS: JointLimits = {
  hip: { min: -30, max: 120, velocityLimit: 200, torqueLimit: 60 },
  knee: { min: 0, max: 135, velocityLimit: 250, torqueLimit: 80 },
  ankle: { min: -30, max: 50, velocityLimit: 150, torqueLimit: 40 },
};

const DEFAULT_ADAPTIVE_CONFIG: AdaptiveAssistanceConfig = {
  enabled: false,
  algorithm: AdaptiveAlgorithm.ERROR_BASED,
  minAssistance: 0,
  maxAssistance: 100,
  adaptationRate: 2.0,
  windowSize: 1000,
  errorThresholdHigh: 10,
  errorThresholdLow: 5,
};

// ============================================================================
// Utility Functions
// ============================================================================

/** Generate unique command ID */
function generateCommandId(): string {
  return `cmd-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
}

/** Clamp value between min and max */
function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

/** Calculate RMSE between two trajectories */
function calculateRMSE(target: number[], actual: number[]): number {
  if (target.length !== actual.length || target.length === 0) {
    return Infinity;
  }
  const sumSquaredError = target.reduce((sum, t, i) => {
    const error = t - actual[i];
    return sum + error * error;
  }, 0);
  return Math.sqrt(sumSquaredError / target.length);
}

// ============================================================================
// Trajectory Generator
// ============================================================================

/** Generate minimum jerk trajectory */
export function generateMinimumJerkTrajectory(
  startAngle: number,
  endAngle: number,
  duration: number,
  sampleRate: number = 200
): TrajectoryPoint[] {
  const points: TrajectoryPoint[] = [];
  const numSamples = Math.floor((duration / 1000) * sampleRate);
  const T = duration / 1000; // Convert to seconds

  for (let i = 0; i <= numSamples; i++) {
    const t = (i / numSamples) * T;
    const tau = t / T;
    const tau3 = tau * tau * tau;
    const tau4 = tau3 * tau;
    const tau5 = tau4 * tau;

    // Position: θ(t) = θ0 + (θf - θ0) × [10τ³ - 15τ⁴ + 6τ⁵]
    const s = 10 * tau3 - 15 * tau4 + 6 * tau5;
    const angle = startAngle + (endAngle - startAngle) * s;

    // Velocity: θ̇(t) = (θf - θ0) / T × [30τ² - 60τ³ + 30τ⁴]
    const ds = (30 * tau * tau - 60 * tau3 + 30 * tau4) / T;
    const velocity = (endAngle - startAngle) * ds;

    // Acceleration: θ̈(t) = (θf - θ0) / T² × [60τ - 180τ² + 120τ³]
    const dds = (60 * tau - 180 * tau * tau + 120 * tau3) / (T * T);
    const acceleration = (endAngle - startAngle) * dds;

    points.push({
      time: t * 1000,
      angle,
      velocity,
      acceleration,
    });
  }

  return points;
}

/** Generate cycloid trajectory */
export function generateCycloidTrajectory(
  amplitude: number,
  period: number,
  cycles: number = 1,
  sampleRate: number = 200
): TrajectoryPoint[] {
  const points: TrajectoryPoint[] = [];
  const duration = period * cycles;
  const numSamples = Math.floor((duration / 1000) * sampleRate);
  const omega = (2 * Math.PI) / (period / 1000);

  for (let i = 0; i <= numSamples; i++) {
    const t = (i / numSamples) * (duration / 1000);
    const phase = omega * t;

    const angle = amplitude * (1 - Math.cos(phase)) / 2;
    const velocity = (amplitude * omega * Math.sin(phase)) / 2;
    const acceleration = (amplitude * omega * omega * Math.cos(phase)) / 2;

    points.push({
      time: t * 1000,
      angle,
      velocity,
      acceleration,
    });
  }

  return points;
}

// ============================================================================
// ExoController Interface
// ============================================================================

export interface IExoController {
  // Control mode
  setControlMode(mode: ControlMode): Promise<ControlResponse>;
  getControlMode(): ControlMode;

  // Position control
  positionControl(
    joint: JointType,
    side: Side,
    params: PositionControlParams
  ): Promise<ControlResponse>;

  // Velocity control
  velocityControl(
    joint: JointType,
    side: Side,
    params: VelocityControlParams
  ): Promise<ControlResponse>;

  // Torque control
  torqueControl(
    joint: JointType,
    side: Side,
    params: TorqueControlParams
  ): Promise<ControlResponse>;

  // Impedance control
  impedanceControl(
    joint: JointType,
    side: Side,
    params: ImpedanceParams
  ): Promise<ControlResponse>;

  // Admittance control
  admittanceControl(
    joint: JointType,
    side: Side,
    params: AdmittanceParams
  ): Promise<ControlResponse>;

  // Zero torque mode
  zeroTorqueControl(
    joint: JointType,
    side: Side,
    params: ZeroTorqueParams
  ): Promise<ControlResponse>;

  // Emergency stop
  emergencyStop(): Promise<ControlResponse>;

  // State
  getState(): ControllerState;
}

// ============================================================================
// AssistanceController Interface
// ============================================================================

export interface IAssistanceController {
  // Static assistance
  setAssistanceLevel(percent: number): void;
  getAssistanceLevel(): number;

  // Joint-specific assistance
  setJointAssistance(joint: JointType, side: Side, percent: number): void;
  getJointAssistance(joint: JointType, side: Side): number;

  // Assistance mode
  setAssistanceMode(mode: AssistanceMode): void;
  getAssistanceMode(): AssistanceMode;

  // Adaptive assistance
  configureAdaptiveAssistance(config: Partial<AdaptiveAssistanceConfig>): void;
  getAdaptiveConfig(): AdaptiveAssistanceConfig;

  // State
  getAssistanceState(): AssistanceState;
}

// ============================================================================
// ExoController Implementation
// ============================================================================

export class ExoController implements IExoController, IAssistanceController {
  private state: ControllerState;
  private assistanceState: AssistanceState;
  private jointLimits: BilateralJointLimits;
  private commandQueue: ControlCommand[] = [];

  constructor(config?: {
    jointLimits?: Partial<BilateralJointLimits>;
    initialAssistance?: number;
    assistanceMode?: AssistanceMode;
  }) {
    // Initialize joint limits
    this.jointLimits = {
      left: { ...DEFAULT_JOINT_LIMITS },
      right: { ...DEFAULT_JOINT_LIMITS },
      ...config?.jointLimits,
    };

    // Initialize controller state
    this.state = {
      mode: ControlMode.ZERO_TORQUE,
      assistanceMode: config?.assistanceMode ?? AssistanceMode.PASSIVE,
      isActive: false,
      activeJoints: [],
    };

    // Initialize assistance state
    const initialLevel = config?.initialAssistance ?? 50;
    this.assistanceState = {
      globalLevel: initialLevel,
      jointLevels: {
        left: { hip: initialLevel, knee: initialLevel, ankle: initialLevel },
        right: { hip: initialLevel, knee: initialLevel, ankle: initialLevel },
      },
      adaptiveConfig: { ...DEFAULT_ADAPTIVE_CONFIG },
      mode: config?.assistanceMode ?? AssistanceMode.PASSIVE,
    };
  }

  // --------------------------------------------------------------------------
  // Control Mode Methods
  // --------------------------------------------------------------------------

  async setControlMode(mode: ControlMode): Promise<ControlResponse> {
    const command: ControlCommand = {
      id: generateCommandId(),
      timestamp: Date.now(),
      joint: JointType.HIP, // Mode change affects all joints
      side: 'left',
      mode,
      params: {},
      priority: CommandPriority.NORMAL,
    };

    this.state.mode = mode;

    return {
      commandId: command.id,
      status: ResponseStatus.COMPLETED,
      timestamp: Date.now(),
    };
  }

  getControlMode(): ControlMode {
    return this.state.mode;
  }

  // --------------------------------------------------------------------------
  // Joint Control Methods
  // --------------------------------------------------------------------------

  async positionControl(
    joint: JointType,
    side: Side,
    params: PositionControlParams
  ): Promise<ControlResponse> {
    const limits = this.jointLimits[side][joint];

    // Validate target angle
    if (params.targetAngle < limits.min || params.targetAngle > limits.max) {
      return this.createErrorResponse(
        generateCommandId(),
        'ANGLE_OUT_OF_RANGE',
        `Target angle ${params.targetAngle}° is outside limits [${limits.min}°, ${limits.max}°]`
      );
    }

    const command: ControlCommand = {
      id: generateCommandId(),
      timestamp: Date.now(),
      joint,
      side,
      mode: ControlMode.POSITION,
      params: {
        ...params,
        maxVelocity: params.maxVelocity ?? limits.velocityLimit,
        kp: params.kp ?? 2.5,
        kd: params.kd ?? 0.1,
      },
      priority: CommandPriority.NORMAL,
    };

    return this.executeCommand(command);
  }

  async velocityControl(
    joint: JointType,
    side: Side,
    params: VelocityControlParams
  ): Promise<ControlResponse> {
    const limits = this.jointLimits[side][joint];

    // Validate velocity
    if (Math.abs(params.targetVelocity) > (limits.velocityLimit ?? 200)) {
      return this.createErrorResponse(
        generateCommandId(),
        'VELOCITY_OUT_OF_RANGE',
        `Target velocity ${params.targetVelocity}°/s exceeds limit`
      );
    }

    const command: ControlCommand = {
      id: generateCommandId(),
      timestamp: Date.now(),
      joint,
      side,
      mode: ControlMode.VELOCITY,
      params: {
        ...params,
        maxTorque: params.maxTorque ?? limits.torqueLimit,
      },
      priority: CommandPriority.NORMAL,
    };

    return this.executeCommand(command);
  }

  async torqueControl(
    joint: JointType,
    side: Side,
    params: TorqueControlParams
  ): Promise<ControlResponse> {
    const limits = this.jointLimits[side][joint];

    // Validate torque
    if (Math.abs(params.targetTorque) > (limits.torqueLimit ?? 80)) {
      return this.createErrorResponse(
        generateCommandId(),
        'TORQUE_OUT_OF_RANGE',
        `Target torque ${params.targetTorque}Nm exceeds limit`
      );
    }

    const command: ControlCommand = {
      id: generateCommandId(),
      timestamp: Date.now(),
      joint,
      side,
      mode: ControlMode.TORQUE,
      params: {
        ...params,
        rampRate: params.rampRate ?? 50, // Default 50 Nm/s
      },
      priority: CommandPriority.NORMAL,
    };

    return this.executeCommand(command);
  }

  async impedanceControl(
    joint: JointType,
    side: Side,
    params: ImpedanceParams
  ): Promise<ControlResponse> {
    // Validate stiffness and damping (must be non-negative)
    if (params.stiffness < 0 || params.damping < 0) {
      return this.createErrorResponse(
        generateCommandId(),
        'INVALID_IMPEDANCE_PARAMS',
        'Stiffness and damping must be non-negative'
      );
    }

    const command: ControlCommand = {
      id: generateCommandId(),
      timestamp: Date.now(),
      joint,
      side,
      mode: ControlMode.IMPEDANCE,
      params,
      priority: CommandPriority.NORMAL,
    };

    return this.executeCommand(command);
  }

  async admittanceControl(
    joint: JointType,
    side: Side,
    params: AdmittanceParams
  ): Promise<ControlResponse> {
    // Validate parameters
    if (params.virtualMass <= 0) {
      return this.createErrorResponse(
        generateCommandId(),
        'INVALID_ADMITTANCE_PARAMS',
        'Virtual mass must be positive'
      );
    }

    const command: ControlCommand = {
      id: generateCommandId(),
      timestamp: Date.now(),
      joint,
      side,
      mode: ControlMode.ADMITTANCE,
      params,
      priority: CommandPriority.NORMAL,
    };

    return this.executeCommand(command);
  }

  async zeroTorqueControl(
    joint: JointType,
    side: Side,
    params: ZeroTorqueParams
  ): Promise<ControlResponse> {
    const command: ControlCommand = {
      id: generateCommandId(),
      timestamp: Date.now(),
      joint,
      side,
      mode: ControlMode.ZERO_TORQUE,
      params,
      priority: CommandPriority.NORMAL,
    };

    return this.executeCommand(command);
  }

  async emergencyStop(): Promise<ControlResponse> {
    const command: ControlCommand = {
      id: generateCommandId(),
      timestamp: Date.now(),
      joint: JointType.HIP, // Affects all joints
      side: 'left',
      mode: ControlMode.ZERO_TORQUE,
      params: {
        frictionCompensation: false,
        gravityCompensation: false,
        inertiaCompensation: false,
      },
      priority: CommandPriority.EMERGENCY,
    };

    // Clear command queue
    this.commandQueue = [];
    this.state.isActive = false;
    this.state.activeJoints = [];

    return {
      commandId: command.id,
      status: ResponseStatus.COMPLETED,
      timestamp: Date.now(),
    };
  }

  getState(): ControllerState {
    return { ...this.state };
  }

  // --------------------------------------------------------------------------
  // Assistance Control Methods
  // --------------------------------------------------------------------------

  setAssistanceLevel(percent: number): void {
    const level = clamp(percent, 0, 100);
    this.assistanceState.globalLevel = level;

    // Update all joint levels
    const sides: Side[] = ['left', 'right'];
    const joints: JointType[] = [JointType.HIP, JointType.KNEE, JointType.ANKLE];

    for (const side of sides) {
      for (const joint of joints) {
        this.assistanceState.jointLevels[side][joint] = level;
      }
    }
  }

  getAssistanceLevel(): number {
    return this.assistanceState.globalLevel;
  }

  setJointAssistance(joint: JointType, side: Side, percent: number): void {
    const level = clamp(percent, 0, 100);
    this.assistanceState.jointLevels[side][joint] = level;
  }

  getJointAssistance(joint: JointType, side: Side): number {
    return this.assistanceState.jointLevels[side][joint];
  }

  setAssistanceMode(mode: AssistanceMode): void {
    this.assistanceState.mode = mode;
    this.state.assistanceMode = mode;
  }

  getAssistanceMode(): AssistanceMode {
    return this.assistanceState.mode;
  }

  configureAdaptiveAssistance(config: Partial<AdaptiveAssistanceConfig>): void {
    this.assistanceState.adaptiveConfig = {
      ...this.assistanceState.adaptiveConfig,
      ...config,
    };
  }

  getAdaptiveConfig(): AdaptiveAssistanceConfig {
    return { ...this.assistanceState.adaptiveConfig };
  }

  getAssistanceState(): AssistanceState {
    return { ...this.assistanceState };
  }

  // --------------------------------------------------------------------------
  // Adaptive Assistance Algorithm
  // --------------------------------------------------------------------------

  /**
   * Update assistance level based on tracking error (error-based adaptation)
   */
  updateAdaptiveAssistance(
    targetTrajectory: number[],
    actualTrajectory: number[]
  ): number {
    const config = this.assistanceState.adaptiveConfig;

    if (!config.enabled) {
      return this.assistanceState.globalLevel;
    }

    const rmse = calculateRMSE(targetTrajectory, actualTrajectory);
    let newAssistance = this.assistanceState.globalLevel;

    if (rmse > (config.errorThresholdHigh ?? 10)) {
      // Increase assistance
      newAssistance = Math.min(
        this.assistanceState.globalLevel + config.adaptationRate,
        config.maxAssistance
      );
    } else if (rmse < (config.errorThresholdLow ?? 5)) {
      // Decrease assistance (slower rate)
      newAssistance = Math.max(
        this.assistanceState.globalLevel - config.adaptationRate / 2,
        config.minAssistance
      );
    }

    this.setAssistanceLevel(newAssistance);
    return newAssistance;
  }

  // --------------------------------------------------------------------------
  // Private Methods
  // --------------------------------------------------------------------------

  private async executeCommand(command: ControlCommand): Promise<ControlResponse> {
    this.state.currentCommand = command;
    this.state.mode = command.mode;

    // Add to active joints if not already present
    const jointKey = `${command.joint}-${command.side}`;
    const isActive = this.state.activeJoints.some(
      j => j.joint === command.joint && j.side === command.side
    );
    if (!isActive) {
      this.state.activeJoints.push({ joint: command.joint, side: command.side });
    }

    this.state.isActive = true;

    // Simulate command execution
    const response: ControlResponse = {
      commandId: command.id,
      status: ResponseStatus.COMPLETED,
      timestamp: Date.now(),
    };

    this.state.lastResponse = response;
    return response;
  }

  private createErrorResponse(
    commandId: string,
    code: string,
    message: string
  ): ControlResponse {
    const error: ControlError = { code, message };
    this.state.errorState = error;

    return {
      commandId,
      status: ResponseStatus.ERROR,
      timestamp: Date.now(),
      error,
    };
  }
}

// ============================================================================
// Factory Function
// ============================================================================

export function createExoController(config?: {
  jointLimits?: Partial<BilateralJointLimits>;
  initialAssistance?: number;
  assistanceMode?: AssistanceMode;
}): ExoController {
  return new ExoController(config);
}
