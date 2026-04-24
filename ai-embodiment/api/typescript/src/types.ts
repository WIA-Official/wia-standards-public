/**
 * WIA AI Embodiment Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-ai-embodiment
 */

/**
 * Embodiment type
 */
export enum EmbodimentType {
  Humanoid = 'humanoid',
  Quadruped = 'quadruped',
  Wheeled = 'wheeled',
  Aerial = 'aerial',
  Aquatic = 'aquatic',
  Manipulator = 'manipulator',
  SoftRobot = 'soft_robot',
  Hybrid = 'hybrid'
}

/**
 * Sensor modality
 */
export enum SensorModality {
  Vision = 'vision',
  Depth = 'depth',
  LIDAR = 'lidar',
  Touch = 'touch',
  Force = 'force',
  Proprioception = 'proprioception',
  Audio = 'audio',
  IMU = 'imu',
  GPS = 'gps',
  Temperature = 'temperature'
}

/**
 * Motor type
 */
export enum MotorType {
  Servo = 'servo',
  Stepper = 'stepper',
  Brushless = 'brushless',
  Linear = 'linear',
  Hydraulic = 'hydraulic',
  Pneumatic = 'pneumatic'
}

/**
 * AI capability
 */
export enum AICapability {
  Perception = 'perception',
  Navigation = 'navigation',
  Manipulation = 'manipulation',
  Speech = 'speech',
  NaturalLanguage = 'natural_language',
  Reasoning = 'reasoning',
  Learning = 'learning',
  Planning = 'planning',
  Emotion = 'emotion'
}

/**
 * Safety level
 */
export enum SafetyLevel {
  Minimal = 'minimal',
  Basic = 'basic',
  Collaborative = 'collaborative',
  HighSafety = 'high_safety'
}

/**
 * 3D position
 */
export interface Position3D {
  /** X coordinate */
  x: number;
  /** Y coordinate */
  y: number;
  /** Z coordinate */
  z: number;
}

/**
 * 3D orientation (quaternion)
 */
export interface Orientation3D {
  /** W component */
  w: number;
  /** X component */
  x: number;
  /** Y component */
  y: number;
  /** Z component */
  z: number;
}

/**
 * 6DOF pose
 */
export interface Pose {
  /** Position */
  position: Position3D;
  /** Orientation */
  orientation: Orientation3D;
  /** Reference frame */
  frame: string;
  /** Timestamp */
  timestamp: Date;
}

/**
 * Velocity
 */
export interface Velocity {
  /** Linear velocity */
  linear: Position3D;
  /** Angular velocity */
  angular: Position3D;
}

/**
 * Joint state
 */
export interface JointState {
  /** Joint name */
  name: string;
  /** Joint position (rad or m) */
  position: number;
  /** Joint velocity */
  velocity: number;
  /** Joint effort/torque */
  effort: number;
  /** Is joint active */
  active: boolean;
}

/**
 * Robot morphology
 */
export interface Morphology {
  /** Number of degrees of freedom */
  dof: number;
  /** Joint definitions */
  joints: JointDefinition[];
  /** Link definitions */
  links: LinkDefinition[];
  /** End effectors */
  endEffectors: EndEffector[];
}

/**
 * Joint definition
 */
export interface JointDefinition {
  /** Joint name */
  name: string;
  /** Joint type */
  type: 'revolute' | 'prismatic' | 'continuous' | 'fixed';
  /** Parent link */
  parentLink: string;
  /** Child link */
  childLink: string;
  /** Axis of rotation/translation */
  axis: Position3D;
  /** Lower limit */
  lowerLimit: number;
  /** Upper limit */
  upperLimit: number;
  /** Maximum velocity */
  maxVelocity: number;
  /** Maximum effort */
  maxEffort: number;
}

/**
 * Link definition
 */
export interface LinkDefinition {
  /** Link name */
  name: string;
  /** Link mass in kg */
  mass: number;
  /** Center of mass */
  centerOfMass: Position3D;
  /** Inertia matrix */
  inertia: number[];
  /** Visual geometry */
  visual?: Geometry;
  /** Collision geometry */
  collision?: Geometry;
}

/**
 * Geometry definition
 */
export interface Geometry {
  /** Geometry type */
  type: 'box' | 'sphere' | 'cylinder' | 'mesh';
  /** Dimensions */
  dimensions: number[];
  /** Mesh file path */
  meshPath?: string;
  /** Material/color */
  material?: string;
}

/**
 * End effector
 */
export interface EndEffector {
  /** Effector name */
  name: string;
  /** Effector type */
  type: 'gripper' | 'hand' | 'suction' | 'tool' | 'custom';
  /** Associated link */
  link: string;
  /** Workspace radius */
  workspaceRadius: number;
  /** Payload capacity in kg */
  payloadCapacity: number;
}

/**
 * Sensor definition
 */
export interface SensorDefinition {
  /** Sensor ID */
  id: string;
  /** Sensor name */
  name: string;
  /** Modality */
  modality: SensorModality;
  /** Mount link */
  mountLink: string;
  /** Pose relative to mount */
  mountPose: Pose;
  /** Update rate in Hz */
  updateRate: number;
  /** Resolution */
  resolution?: number[];
  /** Range */
  range?: [number, number];
  /** Field of view */
  fov?: number[];
}

/**
 * Sensor reading
 */
export interface SensorReading {
  /** Sensor ID */
  sensorId: string;
  /** Reading type */
  type: string;
  /** Data */
  data: unknown;
  /** Timestamp */
  timestamp: Date;
  /** Sequence number */
  sequence: number;
}

/**
 * Perception output
 */
export interface PerceptionOutput {
  /** Output type */
  type: 'object_detection' | 'scene_understanding' | 'pose_estimation' | 'segmentation';
  /** Detections */
  detections: Detection[];
  /** Confidence */
  confidence: number;
  /** Processing time in ms */
  processingTime: number;
  /** Timestamp */
  timestamp: Date;
}

/**
 * Object detection
 */
export interface Detection {
  /** Object class */
  class: string;
  /** Confidence score */
  confidence: number;
  /** 3D position */
  position?: Position3D;
  /** Bounding box */
  boundingBox?: BoundingBox;
  /** Object ID for tracking */
  trackId?: string;
}

/**
 * Bounding box
 */
export interface BoundingBox {
  /** Center position */
  center: Position3D;
  /** Dimensions */
  dimensions: Position3D;
  /** Orientation */
  orientation: Orientation3D;
}

/**
 * Motion command
 */
export interface MotionCommand {
  /** Command ID */
  id: string;
  /** Command type */
  type: 'joint' | 'cartesian' | 'velocity' | 'trajectory';
  /** Target */
  target: JointState[] | Pose | Velocity | Trajectory;
  /** Speed scale (0-1) */
  speedScale: number;
  /** Acceleration scale (0-1) */
  accelerationScale: number;
  /** Blend radius */
  blendRadius?: number;
}

/**
 * Trajectory
 */
export interface Trajectory {
  /** Waypoints */
  waypoints: Pose[];
  /** Timing */
  timing: number[];
  /** Interpolation method */
  interpolation: 'linear' | 'spline' | 'trapezoidal';
}

/**
 * Grasp definition
 */
export interface Grasp {
  /** Grasp ID */
  id: string;
  /** Object to grasp */
  objectId: string;
  /** Pre-grasp pose */
  preGraspPose: Pose;
  /** Grasp pose */
  graspPose: Pose;
  /** Grasp width */
  width: number;
  /** Grasp force */
  force: number;
  /** Approach vector */
  approachVector: Position3D;
}

/**
 * Task definition
 */
export interface Task {
  /** Task ID */
  id: string;
  /** Task name */
  name: string;
  /** Task type */
  type: 'pick' | 'place' | 'navigate' | 'inspect' | 'interact' | 'custom';
  /** Parameters */
  parameters: Record<string, unknown>;
  /** Priority */
  priority: number;
  /** Status */
  status: 'pending' | 'running' | 'completed' | 'failed' | 'cancelled';
  /** Created at */
  createdAt: Date;
  /** Started at */
  startedAt?: Date;
  /** Completed at */
  completedAt?: Date;
}

/**
 * Behavior definition
 */
export interface Behavior {
  /** Behavior ID */
  id: string;
  /** Behavior name */
  name: string;
  /** Behavior tree or state machine */
  structure: BehaviorNode;
  /** Is active */
  active: boolean;
}

/**
 * Behavior node
 */
export interface BehaviorNode {
  /** Node type */
  type: 'sequence' | 'selector' | 'parallel' | 'action' | 'condition';
  /** Node name */
  name: string;
  /** Children */
  children?: BehaviorNode[];
  /** Action function name */
  action?: string;
  /** Condition expression */
  condition?: string;
}

/**
 * Safety constraint
 */
export interface SafetyConstraint {
  /** Constraint ID */
  id: string;
  /** Constraint type */
  type: 'workspace' | 'velocity' | 'force' | 'proximity' | 'custom';
  /** Enabled */
  enabled: boolean;
  /** Parameters */
  parameters: Record<string, unknown>;
  /** Violation action */
  violationAction: 'stop' | 'slow' | 'warn' | 'avoid';
}

/**
 * Collision object
 */
export interface CollisionObject {
  /** Object ID */
  id: string;
  /** Object name */
  name: string;
  /** Geometry */
  geometry: Geometry;
  /** Pose */
  pose: Pose;
  /** Is static */
  isStatic: boolean;
}

/**
 * Natural language command
 */
export interface NLCommand {
  /** Command ID */
  id: string;
  /** Original text */
  text: string;
  /** Parsed intent */
  intent: string;
  /** Entities */
  entities: NLEntity[];
  /** Confidence */
  confidence: number;
  /** Generated actions */
  actions: Task[];
}

/**
 * NL entity
 */
export interface NLEntity {
  /** Entity type */
  type: string;
  /** Entity value */
  value: string;
  /** Position in text */
  start: number;
  /** End position */
  end: number;
}

/**
 * Emotional state
 */
export interface EmotionalState {
  /** Primary emotion */
  primary: string;
  /** Intensity (0-1) */
  intensity: number;
  /** Valence (-1 to 1) */
  valence: number;
  /** Arousal (0-1) */
  arousal: number;
  /** Expressed through */
  expression: 'facial' | 'body' | 'voice' | 'combined';
}

/**
 * Embodiment configuration
 */
export interface EmbodimentConfig {
  /** Robot ID */
  robotId: string;
  /** Robot name */
  robotName: string;
  /** Embodiment type */
  type: EmbodimentType;
  /** Morphology */
  morphology: Morphology;
  /** Sensors */
  sensors: SensorDefinition[];
  /** AI capabilities */
  capabilities: AICapability[];
  /** Safety level */
  safetyLevel: SafetyLevel;
  /** Control rate in Hz */
  controlRate: number;
}

/**
 * Certification level
 */
export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

/**
 * Compliance report
 */
export interface ComplianceReport {
  /** Standard */
  standard: 'WIA-AI-EMBODIMENT';
  /** Test date */
  testDate: string;
  /** Configuration */
  config: EmbodimentConfig;
  /** Target level */
  targetLevel: CertificationLevel;
  /** Test results */
  tests: TestResult[];
  /** Overall pass */
  passed: boolean;
  /** Achieved level */
  achievedLevel?: CertificationLevel;
}

/**
 * Test result
 */
export interface TestResult {
  /** Test name */
  testName: string;
  /** Passed */
  passed: boolean;
  /** Notes */
  notes?: string;
}
