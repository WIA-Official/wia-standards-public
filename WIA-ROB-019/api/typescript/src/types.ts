/**
 * WIA-ROB-019: Advanced Robot Standard - Type Definitions
 * @module wia-rob-019
 */

export enum AICapability {
  Vision = 'vision', Speech = 'speech', NLP = 'nlp',
  Learning = 'learning', Planning = 'planning', Reasoning = 'reasoning'
}

export enum LearningMode {
  Supervised = 'supervised', Reinforcement = 'reinforcement',
  Imitation = 'imitation', Transfer = 'transfer'
}

export enum AutonomyLevel {
  L0_NoAutonomy = 'l0', L1_Assisted = 'l1', L2_Partial = 'l2',
  L3_Conditional = 'l3', L4_High = 'l4', L5_Full = 'l5'
}

export interface AIModel {
  id: string;
  name: string;
  type: string;
  version: string;
  capability: AICapability;
  accuracy: number;
  latency: number;
  memoryUsage: number;
  lastUpdated: string;
}

export interface RobotConfig {
  standard: 'WIA-ROB-019';
  version: string;
  robotId: string;
  name: string;
  autonomyLevel: AutonomyLevel;
  aiModels: AIModel[];
  capabilities: AICapability[];
  sensors: SensorConfig[];
  actuators: ActuatorConfig[];
}

export interface SensorConfig {
  id: string;
  type: 'camera' | 'lidar' | 'radar' | 'imu' | 'force' | 'tactile' | 'microphone';
  resolution?: { width: number; height: number };
  frameRate?: number;
  range?: { min: number; max: number };
  accuracy: number;
}

export interface ActuatorConfig {
  id: string;
  type: 'motor' | 'servo' | 'hydraulic' | 'pneumatic' | 'speaker';
  maxForce?: number;
  maxSpeed?: number;
  precision: number;
}

export interface Perception {
  timestamp: number;
  objects: DetectedObject[];
  scene: SceneUnderstanding;
  poses: HumanPose[];
}

export interface DetectedObject {
  id: string;
  class: string;
  confidence: number;
  boundingBox: { x: number; y: number; width: number; height: number };
  position3D?: { x: number; y: number; z: number };
  velocity?: { x: number; y: number; z: number };
  attributes?: Record<string, unknown>;
}

export interface SceneUnderstanding {
  segmentation: { class: string; mask: number[][] }[];
  depth: number[][];
  semanticLabels: string[];
  navigableAreas: { x: number; y: number; width: number; height: number }[];
}

export interface HumanPose {
  personId: string;
  keypoints: { name: string; x: number; y: number; confidence: number }[];
  gesture?: string;
  activity?: string;
}

export interface Intent {
  type: string;
  confidence: number;
  entities: { name: string; value: string; type: string }[];
  context?: Record<string, unknown>;
}

export interface Plan {
  id: string;
  goal: string;
  steps: PlanStep[];
  estimatedDuration: number;
  successProbability: number;
}

export interface PlanStep {
  id: string;
  action: string;
  parameters: Record<string, unknown>;
  preconditions: string[];
  effects: string[];
  duration: number;
}

export interface LearningSession {
  id: string;
  mode: LearningMode;
  startTime: number;
  samples: number;
  performance: { metric: string; value: number }[];
  status: 'training' | 'evaluating' | 'completed' | 'failed';
}

export interface Explanation {
  decision: string;
  factors: { factor: string; contribution: number; description: string }[];
  alternatives: { action: string; reason: string }[];
  confidence: number;
}

export enum CertificationLevel { Bronze = 'bronze', Silver = 'silver', Gold = 'gold' }

export interface ComplianceReport {
  standard: 'WIA-ROB-019';
  testDate: string;
  robotId: string;
  certificationLevel: CertificationLevel;
  tests: { name: string; passed: boolean }[];
  compliant: boolean;
}

export type AdvancedRobotEventType = 'perception-update' | 'intent-recognized' | 'plan-generated' | 'learning-complete' | 'anomaly-detected';
export type EventCallback<T = unknown> = (data: T) => void;
