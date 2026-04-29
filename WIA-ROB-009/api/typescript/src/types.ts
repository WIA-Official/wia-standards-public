/**
 * WIA-ROB-009: Robot Standard - Type Definitions
 * @module wia-rob-009
 */

export enum RobotType {
  Industrial = 'industrial', Collaborative = 'collaborative',
  Mobile = 'mobile', Humanoid = 'humanoid', Drone = 'drone',
  Surgical = 'surgical', Agricultural = 'agricultural'
}

export enum OperationMode {
  Manual = 'manual', Autonomous = 'autonomous',
  Teleoperated = 'teleoperated', Collaborative = 'collaborative'
}

export enum SafetyLevel {
  Stop = 'stop', Reduced = 'reduced', Normal = 'normal', Maximum = 'maximum'
}

export interface RobotSpec {
  standard: 'WIA-ROB-009';
  version: string;
  robotId: string;
  type: RobotType;
  manufacturer: string;
  model: string;
  payload: number;
  reach: number;
  degreesOfFreedom: number;
  repeatability: number;
  sensors: string[];
  connectivity: string[];
}

export interface JointState {
  jointId: number;
  position: number;
  velocity: number;
  torque: number;
  temperature: number;
}

export interface RobotState {
  robotId: string;
  timestamp: number;
  mode: OperationMode;
  joints: JointState[];
  endEffectorPose: Pose;
  velocity: number;
  power: number;
  safetyLevel: SafetyLevel;
  errors: string[];
}

export interface Pose {
  position: { x: number; y: number; z: number };
  orientation: { x: number; y: number; z: number; w: number };
}

export interface Trajectory {
  id: string;
  waypoints: { pose: Pose; velocity: number; blendRadius?: number }[];
  duration: number;
  interpolation: 'linear' | 'spline' | 'circular';
}

export interface Task {
  id: string;
  name: string;
  type: string;
  trajectory?: Trajectory;
  parameters: Record<string, unknown>;
  priority: number;
  status: 'pending' | 'running' | 'completed' | 'failed' | 'paused';
  startTime?: number;
  endTime?: number;
}

export interface SensorReading {
  sensorId: string;
  type: string;
  value: number | number[] | Record<string, unknown>;
  unit: string;
  timestamp: number;
  confidence?: number;
}

export interface CollisionZone {
  id: string;
  name: string;
  type: 'box' | 'sphere' | 'cylinder' | 'mesh';
  dimensions: Record<string, number>;
  pose: Pose;
  action: 'stop' | 'slow' | 'avoid';
}

export interface MaintenanceRecord {
  robotId: string;
  date: string;
  type: 'scheduled' | 'preventive' | 'corrective';
  components: string[];
  description: string;
  technician: string;
  nextScheduled?: string;
}

export interface DiagnosticsReport {
  robotId: string;
  timestamp: number;
  overall: 'healthy' | 'warning' | 'critical';
  components: { name: string; status: string; health: number; issues?: string[] }[];
  recommendations: string[];
}

export enum CertificationLevel { Bronze = 'bronze', Silver = 'silver', Gold = 'gold' }

export interface ComplianceReport {
  standard: 'WIA-ROB-009';
  testDate: string;
  robotId: string;
  certificationLevel: CertificationLevel;
  tests: { name: string; passed: boolean }[];
  compliant: boolean;
}

export type RobotEventType = 'state-update' | 'task-complete' | 'collision-warning' | 'error' | 'maintenance-due';
export type EventCallback<T = unknown> = (data: T) => void;
