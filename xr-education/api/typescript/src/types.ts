/**
 * WIA-EDU-013: XR Education Standard - Type Definitions
 * @version 1.0.0
 * @description TypeScript types for XR educational experiences
 */

// Core XR Types
export type XRMode = 'immersive-vr' | 'immersive-ar' | 'immersive-mr' | 'inline';
export type XRDeviceType = 'quest3' | 'vision-pro' | 'hololens' | 'mobile' | 'webxr' | 'vive' | 'index';
export type TrackingMode = '3dof' | '6dof';
export type EnvironmentBlending = 'opaque' | 'additive' | 'alpha-blend';

// Vector and Quaternion
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

// XR Session Configuration
export interface XRCapabilities {
  handTracking: boolean;
  eyeTracking: boolean;
  spatialAudio: boolean;
  haptics: boolean;
  voiceCommands: boolean;
  passthrough: boolean;
}

export interface XRSessionConfig {
  mode: XRMode;
  deviceType: XRDeviceType;
  trackingMode: TrackingMode;
  environmentBlending: EnvironmentBlending;
  capabilities: XRCapabilities;
  apiKey?: string;
}

// Virtual Classroom
export interface VirtualClassroom {
  classroomId: string;
  capacity: number;
  dimensions: {
    width: number;
    length: number;
    height: number;
  };
  features: {
    spatialAudio: boolean;
    whiteboard3D: boolean;
    screenSharing: boolean;
    handRaising: boolean;
    breakoutRooms: boolean;
  };
  teacher?: Participant;
  students: Participant[];
  assets: LearningAsset[];
}

// Participant (Student/Teacher)
export type ParticipantRole = 'teacher' | 'student' | 'observer';

export interface Participant {
  userId: string;
  avatar: AvatarConfig;
  position: Vector3;
  rotation: Quaternion;
  role: ParticipantRole;
  permissions: Permission[];
}

export interface AvatarConfig {
  modelUrl?: string;
  color?: string;
  facialTracking?: boolean;
  bodyTracking?: boolean;
  customization?: Record<string, any>;
}

export type Permission = 'view' | 'interact' | 'speak' | 'present' | 'moderate' | 'all';

// Learning Asset
export type AssetType = 'model' | 'audio' | 'video' | 'texture' | 'scene';

export interface LearningAsset {
  assetId: string;
  type: AssetType;
  url: string;
  format: string;
  metadata: {
    subject: string;
    gradeLevel: string;
    learningObjectives: string[];
    interactionType: string[];
  };
}

// Virtual Laboratory
export type LabSubject = 'chemistry' | 'physics' | 'biology' | 'engineering' | 'medical';
export type SafetyLevel = 'low' | 'medium' | 'high' | 'critical';

export interface VirtualLab {
  labId: string;
  subject: LabSubject;
  scenario: string;
  safety: {
    level: SafetyLevel;
    protectiveEquipment: string[];
    hazardWarnings: boolean;
    emergencyProcedures: boolean;
  };
  physics: {
    enabled: boolean;
    gravity: Vector3;
    collisionDetection: boolean;
    fluidDynamics: boolean;
    chemicalReactions: boolean;
  };
  equipment: LabEquipment[];
  materials: Material[];
  interactions: Interaction[];
}

export interface LabEquipment {
  equipmentId: string;
  name: string;
  type: string;
  position: Vector3;
  interactable: boolean;
}

export interface Material {
  materialId: string;
  name: string;
  properties: Record<string, any>;
  concentration?: number;
  volume?: number;
  hazardous: boolean;
}

export interface Interaction {
  interactionId: string;
  type: 'grab' | 'pour' | 'measure' | 'observe' | 'combine';
  objects: string[];
  result?: string;
}

// Accessibility
export type ColorblindMode = 'deuteranopia' | 'protanopia' | 'tritanopia' | 'none';
export type FontSize = 'small' | 'medium' | 'large' | 'xlarge';

export interface XRAccessibility {
  visual: {
    colorblindMode: ColorblindMode;
    highContrast: boolean;
    textToSpeech: boolean;
    subtitles: boolean;
    fontSize: FontSize;
  };
  motor: {
    seatedMode: boolean;
    reduceReach: boolean;
    snapTurning: boolean;
    teleportMovement: boolean;
    oneHandedMode: boolean;
  };
  comfort: {
    vignette: number;
    reducedMotion: boolean;
    fieldOfViewReduction: number;
  };
  cognitive: {
    simplifiedUI: boolean;
    guidedMode: boolean;
    pauseAnytime: boolean;
  };
}

// XR Analytics
export interface XRAnalytics {
  sessionId: string;
  studentId: string;
  duration: number;
  engagement: {
    gazeTracking: HeatmapData;
    interactionCount: number;
    objectsTouched: string[];
    timeInExperience: number;
  };
  performance: {
    taskCompletion: number;
    accuracy: number;
    efficiency: number;
    mistakeCount: number;
    hintsUsed: number;
  };
  spatial: {
    navigationPath: Vector3[];
    objectManipulation: ManipulationEvent[];
    spatialMemory: number;
  };
  comfort: {
    motionSicknessIndicators: number;
    pauseCount: number;
    exitReason?: string;
  };
}

export interface HeatmapData {
  points: Array<{
    position: Vector3;
    duration: number;
    timestamp: number;
  }>;
}

export interface ManipulationEvent {
  objectId: string;
  action: 'grab' | 'rotate' | 'scale' | 'throw';
  timestamp: number;
  success: boolean;
}

// Input Systems
export interface XRInput {
  controllers: {
    left?: XRController;
    right?: XRController;
  };
  handTracking: {
    enabled: boolean;
    precision: 'low' | 'medium' | 'high';
    gestures: Gesture[];
  };
  eyeTracking: {
    enabled: boolean;
    gazePoint: Vector3;
    focusedObject?: string;
  };
  voiceCommands: {
    enabled: boolean;
    language: string;
    commands: VoiceCommand[];
  };
}

export interface XRController {
  position: Vector3;
  rotation: Quaternion;
  buttons: Record<string, boolean>;
  axes: number[];
  hapticFeedback?: boolean;
}

export interface Gesture {
  name: string;
  handedness: 'left' | 'right' | 'both';
  action: string;
  confidence: number;
}

export interface VoiceCommand {
  phrase: string;
  action: string;
  parameters?: Record<string, any>;
}

// Multiplayer
export interface MultiplayerSession {
  sessionId: string;
  host: string;
  participants: Participant[];
  maxCapacity: number;
  networking: {
    protocol: 'webrtc' | 'websocket' | 'photon' | 'custom';
    latency: number;
    bandwidth: number;
  };
  sync: {
    positionUpdate: number;
    physicsSync: boolean;
    stateSync: boolean;
    voiceChat: boolean;
  };
}

// Events
export interface XREvent {
  type: string;
  timestamp: number;
  data: any;
}

export interface InteractionEvent extends XREvent {
  type: 'interaction';
  data: {
    userId: string;
    objectId: string;
    action: string;
    position: Vector3;
  };
}

// Configuration Options
export interface XREducationOptions {
  apiKey?: string;
  mode: XRMode;
  device?: XRDeviceType;
  baseUrl?: string;
  accessibility?: Partial<XRAccessibility>;
  analytics?: boolean;
}

// API Response Types
export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
  };
}

export interface SessionResponse {
  sessionId: string;
  status: 'active' | 'paused' | 'ended';
  startTime: Date;
  participants: number;
}
