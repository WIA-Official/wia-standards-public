/**
 * WIA-EDU-021: Immersive Media TypeScript Type Definitions
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export type Vector3 = [number, number, number]; // [x, y, z]

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Transform {
  position: Vector3;
  rotation: Quaternion;
  scale: Vector3;
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface ImmersiveMediaConfig {
  apiKey: string;
  region?: string;
  quality?: 'low' | 'medium' | 'high' | 'auto';
  logging?: boolean;
}

// ============================================================================
// 360° Video Types
// ============================================================================

export type ProjectionType = 'equirectangular' | 'cubemap' | 'fisheye';
export type StereoMode = 'mono' | 'top-bottom' | 'left-right';

export interface Video360Config {
  source: string;
  projection: ProjectionType;
  stereoMode: StereoMode;
  autoplay?: boolean;
  controls?: boolean;
  fov?: number; // Field of view 60-120
  initialView?: { yaw: number; pitch: number };
}

export interface HotspotConfig {
  position: { yaw: number; pitch: number }; // Spherical coordinates
  time?: number; // Time in video (seconds)
  content: string;
  type?: 'info' | 'quiz' | 'link' | 'custom';
  data?: any;
  onClick?: () => void;
  autoShow?: boolean;
}

export interface Hotspot {
  id: string;
  config: HotspotConfig;
  show(): void;
  hide(): void;
  remove(): void;
}

export interface Video360 {
  readonly duration: number;
  readonly currentTime: number;
  readonly paused: boolean;

  play(): Promise<void>;
  pause(): void;
  seek(time: number): void;
  setQuality(quality: '4k' | '5.7k' | '8k'): void;
  addHotspot(config: HotspotConfig): Hotspot;
  removeHotspot(id: string): void;

  on(event: string, handler: (...args: any[]) => void): void;
  off(event: string, handler: (...args: any[]) => void): void;
}

// ============================================================================
// 3D Model Types
// ============================================================================

export interface Model3DConfig {
  source: string; // .glb, .gltf, .fbx, .obj, .usdz
  position?: Vector3;
  scale?: Vector3 | number;
  rotation?: Vector3;
  animations?: string[];
  lighting?: 'realistic' | 'stylized' | 'none';
  castShadows?: boolean;
  receiveShadows?: boolean;
}

export interface AnnotationConfig {
  position: Vector3;
  label: string;
  description?: string;
  autoShow?: boolean;
  showOnHover?: boolean;
}

export interface Annotation {
  id: string;
  config: AnnotationConfig;
  show(): void;
  hide(): void;
  remove(): void;
}

export interface MaterialConfig {
  type: 'pbr' | 'standard' | 'custom';
  baseColor?: [number, number, number, number]; // RGBA
  metallic?: number; // 0.0-1.0
  roughness?: number; // 0.0-1.0
  emissive?: [number, number, number]; // RGB
  textures?: {
    baseColor?: string;
    normal?: string;
    metallicRoughness?: string;
    occlusion?: string;
    emissive?: string;
  };
}

export interface Model3D {
  readonly loaded: boolean;
  readonly boundingBox: { min: Vector3; max: Vector3 };

  load(): Promise<void>;
  playAnimation(name: string, loop?: boolean): void;
  stopAnimation(name?: string): void;
  addAnnotation(config: AnnotationConfig): Annotation;
  removeAnnotation(id: string): void;
  setMaterial(material: MaterialConfig): void;
  rotate(axis: Vector3, angle: number): void;
  scale(factor: number | Vector3): void;
  moveTo(position: Vector3, duration?: number): void;

  on(event: string, handler: (...args: any[]) => void): void;
  off(event: string, handler: (...args: any[]) => void): void;
}

// ============================================================================
// Spatial Audio Types
// ============================================================================

export type EnvironmentType = 'outdoor' | 'classroom' | 'auditorium' | 'cave' | 'custom';
export type AttenuationType = 'linear' | 'inverse' | 'exponential';

export interface SpatialAudioConfig {
  environment: EnvironmentType;
  reverb?: number; // 0.0-1.0
  attenuation?: AttenuationType;
}

export interface AudioSourceConfig {
  id: string;
  position: Vector3;
  audio: string | AudioBuffer;
  volume?: number; // 0.0-1.0
  loop?: boolean;
  spatial?: boolean;
  maxDistance?: number;
  refDistance?: number;
}

export interface AudioSource {
  readonly id: string;
  readonly playing: boolean;

  play(): Promise<void>;
  pause(): void;
  stop(): void;
  setVolume(volume: number): void;
  setPosition(position: Vector3): void;
  remove(): void;
}

export interface SpatialAudio {
  addSource(config: AudioSourceConfig): AudioSource;
  removeSource(id: string): void;
  setListenerPosition(position: Vector3): void;
  setListenerOrientation(forward: Vector3, up: Vector3): void;
  updateSourcePosition(id: string, position: Vector3): void;
  setEnvironment(environment: EnvironmentType): void;

  on(event: string, handler: (...args: any[]) => void): void;
  off(event: string, handler: (...args: any[]) => void): void;
}

// ============================================================================
// Haptic Feedback Types
// ============================================================================

export type HapticType = 'vibration' | 'impact' | 'texture' | 'resistance';
export type HapticPattern = 'constant' | 'pulse' | 'ramp' | 'wave';

export interface HapticConfig {
  device?: 'controller' | 'glove' | 'vest' | 'auto';
  hand?: 'left' | 'right' | 'both';
}

export interface HapticEffect {
  type: HapticType;
  intensity: number; // 0.0-1.0
  duration: number; // milliseconds
  pattern?: HapticPattern;
}

export interface HapticPatternItem {
  intensity: number;
  duration: number;
}

export interface Haptic {
  trigger(effect: HapticEffect): Promise<void>;
  playPattern(pattern: HapticPatternItem[]): Promise<void>;
  stop(): void;

  on(event: string, handler: (...args: any[]) => void): void;
  off(event: string, handler: (...args: any[]) => void): void;
}

// ============================================================================
// XR Device Types
// ============================================================================

export type XRSessionMode = 'immersive-vr' | 'immersive-ar' | 'inline';

export interface DeviceCapabilities {
  vr: boolean;
  ar: boolean;
  handTracking: boolean;
  eyeTracking: boolean;
  passthrough: boolean;
  spatial6DOF: boolean;
}

export interface XRController {
  readonly handedness: 'left' | 'right' | 'none';
  readonly connected: boolean;
  readonly buttons: GamepadButton[];
  readonly axes: number[];

  vibrate(intensity: number, duration: number): Promise<void>;

  on(event: string, handler: (...args: any[]) => void): void;
  off(event: string, handler: (...args: any[]) => void): void;
}

export interface XRSession {
  readonly mode: XRSessionMode;
  readonly active: boolean;

  addObject(object: Model3D): void;
  removeObject(object: Model3D): void;
  enableHandTracking(): Promise<void>;
  enablePassthrough(): Promise<void>;
  end(): Promise<void>;

  on(event: string, handler: (...args: any[]) => void): void;
  off(event: string, handler: (...args: any[]) => void): void;
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface AnalyticsEvent {
  type: 'interaction' | 'navigation' | 'completion' | 'error' | 'custom';
  target?: string;
  action?: string;
  value?: any;
  timestamp: number;
}

export interface ProgressConfig {
  lessonId: string;
  section?: string;
  completionPercentage: number;
  score?: number;
  timeSpent?: number;
}

export interface EngagementMetrics {
  totalTime: number;
  interactions: number;
  completionRate: number;
  averageScore: number;
  commonPaths: string[];
}

export interface Analytics {
  trackEvent(event: AnalyticsEvent): void;
  trackProgress(config: ProgressConfig): void;
  getEngagementMetrics(): Promise<EngagementMetrics>;

  on(event: string, handler: (...args: any[]) => void): void;
  off(event: string, handler: (...args: any[]) => void): void;
}

// ============================================================================
// Error Types
// ============================================================================

export class ImmersiveMediaError extends Error {
  constructor(
    public code: string,
    message: string,
    public details?: any
  ) {
    super(message);
    this.name = 'ImmersiveMediaError';
  }
}

export type ErrorCode =
  | 'INVALID_CONFIG'
  | 'RESOURCE_NOT_FOUND'
  | 'UNSUPPORTED_FORMAT'
  | 'DEVICE_NOT_SUPPORTED'
  | 'NETWORK_ERROR'
  | 'PLAYBACK_ERROR'
  | 'PERMISSION_DENIED'
  | 'UNKNOWN_ERROR';
