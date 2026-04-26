/**
 * WIA-XR Types
 * Extended Reality (VR/AR/MR) Standard Type Definitions
 * @module @wia/xr/types
 */

// ============================================================================
// Device Types
// ============================================================================

/**
 * XR Device Type
 */
export enum XRDeviceType {
  VR = 'vr',           // Virtual Reality (fully immersive)
  AR = 'ar',           // Augmented Reality (overlay on real world)
  MR = 'mr',           // Mixed Reality (interaction with real world)
  PASSTHROUGH = 'passthrough', // VR with camera passthrough
}

/**
 * XR Session Mode
 */
export enum XRSessionMode {
  INLINE = 'inline',           // Non-immersive (screen-based)
  IMMERSIVE_VR = 'immersive-vr', // Fully immersive VR
  IMMERSIVE_AR = 'immersive-ar', // Immersive AR
}

/**
 * XR Device Information
 */
export interface XRDeviceInfo {
  id: string;
  name: string;
  type: XRDeviceType;
  manufacturer: string;
  capabilities: XRDeviceCapabilities;
  metadata?: Record<string, any>;
}

/**
 * XR Device Capabilities
 */
export interface XRDeviceCapabilities {
  hasPositionalTracking: boolean;
  hasRotationalTracking: boolean;
  hasHandTracking: boolean;
  hasEyeTracking: boolean;
  hasFaceTracking: boolean;
  hasHaptics: boolean;
  hasSpatialAudio: boolean;
  hasDepthSensing: boolean;
  hasPlaneDetection: boolean;
  hasImageTracking: boolean;
  hasLightEstimation: boolean;
  supportedSessionModes: XRSessionMode[];
}

// ============================================================================
// Spatial Tracking Types
// ============================================================================

/**
 * 3D Vector
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Quaternion Rotation
 */
export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

/**
 * 3D Transform (Position + Rotation + Scale)
 */
export interface Transform {
  position: Vector3;
  rotation: Quaternion;
  scale?: Vector3;
}

/**
 * XR Pose (Position + Orientation)
 */
export interface XRPose {
  transform: Transform;
  linearVelocity?: Vector3;
  angularVelocity?: Vector3;
  emulatedPosition?: boolean;
}

/**
 * XR Reference Space Type
 */
export enum XRReferenceSpaceType {
  VIEWER = 'viewer',           // Head-locked space
  LOCAL = 'local',             // Session origin
  LOCAL_FLOOR = 'local-floor', // Floor-level origin
  BOUNDED_FLOOR = 'bounded-floor', // Play area with bounds
  UNBOUNDED = 'unbounded',     // Unlimited tracking
}

/**
 * Spatial Tracking Data
 */
export interface SpatialTrackingData {
  timestamp: number;
  headPose: XRPose;
  leftHandPose?: XRPose;
  rightHandPose?: XRPose;
  eyeGaze?: XRPose;
  referenceSpace: XRReferenceSpaceType;
}

// ============================================================================
// 3D Scene Types
// ============================================================================

/**
 * XR Layer Type
 */
export enum XRLayerType {
  PROJECTION = 'projection', // 3D scene layer
  QUAD = 'quad',             // 2D quad in 3D space
  CYLINDER = 'cylinder',     // Cylindrical display
  EQUIRECT = 'equirect',     // 360-degree image/video
}

/**
 * XR Scene Object
 */
export interface XRSceneObject {
  id: string;
  type: string;
  transform: Transform;
  visible: boolean;
  interactive?: boolean;
  userData?: Record<string, any>;
}

/**
 * XR Mesh
 */
export interface XRMesh extends XRSceneObject {
  type: 'mesh';
  vertices: Float32Array;
  indices: Uint32Array;
  normals?: Float32Array;
  uvs?: Float32Array;
  materialId?: string;
}

/**
 * XR Light
 */
export interface XRLight extends XRSceneObject {
  type: 'light';
  lightType: 'ambient' | 'directional' | 'point' | 'spot';
  color: { r: number; g: number; b: number };
  intensity: number;
}

/**
 * XR Scene
 */
export interface XRScene {
  id: string;
  name: string;
  objects: XRSceneObject[];
  lights: XRLight[];
  background?: string; // Color or texture
  environment?: string; // Environment map
}

// ============================================================================
// Interaction Types
// ============================================================================

/**
 * XR Input Source Type
 */
export enum XRInputSourceType {
  HAND = 'hand',
  CONTROLLER = 'controller',
  GAZE = 'gaze',
  SCREEN = 'screen',
}

/**
 * XR Hand
 */
export enum XRHandedness {
  NONE = 'none',
  LEFT = 'left',
  RIGHT = 'right',
}

/**
 * XR Button State
 */
export interface XRButtonState {
  pressed: boolean;
  touched: boolean;
  value: number; // 0.0 to 1.0
}

/**
 * XR Input Source
 */
export interface XRInputSource {
  id: string;
  type: XRInputSourceType;
  handedness: XRHandedness;
  targetRayPose: XRPose;
  gripPose?: XRPose;
  buttons: Map<string, XRButtonState>;
  axes: number[];
  profiles: string[];
}

/**
 * XR Interaction Event
 */
export interface XRInteractionEvent {
  type: 'select' | 'selectstart' | 'selectend' | 'squeeze' | 'squeezestart' | 'squeezeend';
  inputSource: XRInputSource;
  target?: XRSceneObject;
  timestamp: number;
}

// ============================================================================
// Haptic Feedback Types
// ============================================================================

/**
 * Haptic Pulse
 */
export interface HapticPulse {
  intensity: number; // 0.0 to 1.0
  duration: number;  // milliseconds
}

/**
 * Haptic Pattern
 */
export interface HapticPattern {
  pulses: HapticPulse[];
  repeat?: number;
}

/**
 * Haptic Actuator
 */
export interface HapticActuator {
  type: string;
  canVibrate: boolean;
  pulseHaptic(intensity: number, duration: number): Promise<boolean>;
  playPattern(pattern: HapticPattern): Promise<boolean>;
}

// ============================================================================
// Avatar Types
// ============================================================================

/**
 * Avatar Representation
 */
export interface Avatar {
  id: string;
  userId?: string;
  displayName?: string;
  modelUrl?: string;
  headPose: XRPose;
  leftHandPose?: XRPose;
  rightHandPose?: XRPose;
  expressions?: Record<string, number>;
  metadata?: Record<string, any>;
}

/**
 * Avatar Animation State
 */
export interface AvatarAnimationState {
  avatarId: string;
  animationName: string;
  timestamp: number;
  weight: number; // 0.0 to 1.0
  looping: boolean;
}

// ============================================================================
// Spatial Anchor Types
// ============================================================================

/**
 * Spatial Anchor
 */
export interface SpatialAnchor {
  id: string;
  name?: string;
  transform: Transform;
  persistent: boolean;
  createdAt: number;
  lastUpdatedAt: number;
  metadata?: Record<string, any>;
}

/**
 * Anchor Persistence Storage
 */
export interface AnchorStorage {
  save(anchor: SpatialAnchor): Promise<void>;
  load(id: string): Promise<SpatialAnchor | null>;
  loadAll(): Promise<SpatialAnchor[]>;
  delete(id: string): Promise<boolean>;
}

// ============================================================================
// Rendering Types
// ============================================================================

/**
 * XR Viewport
 */
export interface XRViewport {
  x: number;
  y: number;
  width: number;
  height: number;
}

/**
 * XR View (one eye or screen)
 */
export interface XRView {
  eye: 'left' | 'right' | 'none';
  projectionMatrix: Float32Array; // 4x4 matrix
  transform: Transform;
  viewport?: XRViewport;
}

/**
 * XR Frame Data
 */
export interface XRFrameData {
  timestamp: number;
  views: XRView[];
  tracking: SpatialTrackingData;
  inputSources: XRInputSource[];
}

/**
 * Render Options
 */
export interface RenderOptions {
  antialias?: boolean;
  alpha?: boolean;
  depth?: boolean;
  stencil?: boolean;
  preserveDrawingBuffer?: boolean;
  powerPreference?: 'default' | 'low-power' | 'high-performance';
}

// ============================================================================
// Session Types
// ============================================================================

/**
 * XR Session Configuration
 */
export interface XRSessionConfig {
  mode: XRSessionMode;
  requiredFeatures?: string[];
  optionalFeatures?: string[];
  renderOptions?: RenderOptions;
}

/**
 * XR Session State
 */
export enum XRSessionState {
  INACTIVE = 'inactive',
  STARTING = 'starting',
  ACTIVE = 'active',
  ENDING = 'ending',
}

/**
 * XR Session
 */
export interface XRSession {
  id: string;
  mode: XRSessionMode;
  state: XRSessionState;
  deviceInfo: XRDeviceInfo;
  config: XRSessionConfig;
  startTime: number;
  endTime?: number;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * XR Event Type
 */
export type XREventType =
  | 'sessionstart'
  | 'sessionend'
  | 'select'
  | 'selectstart'
  | 'selectend'
  | 'squeeze'
  | 'squeezestart'
  | 'squeezeend'
  | 'inputsourceschange'
  | 'visibilitychange'
  | 'anchoradded'
  | 'anchorupdated'
  | 'anchorremoved';

/**
 * XR Event Handler
 */
export type XREventHandler<T = any> = (event: T) => void | Promise<void>;

/**
 * XR Event Map
 */
export interface XREventMap {
  sessionstart: XRSession;
  sessionend: XRSession;
  select: XRInteractionEvent;
  selectstart: XRInteractionEvent;
  selectend: XRInteractionEvent;
  squeeze: XRInteractionEvent;
  squeezestart: XRInteractionEvent;
  squeezeend: XRInteractionEvent;
  inputsourceschange: { added: XRInputSource[]; removed: XRInputSource[] };
  visibilitychange: { visible: boolean };
  anchoradded: SpatialAnchor;
  anchorupdated: SpatialAnchor;
  anchorremoved: { id: string };
}
