# WIA Eye Gaze Standard - API Reference

**Version**: 1.0.0
**弘益人間** - 널리 인간을 이롭게

---

## Table of Contents

1. [Core Types](#1-core-types)
2. [Tracker API](#2-tracker-api)
3. [Dwell Controller](#3-dwell-controller)
4. [Communication Protocol](#4-communication-protocol)
5. [Integration APIs](#5-integration-apis)

---

## 1. Core Types

### GazePoint

Primary data structure for gaze data.

```typescript
interface GazePoint {
  /** Unix timestamp (ms) */
  timestamp: number;

  /** Normalized X coordinate (0.0 - 1.0) */
  x: number;

  /** Normalized Y coordinate (0.0 - 1.0) */
  y: number;

  /** Data validity flag */
  valid: boolean;

  /** Confidence score (0.0 - 1.0) */
  confidence: number;

  /** Left eye data */
  leftEye?: EyeData;

  /** Right eye data */
  rightEye?: EyeData;

  /** Current fixation */
  fixation?: FixationData;

  /** Current saccade */
  saccade?: SaccadeData;

  /** Fixation ID for tracking */
  fixationId?: number;

  /** Device timestamp (if different from system) */
  deviceTimestamp?: number;
}
```

### EyeData

Per-eye tracking data.

```typescript
interface EyeData {
  /** Eye identifier */
  eye: 'left' | 'right';

  /** Gaze point for this eye */
  gaze: Vector2D;

  /** Eye position in 3D space (mm) */
  position?: Vector3D;

  /** Gaze direction vector */
  direction?: Vector3D;

  /** Pupil diameter (mm) */
  pupilDiameter?: number;

  /** Eye openness (0.0 - 1.0) */
  eyeOpenness?: number;

  /** Data validity */
  valid: boolean;
}
```

### GazeEvent

Event notification.

```typescript
interface GazeEvent {
  /** Event type */
  eventType: GazeEventType;

  /** Unique event ID */
  eventId: string;

  /** Event timestamp */
  timestamp: number;

  /** Event duration (ms) */
  duration?: number;

  /** Event position */
  position?: Vector2D;

  /** Associated target */
  target?: GazeTarget;
}

type GazeEventType =
  | 'FIXATION_START'
  | 'FIXATION_END'
  | 'SACCADE_START'
  | 'SACCADE_END'
  | 'BLINK'
  | 'DWELL_START'
  | 'DWELL_PROGRESS'
  | 'DWELL_COMPLETE'
  | 'DWELL_CANCEL'
  | 'GAZE_ENTER'
  | 'GAZE_EXIT'
  | 'TRACKING_START'
  | 'TRACKING_STOP'
  | 'TRACKING_LOST'
  | 'TRACKING_RECOVERED';
```

### GazeTarget

Interactive target for gaze selection.

```typescript
interface GazeTarget {
  /** Target element ID */
  elementId: string;

  /** Bounding box */
  bounds: {
    x: number;
    y: number;
    width: number;
    height: number;
  };

  /** Human-readable label */
  label?: string;

  /** Custom dwell time (ms) */
  dwellTime?: number;

  /** Custom metadata */
  metadata?: Record<string, unknown>;
}
```

### Vector Types

```typescript
interface Vector2D {
  x: number;
  y: number;
}

interface Vector3D {
  x: number;
  y: number;
  z: number;
}
```

---

## 2. Tracker API

### IEyeTracker Interface

```typescript
interface IEyeTracker {
  /** Connect to eye tracker */
  connect(): Promise<void>;

  /** Disconnect from eye tracker */
  disconnect(): Promise<void>;

  /** Start calibration */
  startCalibration(points?: CalibrationPoint[]): Promise<CalibrationResult>;

  /** Subscribe to gaze data */
  subscribe(callback: (gaze: GazePoint) => void): Subscription;

  /** Start tracking */
  startTracking(): void;

  /** Stop tracking */
  stopTracking(): void;

  /** Register event handler */
  on<T extends GazeEvent>(event: GazeEventType, handler: (event: T) => void): void;

  /** Get device capabilities */
  getCapabilities(): EyeTrackerCapabilities;

  /** Get current status */
  getStatus(): TrackerStatus;
}
```

### WiaEyeTracker

Main tracker implementation.

```typescript
class WiaEyeTracker implements IEyeTracker {
  constructor(adapter: EyeTrackerAdapter);

  // IEyeTracker implementation
  async connect(): Promise<void>;
  async disconnect(): Promise<void>;
  async startCalibration(points?: CalibrationPoint[]): Promise<CalibrationResult>;
  subscribe(callback: GazeCallback): Subscription;
  startTracking(): void;
  stopTracking(): void;
  on<T extends GazeEvent>(event: GazeEventType, handler: EventCallback<T>): void;
  getCapabilities(): EyeTrackerCapabilities;
  getStatus(): TrackerStatus;

  // Additional methods
  setSmoothingFactor(factor: number): void;
  setFixationThreshold(velocity: number, duration: number): void;
}
```

### Subscription

```typescript
interface Subscription {
  /** Unsubscribe from gaze data */
  unsubscribe(): void;
}
```

### TrackerStatus

```typescript
type TrackerStatus =
  | 'disconnected'
  | 'connecting'
  | 'connected'
  | 'calibrating'
  | 'tracking'
  | 'error';
```

### EyeTrackerCapabilities

```typescript
interface EyeTrackerCapabilities {
  /** Device vendor */
  vendor: string;

  /** Device model */
  model: string;

  /** Maximum sampling rate (Hz) */
  maxSamplingRate: number;

  /** Supports binocular tracking */
  binocular: boolean;

  /** Supports 3D gaze */
  gaze3D: boolean;

  /** Supports pupil size */
  pupilSize: boolean;

  /** Supports eye openness */
  eyeOpenness: boolean;

  /** Supports head tracking */
  headTracking: boolean;

  /** Supported calibration point counts */
  calibrationPoints: number[];
}
```

---

## 3. Dwell Controller

### DwellController

```typescript
class DwellController {
  constructor(tracker?: IEyeTracker);

  /** Set dwell time threshold (ms) */
  setDwellTime(ms: number): void;

  /** Get current dwell time */
  getDwellTime(): number;

  /** Register gaze target */
  registerTarget(target: GazeTarget): void;

  /** Unregister target */
  unregisterTarget(elementId: string): void;

  /** Clear all targets */
  clearTargets(): void;

  /** Register dwell start callback */
  onDwellStart(handler: DwellEventHandler): void;

  /** Register dwell progress callback */
  onDwellProgress(handler: DwellEventHandler): void;

  /** Register dwell complete callback */
  onDwellComplete(handler: DwellEventHandler): void;

  /** Register dwell cancel callback */
  onDwellCancel(handler: DwellEventHandler): void;

  /** Start dwell detection */
  start(): void;

  /** Stop dwell detection */
  stop(): void;

  /** Process gaze point manually */
  processGaze(point: GazePoint): void;

  /** Get current dwell state */
  getCurrentDwell(): DwellState | null;
}
```

### DwellState

```typescript
interface DwellState {
  /** Target being dwelled on */
  target: GazeTarget;

  /** Dwell start time */
  startTime: number;

  /** Current duration (ms) */
  duration: number;

  /** Progress (0.0 - 1.0) */
  progress: number;
}
```

### DwellEvent

```typescript
interface DwellEvent {
  /** Event type */
  type: 'start' | 'progress' | 'complete' | 'cancel';

  /** Target */
  target: GazeTarget;

  /** Duration (ms) */
  duration: number;

  /** Progress (0.0 - 1.0) */
  progress: number;

  /** Timestamp */
  timestamp: number;
}

type DwellEventHandler = (event: DwellEvent) => void;
```

---

## 4. Communication Protocol

### WebSocket Messages

#### Server → Client

**GazeData**
```typescript
interface GazeDataMessage {
  type: 'gaze_data';
  timestamp: number;
  sequence: number;
  frequency: number;
  points: GazePointJson[];
}
```

**GazeEvent**
```typescript
interface GazeEventMessage {
  type: 'gaze_event';
  timestamp: number;
  event: {
    eventType: GazeEventType;
    eventId: string;
    duration?: number;
    position?: Vector2D;
    target?: GazeTarget;
  };
}
```

**Status**
```typescript
interface StatusMessage {
  type: 'status';
  timestamp: number;
  status: {
    connected: boolean;
    tracking: boolean;
    calibrated: boolean;
    state: TrackerStatus;
    device?: DeviceInfo;
    metrics?: StreamMetrics;
  };
}
```

#### Client → Server

**Control**
```typescript
interface ControlMessage {
  type: 'control';
  timestamp: number;
  requestId?: string;
  action: ControlAction;
  params?: Record<string, unknown>;
}

type ControlAction =
  | 'start'
  | 'stop'
  | 'pause'
  | 'resume'
  | 'calibrate'
  | 'set_frequency'
  | 'set_format'
  | 'subscribe'
  | 'unsubscribe';
```

### Binary Protocol

#### GazePointBinary (21 bytes)

| Offset | Size | Type | Field |
|--------|------|------|-------|
| 0 | 2 | u16 | timestamp_offset |
| 2 | 4 | f32 | x |
| 6 | 4 | f32 | y |
| 10 | 2 | u16 | confidence (×65535) |
| 12 | 1 | u8 | flags |
| 13 | 2 | u16 | left_pupil (×100) |
| 15 | 2 | u16 | right_pupil (×100) |
| 17 | 1 | u8 | left_openness (×255) |
| 18 | 1 | u8 | right_openness (×255) |
| 19 | 2 | u16 | fixation_id |

#### Flags Byte

| Bit | Name |
|-----|------|
| 0 | valid |
| 1 | left_valid |
| 2 | right_valid |
| 3 | is_fixation |
| 4 | is_saccade |
| 5 | is_blink |
| 6 | has_3d |
| 7 | reserved |

---

## 5. Integration APIs

### GazeToAAC

```typescript
class GazeToAAC {
  constructor(options?: {
    dwellTime?: number;
    blinkSelectionEnabled?: boolean;
    wordPrediction?: boolean;
  });

  /** Load AAC grid */
  loadGrid(grid: AACGrid): void;

  /** Set dwell controller */
  setDwellController(controller: DwellController): void;

  /** Select symbol by ID */
  selectSymbol(symbolId: string): void;

  /** Start gaze scanning */
  startGazeScan(grid?: AACGrid): void;

  /** Stop scanning */
  stopGazeScan(): void;

  /** Convert targets to text */
  gazeToText(targets: GazeTarget[]): string;

  /** Get message buffer */
  getMessageBuffer(): MessageBuffer;

  /** Speak buffer contents */
  speakBuffer(voice?: VoiceSettings): void;

  /** Event handlers */
  onSelection(callback: SelectionCallback): void;
  onScan(callback: ScanCallback): void;
  onMessage(callback: MessageCallback): void;

  /** Cleanup */
  dispose(): void;
}
```

### GazeToBCI

```typescript
class GazeToBCI {
  constructor(options?: {
    useP300?: boolean;
    useSsvep?: boolean;
    useMotorImagery?: boolean;
    gazeWeight?: number;
    confidenceThreshold?: number;
  });

  /** Combine gaze and EEG data */
  combineWithEEG(gazeData: GazePoint, eegData: BCIData): MultimodalIntent;

  /** Set calibration target */
  setBCICalibrationTarget(point: GazePoint): void;

  /** Start calibration */
  startCalibration(): void;

  /** Complete calibration */
  completeCalibration(): BCICalibrationResult;

  /** Register intent callback */
  onIntent(callback: (intent: MultimodalIntent) => void): void;

  /** Update configuration */
  configure(options: Partial<BCIConfig>): void;

  /** Cleanup */
  dispose(): void;
}
```

### GazeToCI

```typescript
class GazeToCI {
  constructor(options?: CocktailPartyConfig);

  /** Set CI device capabilities */
  setCapabilities(capabilities: CICapabilities): void;

  /** Register audio source */
  registerAudioSource(source: AudioSource): void;

  /** Update audio source */
  updateAudioSource(sourceId: string, updates: Partial<AudioSource>): void;

  /** Remove audio source */
  removeAudioSource(sourceId: string): void;

  /** Enhance gazed audio source */
  enhanceGazedAudioSource(gazeDirection: Vector3D): AudioEnhancement | null;

  /** Get current enhancement */
  getCurrentEnhancement(): AudioEnhancement | null;

  /** Get all audio sources */
  getAudioSources(): AudioSource[];

  /** Register enhancement callback */
  onEnhancement(callback: (event: EnhancementEvent) => void): void;

  /** Update configuration */
  configure(options: Partial<CocktailPartyConfig>): void;

  /** Cleanup */
  dispose(): void;
}
```

### GamingIntegration

```typescript
class GamingIntegration {
  constructor(screenWidth?: number, screenHeight?: number);

  /** Process gaze point */
  processGaze(point: GazePoint): void;

  /** Convert gaze to mouse event */
  gazeToMouse(point: GazePoint): EmulatedMouseEvent;

  /** Convert gaze to analog stick */
  gazeToAnalogStick(point: GazePoint): AnalogStickValue;

  /** Apply aim assist */
  aimAssist(gazeDirection: Vector3D): void;

  /** Register aim target */
  registerAimTarget(target: AimTarget): void;

  /** Remove aim target */
  removeAimTarget(targetId: string): void;

  /** Configure aim assist */
  configureAimAssist(config: Partial<AimAssistConfig>): void;

  /** Initialize XR integration */
  initializeXR(platform: 'openXR' | 'steamVR' | 'metaQuest'): boolean;

  /** Event handlers */
  onMouseEvent(callback: (event: EmulatedMouseEvent) => void): void;
  onGamepadState(callback: (state: GamepadState) => void): void;

  /** Cleanup */
  dispose(): void;
}
```

### SmartHomeIntegration

```typescript
class SmartHomeIntegration {
  constructor(config?: Partial<SmartHomeConfig>);

  /** Detect device from gaze direction */
  detectGazedDevice(direction: Vector3D): SmartDevice | null;

  /** Control device */
  controlDevice(device: SmartDevice, action: DeviceAction): boolean;

  /** Register device */
  registerDevice(device: SmartDevice): void;

  /** Get device by ID */
  getDevice(id: string): SmartDevice | undefined;

  /** Get all devices */
  getAllDevices(): SmartDevice[];

  /** Add room */
  addRoom(room: Room): void;

  /** Configure Home Assistant */
  configureHomeAssistant(url: string, token: string): void;

  /** Send webhook to Home Assistant */
  homeAssistantWebhook(event: GazeEvent): void;

  /** Enable Matter support */
  enableMatter(): void;

  /** Register event callback */
  onEvent(callback: (event: SmartHomeEvent) => void): void;

  /** Cleanup */
  dispose(): void;
}
```

---

## Error Codes

| Code | Name | Description |
|------|------|-------------|
| E001 | DEVICE_NOT_FOUND | Eye tracker not detected |
| E002 | CONNECTION_FAILED | Failed to connect to device |
| E003 | CALIBRATION_FAILED | Calibration did not complete |
| E004 | TRACKING_ERROR | Error during tracking |
| E005 | INVALID_DATA | Received invalid gaze data |
| E006 | TIMEOUT | Operation timed out |
| E007 | NOT_SUPPORTED | Feature not supported |
| E008 | PERMISSION_DENIED | Accessibility permission denied |

---

## Constants

```typescript
// Sampling rates
const SAMPLING_RATE_30HZ = 30;
const SAMPLING_RATE_60HZ = 60;
const SAMPLING_RATE_120HZ = 120;

// Default thresholds
const DEFAULT_DWELL_TIME = 800;  // ms
const DEFAULT_FIXATION_VELOCITY = 30;  // deg/s
const DEFAULT_FIXATION_DURATION = 100;  // ms

// Protocol
const WEBSOCKET_PORT = 8765;
const IPC_SOCKET_PATH = '/tmp/wia-eye-gaze.sock';
const BINARY_MAGIC = [0x57, 0x49];  // "WI"
const PROTOCOL_VERSION = 0x01;
```

---

**WIA Eye Gaze Standard API Reference v1.0.0**

**弘益人間** - 널리 인간을 이롭게
