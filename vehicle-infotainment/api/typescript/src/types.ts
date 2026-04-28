/**
 * WIA-AUTO-010: Vehicle Infotainment - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Technology Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core System Types
// ============================================================================

/**
 * Resolution specification
 */
export interface Resolution {
  width: number;
  height: number;
}

/**
 * Position in 2D space
 */
export interface Position {
  x: number;
  y: number;
}

/**
 * Geographic coordinates
 */
export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude?: number;
  heading?: number;
  speed?: number;
}

// ============================================================================
// Display System Types
// ============================================================================

/**
 * Display type enumeration
 */
export type DisplayType =
  | 'primary'        // Center stack display
  | 'cluster'        // Instrument cluster
  | 'hud'            // Head-up display
  | 'secondary'      // Passenger display
  | 'rear'           // Rear entertainment
  | 'climate';       // Climate control display

/**
 * Display technology
 */
export type DisplayTechnology =
  | 'LCD'
  | 'OLED'
  | 'mini-LED'
  | 'TFT'
  | 'DLP'
  | 'Laser';

/**
 * Display mode
 */
export type DisplayMode = 'day' | 'night' | 'auto';

/**
 * Display configuration
 */
export interface DisplayConfig {
  /** Display identifier */
  id: string;

  /** Display type */
  type: DisplayType;

  /** Physical size in inches */
  size: number;

  /** Display resolution */
  resolution: Resolution;

  /** Display technology */
  technology?: DisplayTechnology;

  /** Touch enabled */
  touchEnabled: boolean;

  /** Multi-touch support (number of simultaneous touches) */
  multiTouch?: number;

  /** Brightness level (0-100) */
  brightness?: number | 'auto';

  /** Refresh rate in Hz */
  refreshRate?: number;

  /** Current display mode */
  mode?: DisplayMode;

  /** Display position */
  position?: Position;
}

/**
 * Display zone configuration
 */
export interface DisplayZone {
  /** Zone identifier */
  id: string;

  /** Zone name */
  name: string;

  /** Zone bounds */
  bounds: {
    x: number;
    y: number;
    width: number;
    height: number;
  };

  /** Current content */
  content?: ContentType[];

  /** Layout mode */
  layout?: 'single' | 'split-screen' | 'grid' | 'overlay';
}

/**
 * Content type for display zones
 */
export type ContentType =
  | 'navigation'
  | 'media'
  | 'phone'
  | 'vehicle-info'
  | 'climate'
  | 'apps'
  | 'speedometer'
  | 'adas'
  | 'camera'
  | 'smartphone-projection';

// ============================================================================
// Audio System Types
// ============================================================================

/**
 * Audio source type
 */
export type AudioSource =
  | 'radio-am'
  | 'radio-fm'
  | 'radio-hd'
  | 'radio-satellite'
  | 'bluetooth'
  | 'usb'
  | 'aux'
  | 'streaming'
  | 'navigation'
  | 'phone'
  | 'notification'
  | 'adas';

/**
 * Audio channel configuration
 */
export interface AudioChannelConfig {
  /** Total number of channels */
  channels: number;

  /** Speaker configuration */
  speakers: {
    frontLeft?: boolean;
    frontRight?: boolean;
    center?: boolean;
    rearLeft?: boolean;
    rearRight?: boolean;
    subwoofer?: boolean;
    ceiling?: number;  // Number of ceiling speakers
  };

  /** Spatial audio support */
  spatialAudio?: boolean;

  /** Dolby Atmos support */
  dolbyAtmos?: boolean;
}

/**
 * Audio equalizer settings
 */
export interface EqualizerSettings {
  /** Bass level (-12 to +12 dB) */
  bass: number;

  /** Mid-range level (-12 to +12 dB) */
  mid: number;

  /** Treble level (-12 to +12 dB) */
  treble: number;

  /** 31-band equalizer values */
  bands?: number[];
}

/**
 * Audio profile
 */
export interface AudioProfile {
  /** Profile name */
  name: string;

  /** Profile identifier */
  id?: string;

  /** Volume level (0-100) */
  volume: number;

  /** Balance (-9 left to +9 right) */
  balance: number;

  /** Fade (-9 rear to +9 front) */
  fade: number;

  /** Equalizer settings */
  equalizer: EqualizerSettings;

  /** Spatial audio enabled */
  spatialAudio?: boolean;

  /** Active noise cancellation */
  anc?: boolean;

  /** Speed-compensated volume */
  speedCompensation?: boolean;
}

/**
 * Media metadata
 */
export interface MediaMetadata {
  /** Track title */
  title: string;

  /** Artist name */
  artist: string;

  /** Album name */
  album?: string;

  /** Track duration in seconds */
  duration: number;

  /** Current position in seconds */
  position: number;

  /** Artwork URL */
  artwork?: string;

  /** Genre */
  genre?: string;

  /** Year */
  year?: number;
}

/**
 * Playback state
 */
export interface PlaybackState {
  /** Current state */
  state: 'playing' | 'paused' | 'stopped' | 'buffering';

  /** Current source */
  source: AudioSource;

  /** Metadata */
  metadata?: MediaMetadata;

  /** Repeat mode */
  repeat: 'off' | 'one' | 'all';

  /** Shuffle enabled */
  shuffle: boolean;

  /** Volume level */
  volume: number;
}

// ============================================================================
// Navigation Types
// ============================================================================

/**
 * Location with address
 */
export interface Location extends GeoLocation {
  /** Street address */
  address?: string;

  /** City */
  city?: string;

  /** State/Province */
  state?: string;

  /** Country */
  country?: string;

  /** Postal code */
  postalCode?: string;

  /** Place name */
  name?: string;
}

/**
 * Route options
 */
export interface RouteOptions {
  /** Avoid tolls */
  avoidTolls?: boolean;

  /** Avoid highways */
  avoidHighways?: boolean;

  /** Avoid ferries */
  avoidFerries?: boolean;

  /** Prefer scenic routes */
  preferScenic?: boolean;

  /** Route optimization */
  optimize?: 'time' | 'distance' | 'eco';
}

/**
 * Route maneuver type
 */
export type ManeuverType =
  | 'straight'
  | 'turn-left'
  | 'turn-right'
  | 'turn-slight-left'
  | 'turn-slight-right'
  | 'turn-sharp-left'
  | 'turn-sharp-right'
  | 'u-turn'
  | 'merge-left'
  | 'merge-right'
  | 'exit-left'
  | 'exit-right'
  | 'roundabout'
  | 'ferry'
  | 'arrive';

/**
 * Route step
 */
export interface RouteStep {
  /** Step instruction */
  instruction: string;

  /** Distance in meters */
  distance: number;

  /** Duration in seconds */
  duration: number;

  /** Maneuver type */
  maneuver: ManeuverType;

  /** Road name */
  roadName?: string;

  /** Lane guidance */
  lanes?: {
    lane: number;
    direction: 'straight' | 'left' | 'right';
    recommended: boolean;
  }[];
}

/**
 * Navigation route
 */
export interface Route {
  /** Route identifier */
  id: string;

  /** Origin location */
  origin: Location;

  /** Destination location */
  destination: Location;

  /** Waypoints */
  waypoints?: Location[];

  /** Route summary */
  summary: {
    /** Total distance in meters */
    distance: number;

    /** Estimated duration in seconds */
    duration: number;

    /** Duration with traffic in seconds */
    trafficDuration?: number;
  };

  /** Route steps */
  steps: RouteStep[];

  /** Polyline encoded path */
  polyline?: string;
}

/**
 * Point of Interest (POI)
 */
export interface POI {
  /** POI identifier */
  id: string;

  /** POI name */
  name: string;

  /** POI category */
  category: string;

  /** Location */
  location: Location;

  /** Distance from current location in meters */
  distance?: number;

  /** Phone number */
  phone?: string;

  /** Website */
  website?: string;

  /** Rating (0-5) */
  rating?: number;

  /** Opening hours */
  hours?: string;
}

// ============================================================================
// Smartphone Integration Types
// ============================================================================

/**
 * Smartphone protocol
 */
export type SmartphoneProtocol = 'carplay' | 'android-auto' | 'mirrorlink';

/**
 * Connection type
 */
export type ConnectionType = 'wired' | 'wireless';

/**
 * Smartphone connection configuration
 */
export interface SmartphoneConnection {
  /** Connection protocol */
  protocol: SmartphoneProtocol;

  /** Connection type */
  type: ConnectionType;

  /** Auto-connect enabled */
  autoConnect?: boolean;

  /** Wireless enabled (if supported) */
  wireless?: boolean;

  /** Device name */
  deviceName?: string;

  /** Connection status */
  status?: 'disconnected' | 'connecting' | 'connected' | 'error';
}

/**
 * CarPlay configuration
 */
export interface CarPlayConfig extends SmartphoneConnection {
  protocol: 'carplay';

  /** Display configuration */
  display?: {
    /** Native resolution */
    resolution: Resolution;

    /** Safe area insets */
    safeArea?: {
      top: number;
      bottom: number;
      left: number;
      right: number;
    };
  };
}

/**
 * Android Auto configuration
 */
export interface AndroidAutoConfig extends SmartphoneConnection {
  protocol: 'android-auto';

  /** Display configuration */
  display?: {
    /** Resolution */
    resolution: Resolution;

    /** Density (DPI) */
    density: number;
  };
}

// ============================================================================
// Voice Assistant Types
// ============================================================================

/**
 * Voice command intent
 */
export type VoiceIntent =
  | 'navigation'
  | 'media'
  | 'phone'
  | 'climate'
  | 'vehicle'
  | 'information'
  | 'unknown';

/**
 * Voice command
 */
export interface VoiceCommand {
  /** Raw command text */
  text: string;

  /** Recognized intent */
  intent: VoiceIntent;

  /** Extracted entities */
  entities?: {
    destination?: string;
    contact?: string;
    temperature?: number;
    media?: string;
    poi?: string;
  };

  /** Confidence score (0-1) */
  confidence: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Voice assistant configuration
 */
export interface VoiceAssistantConfig {
  /** Wake word */
  wakeWord: string;

  /** Language code (e.g., 'en-US') */
  language: string;

  /** Voice type */
  voice?: 'male' | 'female' | 'neutral';

  /** Continuous listening */
  continuousListening?: boolean;

  /** Barge-in enabled */
  bargeIn?: boolean;

  /** Cloud-based processing */
  cloudProcessing?: boolean;
}

// ============================================================================
// Climate Control Types
// ============================================================================

/**
 * Climate zone
 */
export type ClimateZone = 'driver' | 'passenger' | 'rear-left' | 'rear-right';

/**
 * Fan speed level (0-10)
 */
export type FanSpeed = 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10;

/**
 * Air distribution mode
 */
export type AirDistribution = 'face' | 'feet' | 'windshield' | 'face-feet' | 'face-windshield' | 'auto';

/**
 * Climate settings
 */
export interface ClimateSettings {
  /** Target temperature in Fahrenheit or Celsius */
  temperature: number;

  /** Temperature unit */
  unit: 'F' | 'C';

  /** Fan speed */
  fanSpeed: FanSpeed;

  /** Air distribution */
  airDistribution: AirDistribution;

  /** Recirculation mode */
  recirculation: boolean;

  /** AC enabled */
  acEnabled: boolean;

  /** Auto mode */
  autoMode: boolean;

  /** Seat heating level (0-5) */
  seatHeating?: number;

  /** Seat cooling level (0-3) */
  seatCooling?: number;

  /** Steering wheel heating */
  steeringWheelHeating?: boolean;
}

/**
 * Air quality data
 */
export interface AirQuality {
  /** PM2.5 level (μg/m³) */
  pm25: number;

  /** CO2 level (ppm) */
  co2: number;

  /** Air quality index (0-500) */
  aqi: number;

  /** Quality rating */
  rating: 'good' | 'moderate' | 'unhealthy' | 'hazardous';
}

// ============================================================================
// Vehicle Status Types
// ============================================================================

/**
 * Powertrain type
 */
export type PowertrainType = 'gasoline' | 'diesel' | 'hybrid' | 'electric' | 'hydrogen';

/**
 * Door status
 */
export type DoorStatus = 'open' | 'closed' | 'ajar';

/**
 * Powertrain status
 */
export interface PowertrainStatus {
  /** Powertrain type */
  type: PowertrainType;

  /** Battery level (%) for EV/Hybrid */
  batteryLevel?: number;

  /** Fuel level (%) for ICE/Hybrid */
  fuelLevel?: number;

  /** Estimated range in km */
  range: number;

  /** Charging status */
  charging?: boolean;

  /** Charging rate in kW */
  chargingRate?: number;
}

/**
 * Door lock status
 */
export interface DoorLockStatus {
  /** Driver front door */
  driverFront: DoorStatus;

  /** Passenger front door */
  passengerFront: DoorStatus;

  /** Driver rear door */
  driverRear: DoorStatus;

  /** Passenger rear door */
  passengerRear: DoorStatus;

  /** Trunk/boot */
  trunk: DoorStatus;

  /** Hood */
  hood?: DoorStatus;

  /** All doors locked */
  locked: boolean;
}

/**
 * Light status
 */
export interface LightStatus {
  /** Headlights */
  headlights: 'off' | 'parking' | 'low-beam' | 'high-beam' | 'auto';

  /** Fog lights */
  fogLights?: boolean;

  /** Interior lights */
  interior: boolean;

  /** Ambient lighting */
  ambient?: {
    enabled: boolean;
    color?: string;
    brightness?: number;
  };
}

/**
 * Vehicle status
 */
export interface VehicleStatus {
  /** Timestamp */
  timestamp: Date;

  /** Powertrain status */
  powertrain: PowertrainStatus;

  /** Speed in km/h */
  speed: number;

  /** Odometer reading in km */
  odometer: number;

  /** Door status */
  doors: DoorLockStatus;

  /** Light status */
  lights: LightStatus;

  /** Climate settings */
  climate?: ClimateSettings;

  /** Tire pressure (PSI) */
  tirePressure?: {
    frontLeft: number;
    frontRight: number;
    rearLeft: number;
    rearRight: number;
  };
}

// ============================================================================
// Infotainment System Types
// ============================================================================

/**
 * System status
 */
export type SystemStatus =
  | 'initializing'
  | 'ready'
  | 'error'
  | 'updating';

/**
 * Infotainment system configuration
 */
export interface InfotainmentConfig {
  /** Display configuration */
  displays: {
    primary?: DisplayConfig;
    cluster?: DisplayConfig;
    secondary?: DisplayConfig;
    hud?: DisplayConfig;
  };

  /** Audio configuration */
  audio: AudioChannelConfig;

  /** Voice assistant configuration */
  voice?: VoiceAssistantConfig;

  /** Smartphone integration */
  smartphone?: SmartphoneConnection;

  /** System locale */
  locale?: string;

  /** Time zone */
  timezone?: string;
}

/**
 * Infotainment system state
 */
export interface InfotainmentState {
  /** System status */
  status: SystemStatus;

  /** Current display zones */
  displayZones?: Map<string, DisplayZone>;

  /** Playback state */
  playback?: PlaybackState;

  /** Current route */
  route?: Route;

  /** Vehicle status */
  vehicle?: VehicleStatus;

  /** Connected smartphone */
  smartphone?: SmartphoneConnection;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Event type
 */
export type EventType =
  | 'display-touch'
  | 'voice-command'
  | 'route-updated'
  | 'media-changed'
  | 'smartphone-connected'
  | 'smartphone-disconnected'
  | 'climate-changed'
  | 'vehicle-status-updated'
  | 'error';

/**
 * System event
 */
export interface SystemEvent<T = unknown> {
  /** Event type */
  type: EventType;

  /** Event timestamp */
  timestamp: Date;

  /** Event data */
  data: T;

  /** Event source */
  source?: string;
}

/**
 * Event handler
 */
export type EventHandler<T = unknown> = (event: SystemEvent<T>) => void;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Error code
 */
export enum InfotainmentErrorCode {
  // Display errors (1000-1099)
  DISPLAY_NOT_FOUND = 'IVI-1001',
  DISPLAY_INIT_FAILED = 'IVI-1002',

  // Audio errors (1100-1199)
  AUDIO_SOURCE_UNAVAILABLE = 'IVI-1101',
  AUDIO_PLAYBACK_FAILED = 'IVI-1102',

  // Navigation errors (1200-1299)
  NAVIGATION_ROUTE_NOT_FOUND = 'IVI-1201',
  NAVIGATION_GPS_UNAVAILABLE = 'IVI-1202',

  // Smartphone errors (1300-1399)
  SMARTPHONE_CONNECTION_FAILED = 'IVI-1301',
  SMARTPHONE_PROTOCOL_UNSUPPORTED = 'IVI-1302',

  // Voice errors (1400-1499)
  VOICE_RECOGNITION_FAILED = 'IVI-1401',
  VOICE_NETWORK_ERROR = 'IVI-1402',

  // System errors (1500-1599)
  SYSTEM_INIT_FAILED = 'IVI-1501',
  SYSTEM_RESOURCE_UNAVAILABLE = 'IVI-1502',
}

/**
 * Infotainment error
 */
export class InfotainmentError extends Error {
  constructor(
    public code: InfotainmentErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'InfotainmentError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core
  Resolution,
  Position,
  GeoLocation,

  // Display
  DisplayType,
  DisplayTechnology,
  DisplayMode,
  DisplayConfig,
  DisplayZone,
  ContentType,

  // Audio
  AudioSource,
  AudioChannelConfig,
  EqualizerSettings,
  AudioProfile,
  MediaMetadata,
  PlaybackState,

  // Navigation
  Location,
  RouteOptions,
  ManeuverType,
  RouteStep,
  Route,
  POI,

  // Smartphone
  SmartphoneProtocol,
  ConnectionType,
  SmartphoneConnection,
  CarPlayConfig,
  AndroidAutoConfig,

  // Voice
  VoiceIntent,
  VoiceCommand,
  VoiceAssistantConfig,

  // Climate
  ClimateZone,
  FanSpeed,
  AirDistribution,
  ClimateSettings,
  AirQuality,

  // Vehicle
  PowertrainType,
  DoorStatus,
  PowertrainStatus,
  DoorLockStatus,
  LightStatus,
  VehicleStatus,

  // System
  SystemStatus,
  InfotainmentConfig,
  InfotainmentState,

  // Events
  EventType,
  SystemEvent,
  EventHandler,
};

export { InfotainmentErrorCode, InfotainmentError };
