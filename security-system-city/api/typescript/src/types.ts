/**
 * WIA-CITY-014: Security System Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

/**
 * ISO 8601 timestamp string
 */
export type Timestamp = string;

/**
 * Geographic coordinates
 */
export interface Coordinates {
  latitude: number;
  longitude: number;
  altitude?: number;
}

/**
 * Location information
 */
export interface Location {
  building?: string;
  floor?: string;
  room?: string;
  zone?: string;
  area?: string;
  coordinates?: {
    x: number;
    y: number;
    z?: number;
  };
  gps?: Coordinates;
}

// ============================================================================
// CCTV Camera Types
// ============================================================================

/**
 * Camera types
 */
export enum CameraType {
  FIXED = 'fixed',
  PTZ = 'ptz',
  DOME = 'dome',
  BULLET = 'bullet',
  THERMAL = 'thermal',
  FISHEYE = 'fisheye',
  PANORAMIC = 'panoramic',
}

/**
 * Camera resolution
 */
export type CameraResolution =
  | '640x480'      // VGA
  | '1280x720'     // 720p
  | '1920x1080'    // 1080p
  | '2048x1080'    // 2K
  | '3840x2160'    // 4K
  | '7680x4320';   // 8K

/**
 * CCTV Camera
 */
export interface CCTVCamera {
  // Identifiers
  camera_id: string;
  name: string;
  description?: string;

  // Location
  location: Location;

  // Camera specifications
  type: CameraType;
  resolution: CameraResolution;
  fps: number;

  lens?: {
    focal_length: string;         // "3.6mm", "2.8-12mm"
    field_of_view: number;        // degrees
    aperture: string;             // "f/1.6"
  };

  // Features
  features: {
    night_vision: boolean;
    ir_distance?: number;         // meters
    wdr: boolean;                 // Wide Dynamic Range
    defog: boolean;
    audio: boolean;
    microphone?: boolean;
    speaker?: boolean;
  };

  // PTZ capabilities (if applicable)
  ptz?: {
    pan: {
      range: number;              // degrees (360)
      speed: number;              // degrees/second
    };
    tilt: {
      range: number;              // degrees (180)
      speed: number;              // degrees/second
    };
    zoom: {
      optical: string;            // "30x"
      digital: string;            // "16x"
    };
    presets: number;              // Maximum presets
    tours: number;                // Maximum tours
    auto_tracking: boolean;
  };

  // Thermal camera specs (if applicable)
  thermal?: {
    detector: string;
    spectral_range: string;       // "8-14μm"
    temperature_range: {
      min: number;                // °C
      max: number;                // °C
    };
    sensitivity: string;          // "<50mK"
    fire_detection: boolean;
    temperature_alarm: boolean;
  };

  // Network and communication
  network: {
    ip_address: string;
    mac_address: string;
    protocol: 'onvif' | 'rtsp' | 'http' | 'proprietary';
    port: number;
    nvr_id?: string;
  };

  // Status
  status: {
    online: boolean;
    recording: boolean;
    motion_detected: boolean;
    last_heartbeat: Timestamp;
  };

  // Recording settings
  recording: {
    mode: 'continuous' | 'motion' | 'schedule' | 'event';
    quality: 'low' | 'medium' | 'high' | 'ultra';
    retention_days: number;
    storage_location: string;     // NVR ID or cloud
  };

  // Installation info
  installation: {
    install_date: Timestamp;
    installer: string;
    warranty_until?: Timestamp;
    last_maintenance?: Timestamp;
    next_maintenance?: Timestamp;
  };
}

/**
 * Camera zone (group of cameras covering an area)
 */
export interface CameraZone {
  zone_id: string;
  zone_name: string;
  cameras: string[];              // Camera IDs
  priority: 'high' | 'medium' | 'low';
  recording_mode: 'continuous' | 'motion' | 'schedule' | 'event';
  retention_days: number;
  analytics_enabled: boolean;
}

/**
 * Video stream
 */
export interface VideoStream {
  camera_id: string;
  stream_url: string;
  protocol: 'rtsp' | 'hls' | 'webrtc';
  resolution: CameraResolution;
  fps: number;
  bitrate: number;                // kbps
  codec: 'h264' | 'h265' | 'mjpeg';
}

/**
 * NVR (Network Video Recorder)
 */
export interface NVR {
  nvr_id: string;
  name: string;
  model: string;

  specifications: {
    channels: number;
    max_resolution: CameraResolution;
    recording_bandwidth: number;  // Mbps
    playback_channels: number;
  };

  storage: {
    bays: number;
    total_capacity: number;       // TB
    used_capacity: number;        // TB
    raid_type?: 'RAID0' | 'RAID1' | 'RAID5' | 'RAID6' | 'RAID10';
  };

  network: {
    ip_address: string;
    ports: string[];              // ["GbE x4", "10GbE x2"]
    redundant: boolean;
  };

  status: {
    online: boolean;
    recording: boolean;
    disk_health: 'good' | 'warning' | 'critical';
  };
}

// ============================================================================
// Intrusion Detection System Types
// ============================================================================

/**
 * Security sensor types
 */
export enum SensorType {
  PIR_MOTION = 'pir_motion',
  DOOR_WINDOW = 'door_window',
  GLASS_BREAK = 'glass_break',
  VIBRATION = 'vibration',
  SMOKE = 'smoke',
  HEAT = 'heat',
  CO = 'carbon_monoxide',
  WATER_LEAK = 'water_leak',
  FENCE = 'fence',
  BEAM = 'beam',
}

/**
 * Security sensor
 */
export interface SecuritySensor {
  sensor_id: string;
  name: string;
  type: SensorType;
  location: Location;

  specifications: {
    manufacturer: string;
    model: string;
    detection_range?: number;     // meters
    detection_angle?: number;     // degrees
  };

  communication: {
    protocol: 'wired' | 'zigbee' | 'z-wave' | 'lorawan' | 'nb-iot';
    address?: string;
    gateway_id?: string;
  };

  state: {
    armed: boolean;
    triggered: boolean;
    battery_level?: number;       // %
    signal_strength?: number;     // dBm
    tampered: boolean;
    malfunction: boolean;
    last_update: Timestamp;
  };

  settings: {
    sensitivity: number;          // 0-10
    delay: number;                // seconds
    pet_immunity?: boolean;
    pet_weight_limit?: number;    // kg
  };
}

/**
 * Intrusion detector (advanced sensor with multiple detection methods)
 */
export interface IntrusionDetector {
  detector_id: string;
  name: string;
  location: Location;

  detection_methods: {
    pir: boolean;
    microwave: boolean;
    dual_tech: boolean;           // PIR + Microwave
    vibration: boolean;
    acoustic: boolean;
  };

  coverage: {
    range: number;                // meters
    angle: number;                // degrees
    area: number;                 // m²
  };

  state: {
    armed: boolean;
    triggered: boolean;
    last_trigger: Timestamp;
    trigger_count_today: number;
  };
}

/**
 * Security zone (group of sensors)
 */
export interface SecurityZone {
  zone_id: string;
  zone_name: string;
  zone_type: 'perimeter' | 'interior' | 'critical' | 'access_control';

  sensors: string[];              // Sensor IDs
  cameras: string[];              // Camera IDs for verification

  arming_schedule: {
    weekdays: { start: string; end: string };
    weekends: { start: string; end: string };
    holidays: string[];           // Exception dates
  };

  alarm_settings: {
    entry_delay: number;          // seconds
    exit_delay: number;           // seconds
    alarm_duration: number;       // seconds
    chime: boolean;
    notification: {
      sms: string[];
      email: string[];
      push: boolean;
      patrol: boolean;
    };
  };

  response_plan: {
    priority: 'critical' | 'high' | 'medium' | 'low';
    auto_actions: string[];       // ["lock_doors", "turn_on_lights", "sound_siren"]
    dispatch_security: boolean;
    notify_police: boolean;
  };

  state: {
    armed: boolean;
    status: 'secure' | 'alarm' | 'trouble' | 'bypass';
    last_armed: Timestamp;
    last_disarmed: Timestamp;
  };
}

// ============================================================================
// Guard System Types
// ============================================================================

/**
 * Guard/security personnel
 */
export interface Guard {
  guard_id: string;
  name: string;
  employee_id: string;

  contact: {
    phone: string;
    radio_channel?: number;
    emergency_contact?: string;
  };

  shift: 'day' | 'night' | 'graveyard';
  status: 'on_patrol' | 'at_post' | 'break' | 'responding' | 'off_duty';

  current_location?: {
    timestamp: Timestamp;
    location: Location;
    tracking_method: 'gps' | 'wifi' | 'ble_beacon' | 'checkpoint_scan';
  };

  assigned_zones: string[];
  equipment: string[];            // ["flashlight", "radio", "taser"]

  panic_button: {
    enabled: boolean;
    last_test: Timestamp;
  };

  certifications: {
    name: string;
    expires: Timestamp;
  }[];
}

/**
 * Patrol route
 */
export interface PatrolRoute {
  route_id: string;
  route_name: string;
  shift: 'day' | 'night' | 'graveyard';

  checkpoints: {
    checkpoint_id: string;
    location: Location;
    sequence: number;
    scan_method: 'nfc' | 'qr_code' | 'rfid' | 'gps';
    required_tasks: string[];
    estimated_duration: number;   // minutes
  }[];

  schedule: {
    frequency: number;            // times per shift
    interval: number;             // minutes between patrols
    start_time: string;
    end_time: string;
  };

  compliance: {
    tolerance_minutes: number;
    missed_checkpoint_action: 'alert' | 'log' | 'critical';
    incomplete_patrol_action: 'alert' | 'log' | 'escalate';
  };
}

/**
 * Patrol checkpoint
 */
export interface PatrolCheckpoint {
  checkpoint_id: string;
  name: string;
  location: Location;
  scan_code: string;              // NFC/QR code data

  tasks: {
    task: string;
    required: boolean;
  }[];

  last_scanned?: {
    timestamp: Timestamp;
    guard_id: string;
    tasks_completed: string[];
    notes?: string;
  };
}

// ============================================================================
// Emergency Call System Types
// ============================================================================

/**
 * Panic button
 */
export interface PanicButton {
  device_id: string;
  name: string;
  location: Location;

  features: {
    single_press: 'alert' | 'silent_alert';
    double_press: 'critical_alert';
    hold_3s: 'evacuation_alert';
  };

  indicators: {
    led: boolean;
    audible: boolean;
    local_siren: boolean;
    siren_delay: number;          // seconds
  };

  communication: {
    protocol: 'ip' | 'cellular' | 'radio';
    backup: 'cellular' | 'radio';
    two_way_audio: boolean;
  };

  status: {
    online: boolean;
    battery_level?: number;
    last_test: Timestamp;
  };
}

/**
 * Video intercom
 */
export interface VideoIntercom {
  device_id: string;
  name: string;
  location: Location;

  features: {
    video: {
      resolution: CameraResolution;
      night_vision: boolean;
      wdr: boolean;
    };
    audio: {
      two_way: boolean;
      noise_cancellation: boolean;
    };
    display: {
      size: string;               // "7 inch"
      touchscreen: boolean;
    };
    access_control: {
      integrated: boolean;
      unlock_methods: string[];   // ["keypad", "rfid", "face_recognition"]
    };
  };

  call_routing: {
    primary: string;              // Guard station ID
    backup: string[];
    timeout: number;              // seconds
    recording: {
      enabled: boolean;
      storage_days: number;
    };
  };

  status: {
    online: boolean;
    in_call: boolean;
  };
}

/**
 * Emergency types
 */
export type EmergencyType =
  | 'medical'
  | 'fire'
  | 'intrusion'
  | 'assault'
  | 'natural_disaster'
  | 'bomb_threat'
  | 'active_shooter'
  | 'hazmat'
  | 'evacuation'
  | 'other';

/**
 * Emergency alert
 */
export interface EmergencyAlert {
  alert_id: string;
  timestamp: Timestamp;
  type: EmergencyType;
  priority: 'critical' | 'high' | 'medium';

  location: Location;

  reporter: {
    type: 'guard' | 'employee' | 'visitor' | 'sensor' | 'ai_detection';
    id?: string;
    name?: string;
  };

  details: {
    description: string;
    casualties?: number;
    hazard_level?: 'low' | 'medium' | 'high';
    spread_risk?: boolean;
  };

  response: {
    dispatched: string[];         // ["guard_team_1", "ambulance", "fire_dept"]
    eta: number[];                // minutes
    status: 'dispatched' | 'en_route' | 'on_scene' | 'resolved';
  };

  communications: {
    pa_announcement: boolean;
    sms_broadcast: boolean;
    app_notification: boolean;
    email_notification: boolean;
  };

  resolution?: {
    resolved_at: Timestamp;
    resolved_by: string;
    outcome: string;
    report_id?: string;
  };
}

// ============================================================================
// Video Analytics Types
// ============================================================================

/**
 * Object detection
 */
export interface ObjectDetection {
  detection_id: string;
  timestamp: Timestamp;
  camera_id: string;

  object: {
    type: 'person' | 'vehicle' | 'animal' | 'package' | 'unknown';
    subtype?: string;
    confidence: number;           // 0.0 - 1.0

    bounding_box: {
      x: number;
      y: number;
      width: number;
      height: number;
    };

    attributes?: {
      color?: string;
      size?: 'small' | 'medium' | 'large';
      speed?: number;             // pixels/second
      direction?: number;         // degrees (0-360)
    };
  };

  tracking?: {
    track_id: string;
    first_seen: Timestamp;
    last_seen: Timestamp;
    path: { x: number; y: number; timestamp: Timestamp }[];
    cameras_visited: string[];
  };
}

/**
 * Behavior types
 */
export type BehaviorType =
  | 'loitering'
  | 'running'
  | 'fighting'
  | 'falling'
  | 'crowd_forming'
  | 'wrong_direction'
  | 'tailgating'
  | 'abandoned_object'
  | 'object_removed'
  | 'fence_jumping'
  | 'vehicle_stopped';

/**
 * Behavior analysis
 */
export interface BehaviorAnalysis {
  analysis_id: string;
  timestamp: Timestamp;
  camera_id: string;

  detected_behaviors: {
    behavior_type: BehaviorType;
    confidence: number;
    duration: number;             // seconds
    location: { x: number; y: number };
    subjects: string[];           // tracking IDs

    alert: {
      triggered: boolean;
      severity: 'info' | 'warning' | 'critical';
      notification_sent: boolean;
    };
  }[];
}

/**
 * Face recognition
 */
export interface FaceRecognition {
  detection_id: string;
  timestamp: Timestamp;
  camera_id: string;

  face: {
    bounding_box: {
      x: number;
      y: number;
      width: number;
      height: number;
    };

    quality: {
      score: number;
      sharpness: number;
      brightness: number;
      angle: {
        yaw: number;
        pitch: number;
        roll: number;
      };
    };

    features: {
      embedding: number[];        // 512-dimensional vector
      landmarks: {
        left_eye: { x: number; y: number };
        right_eye: { x: number; y: number };
        nose: { x: number; y: number };
        left_mouth: { x: number; y: number };
        right_mouth: { x: number; y: number };
      };
    };
  };

  recognition?: {
    matched: boolean;
    person_id?: string;
    confidence?: number;
    watchlist_match?: {
      list_name: string;
      reason: string;
      alert_triggered: boolean;
    };
  };

  privacy: {
    anonymized: boolean;
    consent_obtained: boolean;
    retention_until: Timestamp;
  };
}

/**
 * License plate recognition
 */
export interface LicensePlateRecognition {
  detection_id: string;
  timestamp: Timestamp;
  camera_id: string;

  vehicle: {
    type: 'car' | 'truck' | 'motorcycle' | 'bus';
    color?: string;
    make?: string;
    model?: string;
    bounding_box: {
      x: number;
      y: number;
      width: number;
      height: number;
    };
  };

  license_plate: {
    number: string;
    country: string;
    region?: string;
    confidence: number;

    bounding_box: {
      x: number;
      y: number;
      width: number;
      height: number;
    };

    image: {
      full_frame: string;         // URL
      cropped_plate: string;      // URL
    };
  };

  access_control?: {
    authorized: boolean;
    whitelist_match?: boolean;
    blacklist_match?: boolean;
    visitor_pass?: string;
    action: 'allow' | 'deny' | 'alert' | 'manual_review';
  };

  parking?: {
    entry_time?: Timestamp;
    exit_time?: Timestamp;
    duration?: number;            // minutes
    parking_spot?: string;
    fee?: number;
  };
}

/**
 * Crowd analytics
 */
export interface CrowdAnalytics {
  analysis_id: string;
  timestamp: Timestamp;
  camera_id: string;
  zone_id: string;

  crowd_metrics: {
    people_count: number;
    density: number;              // people per m²
    flow_rate: number;            // people per minute

    occupancy: {
      current: number;
      capacity: number;
      percentage: number;
      status: 'normal' | 'busy' | 'crowded' | 'critical';
    };

    movement: {
      average_speed: number;      // m/s
      direction: {
        dominant: number;         // degrees
        entropy: number;          // 0.0 - 1.0
      };
      bottleneck_detected: boolean;
    };
  };

  alerts: {
    overcrowding: boolean;
    stampede_risk: boolean;
    panic_detected: boolean;
    queue_too_long: boolean;
    exit_blocked: boolean;
  };

  recommendations: string[];
}

// ============================================================================
// Security Event Types
// ============================================================================

/**
 * Security event types
 */
export type SecurityEventType =
  | 'alarm'
  | 'access_granted'
  | 'access_denied'
  | 'sensor_triggered'
  | 'camera_motion'
  | 'patrol_checkpoint'
  | 'emergency_call'
  | 'system_status'
  | 'user_action'
  | 'configuration_change';

/**
 * Security event
 */
export interface SecurityEvent {
  event_id: string;
  timestamp: Timestamp;
  event_type: SecurityEventType;
  severity: 'critical' | 'high' | 'medium' | 'low' | 'info';

  source: {
    type: 'camera' | 'sensor' | 'access_control' | 'guard' | 'system' | 'user';
    id: string;
    location?: string;
  };

  details: {
    description: string;
    metadata: Record<string, any>;
    related_events?: string[];
  };

  response?: {
    acknowledged: boolean;
    acknowledged_by?: string;
    acknowledged_at?: Timestamp;
    actions_taken?: string[];
    resolved: boolean;
    resolved_at?: Timestamp;
    notes?: string;
  };

  evidence?: {
    video_clips?: string[];       // URLs
    photos?: string[];            // URLs
    audio_recordings?: string[];  // URLs
    documents?: string[];         // URLs
  };
}

/**
 * Security alert
 */
export interface SecurityAlert {
  alert_id: string;
  timestamp: Timestamp;
  type: string;
  severity: 'critical' | 'high' | 'medium' | 'low';
  status: 'active' | 'acknowledged' | 'resolved' | 'closed';

  source: {
    type: 'camera' | 'sensor' | 'system' | 'analytics';
    id: string;
    name: string;
    location: Location;
  };

  message: string;
  description?: string;

  actions_required: string[];
  auto_actions_taken?: string[];

  acknowledged_by?: string;
  acknowledged_at?: Timestamp;
  resolved_by?: string;
  resolved_at?: Timestamp;

  related_events: string[];       // Event IDs
}

// ============================================================================
// Control Center Types
// ============================================================================

/**
 * Control center
 */
export interface ControlCenter {
  center_id: string;
  name: string;
  location: Location;

  operators: {
    shift: 'day' | 'night' | 'graveyard';
    operators_on_duty: number;
    supervisor: string;
  };

  workstations: {
    workstation_id: string;
    operator_id?: string;
    displays: {
      primary: {
        size: string;
        resolution: CameraResolution;
        video_wall_feed: boolean;
      };
      secondary: {
        size: string;
        resolution: CameraResolution;
        applications: string[];
      }[];
    };
  }[];

  systems_integrated: {
    cctv: boolean;
    intrusion_detection: boolean;
    access_control: boolean;
    fire_alarm: boolean;
    building_automation: boolean;
    intercom: boolean;
    parking: boolean;
  };

  status: {
    operational: boolean;
    active_alerts: number;
    cameras_online: number;
    cameras_total: number;
    sensors_online: number;
    sensors_total: number;
  };
}

/**
 * Video wall
 */
export interface VideoWall {
  wall_id: string;
  layout: {
    rows: number;
    columns: number;
    total_screens: number;
  };

  displays: {
    display_id: string;
    resolution: CameraResolution;
    size: number;                 // inches
    bezel_width: number;          // mm
  }[];

  presets: {
    preset_id: string;
    name: string;
    camera_assignments: {
      display_id: string;
      cameras: string[];
      layout: string;             // "1x1", "2x2", etc.
    }[];
  }[];

  current_preset?: string;
}

// ============================================================================
// Incident Investigation Types
// ============================================================================

/**
 * Incident investigation
 */
export interface IncidentInvestigation {
  incident_id: string;
  created_at: Timestamp;
  status: 'open' | 'investigating' | 'closed' | 'archived';

  incident_details: {
    date: string;
    time: string;
    location: Location;
    type: string;
    severity: 'critical' | 'high' | 'medium' | 'low';
    description: string;
  };

  involved_parties: {
    victims?: { name: string; contact: string; statement?: string }[];
    suspects?: { description: string; photo?: string }[];
    witnesses?: { name: string; contact: string; statement?: string }[];
  };

  evidence_collected: {
    video: {
      camera_id: string;
      start_time: Timestamp;
      end_time: Timestamp;
      file_url: string;
      notes?: string;
    }[];

    photos: {
      file_url: string;
      timestamp: Timestamp;
      description: string;
    }[];

    sensor_logs: {
      sensor_id: string;
      data: any[];
    }[];

    other_evidence: {
      type: string;
      description: string;
      file_url?: string;
    }[];
  };

  investigation: {
    lead_investigator: string;
    team_members: string[];
    findings: string;
    conclusion: string;
    recommendations: string[];
  };

  actions_taken?: {
    disciplinary?: string;
    policy_changes?: string;
    system_upgrades?: string;
    training?: string;
  };

  reporting: {
    internal_report?: string;     // URL
    police_report_filed: boolean;
    police_report_number?: string;
    insurance_claim?: string;
  };
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Generic API response
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: Timestamp;
    version: string;
  };
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page: number;
  limit: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  limit: number;
  hasMore: boolean;
}

/**
 * Date range filter
 */
export interface DateRangeFilter {
  startDate: Timestamp;
  endDate: Timestamp;
}

// ============================================================================
// Security and Access Control
// ============================================================================

/**
 * User role
 */
export enum Role {
  ADMIN = 'admin',
  SUPERVISOR = 'supervisor',
  OPERATOR = 'operator',
  GUARD = 'guard',
  VIEWER = 'viewer',
}

/**
 * Permission
 */
export interface Permission {
  resource: 'cameras' | 'sensors' | 'alerts' | 'zones' | 'guards' | 'analytics' | 'reports';
  action: 'read' | 'write' | 'delete' | 'control';
  scope?: string;
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Location,
};
