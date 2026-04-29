/**
 * WIA-MARITIME_SAFETY TypeScript Type Definitions
 * Version: 1.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geographic Types
// ============================================================================

export interface Position {
  latitude: number;   // -90 to 90
  longitude: number;  // -180 to 180
  accuracy?: boolean; // true = high (<10m), false = low (>10m)
}

export interface GeoArea {
  type: 'circle' | 'polygon' | 'rectangle';
  center?: Position;
  radius?: number;    // nautical miles
  coordinates?: Position[];
}

// ============================================================================
// Vessel Identity & Information
// ============================================================================

export interface VesselIdentity {
  mmsi: string;       // 9 digits
  imo?: string;       // 7 digits
  callSign: string;
  vesselName: string;
  vesselType: VesselType;
  flag: string;       // ISO 3166-1 alpha-2
  timestamp: string;  // ISO 8601
  signature?: string; // Ed25519
}

export type VesselType =
  | 'wing_in_ground'
  | 'fishing'
  | 'towing'
  | 'dredging'
  | 'diving'
  | 'military'
  | 'sailing'
  | 'pleasure'
  | 'high_speed'
  | 'pilot'
  | 'sar'
  | 'tug'
  | 'port_tender'
  | 'passenger'
  | 'cargo'
  | 'tanker'
  | 'other';

export interface VesselDimensions {
  length: number;     // meters
  beam: number;       // meters
  draft: number;      // meters
  height?: number;    // meters
}

export interface VesselCharacteristics {
  type: VesselType;
  dimensions: VesselDimensions;
  grossTonnage?: number;
  deadweight?: number;
  maxSpeed?: number;  // knots
  yearBuilt?: number;
}

// ============================================================================
// AIS Data Types
// ============================================================================

export type NavigationStatus =
  | 'under_way_engine'
  | 'at_anchor'
  | 'not_under_command'
  | 'restricted_maneuverability'
  | 'constrained_by_draught'
  | 'moored'
  | 'aground'
  | 'engaged_in_fishing'
  | 'under_way_sailing'
  | 'ais_sart'
  | 'undefined';

export interface NavigationData {
  status: NavigationStatus;
  rateOfTurn?: number;        // degrees/minute
  speedOverGround: number;    // knots
  courseOverGround: number;   // degrees
  heading: number;            // degrees
}

export interface AISPositionReport {
  type: 'WIA-MARITIME_SAFETY-AISPosition';
  version: string;
  mmsi: string;
  timestamp: string;          // ISO 8601
  position: Position;
  navigation: NavigationData;
  maneuver?: string;
  raim?: boolean;             // Receiver Autonomous Integrity Monitoring
  signature?: string;
}

export interface AISStaticData {
  type: 'WIA-MARITIME_SAFETY-VesselIdentity';
  version: string;
  mmsi: string;
  imo?: string;
  callSign: string;
  vesselName: string;
  vesselType: string;
  dimensions: VesselDimensions;
  flag: string;
  timestamp: string;
  signature?: string;
}

// ============================================================================
// Weather & Environmental Data
// ============================================================================

export interface WeatherConditions {
  windSpeed: number;          // knots
  windDirection: number;      // degrees
  waveHeight: number;         // meters
  wavePeriod?: number;        // seconds
  visibility: number;         // nautical miles
  seaTemperature?: number;    // celsius
  airTemperature?: number;    // celsius
  barometricPressure?: number; // hPa
}

export interface WeatherForecast {
  validUntil: string;         // ISO 8601
  warnings: string[];
  hourly?: WeatherConditions[];
}

export interface WeatherData {
  type: 'WIA-MARITIME_SAFETY-Weather';
  version: string;
  id: string;                 // UUID v4
  timestamp: string;          // ISO 8601
  position: Position;
  conditions: WeatherConditions;
  forecast?: WeatherForecast;
}

// ============================================================================
// Safety Alerts & Emergencies
// ============================================================================

export type AlertSeverity = 'info' | 'warning' | 'critical' | 'distress';

export type AlertCategory =
  | 'collision'
  | 'grounding'
  | 'fire'
  | 'flooding'
  | 'medical'
  | 'piracy'
  | 'pollution'
  | 'man_overboard'
  | 'other';

export type AlertStatus = 'active' | 'acknowledged' | 'resolved';

export interface SafetyAlert {
  type: 'WIA-MARITIME_SAFETY-Alert';
  version: string;
  id: string;                 // UUID v4
  timestamp: string;          // ISO 8601
  severity: AlertSeverity;
  category: AlertCategory;
  position: Position;
  vessel: {
    mmsi: string;
    name: string;
  };
  description: string;
  status: AlertStatus;
  responders?: string[];      // Array of MMSI strings
}

export interface DistressCall {
  format: 'DSC' | 'MAYDAY' | 'EPIRB' | 'SART';
  nature: AlertCategory;
  position: Position;
  timestamp: string;
  pob?: number;               // Persons on Board
  vessel: VesselIdentity;
  additionalInfo?: string;
}

// ============================================================================
// Route Planning & Navigation
// ============================================================================

export interface Waypoint {
  position: Position;
  name?: string;
  eta?: string;               // ISO 8601
  speed?: number;             // knots
}

export interface Route {
  id: string;
  name?: string;
  waypoints: Waypoint[];
  totalDistance: number;      // nautical miles
  estimatedDuration: number;  // hours
  departureTime?: string;     // ISO 8601
  arrivalTime?: string;       // ISO 8601
}

export interface RouteCalculationRequest {
  origin: Position;
  destination: Position;
  vessel: VesselCharacteristics;
  preferences: {
    avoidWeather?: boolean;
    avoidTraffic?: boolean;
    minimizeFuel?: boolean;
    departureTime?: string;   // ISO 8601
  };
}

export interface RouteCalculationResponse {
  route: Route;
  fuelEstimate?: number;      // metric tons
  warnings: string[];
  alternateRoutes?: Route[];
  maxConditions?: {
    waveHeight: number;
    windSpeed: number;
  };
}

// ============================================================================
// Port & Berth Management
// ============================================================================

export interface PortInfo {
  portId: string;             // UN/LOCODE
  name: string;
  position: Position;
  facilities: {
    maxDraft: number;         // meters
    maxLength: number;        // meters
    berths: number;
    services: string[];
  };
  currentConditions?: {
    berthsAvailable: number;
    averageWaitTime: number;  // hours
    restrictions: string[];
  };
}

export interface BerthRequest {
  vessel: VesselIdentity & VesselDimensions;
  eta: string;                // ISO 8601
  serviceRequired: string[];
  cargoType?: string;
  stayDuration?: number;      // hours
}

export interface BerthAssignment {
  requestId: string;
  berthNumber: string;
  confirmedEta: string;       // ISO 8601
  services: string[];
  specialInstructions?: string[];
}

// ============================================================================
// Communication Protocols
// ============================================================================

export type VHFChannel = 6 | 9 | 12 | 13 | 16 | 70;

export interface VHFMessage {
  channel: VHFChannel;
  priority: 'distress' | 'urgent' | 'safety' | 'routine';
  from: string;               // Call sign or MMSI
  to?: string;                // Call sign or MMSI (broadcast if omitted)
  message: string;
  timestamp: string;          // ISO 8601
}

export type GMDSSPriority = 'MAYDAY' | 'PAN-PAN' | 'SECURITE' | 'ROUTINE';

export interface GMDSSMessage {
  priority: GMDSSPriority;
  category?: AlertCategory;
  position?: Position;
  content: string;
  timestamp: string;          // ISO 8601
}

// ============================================================================
// Search and Rescue (SAR)
// ============================================================================

export interface SARIncident {
  id: string;
  distressType: AlertCategory;
  position: Position;
  timestamp: string;          // ISO 8601
  pob: number;                // Persons on Board
  vessel: VesselIdentity;
  conditions?: WeatherConditions;
}

export interface SARAsset {
  type: 'helicopter' | 'vessel' | 'aircraft' | 'shore_team';
  callSign: string;
  position?: Position;
  eta?: string;               // ISO 8601
  capabilities?: string[];
}

export interface SARCoordination {
  incident: SARIncident;
  sarMission: string;
  onSceneCoordinator?: string;
  assets: SARAsset[];
  searchArea?: GeoArea;
  status: 'planning' | 'active' | 'suspended' | 'completed';
}

// ============================================================================
// Collision Avoidance (ARPA/ECDIS)
// ============================================================================

export interface TargetVessel {
  mmsi: string;
  position: Position;
  course: number;             // degrees
  speed: number;              // knots
  vesselType?: VesselType;
  name?: string;
}

export interface CollisionRisk {
  target: TargetVessel;
  ownVessel: {
    position: Position;
    course: number;
    speed: number;
  };
  assessment: {
    risk: 'low' | 'medium' | 'high' | 'critical';
    cpa: number;              // Closest Point of Approach (nautical miles)
    tcpa: number;             // Time to CPA (minutes)
    bearingChange: boolean;   // True if bearing not changing
  };
  recommendation?: {
    action: string;
    colregsRule: number;      // COLREGS Rule number (13-18)
    soundSignal?: string;
  };
}

// ============================================================================
// SOLAS & IMO Compliance
// ============================================================================

export interface Certificate {
  number: string;
  type: string;
  issued: string;             // ISO 8601 date
  expires: string;            // ISO 8601 date
  issuer: string;
  status: 'valid' | 'expired' | 'suspended' | 'revoked';
}

export interface SOLASCompliance {
  vessel: VesselIdentity;
  certificates: {
    safetyConstruction?: Certificate;
    safetyEquipment?: Certificate;
    safetyRadio?: Certificate;
    loadLine?: Certificate;
  };
  inspections: {
    lastAnnual?: string;      // ISO 8601 date
    lastIntermediate?: string; // ISO 8601 date
    nextDue?: string;         // ISO 8601 date
  };
}

export interface MARPOLCompliance {
  vessel: VesselIdentity;
  annexI?: {
    iopp: Certificate;        // International Oil Pollution Prevention
    oilRecordBook: boolean;
    sopep: boolean;           // Shipboard Oil Pollution Emergency Plan
  };
  annexVI?: {
    eiapp: Certificate;       // Engine International Air Pollution Prevention
    fuelSulphur: {
      global: number;         // percentage
      eca: number;            // Emission Control Area percentage
    };
    seemp: boolean;           // Ship Energy Efficiency Management Plan
  };
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIResponse<T> {
  status: 'success' | 'error';
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: Record<string, unknown>;
  };
  timestamp: string;          // ISO 8601
}

export interface PaginatedResponse<T> {
  data: T[];
  total: number;
  page: number;
  pageSize: number;
  hasMore: boolean;
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface MaritimeSafetyConfig {
  apiEndpoint: string;
  apiKey?: string;
  timeout?: number;           // milliseconds
  retryAttempts?: number;
  websocket?: {
    enabled: boolean;
    endpoint?: string;
    reconnect?: boolean;
  };
}

// ============================================================================
// Real-time Streaming Types
// ============================================================================

export type StreamChannel = 'vessels' | 'alerts' | 'weather' | 'sar';

export interface StreamSubscription {
  action: 'subscribe' | 'unsubscribe';
  channels: StreamChannel[];
  filters?: {
    area?: GeoArea;
    vesselTypes?: VesselType[];
    severity?: AlertSeverity[];
  };
}

export interface StreamUpdate<T> {
  channel: StreamChannel;
  event: string;
  data: T;
  timestamp: string;          // ISO 8601
}

// ============================================================================
// Vessel Tracking Types
// ============================================================================

export interface VesselTrackRequest {
  mmsiList: string[];
  fields?: ('position' | 'navigation' | 'identity' | 'weather')[];
  realtime?: boolean;
}

export interface VesselTrackResponse {
  mmsi: string;
  vesselName?: string;
  position?: Position;
  navigation?: NavigationData;
  identity?: VesselIdentity;
  weather?: WeatherConditions;
  lastUpdate: string;         // ISO 8601
}

export interface VesselSearchRequest {
  area: GeoArea;
  filters?: {
    vesselTypes?: VesselType[];
    minLength?: number;
    maxLength?: number;
    flags?: string[];
    navigationStatus?: NavigationStatus[];
  };
  page?: number;
  pageSize?: number;
}

// ============================================================================
// LRIT (Long Range Identification and Tracking) Types
// ============================================================================

export interface LRITReport {
  vessel: VesselIdentity;
  position: Position;
  timestamp: string;          // ISO 8601
  flagState: string;
  reportingInterval: number;  // minutes
}

// ============================================================================
// AMVER (Automated Mutual-Assistance Vessel Rescue) Types
// ============================================================================

export interface AMVERReport {
  vessel: VesselIdentity;
  position: Position;
  voyage: {
    from: string;             // Port code
    to: string;               // Port code
    eta: string;              // ISO 8601
    speed: number;            // knots
  };
  capabilities: {
    medicalOfficer: boolean;
    hospital: boolean;
    helipad: boolean;
    divingEquipment: boolean;
    salvageEquipment: boolean;
    pollutionControl: boolean;
  };
  participation: {
    sarAvailable: boolean;
    diversionRadius: number;  // nautical miles
    responseTime: number;     // hours
  };
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  // Add any additional type exports here
};
