/**
 * WIA-SOC-006: Disaster Management System Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

export type Timestamp = string;
export type UUID = string;

export interface Coordinates {
  lat: number;
  lon: number;
  altitude?: number;
}

export interface BoundingBox {
  north: number;
  south: number;
  east: number;
  west: number;
}

export interface Polygon {
  type: 'Polygon';
  coordinates: number[][][];
  boundingBox?: BoundingBox;
}

// ============================================================================
// Disaster Event Types
// ============================================================================

export enum DisasterType {
  TORNADO = 'tornado',
  EARTHQUAKE = 'earthquake',
  FLOOD = 'flood',
  WILDFIRE = 'wildfire',
  HURRICANE = 'hurricane',
  TSUNAMI = 'tsunami',
  PANDEMIC = 'pandemic'
}

export enum EventStatus {
  WATCH = 'watch',
  ADVISORY = 'advisory',
  WARNING = 'warning',
  EMERGENCY = 'emergency',
  CRITICAL = 'critical',
  RESOLVED = 'resolved'
}

export interface DisasterEvent {
  eventId: UUID;
  disasterType: DisasterType;
  severity: number;
  status: EventStatus;
  affectedArea: Polygon;
  startTime: Timestamp;
  endTime?: Timestamp;
  estimatedAffectedPopulation: number;
  confirmedCasualties?: number;
  description: string;
  resources?: UUID[];
  alerts?: UUID[];
}

// ============================================================================
// Alert Types
// ============================================================================

export enum AlertPriority {
  INFO = 'info',
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high',
  CRITICAL = 'critical'
}

export enum AlertCategory {
  GEO = 'geo',
  MET = 'met',
  SAFETY = 'safety',
  SECURITY = 'security',
  RESCUE = 'rescue',
  FIRE = 'fire',
  HEALTH = 'health',
  ENV = 'env',
  TRANSPORT = 'transport',
  INFRA = 'infra',
  CBRNE = 'cbrne',
  OTHER = 'other'
}

export enum AlertUrgency {
  IMMEDIATE = 'immediate',
  EXPECTED = 'expected',
  FUTURE = 'future',
  PAST = 'past'
}

export enum AlertSeverity {
  EXTREME = 'extreme',
  SEVERE = 'severe',
  MODERATE = 'moderate',
  MINOR = 'minor',
  UNKNOWN = 'unknown'
}

export enum AlertCertainty {
  OBSERVED = 'observed',
  LIKELY = 'likely',
  POSSIBLE = 'possible',
  UNLIKELY = 'unlikely',
  UNKNOWN = 'unknown'
}

export interface AlertArea {
  description: string;
  polygon?: Coordinates[];
  circle?: {
    lat: number;
    lon: number;
    radius: number;
  };
  geocode?: {
    name: string;
    value: string;
  };
}

export interface AlertMessage {
  alertId: UUID;
  timestamp: Timestamp;
  eventId: UUID;
  priority: AlertPriority;
  category: AlertCategory;
  urgency: AlertUrgency;
  severity: AlertSeverity;
  certainty: AlertCertainty;
  scope: 'public' | 'restricted' | 'private';
  headline: string;
  description: string;
  instruction: string;
  web?: string;
  contact?: string;
  area: AlertArea;
  resources?: UUID[];
  expiresAt: Timestamp;
}

// ============================================================================
// Resource Types
// ============================================================================

export enum ResourceType {
  MEDICAL_TEAM = 'medical_team',
  RESCUE_TEAM = 'rescue_team',
  FIRE_UNIT = 'fire_unit',
  POLICE_UNIT = 'police_unit',
  SHELTER = 'shelter',
  SUPPLIES = 'supplies',
  EQUIPMENT = 'equipment',
  VEHICLE = 'vehicle'
}

export enum ResourceStatus {
  AVAILABLE = 'available',
  DEPLOYED = 'deployed',
  STANDBY = 'standby',
  MAINTENANCE = 'maintenance',
  OFFLINE = 'offline'
}

export interface ResourceCapacity {
  personnel?: number;
  equipment?: Record<string, number>;
  supplies?: Record<string, number>;
  shelter_capacity?: number;
}

export interface ResourceContact {
  primary: string;
  secondary?: string;
  radio?: string;
  email?: string;
}

export interface EmergencyResource {
  resourceId: UUID;
  type: ResourceType;
  name: string;
  agency: string;
  status: ResourceStatus;
  location: Coordinates & { address?: string };
  capacity: ResourceCapacity;
  capabilities: string[];
  availability: {
    '24_7': boolean;
    schedule?: Record<string, any>;
  };
  contact: ResourceContact;
}

// ============================================================================
// Deployment Types
// ============================================================================

export enum DeploymentMission {
  SEARCH_RESCUE = 'search_rescue',
  MEDICAL_AID = 'medical_aid',
  EVACUATION = 'evacuation',
  FIRE_SUPPRESSION = 'fire_suppression',
  SECURITY = 'security',
  LOGISTICS = 'logistics',
  ASSESSMENT = 'assessment'
}

export enum DeploymentStatus {
  EN_ROUTE = 'en_route',
  ON_SITE = 'on_site',
  ACTIVE = 'active',
  RETURNING = 'returning',
  COMPLETE = 'complete'
}

export interface DeploymentPersonnel {
  id: UUID;
  name: string;
  role: string;
  certifications: string[];
}

export interface DeploymentProgress {
  people_rescued?: number;
  area_covered?: number;
  tasks_completed?: number;
}

export interface ResourceDeployment {
  deploymentId: UUID;
  eventId: UUID;
  resourceId: UUID;
  deployedAt: Timestamp;
  recalledAt?: Timestamp;
  assignedArea: Polygon;
  mission: DeploymentMission;
  status: DeploymentStatus;
  personnel?: DeploymentPersonnel[];
  progress?: DeploymentProgress;
}

// ============================================================================
// Evacuation Types
// ============================================================================

export enum EvacuationType {
  MANDATORY = 'mandatory',
  RECOMMENDED = 'recommended',
  VOLUNTARY = 'voluntary'
}

export interface EvacuationRoute {
  routeId: UUID;
  name: string;
  waypoints: Coordinates[];
  capacity: number;
  status: 'open' | 'congested' | 'closed';
}

export interface EvacuationSpecialNeeds {
  medical: boolean;
  pets: boolean;
  mobility_assistance: boolean;
}

export interface EvacuationOrder {
  orderId: UUID;
  eventId: UUID;
  issuedAt: Timestamp;
  issuedBy: string;
  type: EvacuationType;
  evacuationZone: Polygon;
  estimatedPopulation: number;
  evacuationRoutes: EvacuationRoute[];
  shelters: UUID[];
  deadline?: Timestamp;
  instructions: string;
  specialNeeds?: EvacuationSpecialNeeds;
}

// ============================================================================
// Sensor Data Types
// ============================================================================

export interface WeatherData {
  timestamp: Timestamp;
  sensorId: UUID;
  location: Coordinates;
  temperature: number;
  humidity: number;
  pressure: number;
  windSpeed: number;
  windDirection: number;
  precipitation: number;
  visibility: number;
  conditions: string;
}

export interface SeismicData {
  timestamp: Timestamp;
  stationId: UUID;
  location: Coordinates & { depth: number };
  magnitude: number;
  scale: 'richter' | 'moment' | 'jma';
  epicenter: Coordinates;
  depth: number;
  intensity: number;
  waveforms?: {
    p_wave?: string;
    s_wave?: string;
  };
}

export interface WaterLevelData {
  timestamp: Timestamp;
  sensorId: UUID;
  location: Coordinates;
  waterLevel: number;
  flowRate: number;
  floodStage: number;
  status: 'normal' | 'watch' | 'minor' | 'moderate' | 'major';
  trend: 'rising' | 'falling' | 'stable';
}

// ============================================================================
// Communication Types
// ============================================================================

export enum MessagePriority {
  ROUTINE = 'routine',
  PRIORITY = 'priority',
  IMMEDIATE = 'immediate',
  FLASH = 'flash'
}

export interface MessageAttachment {
  type: 'document' | 'image' | 'video' | 'data';
  url: string;
  checksum: string;
}

export interface InterAgencyMessage {
  messageId: UUID;
  timestamp: Timestamp;
  fromAgency: string;
  toAgency: string | string[];
  priority: MessagePriority;
  subject: string;
  body: string;
  attachments?: MessageAttachment[];
  requiresResponse: boolean;
  expiresAt?: Timestamp;
}

// ============================================================================
// Incident Log Types
// ============================================================================

export enum LogSeverity {
  INFO = 'info',
  WARNING = 'warning',
  ERROR = 'error',
  CRITICAL = 'critical'
}

export enum LogCategory {
  ALERT = 'alert',
  DEPLOYMENT = 'deployment',
  EVACUATION = 'evacuation',
  CASUALTY = 'casualty',
  DAMAGE = 'damage',
  RESOURCE = 'resource',
  COMMUNICATION = 'communication'
}

export interface IncidentLog {
  logId: UUID;
  eventId: UUID;
  timestamp: Timestamp;
  severity: LogSeverity;
  category: LogCategory;
  actor: string;
  action: string;
  location?: Coordinates;
  details?: Record<string, any>;
  metadata?: Record<string, any>;
}

// ============================================================================
// Damage Assessment Types
// ============================================================================

export enum StructureType {
  RESIDENTIAL = 'residential',
  COMMERCIAL = 'commercial',
  INDUSTRIAL = 'industrial',
  INFRASTRUCTURE = 'infrastructure',
  AGRICULTURAL = 'agricultural'
}

export enum DamageLevel {
  NONE = 'none',
  MINOR = 'minor',
  MODERATE = 'moderate',
  SEVERE = 'severe',
  DESTROYED = 'destroyed'
}

export interface DamageAssessment {
  assessmentId: UUID;
  eventId: UUID;
  timestamp: Timestamp;
  assessor: string;
  location: Coordinates;
  structureType: StructureType;
  damageLevel: DamageLevel;
  estimatedCost: number;
  hazards: string[];
  habitability: 'safe' | 'unsafe' | 'unknown';
  photosUrls?: string[];
  notes?: string;
}

// ============================================================================
// API Client Options
// ============================================================================

export interface ApiClientOptions {
  host: string;
  port?: number;
  protocol?: 'http' | 'https';
  token?: string;
  apiKey?: string;
  timeout?: number;
  retries?: number;
}

export interface WebSocketOptions {
  reconnect?: boolean;
  reconnectInterval?: number;
  maxReconnectAttempts?: number;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface ApiResponse<T = any> {
  status: 'success' | 'error';
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: Timestamp;
  requestId?: UUID;
}

export interface PaginatedResponse<T> {
  total: number;
  limit: number;
  offset: number;
  hasMore: boolean;
  data: T[];
  links?: {
    next?: string;
    prev?: string;
  };
}

export interface CommandResponse {
  commandId: UUID;
  status: 'accepted' | 'rejected' | 'completed' | 'failed';
  message?: string;
  data?: Record<string, any>;
}

// ============================================================================
// Event Types
// ============================================================================

export type EventCallback = (event: any) => void;

export interface SystemEvent {
  eventId: UUID;
  timestamp: Timestamp;
  type: string;
  data: any;
}
