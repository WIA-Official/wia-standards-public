/**
 * WIA Smart City Standard
 * TypeScript Type Definitions
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Core City Infrastructure Types
// ============================================================================

export type InfrastructureType =
  | 'transportation'
  | 'utilities'
  | 'public-safety'
  | 'healthcare'
  | 'education'
  | 'waste-management'
  | 'energy'
  | 'water'
  | 'communications';

export type CityZoneType =
  | 'residential'
  | 'commercial'
  | 'industrial'
  | 'mixed-use'
  | 'recreational'
  | 'administrative'
  | 'special';

export type ServiceStatus =
  | 'operational'
  | 'degraded'
  | 'maintenance'
  | 'outage'
  | 'emergency';

// ============================================================================
// IoT Sensor Network Types
// ============================================================================

export type SensorType =
  | 'temperature'
  | 'humidity'
  | 'air-quality'
  | 'noise'
  | 'light'
  | 'motion'
  | 'traffic'
  | 'parking'
  | 'water-quality'
  | 'waste-level'
  | 'energy-meter'
  | 'security-camera'
  | 'weather';

export interface IoTSensor {
  /** Unique sensor identifier */
  sensorId: string;

  /** Type of sensor */
  type: SensorType;

  /** Geographic location */
  location: GeoLocation;

  /** Current status */
  status: 'active' | 'inactive' | 'error' | 'maintenance';

  /** Last reading timestamp */
  lastReading?: Date;

  /** Current sensor value */
  value?: number | string;

  /** Sensor unit of measurement */
  unit?: string;

  /** Battery level (0-100) */
  batteryLevel?: number;

  /** Signal strength (0-100) */
  signalStrength?: number;

  /** Metadata */
  metadata?: Record<string, any>;
}

export interface SensorNetwork {
  /** Network identifier */
  networkId: string;

  /** Network name */
  name: string;

  /** Sensors in the network */
  sensors: IoTSensor[];

  /** Zone coverage */
  zones: CityZoneType[];

  /** Network health status */
  health: 'healthy' | 'degraded' | 'critical';

  /** Total active sensors */
  activeSensors: number;

  /** Last update timestamp */
  lastUpdate: Date;
}

export interface SensorReading {
  sensorId: string;
  timestamp: Date;
  value: number | string;
  unit: string;
  quality: 'good' | 'fair' | 'poor';
  metadata?: Record<string, any>;
}

// ============================================================================
// Traffic Management Types
// ============================================================================

export type TrafficCondition =
  | 'free-flow'
  | 'light'
  | 'moderate'
  | 'heavy'
  | 'congested'
  | 'blocked';

export interface TrafficSignal {
  /** Signal identifier */
  signalId: string;

  /** Location */
  location: GeoLocation;

  /** Current phase */
  currentPhase: 'red' | 'yellow' | 'green' | 'flashing' | 'off';

  /** Phase timing (seconds) */
  phaseTiming: {
    red: number;
    yellow: number;
    green: number;
  };

  /** Adaptive timing enabled */
  adaptive: boolean;

  /** Status */
  status: ServiceStatus;

  /** Last maintenance */
  lastMaintenance?: Date;
}

export interface RoadSegment {
  /** Segment identifier */
  segmentId: string;

  /** Segment name/description */
  name: string;

  /** Start and end coordinates */
  startPoint: GeoLocation;
  endPoint: GeoLocation;

  /** Current traffic condition */
  condition: TrafficCondition;

  /** Average speed (km/h) */
  averageSpeed: number;

  /** Speed limit (km/h) */
  speedLimit: number;

  /** Vehicle count */
  vehicleCount: number;

  /** Incident detected */
  incident?: TrafficIncident;

  /** Last updated */
  lastUpdate: Date;
}

export interface TrafficIncident {
  incidentId: string;
  type: 'accident' | 'roadwork' | 'event' | 'weather' | 'hazard' | 'other';
  severity: 'low' | 'medium' | 'high' | 'critical';
  description: string;
  location: GeoLocation;
  startTime: Date;
  estimatedClearTime?: Date;
  affectedSegments: string[];
  lanesClosed?: number;
  detourAvailable: boolean;
}

export interface ParkingFacility {
  facilityId: string;
  name: string;
  location: GeoLocation;
  totalSpaces: number;
  availableSpaces: number;
  occupancyRate: number;
  pricing: ParkingPricing;
  type: 'street' | 'garage' | 'lot' | 'private';
  evChargingSpaces?: number;
  disabledSpaces?: number;
  lastUpdate: Date;
}

export interface ParkingPricing {
  currency: string;
  hourlyRate: number;
  dailyMax?: number;
  monthlyPass?: number;
  evChargingFee?: number;
}

// ============================================================================
// Public Safety and Emergency Types
// ============================================================================

export type EmergencyType =
  | 'fire'
  | 'medical'
  | 'police'
  | 'natural-disaster'
  | 'infrastructure-failure'
  | 'chemical-hazard'
  | 'security-threat';

export type EmergencyPriority =
  | 'critical'
  | 'high'
  | 'medium'
  | 'low'
  | 'informational';

export interface EmergencyAlert {
  /** Alert identifier */
  alertId: string;

  /** Emergency type */
  type: EmergencyType;

  /** Priority level */
  priority: EmergencyPriority;

  /** Alert title */
  title: string;

  /** Detailed description */
  description: string;

  /** Location of emergency */
  location: GeoLocation;

  /** Affected area radius (meters) */
  affectedRadius: number;

  /** Affected zones */
  affectedZones: string[];

  /** Issue time */
  issuedAt: Date;

  /** Expected resolution time */
  expectedResolution?: Date;

  /** Active status */
  active: boolean;

  /** Response units dispatched */
  responseUnits?: ResponseUnit[];

  /** Evacuation required */
  evacuationRequired: boolean;

  /** Public instructions */
  instructions?: string[];
}

export interface ResponseUnit {
  unitId: string;
  type: 'fire' | 'medical' | 'police' | 'hazmat' | 'rescue';
  status: 'dispatched' | 'en-route' | 'on-scene' | 'returning' | 'available';
  location: GeoLocation;
  eta?: number; // seconds
  personnel: number;
}

export interface PublicSafetyMetrics {
  /** Active incidents count */
  activeIncidents: number;

  /** Response time average (minutes) */
  averageResponseTime: number;

  /** Available emergency units */
  availableUnits: number;

  /** Crime rate (per 100k) */
  crimeRate?: number;

  /** Safety index (0-100) */
  safetyIndex: number;

  /** Last updated */
  lastUpdate: Date;
}

// ============================================================================
// Environmental Monitoring Types
// ============================================================================

export interface AirQualityIndex {
  /** Overall AQI value (0-500) */
  aqi: number;

  /** AQI category */
  category: 'good' | 'moderate' | 'unhealthy-sensitive' | 'unhealthy' | 'very-unhealthy' | 'hazardous';

  /** Pollutant concentrations */
  pollutants: {
    pm25?: number;  // µg/m³
    pm10?: number;  // µg/m³
    o3?: number;    // ppb
    no2?: number;   // ppb
    so2?: number;   // ppb
    co?: number;    // ppm
  };

  /** Dominant pollutant */
  dominantPollutant: string;

  /** Health recommendations */
  healthRecommendations: string[];

  /** Measurement location */
  location: GeoLocation;

  /** Timestamp */
  timestamp: Date;
}

export interface WeatherConditions {
  temperature: number;      // Celsius
  humidity: number;         // Percentage
  pressure: number;         // hPa
  windSpeed: number;        // km/h
  windDirection: number;    // degrees
  precipitation: number;    // mm
  visibility: number;       // km
  uvIndex: number;          // 0-11+
  condition: string;        // clear, cloudy, rain, etc.
  timestamp: Date;
}

export interface NoiseLevel {
  level: number;            // dB
  category: 'quiet' | 'normal' | 'loud' | 'very-loud' | 'hazardous';
  location: GeoLocation;
  duration: number;         // minutes
  source?: string;
  timestamp: Date;
}

// ============================================================================
// Citizen Services Types
// ============================================================================

export type ServiceRequestType =
  | 'pothole'
  | 'streetlight'
  | 'graffiti'
  | 'waste-collection'
  | 'tree-maintenance'
  | 'noise-complaint'
  | 'water-leak'
  | 'parking-violation'
  | 'other';

export type RequestStatus =
  | 'submitted'
  | 'acknowledged'
  | 'in-progress'
  | 'resolved'
  | 'closed'
  | 'rejected';

export interface ServiceRequest {
  /** Request identifier */
  requestId: string;

  /** Request type */
  type: ServiceRequestType;

  /** Current status */
  status: RequestStatus;

  /** Title/summary */
  title: string;

  /** Detailed description */
  description: string;

  /** Location */
  location: GeoLocation;

  /** Submitter information */
  submittedBy: string;

  /** Submission timestamp */
  submittedAt: Date;

  /** Priority */
  priority: 'low' | 'medium' | 'high' | 'urgent';

  /** Assigned department */
  department?: string;

  /** Assigned to */
  assignedTo?: string;

  /** Resolution time estimate */
  estimatedResolution?: Date;

  /** Resolution notes */
  resolution?: string;

  /** Photos/attachments */
  attachments?: string[];

  /** Upvotes from other citizens */
  upvotes?: number;
}

export interface CitizenProfile {
  citizenId: string;
  name: string;
  email: string;
  phone?: string;
  address?: Address;
  preferences: CitizenPreferences;
  verificationStatus: 'verified' | 'unverified';
  registeredAt: Date;
}

export interface CitizenPreferences {
  notifications: boolean;
  alertTypes: EmergencyType[];
  language: string;
  accessibility?: AccessibilityOptions;
}

export interface AccessibilityOptions {
  screenReader: boolean;
  highContrast: boolean;
  largeText: boolean;
  audioAlerts: boolean;
}

// ============================================================================
// Urban Analytics Types
// ============================================================================

export interface CityDashboard {
  /** City identifier */
  cityId: string;

  /** City name */
  cityName: string;

  /** Population */
  population: number;

  /** Area (km²) */
  area: number;

  /** Overall health score (0-100) */
  healthScore: number;

  /** Key metrics */
  metrics: {
    traffic: TrafficMetrics;
    environment: EnvironmentMetrics;
    safety: PublicSafetyMetrics;
    services: ServiceMetrics;
    energy: EnergyMetrics;
  };

  /** Active alerts */
  activeAlerts: number;

  /** Service requests open */
  openServiceRequests: number;

  /** Last updated */
  lastUpdate: Date;
}

export interface TrafficMetrics {
  averageSpeed: number;
  congestionLevel: number;      // 0-100
  publicTransitUsage: number;   // percentage
  parkingAvailability: number;  // percentage
  averageCommute: number;       // minutes
}

export interface EnvironmentMetrics {
  airQualityIndex: number;
  noiseLevel: number;          // dB average
  greenSpaceRatio: number;     // percentage
  wasteRecyclingRate: number;  // percentage
  waterQuality: number;        // 0-100
}

export interface ServiceMetrics {
  averageResponseTime: number; // minutes
  serviceRequestResolutionRate: number; // percentage
  citizenSatisfaction: number; // 0-100
  uptime: number;              // percentage
}

export interface EnergyMetrics {
  totalConsumption: number;    // kWh
  renewablePercentage: number; // percentage
  gridStability: number;       // 0-100
  peakDemand: number;          // kW
  carbonEmissions: number;     // kg CO2
}

// ============================================================================
// Utility Types
// ============================================================================

export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude?: number;
  accuracy?: number;
}

export interface Address {
  street: string;
  city: string;
  state?: string;
  postalCode: string;
  country: string;
}

export interface TimeRange {
  start: Date;
  end: Date;
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface SmartCityConfig {
  /** City identifier */
  cityId: string;

  /** API endpoint */
  apiEndpoint: string;

  /** API key */
  apiKey: string;

  /** Refresh interval (seconds) */
  refreshInterval?: number;

  /** Enable real-time updates */
  realTimeUpdates?: boolean;

  /** Data retention period (days) */
  dataRetention?: number;

  /** Timezone */
  timezone?: string;

  /** Language */
  language?: string;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: Date;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
}

// ============================================================================
// Event Types
// ============================================================================

export type CityEventType =
  | 'sensor-update'
  | 'traffic-change'
  | 'emergency-alert'
  | 'service-request'
  | 'infrastructure-update'
  | 'environmental-alert'
  | 'maintenance-scheduled';

export interface CityEvent {
  eventType: CityEventType;
  eventId: string;
  timestamp: Date;
  data: any;
  severity?: 'info' | 'warning' | 'critical';
}

// ============================================================================
// Export Main Interface
// ============================================================================

export interface ISmartCity {
  // Infrastructure Management
  getInfrastructureStatus(type: InfrastructureType): Promise<any>;
  monitorSensorNetwork(networkId: string): Promise<SensorNetwork>;
  getSensorReading(sensorId: string): Promise<SensorReading>;

  // Traffic Management
  getTrafficConditions(zoneId?: string): Promise<RoadSegment[]>;
  getTrafficSignalStatus(signalId: string): Promise<TrafficSignal>;
  reportTrafficIncident(incident: Partial<TrafficIncident>): Promise<string>;
  getParkingAvailability(location: GeoLocation, radius: number): Promise<ParkingFacility[]>;

  // Public Safety
  getEmergencyAlerts(active?: boolean): Promise<EmergencyAlert[]>;
  reportEmergency(emergency: Partial<EmergencyAlert>): Promise<string>;
  getPublicSafetyMetrics(): Promise<PublicSafetyMetrics>;

  // Environmental Monitoring
  getAirQuality(location?: GeoLocation): Promise<AirQualityIndex>;
  getWeatherConditions(): Promise<WeatherConditions>;
  getNoiseLevel(location: GeoLocation): Promise<NoiseLevel>;

  // Citizen Services
  submitServiceRequest(request: Partial<ServiceRequest>): Promise<string>;
  getServiceRequest(requestId: string): Promise<ServiceRequest>;
  updateServiceRequest(requestId: string, update: Partial<ServiceRequest>): Promise<void>;
  getServiceRequests(status?: RequestStatus): Promise<ServiceRequest[]>;

  // Analytics & Dashboard
  getCityDashboard(): Promise<CityDashboard>;
  getUrbanAnalytics(timeRange: TimeRange): Promise<any>;

  // Event Handling
  on(event: CityEventType, callback: (data: any) => void): void;
  off(event: CityEventType, callback: (data: any) => void): void;
}

/**
 * 弘益人間 (홍익인간)
 * Benefit All Humanity
 *
 * Smart Cities for a Better Tomorrow
 */
