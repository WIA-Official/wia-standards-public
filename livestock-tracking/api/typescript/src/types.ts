/**
 * WIA-AGRI-008 Livestock Tracking Standard - TypeScript Type Definitions
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

/**
 * WIA Livestock Tracking Standard Version
 */
export type WIAVersion = '1.0' | '1.1' | '2.0';

/**
 * Animal Types
 */
export type AnimalType = 'cattle' | 'sheep' | 'goat' | 'pig' | 'chicken' | 'horse' | 'other';

/**
 * Animal Status
 */
export type AnimalStatus = 'healthy' | 'sick' | 'pregnant' | 'lactating' | 'quarantine' | 'deceased';

/**
 * Tracking Device Type
 */
export type DeviceType = 'ear_tag' | 'collar' | 'implant' | 'leg_band' | 'bolus';

/**
 * Activity Level
 */
export type ActivityLevel = 'resting' | 'low' | 'moderate' | 'high' | 'abnormal';

/**
 * Livestock Tracking Configuration
 */
export interface LivestockTrackingConfig {
  animalId: string;
  farmId?: string;
  identification: AnimalIdentification;
  type: AnimalType;
  breed?: string;
  birthDate?: string;
  gender: 'male' | 'female';
  status: AnimalStatus;
  location?: Location;
  device?: TrackingDevice;
  parentIds?: {
    sire?: string;
    dam?: string;
  };
  metadata?: Record<string, any>;
}

/**
 * Animal Identification
 */
export interface AnimalIdentification {
  officialId?: string; // e.g., RFID, national ID
  farmId: string; // Farm-specific ID
  visualId?: string; // Visual tag number
  name?: string;
  microchipId?: string;
  tattoo?: string;
}

/**
 * Tracking Device
 */
export interface TrackingDevice {
  deviceId: string;
  type: DeviceType;
  manufacturer?: string;
  model?: string;
  batteryLevel?: number; // percentage
  status: 'active' | 'inactive' | 'error';
  installDate?: string;
  lastSeen?: string;
  sensors?: SensorType[];
}

/**
 * Sensor Types
 */
export type SensorType =
  | 'gps'
  | 'temperature'
  | 'heart_rate'
  | 'accelerometer'
  | 'rumination'
  | 'activity';

/**
 * Geographic Location
 */
export interface Location {
  latitude: number;
  longitude: number;
  altitude?: number;
  accuracy?: number; // meters
  timestamp?: string;
  zone?: string;
  pasture?: string;
}

/**
 * Location History
 */
export interface LocationHistory {
  animalId: string;
  locations: LocationPoint[];
  startTime: string;
  endTime: string;
  distance?: number; // meters
  averageSpeed?: number; // m/s
}

/**
 * Location Point
 */
export interface LocationPoint {
  timestamp: string;
  location: Location;
  speed?: number;
  heading?: number;
}

/**
 * Health Record
 */
export interface HealthRecord {
  recordId: string;
  animalId: string;
  date: string;
  type: 'checkup' | 'vaccination' | 'treatment' | 'illness' | 'injury';
  diagnosis?: string;
  treatment?: string;
  medication?: MedicationRecord;
  veterinarian?: string;
  temperature?: number;
  weight?: number;
  notes?: string;
  followUpDate?: string;
}

/**
 * Medication Record
 */
export interface MedicationRecord {
  name: string;
  dosage: number;
  unit: string;
  frequency?: string;
  duration?: number; // days
  withdrawalPeriod?: number; // days
  administeredBy?: string;
}

/**
 * Weight Record
 */
export interface WeightRecord {
  recordId: string;
  animalId: string;
  timestamp: string;
  weight: number; // kg
  unit: string;
  method?: 'scale' | 'estimated' | 'calculated';
  bodyConditionScore?: number; // 1-5 or 1-9
  notes?: string;
}

/**
 * Activity Data
 */
export interface ActivityData {
  dataId: string;
  animalId: string;
  timestamp: string;
  activityLevel: ActivityLevel;
  steps?: number;
  distance?: number; // meters
  restingTime?: number; // minutes
  activeTime?: number; // minutes
  rumination?: RuminationData;
}

/**
 * Rumination Data
 */
export interface RuminationData {
  duration: number; // minutes
  chewsPerMinute?: number;
  quality?: 'poor' | 'fair' | 'good';
}

/**
 * Vital Signs
 */
export interface VitalSigns {
  recordId: string;
  animalId: string;
  timestamp: string;
  temperature?: number; // Celsius
  heartRate?: number; // bpm
  respirationRate?: number; // breaths per minute
  bloodPressure?: {
    systolic: number;
    diastolic: number;
  };
  oxygenSaturation?: number; // percentage
}

/**
 * Reproduction Record
 */
export interface ReproductionRecord {
  recordId: string;
  animalId: string;
  type: 'heat' | 'breeding' | 'pregnancy_check' | 'calving' | 'abortion';
  date: string;
  matingDate?: string;
  expectedDueDate?: string;
  actualBirthDate?: string;
  offspring?: OffspringRecord[];
  complications?: string;
  notes?: string;
}

/**
 * Offspring Record
 */
export interface OffspringRecord {
  animalId?: string;
  gender: 'male' | 'female';
  weight?: number; // kg at birth
  status: 'alive' | 'deceased' | 'stillborn';
  vigor?: 'strong' | 'weak';
}

/**
 * Feeding Record
 */
export interface FeedingRecord {
  recordId: string;
  animalId?: string;
  groupId?: string;
  timestamp: string;
  feedType: string;
  quantity: number; // kg
  supplementation?: Supplement[];
  feedingMethod?: string;
  notes?: string;
}

/**
 * Supplement
 */
export interface Supplement {
  name: string;
  amount: number;
  unit: string;
}

/**
 * Milk Production Record
 */
export interface MilkProductionRecord {
  recordId: string;
  animalId: string;
  date: string;
  milkingTime: 'morning' | 'afternoon' | 'evening';
  quantity: number; // liters
  quality?: MilkQuality;
  notes?: string;
}

/**
 * Milk Quality
 */
export interface MilkQuality {
  fat?: number; // percentage
  protein?: number; // percentage
  lactose?: number; // percentage
  scc?: number; // Somatic Cell Count
  bacteria?: number; // CFU/ml
  temperature?: number;
}

/**
 * Alert
 */
export interface LivestockAlert {
  alertId: string;
  animalId: string;
  timestamp: string;
  type: 'health' | 'location' | 'activity' | 'reproduction' | 'device';
  severity: 'info' | 'warning' | 'critical';
  message: string;
  recommendation?: string;
  acknowledged?: boolean;
  acknowledgedAt?: string;
  resolvedAt?: string;
}

/**
 * Herd/Group
 */
export interface Herd {
  herdId: string;
  farmId: string;
  name: string;
  type: AnimalType;
  animalIds: string[];
  location?: string;
  purpose?: 'breeding' | 'dairy' | 'meat' | 'mixed';
  metadata?: Record<string, any>;
}

/**
 * Movement Record
 */
export interface MovementRecord {
  recordId: string;
  animalIds: string[];
  fromLocation: string;
  toLocation: string;
  movementDate: string;
  reason?: string;
  transportMethod?: string;
  distance?: number; // km
  operator?: string;
  documents?: string[]; // Document URLs
}

/**
 * Slaughter Record
 */
export interface SlaughterRecord {
  recordId: string;
  animalId: string;
  slaughterDate: string;
  facility?: string;
  liveWeight?: number;
  carcassWeight?: number;
  grade?: string;
  condemnation?: boolean;
  condemnationReason?: string;
}

/**
 * Analytics Report
 */
export interface AnalyticsReport {
  reportId: string;
  farmId?: string;
  herdId?: string;
  period: string;
  generatedDate: string;
  metrics: HerdMetrics;
  trends?: TrendData[];
  insights?: string[];
}

/**
 * Herd Metrics
 */
export interface HerdMetrics {
  totalAnimals: number;
  healthyAnimals: number;
  sickAnimals: number;
  averageWeight?: number;
  averageDailyGain?: number;
  mortalityRate?: number;
  birthRate?: number;
  milkProduction?: number; // total liters
  feedConversionRatio?: number;
}

/**
 * Trend Data
 */
export interface TrendData {
  metric: string;
  values: number[];
  timestamps: string[];
  trend: 'increasing' | 'stable' | 'decreasing';
}

/**
 * Geofence
 */
export interface Geofence {
  fenceId: string;
  name: string;
  farmId?: string;
  type: 'pasture' | 'restricted' | 'alert_zone';
  geometry: GeoJSON;
  alertOnEntry?: boolean;
  alertOnExit?: boolean;
  active: boolean;
}

/**
 * GeoJSON Geometry
 */
export interface GeoJSON {
  type: 'Polygon' | 'MultiPolygon';
  coordinates: number[][][] | number[][][][];
}

/**
 * SDK Configuration
 */
export interface SDKConfig {
  baseURL?: string;
  apiKey?: string;
  farmId?: string;
  timeout?: number;
}

/**
 * List Query Parameters
 */
export interface ListParams {
  limit?: number;
  offset?: number;
  startDate?: string;
  endDate?: string;
  animalType?: string;
  status?: string;
  herdId?: string;
}

/**
 * Error Response
 */
export interface ErrorResponse {
  error: {
    code: string;
    message: string;
    details?: any;
  };
}
