/**
 * WIA-AGRI-027 Aeroponics Standard - TypeScript Type Definitions
 * @module @wia/aeroponics/types
 */

// ============================================================================
// System Types
// ============================================================================

export interface AeroponicsSystem {
  systemId: string;
  name: string;
  location: Location;
  type: 'vertical' | 'horizontal' | 'tower' | 'hybrid';
  capacity: number; // number of plants
  status: SystemStatus;
  installDate: string;
  lastMaintenance?: string;
  manufacturer?: string;
}

export type SystemStatus = 'active' | 'idle' | 'maintenance' | 'error' | 'offline';

export interface Location {
  latitude: number;
  longitude: number;
  altitude?: number;
  address?: string;
  facility?: string;
}

// ============================================================================
// Misting System Types
// ============================================================================

export interface MistingSystem {
  systemId: string;
  nozzles: Nozzle[];
  pump: Pump;
  cycleInterval: number; // seconds
  mistDuration: number; // seconds
  pressure: number; // PSI
  dropletSize: number; // microns
  status: 'running' | 'stopped' | 'error';
}

export interface Nozzle {
  nozzleId: string;
  position: { x: number; y: number; z: number };
  flowRate: number; // mL/min
  status: 'operational' | 'clogged' | 'damaged';
  lastCleaned?: string;
}

export interface Pump {
  pumpId: string;
  type: 'centrifugal' | 'diaphragm' | 'piston';
  power: number; // watts
  maxPressure: number; // PSI
  currentPressure: number; // PSI
  flowRate: number; // L/min
  status: 'on' | 'off' | 'error';
  runningHours: number;
}

// ============================================================================
// Nutrient Solution Types
// ============================================================================

export interface NutrientSolution {
  solutionId: string;
  timestamp: number;
  pH: number;
  ec: number; // electrical conductivity (mS/cm)
  temperature: number; // Celsius
  dissolvedOxygen: number; // mg/L
  nutrients: NutrientLevels;
  volume: number; // liters
}

export interface NutrientLevels {
  nitrogen: number; // ppm
  phosphorus: number; // ppm
  potassium: number; // ppm
  calcium: number; // ppm
  magnesium: number; // ppm
  sulfur: number; // ppm
  iron: number; // ppm
  manganese?: number; // ppm
  zinc?: number; // ppm
  copper?: number; // ppm
  boron?: number; // ppm
  molybdenum?: number; // ppm
}

export interface NutrientRecipe {
  recipeId: string;
  name: string;
  cropType: string;
  growthStage: GrowthStage;
  targetPH: number;
  targetEC: number;
  nutrients: NutrientLevels;
  description?: string;
}

export type GrowthStage = 'seedling' | 'vegetative' | 'flowering' | 'fruiting' | 'harvest';

// ============================================================================
// Plant & Crop Types
// ============================================================================

export interface Plant {
  plantId: string;
  systemId: string;
  species: string;
  variety: string;
  plantDate: string;
  growthStage: GrowthStage;
  position: { x: number; y: number; z: number };
  rootHealth: 'excellent' | 'good' | 'fair' | 'poor' | 'diseased';
  height?: number; // cm
  leafCount?: number;
  expectedHarvestDate?: string;
  health: PlantHealth;
}

export interface PlantHealth {
  overallScore: number; // 0-100
  rootColor: string;
  rootLength: number; // cm
  leafColor: string;
  diseaseDetected: boolean;
  diseases?: string[];
  pestDetected: boolean;
  pests?: string[];
  stressIndicators?: string[];
}

export interface CropYield {
  harvestId: string;
  systemId: string;
  cropType: string;
  harvestDate: string;
  quantity: number; // kg
  quality: 'premium' | 'grade_a' | 'grade_b' | 'grade_c';
  marketValue?: number;
  notes?: string;
}

// ============================================================================
// Environmental Monitoring Types
// ============================================================================

export interface EnvironmentalData {
  timestamp: number;
  systemId: string;
  airTemperature: number; // Celsius
  airHumidity: number; // percentage
  co2Level: number; // ppm
  lightIntensity: number; // lux or PPFD
  photoperiod: number; // hours
  airflow: number; // m/s
}

export interface ClimateControl {
  systemId: string;
  targetTemperature: number; // Celsius
  targetHumidity: number; // percentage
  targetCO2: number; // ppm
  lightingSchedule: LightingSchedule;
  ventilation: VentilationConfig;
  hvac: HVACConfig;
}

export interface LightingSchedule {
  type: 'led' | 'hps' | 'fluorescent' | 'natural';
  spectrum: string;
  intensity: number; // percentage
  onTime: string; // HH:mm
  offTime: string; // HH:mm
  photoperiod: number; // hours
}

export interface VentilationConfig {
  fanSpeed: number; // percentage
  airChangesPerHour: number;
  co2Injection: boolean;
  co2FlowRate?: number; // L/min
}

export interface HVACConfig {
  heatingEnabled: boolean;
  coolingEnabled: boolean;
  dehumidifierEnabled: boolean;
  humidifierEnabled: boolean;
  targetTemperature: number;
  targetHumidity: number;
}

// ============================================================================
// Monitoring & Alerts
// ============================================================================

export type AlertLevel = 'info' | 'warning' | 'error' | 'critical';
export type AlertType = 'system' | 'nutrient' | 'environment' | 'plant_health' | 'equipment';

export interface Alert {
  alertId: string;
  timestamp: number;
  systemId: string;
  level: AlertLevel;
  type: AlertType;
  message: string;
  data?: Record<string, any>;
  acknowledged: boolean;
  resolvedAt?: number;
}

export interface SensorReading {
  sensorId: string;
  systemId: string;
  sensorType: string;
  value: number;
  unit: string;
  timestamp: number;
  calibratedAt?: number;
}

// ============================================================================
// Maintenance & Operations
// ============================================================================

export interface MaintenanceLog {
  logId: string;
  systemId: string;
  timestamp: number;
  type: 'routine' | 'repair' | 'calibration' | 'cleaning' | 'inspection';
  description: string;
  performedBy: string;
  partsReplaced?: string[];
  cost?: number;
  nextScheduled?: number;
}

export interface SystemMetrics {
  systemId: string;
  period: { start: number; end: number };
  waterUsage: number; // liters
  nutrientUsage: Record<string, number>; // nutrient name -> kg
  powerConsumption: number; // kWh
  cropYield: number; // kg
  growthRate: number; // cm/day average
  systemUptime: number; // percentage
  alertsTriggered: number;
}

// ============================================================================
// Client Configuration
// ============================================================================

export interface ClientConfig {
  apiKey: string;
  apiSecret?: string;
  baseURL?: string;
  timeout?: number;
  retryAttempts?: number;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: number;
}

export interface PaginatedResponse<T> {
  data: T[];
  total: number;
  page: number;
  pageSize: number;
  hasMore: boolean;
}

// ============================================================================
// Export all types
// ============================================================================

export * from './types';
