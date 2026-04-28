/**
 * WIA Zero Energy Building Types
 *
 * Comprehensive types for net-zero energy building standards,
 * energy efficiency monitoring, and certification assessment.
 *
 * @module @wia/zero-energy-building/types
 */

/**
 * Energy production source types
 */
export enum EnergySourceType {
  SOLAR_PV = 'solar_pv',
  SOLAR_THERMAL = 'solar_thermal',
  WIND = 'wind',
  GEOTHERMAL = 'geothermal',
  BIOMASS = 'biomass',
  HYDRO = 'hydro',
  BATTERY_STORAGE = 'battery_storage',
  GRID = 'grid'
}

/**
 * Energy consumption category
 */
export enum ConsumptionCategory {
  HEATING = 'heating',
  COOLING = 'cooling',
  VENTILATION = 'ventilation',
  LIGHTING = 'lighting',
  APPLIANCES = 'appliances',
  HOT_WATER = 'hot_water',
  PLUG_LOADS = 'plug_loads',
  OTHER = 'other'
}

/**
 * Building insulation zones
 */
export enum InsulationZone {
  ROOF = 'roof',
  WALLS = 'walls',
  FOUNDATION = 'foundation',
  WINDOWS = 'windows',
  DOORS = 'doors',
  FLOOR = 'floor'
}

/**
 * Net-zero certification levels
 */
export enum CertificationLevel {
  PLATINUM = 'platinum',
  GOLD = 'gold',
  SILVER = 'silver',
  BRONZE = 'bronze',
  NOT_CERTIFIED = 'not_certified'
}

/**
 * Energy production data
 */
export interface EnergyProduction {
  sourceType: EnergySourceType;
  capacity: number; // kW
  currentOutput: number; // kW
  dailyProduction: number; // kWh
  monthlyProduction: number; // kWh
  yearlyProduction: number; // kWh
  efficiency: number; // percentage 0-100
  timestamp: Date;
  metadata?: Record<string, any>;
}

/**
 * Solar panel system configuration
 */
export interface SolarPVSystem {
  panelCount: number;
  panelWattage: number; // Watts per panel
  totalCapacity: number; // kW
  orientation: string; // e.g., "south", "southeast"
  tiltAngle: number; // degrees
  inverterEfficiency: number; // percentage
  shadingFactor: number; // 0-1, 1 = no shading
  installationDate: Date;
  expectedLifespan: number; // years
}

/**
 * Wind turbine configuration
 */
export interface WindTurbineSystem {
  turbineCount: number;
  ratedPower: number; // kW per turbine
  cutInSpeed: number; // m/s
  ratedSpeed: number; // m/s
  cutOutSpeed: number; // m/s
  hubHeight: number; // meters
  rotorDiameter: number; // meters
  installationDate: Date;
}

/**
 * Energy consumption tracking
 */
export interface EnergyConsumption {
  category: ConsumptionCategory;
  currentPower: number; // kW
  dailyConsumption: number; // kWh
  monthlyConsumption: number; // kWh
  yearlyConsumption: number; // kWh
  peakDemand: number; // kW
  timestamp: Date;
  metadata?: Record<string, any>;
}

/**
 * Insulation specifications
 */
export interface InsulationSpec {
  zone: InsulationZone;
  rValue: number; // thermal resistance (ft²·°F·h/BTU or m²·K/W)
  material: string;
  thickness: number; // inches or cm
  area: number; // square feet or m²
  installationDate: Date;
  condition: 'excellent' | 'good' | 'fair' | 'poor';
}

/**
 * HVAC system specifications
 */
export interface HVACSystem {
  type: 'heat_pump' | 'furnace' | 'boiler' | 'geothermal' | 'hybrid';
  heatingCapacity: number; // BTU/h or kW
  coolingCapacity: number; // BTU/h or kW
  seer: number; // Seasonal Energy Efficiency Ratio
  hspf: number; // Heating Seasonal Performance Factor
  afue?: number; // Annual Fuel Utilization Efficiency
  cop?: number; // Coefficient of Performance
  installationDate: Date;
  maintenanceSchedule: string;
  smartControls: boolean;
}

/**
 * Energy balance calculation
 */
export interface EnergyBalance {
  totalProduction: number; // kWh
  totalConsumption: number; // kWh
  netBalance: number; // kWh (positive = surplus, negative = deficit)
  gridExport: number; // kWh
  gridImport: number; // kWh
  selfConsumptionRate: number; // percentage
  selfSufficiencyRate: number; // percentage
  period: 'daily' | 'monthly' | 'yearly';
  timestamp: Date;
}

/**
 * Net-zero certification criteria
 */
export interface CertificationCriteria {
  minimumSelfSufficiency: number; // percentage
  maximumGridImport: number; // kWh/year
  minimumRenewablePercentage: number; // percentage
  requiredInsulationStandard: number; // minimum R-value
  hvacEfficiencyMinimum: number; // minimum SEER/HSPF
  airLeakageMaximum: number; // ACH50
  requiredMonitoringPeriod: number; // months
}

/**
 * Certification assessment result
 */
export interface CertificationAssessment {
  level: CertificationLevel;
  score: number; // 0-100
  meetsMinimumRequirements: boolean;
  selfSufficiencyRate: number; // percentage
  renewablePercentage: number; // percentage
  energyIntensity: number; // kWh/m²/year
  carbonFootprint: number; // kg CO2/year
  recommendations: string[];
  assessmentDate: Date;
  validUntil: Date;
}

/**
 * Optimization recommendation
 */
export interface OptimizationRecommendation {
  category: string;
  priority: 'critical' | 'high' | 'medium' | 'low';
  title: string;
  description: string;
  estimatedSavings: number; // kWh/year
  estimatedCost: number; // currency
  paybackPeriod: number; // years
  implementationComplexity: 'simple' | 'moderate' | 'complex';
  carbonReduction: number; // kg CO2/year
}

/**
 * Building metadata
 */
export interface BuildingMetadata {
  id: string;
  name: string;
  type: 'residential' | 'commercial' | 'industrial' | 'mixed';
  location: {
    address: string;
    latitude: number;
    longitude: number;
    climateZone: string;
  };
  size: {
    floorArea: number; // m² or ft²
    volume: number; // m³ or ft³
    stories: number;
  };
  occupancy: number;
  yearBuilt: number;
  yearRetrofitted?: number;
}

/**
 * Event types for zero energy building monitoring
 */
export enum ZEBEventType {
  ENERGY_SURPLUS = 'energy_surplus',
  ENERGY_DEFICIT = 'energy_deficit',
  PRODUCTION_ANOMALY = 'production_anomaly',
  CONSUMPTION_SPIKE = 'consumption_spike',
  SYSTEM_MAINTENANCE = 'system_maintenance',
  CERTIFICATION_RENEWAL = 'certification_renewal',
  OPTIMIZATION_AVAILABLE = 'optimization_available'
}

/**
 * Event data structure
 */
export interface ZEBEvent {
  type: ZEBEventType;
  timestamp: Date;
  severity: 'info' | 'warning' | 'critical';
  message: string;
  data?: Record<string, any>;
}

/**
 * Event handler callback
 */
export type ZEBEventHandler = (event: ZEBEvent) => void | Promise<void>;

/**
 * Configuration options for Zero Energy Building SDK
 */
export interface ZEBConfig {
  buildingId: string;
  apiKey?: string;
  monitoringInterval?: number; // seconds
  enableRealTimeMonitoring?: boolean;
  certificationStandard?: string;
  units?: 'metric' | 'imperial';
}
