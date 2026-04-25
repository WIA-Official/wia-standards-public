/**
 * WIA-AGRI-034 Hydroponics Standard - TypeScript Type Definitions
 * @version 1.0.0
 * @standard WIA-AGRI-034
 */

export type SystemType = 'nft' | 'dwc' | 'ebb-flow' | 'drip' | 'aeroponics' | 'aquaponics' | 'kratky';
export type CropType = 'lettuce' | 'tomato' | 'cucumber' | 'herbs' | 'strawberry' | 'pepper' | 'other';
export type GrowthStage = 'germination' | 'seedling' | 'vegetative' | 'flowering' | 'fruiting' | 'harvest-ready';
export type NutrientDelivery = 'continuous' | 'intermittent' | 'cyclic' | 'demand-based';

export interface HydroponicFarm {
  farmId: string;
  name: string;
  location: LocationInfo;
  systemType: SystemType;
  crops: CropInfo[];
  nutrientManagement: NutrientManagement;
  waterSystem: WaterSystem;
  environmental: EnvironmentalControl;
  automation: AutomationSystem;
  yield: YieldMetrics;
  sustainability: SustainabilityMetrics;
  createdAt: string;
  updatedAt: string;
  version: string;
  standard: string;
}

export interface LocationInfo {
  address: string;
  city: string;
  country: string;
  coordinates: { latitude: number; longitude: number };
  facilityType: 'greenhouse' | 'indoor' | 'vertical-farm' | 'outdoor';
  area: Measurement;
}

export interface Measurement {
  value: number;
  unit: string;
}

export interface CropInfo {
  cropId: string;
  type: CropType;
  variety: string;
  quantity: number;
  plantingDate: string;
  expectedHarvestDate: string;
  growthStage: GrowthStage;
  health: CropHealth;
  growthMetrics: GrowthMetrics;
  spacing: Measurement;
}

export interface CropHealth {
  overallScore: number; // 0-100
  vigor: number; // 0-100
  diseases: DiseaseInfo[];
  pests: PestInfo[];
  deficiencies: NutrientDeficiency[];
}

export interface DiseaseInfo {
  disease: string;
  severity: 'low' | 'medium' | 'high';
  affectedPlants: number;
  treatment: string;
}

export interface PestInfo {
  pest: string;
  population: 'low' | 'medium' | 'high';
  control: string;
  biologicalControl: boolean;
}

export interface NutrientDeficiency {
  nutrient: string;
  symptoms: string[];
  correctionApplied: boolean;
}

export interface GrowthMetrics {
  height: Measurement;
  leafArea: Measurement;
  stemDiameter?: Measurement;
  fruitCount?: number;
  dailyGrowthRate: Measurement;
  daysToHarvest: number;
}

export interface NutrientManagement {
  solution: NutrientSolution;
  delivery: NutrientDeliverySystem;
  monitoring: NutrientMonitoring;
  adjustments: NutrientAdjustment[];
}

export interface NutrientSolution {
  formulaId: string;
  formulaName: string;
  ec: number; // electrical conductivity (mS/cm)
  pH: number;
  temperature: number; // celsius
  composition: NutrientComposition[];
  mixingDate: string;
  replaceDate: string;
}

export interface NutrientComposition {
  nutrient: string;
  concentration: number; // ppm
  target: number; // ppm
  withinRange: boolean;
}

export interface NutrientDeliverySystem {
  type: NutrientDelivery;
  flowRate: Measurement;
  schedule: string;
  automated: boolean;
  dosing: DosingSystem[];
}

export interface DosingSystem {
  nutrientType: string;
  dosingRate: Measurement;
  stockConcentration: number;
  automated: boolean;
}

export interface NutrientMonitoring {
  realtime: boolean;
  sensors: NutrientSensor[];
  testingFrequency: string;
  labAnalysis: boolean;
}

export interface NutrientSensor {
  sensorId: string;
  type: string;
  value: number;
  unit: string;
  status: 'active' | 'inactive' | 'calibration-needed';
  lastCalibration: string;
}

export interface NutrientAdjustment {
  adjustmentId: string;
  timestamp: string;
  reason: string;
  parameter: string;
  previousValue: number;
  newValue: number;
  outcome: string;
}

export interface WaterSystem {
  source: WaterSource;
  quality: WaterQuality;
  circulation: CirculationSystem;
  treatment: WaterTreatment;
  usage: WaterUsageMetrics;
}

export interface WaterSource {
  type: 'municipal' | 'well' | 'rainwater' | 'recycled' | 'mixed';
  quality: string;
  pretreatment: string[];
}

export interface WaterQuality {
  pH: number;
  ec: number; // mS/cm
  tds: number; // ppm (total dissolved solids)
  alkalinity: number; // ppm CaCO3
  hardness: number; // ppm CaCO3
  chlorine: number; // ppm
  temperature: number; // celsius
  dissolved Oxygen: number; // ppm
}

export interface CirculationSystem {
  type: string;
  flowRate: Measurement;
  volumeInSystem: Measurement;
  circulationRate: number; // cycles per hour
  pumps: PumpInfo[];
}

export interface PumpInfo {
  pumpId: string;
  type: string;
  flowRate: Measurement;
  power: Measurement; // watts
  status: 'active' | 'inactive' | 'maintenance';
  hoursRun: number;
}

export interface WaterTreatment {
  filtration: FiltrationSystem[];
  uv: boolean;
  ozone: boolean;
  additives: string[];
}

export interface FiltrationSystem {
  type: string;
  poreSize?: Measurement;
  flowRate: Measurement;
  lastReplacement: string;
  nextReplacement: string;
}

export interface WaterUsageMetrics {
  dailyConsumption: Measurement;
  recyclingRate: number; // percentage
  evaporationLoss: Measurement;
  wasteWater: Measurement;
  efficiency: number; // percentage
}

export interface EnvironmentalControl {
  temperature: TemperatureControl;
  humidity: HumidityControl;
  lighting: LightingSystem;
  co2: CO2Management;
  airflow: AirflowSystem;
}

export interface TemperatureControl {
  current: number; // celsius
  target: number;
  range: { min: number; max: number };
  dayTarget: number;
  nightTarget: number;
  controlMethod: string;
  heating: boolean;
  cooling: boolean;
}

export interface HumidityControl {
  current: number; // percentage
  target: number;
  range: { min: number; max: number };
  controlMethod: string;
  humidification: boolean;
  dehumidification: boolean;
  vpd: number; // kPa (Vapor Pressure Deficit)
}

export interface LightingSystem {
  type: 'natural' | 'led' | 'hps' | 'fluorescent' | 'hybrid';
  intensity: Measurement; // PAR or lumens
  spectrum: string[];
  photoperiod: number; // hours
  dli: number; // Daily Light Integral (mol/m²/day)
  automated: boolean;
  energyEfficiency: number; // lumens per watt
}

export interface CO2Management {
  current: number; // ppm
  target: number;
  enrichment: boolean;
  source: string;
  controlMethod: string;
}

export interface AirflowSystem {
  ventilation: boolean;
  exhaustFans: number;
  circulationFans: number;
  airExchangeRate: number; // per hour
  automated: boolean;
}

export interface AutomationSystem {
  level: 'manual' | 'semi-automated' | 'fully-automated' | 'ai-controlled';
  controllers: ControllerInfo[];
  sensors: SensorDeployment[];
  actuators: ActuatorInfo[];
  monitoring: MonitoringDashboard;
  alerts: AlertSystem;
}

export interface ControllerInfo {
  controllerId: string;
  type: string;
  functions: string[];
  connectivity: string;
  firmware: string;
}

export interface SensorDeployment {
  sensorType: string;
  count: number;
  locations: string[];
  updateFrequency: string;
}

export interface ActuatorInfo {
  actuatorId: string;
  type: string;
  controlledParameter: string;
  status: 'active' | 'inactive' | 'fault';
}

export interface MonitoringDashboard {
  realtime: boolean;
  dataVisualization: boolean;
  historicalData: boolean;
  remoteAccess: boolean;
  mobileApp: boolean;
}

export interface AlertSystem {
  enabled: boolean;
  thresholds: AlertThreshold[];
  notifications: NotificationChannel[];
  escalation: boolean;
}

export interface AlertThreshold {
  parameter: string;
  min?: number;
  max?: number;
  severity: 'info' | 'warning' | 'critical';
}

export interface NotificationChannel {
  type: 'email' | 'sms' | 'push' | 'webhook';
  enabled: boolean;
  recipients: string[];
}

export interface YieldMetrics {
  totalYield: Measurement;
  yieldPerArea: Measurement;
  yieldPerPlant: Measurement;
  quality: QualityGrade[];
  cropCycles: number; // per year
  harvestFrequency: string;
}

export interface QualityGrade {
  grade: string;
  percentage: number;
  criteria: string;
}

export interface SustainabilityMetrics {
  waterEfficiency: WaterEfficiency;
  energyEfficiency: EnergyEfficiency;
  resourceUtilization: ResourceUtilization;
  environmentalImpact: EnvironmentalImpact;
  economicMetrics: EconomicMetrics;
}

export interface WaterEfficiency {
  waterUseEfficiency: number; // L per kg yield
  recyclingRate: number; // percentage
  comparisonToSoil: number; // percentage reduction
}

export interface EnergyEfficiency {
  totalEnergyUse: Measurement; // kWh per day
  energyPerKg: Measurement; // kWh per kg yield
  renewableEnergy: number; // percentage
  ledEfficiency: number; // percentage of total lighting
}

export interface ResourceUtilization {
  nutrientEfficiency: number; // percentage
  spaceUtilization: number; // kg per m²
  yearRoundProduction: boolean;
  cropDiversity: number; // number of crop types
}

export interface EnvironmentalImpact {
  carbonFootprint: Measurement; // kg CO2 per kg yield
  pesticideUse: string;
  localProduction: boolean;
  transportationReduction: number; // percentage
}

export interface EconomicMetrics {
  setupCost: number;
  operatingCost: number; // per month
  revenue: number; // per month
  profit: number;
  currency: string;
  roi: number; // percentage
  paybackPeriod: number; // months
}

export interface ClientConfig {
  apiKey?: string;
  baseURL: string;
  timeout?: number;
  retryAttempts?: number;
}

export interface AuthTokens {
  accessToken: string;
  refreshToken?: string;
  expiresAt: number;
}

export class WIAHydroponicsError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'WIAHydroponicsError';
  }
}
