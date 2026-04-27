/**
 * WIA Smart Aquaculture Standard - TypeScript Types
 * @version 1.0.0
 * @license MIT
 */

/**
 * Farm types
 */
export type FarmType = 'marine' | 'freshwater' | 'brackish' | 'recirculating';

/**
 * Water body types
 */
export type WaterBody = 'ocean' | 'sea' | 'lake' | 'river' | 'pond';

/**
 * Sensor types
 */
export type SensorType = 'multiparameter' | 'dissolved_oxygen' | 'temperature' | 'pH';

/**
 * Status levels
 */
export type Status = 'normal' | 'warning' | 'critical';

/**
 * Alert severity
 */
export type AlertSeverity = 'info' | 'warning' | 'critical';

/**
 * Buffer capacity
 */
export type BufferCapacity = 'low' | 'medium' | 'high';

/**
 * Risk levels
 */
export type RiskLevel = 'low' | 'medium' | 'high';

/**
 * Feeding response
 */
export type FeedingResponse = 'excellent' | 'good' | 'fair' | 'poor';

/**
 * Stress level
 */
export type StressLevel = 'low' | 'medium' | 'high';

/**
 * Farm location
 */
export interface FarmLocation {
  latitude: number;
  longitude: number;
  waterBody: WaterBody;
  depth: number;
  address: string;
  country: string;
  timezone: string;
}

/**
 * Farm capacity
 */
export interface FarmCapacity {
  totalVolume: number;
  tankCount: number;
  cageCount: number;
  pondCount: number;
  maxBiomass: number;
}

/**
 * Farm operator
 */
export interface FarmOperator {
  name: string;
  contact: string;
  did?: string;
  license: string;
}

/**
 * Aquaculture farm entity
 */
export interface AquacultureFarm {
  farmId: string;
  farmName: string;
  farmType: FarmType;
  location: FarmLocation;
  capacity: FarmCapacity;
  certifications: string[];
  establishedDate: string;
  operator: FarmOperator;
}

/**
 * Measurement with status
 */
export interface MeasurementWithStatus {
  value: number;
  unit: string;
  status: Status;
}

/**
 * Temperature measurement
 */
export interface TemperatureMeasurement extends MeasurementWithStatus {
  unit: '°C';
}

/**
 * Salinity measurement
 */
export interface SalinityMeasurement extends MeasurementWithStatus {
  unit: 'ppt';
}

/**
 * Dissolved oxygen measurement
 */
export interface DissolvedOxygenMeasurement extends MeasurementWithStatus {
  unit: 'mg/L';
  saturation: number;
}

/**
 * pH measurement
 */
export interface pHMeasurement {
  value: number;
  status: Status;
}

/**
 * Ammonia measurement
 */
export interface AmmoniaMeasurement {
  nh3: number;
  nh4: number;
  totalAmmonia: number;
  status: Status;
}

/**
 * Nitrite measurement
 */
export interface NitriteMeasurement extends MeasurementWithStatus {
  unit: 'mg/L';
}

/**
 * Nitrate measurement
 */
export interface NitrateMeasurement extends MeasurementWithStatus {
  unit: 'mg/L';
}

/**
 * Turbidity measurement
 */
export interface TurbidityMeasurement {
  value: number;
  unit: 'NTU';
  visibility: number;
}

/**
 * Alkalinity measurement
 */
export interface AlkalinityMeasurement {
  value: number;
  unit: 'mg/L';
  bufferCapacity: BufferCapacity;
}

/**
 * Chlorophyll measurement
 */
export interface ChlorophyllMeasurement {
  value: number;
  unit: 'μg/L';
  algaeBloomRisk: RiskLevel;
}

/**
 * Water quality measurements
 */
export interface WaterQualityMeasurements {
  temperature: TemperatureMeasurement;
  salinity: SalinityMeasurement;
  dissolvedOxygen: DissolvedOxygenMeasurement;
  pH: pHMeasurement;
  ammonia: AmmoniaMeasurement;
  nitrite: NitriteMeasurement;
  nitrate: NitrateMeasurement;
  turbidity: TurbidityMeasurement;
  alkalinity: AlkalinityMeasurement;
  chlorophyll: ChlorophyllMeasurement;
}

/**
 * Sensor alert
 */
export interface SensorAlert {
  parameter: string;
  severity: AlertSeverity;
  message: string;
  threshold: number;
  currentValue: number;
}

/**
 * Water quality sensor data
 */
export interface WaterQualitySensorData {
  sensorId: string;
  farmId: string;
  tankId: string;
  timestamp: string;
  sensorType: SensorType;
  depth: number;
  measurements: WaterQualityMeasurements;
  alerts: SensorAlert[];
}

/**
 * Species information
 */
export interface SpeciesInfo {
  scientificName: string;
  commonName: string;
  geneticLineage: string;
}

/**
 * Population information
 */
export interface PopulationInfo {
  totalCount: number;
  density: number;
  stockingDate: string;
  daysInCulture: number;
}

/**
 * Weight distribution
 */
export interface WeightDistribution {
  min: number;
  max: number;
  median: number;
  stdDev: number;
}

/**
 * Biomass data
 */
export interface BiomassData {
  total: number;
  averageWeight: number;
  weightDistribution: WeightDistribution;
}

/**
 * Growth data
 */
export interface GrowthData {
  dailyGrowthRate: number;
  specificGrowthRate: number;
  projectedHarvestWeight: number;
  projectedHarvestDate: string;
}

/**
 * Health data
 */
export interface HealthData {
  mortalityRate: number;
  diseaseIncidence: number;
  parasiteLoad: string;
  abnormalBehavior: number;
  feedingResponse: FeedingResponse;
  bodyConditionFactor: number;
  stressLevel: StressLevel;
}

/**
 * Vaccination record
 */
export interface VaccinationRecord {
  disease: string;
  date: string;
  vaccine: string;
  effectiveness: number;
}

/**
 * Treatment record
 */
export interface TreatmentRecord {
  condition: string;
  treatment: string;
  startDate: string;
  endDate: string;
  dosage: string;
  withdrawalPeriod: number;
}

/**
 * Fish health and biomass data
 */
export interface FishHealthBiomass {
  batchId: string;
  farmId: string;
  tankId: string;
  timestamp: string;
  species: SpeciesInfo;
  population: PopulationInfo;
  biomass: BiomassData;
  growth: GrowthData;
  health: HealthData;
  vaccinations: VaccinationRecord[];
  treatments: TreatmentRecord[];
}

/**
 * Feeding schedule
 */
export interface FeedingSchedule {
  feedingTimes: string[];
  portionSize: number;
  feedType: string;
  frequency: number;
}

/**
 * Feed composition
 */
export interface FeedComposition {
  protein: number;
  fat: number;
  carbohydrate: number;
  fiber: number;
  moisture: number;
}

/**
 * Feeding system data
 */
export interface FeedingSystemData {
  feedingId: string;
  farmId: string;
  tankId: string;
  timestamp: string;
  schedule: FeedingSchedule;
  composition: FeedComposition;
  feedConversionRatio: number;
  totalFeedUsed: number;
}

/**
 * Harvest record
 */
export interface HarvestRecord {
  harvestId: string;
  farmId: string;
  batchId: string;
  harvestDate: string;
  totalWeight: number;
  averageWeight: number;
  grade: string;
  destination: string;
}

/**
 * API Response wrapper
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
  timestamp: string;
}

/**
 * Query parameters for aquaculture data
 */
export interface AquacultureQuery {
  farmId?: string;
  tankId?: string;
  batchId?: string;
  startDate?: string;
  endDate?: string;
  limit?: number;
  offset?: number;
}
