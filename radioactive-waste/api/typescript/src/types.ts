/**
 * WIA-ENE-026: Radioactive Waste Management Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license CC BY 4.0
 * @description Type definitions for radioactive waste tracking, radiation monitoring, and storage management
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

// ==================== Waste Classification Codes ====================

/**
 * Low-Level Waste codes (LLW)
 */
export type LLWCode =
  | 'LLW-1' // Very Low Level (< 100 Bq/g, < 5 years)
  | 'LLW-2' // Low Level A (100-10,000 Bq/g, 5-30 years)
  | 'LLW-3' // Low Level B (10,000-100,000 Bq/g, 30-100 years)
  | 'LLW-4'; // Low Level C (> 100,000 Bq/g, > 100 years)

/**
 * Intermediate-Level Waste codes (ILW)
 */
export type ILWCode =
  | 'ILW-1' // Short half-life (10⁵-10⁸ Bq/g, < 30 years)
  | 'ILW-2' // Long half-life (10⁸-10¹¹ Bq/g, > 30 years)
  | 'ILW-3'; // Alpha-bearing (> 400 Bq/g alpha, thousands of years)

/**
 * High-Level Waste codes (HLW)
 */
export type HLWCode =
  | 'HLW-1' // Spent Nuclear Fuel (10¹⁴-10¹⁶ Bq/kg)
  | 'HLW-2' // Vitrified waste (10¹²-10¹⁴ Bq/kg)
  | 'HLW-3'; // Reprocessing waste (10¹¹-10¹³ Bq/kg)

/**
 * Special Radioactive Waste codes (SRW)
 */
export type SRWCode =
  | 'SRW-1' // Transuranic waste (TRU: Pu, Am, Np)
  | 'SRW-2' // Mixed waste (radioactive + hazardous chemical)
  | 'SRW-3' // Sealed sources (Co-60, Cs-137)
  | 'SRW-4'; // Tritium waste (H-3)

/**
 * All waste classification codes
 */
export type WasteClass = LLWCode | ILWCode | HLWCode | SRWCode;

// ==================== Isotope Definitions ====================

/**
 * Common fission product isotopes
 */
export type FissionProductIsotope =
  | 'Cs-137' // Cesium-137 (30.17 years)
  | 'Sr-90' // Strontium-90 (28.8 years)
  | 'I-131' // Iodine-131 (8.02 days)
  | 'Kr-85' // Krypton-85 (10.76 years)
  | 'Xe-133' // Xenon-133 (5.24 days)
  | 'Ru-106' // Ruthenium-106 (373.6 days)
  | 'H-3'; // Tritium (12.32 years)

/**
 * Transuranic isotopes
 */
export type TransuranicIsotope =
  | 'Pu-239' // Plutonium-239 (24,110 years)
  | 'Pu-240' // Plutonium-240 (6,561 years)
  | 'Am-241' // Americium-241 (432.2 years)
  | 'Np-237' // Neptunium-237 (2,144,000 years)
  | 'Cm-244'; // Curium-244 (18.1 years)

/**
 * Activation product isotopes
 */
export type ActivationProductIsotope =
  | 'Co-60' // Cobalt-60 (5.27 years)
  | 'Ni-63' // Nickel-63 (100.1 years)
  | 'Fe-55' // Iron-55 (2.73 years)
  | 'C-14' // Carbon-14 (5,730 years)
  | 'Cl-36'; // Chlorine-36 (301,000 years)

/**
 * All isotope types
 */
export type IsotopeCode = FissionProductIsotope | TransuranicIsotope | ActivationProductIsotope | string;

/**
 * Radiation types
 */
export type RadiationType = 'alpha' | 'beta' | 'gamma' | 'neutron' | 'x-ray';

// ==================== Core Data Structures ====================

/**
 * Geographic coordinates
 */
export interface Coordinates {
  latitude: number; // Decimal degrees
  longitude: number; // Decimal degrees
}

/**
 * Address information
 */
export interface Address {
  street?: string;
  city: string;
  state: string;
  country: string; // ISO 3166-1 alpha-2 code (e.g., "KR", "US")
  postalCode?: string;
}

/**
 * Measurement with unit
 */
export interface Measurement {
  value: number;
  unit: string;
}

/**
 * Activity measurement (radioactivity)
 */
export interface Activity {
  value: number;
  unit: 'Bq' | 'Ci' | 'mCi' | 'µCi' | 'GBq' | 'TBq';
}

/**
 * Dose measurement
 */
export interface Dose {
  value: number;
  unit: 'Sv' | 'mSv' | 'µSv' | 'rem' | 'mrem';
}

/**
 * Half-life information
 */
export interface HalfLife {
  value: number;
  unit: 'seconds' | 'minutes' | 'hours' | 'days' | 'years';
}

/**
 * Temperature measurement
 */
export interface Temperature {
  value: number;
  unit: 'celsius' | 'fahrenheit' | 'kelvin';
}

/**
 * Mass/weight measurement
 */
export interface Mass {
  value: number;
  unit: 'g' | 'kg' | 'tonnes' | 'lbs';
}

/**
 * Volume measurement
 */
export interface Volume {
  value: number;
  unit: 'm3' | 'liters' | 'gallons' | 'cubic_feet';
}

/**
 * Dimensions
 */
export interface Dimensions {
  height?: number;
  width?: number;
  depth?: number;
  diameter?: number;
  thickness?: number;
  unit: 'meters' | 'cm' | 'mm' | 'inches' | 'feet';
}

// ==================== Isotope Information ====================

/**
 * Energy levels
 */
export interface EnergyLevels {
  alpha?: number; // MeV
  beta?: number; // MeV
  gamma?: number; // MeV
  neutron?: number; // MeV
  unit: 'MeV' | 'keV' | 'eV';
}

/**
 * Isotope information
 */
export interface IsotopeInfo {
  isotope: IsotopeCode;
  activity: Activity;
  halfLife: HalfLife;
  decayConstant: number; // per second
  mass?: Mass;
  radiationType: RadiationType[];
  energy: EnergyLevels;
  classification?: 'fission_product' | 'transuranic' | 'activation_product';
}

/**
 * Uncertainties in measurements
 */
export interface MeasurementUncertainties {
  activityUncertainty: number; // percentage
  doseRateUncertainty: number; // percentage
  massUncertainty?: number; // percentage
  unit: 'percent' | 'sigma';
}

// ==================== Facility Information ====================

/**
 * Facility type
 */
export type FacilityType =
  | 'nuclear_power_plant'
  | 'medical'
  | 'research'
  | 'industrial'
  | 'interim_storage'
  | 'permanent_disposal'
  | 'processing'
  | 'decommissioning';

/**
 * License information
 */
export interface License {
  licenseNumber: string;
  authority: string; // e.g., "NSSC", "NRC", "STUK"
  issueDate?: string; // ISO 8601 date
  validUntil: string; // ISO 8601 date
}

/**
 * Facility location
 */
export interface FacilityLocation {
  address: Address;
  coordinates: Coordinates;
}

/**
 * Facility information
 */
export interface Facility {
  facilityId: string;
  facilityName: string;
  facilityType: FacilityType;
  location: FacilityLocation;
  license: License;
  operator?: string;
  contact?: ContactInfo;
}

/**
 * Contact information
 */
export interface ContactInfo {
  name: string;
  role?: string;
  phone: string;
  email: string;
  emergencyContact?: string;
}

// ==================== Container Information ====================

/**
 * Container type
 */
export type ContainerType =
  | 'Type_A' // Low activity materials
  | 'Type_B' // High activity materials
  | 'Type_C' // Aircraft transport
  | 'IP-1' // Industrial Package Level 1
  | 'IP-2' // Industrial Package Level 2
  | 'IP-3' // Industrial Package Level 3
  | 'spent_fuel_cask' // Spent fuel transport/storage cask
  | 'drum_200L' // 200L drum (standard)
  | 'concrete_box'
  | 'metal_box'
  | 'HIC'; // High Integrity Container

/**
 * Container material
 */
export type ContainerMaterial =
  | 'stainless_steel_304'
  | 'stainless_steel_316'
  | 'carbon_steel'
  | 'concrete'
  | 'lead_lined'
  | 'copper_steel_composite'
  | 'ductile_iron';

/**
 * Shielding information
 */
export interface Shielding {
  gammaShielding: number; // cm lead equivalent
  neutronShielding: number; // cm polyethylene equivalent
  unit: 'cm_lead_equivalent' | 'cm_poly_equivalent' | 'mm';
}

/**
 * Seal information
 */
export interface Seal {
  sealId: string;
  type: 'tamper_evident' | 'security_seal' | 'IAEA_seal';
  installDate: string; // ISO 8601 date
  serialNumber: string;
  verifiedBy?: string;
}

/**
 * Container information
 */
export interface ContainerInfo {
  containerId: string;
  containerType: ContainerType;
  material: ContainerMaterial;
  dimensions: Dimensions;
  weight: {
    empty: number;
    loaded: number;
    unit: 'kg' | 'lbs' | 'tonnes';
  };
  shielding?: Shielding;
  seals?: Seal[];
  manufacturer?: string;
  manufacturingDate?: string; // ISO 8601 date
  certification?: string;
}

// ==================== Radiological Data ====================

/**
 * Dose rate measurements
 */
export interface DoseRate {
  contact: Dose; // Dose rate at container surface
  oneMeter: Dose; // Dose rate at 1 meter distance
  twoMeters?: Dose; // Optional: Dose rate at 2 meters
}

/**
 * Radiological data
 */
export interface RadiologicalData {
  totalActivity: Activity;
  referenceDate: string; // ISO 8601 date
  doseRate: DoseRate;
  thermalOutput?: Measurement; // kW
  isotopes: IsotopeInfo[];
  uncertainties?: MeasurementUncertainties;
  dominantIsotope?: IsotopeCode;
}

// ==================== Physical & Chemical Properties ====================

/**
 * Waste form
 */
export type WasteForm = 'solid' | 'liquid' | 'gas' | 'sludge' | 'slurry';

/**
 * Waste matrix
 */
export type WasteMatrix =
  | 'metal'
  | 'concrete'
  | 'resin'
  | 'glass'
  | 'ceramic'
  | 'bitumen'
  | 'polymer'
  | 'organic'
  | 'mixed';

/**
 * Physical properties
 */
export interface PhysicalProperties {
  form: WasteForm;
  matrix: WasteMatrix;
  volume: Volume;
  density?: Measurement; // kg/m³
  moisture?: Measurement; // percentage
  pH?: number;
  temperature?: Temperature;
  flashPoint?: Temperature; // For flammable materials
}

/**
 * Chemical composition element
 */
export interface ChemicalElement {
  element: string; // Chemical symbol or compound name
  percentage: number; // weight percentage (0-100)
  mass?: Mass;
  casNumber?: string; // CAS Registry Number
}

// ==================== Processing ====================

/**
 * Processing method
 */
export type ProcessingMethod =
  | 'vitrification' // Glass solidification
  | 'cementation' // Concrete solidification
  | 'compaction' // Volume reduction
  | 'incineration' // Thermal treatment
  | 'evaporation' // Liquid volume reduction
  | 'ion_exchange' // Chemical treatment
  | 'filtration'
  | 'none'; // No processing

/**
 * Quality control information
 */
export interface QualityControl {
  qcPassed: boolean;
  qcDate: string; // ISO 8601 date
  inspector: string;
  certificate: string;
  testResults?: Record<string, any>;
}

/**
 * Processing information
 */
export interface Processing {
  method: ProcessingMethod;
  processingDate: string; // ISO 8601 date
  processingFacility: string;
  qualityControl: QualityControl;
  wasteReduction?: number; // percentage
  stabilizationAgent?: string;
}

// ==================== Storage ====================

/**
 * Storage type
 */
export type StorageType =
  | 'wet_storage' // Spent fuel pool
  | 'dry_storage' // Dry cask storage
  | 'vault' // Concrete vault
  | 'near_surface' // Near-surface disposal
  | 'intermediate_depth' // 50-200m depth
  | 'deep_geological'; // > 300m depth

/**
 * Storage location details
 */
export interface StorageLocationDetails {
  facilityId: string;
  facilityName: string;
  building?: string;
  room?: string;
  row?: string;
  column?: string;
  level?: string;
  position?: string;
  gpsCoordinates?: Coordinates;
}

/**
 * Storage conditions
 */
export interface StorageConditions {
  temperature: {
    value: number;
    max: number;
    min?: number;
    unit: 'celsius' | 'fahrenheit';
  };
  humidity?: {
    value: number;
    max: number;
    unit: 'percent';
  };
  ventilation?: 'natural' | 'forced_air' | 'none';
  shielding?: string;
  atmosphere?: 'air' | 'inert_gas' | 'vacuum';
}

/**
 * Storage duration
 */
export interface StorageDuration {
  planned: number; // years
  maximum: number; // years
  unit: 'years' | 'months' | 'days';
  startDate?: string; // ISO 8601 date
}

/**
 * Storage information
 */
export interface Storage {
  storageLocation: StorageLocationDetails;
  storageType: StorageType;
  storageDuration: StorageDuration;
  storageConditions: StorageConditions;
  retrievable: boolean;
  retrievalPlan?: string;
}

// ==================== Monitoring ====================

/**
 * Sensor type
 */
export type SensorType =
  | 'gamma_detector'
  | 'neutron_detector'
  | 'alpha_beta_detector'
  | 'temperature'
  | 'pressure'
  | 'humidity'
  | 'gas_monitor'
  | 'vibration'
  | 'crack_detection';

/**
 * Monitoring frequency
 */
export type MonitoringFrequency = 'continuous' | 'hourly' | 'daily' | 'weekly' | 'monthly' | 'quarterly';

/**
 * Sensor information
 */
export interface Sensor {
  sensorId: string;
  type: SensorType;
  model: string;
  manufacturer?: string;
  location: string; // Description of sensor location
  readingInterval: number; // seconds
  unit: 'seconds' | 'minutes' | 'hours';
  calibrationDate?: string; // ISO 8601 date
  nextCalibration?: string; // ISO 8601 date
}

/**
 * Alarm thresholds
 */
export interface AlarmThresholds {
  doseRateAlarm?: Dose;
  temperatureAlarm?: Temperature;
  pressureAlarm?: Measurement;
  contaminationAlarm?: {
    value: number;
    unit: 'Bq/cm2' | 'dpm/100cm2';
  };
}

/**
 * Monitoring information
 */
export interface Monitoring {
  monitoringFrequency: MonitoringFrequency;
  sensors: Sensor[];
  lastInspection: string; // ISO 8601 date
  nextInspection: string; // ISO 8601 date
  alarmThresholds: AlarmThresholds;
  inspectionReport?: string;
}

/**
 * Monitoring reading (single measurement)
 */
export interface MonitoringReading {
  readingId: string;
  timestamp: string; // ISO 8601 datetime
  sensorId: string;
  packageId?: string;
  measurements: {
    doseRate?: Dose;
    temperature?: Temperature;
    pressure?: Measurement;
    humidity?: number; // percentage
    contamination?: Measurement;
    [key: string]: any;
  };
  alarmTriggered: boolean;
  alarmLevel?: 'info' | 'warning' | 'alarm' | 'emergency';
}

// ==================== Traceability ====================

/**
 * Chain of custody event
 */
export interface ChainOfCustodyEvent {
  timestamp: string; // ISO 8601 datetime
  event: string; // Event description
  location: string;
  responsible: string; // Person responsible
  signature?: string;
  witness?: string;
}

/**
 * Blockchain integration
 */
export interface BlockchainTrace {
  enabled: boolean;
  network?: 'Hyperledger_Fabric' | 'Ethereum' | 'Polygon' | 'other';
  contractAddress?: string;
  transactionHash?: string;
  verified: boolean;
}

/**
 * Traceability information
 */
export interface Traceability {
  originReactor?: string;
  fuelAssemblyId?: string;
  burnup?: Measurement; // MWd/tU
  dischargeDate?: string; // ISO 8601 date
  coolingTime?: Measurement; // years
  chainOfCustody: ChainOfCustodyEvent[];
  blockchain?: BlockchainTrace;
  parentPackageId?: string; // For waste arising from reprocessing
}

// ==================== Regulatory ====================

/**
 * Material classification
 */
export type MaterialClassification =
  | 'Special_Nuclear_Material' // SNM: Pu-239, U-233, U-235
  | 'Source_Material' // Natural/depleted uranium, thorium
  | 'Byproduct_Material' // Fission products, activation products
  | 'Naturally_Occurring_Radioactive_Material'; // NORM

/**
 * IAEA reporting
 */
export interface IAEAReporting {
  required: boolean;
  reportId?: string;
  lastReported?: string; // ISO 8601 date
  nextReport?: string; // ISO 8601 date
  safeguardsCategory?: 'Category I' | 'Category II' | 'Category III';
}

/**
 * Transport approval
 */
export interface TransportApproval {
  approvalNumber: string;
  authority: string;
  validUntil: string; // ISO 8601 date
  restrictions?: string;
  mode?: 'road' | 'rail' | 'sea' | 'air';
}

/**
 * Disposal plan
 */
export interface DisposalPlan {
  plannedDisposalDate?: string; // ISO 8601 date
  disposalMethod: 'deep_geological_repository' | 'near_surface' | 'borehole' | 'other';
  disposalLocation?: string;
  fundingAllocated: boolean;
  fundingAmount?: Measurement; // currency
}

/**
 * Regulatory information
 */
export interface Regulatory {
  classification: MaterialClassification;
  safeguardsApplicable: boolean;
  IAEAreporting?: IAEAReporting;
  transportApproval?: TransportApproval;
  disposalPlan: DisposalPlan;
  exemptionCriteria?: string;
  clearanceLevels?: Activity;
}

// ==================== Safety ====================

/**
 * Risk level
 */
export type RiskLevel = 'very_high' | 'high' | 'medium' | 'low' | 'very_low';

/**
 * Emergency contact
 */
export interface EmergencyContact {
  role: string; // e.g., "Radiation_Protection_Officer"
  name: string;
  phone: string;
  email: string;
  alternatePhone?: string;
}

/**
 * Safety information
 */
export interface Safety {
  riskLevel: RiskLevel;
  emergencyContacts: EmergencyContact[];
  emergencyProcedure: string; // Procedure reference number
  lastDrill: string; // ISO 8601 date
  nextDrill: string; // ISO 8601 date
  incidentHistory?: string[];
}

// ==================== Main Package Structure ====================

/**
 * Radioactive waste package (complete data structure)
 */
export interface RadioactiveWastePackage {
  packageId: string;
  timestamp: string; // ISO 8601 datetime
  wasteClass: WasteClass;
  facility: Facility;
  containerInfo: ContainerInfo;
  radiologicalData: RadiologicalData;
  physicalProperties: PhysicalProperties;
  chemicalComposition?: ChemicalElement[];
  processing: Processing;
  storage: Storage;
  monitoring: Monitoring;
  traceability: Traceability;
  regulatory: Regulatory;
  safety: Safety;
  metadata?: {
    createdBy?: string;
    lastModified?: string;
    version?: string;
    [key: string]: any;
  };
}

// ==================== Storage Facility ====================

/**
 * Facility operational status
 */
export type OperationalStatus = 'active' | 'under_construction' | 'standby' | 'decommissioning' | 'closed';

/**
 * Geological information
 */
export interface GeologyInfo {
  rockType: string; // e.g., "granite", "clay", "salt"
  depth: number; // meters below surface
  unit: 'meters' | 'feet';
  seismicZone: 'very_low' | 'low' | 'moderate' | 'high';
  groundwaterDepth?: number; // meters
  waterFlowRate?: Measurement; // L/s
}

/**
 * Storage capacity
 */
export interface StorageCapacity {
  design: {
    HLW?: number;
    ILW?: number;
    LLW?: number;
    unit: 'm3' | 'packages';
  };
  current: {
    HLW?: number;
    ILW?: number;
    LLW?: number;
    unit: 'm3' | 'packages';
  };
  available: {
    HLW?: number;
    ILW?: number;
    LLW?: number;
    unit: 'm3' | 'packages';
  };
}

/**
 * Engineered barrier
 */
export interface EngineeredBarrier {
  barrier: string; // e.g., "waste_form", "container", "buffer"
  material: string;
  thickness: number | string; // number or "N/A", ">500"
  unit?: 'mm' | 'cm' | 'm';
  purpose: string;
}

/**
 * Monitoring systems
 */
export interface MonitoringSystems {
  radiation: {
    detectorType: SensorType[];
    coverage: string;
    dataLogging: 'real_time' | 'batch' | 'periodic';
    alarmSystem: boolean;
  };
  environmental: {
    airMonitoring: boolean;
    waterMonitoring: boolean;
    soilMonitoring: boolean;
    frequency: MonitoringFrequency;
  };
  structural?: {
    temperatureSensors?: number;
    pressureSensors?: number;
    seismicMonitoring?: boolean;
    crackDetection?: boolean;
  };
}

/**
 * Facility certification
 */
export interface FacilityCertification {
  type: string; // e.g., "IAEA_Safety_Standards", "WIA_ENE_026"
  number: string;
  level?: 'bronze' | 'silver' | 'gold' | 'platinum';
  issueDate: string; // ISO 8601 date
  expiryDate: string; // ISO 8601 date
  certifyingBody?: string;
}

/**
 * Facility performance metrics
 */
export interface FacilityPerformance {
  operationalUptime: number; // percentage
  unit: 'percent';
  safetyIncidents: number;
  environmentalCompliance: number; // percentage
  inspectionScore?: number; // 0-100
  averageDoseRate?: Dose;
}

/**
 * Storage facility (complete structure)
 */
export interface StorageFacility {
  facilityId: string;
  facilityName: string;
  facilityType: FacilityType;
  operationalStatus: OperationalStatus;
  location: {
    address: Address;
    coordinates: Coordinates;
    geology?: GeologyInfo;
  };
  capacity: StorageCapacity;
  engineeredBarriers?: EngineeredBarrier[];
  monitoringSystems: MonitoringSystems;
  certifications: FacilityCertification[];
  performance: FacilityPerformance;
  contact: ContactInfo;
  license?: License;
  emergencyPlan?: string;
}

// ==================== API Request/Response Types ====================

/**
 * Create package request
 */
export interface CreatePackageRequest {
  wasteClass: WasteClass;
  facilityId: string;
  containerInfo: Omit<ContainerInfo, 'containerId'>;
  radiologicalData: RadiologicalData;
  physicalProperties: PhysicalProperties;
  chemicalComposition?: ChemicalElement[];
  processing?: Partial<Processing>;
  storage: Omit<Storage, 'storageLocation'> & {
    storageLocation: Omit<StorageLocationDetails, 'facilityName'>;
  };
}

/**
 * Create package response
 */
export interface CreatePackageResponse {
  packageId: string;
  timestamp: string;
  status: 'success' | 'error';
  message?: string;
}

/**
 * Update location request
 */
export interface UpdateLocationRequest {
  packageId: string;
  newLocation: StorageLocationDetails;
  transferDate: string; // ISO 8601 date
  responsible: string;
  reason?: string;
}

/**
 * Query parameters for packages
 */
export interface PackageQueryParams {
  facilityId?: string;
  wasteClass?: WasteClass;
  isotope?: IsotopeCode;
  startDate?: string; // ISO 8601 date
  endDate?: string; // ISO 8601 date
  minActivity?: number; // Bq
  maxActivity?: number; // Bq
  storageType?: StorageType;
  page?: number;
  limit?: number;
}

/**
 * Dose calculation request
 */
export interface DoseCalculationRequest {
  packageId: string;
  distance: number; // meters
  duration: number; // seconds
  shielding?: {
    material: 'lead' | 'concrete' | 'steel' | 'water' | 'polyethylene';
    thickness: number; // meters or cm
    unit?: 'meters' | 'cm';
  };
}

/**
 * Dose calculation response
 */
export interface DoseCalculationResponse {
  effectiveDose: Dose;
  equivalentDose?: Dose;
  absorbedDose?: Dose;
  shieldingFactor?: number;
  calculationMethod: string;
}

/**
 * Decay calculation request
 */
export interface DecayCalculationRequest {
  isotope: IsotopeCode;
  initialActivity: number; // Bq
  decayTime: number;
  unit: 'seconds' | 'minutes' | 'hours' | 'days' | 'years';
}

/**
 * Decay calculation response
 */
export interface DecayCalculationResponse {
  isotope: IsotopeCode;
  initialActivity: Activity;
  finalActivity: Activity;
  decayTime: Measurement;
  halfLivesElapsed: number;
  remainingFraction: number; // 0-1
}

/**
 * Monitoring alert
 */
export interface MonitoringAlert {
  alertId: string;
  timestamp: string; // ISO 8601 datetime
  packageId?: string;
  facilityId?: string;
  sensorId?: string;
  severity: 'info' | 'warning' | 'alarm' | 'emergency';
  status: 'active' | 'acknowledged' | 'resolved';
  message: string;
  value?: Measurement;
  threshold?: Measurement;
  acknowledgedBy?: string;
  acknowledgedAt?: string; // ISO 8601 datetime
  resolvedAt?: string; // ISO 8601 datetime
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    limit: number;
    total: number;
    totalPages: number;
  };
}

/**
 * Facility inventory summary
 */
export interface FacilityInventory {
  facilityId: string;
  facilityName: string;
  timestamp: string; // ISO 8601 datetime
  summary: {
    totalPackages: number;
    byWasteClass: {
      HLW?: number;
      ILW?: number;
      LLW?: number;
      SRW?: number;
    };
    totalActivity: Activity;
    averageDoseRate: Dose;
    capacityUtilization: number; // percentage
  };
  packages: string[]; // Array of package IDs
}

// ==================== Webhook Types ====================

/**
 * Webhook event types
 */
export type WebhookEventType =
  | 'package.created'
  | 'package.moved'
  | 'package.updated'
  | 'monitoring.alarm'
  | 'dose.exceeded'
  | 'inspection.due'
  | 'safeguards.report'
  | 'facility.capacity.warning';

/**
 * Webhook payload
 */
export interface WebhookPayload<T = any> {
  event: WebhookEventType;
  timestamp: string; // ISO 8601 datetime
  data: T;
  signature?: string; // HMAC signature for verification
}

// ==================== Client Configuration ====================

/**
 * API client configuration
 */
export interface ClientConfig {
  apiKey: string;
  endpoint?: string; // Default: https://api.wia.org/ene-026/v1
  timeout?: number; // milliseconds
  retries?: number;
  facility?: {
    id: string;
    license: string;
  };
}

/**
 * API error response
 */
export interface APIError {
  code: string;
  message: string;
  details?: any;
  timestamp: string; // ISO 8601 datetime
  requestId?: string;
}

// ==================== Export All ====================

export default {
  // Types are exported individually above
};
