/**
 * WIA Battery Passport Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-battery-passport
 */

/**
 * Battery chemistry type
 */
export enum BatteryChemistry {
  LithiumIon = 'li_ion',
  LithiumIronPhosphate = 'lfp',
  NickelCobaltManganese = 'ncm',
  NickelCobaltAluminum = 'nca',
  LithiumPolymer = 'li_po',
  SolidState = 'solid_state',
  SodiumIon = 'na_ion',
  LeadAcid = 'lead_acid'
}

/**
 * Battery application type
 */
export enum BatteryApplication {
  ElectricVehicle = 'ev',
  EnergyStorage = 'ess',
  Industrial = 'industrial',
  Consumer = 'consumer',
  MedicalDevice = 'medical',
  Aerospace = 'aerospace',
  Marine = 'marine'
}

/**
 * Lifecycle stage
 */
export enum LifecycleStage {
  RawMaterial = 'raw_material',
  Manufacturing = 'manufacturing',
  Assembly = 'assembly',
  FirstUse = 'first_use',
  InUse = 'in_use',
  Refurbishment = 'refurbishment',
  SecondLife = 'second_life',
  Collection = 'collection',
  Recycling = 'recycling',
  Disposal = 'disposal'
}

/**
 * Battery state of health
 */
export enum StateOfHealth {
  Excellent = 'excellent',
  Good = 'good',
  Fair = 'fair',
  Poor = 'poor',
  EndOfLife = 'end_of_life'
}

/**
 * Battery passport identifier
 */
export interface BatteryPassportId {
  /** Unique passport ID */
  passportId: string;
  /** Battery unique identifier */
  batteryId: string;
  /** QR code data */
  qrCode: string;
  /** Digital product passport URL */
  dppUrl: string;
  /** GS1 identifier */
  gs1Id?: string;
}

/**
 * Battery specification
 */
export interface BatterySpecification {
  /** Manufacturer name */
  manufacturer: string;
  /** Manufacturing date */
  manufacturingDate: Date;
  /** Manufacturing location */
  manufacturingLocation: string;
  /** Battery model */
  model: string;
  /** Serial number */
  serialNumber: string;
  /** Chemistry type */
  chemistry: BatteryChemistry;
  /** Application type */
  application: BatteryApplication;
  /** Nominal capacity in Wh */
  nominalCapacity: number;
  /** Nominal voltage in V */
  nominalVoltage: number;
  /** Maximum voltage in V */
  maxVoltage: number;
  /** Minimum voltage in V */
  minVoltage: number;
  /** Weight in kg */
  weight: number;
  /** Dimensions */
  dimensions: Dimensions;
  /** Expected cycle life */
  expectedCycleLife: number;
  /** Warranty period in months */
  warrantyMonths: number;
}

/**
 * Dimensions
 */
export interface Dimensions {
  /** Length in mm */
  length: number;
  /** Width in mm */
  width: number;
  /** Height in mm */
  height: number;
}

/**
 * Material composition
 */
export interface MaterialComposition {
  /** Material name */
  material: string;
  /** Percentage by weight */
  percentage: number;
  /** Is critical raw material */
  isCritical: boolean;
  /** Is hazardous */
  isHazardous: boolean;
  /** Source country */
  sourceCountry?: string;
  /** Recycled content percentage */
  recycledContent?: number;
}

/**
 * Carbon footprint
 */
export interface CarbonFootprint {
  /** Total CO2 equivalent in kg */
  totalCO2e: number;
  /** Per kWh capacity */
  co2ePerKwh: number;
  /** Breakdown by lifecycle stage */
  breakdown: CarbonBreakdown[];
  /** Calculation methodology */
  methodology: string;
  /** Third-party verified */
  verified: boolean;
  /** Verifier name */
  verifier?: string;
  /** Verification date */
  verificationDate?: Date;
}

/**
 * Carbon breakdown by stage
 */
export interface CarbonBreakdown {
  /** Lifecycle stage */
  stage: LifecycleStage;
  /** CO2e in kg */
  co2e: number;
  /** Percentage of total */
  percentage: number;
}

/**
 * Performance data
 */
export interface PerformanceData {
  /** Current state of health (0-100) */
  stateOfHealth: number;
  /** State of health category */
  sohCategory: StateOfHealth;
  /** Current state of charge (0-100) */
  stateOfCharge: number;
  /** Total cycle count */
  cycleCount: number;
  /** Total energy throughput in kWh */
  energyThroughput: number;
  /** Remaining capacity in Wh */
  remainingCapacity: number;
  /** Internal resistance in mΩ */
  internalResistance: number;
  /** Maximum temperature recorded in °C */
  maxTemperature: number;
  /** Minimum temperature recorded in °C */
  minTemperature: number;
  /** Last updated */
  lastUpdated: Date;
}

/**
 * Charging event
 */
export interface ChargingEvent {
  /** Event ID */
  id: string;
  /** Start time */
  startTime: Date;
  /** End time */
  endTime: Date;
  /** Start SOC */
  startSoc: number;
  /** End SOC */
  endSoc: number;
  /** Energy charged in kWh */
  energyCharged: number;
  /** Charging rate in kW */
  chargingRate: number;
  /** Is fast charging */
  isFastCharge: boolean;
  /** Temperature during charging */
  temperature: number;
}

/**
 * Maintenance record
 */
export interface MaintenanceRecord {
  /** Record ID */
  id: string;
  /** Date */
  date: Date;
  /** Type */
  type: 'inspection' | 'repair' | 'replacement' | 'firmware_update' | 'calibration';
  /** Description */
  description: string;
  /** Technician */
  technician: string;
  /** Service center */
  serviceCenter: string;
  /** Parts replaced */
  partsReplaced?: string[];
  /** Next maintenance date */
  nextMaintenanceDate?: Date;
}

/**
 * Ownership record
 */
export interface OwnershipRecord {
  /** Record ID */
  id: string;
  /** Owner type */
  ownerType: 'manufacturer' | 'distributor' | 'retailer' | 'end_user' | 'refurbisher' | 'recycler';
  /** Owner name */
  ownerName: string;
  /** Owner country */
  country: string;
  /** Ownership start date */
  startDate: Date;
  /** Ownership end date */
  endDate?: Date;
  /** Transfer transaction ID */
  transferTxId?: string;
}

/**
 * Due diligence record
 */
export interface DueDiligenceRecord {
  /** Record ID */
  id: string;
  /** Check type */
  checkType: 'supply_chain' | 'conflict_minerals' | 'labor_rights' | 'environmental';
  /** Status */
  status: 'passed' | 'failed' | 'pending' | 'not_applicable';
  /** Details */
  details: string;
  /** Evidence documents */
  evidenceDocuments?: string[];
  /** Check date */
  checkDate: Date;
  /** Checker organization */
  checker: string;
}

/**
 * Safety certification
 */
export interface SafetyCertification {
  /** Certification name */
  name: string;
  /** Standard */
  standard: string;
  /** Certificate number */
  certificateNumber: string;
  /** Issue date */
  issueDate: Date;
  /** Expiry date */
  expiryDate: Date;
  /** Certifying body */
  certifyingBody: string;
  /** Certificate URL */
  certificateUrl?: string;
}

/**
 * Recycling information
 */
export interface RecyclingInfo {
  /** Recycling instructions */
  instructions: string;
  /** Collection points URL */
  collectionPointsUrl: string;
  /** Recyclability percentage */
  recyclabilityPercentage: number;
  /** Hazardous materials handling */
  hazardousHandling: string;
  /** Disassembly instructions */
  disassemblyInstructions?: string;
  /** Recycler requirements */
  recyclerRequirements?: string[];
}

/**
 * Complete battery passport
 */
export interface BatteryPassport {
  /** Passport identifier */
  id: BatteryPassportId;
  /** Battery specification */
  specification: BatterySpecification;
  /** Material composition */
  materials: MaterialComposition[];
  /** Carbon footprint */
  carbonFootprint: CarbonFootprint;
  /** Performance data */
  performance: PerformanceData;
  /** Current lifecycle stage */
  lifecycleStage: LifecycleStage;
  /** Charging history */
  chargingHistory: ChargingEvent[];
  /** Maintenance records */
  maintenanceRecords: MaintenanceRecord[];
  /** Ownership history */
  ownershipHistory: OwnershipRecord[];
  /** Due diligence records */
  dueDiligence: DueDiligenceRecord[];
  /** Safety certifications */
  certifications: SafetyCertification[];
  /** Recycling info */
  recycling: RecyclingInfo;
  /** Created at */
  createdAt: Date;
  /** Last updated at */
  updatedAt: Date;
}

/**
 * SDK configuration
 */
export interface BatteryPassportConfig {
  /** API endpoint */
  apiEndpoint: string;
  /** API key */
  apiKey?: string;
  /** Enable blockchain verification */
  blockchainEnabled: boolean;
  /** IPFS gateway */
  ipfsGateway?: string;
  /** Auto-sync interval in seconds */
  syncInterval: number;
}

/**
 * Certification level
 */
export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

/**
 * Compliance report
 */
export interface ComplianceReport {
  /** Standard */
  standard: 'WIA-BATTERY-PASSPORT';
  /** Test date */
  testDate: string;
  /** Configuration */
  config: BatteryPassportConfig;
  /** Target level */
  targetLevel: CertificationLevel;
  /** Tests */
  tests: TestResult[];
  /** Passed */
  passed: boolean;
  /** Achieved level */
  achievedLevel?: CertificationLevel;
}

/**
 * Test result
 */
export interface TestResult {
  /** Test name */
  testName: string;
  /** Passed */
  passed: boolean;
  /** Notes */
  notes?: string;
}
