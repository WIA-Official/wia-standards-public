/**
 * WIA-ENE-036: Oil & Gas Drilling Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license CC BY 4.0
 * @description Type definitions for oil and gas drilling operations, production monitoring,
 *              and environmental management
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

// ==================== Well Type Classification ====================

/**
 * Exploration well codes
 */
export type ExplorationWellCode =
  | 'EXP-1' // Wildcat - Unexplored area
  | 'EXP-2' // Appraisal - Discovered reservoir evaluation
  | 'EXP-3' // Stratigraphic test - Geological structure confirmation

/**
 * Production well codes
 */
export type ProductionWellCode =
  | 'PROD-1' // Vertical well
  | 'PROD-2' // Deviated well
  | 'PROD-3' // Horizontal well
  | 'PROD-4' // Multilateral well

/**
 * Injection well codes
 */
export type InjectionWellCode =
  | 'INJ-1' // Water injection
  | 'INJ-2' // Gas reinjection (Natural gas, CO₂)
  | 'INJ-3' // Chemical injection (EOR)
  | 'INJ-4' // Wastewater disposal

/**
 * All well type codes
 */
export type WellTypeCode = ExplorationWellCode | ProductionWellCode | InjectionWellCode;

/**
 * Well status
 */
export type WellStatus =
  | 'drilling' // Currently drilling
  | 'completing' // Completion operations
  | 'producing' // Active production
  | 'injecting' // Active injection
  | 'shut_in' // Temporarily shut in
  | 'suspended' // Suspended operations
  | 'abandoned' // Permanently abandoned
  | 'plugged' // Plugged and abandoned

// ==================== Core Data Structures ====================

/**
 * Geographic coordinates
 */
export interface Coordinates {
  latitude: number; // Decimal degrees
  longitude: number; // Decimal degrees
  datum?: 'WGS84' | 'NAD83' | 'NAD27';
}

/**
 * Address information
 */
export interface Address {
  county?: string;
  state: string;
  country: string; // ISO 3166-1 alpha-2 code
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
 * Pressure measurement
 */
export interface Pressure {
  value: number;
  unit: 'psi' | 'bar' | 'kPa' | 'MPa';
}

/**
 * Temperature measurement
 */
export interface Temperature {
  value: number;
  unit: 'celsius' | 'fahrenheit' | 'kelvin';
}

/**
 * Volume measurement
 */
export interface Volume {
  value: number;
  unit: 'bbl' | 'm3' | 'scf' | 'mcf' | 'mmcf' | 'bcf';
}

/**
 * Flow rate measurement
 */
export interface FlowRate {
  value: number;
  unit: 'bbl/day' | 'm3/day' | 'scf/day' | 'mcf/day';
}

/**
 * Length/depth measurement
 */
export interface Length {
  value: number;
  unit: 'meters' | 'feet' | 'inches';
}

// ==================== Operator Information ====================

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

/**
 * Operator information
 */
export interface Operator {
  operatorId: string;
  operatorName: string;
  license: string;
  contact: ContactInfo;
  operatorType?: 'major' | 'independent' | 'national_oil_company';
}

// ==================== Well Location ====================

/**
 * Well location details
 */
export interface WellLocation {
  address: Address;
  coordinates: Coordinates;
  surfaceElevation?: Length;
  landOwnership?: 'federal' | 'state' | 'private' | 'tribal';
  legalDescription?: string; // e.g., "Section 12, Township 5N, Range 3W"
}

// ==================== Wellbore Geometry ====================

/**
 * Casing type
 */
export type CasingType =
  | 'conductor' // Surface protection
  | 'surface' // Shallow formations
  | 'intermediate' // Transition zones
  | 'production' // Production zone
  | 'liner'; // Partial string

/**
 * Casing information
 */
export interface Casing {
  casingType: CasingType;
  outerDiameter: number; // inches
  weight: number; // lbs/ft
  grade: string; // e.g., "K55", "L80", "P110"
  depth: number; // meters or feet
  unit: 'meters' | 'feet';
  cementTop?: number; // Top of cement
  testPressure?: Pressure;
}

/**
 * Wellbore trajectory
 */
export interface WellboreGeometry {
  totalDepth: {
    measuredDepth: number; // MD - along hole
    trueVerticalDepth: number; // TVD - vertical depth
    unit: 'meters' | 'feet';
  };
  kickoffPoint?: {
    depth: number;
    unit: 'meters' | 'feet';
  };
  horizontalSection?: {
    length: number;
    azimuth: number; // degrees
    unit: 'meters' | 'feet';
  };
  casingProgram: Casing[];
}

// ==================== Reservoir Information ====================

/**
 * Reservoir type
 */
export type ReservoirType =
  | 'conventional_sandstone'
  | 'conventional_carbonate'
  | 'unconventional_shale'
  | 'unconventional_tight_sand'
  | 'heavy_oil'
  | 'coalbed_methane';

/**
 * Fluid type
 */
export type FluidType =
  | 'oil' // Primarily oil
  | 'gas' // Primarily gas
  | 'oil_gas_mixture' // Both oil and gas
  | 'condensate'; // Gas condensate

/**
 * Reservoir properties
 */
export interface ReservoirProperties {
  porosity: {
    value: number;
    unit: 'percent' | 'fraction';
  };
  permeability: {
    value: number;
    unit: 'millidarcy' | 'darcy';
  };
  temperature: Temperature;
  initialPressure: Pressure;
  currentPressure?: Pressure;
  saturationPressure?: Pressure; // Bubble point or dew point
  waterSaturation?: {
    value: number;
    unit: 'percent';
  };
}

/**
 * Reservoir information
 */
export interface Reservoir {
  formationName: string;
  reservoirType: ReservoirType;
  depth: {
    top: number;
    bottom: number;
    unit: 'meters' | 'feet';
  };
  properties: ReservoirProperties;
  fluidType: FluidType;
  apiGravity?: number; // For oil
  gasGravity?: number; // For gas (air = 1.0)
  gasOilRatio?: {
    value: number;
    unit: 'scf/bbl' | 'm3/m3';
  };
  oilViscosity?: {
    value: number;
    unit: 'cP' | 'mPa·s';
  };
}

// ==================== Drilling Operations ====================

/**
 * Rig type
 */
export type RigType =
  | 'land_rig_mechanical'
  | 'land_rig_AC_drive'
  | 'land_rig_SCR'
  | 'jackup_rig'
  | 'semi_submersible'
  | 'drillship';

/**
 * Mud type
 */
export type MudType =
  | 'water_based'
  | 'oil_based'
  | 'synthetic_based'
  | 'foam';

/**
 * Mud program
 */
export interface MudProgram {
  mudType: MudType;
  mudWeight: {
    value: number;
    unit: 'ppg' | 'g/cm3' | 'kg/m3';
  };
  viscosity: {
    value: number;
    unit: 'seconds' | 'cP';
  };
  pH?: number;
  chlorides?: {
    value: number;
    unit: 'mg/L' | 'ppm';
  };
}

/**
 * Bit type
 */
export type BitType =
  | 'PDC' // Polycrystalline Diamond Compact
  | 'PDC_hybrid'
  | 'roller_cone'
  | 'diamond'
  | 'tricone';

/**
 * Bit run information
 */
export interface BitRun {
  runNumber: number;
  bitType: BitType;
  bitSize: number; // inches
  manufacturer?: string;
  serialNumber?: string;
  depthIn: number;
  depthOut: number;
  footageDrilled?: number;
  hoursRotating?: number;
  rop: {
    // Rate of Penetration
    average: number;
    max?: number;
    unit: 'm/hr' | 'ft/hr';
  };
  bitCondition?: string; // e.g., "BT 1-1, DG I, OC G"
}

/**
 * Drilling operations data
 */
export interface DrillingOperations {
  spudDate: string; // ISO 8601 date
  completionDate?: string; // ISO 8601 date
  drillingDays?: number;
  drillingContractor: string;
  rigType: RigType;
  rigNumber?: string;
  mudProgram: MudProgram;
  bitRuns: BitRun[];
  incidents?: DrillingIncident[];
}

/**
 * Drilling incident
 */
export interface DrillingIncident {
  incidentType: 'kick' | 'lost_circulation' | 'stuck_pipe' | 'equipment_failure' | 'other';
  date: string; // ISO 8601 datetime
  depth: number;
  description: string;
  nptHours?: number; // Non-productive time
  resolved: boolean;
}

// ==================== Completion Design ====================

/**
 * Completion type
 */
export type CompletionType =
  | 'openhole'
  | 'cased_hole_perforated'
  | 'gravel_pack'
  | 'frac_pack'
  | 'multi_stage_fracturing';

/**
 * Proppant type
 */
export type ProppantType =
  | 'sand_20_40_mesh'
  | 'sand_40_70_mesh'
  | 'ceramic_20_40_mesh'
  | 'ceramic_40_70_mesh'
  | 'resin_coated';

/**
 * Fracturing fluid composition
 */
export interface FracFluidComposition {
  water: number; // percent
  proppant: number; // percent
  chemicals: number; // percent
  unit: 'percent';
  chemicalDetails?: {
    friction_reducer?: number;
    biocide?: number;
    scale_inhibitor?: number;
    surfactant?: number;
    acid?: number;
  };
}

/**
 * Completion design
 */
export interface CompletionDesign {
  completionType: CompletionType;
  totalStages?: number; // For multi-stage fracturing
  stageSpacing?: Length;
  fracturingFluid?: {
    totalVolume: number;
    unit: 'm3' | 'bbl';
    composition: FracFluidComposition;
  };
  proppant?: {
    type: ProppantType;
    totalMass: number;
    unit: 'kg' | 'lbs';
  };
  perforatingGuns?: {
    shotsPerFoot: number;
    phasing: number; // degrees
    unit: 'shots' | 'degrees';
  };
  productionTubing?: {
    size: number; // inches
    depth: number;
    unit: 'meters' | 'feet';
  };
}

// ==================== Production Data ====================

/**
 * Oil production
 */
export interface OilProduction {
  gross?: Volume; // Total liquid
  net: Volume; // Oil only
  bsw?: {
    // Basic Sediment and Water
    value: number;
    unit: 'percent';
  };
}

/**
 * Gas production
 */
export interface GasProduction {
  gross: Volume; // Total gas produced
  sales?: Volume; // Gas sold
  flared?: Volume; // Gas flared
  vented?: Volume; // Gas vented
  fuel?: Volume; // Used as fuel
}

/**
 * Water production
 */
export interface WaterProduction {
  produced: Volume;
  disposed?: Volume;
  injected?: Volume;
  salinity?: {
    value: number;
    unit: 'ppm' | 'mg/L' | 'ppm_TDS';
  };
}

/**
 * Daily production data
 */
export interface DailyProduction {
  oil?: OilProduction;
  gas?: GasProduction;
  water?: WaterProduction;
  uptime?: {
    value: number;
    unit: 'hours' | 'percent';
  };
}

/**
 * Wellhead conditions
 */
export interface WellheadConditions {
  pressure: {
    tubing: Pressure;
    casing: Pressure;
  };
  temperature: Temperature;
  chokeSize?: {
    value: number;
    unit: '64ths_inch' | 'mm';
  };
}

/**
 * Production record
 */
export interface ProductionRecord {
  recordId: string;
  wellId: string;
  timestamp: string; // ISO 8601 datetime
  dailyProduction: DailyProduction;
  wellheadConditions?: WellheadConditions;
  separatorConditions?: {
    pressure: Pressure;
    temperature: Temperature;
    liquidLevel?: {
      value: number;
      unit: 'percent';
    };
  };
  comments?: string;
}

/**
 * Cumulative production
 */
export interface CumulativeProduction {
  oil?: Volume;
  gas?: Volume;
  water?: Volume;
  asOfDate: string; // ISO 8601 date
}

/**
 * Production forecast
 */
export interface ProductionForecast {
  estimatedUltimateRecovery: {
    oil?: Volume;
    gas?: Volume;
  };
  reserveCategory?: 'proved' | 'probable' | 'possible';
  forecastMethod?: 'decline_curve' | 'reservoir_simulation' | 'analogy';
  economicLimit?: {
    oil?: FlowRate;
    gas?: FlowRate;
  };
}

// ==================== Environmental Monitoring ====================

/**
 * Water usage tracking
 */
export interface WaterUsage {
  drilling?: Volume;
  fracturing?: Volume;
  operations?: Volume;
  totalConsumed: Volume;
  source?: 'surface_water' | 'groundwater' | 'municipal' | 'recycled';
  recycledPercentage?: number;
}

/**
 * Methane emissions
 */
export interface MethaneEmissions {
  venting?: {
    value: number;
    unit: 'tonnes' | 'tonnes_CO2e/year' | 'kg/day';
  };
  flaring?: {
    value: number;
    unit: 'tonnes' | 'tonnes_CO2e/year' | 'kg/day';
  };
  fugitiveEmissions?: {
    value: number;
    unit: 'tonnes' | 'tonnes_CO2e/year' | 'kg/day';
  };
  totalEmissions?: {
    value: number;
    unit: 'tonnes_CO2e/year';
  };
}

/**
 * Flaring data
 */
export interface FlaringData {
  volumeFlared: Volume;
  flaringIntensity?: {
    value: number;
    unit: 'percent_of_gas_production' | 'scf/bbl';
  };
  combustionEfficiency?: {
    value: number;
    unit: 'percent';
  };
  flaringReason?: 'no_pipeline' | 'safety' | 'maintenance' | 'startup' | 'emergency';
}

/**
 * Emissions monitoring
 */
export interface EmissionsMonitoring {
  methaneEmissions: MethaneEmissions;
  flaringData?: FlaringData;
  co2Emissions?: {
    value: number;
    unit: 'tonnes/year';
  };
  vocEmissions?: {
    // Volatile Organic Compounds
    value: number;
    unit: 'tonnes/year';
  };
  monitoringMethod?: 'continuous' | 'periodic' | 'estimated';
}

/**
 * Containment system
 */
export interface ContainmentSystem {
  systemType: 'secondary_containment' | 'berm' | 'lined_pit' | 'tank_dike';
  location: string;
  capacity: Volume;
  lastInspection: string; // ISO 8601 date
  status: 'operational' | 'maintenance' | 'failed';
  material?: string;
}

/**
 * Spill incident
 */
export interface SpillIncident {
  incidentId: string;
  date: string; // ISO 8601 datetime
  substance: 'crude_oil' | 'condensate' | 'produced_water' | 'drilling_fluid' | 'chemicals';
  volumeSpilled: Volume;
  location: string;
  cause: string;
  environmentalImpact?: 'none' | 'minor' | 'moderate' | 'major';
  cleanupStatus: 'ongoing' | 'completed';
  cleanupCost?: {
    value: number;
    currency: 'USD' | 'EUR' | 'CAD';
  };
  reportedToAuthorities: boolean;
  reportDate?: string; // ISO 8601 datetime
}

/**
 * Spill prevention
 */
export interface SpillPrevention {
  containmentSystems: ContainmentSystem[];
  spillHistory: SpillIncident[];
  preventionPlan?: string;
  lastDrill?: string; // ISO 8601 date
}

/**
 * Wastewater management
 */
export interface WastewaterManagement {
  producedWater: {
    volume: FlowRate;
    salinity?: {
      value: number;
      unit: 'ppm_TDS' | 'mg/L';
    };
    disposalMethod: 'injection_well' | 'treatment_discharge' | 'evaporation' | 'reuse';
    disposalWellId?: string;
  };
  flowbackWater?: {
    volume: Volume;
    treatmentMethod?: string;
  };
}

/**
 * Environmental monitoring
 */
export interface EnvironmentalMonitoring {
  waterUsage: WaterUsage;
  emissionsMonitoring: EmissionsMonitoring;
  spillPrevention: SpillPrevention;
  wastewaterManagement: WastewaterManagement;
  airQualityMonitoring?: boolean;
  noiseMonitoring?: boolean;
  soilTesting?: boolean;
}

// ==================== Safety Systems ====================

/**
 * BOP type
 */
export type BOPType =
  | 'ram'
  | 'annular'
  | 'ram_annular'
  | 'subsea';

/**
 * Blowout Preventer (BOP)
 */
export interface BlowoutPreventer {
  bopType: BOPType;
  manufacturer?: string;
  model?: string;
  workingPressure: Pressure;
  lastTest: string; // ISO 8601 date
  testPressure: Pressure;
  testResult: 'passed' | 'failed';
  testInterval: {
    value: number;
    unit: 'days';
  };
  components?: {
    annularPreventer?: boolean;
    pipeRams?: boolean;
    blindRams?: boolean;
    shearRams?: boolean;
  };
}

/**
 * H2S detection system
 */
export interface H2SDetection {
  h2sPresent: boolean;
  concentration?: {
    value: number;
    unit: 'ppm';
  };
  detectionThreshold: {
    value: number;
    unit: 'ppm';
  };
  alarmSetpoint: {
    value: number;
    unit: 'ppm';
  };
  scbaAvailable?: boolean; // Self-Contained Breathing Apparatus
  windSockInstalled?: boolean;
}

/**
 * Emergency shutdown system
 */
export interface EmergencyShutdownSystem {
  esdValves: number;
  lastTest: string; // ISO 8601 date
  responseTime?: {
    value: number;
    unit: 'seconds';
  };
  automatedShutdown: boolean;
}

/**
 * Safety systems
 */
export interface SafetySystems {
  blowoutPreventer?: BlowoutPreventer;
  h2sDetection?: H2SDetection;
  emergencyShutdownSystem?: EmergencyShutdownSystem;
  fireDetection?: boolean;
  gasDetection?: boolean;
  safetyTraining?: {
    frequency: 'annual' | 'biannual' | 'quarterly';
    lastTraining: string; // ISO 8601 date
  };
}

// ==================== Regulatory ====================

/**
 * Permit type
 */
export type PermitType =
  | 'drilling_permit'
  | 'completion_permit'
  | 'production_permit'
  | 'injection_permit'
  | 'air_quality_permit'
  | 'water_discharge_permit'
  | 'waste_disposal_permit';

/**
 * Regulatory permit
 */
export interface Permit {
  permitType: PermitType;
  permitNumber: string;
  authority: string; // e.g., "Texas_Railroad_Commission", "EPA"
  issueDate: string; // ISO 8601 date
  expiryDate: string; // ISO 8601 date
  conditions?: string[];
  status: 'active' | 'expired' | 'suspended' | 'revoked';
}

/**
 * Reporting requirements
 */
export interface ReportingRequirements {
  productionReporting: 'daily' | 'weekly' | 'monthly' | 'quarterly';
  emissionsReporting: 'monthly' | 'quarterly' | 'annual';
  safetyIncidents: 'immediate' | 'within_24h' | 'within_7days';
  spills: 'immediate';
}

/**
 * Regulatory information
 */
export interface RegulatoryInfo {
  permits: Permit[];
  reporting: ReportingRequirements;
  inspections?: {
    lastInspection: string; // ISO 8601 date
    nextInspection?: string; // ISO 8601 date
    violations?: number;
  };
  fines?: {
    totalAmount: number;
    currency: 'USD' | 'EUR' | 'CAD';
    resolved: boolean;
  }[];
}

// ==================== Decommissioning ====================

/**
 * Plug type
 */
export type PlugType =
  | 'cement_plug'
  | 'mechanical_plug'
  | 'bridge_plug';

/**
 * Cement plug
 */
export interface CementPlug {
  plugNumber: number;
  plugType: PlugType;
  topDepth: number;
  bottomDepth: number;
  length: number;
  unit: 'meters' | 'feet';
  purpose: 'reservoir_isolation' | 'casing_shoe' | 'surface_plug' | 'intermediate';
  cementType?: string;
  cementStrength?: Pressure; // Compressive strength
  testPressure?: Pressure;
  testResult?: 'passed' | 'failed';
}

/**
 * P&A (Plug and Abandonment) procedure
 */
export interface PlugAndAbandonment {
  planApproved: boolean;
  approvalDate?: string; // ISO 8601 date
  authority?: string;
  executionDate?: string; // ISO 8601 date
  contractor?: string;
  cementPlugs: CementPlug[];
  tubingRemoved: boolean;
  wellheadRemoved: boolean;
  casingCutDepth?: {
    value: number;
    unit: 'meters' | 'feet';
  };
  siteClearedAndRestored: boolean;
  finalReportSubmitted: boolean;
  cost?: {
    value: number;
    currency: 'USD' | 'EUR' | 'CAD';
  };
}

/**
 * Site restoration
 */
export interface SiteRestoration {
  wellPadArea: {
    value: number;
    unit: 'hectares' | 'acres';
  };
  restorationPlan: {
    soilRemediation?: {
      method: 'excavation_and_disposal' | 'bioremediation' | 'thermal_treatment';
      contaminatedVolume?: Volume;
      targetContaminationLevel?: Record<string, Measurement>;
    };
    revegetation?: {
      seedMix: string;
      plantingDensity?: Measurement;
      monitoringPeriod?: {
        value: number;
        unit: 'years';
      };
    };
    recontouring?: {
      finalGrade: string;
      erosionControl?: string;
    };
  };
  postClosureMonitoring?: {
    groundwaterMonitoring?: {
      wells: number;
      frequency: 'monthly' | 'quarterly' | 'annual';
      duration: {
        value: number;
        unit: 'years';
      };
      parameters: string[];
    };
    vegetationMonitoring?: {
      frequency: 'annual' | 'biannual';
      duration: {
        value: number;
        unit: 'years';
      };
    };
  };
  financialAssurance?: {
    bondType: 'surety_bond' | 'cash_deposit' | 'letter_of_credit';
    bondAmount: {
      value: number;
      currency: 'USD' | 'EUR' | 'CAD';
    };
    bondHolder: string;
  };
}

/**
 * Decommissioning information
 */
export interface Decommissioning {
  status: 'not_planned' | 'planned' | 'in_progress' | 'completed';
  plannedDate?: string; // ISO 8601 date
  plugAndAbandonment?: PlugAndAbandonment;
  siteRestoration?: SiteRestoration;
}

// ==================== Main Well Structure ====================

/**
 * Complete well data structure
 */
export interface Well {
  wellId: string;
  timestamp: string; // ISO 8601 datetime
  wellType: WellTypeCode;
  wellName: string;
  status: WellStatus;
  operator: Operator;
  location: WellLocation;
  wellboreGeometry: WellboreGeometry;
  reservoir?: Reservoir;
  drillingOperations?: DrillingOperations;
  completionDesign?: CompletionDesign;
  production?: {
    firstProduction?: string; // ISO 8601 date
    currentRate?: DailyProduction;
    cumulativeProduction?: CumulativeProduction;
    estimatedUltimateRecovery?: ProductionForecast;
  };
  environmental?: EnvironmentalMonitoring;
  safety?: SafetySystems;
  regulatory?: RegulatoryInfo;
  decommissioning?: Decommissioning;
  metadata?: {
    createdBy?: string;
    lastModified?: string;
    version?: string;
    dataSource?: string;
    [key: string]: any;
  };
}

// ==================== API Request/Response Types ====================

/**
 * Create well request
 */
export interface CreateWellRequest {
  wellType: WellTypeCode;
  wellName: string;
  operatorId: string;
  location: WellLocation;
  wellboreGeometry: Partial<WellboreGeometry>;
}

/**
 * Create well response
 */
export interface CreateWellResponse {
  wellId: string;
  timestamp: string;
  status: 'success' | 'error';
  message?: string;
}

/**
 * Update well status request
 */
export interface UpdateWellStatusRequest {
  wellId: string;
  status: WellStatus;
  effectiveDate: string; // ISO 8601 date
  reason?: string;
}

/**
 * Query parameters for wells
 */
export interface WellQueryParams {
  operatorId?: string;
  wellType?: WellTypeCode;
  state?: string;
  status?: WellStatus;
  county?: string;
  minDepth?: number;
  maxDepth?: number;
  reservoirType?: ReservoirType;
  page?: number;
  limit?: number;
}

/**
 * Production data submission request
 */
export interface SubmitProductionRequest {
  wellId: string;
  date: string; // ISO 8601 date
  production: DailyProduction;
  wellheadConditions?: WellheadConditions;
}

/**
 * Production query parameters
 */
export interface ProductionQueryParams {
  wellId?: string;
  startDate?: string; // ISO 8601 date
  endDate?: string; // ISO 8601 date
  aggregation?: 'daily' | 'monthly' | 'yearly';
  page?: number;
  limit?: number;
}

/**
 * Environmental data submission
 */
export interface SubmitEnvironmentalDataRequest {
  wellId: string;
  reportingPeriod: {
    startDate: string; // ISO 8601 date
    endDate: string; // ISO 8601 date
  };
  emissionsData: EmissionsMonitoring;
  spillIncidents?: SpillIncident[];
}

/**
 * Spill report request
 */
export interface SpillReportRequest {
  wellId: string;
  incidentDate: string; // ISO 8601 datetime
  substance: string;
  volumeSpilled: Volume;
  location: string;
  cause: string;
  immediateActions: string;
}

/**
 * Production forecast request
 */
export interface ProductionForecastRequest {
  wellId: string;
  forecastPeriod: number; // months
  method?: 'decline_curve' | 'reservoir_simulation' | 'analogy';
}

/**
 * Production forecast response
 */
export interface ProductionForecastResponse {
  wellId: string;
  forecastPeriod: number;
  forecast: {
    month: number;
    oilRate?: FlowRate;
    gasRate?: FlowRate;
    waterRate?: FlowRate;
  }[];
  estimatedUltimateRecovery: ProductionForecast;
  confidenceLevel?: number; // 0-100
}

/**
 * Emissions alert
 */
export interface EmissionsAlert {
  alertId: string;
  wellId: string;
  timestamp: string; // ISO 8601 datetime
  alertType: 'methane_leak' | 'flaring_excess' | 'voc_threshold' | 'equipment_malfunction';
  severity: 'info' | 'warning' | 'critical';
  measurement?: Measurement;
  threshold?: Measurement;
  status: 'active' | 'acknowledged' | 'resolved';
  acknowledgedBy?: string;
  resolvedAt?: string; // ISO 8601 datetime
}

/**
 * Safety incident report
 */
export interface SafetyIncidentReport {
  incidentId: string;
  wellId: string;
  incidentDate: string; // ISO 8601 datetime
  incidentType: 'kick' | 'blowout' | 'h2s_release' | 'fire' | 'injury' | 'equipment_failure' | 'near_miss';
  severity: 'minor' | 'moderate' | 'serious' | 'fatal';
  description: string;
  injuries?: number;
  fatalities?: number;
  propertyDamage?: {
    value: number;
    currency: 'USD' | 'EUR' | 'CAD';
  };
  rootCause?: string;
  correctiveActions?: string[];
  reportedToAuthorities: boolean;
  investigationStatus: 'pending' | 'ongoing' | 'completed';
}

/**
 * Compliance status
 */
export interface ComplianceStatus {
  wellId: string;
  asOfDate: string; // ISO 8601 date
  permits: {
    total: number;
    active: number;
    expired: number;
    expiringSoon: number; // Within 90 days
  };
  reporting: {
    productionReports: {
      required: number;
      submitted: number;
      overdue: number;
    };
    emissionsReports: {
      required: number;
      submitted: number;
      overdue: number;
    };
  };
  inspections: {
    passed: number;
    failed: number;
    pending: number;
  };
  violations: {
    total: number;
    resolved: number;
    outstanding: number;
  };
  overallCompliance: 'compliant' | 'non_compliant' | 'conditional';
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
 * Operator dashboard summary
 */
export interface OperatorDashboard {
  operatorId: string;
  operatorName: string;
  timestamp: string; // ISO 8601 datetime
  summary: {
    totalWells: number;
    producingWells: number;
    shutInWells: number;
    abandonedWells: number;
  };
  production: {
    daily: {
      oil?: Volume;
      gas?: Volume;
    };
    monthly: {
      oil?: Volume;
      gas?: Volume;
    };
  };
  environmental: {
    totalEmissions: {
      value: number;
      unit: 'tonnes_CO2e/year';
    };
    flaringIntensity: {
      value: number;
      unit: 'percent';
    };
    spillIncidents: number;
  };
  compliance: {
    permitCompliance: number; // percent
    reportingCompliance: number; // percent
    safetyScore: number; // 0-100
  };
}

// ==================== Webhook Types ====================

/**
 * Webhook event types
 */
export type WebhookEventType =
  | 'well.created'
  | 'well.status_changed'
  | 'production.threshold'
  | 'emissions.alarm'
  | 'spill.reported'
  | 'safety.incident'
  | 'permit.expiring'
  | 'compliance.violation';

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
  endpoint?: string; // Default: https://api.wia.org/ene-036/v1
  timeout?: number; // milliseconds
  retries?: number;
  operatorId?: string;
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
