/**
 * WIA-ENE-037: Seabed Resource Development Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license CC BY 4.0
 * @description Type definitions for seabed resource exploration, mining operations,
 *              and environmental monitoring
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

// ==================== Resource Type Classification ====================

/**
 * Seabed resource types
 */
export type SeabedResourceType =
  | 'polymetallic_nodules' // 다금속 단괴
  | 'seafloor_massive_sulfides' // 해저 열수광상
  | 'cobalt_crusts' // 코발트 각
  | 'methane_hydrates' // 메탄 하이드레이트
  | 'placer_deposits' // 사광상
  | 'phosphorites'; // 인광석

/**
 * License types
 */
export type LicenseType =
  | 'exploration' // 탐사
  | 'development' // 개발
  | 'exploitation'; // 상업 채굴

/**
 * Mining zone classification
 */
export type MiningZone =
  | 'continental_shelf' // 0-200m
  | 'continental_slope' // 200-2,000m
  | 'continental_rise' // 2,000-4,000m
  | 'abyssal_plain' // 4,000-6,000m
  | 'hadal_zone'; // 6,000-11,000m

/**
 * ISA regulatory regions
 */
export type ISARegion =
  | 'CCZ' // Clarion-Clipperton Zone
  | 'Indian_Ocean'
  | 'Mid_Atlantic_Ridge'
  | 'Western_Pacific'
  | 'Peru_Basin'
  | 'other';

// ==================== Core Data Structures ====================

/**
 * Geographic coordinates
 */
export interface Coordinates {
  latitude: number; // Decimal degrees
  longitude: number; // Decimal degrees
  datum?: 'WGS84' | 'NAD83';
}

/**
 * Measurement with unit
 */
export interface Measurement {
  value: number;
  unit: string;
}

/**
 * Area measurement
 */
export interface Area {
  value: number;
  unit: 'km²' | 'm²' | 'hectares';
}

/**
 * Depth measurement
 */
export interface Depth {
  value: number;
  unit: 'meters' | 'feet';
}

/**
 * Volume measurement
 */
export interface Volume {
  value: number;
  unit: 'm³' | 'tonnes' | 'kg';
}

/**
 * Flow rate measurement
 */
export interface FlowRate {
  value: number;
  unit: 'tonnes/hour' | 'tonnes/day' | 'm³/hour';
}

/**
 * Temperature measurement
 */
export interface Temperature {
  value: number;
  unit: 'celsius' | 'fahrenheit' | 'kelvin';
}

/**
 * Pressure measurement
 */
export interface Pressure {
  value: number;
  unit: 'bar' | 'psi' | 'kPa' | 'MPa';
}

/**
 * Concentration measurement
 */
export interface Concentration {
  value: number;
  unit: 'mg/L' | 'ppm' | 'ppb' | 'percent';
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
  country: string; // ISO 3166-1 alpha-2
  contact: ContactInfo;
  operatorType?: 'commercial' | 'government' | 'research' | 'joint_venture';
}

// ==================== License Information ====================

/**
 * ISA License
 */
export interface ISALicense {
  licenseNumber: string;
  licenseType: LicenseType;
  authority: 'International_Seabed_Authority' | string;
  areaCode: string; // e.g., "CCZ-12"
  region: ISARegion;
  issueDate: string; // ISO 8601 date
  expiryDate: string; // ISO 8601 date
  areaSize: Area;
  sponsor: {
    country: string;
    sponsoringAuthority: string;
  };
  status: 'active' | 'suspended' | 'expired' | 'relinquished';
  conditions?: string[];
}

/**
 * National permit
 */
export interface NationalPermit {
  permitNumber: string;
  permitType: 'exploration' | 'mining' | 'environmental' | 'vessel';
  authority: string;
  jurisdiction: 'EEZ' | 'territorial_waters' | 'continental_shelf';
  issueDate: string; // ISO 8601 date
  expiryDate: string; // ISO 8601 date
  status: 'active' | 'expired' | 'revoked';
}

// ==================== Location Information ====================

/**
 * Seabed location
 */
export interface SeabedLocation {
  zone: MiningZone;
  region: ISARegion | string;
  coordinates: {
    centerLatitude: number;
    centerLongitude: number;
    datum: 'WGS84' | 'NAD83';
  };
  depthRange: {
    min: number;
    max: number;
    unit: 'meters' | 'feet';
  };
  seabedTopography?:
    | 'abyssal_plain'
    | 'seamount'
    | 'ridge'
    | 'trough'
    | 'plateau'
    | 'slope';
}

// ==================== Resource Assessment ====================

/**
 * Metal grades
 */
export interface MetalGrades {
  nickel?: Concentration;
  copper?: Concentration;
  cobalt?: Concentration;
  manganese?: Concentration;
  zinc?: Concentration;
  lead?: Concentration;
  gold?: { value: number; unit: 'g/t' | 'oz/t' };
  silver?: { value: number; unit: 'g/t' | 'oz/t' };
  platinum?: { value: number; unit: 'g/t' };
  ree?: Concentration; // Rare Earth Elements
  lithium?: Concentration;
  molybdenum?: Concentration;
}

/**
 * Polymetallic nodules characteristics
 */
export interface PolymetallicNodules {
  resourceType: 'polymetallic_nodules';
  noduleDensity: {
    value: number;
    unit: 'kg/m²';
  };
  noduleSize: {
    averageDiameter: { value: number; unit: 'cm' };
    sizeDistribution?: string;
  };
  coverageArea: Area;
  burialDepth?: {
    value: number;
    unit: 'cm';
  };
  metalGrades: MetalGrades;
}

/**
 * Seafloor massive sulfides characteristics
 */
export interface SeafloorMassiveSulfides {
  resourceType: 'seafloor_massive_sulfides';
  depositType: 'active_vent' | 'inactive_vent' | 'extinct_vent';
  ventTemperature?: Temperature;
  depositMass: Volume;
  thickness: {
    average: { value: number; unit: 'meters' };
    maximum: { value: number; unit: 'meters' };
  };
  metalGrades: MetalGrades;
  sulfurContent?: Concentration;
}

/**
 * Cobalt-rich crusts characteristics
 */
export interface CobaltCrusts {
  resourceType: 'cobalt_crusts';
  crustThickness: {
    average: { value: number; unit: 'cm' };
    maximum: { value: number; unit: 'cm' };
  };
  growthRate?: {
    value: number;
    unit: 'mm/million_years';
  };
  substrateType: 'basalt' | 'volcanic_rock' | 'sediment';
  coverageArea: Area;
  metalGrades: MetalGrades;
}

/**
 * Methane hydrates characteristics
 */
export interface MethaneHydrates {
  resourceType: 'methane_hydrates';
  hydrateType: 'pore_filling' | 'fracture_filling' | 'nodular' | 'massive';
  hydrateConcentration: Concentration;
  gasContent: {
    value: number;
    unit: 'm³_CH4/m³_hydrate';
  };
  stabilityConditions: {
    temperature: Temperature;
    pressure: Pressure;
  };
  estimatedVolume: Volume;
  gasComposition?: {
    methane: Concentration;
    ethane?: Concentration;
    propane?: Concentration;
    carbonDioxide?: Concentration;
  };
}

/**
 * Resource estimate
 */
export interface ResourceEstimate {
  estimateId: string;
  timestamp: string; // ISO 8601 datetime
  resourceType: SeabedResourceType;
  surveyArea: Area;
  resource:
    | PolymetallicNodules
    | SeafloorMassiveSulfides
    | CobaltCrusts
    | MethaneHydrates;
  totalResource: Volume;
  containedMetal?: {
    [metal: string]: Volume;
  };
  confidenceLevel?: 'inferred' | 'indicated' | 'measured';
  assessmentMethod?: 'visual_survey' | 'sampling' | 'geophysical' | 'modeling';
}

// ==================== Exploration Operations ====================

/**
 * Survey type
 */
export type SurveyType =
  | 'bathymetric' // 해저 지형
  | 'geophysical' // 지구물리
  | 'geochemical' // 지구화학
  | 'biological' // 생물학적
  | 'visual' // 시각 조사
  | 'sampling'; // 샘플링

/**
 * Exploration survey
 */
export interface ExplorationSurvey {
  surveyId: string;
  timestamp: string; // ISO 8601 datetime
  licenseNumber: string;
  surveyType: SurveyType;
  surveyArea: Area;
  surveyPeriod: {
    startDate: string; // ISO 8601 date
    endDate: string; // ISO 8601 date
  };
  vessel?: {
    vesselName: string;
    vesselType: string;
    imo?: string;
  };
  equipment: string[];
  dataCollected: {
    bathymetry?: boolean;
    sideScanSonar?: boolean;
    subBottomProfiler?: boolean;
    magnetometer?: boolean;
    samples?: number;
    videoFootage?: { value: number; unit: 'hours' };
  };
}

/**
 * Sample information
 */
export interface Sample {
  sampleId: string;
  timestamp: string; // ISO 8601 datetime
  sampleType: 'nodule' | 'crust' | 'sulfide' | 'sediment' | 'water' | 'hydrate';
  location: {
    coordinates: Coordinates;
    depth: Depth;
  };
  collectionMethod: 'ROV' | 'AUV' | 'dredge' | 'corer' | 'grab';
  weight?: Volume;
  analysis?: {
    metalGrades?: MetalGrades;
    mineralogy?: string[];
    geochemistry?: Record<string, Concentration>;
  };
}

// ==================== Mining Operations ====================

/**
 * Mining system type
 */
export type MiningSystemType =
  | 'tracked_collector' // 궤도식 집광기
  | 'wheeled_collector' // 바퀴식 집광기
  | 'suction_dredge' // 흡입 준설
  | 'hydraulic_lift' // 유압 양광
  | 'continuous_line_bucket'; // 연속 버킷

/**
 * Mining system
 */
export interface MiningSystem {
  systemId: string;
  timestamp: string; // ISO 8601 datetime
  operatorId: string;
  operatorName: string;
  license: ISALicense;
  location: SeabedLocation;
  miningVessel: {
    vesselName: string;
    vesselType: string;
    imo?: string;
    length: Measurement;
    displacement: Volume;
    dynamicPositioning: 'DP-1' | 'DP-2' | 'DP-3';
    crew: number;
  };
  collectorSystem: {
    collectorType: MiningSystemType;
    operatingDepth: Depth;
    collectionWidth: Measurement;
    travelSpeed: { value: number; unit: 'm/s' };
    hydraulicPower: { value: number; unit: 'kW' };
    autonomy: 'manual' | 'semi-autonomous' | 'autonomous';
  };
  riserSystem: {
    riserType: 'vertical_transport_pipe' | 'flexible_riser' | 'airlift';
    length: Measurement;
    diameter: Measurement;
    material: string;
    pumpingSystem: 'airlift_pump' | 'slurry_pump' | 'hydraulic';
    flowRate: FlowRate;
  };
}

/**
 * Daily production data
 */
export interface DailyProduction {
  productionId: string;
  systemId: string;
  date: string; // ISO 8601 date
  operatingHours: number;
  production: {
    nodulesCollected?: {
      wetWeight: Volume;
      dryWeight: Volume;
      moistureContent: Concentration;
    };
    sulfideCollected?: Volume;
    crustCollected?: Volume;
    hydrateCollected?: Volume;
    sedimentCollected: Volume;
    coverageArea: Area;
    collectorTrackLength?: Measurement;
  };
  metalContent?: {
    [metal: string]: Volume;
  };
  systemPerformance: {
    collectorUptime: Concentration; // percent
    riserEfficiency: Concentration; // percent
    processingRate: FlowRate;
  };
  weatherConditions?: {
    waveHeight: Measurement;
    windSpeed: Measurement;
    seaState: number; // Beaufort scale 0-12
  };
}

// ==================== Environmental Monitoring ====================

/**
 * Baseline study
 */
export interface BaselineStudy {
  studyId: string;
  studyPeriod: {
    startDate: string; // ISO 8601 date
    endDate: string; // ISO 8601 date
    duration: Measurement;
  };
  surveyArea: Area;
  parameters: {
    physicalOceanography: {
      temperature: boolean;
      salinity: boolean;
      currents: boolean;
      turbidity: boolean;
    };
    geochemistry: {
      sedimentComposition: boolean;
      porewaterChemistry: boolean;
      metalConcentrations: boolean;
      organicCarbon: boolean;
    };
    biology: {
      megafauna?: {
        surveyed: boolean;
        speciesIdentified?: number;
        density?: Measurement;
        biomass?: Measurement;
      };
      macrofauna?: {
        surveyed: boolean;
        speciesIdentified?: number;
        density?: Measurement;
      };
      meiofauna?: {
        surveyed: boolean;
        density?: Measurement;
      };
      microbes?: {
        surveyed: boolean;
        diversity?: string;
      };
    };
    ecosystemFunctions?: {
      primaryProductivity?: boolean;
      bioturbation?: boolean;
      nutrientCycling?: boolean;
      carbonSequestration?: boolean;
    };
  };
}

/**
 * Sediment plume monitoring
 */
export interface SedimentPlume {
  monitoringId: string;
  timestamp: string; // ISO 8601 datetime
  plumeType: 'collector_plume' | 'discharge_plume' | 'natural';
  measurements: {
    turbidity: {
      background: Concentration;
      nearField: Concentration;
      farField: Concentration;
    };
    suspendedSediment: {
      concentration: Concentration;
      settlingRate?: { value: number; unit: 'mm/s' };
    };
    plumeExtent: {
      length: Measurement;
      width: Measurement;
      height: Measurement;
    };
    dispersalPattern?: {
      direction: number; // degrees
      currentSpeed: { value: number; unit: 'm/s' };
    };
  };
  exceedanceEvents?: {
    turbidityThreshold: Concentration;
    exceeded: boolean;
    duration?: Measurement;
  };
}

/**
 * Biological monitoring
 */
export interface BiologicalMonitoring {
  monitoringId: string;
  reportingPeriod: {
    startDate: string; // ISO 8601 date
    endDate: string; // ISO 8601 date
  };
  impactAssessment: {
    directImpact: {
      areaCovered: Area;
      habitatLoss: Concentration; // percent
      recoveryTime: Measurement;
    };
    indirectImpact?: {
      sedimentationArea: Area;
      burialDepth: Measurement;
      mortalityRate: Concentration;
    };
  };
  biodiversity: {
    megafaunaAbundance?: {
      miningArea: Measurement;
      referenceArea: Measurement;
      reduction: Concentration;
    };
    endemicSpecies?: {
      identified: number;
      threatened: number;
      protectionMeasures?: string;
    };
  };
  noiseImpact?: {
    sourceLevel: { value: number; unit: 'dB re 1 µPa @ 1m' };
    frequency: { value: number; unit: 'Hz' };
    impactRadius: Measurement;
  };
}

/**
 * Water quality monitoring
 */
export interface WaterQuality {
  monitoringId: string;
  timestamp: string; // ISO 8601 datetime
  location: {
    coordinates: Coordinates;
    depth: Depth;
  };
  measurements: {
    temperature: Temperature;
    salinity: { value: number; unit: 'psu' };
    dissolvedOxygen: Concentration;
    pH: number;
    turbidity: Concentration;
    metalConcentrations?: {
      [metal: string]: Concentration;
    };
  };
}

/**
 * Environmental management plan
 */
export interface EnvironmentalManagementPlan {
  planId: string;
  licenseNumber: string;
  version: string;
  effectiveDate: string; // ISO 8601 date
  spatialManagement: {
    conservationReferenceZones: Area; // % of license area
    impactReferenceZones: Area;
    preservationZones?: Area;
  };
  temporalManagement: {
    seasonalRestrictions?: string[];
    continuousMiningLimit?: Measurement;
    recoveryPeriods?: Measurement;
  };
  technicalMitigation: {
    sedimentReplacementSystem: boolean;
    optimizedCollector: boolean;
    realTimeMonitoring: boolean;
  };
  thresholds: EnvironmentalThresholds;
  adaptiveManagement: {
    reviewFrequency: string;
    triggerCriteria: string[];
    emergencyShutdownProtocol: boolean;
  };
}

/**
 * Environmental thresholds
 */
export interface EnvironmentalThresholds {
  turbidity: {
    baseline: Concentration;
    warning: Concentration;
    danger: Concentration;
    shutdown: Concentration;
  };
  sedimentationRate: {
    baseline: { value: number; unit: 'mm/day' };
    warning: { value: number; unit: 'mm/day' };
    danger: { value: number; unit: 'mm/day' };
    shutdown: { value: number; unit: 'mm/day' };
  };
  dissolvedOxygen: {
    baseline: Concentration;
    warning: Concentration;
    danger: Concentration;
    shutdown: Concentration;
  };
  pH: {
    baseline: { min: number; max: number };
    warning: { min: number; max: number };
    danger: { min: number; max: number };
  };
  noise: {
    baseline: { value: number; unit: 'dB' };
    warning: { value: number; unit: 'dB' };
    danger: { value: number; unit: 'dB' };
  };
}

// ==================== ROV/AUV Operations ====================

/**
 * Vehicle type
 */
export type VehicleType = 'ROV' | 'AUV' | 'HOV';

/**
 * Mission type
 */
export type MissionType =
  | 'sample_collection'
  | 'video_survey'
  | 'bathymetric_mapping'
  | 'environmental_monitoring'
  | 'equipment_installation'
  | 'equipment_inspection';

/**
 * ROV operation
 */
export interface ROVOperation {
  rovId: string;
  rovType: 'work_class_rov' | 'observation_rov' | 'heavy_work_rov';
  specifications: {
    maxDepth: Depth;
    payload: Volume;
    thrusters: number;
    power: { value: number; unit: 'kW' };
    umbilicalLength: Measurement;
  };
  mission: {
    missionId: string;
    missionType: MissionType;
    startTime: string; // ISO 8601 datetime
    endTime: string; // ISO 8601 datetime
    duration: Measurement;
    maxDepth: Depth;
  };
  tasks: {
    taskType: string;
    samplesCollected?: number;
    successRate?: Concentration;
    coresCollected?: number;
    coreDepth?: Measurement;
    trackLength?: Measurement;
    videoQuality?: string;
    measurements?: string[];
  }[];
}

/**
 * AUV operation
 */
export interface AUVOperation {
  auvId: string;
  auvType: 'survey_auv' | 'hybrid_auv';
  specifications: {
    maxDepth: Depth;
    speed: { value: number; unit: 'm/s' };
    endurance: Measurement;
    range: Measurement;
    sensors: string[];
  };
  mission: {
    missionId: string;
    missionType: MissionType;
    surveyPattern: 'lawnmower' | 'spiral' | 'adaptive';
    lineSpacing?: Measurement;
    altitude?: Measurement;
    coverageArea: Area;
    dataQuality: 'high_resolution' | 'medium_resolution' | 'low_resolution';
  };
}

// ==================== Emergency Response ====================

/**
 * Environmental incident type
 */
export type EnvironmentalIncidentType =
  | 'sediment_plume_exceedance'
  | 'equipment_failure'
  | 'spill'
  | 'biodiversity_impact'
  | 'threshold_violation'
  | 'other';

/**
 * Emergency response
 */
export interface EmergencyResponse {
  incidentId: string;
  timestamp: string; // ISO 8601 datetime
  incidentType: EnvironmentalIncidentType;
  severity: 'minor' | 'moderate' | 'major' | 'critical';
  location: {
    coordinates: Coordinates;
    depth?: Depth;
  };
  triggerLevel?: {
    parameter: string;
    threshold: Measurement;
    exceedance: Measurement;
  };
  responseActions: {
    action: string;
    timestamp: string; // ISO 8601 datetime
    duration?: Measurement;
    responsible?: string;
  }[];
  resumptionCriteria?: {
    [key: string]: any;
    regulatorApproval: boolean;
  };
  resolved: boolean;
  resolvedAt?: string; // ISO 8601 datetime
}

// ==================== Reporting ====================

/**
 * Annual report
 */
export interface AnnualReport {
  reportId: string;
  reportingYear: number;
  operatorId: string;
  licenseNumber: string;
  submissionDate: string; // ISO 8601 date
  summary: {
    explorationActivities?: {
      rovDives?: number;
      auvMissions?: number;
      areaSurveyed?: Area;
      samplesCollected?: number;
    };
    miningActivities?: {
      operatingDays?: number;
      totalProduction?: Volume;
      areaMined?: Area;
    };
    resourceAssessment?: {
      estimatedResource?: Volume;
      averageGrade?: MetalGrades;
    };
    environmentalMonitoring: {
      baselineStations?: number;
      biodiversitySurveys?: number;
      waterQualityMeasurements?: number;
      newSpeciesIdentified?: number;
      incidents?: number;
    };
    expenditure: {
      exploration?: { value: number; currency: string };
      mining?: { value: number; currency: string };
      environmental: { value: number; currency: string };
      research?: { value: number; currency: string };
    };
  };
}

// ==================== Main System Structure ====================

/**
 * Seabed resource system
 */
export interface SeabedResourceSystem {
  systemId: string;
  timestamp: string; // ISO 8601 datetime
  resourceType: SeabedResourceType;
  operator: Operator;
  license: ISALicense;
  nationalPermits?: NationalPermit[];
  location: SeabedLocation;
  phase: 'exploration' | 'development' | 'exploitation' | 'closure';
  exploration?: {
    surveys: ExplorationSurvey[];
    samples: Sample[];
    resourceEstimate?: ResourceEstimate;
  };
  mining?: MiningSystem;
  production?: {
    dailyRecords: DailyProduction[];
    cumulativeProduction?: Volume;
  };
  environmental: {
    baselineStudy?: BaselineStudy;
    managementPlan: EnvironmentalManagementPlan;
    monitoring: {
      sedimentPlumes?: SedimentPlume[];
      biologicalMonitoring?: BiologicalMonitoring[];
      waterQuality?: WaterQuality[];
    };
    incidents?: EmergencyResponse[];
  };
  vehicles?: {
    rovs?: ROVOperation[];
    auvs?: AUVOperation[];
  };
  reporting: {
    annualReports: AnnualReport[];
    lastSubmission?: string; // ISO 8601 date
  };
  metadata?: {
    createdBy?: string;
    lastModified?: string;
    version?: string;
    [key: string]: any;
  };
}

// ==================== API Request/Response Types ====================

/**
 * Create license request
 */
export interface CreateLicenseRequest {
  operatorId: string;
  licenseType: LicenseType;
  resourceType: SeabedResourceType;
  region: ISARegion;
  proposedArea: Area;
  location: SeabedLocation;
}

/**
 * Create license response
 */
export interface CreateLicenseResponse {
  licenseNumber: string;
  timestamp: string;
  status: 'success' | 'pending' | 'error';
  message?: string;
}

/**
 * Submit exploration survey request
 */
export interface SubmitExplorationSurveyRequest {
  licenseNumber: string;
  survey: Omit<ExplorationSurvey, 'surveyId' | 'timestamp'>;
}

/**
 * Submit sample request
 */
export interface SubmitSampleRequest {
  licenseNumber: string;
  sample: Omit<Sample, 'sampleId' | 'timestamp'>;
}

/**
 * Submit production request
 */
export interface SubmitProductionRequest {
  systemId: string;
  production: Omit<DailyProduction, 'productionId'>;
}

/**
 * Submit environmental data request
 */
export interface SubmitEnvironmentalDataRequest {
  systemId: string;
  licenseNumber: string;
  reportingPeriod: {
    startDate: string;
    endDate: string;
  };
  monitoringData: {
    sedimentPlumes?: Omit<SedimentPlume, 'monitoringId'>[];
    biologicalMonitoring?: Omit<BiologicalMonitoring, 'monitoringId'>[];
    waterQuality?: Omit<WaterQuality, 'monitoringId'>[];
  };
}

/**
 * Report incident request
 */
export interface ReportIncidentRequest {
  systemId: string;
  incidentType: EnvironmentalIncidentType;
  severity: 'minor' | 'moderate' | 'major' | 'critical';
  location: {
    coordinates: Coordinates;
    depth?: Depth;
  };
  description: string;
  immediateActions: string;
}

/**
 * Query parameters
 */
export interface QueryParams {
  operatorId?: string;
  licenseNumber?: string;
  resourceType?: SeabedResourceType;
  region?: ISARegion;
  phase?: string;
  startDate?: string;
  endDate?: string;
  page?: number;
  limit?: number;
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
 * Compliance status
 */
export interface ComplianceStatus {
  systemId: string;
  licenseNumber: string;
  asOfDate: string; // ISO 8601 date
  licenses: {
    total: number;
    active: number;
    expired: number;
    expiringSoon: number;
  };
  reporting: {
    annualReports: {
      required: number;
      submitted: number;
      overdue: number;
    };
    environmentalReports: {
      required: number;
      submitted: number;
      overdue: number;
    };
  };
  environmental: {
    thresholdViolations: number;
    incidents: number;
    resolved: number;
  };
  overallCompliance: 'compliant' | 'non_compliant' | 'conditional';
}

/**
 * Operator dashboard
 */
export interface OperatorDashboard {
  operatorId: string;
  operatorName: string;
  timestamp: string; // ISO 8601 datetime
  summary: {
    totalSystems: number;
    activeExploration: number;
    activeMining: number;
    totalLicenseArea: Area;
  };
  production?: {
    daily?: Volume;
    monthly?: Volume;
    yearly?: Volume;
  };
  environmental: {
    activeMonitoringStations: number;
    thresholdViolations: number;
    incidents: number;
  };
  compliance: {
    licenseCompliance: number; // percent
    reportingCompliance: number; // percent
    environmentalScore: number; // 0-100
  };
}

// ==================== Webhook Types ====================

/**
 * Webhook event types
 */
export type WebhookEventType =
  | 'license.created'
  | 'license.expiring'
  | 'production.submitted'
  | 'environmental.threshold_exceeded'
  | 'environmental.incident'
  | 'compliance.violation'
  | 'report.due';

/**
 * Webhook payload
 */
export interface WebhookPayload<T = any> {
  event: WebhookEventType;
  timestamp: string; // ISO 8601 datetime
  data: T;
  signature?: string;
}

// ==================== Client Configuration ====================

/**
 * API client configuration
 */
export interface ClientConfig {
  apiKey: string;
  endpoint?: string; // Default: https://api.wia.org/ene-037/v1
  timeout?: number;
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
  timestamp: string;
  requestId?: string;
}

// ==================== Export All ====================

export default {
  // Types are exported individually above
};
