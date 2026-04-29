/**
 * WIA-OCEAN_CONSERVATION TypeScript Type Definitions
 * Version 1.0
 * 弘益人間 (Benefit All Humanity)
 *
 * Comprehensive type definitions for ocean conservation data,
 * marine species tracking, ecosystem monitoring, and MPA management.
 */

import { GeoJSON, Polygon, Point } from 'geojson';

// ============================================================================
// Core Location and Geographic Types
// ============================================================================

export interface Location {
  latitude: number;
  longitude: number;
  depth?: number;
  coordinateSystem?: 'WGS84' | 'NAD83';
  accuracy?: number; // meters
}

export interface BoundingBox {
  minLatitude: number;
  minLongitude: number;
  maxLatitude: number;
  maxLongitude: number;
}

export interface GeographicArea {
  name: string;
  boundary: Polygon;
  area: number; // km²
  description?: string;
}

// ============================================================================
// Marine Species Types
// ============================================================================

export type ConservationStatus =
  | 'EX'  // Extinct
  | 'EW'  // Extinct in the Wild
  | 'CR'  // Critically Endangered
  | 'EN'  // Endangered
  | 'VU'  // Vulnerable
  | 'NT'  // Near Threatened
  | 'LC'  // Least Concern
  | 'DD'  // Data Deficient
  | 'NE'; // Not Evaluated

export type ObservationType =
  | 'VISUAL'
  | 'ACOUSTIC'
  | 'SATELLITE'
  | 'CAMERA_TRAP'
  | 'DNA'
  | 'DRONE';

export type Behavior =
  | 'FEEDING'
  | 'BREEDING'
  | 'MIGRATING'
  | 'RESTING'
  | 'SOCIALIZING'
  | 'UNKNOWN';

export type AgeClass =
  | 'JUVENILE'
  | 'SUBADULT'
  | 'ADULT'
  | 'UNKNOWN';

export type HealthCondition =
  | 'HEALTHY'
  | 'INJURED'
  | 'DISEASED'
  | 'DECEASED';

export interface SpeciesHealth {
  condition: HealthCondition;
  injuries?: string[];
  diseases?: string[];
  parasites?: boolean;
  bodyConditionScore?: number; // 1-5
  notes?: string;
}

export interface Observer {
  observerId: string;
  type: 'RESEARCHER' | 'CITIZEN_SCIENTIST' | 'AUTOMATED_SYSTEM' | 'ENFORCEMENT';
  name?: string;
  credentials?: string;
  organization?: string;
}

export interface MediaAttachment {
  type: 'PHOTO' | 'VIDEO' | 'AUDIO';
  url: string;
  timestamp: string;
  caption?: string;
  metadata?: Record<string, any>;
}

export interface EnvironmentalConditions {
  waterTemperature?: number; // Celsius
  airTemperature?: number; // Celsius
  salinity?: number; // PSU
  visibility?: number; // meters
  currentSpeed?: number; // m/s
  currentDirection?: number; // degrees
  waveHeight?: number; // meters
  weather?: string;
  seaState?: number; // Beaufort scale 0-12
}

export interface SpeciesObservation {
  observationId: string;
  speciesCode: string;
  commonName: string;
  scientificName: string;
  conservationStatus: ConservationStatus;
  timestamp: string;
  location: Location;
  observationType: ObservationType;
  count: number;
  behavior?: Behavior;
  ageClass?: AgeClass;
  health?: SpeciesHealth;
  observer: Observer;
  media?: MediaAttachment[];
  environmentalConditions?: EnvironmentalConditions;
  metadata?: {
    verified: boolean;
    verifiedBy?: string;
    verifiedAt?: string;
    confidence: number;
    tags?: string[];
  };
}

export interface PopulationEstimate {
  populationId: string;
  speciesCode: string;
  region: string;
  estimatedPopulation: number;
  estimationMethod: 'MARK_RECAPTURE' | 'AERIAL_SURVEY' | 'ACOUSTIC_MONITORING' | 'STATISTICAL_MODEL';
  confidenceInterval: {
    lower: number;
    upper: number;
    confidence: number;
  };
  surveyPeriod: {
    start: string;
    end: string;
  };
  populationTrend: 'INCREASING' | 'DECREASING' | 'STABLE' | 'UNKNOWN';
  trendRate?: number; // annual rate of change
  threats?: Threat[];
}

export interface Threat {
  threatType: string;
  severity: 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW';
  affectedPercentage?: number;
  description?: string;
  mitigationActions?: string[];
}

// ============================================================================
// Ecosystem and Habitat Types
// ============================================================================

export type BleachingLevel =
  | 'NONE'
  | 'MILD'
  | 'MODERATE'
  | 'SEVERE'
  | 'TOTAL';

export interface DiversityIndex {
  shannonIndex?: number;
  simpsonIndex?: number;
  margalefIndex?: number;
  pielouEvenness?: number;
}

export interface SpeciesComposition {
  species: string;
  scientificName: string;
  coverage?: number; // percentage
  abundance?: number; // count
  biomass?: number; // kg
}

export interface WaterQuality {
  temperature: number; // Celsius
  pH: number;
  dissolvedOxygen: number; // mg/L
  salinity?: number; // PSU
  turbidity?: number; // NTU
  conductivity?: number; // µS/cm
  nutrientLevels?: {
    nitrate?: number; // mg/L
    nitrite?: number; // mg/L
    phosphate?: number; // mg/L
    ammonia?: number; // mg/L
    silicate?: number; // mg/L
  };
  heavyMetals?: Record<string, number>;
  contaminants?: Record<string, number>;
}

export interface CoralReefAssessment {
  assessmentId: string;
  reefId: string;
  reefName: string;
  location: Location;
  assessmentDate: string;
  coralCoverPercentage: number;
  bleachingLevel: BleachingLevel;
  bleachedPercentage: number;
  diversityIndex: DiversityIndex;
  dominantSpecies: SpeciesComposition[];
  healthIndicators: {
    algaeCoverage: number; // percentage
    diseasePrevalence: number; // percentage
    recruitmentRate: number; // juveniles per m²
    mortality: number; // percentage
    macroalgaeIndex?: number;
  };
  waterQuality: WaterQuality;
  threats: string[];
  restorationActivities?: RestorationActivity[];
  surveyMethod?: 'LINE_INTERCEPT' | 'PHOTO_QUADRAT' | 'POINT_CONTACT' | 'VIDEO_TRANSECT';
  assessor?: Observer;
}

export interface RestorationActivity {
  activityId?: string;
  type: 'CORAL_TRANSPLANTATION' | 'CORAL_NURSERY' | 'ARTIFICIAL_REEF' | 'ALGAE_REMOVAL' | 'PREDATOR_CONTROL';
  area: number; // m²
  date: string;
  coralSpecies?: string[];
  fragmentCount?: number;
  survivalRate?: number; // 0-1
  cost?: number;
  fundingSource?: string;
  status: 'PLANNED' | 'IN_PROGRESS' | 'COMPLETED' | 'MONITORING';
}

export interface OceanAcidificationData {
  measurementId: string;
  location: Location;
  timestamp: string;
  pH: number;
  pCO2: number; // µatm
  totalAlkalinity?: number; // µmol/kg
  dissolvedInorganicCarbon?: number; // µmol/kg
  aragoniteSaturation: number; // Omega aragonite
  calciteSaturation: number; // Omega calcite
  temperature: number;
  salinity: number;
  trend?: {
    pHChangePerDecade: number;
    projectedPH2050?: number;
    projectedPH2100?: number;
  };
}

export interface SeagrassAssessment {
  assessmentId: string;
  location: Location;
  assessmentDate: string;
  species: string[];
  coverage: number; // percentage
  density: number; // shoots per m²
  shootHeight: number; // cm
  epiphyteLoad: 'LOW' | 'MEDIUM' | 'HIGH';
  health: 'EXCELLENT' | 'GOOD' | 'FAIR' | 'POOR';
  threats: string[];
}

export interface MangroveAssessment {
  assessmentId: string;
  location: Location;
  assessmentDate: string;
  species: string[];
  coverageArea: number; // hectares
  canopyHeight: number; // meters
  density: number; // trees per hectare
  health: 'EXCELLENT' | 'GOOD' | 'FAIR' | 'POOR';
  regeneration: number; // saplings per hectare
  threats: string[];
  carbonStock?: number; // tonnes C per hectare
}

// ============================================================================
// Marine Protected Area Types
// ============================================================================

export type IUCNCategory = 'Ia' | 'Ib' | 'II' | 'III' | 'IV' | 'V' | 'VI';

export type ProtectionLevel =
  | 'NO_TAKE'
  | 'NO_TAKE_SEASONAL'
  | 'RESTRICTED'
  | 'MULTIPLE_USE'
  | 'SUSTAINABLE_USE';

export interface MPARegulation {
  type: string;
  description: string;
  penalties?: string;
  exemptions?: string[];
}

export interface MPAZone {
  zoneName: string;
  zoneType: string;
  area: number; // km²
  boundary?: Polygon;
  allowedActivities: string[];
  prohibitedActivities: string[];
  seasonalRestrictions?: string;
}

export interface MarineProtectedArea {
  mpaId: string;
  name: string;
  designation: string;
  iucnCategory?: IUCNCategory;
  boundary: Polygon;
  area: number; // km²
  establishedDate: string;
  managementAuthority: string;
  protectionLevel: ProtectionLevel;
  regulations: MPARegulation[];
  zoning?: MPAZone[];
  biodiversity?: {
    speciesCount: number;
    endemicSpecies: number;
    threatenedSpecies: number;
    keySpecies?: string[];
  };
  monitoring?: {
    frequency: 'DAILY' | 'WEEKLY' | 'MONTHLY' | 'QUARTERLY' | 'ANNUALLY';
    lastSurvey: string;
    complianceRate: number;
    patrolHours?: number;
  };
  managementPlan?: {
    planId: string;
    startDate: string;
    endDate: string;
    objectives: string[];
    budget?: number;
  };
}

export interface MPAEffectiveness {
  mpaId: string;
  evaluationDate: string;
  biologicalIndicators: {
    fishBiomass: number; // kg per hectare
    speciesRichness: number;
    trophicBalance: number;
    sizeStructure?: Record<string, number>;
  };
  socioeconomicIndicators: {
    complianceRate: number;
    communitySupport: number; // percentage
    alternativeLivelihoodParticipation: number;
    tourismRevenue?: number;
    fishingEffortReduction?: number; // percentage
  };
  overallEffectiveness: 'HIGHLY_EFFECTIVE' | 'EFFECTIVE' | 'MODERATELY_EFFECTIVE' | 'INEFFECTIVE';
}

// ============================================================================
// Illegal Fishing and Enforcement Types
// ============================================================================

export type ViolationType =
  | 'UNAUTHORIZED_FISHING'
  | 'GEAR_VIOLATION'
  | 'QUOTA_EXCEED'
  | 'SIZE_LIMIT_VIOLATION'
  | 'SPECIES_BAN_VIOLATION'
  | 'MPA_INTRUSION'
  | 'NO_LICENSE'
  | 'AIS_TAMPERING';

export interface Vessel {
  vesselId?: string;
  name?: string;
  imo?: string;
  mmsi?: string;
  callSign?: string;
  flag?: string;
  type: string;
  length?: number; // meters
  tonnage?: number;
  owner?: string;
  operator?: string;
}

export interface AISPattern {
  gapDuration?: number; // hours
  speedInconsistency?: boolean;
  locationSpoofing?: boolean;
  unusualRoute?: boolean;
  rendezvousDetected?: boolean;
  nighttimeActivity?: boolean;
}

export interface Violation {
  type: ViolationType;
  severity: 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW';
  evidence: string[];
  confidence: number; // 0-1
  estimatedValue?: number; // USD
  description?: string;
}

export interface IllegalFishingDetection {
  detectionId: string;
  timestamp: string;
  detectionMethod: 'SATELLITE' | 'AIS' | 'RADAR' | 'PATROL' | 'DRONE' | 'CITIZEN_REPORT';
  vessel: Vessel;
  location: Location;
  violation: Violation;
  aisPattern?: AISPattern;
  enforcement?: {
    reported: boolean;
    reportedTo: string[];
    caseNumber?: string;
    actionTaken?: string;
    outcomeDate?: string;
    penalty?: number;
  };
}

export interface PatrolLog {
  patrolId: string;
  patrolDate: string;
  patrolType: 'VESSEL' | 'AERIAL' | 'SHORE' | 'UNDERWATER';
  startTime: string;
  endTime: string;
  route: Location[];
  area: string;
  vesselsEncountered: Vessel[];
  violations: Violation[];
  incidentReports?: string[];
  patrolOfficers: string[];
  weather?: string;
  notes?: string;
}

// ============================================================================
// Pollution Types
// ============================================================================

export type PollutionType =
  | 'PLASTIC_DEBRIS'
  | 'MICROPLASTICS'
  | 'OIL_SPILL'
  | 'CHEMICAL'
  | 'NUTRIENT_RUNOFF'
  | 'SEWAGE'
  | 'HEAVY_METALS'
  | 'GHOST_GEAR';

export type PollutionSeverity =
  | 'CATASTROPHIC'
  | 'MAJOR'
  | 'MODERATE'
  | 'MINOR';

export interface PollutantDetails {
  type: string;
  concentration: number;
  unit: string;
  composition?: string[];
  toxicity?: 'VERY_HIGH' | 'HIGH' | 'MODERATE' | 'LOW';
}

export interface PollutionSource {
  type: 'LAND_BASED' | 'MARINE_BASED' | 'ATMOSPHERIC' | 'UNKNOWN';
  identified: boolean;
  responsible?: string;
  location?: Location;
}

export interface PollutionImpact {
  wildlifeAffected: number;
  speciesAffected?: string[];
  habitatDamage: 'SEVERE' | 'MODERATE' | 'MINOR' | 'NONE';
  economicLoss?: number; // USD
  humanHealthRisk?: 'HIGH' | 'MEDIUM' | 'LOW' | 'NONE';
}

export interface CleanupResponse {
  cleanupInitiated: string;
  method: 'MECHANICAL' | 'CHEMICAL' | 'BIOLOGICAL' | 'MANUAL' | 'COMBINED';
  percentageRemoved: number;
  status: 'PLANNED' | 'IN_PROGRESS' | 'COMPLETED' | 'ABANDONED';
  cost?: number;
  volunteers?: number;
  organizations?: string[];
}

export interface PollutionEvent {
  pollutionEventId: string;
  eventType: PollutionType;
  severity: PollutionSeverity;
  detectedDate: string;
  location: Location;
  affectedArea: number; // km²
  pollutantDetails: PollutantDetails;
  source?: PollutionSource;
  impact: PollutionImpact;
  response?: CleanupResponse;
  photos?: string[];
  reportedBy?: Observer;
}

// ============================================================================
// Conservation Alert Types
// ============================================================================

export type AlertType =
  | 'BLEACHING_EVENT'
  | 'BLEACHING_RISK'
  | 'ILLEGAL_FISHING'
  | 'POLLUTION'
  | 'SPECIES_DISTRESS'
  | 'HABITAT_DESTRUCTION'
  | 'ENFORCEMENT_NEEDED'
  | 'STORM_DAMAGE'
  | 'TEMPERATURE_ANOMALY';

export type AlertSeverity = 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW' | 'INFO';

export interface ConservationAlert {
  alertId: string;
  type: AlertType;
  severity: AlertSeverity;
  region: string;
  title: string;
  description: string;
  affectedArea?: number; // km²
  issuedAt: string;
  expiresAt?: string;
  actionRequired?: string;
  responsibleAgency?: string;
  updates?: AlertUpdate[];
  relatedEvents?: string[];
  status: 'ACTIVE' | 'MONITORING' | 'RESOLVED' | 'EXPIRED';
}

export interface AlertUpdate {
  timestamp: string;
  message: string;
  updatedBy: string;
}

export interface AlertSubscription {
  subscriptionId: string;
  subscriberId: string;
  alertTypes: AlertType[];
  regions: string[];
  severityThreshold: AlertSeverity;
  deliveryMethod: 'EMAIL' | 'SMS' | 'WEBHOOK' | 'PUSH';
  webhookUrl?: string;
  email?: string;
  phone?: string;
  active: boolean;
}

// ============================================================================
// Report and Analytics Types
// ============================================================================

export type ReportType =
  | 'SPECIES_POPULATION'
  | 'ECOSYSTEM_HEALTH'
  | 'MPA_EFFECTIVENESS'
  | 'ENFORCEMENT_SUMMARY'
  | 'POLLUTION_ASSESSMENT'
  | 'CONSERVATION_IMPACT'
  | 'ANNUAL_REVIEW';

export interface ReportRequest {
  reportType: ReportType;
  region?: string;
  period: {
    start: string;
    end: string;
  };
  includeMetrics: string[];
  format: 'PDF' | 'HTML' | 'JSON' | 'CSV';
  language?: 'en' | 'es' | 'fr' | 'zh' | 'ar';
}

export interface GeneratedReport {
  reportId: string;
  reportType: ReportType;
  generatedAt: string;
  status: 'GENERATING' | 'COMPLETED' | 'FAILED';
  downloadUrl?: string;
  expiresAt?: string;
  metadata?: Record<string, any>;
}

// ============================================================================
// Configuration and Client Types
// ============================================================================

export interface WIAOceanConservationConfig {
  apiKey: string;
  baseUrl?: string;
  timeout?: number;
  retries?: number;
  version?: string;
}

export interface APIResponse<T> {
  data: T;
  status: number;
  message?: string;
  timestamp: string;
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  limit: number;
  offset: number;
  hasMore: boolean;
}

export interface QueryOptions {
  limit?: number;
  offset?: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

// ============================================================================
// Webhook Types
// ============================================================================

export interface WebhookEvent {
  eventType: string;
  eventId: string;
  timestamp: string;
  data: any;
  signature: string;
}

export interface WebhookConfig {
  url: string;
  events: string[];
  secret: string;
  active?: boolean;
  retryPolicy?: {
    maxRetries: number;
    backoffMultiplier: number;
  };
}

// ============================================================================
// Citizen Science Types
// ============================================================================

export interface CitizenScientist {
  userId: string;
  username: string;
  email?: string;
  level: 'BEGINNER' | 'INTERMEDIATE' | 'ADVANCED' | 'EXPERT';
  contributionCount: number;
  verificationScore: number; // 0-1
  badges?: string[];
  specializations?: string[];
}

export interface CitizenObservation extends SpeciesObservation {
  needsVerification: boolean;
  verificationCount: number;
  communityScore?: number;
}

// ============================================================================
// Research and Study Types
// ============================================================================

export interface ResearchProject {
  projectId: string;
  title: string;
  description: string;
  principalInvestigator: string;
  organization: string;
  startDate: string;
  endDate?: string;
  studyArea: GeographicArea;
  objectives: string[];
  methods: string[];
  funding?: {
    source: string;
    amount: number;
  };
  permits: string[];
  status: 'PLANNING' | 'ACTIVE' | 'COMPLETED' | 'SUSPENDED';
}

export interface FieldSurvey {
  surveyId: string;
  projectId?: string;
  surveyDate: string;
  location: Location;
  surveyType: string;
  objectives: string[];
  methodology: string;
  observations: SpeciesObservation[];
  samples?: SampleCollection[];
  environmentalData?: EnvironmentalConditions;
  team: Observer[];
}

export interface SampleCollection {
  sampleId: string;
  sampleType: 'WATER' | 'SEDIMENT' | 'TISSUE' | 'DNA' | 'PLANKTON';
  collectionDate: string;
  location: Location;
  preservationMethod?: string;
  storageLocation?: string;
  analysis?: {
    laboratoryId: string;
    analysisType: string;
    results?: Record<string, any>;
  };
}

// Export all types
export * from 'geojson';
