/**
 * WIA-DEF-013: NBC Defense - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core CBRN Types
// ============================================================================

/**
 * Geographic coordinates
 */
export interface GeoCoordinate {
  lat: number;
  lon: number;
  altitude?: number;
}

/**
 * CBRN agent types
 */
export type AgentType =
  | 'chemical'
  | 'biological'
  | 'radiological'
  | 'nuclear';

/**
 * Chemical agent subtypes
 */
export type ChemicalAgentType =
  | 'nerve-agent-g'      // G-series (Tabun, Sarin, Soman)
  | 'nerve-agent-v'      // V-series (VX, VE, VG)
  | 'blister-sulfur'     // Sulfur mustard (H, HD)
  | 'blister-nitrogen'   // Nitrogen mustard (HN)
  | 'blister-lewisite'   // Lewisite (L)
  | 'blood-cyanide'      // Hydrogen cyanide, cyanogen chloride
  | 'blood-arsine'       // Arsine
  | 'choking-phosgene'   // Phosgene, diphosgene
  | 'choking-chlorine'   // Chlorine
  | 'incapacitating-bz'  // BZ
  | 'riot-control';      // CS, CN, OC

/**
 * Biological agent subtypes
 */
export type BiologicalAgentType =
  | 'bacteria-anthrax'     // Bacillus anthracis
  | 'bacteria-plague'      // Yersinia pestis
  | 'bacteria-tularemia'   // Francisella tularensis
  | 'bacteria-brucellosis' // Brucella spp.
  | 'virus-smallpox'       // Variola major
  | 'virus-ebola'          // Ebola virus
  | 'virus-marburg'        // Marburg virus
  | 'virus-vhf'            // Viral hemorrhagic fevers
  | 'toxin-botulinum'      // Botulinum toxin
  | 'toxin-ricin'          // Ricin
  | 'toxin-seb'            // Staphylococcal enterotoxin B
  | 'toxin-t2';            // T-2 mycotoxin

/**
 * Radiological isotopes
 */
export type RadiationIsotope =
  | 'cesium-137'
  | 'cobalt-60'
  | 'iridium-192'
  | 'strontium-90'
  | 'iodine-131'
  | 'plutonium-239'
  | 'americium-241'
  | 'polonium-210';

/**
 * Radiation types
 */
export type RadiationType = 'alpha' | 'beta' | 'gamma' | 'neutron' | 'x-ray';

/**
 * CBRN threat levels (1-5)
 */
export type ThreatLevel = 1 | 2 | 3 | 4 | 5;

/**
 * Mission-Oriented Protective Posture levels
 */
export type MOPPLevel = 0 | 1 | 2 | 3 | 4;

// ============================================================================
// Detection and Monitoring
// ============================================================================

/**
 * Sensor reading from detection equipment
 */
export interface SensorReading {
  /** Sensor identifier */
  sensorId: string;

  /** Sensor type */
  sensorType: 'chemical' | 'biological' | 'radiological' | 'multi-sensor';

  /** Reading timestamp */
  timestamp: Date;

  /** Sensor location */
  location: GeoCoordinate;

  /** Reading value */
  value: number;

  /** Unit of measurement */
  unit: string;

  /** Confidence level (0-1) */
  confidence: number;

  /** Sensor status */
  status: 'operational' | 'degraded' | 'failed' | 'calibrating';

  /** Raw data (optional) */
  rawData?: Record<string, unknown>;
}

/**
 * CBRN agent detection request
 */
export interface CBRNDetectionRequest {
  /** Sensor data from detection systems */
  sensorData: SensorReading[];

  /** Location of detection */
  location: GeoCoordinate;

  /** Detection timestamp */
  timestamp: Date;

  /** Environmental conditions */
  weatherData?: WeatherCondition;

  /** Detection mode */
  mode?: 'real-time' | 'laboratory' | 'field-test';
}

/**
 * CBRN agent detection response
 */
export interface CBRNDetectionResponse {
  /** Primary agent type */
  agentType: AgentType;

  /** Specific agent subtype */
  agentSubtype?: ChemicalAgentType | BiologicalAgentType | RadiationIsotope;

  /** Agent concentration */
  concentration: number;

  /** Unit of measurement */
  unit: 'mg/m³' | 'Bq/m³' | 'R/hr' | 'mSv/hr' | 'CFU/m³';

  /** Detection confidence (0-1) */
  confidence: number;

  /** Threat level assessment */
  threatLevel: ThreatLevel;

  /** Detection method */
  detectionMethod: string;

  /** Time of detection */
  detectionTime: Date;

  /** Recommendations */
  recommendations: string[];

  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Weather conditions affecting CBRN dispersion
 */
export interface WeatherCondition {
  /** Temperature in Celsius */
  temperature: number;

  /** Wind speed in km/h */
  windSpeed: number;

  /** Wind direction in degrees (0-360) */
  windDirection: number;

  /** Relative humidity (0-100) */
  humidity: number;

  /** Atmospheric pressure in hPa */
  pressure?: number;

  /** Precipitation in mm/hr */
  precipitation?: number;

  /** Solar radiation in W/m² */
  solarRadiation?: number;
}

// ============================================================================
// Threat Assessment
// ============================================================================

/**
 * Threat assessment request
 */
export interface ThreatAssessmentRequest {
  /** CBRN agent type */
  agentType: AgentType;

  /** Agent concentration */
  concentration: number;

  /** Weather conditions */
  weatherData: WeatherCondition;

  /** Population at risk */
  population: number;

  /** Critical infrastructure */
  infrastructure?: string[];

  /** Release location */
  releaseLocation?: GeoCoordinate;

  /** Time since release */
  timeSinceRelease?: number;
}

/**
 * Threat assessment response
 */
export interface ThreatAssessmentResponse {
  /** Threat level (1-5) */
  level: ThreatLevel;

  /** Description of threat */
  description: string;

  /** Affected area in km² */
  affectedArea: number;

  /** Estimated casualties */
  estimatedCasualties: number;

  /** Estimated fatalities */
  estimatedFatalities?: number;

  /** Time to impact (seconds) */
  timeToImpact: number;

  /** Plume trajectory */
  plumeTrajectory?: GeoCoordinate[];

  /** Recommended actions */
  recommendedActions: Action[];

  /** Evacuation zones */
  evacuationZones?: EvacuationZone[];
}

/**
 * Recommended action for threat response
 */
export interface Action {
  /** Action priority */
  priority: 'immediate' | 'urgent' | 'routine';

  /** Action type */
  type: 'evacuate' | 'shelter-in-place' | 'don-ppe' | 'decontaminate' | 'medical' | 'other';

  /** Action description */
  description: string;

  /** Affected population */
  affectedPopulation?: number;

  /** Time to complete (seconds) */
  estimatedDuration?: number;

  /** Required resources */
  requiredResources?: string[];
}

/**
 * Evacuation zone definition
 */
export interface EvacuationZone {
  /** Zone identifier */
  zoneId: string;

  /** Zone type */
  type: 'hot' | 'warm' | 'cold' | 'evacuation';

  /** Zone boundary (polygon) */
  boundary: GeoCoordinate[];

  /** Estimated population */
  population: number;

  /** Entry restrictions */
  accessControl: 'prohibited' | 'restricted' | 'controlled' | 'open';

  /** Required PPE level */
  requiredPPE?: PPELevel;
}

// ============================================================================
// Protection Equipment
// ============================================================================

/**
 * PPE protection level
 */
export type PPELevel = 'A' | 'B' | 'C' | 'D';

/**
 * PPE item specification
 */
export interface PPEItem {
  /** Item type */
  type: 'suit' | 'mask' | 'gloves' | 'boots' | 'hood' | 'scba' | 'apr' | 'papr';

  /** Item model/name */
  model: string;

  /** Protection level */
  protectionLevel: PPELevel;

  /** Agents protected against */
  protectedAgents: AgentType[];

  /** Duration of protection (seconds) */
  duration: number;

  /** Size/fit information */
  size?: string;

  /** NSN (NATO Stock Number) */
  nsn?: string;
}

/**
 * Protection requirements request
 */
export interface ProtectionRequest {
  /** CBRN agent type */
  agentType: AgentType;

  /** Agent concentration */
  concentration: number;

  /** Exposure duration (seconds) */
  duration: number;

  /** Number of personnel */
  personnel: number;

  /** Mission type */
  missionType?: 'rescue' | 'decon' | 'medical' | 'security' | 'reconnaissance';
}

/**
 * Protection requirements response
 */
export interface ProtectionResponse {
  /** Required MOPP level */
  moppLevel: MOPPLevel;

  /** Required PPE items */
  ppeRequired: PPEItem[];

  /** Work/rest cycle */
  workRestCycle: WorkRestCycle;

  /** Maximum exposure time (seconds) */
  maxExposureTime: number;

  /** Respiratory protection */
  respiratoryProtection: RespiratoryProtection;

  /** Heat stress risk */
  heatStressRisk: 'low' | 'moderate' | 'high' | 'extreme';

  /** Additional precautions */
  precautions: string[];
}

/**
 * Work/rest cycle for personnel in PPE
 */
export interface WorkRestCycle {
  /** Work duration (seconds) */
  workDuration: number;

  /** Rest duration (seconds) */
  restDuration: number;

  /** Fluid intake (liters per hour) */
  fluidIntake: number;

  /** Based on temperature (Celsius) */
  temperature: number;

  /** MOPP level */
  moppLevel: MOPPLevel;
}

/**
 * Respiratory protection specification
 */
export interface RespiratoryProtection {
  /** Type of respirator */
  type: 'scba' | 'apr' | 'papr' | 'escape-hood';

  /** Filter specification */
  filter: string;

  /** Protection factor */
  protectionFactor: number;

  /** Duration (seconds) */
  duration: number;

  /** NIOSH approval */
  nioshApproval?: string;
}

// ============================================================================
// Decontamination
// ============================================================================

/**
 * Decontamination plan request
 */
export interface DecontaminationRequest {
  /** Number of affected personnel */
  affectedPersonnel: number;

  /** Contaminant type */
  contaminant: ChemicalAgentType | BiologicalAgentType | RadiationIsotope;

  /** Contamination area type */
  area: 'urban' | 'rural' | 'facility' | 'vehicle' | 'equipment';

  /** Available resources */
  resources: string[];

  /** Priority */
  priority?: 'immediate' | 'operational' | 'thorough';
}

/**
 * Decontamination plan response
 */
export interface DecontaminationPlan {
  /** Plan identifier */
  planId: string;

  /** Decontamination type */
  type: 'immediate' | 'operational' | 'thorough';

  /** Estimated time (minutes) */
  estimatedTime: number;

  /** Throughput (people per hour) */
  throughput: number;

  /** Required supplies */
  requiredSupplies: DeconSupply[];

  /** Personnel requirements */
  personnelRequired: number;

  /** Decon corridor layout */
  corridorLayout: DeconStation[];

  /** Waste management */
  wasteManagement: WasteManagement;

  /** Special considerations */
  specialConsiderations: string[];
}

/**
 * Decontamination supply item
 */
export interface DeconSupply {
  /** Item name */
  item: string;

  /** Quantity needed */
  quantity: number;

  /** Unit of measurement */
  unit: string;

  /** NSN (NATO Stock Number) */
  nsn?: string;

  /** Priority */
  priority: 'critical' | 'essential' | 'optional';
}

/**
 * Decontamination station in corridor
 */
export interface DeconStation {
  /** Station number */
  stationNumber: number;

  /** Station type */
  type: 'triage' | 'disrobe' | 'wash' | 'rinse' | 'dress' | 'medical';

  /** Description */
  description: string;

  /** Time per person (seconds) */
  timePerPerson: number;

  /** Personnel required */
  personnelRequired: number;

  /** Equipment needed */
  equipment: string[];
}

/**
 * Waste management plan
 */
export interface WasteManagement {
  /** Waste type */
  wasteType: 'liquid' | 'solid' | 'mixed';

  /** Estimated volume (liters or kg) */
  estimatedVolume: number;

  /** Containment method */
  containment: string;

  /** Disposal method */
  disposal: string;

  /** Special handling */
  specialHandling?: string[];
}

// ============================================================================
// Medical Countermeasures
// ============================================================================

/**
 * Medical countermeasure request
 */
export interface CountermeasureRequest {
  /** CBRN agent */
  agent: ChemicalAgentType | BiologicalAgentType | RadiationIsotope;

  /** Number of affected individuals */
  population: number;

  /** Exposure severity */
  severity: 'mild' | 'moderate' | 'severe';

  /** Time since exposure (seconds) */
  timeSinceExposure?: number;

  /** Available stockpile */
  availableStockpile?: MedicalStockpile[];
}

/**
 * Medical countermeasure response
 */
export interface CountermeasureResponse {
  /** Primary countermeasure */
  primary: MedicalCountermeasure;

  /** Alternative countermeasures */
  alternatives?: MedicalCountermeasure[];

  /** Required quantity */
  requiredQuantity: number;

  /** Administration protocol */
  administrationProtocol: AdministrationProtocol;

  /** Expected outcomes */
  expectedOutcomes: string[];

  /** Contraindications */
  contraindications?: string[];

  /** Stockpile status */
  stockpileStatus: 'adequate' | 'limited' | 'insufficient' | 'depleted';
}

/**
 * Medical countermeasure (antidote, vaccine, treatment)
 */
export interface MedicalCountermeasure {
  /** Countermeasure name */
  name: string;

  /** Generic name */
  genericName?: string;

  /** Type */
  type: 'antidote' | 'vaccine' | 'antibiotic' | 'antitoxin' | 'chelator' | 'supportive';

  /** Dosage */
  dosage: string;

  /** Route of administration */
  route: 'IV' | 'IM' | 'PO' | 'SC' | 'inhalation' | 'topical';

  /** Frequency */
  frequency: string;

  /** Duration */
  duration: string;

  /** NSN (NATO Stock Number) */
  nsn?: string;

  /** Efficacy (0-1) */
  efficacy: number;
}

/**
 * Medical stockpile inventory
 */
export interface MedicalStockpile {
  /** Item name */
  item: string;

  /** Quantity available */
  quantity: number;

  /** Expiration date */
  expiration: Date;

  /** Location */
  location: string;

  /** Lot number */
  lotNumber?: string;
}

/**
 * Administration protocol
 */
export interface AdministrationProtocol {
  /** Step-by-step instructions */
  steps: string[];

  /** Timing requirements */
  timing: string;

  /** Monitoring requirements */
  monitoring: string[];

  /** Side effects to watch for */
  sideEffects: string[];

  /** When to seek advanced care */
  escalationCriteria: string[];
}

// ============================================================================
// Incident Response
// ============================================================================

/**
 * CBRN incident report
 */
export interface CBRNIncident {
  /** Incident identifier */
  incidentId: string;

  /** Incident timestamp */
  timestamp: Date;

  /** Incident location */
  location: GeoCoordinate;

  /** Agent type */
  agentType: AgentType;

  /** Agent subtype */
  agentSubtype?: string;

  /** Concentration/dose */
  concentration: number;

  /** Unit */
  unit: string;

  /** Threat level */
  threatLevel: ThreatLevel;

  /** Affected area (km²) */
  affectedArea: number;

  /** Casualty information */
  casualties: CasualtyInformation;

  /** Response status */
  responseStatus: ResponseStatus;

  /** Weather conditions */
  weatherConditions?: WeatherCondition;

  /** Additional notes */
  notes?: string;
}

/**
 * Casualty information
 */
export interface CasualtyInformation {
  /** Estimated total casualties */
  estimated: number;

  /** Confirmed casualties */
  confirmed: number;

  /** Fatalities */
  fatalities: number;

  /** Hospitalized */
  hospitalized?: number;

  /** Walking wounded */
  ambulatory?: number;

  /** Requiring decontamination */
  requireDecon?: number;
}

/**
 * Response status
 */
export interface ResponseStatus {
  /** Current MOPP level */
  moppLevel: MOPPLevel;

  /** Decontamination status */
  deconStatus: 'not-started' | 'in-progress' | 'completed';

  /** Medical countermeasures dispensed */
  medicalCountermeasures: string[];

  /** Evacuation status */
  evacuationStatus: 'not-started' | 'in-progress' | 'completed';

  /** Incident command established */
  incidentCommandEstablished: boolean;

  /** Hot/warm/cold zones defined */
  zonesEstablished: boolean;
}

/**
 * Emergency response plan request
 */
export interface EmergencyResponseRequest {
  /** Scenario type */
  scenario: string;

  /** Location */
  location: GeoCoordinate;

  /** Expected threat level */
  expectedThreat: ThreatLevel;

  /** Population at risk */
  population: number;

  /** Available resources */
  availableResources?: string[];

  /** Time constraints */
  timeConstraints?: number;
}

/**
 * Emergency response plan
 */
export interface EmergencyResponsePlan {
  /** Plan identifier */
  planId: string;

  /** Incident command structure */
  incidentCommand: IncidentCommand;

  /** Phases of response */
  phases: ResponsePhase[];

  /** Resource requirements */
  resourceRequirements: ResourceRequirement[];

  /** Estimated timeline (minutes) */
  estimatedTimeline: number;

  /** Success criteria */
  successCriteria: string[];

  /** Contingencies */
  contingencies: Contingency[];
}

/**
 * Incident command structure
 */
export interface IncidentCommand {
  /** Incident commander */
  commander: string;

  /** Safety officer */
  safetyOfficer?: string;

  /** Public information officer */
  publicInfoOfficer?: string;

  /** Operations section */
  operations?: string;

  /** Planning section */
  planning?: string;

  /** Logistics section */
  logistics?: string;

  /** Finance/admin section */
  financeAdmin?: string;
}

/**
 * Response phase
 */
export interface ResponsePhase {
  /** Phase number */
  phaseNumber: number;

  /** Phase name */
  name: string;

  /** Description */
  description: string;

  /** Start time (minutes from incident) */
  startTime: number;

  /** Duration (minutes) */
  duration: number;

  /** Key activities */
  keyActivities: string[];

  /** Success criteria */
  successCriteria: string[];
}

/**
 * Resource requirement
 */
export interface ResourceRequirement {
  /** Resource type */
  type: string;

  /** Quantity */
  quantity: number;

  /** Priority */
  priority: 'critical' | 'essential' | 'optional';

  /** When needed (minutes from incident) */
  whenNeeded: number;

  /** Alternative resources */
  alternatives?: string[];
}

/**
 * Contingency plan
 */
export interface Contingency {
  /** Trigger condition */
  trigger: string;

  /** Alternative action */
  action: string;

  /** Resources required */
  resources: string[];

  /** Impact on timeline */
  timelineImpact: number;
}

// ============================================================================
// Physical Constants and Reference Data
// ============================================================================

/**
 * CBRN protection constants
 */
export const CBRN_CONSTANTS = {
  /** LD50 values for common agents (mg for dermal, mg-min/m³ for inhalation) */
  LD50: {
    VX_DERMAL: 10,
    SARIN_INHALATION: 70,
    MUSTARD_DERMAL: 100,
  },

  /** Decontamination solution concentrations */
  DECON_SOLUTIONS: {
    BLEACH_SLURRY: 0.05,      // 5% calcium hypochlorite
    BLEACH_LIQUID: 0.005,     // 0.5% sodium hypochlorite
    SOAP_WATER: 0.01,         // 1% detergent
  },

  /** PPE duration limits (seconds) */
  PPE_DURATION: {
    MOPP4_COLD: 28800,        // 8 hours < 75°F
    MOPP4_MODERATE: 14400,    // 4 hours 75-85°F
    MOPP4_HOT: 7200,          // 2 hours > 85°F
    SCBA: 1800,               // 30 minutes typical
  },

  /** Radiation dose limits (rem) */
  RADIATION_LIMITS: {
    ANNUAL_PUBLIC: 0.1,
    ANNUAL_WORKER: 5,
    EMERGENCY_LIFESAVING: 25,
    LD50_60: 450,             // LD50/60 days
  },

  /** Decontamination throughput (people per hour per lane) */
  DECON_THROUGHPUT: {
    AMBULATORY: 120,
    NON_AMBULATORY: 40,
  },
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-DEF-013 error codes
 */
export enum NBCErrorCode {
  SENSOR_MALFUNCTION = 'D001',
  UNKNOWN_AGENT = 'D002',
  CONCENTRATION_OUT_OF_RANGE = 'D003',
  INSUFFICIENT_PPE = 'P001',
  HEAT_STRESS_WARNING = 'P002',
  ANTIDOTE_SHORTAGE = 'M001',
  PATIENT_OVERLOAD = 'M002',
  DECON_FAILURE = 'C001',
  ZONE_BREACH = 'C002',
  INVALID_PARAMETERS = 'E001',
}

/**
 * NBC Defense error
 */
export class NBCDefenseError extends Error {
  constructor(
    public code: NBCErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'NBCDefenseError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  GeoCoordinate,
  AgentType,
  ChemicalAgentType,
  BiologicalAgentType,
  RadiationIsotope,
  RadiationType,
  ThreatLevel,
  MOPPLevel,

  // Detection
  SensorReading,
  CBRNDetectionRequest,
  CBRNDetectionResponse,
  WeatherCondition,

  // Threat Assessment
  ThreatAssessmentRequest,
  ThreatAssessmentResponse,
  Action,
  EvacuationZone,

  // Protection
  PPELevel,
  PPEItem,
  ProtectionRequest,
  ProtectionResponse,
  WorkRestCycle,
  RespiratoryProtection,

  // Decontamination
  DecontaminationRequest,
  DecontaminationPlan,
  DeconSupply,
  DeconStation,
  WasteManagement,

  // Medical
  CountermeasureRequest,
  CountermeasureResponse,
  MedicalCountermeasure,
  MedicalStockpile,
  AdministrationProtocol,

  // Incident Response
  CBRNIncident,
  CasualtyInformation,
  ResponseStatus,
  EmergencyResponseRequest,
  EmergencyResponsePlan,
  IncidentCommand,
  ResponsePhase,
  ResourceRequirement,
  Contingency,
};

export { CBRN_CONSTANTS, NBCErrorCode, NBCDefenseError };
