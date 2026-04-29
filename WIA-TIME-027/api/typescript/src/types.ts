/**
 * WIA-TIME-027: Traveler Bio-Safety - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geometry Types
// ============================================================================

/**
 * Three-dimensional spatial coordinates
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Destination for time travel
 */
export interface Destination {
  /** Historical era identifier */
  era: string;

  /** Specific year */
  year: number;

  /** Spatial location */
  location: Vector3;

  /** Timeline identifier */
  timeline: string;

  /** Region name */
  region?: string;
}

// ============================================================================
// Bio-Safety Index Types
// ============================================================================

/**
 * Biological Safety Index calculation result
 */
export interface BioSafetyIndex {
  /** Overall Bio-Safety Index (0-1) */
  overall: number;

  /** Component scores */
  components: {
    /** Immune health score (0-1) */
    immuneHealth: number;

    /** Cellular integrity score (0-1) */
    cellularIntegrity: number;

    /** Radiation safety score (0-1) */
    radiationSafety: number;

    /** Pathogen containment score (0-1) */
    pathogenContainment: number;

    /** Decontamination readiness score (0-1) */
    decontaminationReadiness: number;
  };

  /** Component weights used */
  weights: {
    immuneHealth: number;
    cellularIntegrity: number;
    radiationSafety: number;
    pathogenContainment: number;
    decontaminationReadiness: number;
  };

  /** Overall assessment */
  assessment: 'excellent' | 'good' | 'acceptable' | 'marginal' | 'poor' | 'critical';

  /** Calculated at */
  calculatedAt: Date;
}

// ============================================================================
// Bloodwork Types
// ============================================================================

/**
 * Complete blood count and chemistry results
 */
export interface BloodworkResults {
  /** Test date */
  testDate: Date;

  // Complete Blood Count (CBC)
  /** White blood cell count (cells/μL) */
  whiteBloodCells: number;

  /** Red blood cell count (million cells/μL) */
  redBloodCells: number;

  /** Hemoglobin (g/dL) */
  hemoglobin: number;

  /** Hematocrit (%) */
  hematocrit: number;

  /** Platelet count (cells/μL) */
  platelets: number;

  // Differential
  /** Neutrophils (%) */
  neutrophils: number;

  /** Lymphocytes (%) */
  lymphocytes: number;

  /** Monocytes (%) */
  monocytes: number;

  /** Eosinophils (%) */
  eosinophils: number;

  /** Basophils (%) */
  basophils: number;

  // Chemistry
  /** Blood glucose (mg/dL) */
  glucose: number;

  /** Creatinine (mg/dL) */
  creatinine: number;

  /** Bilirubin (mg/dL) */
  bilirubin: number;

  // Liver function
  /** ALT - Alanine aminotransferase (U/L) */
  alt: number;

  /** AST - Aspartate aminotransferase (U/L) */
  ast: number;

  // Immune markers
  /** C-reactive protein (mg/L) */
  cReactiveProtein: number;

  /** Erythrocyte sedimentation rate (mm/hr) */
  sedRate: number;

  /** Detected abnormalities */
  abnormalities: string[];

  /** Overall health score (0-1) */
  overallHealth: number;
}

// ============================================================================
// Immune System Types
// ============================================================================

/**
 * Vaccination record
 */
export interface Vaccination {
  /** Pathogen name */
  pathogen: string;

  /** Vaccine name */
  vaccineName: string;

  /** Vaccination date */
  date: Date;

  /** Booster due date */
  boosterDue?: Date;

  /** Effectiveness (0-1) */
  effectiveness: number;

  /** Lot number */
  lotNumber?: string;

  /** Administrator */
  administeredBy?: string;
}

/**
 * Immune system profile
 */
export interface ImmuneProfile {
  /** Profile date */
  profileDate: Date;

  // T-Cell counts
  /** CD4+ T-cell count (cells/μL) */
  cd4Count: number;

  /** CD8+ T-cell count (cells/μL) */
  cd8Count: number;

  /** CD4/CD8 ratio */
  cd4Cd8Ratio: number;

  // B-Cell and antibodies
  /** B-cell count (cells/μL) */
  bCellCount: number;

  /** Immunoglobulin G (mg/dL) */
  immunoglobulinG: number;

  /** Immunoglobulin A (mg/dL) */
  immunoglobulinA: number;

  /** Immunoglobulin M (mg/dL) */
  immunoglobulinM: number;

  /** Antibody titers for specific pathogens */
  antibodies: Record<string, number>;

  /** Vaccination history */
  vaccinations: Vaccination[];

  /** Known allergies */
  allergies: string[];

  /** Autoimmune conditions */
  autoimmune: string[];

  /** Chronic conditions */
  chronicConditions: string[];

  /** Overall immune function (0-1) */
  overallFunction: number;

  /** Immune competence classification */
  immuneCompetence: 'excellent' | 'good' | 'fair' | 'poor' | 'compromised';
}

// ============================================================================
// Cellular Health Types
// ============================================================================

/**
 * Cellular health assessment
 */
export interface CellularHealth {
  /** Assessment date */
  assessmentDate: Date;

  // DNA stability
  /** DNA integrity score (0-1) */
  dnaIntegrity: number;

  /** Mutation rate (mutations/hour) */
  mutationRate: number;

  /** DNA strand breaks (breaks/cell) */
  strandBreaks: number;

  /** DNA repair efficiency (0-1) */
  repairEfficiency: number;

  // Mitochondrial function
  /** ATP production rate (pmol/min/cell) */
  atpProduction: number;

  /** Oxidative stress level (nmol/L) */
  oxidativeStress: number;

  /** Mitochondria per cell */
  mitochondrialCount: number;

  // Telomere health
  /** Telomere length (kilobases) */
  telomereLength: number;

  /** Telomerase activity (0-1) */
  telomeraseActivity: number;

  /** Telomere erosion rate (bp/day) */
  erosionRate: number;

  // Cell membrane
  /** Membrane integrity (0-1) */
  membraneIntegrity: number;

  /** Membrane permeability (normalized) */
  permeability: number;

  /** Receptor function (0-1) */
  receptorFunction: number;

  // Protein synthesis
  /** Protein synthesis rate (proteins/min) */
  synthesisRate: number;

  /** Protein folding accuracy (0-1) */
  foldingAccuracy: number;

  /** Protein degradation rate (proteins/min) */
  degradationRate: number;

  // Cell death
  /** Apoptosis rate (% cells/day) */
  apoptosisRate: number;

  /** Necrosis rate (% cells/day) */
  necrosisRate: number;

  /** Overall cellular integrity (0-1) */
  overallIntegrity: number;

  /** Expected degradation rate per travel hour */
  expectedDegradation: number;
}

// ============================================================================
// Pathogen Types
// ============================================================================

/**
 * Transmission modes for pathogens
 */
export type TransmissionMode =
  | 'airborne'
  | 'droplet'
  | 'contact'
  | 'fecal-oral'
  | 'vector-borne'
  | 'bloodborne'
  | 'sexual';

/**
 * Historical outbreak information
 */
export interface Outbreak {
  /** Outbreak name */
  name: string;

  /** Year of outbreak */
  year: number;

  /** Geographic location */
  location: string;

  /** Estimated deaths */
  deaths: number;

  /** Description */
  description: string;
}

/**
 * Temporal pathogen definition
 */
export interface TemporalPathogen {
  /** Pathogen identifier */
  pathogenId: string;

  /** Common name */
  name: string;

  /** Scientific name */
  scientificName?: string;

  /** Pathogen type */
  type: 'bacteria' | 'virus' | 'parasite' | 'fungus' | 'prion';

  /** Temporal prevalence data */
  prevalence: {
    /** Historical era */
    era: string;

    /** Start year */
    startYear: number;

    /** End year */
    endYear: number;

    /** Affected regions */
    regions: string[];

    /** Prevalence rate (0-1) */
    prevalenceRate: number;
  }[];

  /** Virulence factor (0-1) */
  virulence: number;

  /** Transmission modes */
  transmission: TransmissionMode[];

  /** Incubation period (hours) */
  incubationPeriod: number;

  /** Symptom onset time (hours) */
  symptomOnset: number;

  /** Case fatality rate (0-1) */
  mortality: number;

  /** Is treatable */
  treatable: boolean;

  /** Effective antibiotics */
  antibiotics?: string[];

  /** Effective antivirals */
  antivirals?: string[];

  /** Available vaccination */
  vaccination?: string;

  /** Modern human resistance (0-1) */
  modernResistance: number;

  /** Estimated historical deaths */
  historicalDeaths: number;

  /** Notable outbreaks */
  notableOutbreaks: Outbreak[];
}

/**
 * Pathogen screening results
 */
export interface PathogenScreen {
  /** Screening date */
  screeningDate: Date;

  /** Pathogens tested for */
  testedPathogens: string[];

  /** Detected pathogens */
  detectedPathogens: {
    /** Pathogen identifier */
    pathogenId: string;

    /** Pathogen name */
    name: string;

    /** Load/concentration */
    load: number;

    /** Detection confidence (0-1) */
    confidence: number;

    /** Requires treatment */
    requiresTreatment: boolean;
  }[];

  /** Number of pathogens detected */
  pathogensDetected: number;

  /** Overall screening result */
  result: 'clean' | 'contaminated' | 'inconclusive';

  /** Radiatiion exposure data */
  radiationExposure: RadiationExposure;
}

// ============================================================================
// Radiation Types
// ============================================================================

/**
 * Radiation exposure measurement
 */
export interface RadiationExposure {
  /** Measurement date */
  measuredAt: Date;

  /** Radiation sources breakdown */
  sources: {
    /** Chronon radiation from temporal field (mSv) */
    chrononRadiation: number;

    /** Tachyon flux (mSv) */
    tachyonFlux: number;

    /** Quantum decay radiation (mSv) */
    quantumDecay: number;

    /** Cosmic rays (mSv) */
    cosmicRays: number;

    /** Background radiation (mSv) */
    backgroundRadiation: number;

    /** Total exposure (mSv) */
    total: number;
  };

  /** Journey duration (hours) */
  journeyDuration: number;

  /** Dosimeter reading (mSv) */
  dosimeterReading: number;

  /** Effective dose (mSv) */
  effectiveDose: number;

  /** Organ-specific doses (mSv) */
  organDoses: Record<string, number>;

  /** Within safety limits */
  withinLimits: boolean;

  /** Safety margin (mSv) */
  safetyMargin: number;

  /** Risk level assessment */
  riskLevel: 'minimal' | 'low' | 'moderate' | 'high' | 'extreme';

  /** Recommendations */
  recommendations: string[];

  /** Requires treatment */
  requiresTreatment: boolean;

  /** Recommended treatments */
  treatments?: string[];
}

/**
 * Radiation monitoring configuration
 */
export interface RadiationMonitor {
  /** Monitor identifier */
  monitorId: string;

  /** Traveler identifier */
  travelerId: string;

  /** Journey identifier */
  journeyId: string;

  /** Measurement interval (seconds) */
  interval: number;

  /** Alert threshold (mSv) */
  alertThreshold: number;

  /** Auto-abort journey if critical */
  autoAbort: boolean;

  /** Measurements collected */
  measurements: RadiationMeasurement[];

  /** Active alerts */
  alerts: RadiationAlert[];

  /** Monitoring status */
  status: 'active' | 'paused' | 'stopped';
}

/**
 * Single radiation measurement
 */
export interface RadiationMeasurement {
  /** Measurement timestamp */
  timestamp: Date;

  /** Sequence number */
  sequenceNumber: number;

  /** Dose rate (mSv/hour) */
  doseRate: number;

  /** Cumulative dose (mSv) */
  cumulativeDose: number;

  /** Exposure type breakdown */
  breakdown: {
    chronon: number;
    tachyon: number;
    quantum: number;
    cosmic: number;
  };
}

/**
 * Radiation alert
 */
export interface RadiationAlert {
  /** Alert identifier */
  alertId: string;

  /** Alert timestamp */
  timestamp: Date;

  /** Alert severity */
  severity: 'info' | 'warning' | 'critical' | 'emergency';

  /** Alert message */
  message: string;

  /** Current exposure (mSv) */
  currentExposure: number;

  /** Limit exceeded (mSv) */
  limit: number;

  /** Recommended action */
  action: string;
}

// ============================================================================
// Screening Types
// ============================================================================

/**
 * Clearance level for travel
 */
export type ClearanceLevel = 'excellent' | 'good' | 'acceptable' | 'marginal' | 'poor' | 'critical';

/**
 * Required precaution
 */
export interface Precaution {
  /** Precaution type */
  type: string;

  /** Description */
  description: string;

  /** Severity */
  severity: 'required' | 'recommended' | 'optional';

  /** Implementation details */
  implementation?: string;
}

/**
 * Required medical treatment
 */
export interface Treatment {
  /** Treatment type */
  type: string;

  /** Treatment name */
  name: string;

  /** Description */
  description: string;

  /** Urgency */
  urgency: 'immediate' | 'before_travel' | 'recommended';

  /** Estimated duration */
  duration?: number;
}

/**
 * Mental health assessment
 */
export interface MentalHealthAssessment {
  /** Assessment date */
  assessmentDate: Date;

  /** Overall mental health score (0-1) */
  overallScore: number;

  /** Anxiety level (0-1) */
  anxietyLevel: number;

  /** Depression indicators (0-1) */
  depressionLevel: number;

  /** Stress resilience (0-1) */
  stressResilience: number;

  /** Cognitive function (0-1) */
  cognitiveFunction: number;

  /** Cleared for travel */
  cleared: boolean;

  /** Concerns */
  concerns: string[];

  /** Recommendations */
  recommendations: string[];
}

/**
 * Complete biological screening
 */
export interface BiologicalScreening {
  /** Screening identifier */
  screeningId: string;

  /** Traveler identifier */
  travelerId: string;

  /** Screening date */
  screeningDate: Date;

  /** Destination information */
  destination: Destination;

  /** Expected journey duration (hours) */
  duration: number;

  /** Bloodwork results */
  bloodwork: BloodworkResults;

  /** Immune system profile */
  immuneProfile: ImmuneProfile;

  /** Cellular health assessment */
  cellularHealth: CellularHealth;

  /** Baseline radiation measurement */
  radiationBaseline: RadiationExposure;

  /** Pathogen screening */
  pathogenScreen: PathogenScreen;

  /** Mental health assessment */
  mentalHealth: MentalHealthAssessment;

  /** Calculated Bio-Safety Index */
  bioSafetyIndex: number;

  /** Travel clearance status */
  cleared: boolean;

  /** Clearance level */
  clearanceLevel: ClearanceLevel;

  /** Recommendations */
  recommendations: string[];

  /** Required precautions */
  requiredPrecautions: Precaution[];

  /** Denial reasons (if not cleared) */
  denialReasons?: string[];

  /** Required treatments */
  requiredTreatments?: Treatment[];

  /** Screened by */
  screenedBy: string;

  /** Screening facility */
  facility: string;
}

// ============================================================================
// Cellular Monitoring Types
// ============================================================================

/**
 * Real-time cellular measurement
 */
export interface CellularMeasurement {
  /** Measurement timestamp */
  timestamp: Date;

  /** Sequence number */
  sequenceNumber: number;

  // DNA metrics
  /** DNA integrity (0-1) */
  dnaIntegrity: number;

  /** Mutation count */
  mutationCount: number;

  /** DNA strand breaks */
  strandBreaks: number;

  /** DNA repair activity (0-1) */
  repairActivity: number;

  // Mitochondrial metrics
  /** ATP level (pmol/min/cell) */
  atpLevel: number;

  /** Oxygen consumption rate */
  oxygenConsumption: number;

  /** Mitochondrial membrane potential */
  mitochondrialMembranePotential: number;

  // Telomere metrics
  /** Telomere length (kb) */
  telomereLength: number;

  /** Telomerase activity (0-1) */
  telomeraseActivity: number;

  // Membrane metrics
  /** Membrane integrity (0-1) */
  membraneIntegrity: number;

  /** Ion gradient (normalized) */
  ionGradient: number;

  // Protein metrics
  /** Protein synthesis rate */
  proteinSynthesis: number;

  /** Unfolded protein response level */
  unfoldedProteinResponse: number;

  // Cell death metrics
  /** Apoptotic cell percentage */
  apoptoticCells: number;

  /** Necrotic cell percentage */
  necroticCells: number;

  /** Overall cell health (0-1) */
  overallCellHealth: number;

  /** Degradation rate (cells/hour) */
  degradationRate: number;
}

/**
 * Cellular alert types
 */
export type CellularAlertType =
  | 'DNA_DAMAGE'
  | 'MITOCHONDRIAL_FAILURE'
  | 'TELOMERE_CRITICAL'
  | 'MEMBRANE_BREACH'
  | 'PROTEIN_MISFOLDING'
  | 'EXCESSIVE_APOPTOSIS'
  | 'NECROSIS_DETECTED'
  | 'RAPID_DEGRADATION';

/**
 * Cellular health alert
 */
export interface CellularAlert {
  /** Alert identifier */
  alertId: string;

  /** Alert timestamp */
  timestamp: Date;

  /** Alert severity */
  severity: 'info' | 'warning' | 'critical' | 'emergency';

  /** Alert type */
  type: CellularAlertType;

  /** Alert message */
  message: string;

  /** Affected biological systems */
  affectedSystems: string[];

  /** Associated measurement */
  measurement: CellularMeasurement;

  /** Recommended action */
  recommendedAction: string;

  /** Journey auto-aborted */
  autoAborted?: boolean;
}

/**
 * Cellular integrity monitor
 */
export interface CellularMonitor {
  /** Monitor identifier */
  monitorId: string;

  /** Traveler identifier */
  travelerId: string;

  /** Journey identifier */
  journeyId: string;

  /** Start time */
  startTime: Date;

  /** Measurement interval (seconds) */
  interval: number;

  /** Alert threshold (0-1) */
  alertThreshold: number;

  /** Auto-abort if critical */
  autoAbort: boolean;

  /** Collected measurements */
  measurements: CellularMeasurement[];

  /** Active alerts */
  alerts: CellularAlert[];

  /** Monitoring status */
  status: 'active' | 'paused' | 'stopped';
}

// ============================================================================
// Quarantine Types
// ============================================================================

/**
 * Quarantine level
 */
export type QuarantineLevel = 'low' | 'medium' | 'high' | 'critical';

/**
 * Quarantine facility type
 */
export type QuarantineFacilityType = 'hospital' | 'dedicated_quarantine' | 'home' | 'containment_center';

/**
 * Bio-safety level
 */
export type BioSafetyLevel = 'level_1' | 'level_2' | 'level_3' | 'level_4';

/**
 * Quarantine facility
 */
export interface QuarantineFacility {
  /** Facility identifier */
  facilityId: string;

  /** Facility name */
  name: string;

  /** Facility type */
  type: QuarantineFacilityType;

  /** Location */
  location: Vector3;

  /** Bio-safety level */
  biosafety: BioSafetyLevel;

  /** Capacity */
  capacity: number;

  /** Current occupancy */
  currentOccupancy: number;

  /** Available equipment */
  equipment: {
    negativeAirflow: boolean;
    hepaFiltration: boolean;
    decontaminationChamber: boolean;
    medicalFacilities: string[];
    emergencyEquipment: string[];
  };

  /** Staff information */
  staff: {
    physicians: number;
    nurses: number;
    infectiousDisease: number;
    support: number;
  };
}

/**
 * Test result
 */
export interface TestResult {
  /** Test identifier */
  testId: string;

  /** Test type */
  testType: string;

  /** Test timestamp */
  timestamp: Date;

  /** Test result */
  result: 'negative' | 'positive' | 'inconclusive';

  /** Detailed results */
  details: Record<string, any>;

  /** Tested by */
  testedBy: string;
}

/**
 * Test schedule entry
 */
export interface TestSchedule {
  /** Scheduled test time */
  testTime: Date;

  /** Test type */
  testType: string;

  /** Is required */
  required: boolean;

  /** Completed status */
  completed: boolean;

  /** Test result */
  result?: TestResult;
}

/**
 * Quarantine violation record
 */
export interface QuarantineViolation {
  /** Violation identifier */
  violationId: string;

  /** Violation timestamp */
  timestamp: Date;

  /** Violation type */
  type: string;

  /** Description */
  description: string;

  /** Severity */
  severity: 'minor' | 'major' | 'critical';

  /** Response taken */
  response: string;
}

/**
 * Quarantine protocol
 */
export interface QuarantineProtocol {
  /** Quarantine identifier */
  quarantineId: string;

  /** Traveler identifier */
  travelerId: string;

  /** Journey identifier */
  journeyId: string;

  /** Quarantine level */
  level: QuarantineLevel;

  /** Duration (hours) */
  duration: number;

  /** Facility */
  facility: QuarantineFacility;

  /** Monitoring frequency (minutes) */
  monitoringFrequency: number;

  /** Testing schedule */
  testingSchedule: TestSchedule[];

  /** Observation required */
  observationRequired: boolean;

  /** Isolation requirements */
  isolation: {
    roomType: 'standard' | 'negative_pressure' | 'containment';
    contactRestrictions: string[];
    visitorPolicy: 'none' | 'restricted' | 'allowed';
    communicationMethods: string[];
  };

  /** Testing requirements */
  testing: {
    pathogenScreening: string[];
    frequency: number;
    clearanceRequired: number;
  };

  /** Release criteria */
  releaseCriteria: {
    negativeTests: number;
    symptomFree: number;
    biologicalNormalization: boolean;
    physicalExamination: boolean;
  };

  /** Status */
  status: 'active' | 'completed' | 'extended' | 'breached';

  /** Start time */
  startTime: Date;

  /** Estimated end time */
  estimatedEndTime: Date;

  /** Actual end time */
  actualEndTime?: Date;

  /** Violations */
  violations: QuarantineViolation[];
}

// ============================================================================
// Decontamination Types
// ============================================================================

/**
 * Decontamination level
 */
export type DecontaminationLevel = 'basic' | 'standard' | 'comprehensive' | 'critical';

/**
 * Decontamination methods
 */
export type DecontaminationMethod =
  | 'UV_C_IRRADIATION'
  | 'CHEMICAL_STERILIZATION'
  | 'THERMAL_TREATMENT'
  | 'OZONE_TREATMENT'
  | 'PLASMA_STERILIZATION'
  | 'QUANTUM_FIELD_DECONTAMINATION'
  | 'NANITE_PATHOGEN_ELIMINATION'
  | 'MOLECULAR_RECONSTRUCTION'
  | 'TEMPORAL_FIELD_ISOLATION'
  | 'CELLULAR_REGENERATION';

/**
 * Decontamination protocol
 */
export interface DecontaminationProtocol {
  /** Protocol identifier */
  protocolId: string;

  /** Decontamination level */
  level: DecontaminationLevel;

  /** Traveler identifier */
  travelerId: string;

  /** Journey identifier */
  journeyId: string;

  /** Methods to apply */
  methods: DecontaminationMethod[];

  /** Execution sequence */
  sequence: {
    step: number;
    method: DecontaminationMethod;
    duration: number;
    parameters: Record<string, any>;
    verification: string;
  }[];

  /** Start time */
  startTime: Date;

  /** Estimated duration (seconds) */
  estimatedDuration: number;

  /** Completion time */
  completedAt?: Date;

  /** Results */
  results: {
    pathogensNeutralized: number;
    surfaceDecontamination: number;
    internalDecontamination: number;
    verificationTests: TestResult[];
    successful: boolean;
  };

  /** Equipment used */
  equipment: string[];

  /** Chemicals used */
  chemicals: string[];

  /** Personnel involved */
  personnel: string[];
}

// ============================================================================
// Emergency Types
// ============================================================================

/**
 * Emergency classification level
 */
export type EmergencyLevel = 'YELLOW' | 'ORANGE' | 'RED' | 'BLACK';

/**
 * Bio-hazard emergency type
 */
export type EmergencyType =
  | 'PATHOGEN_EXPOSURE'
  | 'OUTBREAK_DETECTED'
  | 'CELLULAR_COLLAPSE'
  | 'RADIATION_POISONING'
  | 'DECONTAMINATION_FAILURE'
  | 'QUARANTINE_BREACH'
  | 'TIMELINE_CONTAMINATION';

/**
 * Bio-hazard emergency
 */
export interface BioHazardEmergency {
  /** Emergency identifier */
  emergencyId: string;

  /** Classification level */
  classificationLevel: EmergencyLevel;

  /** Emergency type */
  type: EmergencyType;

  /** Traveler identifier */
  travelerId: string;

  /** Journey identifier */
  journeyId: string;

  /** Detection timestamp */
  detectedAt: Date;

  /** Location */
  location: Vector3;

  /** Involved pathogen */
  pathogen?: TemporalPathogen;

  /** Exposed individuals */
  exposedIndividuals: string[];

  /** Contaminated areas */
  contaminatedAreas: string[];

  /** Spread risk (0-1) */
  spreadRisk: number;

  /** Response protocol */
  responseProtocol: string;

  /** Response team */
  responseTeam: string[];

  /** Containment measures */
  containmentMeasures: string[];

  /** Treatment protocol */
  treatmentProtocol: string[];

  /** Emergency status */
  status: 'active' | 'contained' | 'resolved';

  /** Resolution timestamp */
  resolvedAt?: Date;

  /** Timeline contamination detected */
  timelineContamination: boolean;

  /** Affected timelines */
  affectedTimelines: string[];
}

// ============================================================================
// Monitoring Types
// ============================================================================

/**
 * Vital signs measurement
 */
export interface VitalSigns {
  /** Measurement timestamp */
  timestamp: Date;

  /** Body temperature (°C) */
  temperature: number;

  /** Heart rate (bpm) */
  heartRate: number;

  /** Blood pressure */
  bloodPressure: {
    systolic: number;
    diastolic: number;
  };

  /** Respiratory rate (breaths/min) */
  respiratoryRate: number;

  /** Oxygen saturation (%) */
  oxygenSaturation: number;

  /** Alert status */
  alertStatus: boolean;

  /** Abnormalities detected */
  abnormalities: string[];
}

/**
 * Symptom report
 */
export interface SymptomReport {
  /** Report timestamp */
  timestamp: Date;

  /** Reported by */
  reportedBy: 'patient' | 'observer' | 'automated';

  /** Symptoms present */
  symptoms: {
    fever: boolean;
    cough: boolean;
    shortnessOfBreath: boolean;
    fatigue: boolean;
    muscleAches: boolean;
    headache: boolean;
    rash: boolean;
    nausea: boolean;
    vomiting: boolean;
    diarrhea: boolean;
    other: string[];
  };

  /** Overall severity */
  severity: 'mild' | 'moderate' | 'severe';

  /** Additional notes */
  notes: string;
}

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * Bio-safety monitor configuration
 */
export interface BioSafetyMonitorConfig {
  /** Strict mode (stricter thresholds) */
  strictMode?: boolean;

  /** Real-time monitoring enabled */
  realTimeMonitoring?: boolean;

  /** Alert threshold (0-1) */
  alertThreshold?: number;

  /** Auto-quarantine on issues */
  autoQuarantine?: boolean;

  /** Alert email */
  alertEmail?: string;

  /** Emergency contact */
  emergencyContact?: string;
}

// ============================================================================
// Result Types
// ============================================================================

/**
 * Screening clearance result
 */
export interface ScreeningResult {
  /** Screening record */
  screening: BiologicalScreening;

  /** Cleared for travel */
  cleared: boolean;

  /** Clearance certificate ID */
  certificateId?: string;

  /** Additional recommendations */
  precautions: string[];

  /** Required treatments before travel */
  requiredTreatments?: Treatment[];
}

/**
 * Decontamination result
 */
export interface DecontaminationResult {
  /** Protocol used */
  protocol: DecontaminationProtocol;

  /** Successful completion */
  successful: boolean;

  /** Pathogens neutralized */
  pathogensNeutralized: number;

  /** Verification test results */
  verificationTests: TestResult[];

  /** Final clearance */
  cleared: boolean;
}

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
 * WIA-TIME-027 error codes
 */
export enum BioSafetyErrorCode {
  SCREENING_FAILED = 'B001',
  PATHOGEN_DETECTED = 'B002',
  CELLULAR_DEGRADATION = 'B003',
  RADIATION_EXCEEDED = 'B004',
  DECONTAMINATION_FAILED = 'B005',
  QUARANTINE_BREACH = 'B006',
  EMERGENCY_PROTOCOL = 'B007',
  INSUFFICIENT_DATA = 'B008',
  CLEARANCE_DENIED = 'B009',
  MONITORING_FAILURE = 'B010',
}

/**
 * Bio-safety error class
 */
export class BioSafetyError extends Error {
  constructor(
    public code: BioSafetyErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BioSafetyError';
  }
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Bio-safety constants
 */
export const BIOSAFETY_CONSTANTS = {
  /** Default component weights for BSI */
  DEFAULT_WEIGHTS: {
    immuneHealth: 0.30,
    cellularIntegrity: 0.25,
    radiationSafety: 0.20,
    pathogenContainment: 0.15,
    decontaminationReadiness: 0.10,
  },

  /** Clearance thresholds */
  CLEARANCE_THRESHOLDS: {
    excellent: 0.95,
    good: 0.85,
    acceptable: 0.75,
    marginal: 0.65,
    poor: 0.50,
  },

  /** Radiation limits (mSv) */
  RADIATION_LIMITS: {
    shortTerm: { maximum: 50, recommended: 25 },
    medium: { maximum: 100, recommended: 50 },
    extended: { maximum: 200, recommended: 100 },
    longDuration: { maximum: 500, recommended: 250 },
    annualLimit: 1000,
  },

  /** Quarantine durations (hours) */
  QUARANTINE_DURATIONS: {
    low: 24,
    medium: 48,
    high: 168,
    critical: 336,
  },

  /** Normal vital sign ranges */
  VITAL_RANGES: {
    temperature: { min: 36.1, max: 37.2 },
    heartRate: { min: 60, max: 100 },
    systolicBP: { min: 90, max: 120 },
    diastolicBP: { min: 60, max: 80 },
    respiratoryRate: { min: 12, max: 20 },
    oxygenSaturation: { min: 95, max: 100 },
  },
} as const;

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core
  Vector3,
  Destination,

  // Bio-Safety Index
  BioSafetyIndex,

  // Medical
  BloodworkResults,
  Vaccination,
  ImmuneProfile,
  CellularHealth,
  MentalHealthAssessment,

  // Pathogen
  TransmissionMode,
  Outbreak,
  TemporalPathogen,
  PathogenScreen,

  // Radiation
  RadiationExposure,
  RadiationMonitor,
  RadiationMeasurement,
  RadiationAlert,

  // Screening
  ClearanceLevel,
  Precaution,
  Treatment,
  BiologicalScreening,
  ScreeningResult,

  // Monitoring
  CellularMeasurement,
  CellularAlertType,
  CellularAlert,
  CellularMonitor,
  VitalSigns,
  SymptomReport,

  // Quarantine
  QuarantineLevel,
  QuarantineFacilityType,
  BioSafetyLevel,
  QuarantineFacility,
  TestResult,
  TestSchedule,
  QuarantineViolation,
  QuarantineProtocol,

  // Decontamination
  DecontaminationLevel,
  DecontaminationMethod,
  DecontaminationProtocol,
  DecontaminationResult,

  // Emergency
  EmergencyLevel,
  EmergencyType,
  BioHazardEmergency,

  // Configuration
  BioSafetyMonitorConfig,
};

export { BIOSAFETY_CONSTANTS, BioSafetyErrorCode, BioSafetyError };
