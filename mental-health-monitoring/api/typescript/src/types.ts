/**
 * WIA-MENTAL-HEALTH: Mental Health Monitoring Standard
 * TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Mental Health Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export type Severity = 'none' | 'mild' | 'moderate' | 'moderately-severe' | 'severe';
export type AgeGroup = 'child' | 'adolescent' | 'adult' | 'elderly';
export type SessionFormat = 'in_person' | 'telehealth' | 'app_based' | 'group';
export type CrisisLevel = 'none' | 'low' | 'moderate' | 'high' | 'imminent';

// ============================================================================
// Assessment Types
// ============================================================================

export interface DepressionInstrument {
  type: 'PHQ-9' | 'BDI-II' | 'HAMD' | 'CES-D';
}

export interface AnxietyInstrument {
  type: 'GAD-7' | 'BAI' | 'STAI' | 'HAM-A';
}

export interface PTSDInstrument {
  type: 'PCL-5' | 'CAPS-5' | 'IES-R';
}

export interface WellbeingInstrument {
  type: 'WHO-5' | 'WEMWBS' | 'SWLS' | 'PANAS';
}

export interface ResilienceInstrument {
  type: 'CD-RISC' | 'BRS' | 'RSA';
}

export interface ScoreResult {
  value: number; // 0.0 - 1.0
  rawScore: number;
  severity: Severity;
  instrument: string;
}

export interface MentalHealthIndex {
  depressionScore: ScoreResult;
  anxietyScore: ScoreResult;
  ptsdScore?: ScoreResult & { meetsCriteria: boolean };
  wellbeingScore: ScoreResult;
  resilienceScore: ScoreResult;
}

export interface MentalHealthAssessment {
  id: string;
  version: string;
  timestamp: Date;
  subject: {
    id: string;
    ageGroup: AgeGroup;
    consentStatus: 'verified';
  };
  mentalHealthIndex: MentalHealthIndex;
  clinicalFlags?: {
    suicideRisk: boolean;
    requiresImmediateAttention: boolean;
  };
}

// ============================================================================
// Biomarker Types
// ============================================================================

export interface CortisolMeasurement {
  value: number;
  unit: 'ng/mL';
  timeOfDay: 'morning' | 'afternoon' | 'evening' | 'night';
  sampleType: 'saliva' | 'blood' | 'urine' | 'hair';
}

export interface InflammatoryMarkers {
  crp: { value: number; unit: 'mg/L'; status: 'normal' | 'elevated' };
  il6: { value: number; unit: 'pg/mL' };
  tnfAlpha?: { value: number; unit: 'pg/mL' };
  il1Beta?: { value: number; unit: 'pg/mL' };
}

export interface HeartRateVariability {
  rmssd: { value: number; unit: 'ms' };
  sdnn: { value: number; unit: 'ms' };
  lfHfRatio: number;
  coherenceScore: number; // 0.0 - 1.0
}

export interface SleepMetrics {
  totalSleepTime: { value: number; unit: 'minutes' };
  sleepEfficiency: number; // 0.0 - 1.0
  remPercentage: number; // 0.0 - 1.0
  deepSleepPercentage: number; // 0.0 - 1.0
  awakenings: number;
}

export interface GutBrainAxis {
  microbiomeDiversity: {
    shannonIndex: number;
    speciesRichness: number;
  };
  keyTaxa: {
    lactobacillus: number; // relative abundance 0.0 - 1.0
    bifidobacterium: number;
    firmicutesBacteroidetesRatio: number;
  };
}

export interface Biomarkers {
  collectionDate: Date;
  source: 'laboratory' | 'wearable' | 'self_report';
  stressAxis?: {
    cortisol: CortisolMeasurement;
    cortisolAwakeningResponse?: {
      baseline: number;
      peak: number;
      auc: number;
    };
    dheaS?: { value: number; unit: 'μg/dL' };
  };
  inflammatoryMarkers?: InflammatoryMarkers;
  heartRateVariability?: HeartRateVariability;
  sleepMetrics?: SleepMetrics;
  gutBrainAxis?: GutBrainAxis;
}

// ============================================================================
// Neuroimaging Types
// ============================================================================

export type ScanType = 'fMRI' | 'PET' | 'EEG' | 'MEG' | 'fNIRS';
export type AmygdalaReactivity = 'hypo' | 'normal' | 'hyper';

export interface NetworkConnectivity {
  connectivityScore: number; // 0.0 - 1.0
  withinNetworkConnectivity?: number; // z-score
}

export interface DefaultModeNetwork extends NetworkConnectivity {
  posteriorCingulate: number; // activation level -1.0 to 1.0
  medialPrefrontal: number;
  angularGyrus: number;
}

export interface Neuroimaging {
  scanType: ScanType;
  acquisitionDate: Date;
  deviceInfo: {
    manufacturer: string;
    model: string;
    fieldStrength?: number; // Tesla, for MRI
  };
  defaultModeNetwork?: DefaultModeNetwork;
  salienceNetwork?: NetworkConnectivity & {
    anteriorInsula: number;
    anteriorCingulate: number;
  };
  centralExecutiveNetwork?: NetworkConnectivity & {
    dlpfc: number;
    posteriorParietal: number;
  };
  amygdala?: {
    activityLevel: number; // 0.0 - 1.0
    prefrontalConnectivity: number; // z-score
    reactivity: AmygdalaReactivity;
  };
  hippocampus?: {
    volumeLeft: number; // mm³
    volumeRight: number;
    volumeNormalized: number; // 0.0 - 1.0
  };
  neuroplasticityMarkers?: {
    bdnfExpression: number; // 0.0 - 1.0
    synapticDensityProxy: number; // z-score
    networkFlexibility: number; // 0.0 - 1.0
  };
}

// ============================================================================
// Digital Phenotyping Types
// ============================================================================

export interface DigitalPhenotype {
  collectionPeriod: {
    start: Date;
    end: Date;
  };
  mobilityPatterns?: {
    homeTimeRatio: number; // 0.0 - 1.0
    locationVariance: number; // km²
    circadianMovementScore: number;
    significantLocationsCount: number;
  };
  socialPatterns?: {
    callFrequency: number; // per day
    messageFrequency: number;
    socialInteractionDiversity: number;
    responseLatency: number; // minutes
  };
  deviceUsage?: {
    screenTimeDaily: number; // minutes
    appSwitchesPerHour: number;
    nighttimeUsage: boolean;
    usageVariability: number;
  };
  speechPatterns?: {
    speechRate: number; // words per minute
    pauseDurationAvg: number; // seconds
    voiceEnergy: number;
    prosodyScore: number;
  };
  activityPatterns?: {
    stepsDaily: number;
    sedentaryTime: number; // minutes
    activityRegularity: number;
  };
}

// ============================================================================
// Treatment Types
// ============================================================================

export type TreatmentCategory =
  | 'psychotherapy'
  | 'pharmacotherapy'
  | 'neuromodulation'
  | 'complementary'
  | 'psychedelic'
  | 'digital';

export type PsychotherapyModality =
  | 'CBT' | 'DBT' | 'ACT' | 'EMDR' | 'IPT'
  | 'psychoanalytic' | 'humanistic' | 'integrative';

export type NeuromodulationModality =
  | 'TMS' | 'rTMS' | 'tDCS' | 'ECT' | 'VNS' | 'DBS' | 'neurofeedback';

export type PsychedelicSubstance =
  | 'psilocybin' | 'MDMA' | 'ketamine' | 'LSD' | 'ayahuasca' | 'ibogaine';

export type EvidenceLevel = '1A' | '1B' | '2A' | '2B' | '3' | '4' | '5';

export interface TreatmentProtocol {
  id: string;
  category: TreatmentCategory;
  modality: string;
  evidenceLevel: EvidenceLevel;
  targetConditions: string[]; // ICD-11 codes
  neuroplasticityMechanism: {
    primary: string;
    pathways: string[];
  };
}

export interface TreatmentSession {
  id: string;
  protocolId: string;
  sessionNumber: number;
  date: Date;
  durationMinutes: number;
  format: SessionFormat;
  provider: {
    id: string;
    credentials: string[];
    specialization?: string;
  };
  interventions: Array<{
    technique: string;
    durationMinutes: number;
    notes?: string; // encrypted
  }>;
  homeworkAssigned: boolean;
  patientEngagement: number; // 0.0 - 1.0
  adverseEvents?: {
    occurred: boolean;
    severity: 'none' | 'mild' | 'moderate' | 'severe';
    description?: string; // encrypted
  };
}

// ============================================================================
// Psychedelic Therapy Types
// ============================================================================

export type RegulatoryStatus =
  | 'approved' | 'breakthrough_therapy' | 'clinical_trial' | 'compassionate_use';

export type SessionEnvironment = 'clinical' | 'retreat' | 'home';

export interface PsychedelicSession {
  id: string;
  substance: PsychedelicSubstance;
  regulatoryStatus: RegulatoryStatus;
  phase: 'preparation' | 'dosing' | 'integration';

  preparationSession?: {
    sessionsCompleted: number;
    therapeuticAllianceScore: number;
    intentionsSet: boolean;
    medicalClearance: 'verified';
  };

  dosingSession?: {
    date: Date;
    dose: {
      amount: number;
      unit: 'mg' | 'μg';
      route: 'oral' | 'sublingual' | 'intramuscular' | 'intranasal';
    };
    setAndSetting: {
      environment: SessionEnvironment;
      musicProtocol: string;
      lighting: string;
      supportPersons: number;
    };
    monitoring: {
      vitalSignsFrequency: number; // minutes
      psychologicalSupportLevel: 'minimal' | 'moderate' | 'intensive';
    };
    experienceMeasures?: {
      mysticalExperienceQuestionnaire: number; // 0.0 - 1.0
      egoDissolutionInventory: number;
      emotionalBreakthrough: number;
      challengingExperience: number;
    };
  };

  integrationSessions?: {
    sessionsScheduled: number;
    sessionsCompleted: number;
    techniques: Array<'talk_therapy' | 'somatic' | 'art' | 'movement' | 'journaling'>;
    insightsProcessed: boolean;
  };

  safetyProtocol: {
    screeningCompleted: boolean;
    contraindicationsCleared: boolean;
    emergencyProtocol: 'documented';
    followUpScheduled: boolean;
  };
}

// ============================================================================
// Digital Therapeutics Types
// ============================================================================

export type DTxRegulatoryStatus = 'FDA_cleared' | 'CE_marked' | 'pending' | 'non_regulated';

export interface DigitalTherapeuticSession {
  id: string;
  productId: string;
  regulatoryStatus: DTxRegulatoryStatus;
  therapeuticClass: 'prescription' | 'OTC';
  targetCondition: string; // ICD-11 code

  session: {
    date: Date;
    durationMinutes: number;
    completionRate: number; // 0.0 - 1.0
    modulesCompleted: string[];
    interactions: number;
    engagementScore: number;
  };

  biometricIntegration?: {
    wearableConnected: boolean;
    dataSynced: Array<'hrv' | 'sleep' | 'activity' | 'location'>;
    justInTimeInterventions: number;
  };

  aiComponents?: {
    chatbotConversations: number;
    personalizationLevel: number;
    recommendationAccuracy: number;
  };

  outcomes?: {
    symptomChange: number; // percentage
    adherenceRate: number;
    userSatisfaction: number;
  };
}

// ============================================================================
// Crisis Types
// ============================================================================

export interface CrisisScreening {
  subjectId: string;
  timestamp: Date;
  responses: {
    suicideIdeation: 0 | 1 | 2 | 3;
    selfHarmUrges: 0 | 1 | 2 | 3;
    substanceIntoxication: boolean;
    psychosisSymptoms: boolean;
  };
  crisisLevel: CrisisLevel;
  recommendedAction: 'continue_care' | 'safety_plan' | 'crisis_line' | 'emergency_services';
  resources: Array<{
    name: string;
    phone: string;
    available: boolean;
  }>;
}

export interface SafetyPlan {
  id: string;
  subjectId: string;
  warningSignsIdentified: string[];
  copingStrategies: string[];
  socialSupports: Array<{
    name: string;
    relationship: string;
    phone: string;
  }>;
  professionalContacts: Array<{
    provider: string;
    phone: string;
    crisisLine?: string;
  }>;
  meansRestriction: {
    lethalMeansIdentified: string[];
    restrictionPlan: string;
  };
  reasonsForLiving: string[];
  lastReviewed: Date;
}

// ============================================================================
// Privacy & Consent Types
// ============================================================================

export interface Consent {
  id: string;
  subjectId: string;
  timestamp: Date;
  consentType: 'treatment' | 'research' | 'data_sharing';
  scope: string[];
  duration: string; // ISO 8601 duration
  withdrawalAllowed: boolean;
  guardianConsent?: boolean;
  capacityVerified: boolean;
  signature: string; // cryptographic signature
}

export interface SecurityConfig {
  encryptionAtRest: 'AES-256-GCM';
  encryptionInTransit: 'TLS 1.3';
  keyManagement: 'HSM';
  pseudonymization: 'required';
  auditLogging: 'required';
  dataRetention: string; // configurable per jurisdiction
}

// ============================================================================
// API Types
// ============================================================================

export interface AssessmentRequest {
  subjectId: string;
  assessmentType: 'screening' | 'diagnostic' | 'monitoring' | 'outcome';
  instruments: string[];
  responses: Record<string, { items: Array<{ itemId: number; response: number }> }>;
  context: {
    setting: 'clinical' | 'telehealth' | 'self_report' | 'research';
    administeredBy?: string;
  };
}

export interface AssessmentResponse {
  assessmentId: string;
  timestamp: Date;
  mentalHealthIndex: MentalHealthIndex;
  clinicalFlags: {
    suicideRisk: boolean;
    requiresImmediateAttention: boolean;
  };
  nextAssessmentRecommended: Date;
}

export interface TreatmentRecommendation {
  rank: number;
  protocol: TreatmentProtocol;
  matchScore: number; // 0.0 - 1.0
  neuroplasticityMechanism: string;
  expectedOutcome: {
    symptomReduction: number;
    confidenceInterval: [number, number];
  };
}

export interface ValidationResult {
  isValid: boolean;
  errors: string[];
  warnings: string[];
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface MentalHealthConfig {
  apiKey?: string;
  endpoint?: string;
  version?: string;
  privacyLevel?: 'standard' | 'high' | 'maximum';
  jurisdiction?: 'US' | 'EU' | 'CA' | 'global';
}

export interface ExportOptions {
  format: 'JSON' | 'CSV' | 'FHIR' | 'CDISC';
  deIdentified: boolean;
  includeMetadata: boolean;
  dateRange?: { start: Date; end: Date };
}
