/**
 * WIA-AUG-011: Bio-Integration - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Bio-Integration Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Integration Level Classification
// ============================================================================

/**
 * Integration level based on anatomical depth and tissue type
 */
export type IntegrationLevel =
  | 'SURFACE'       // Epidermis/dermis interface (0-2mm)
  | 'SUBCUTANEOUS'  // Subcutaneous fat (2-10mm)
  | 'DEEP_TISSUE'   // Muscle/fascia/organ (>10mm)
  | 'NEURAL'        // Nerve tissue integration
  | 'VASCULAR'      // Blood vessel interface
  | 'OSSEOUS';      // Bone integration

/**
 * Tissue interface technology type
 */
export type InterfaceType =
  | 'BIOELECTRONIC'   // Electrical signal exchange
  | 'BIOMECHANICAL'   // Force/motion transfer
  | 'BIOCHEMICAL'     // Molecular exchange
  | 'OPTICAL'         // Light-based interface
  | 'MAGNETIC';       // Magnetic field coupling

/**
 * Tissue type classification
 */
export type TissueType =
  | 'skin'
  | 'fat'
  | 'muscle'
  | 'fascia'
  | 'tendon'
  | 'ligament'
  | 'bone'
  | 'cartilage'
  | 'nerve'
  | 'blood_vessel'
  | 'organ';

/**
 * Integration status classification
 */
export type IntegrationStatus =
  | 'PLANNING'      // Pre-implantation assessment
  | 'ACUTE'         // 0-4 weeks post-implant
  | 'SUBACUTE'      // 4-12 weeks
  | 'CHRONIC'       // 12+ weeks
  | 'STABLE'        // Long-term stable integration
  | 'DEGRADING'     // Declining integration
  | 'FAILED';       // Integration failure

/**
 * Osseointegration phase
 */
export type OsseointegrationPhase =
  | 'INITIAL_CONTACT'     // 0-2 weeks
  | 'FIBROUS_ANCHORING'   // 2-6 weeks
  | 'BONE_APPOSITION'     // 6-12 weeks
  | 'REMODELING'          // 3-6 months
  | 'LONGTERM_STABILITY'; // 6+ months

// ============================================================================
// Integration Site Assessment
// ============================================================================

/**
 * Integration site location and characteristics
 */
export interface IntegrationSite {
  /** Anatomical location (e.g., "forearm_flexor", "left_radius_distal") */
  anatomicalLocation: string;

  /** Integration depth level */
  depth: IntegrationLevel;

  /** Primary tissue type at integration site */
  tissueType: TissueType;

  /** Blood vessel density */
  vascularity: 'minimal' | 'low' | 'moderate' | 'high' | 'very_high';

  /** Nerve fiber density */
  nerveDensity: 'minimal' | 'low' | 'moderate' | 'high';

  /** Mechanical stress level */
  mechanicalStress: 'low' | 'moderate' | 'high' | 'extreme';

  /** Infection risk category */
  exposureRisk: 'sterile' | 'clean' | 'clean_contaminated' | 'contaminated';

  /** Additional site characteristics */
  characteristics?: {
    boneQuality?: 'poor' | 'fair' | 'good' | 'excellent';
    muscleTone?: 'atrophic' | 'normal' | 'hypertrophic';
    skinCondition?: 'poor' | 'fair' | 'good' | 'excellent';
    previousSurgery?: boolean;
    scarTissue?: boolean;
  };
}

/**
 * Site assessment result
 */
export interface SiteAssessment {
  /** Integration site information */
  site: IntegrationSite;

  /** Integration depth score (0-10) */
  integrationDepthScore: number;

  /** Tissue compatibility score (0-100) */
  tissueCompatibility: number;

  /** Recommended integration level */
  recommendedLevel: IntegrationLevel;

  /** Risk factors identified */
  riskFactors: string[];

  /** Recommendations for optimization */
  recommendations: string[];

  /** Overall suitability (0-100) */
  suitabilityScore: number;
}

// ============================================================================
// Integration Protocol
// ============================================================================

/**
 * Integration protocol specification
 */
export interface IntegrationProtocol {
  /** Target integration level */
  level: IntegrationLevel;

  /** Interface technology type */
  interfaceType: InterfaceType;

  /** Target tissue for integration */
  targetTissue: TissueType;

  /** Implant base material */
  implantMaterial: string;

  /** Surface treatment applied */
  surfaceTreatment?: string;

  /** Bioactive coating */
  bioactiveCoating?: string;

  /** Timeline and milestones */
  timeline?: {
    surgeryDate?: Date;
    expectedIntegration?: number; // weeks
    monitoringSchedule?: string[];
  };

  /** Special considerations */
  specialConsiderations?: string[];
}

/**
 * Material specification
 */
export interface MaterialSpecification {
  /** Material name/identifier */
  name: string;

  /** Biocompatibility score (ISO 10993) */
  biocompatibility: number; // 0-100

  /** Mechanical properties */
  mechanicalProperties: {
    tensileStrength: number;        // MPa
    elasticModulus: number;         // GPa
    fatigueResistance: number;      // cycles
  };

  /** Surface properties */
  surfaceProperties: {
    roughness: number;              // μm Ra
    wettability: number;            // contact angle (degrees)
    surfaceEnergy: number;          // mN/m
  };

  /** Bioactivity characteristics */
  bioactivity?: {
    cellAdhesion: number;           // 0-100 score
    proteinAdsorption: string[];    // Favorable proteins
    tissueInduction: boolean;       // Osteoinductive, etc.
  };
}

// ============================================================================
// Integration Status and Monitoring
// ============================================================================

/**
 * Integration status metrics
 */
export interface IntegrationStatusMetrics {
  /** Unique integration identifier */
  integrationId: string;

  /** Current integration status */
  status: IntegrationStatus;

  /** Time since implantation */
  timeSinceImplant: {
    weeks: number;
    phase: string;
  };

  /** Stability score (0-100) */
  stability: number;

  /** Tissue health score (0-100) */
  tissueHealth: number;

  /** Signal quality score (0-100, if applicable) */
  signalQuality?: number;

  /** Immune response score (0-100, 100 = no rejection) */
  immuneResponse: number;

  /** Integration health score (0-100) */
  healthScore: number;

  /** Timestamp of assessment */
  timestamp: Date;
}

/**
 * Stability metrics
 */
export interface StabilityMetrics {
  /** Mechanical fixation score (0-100) */
  mechanicalFixation: number;

  /** Micromotion measurement (μm) */
  micromotion?: number;

  /** Implant Stability Quotient (ISQ) for osseointegration */
  isq?: number;

  /** Pullout force (N) */
  pulloutForce?: number;

  /** Torque resistance (N⋅mm) */
  torqueResistance?: number;

  /** Structural integrity (0-100) */
  structuralIntegrity: number;

  /** Overall stability score (0-100) */
  score: number;
}

/**
 * Tissue health metrics
 */
export interface TissueHealthMetrics {
  /** Cell density (cells/mm³) */
  cellDensity: number;

  /** Cell viability percentage */
  cellViability: number;

  /** Identified cell types */
  cellTypes: string[];

  /** Vascular metrics */
  vascularization: {
    vesselDensity: number;          // vessels/mm²
    perfusion: number;              // % of normal tissue
    oxygenSaturation?: number;      // %
  };

  /** Inflammation assessment */
  inflammation: {
    inflammatoryCells: number;      // cells/mm²
    grade: 'none' | 'minimal' | 'mild' | 'moderate' | 'severe';
    cytokines?: Record<string, number>; // pg/mL
  };

  /** Extracellular matrix */
  ecm?: {
    collagenContent: number;        // mg/g tissue
    collagenRatio?: {
      type1: number;                // %
      type3: number;                // %
    };
  };

  /** Overall tissue health score (0-100) */
  score: number;
}

/**
 * Signal quality metrics (for bioelectronic interfaces)
 */
export interface SignalQualityMetrics {
  /** Impedance (Ω) */
  impedance: {
    value: number;
    frequency: number;              // Hz
    baseline: number;               // Initial impedance
  };

  /** Signal amplitude (μV) */
  signalAmplitude?: {
    mean: number;
    peak: number;
    rms: number;
  };

  /** Signal-to-noise ratio (dB) */
  snr?: number;

  /** Noise level (μV RMS) */
  noiseLevel?: number;

  /** Bandwidth (Hz) */
  bandwidth?: number;

  /** Latency (ms) */
  latency?: number;

  /** Recording channels functional (%) */
  channelYield?: number;

  /** Overall signal quality score (0-100) */
  score: number;
}

/**
 * Immune response metrics
 */
export interface ImmuneResponseMetrics {
  /** Inflammatory cytokines */
  cytokines: {
    il6?: number;                   // pg/mL (pro-inflammatory)
    tnfAlpha?: number;              // pg/mL (pro-inflammatory)
    il10?: number;                  // pg/mL (anti-inflammatory)
    il4?: number;                   // pg/mL (anti-inflammatory)
  };

  /** Macrophage polarization */
  macrophageRatio: {
    m1: number;                     // % M1 (pro-inflammatory)
    m2: number;                     // % M2 (healing)
    ratio: number;                  // M1/M2 (lower is better)
  };

  /** Fibrous capsule formation */
  fibrousCapsule?: {
    present: boolean;
    thickness: number;              // μm
    vascularity: 'none' | 'sparse' | 'moderate' | 'high';
    cellularity: 'low' | 'moderate' | 'high';
  };

  /** Immunogenicity indicators */
  immunogenicity: {
    antibodyProduction: boolean;
    complementActivation: boolean;
    tcellResponse: boolean;
  };

  /** Overall immune response score (0-100, 100 = no rejection) */
  score: number;
}

// ============================================================================
// Osseointegration Specific
// ============================================================================

/**
 * Osseointegration status
 */
export interface OsseointegrationStatus {
  /** Current phase */
  phase: OsseointegrationPhase;

  /** Implant Stability Quotient */
  isq: number;

  /** Bone-implant contact percentage */
  boneImplantContact: number;

  /** Micromotion (μm) */
  micromotion: number;

  /** Bone density at interface (% of native) */
  boneDensity?: number;

  /** Radiographic assessment */
  radiographic?: {
    boneLoss: number;               // mm from baseline
    radiolucency: boolean;
    boneRemodeling: boolean;
  };

  /** Success criteria met */
  successCriteriaMet: boolean;

  /** Overall osseointegration score (0-100) */
  score: number;
}

// ============================================================================
// Neural Integration Specific
// ============================================================================

/**
 * Neural integration status
 */
export interface NeuralIntegrationStatus {
  /** Integration phase */
  phase: 'acute' | 'subacute' | 'chronic';

  /** Time since implantation (weeks) */
  weeksSinceImplant: number;

  /** Recording performance */
  recording?: {
    channelsFunctional: number;     // % of total
    signalAmplitude: number;        // % of acute
    unitYield: number;              // units/channel
  };

  /** Impedance tracking */
  impedance: {
    current: number;                // kΩ
    baseline: number;               // kΩ
    ratio: number;                  // current/baseline
  };

  /** Glial scar assessment */
  glialScar?: {
    thickness: number;              // μm
    density: 'low' | 'moderate' | 'high';
  };

  /** Neuronal health */
  neuronalHealth?: {
    neuronalLoss: number;           // % within radius
    regeneration: boolean;
  };

  /** Overall neural integration score (0-100) */
  score: number;
}

// ============================================================================
// Vascular Integration Specific
// ============================================================================

/**
 * Vascular integration status
 */
export interface VascularIntegrationStatus {
  /** Vessel patency */
  patency: boolean;

  /** Endothelialization */
  endothelialization: {
    coverage: number;               // % of surface
    morphology: 'cobblestone' | 'irregular' | 'sparse';
    functional: boolean;
  };

  /** Thrombogenicity assessment */
  thrombogenicity: {
    thrombusFormation: boolean;
    plateletActivation: number;     // % activated
    anticoagulationRequired: boolean;
  };

  /** Hemodynamics */
  hemodynamics?: {
    flowVelocity: number;           // cm/s
    wallShearStress: number;        // dynes/cm²
    turbulence: boolean;
  };

  /** Neovascularization (peri-implant) */
  neovascularization?: {
    vesselDensity: number;          // vessels/mm²
    perfusion: number;              // % of normal
  };

  /** Overall vascular integration score (0-100) */
  score: number;
}

// ============================================================================
// Biofilm Assessment
// ============================================================================

/**
 * Biofilm risk assessment
 */
export interface BiofilmRiskAssessment {
  /** Risk level */
  riskLevel: 'low' | 'moderate' | 'high' | 'critical';

  /** Biofilm detection */
  detection: {
    detected: boolean;
    method?: string;
    bacterialSpecies?: string[];
  };

  /** Clinical indicators */
  clinicalIndicators: {
    persistentInflammation: boolean;
    deviceMalfunction: boolean;
    refractoryInfection: boolean;
  };

  /** Laboratory findings */
  laboratory?: {
    cReactiveProtein: number;       // mg/L
    whiteBloodCount: number;        // cells/μL
    erythrocyteSedimentation: number; // mm/hr
  };

  /** Prevention measures in place */
  preventionMeasures: string[];

  /** Recommendations */
  recommendations: string[];

  /** Overall risk score (0-100) */
  riskScore: number;
}

// ============================================================================
// Long-term Tracking
// ============================================================================

/**
 * Long-term integration data
 */
export interface LongtermData {
  /** Integration identifier */
  integrationId: string;

  /** Total monitoring duration (months) */
  monitoringDuration: number;

  /** Data points collected */
  dataPoints: IntegrationStatusMetrics[];

  /** Trend analysis */
  trends: {
    stability: 'improving' | 'stable' | 'declining';
    tissueHealth: 'improving' | 'stable' | 'declining';
    signalQuality?: 'improving' | 'stable' | 'declining';
    immuneResponse: 'improving' | 'stable' | 'declining';
  };

  /** Degradation rate (% per year) */
  degradationRate: number;

  /** Predicted stability */
  prediction: {
    predictedIHS: number;           // Integration Health Score at 5 years
    failureRisk: 'low' | 'moderate' | 'high';
    estimatedLifespan: number;      // years
  };

  /** Adverse events */
  adverseEvents: AdverseEvent[];

  /** Overall long-term success */
  success: boolean;
}

/**
 * Adverse event record
 */
export interface AdverseEvent {
  /** Event identifier */
  eventId: string;

  /** Date of occurrence */
  date: Date;

  /** Event type */
  type: 'infection' | 'loosening' | 'fracture' | 'rejection' | 'pain' | 'other';

  /** Severity */
  severity: 'mild' | 'moderate' | 'severe' | 'life_threatening';

  /** Description */
  description: string;

  /** Intervention required */
  intervention?: string;

  /** Resolution */
  resolution: 'resolved' | 'ongoing' | 'chronic';
}

// ============================================================================
// Optimization and Recommendations
// ============================================================================

/**
 * Interface optimization target
 */
export type OptimizationTarget =
  | 'stability'
  | 'tissue_health'
  | 'signal_quality'
  | 'immune_response'
  | 'overall_integration';

/**
 * Optimization result
 */
export interface OptimizationResult {
  /** Target parameter */
  target: OptimizationTarget;

  /** Current value */
  currentValue: number;

  /** Recommended actions */
  recommendations: Recommendation[];

  /** Expected improvement */
  expectedImprovement: number;      // % increase in score

  /** Implementation timeline */
  timeline: string;

  /** Risk assessment */
  risks: string[];
}

/**
 * Recommendation
 */
export interface Recommendation {
  /** Recommendation category */
  category: 'material' | 'surgical' | 'pharmacological' | 'monitoring' | 'lifestyle';

  /** Priority level */
  priority: 'low' | 'medium' | 'high' | 'critical';

  /** Recommendation description */
  description: string;

  /** Expected impact (0-100) */
  expectedImpact: number;

  /** Implementation difficulty */
  difficulty: 'easy' | 'moderate' | 'difficult';

  /** Estimated cost */
  cost?: 'low' | 'moderate' | 'high';
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Integration depth score thresholds
 */
export const INTEGRATION_DEPTH_THRESHOLDS = {
  SURFACE_MAX: 3.0,
  SUBCUTANEOUS_MIN: 3.1,
  SUBCUTANEOUS_MAX: 5.0,
  DEEP_TISSUE_MIN: 5.1,
  DEEP_TISSUE_MAX: 7.0,
  NEURAL_VASCULAR_MIN: 7.1,
  NEURAL_VASCULAR_MAX: 8.5,
  OSSEOUS_MIN: 8.6,
} as const;

/**
 * Integration Health Score weights
 */
export const IHS_WEIGHTS = {
  STABILITY: 0.30,
  TISSUE_HEALTH: 0.30,
  SIGNAL_QUALITY: 0.25,
  IMMUNE_RESPONSE: 0.15,
} as const;

/**
 * Minimum performance thresholds
 */
export const MINIMUM_THRESHOLDS = {
  IHS_12_WEEKS: 60,
  IHS_6_MONTHS: 70,
  STABILITY: 80,
  TISSUE_HEALTH: 70,
  SIGNAL_QUALITY: 60,
  IMMUNE_RESPONSE: 70,
} as const;

/**
 * Osseointegration ISQ thresholds
 */
export const OSSEO_ISQ_THRESHOLDS = {
  INITIAL_MIN: 60,
  WEEK_12_MIN: 75,
  STABLE_MIN: 80,
} as const;

/**
 * Micromotion limits (μm)
 */
export const MICROMOTION_LIMITS = {
  ACUTE_MAX: 150,
  INTEGRATION_MAX: 100,
  STABLE_MAX: 50,
  OPTIMAL_MAX: 20,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Bio-integration error codes
 */
export enum BioIntegrationErrorCode {
  SITE_ASSESSMENT_FAILED = 'BIO001',
  INTEGRATION_INITIATION_FAILED = 'BIO002',
  MONITORING_DATA_UNAVAILABLE = 'BIO003',
  STABILITY_BELOW_THRESHOLD = 'BIO004',
  TISSUE_HEALTH_CRITICAL = 'BIO005',
  IMMUNE_REJECTION_DETECTED = 'BIO006',
  BIOFILM_DETECTED = 'BIO007',
  INTEGRATION_FAILURE = 'BIO008',
  INVALID_INTEGRATION_LEVEL = 'BIO009',
  INVALID_INTERFACE_TYPE = 'BIO010',
}

/**
 * Bio-integration error class
 */
export class BioIntegrationError extends Error {
  constructor(
    public code: BioIntegrationErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BioIntegrationError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  IntegrationLevel,
  InterfaceType,
  TissueType,
  IntegrationStatus,
  OsseointegrationPhase,
  IntegrationSite,
  SiteAssessment,
  IntegrationProtocol,
  MaterialSpecification,
  IntegrationStatusMetrics,
  StabilityMetrics,
  TissueHealthMetrics,
  SignalQualityMetrics,
  ImmuneResponseMetrics,
  OsseointegrationStatus,
  NeuralIntegrationStatus,
  VascularIntegrationStatus,
  BiofilmRiskAssessment,
  LongtermData,
  AdverseEvent,
  OptimizationTarget,
  OptimizationResult,
  Recommendation,
};

export {
  INTEGRATION_DEPTH_THRESHOLDS,
  IHS_WEIGHTS,
  MINIMUM_THRESHOLDS,
  OSSEO_ISQ_THRESHOLDS,
  MICROMOTION_LIMITS,
  BioIntegrationErrorCode,
  BioIntegrationError,
};
