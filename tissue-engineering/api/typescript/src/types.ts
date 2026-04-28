/**
 * WIA-BIO-006: Tissue Engineering - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biomedical Engineering Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Three-dimensional dimensions
 */
export interface Dimensions3D {
  length: number;  // mm
  width: number;   // mm
  height: number;  // mm
}

/**
 * Biomaterial types
 */
export type BiomaterialType =
  | 'collagen'
  | 'gelatin'
  | 'chitosan'
  | 'alginate'
  | 'pcl'
  | 'pla'
  | 'pga'
  | 'plga'
  | 'pcl-collagen'
  | 'gelatin-alginate'
  | 'custom';

/**
 * Tissue types
 */
export type TissueType =
  | 'bone'
  | 'cartilage'
  | 'skin'
  | 'liver'
  | 'cardiac'
  | 'vascular'
  | 'neural'
  | 'muscle'
  | 'tendon'
  | 'ligament'
  | 'custom';

/**
 * Cell types
 */
export type CellType =
  | 'osteoblasts'
  | 'chondrocytes'
  | 'fibroblasts'
  | 'hepatocytes'
  | 'cardiomyocytes'
  | 'endothelial'
  | 'mesenchymal-stem'
  | 'induced-pluripotent'
  | 'custom';

// ============================================================================
// Scaffold Design
// ============================================================================

/**
 * Scaffold design parameters
 */
export interface ScaffoldDesign {
  /** Unique scaffold identifier */
  id?: string;

  /** Target tissue type */
  tissueType: TissueType;

  /** Biomaterial type */
  material: BiomaterialType;

  /** Porosity percentage (0-100) */
  porosity: number;

  /** Average pore size in micrometers */
  poreSize: number;

  /** Scaffold dimensions in millimeters */
  dimensions: Dimensions3D;

  /** Pore interconnectivity percentage (0-100) */
  interconnectivity?: number;

  /** Mechanical properties target */
  mechanicalTarget?: MechanicalProperties;

  /** Degradation time in weeks */
  degradationTime?: number;
}

/**
 * Scaffold properties
 */
export interface Scaffold {
  /** Unique identifier */
  id: string;

  /** Design parameters */
  design: ScaffoldDesign;

  /** Volume in mm³ */
  volume: number;

  /** Surface area in mm² */
  surfaceArea: number;

  /** Void volume in mm³ */
  voidVolume: number;

  /** Estimated cell capacity */
  cellCapacity: number;

  /** Mechanical properties */
  mechanicalProperties: MechanicalProperties;

  /** Recommended fabrication method */
  fabricationMethod: FabricationMethod;

  /** Quality score (0-1) */
  qualityScore: number;

  /** Creation timestamp */
  created: Date;
}

/**
 * Mechanical properties
 */
export interface MechanicalProperties {
  /** Young's modulus in Pa */
  youngsModulus: number;

  /** Ultimate tensile/compressive strength in Pa */
  ultimateStrength: number;

  /** Strain at failure (0-1) */
  strainAtFailure: number;

  /** Compressive modulus in Pa (if applicable) */
  compressiveModulus?: number;

  /** Shear modulus in Pa (if applicable) */
  shearModulus?: number;
}

/**
 * Fabrication methods
 */
export type FabricationMethod =
  | 'freeze-drying'
  | 'electrospinning'
  | 'gas-foaming'
  | 'solvent-casting'
  | '3d-printing'
  | 'bioprinting'
  | 'decellularization'
  | 'custom';

// ============================================================================
// 3D Bioprinting
// ============================================================================

/**
 * Bioprinting technology types
 */
export type BioprintingTechnology =
  | 'extrusion'
  | 'inkjet'
  | 'laser-assisted'
  | 'stereolithography'
  | 'custom';

/**
 * Bioink formulation
 */
export interface Bioink {
  /** Unique bioink identifier */
  id: string;

  /** Base polymer */
  polymer: BiomaterialType;

  /** Cell type */
  cellType: CellType;

  /** Cell density in cells/mL */
  cellDensity: number;

  /** Viscosity in mPa·s */
  viscosity: number;

  /** Storage modulus (G') in Pa */
  storageModulus: number;

  /** Loss modulus (G") in Pa */
  lossModulus: number;

  /** Cross-linking method */
  crosslinking: CrosslinkingMethod;

  /** Growth factors (optional) */
  growthFactors?: GrowthFactor[];

  /** Temperature in celsius */
  temperature: number;
}

/**
 * Cross-linking methods
 */
export type CrosslinkingMethod =
  | 'ionic'
  | 'uv'
  | 'thermal'
  | 'enzymatic'
  | 'chemical'
  | 'none';

/**
 * Growth factors
 */
export interface GrowthFactor {
  /** Growth factor name */
  name: string;

  /** Concentration in ng/mL */
  concentration: number;

  /** Release kinetics */
  releaseKinetics?: 'burst' | 'sustained' | 'controlled';
}

/**
 * Bioprinting parameters
 */
export interface BioprintingParams {
  /** Technology type */
  technology: BioprintingTechnology;

  /** Bioink formulation */
  bioink: Bioink;

  /** Printing pressure in kPa (extrusion) */
  pressure?: number;

  /** Nozzle diameter in micrometers */
  nozzleDiameter: number;

  /** Print speed in mm/s */
  printSpeed: number;

  /** Layer height in micrometers */
  layerHeight: number;

  /** Temperature in celsius */
  temperature: number;

  /** CAD model file path (STL) */
  modelFile?: string;
}

/**
 * Bioprinting result
 */
export interface BioprintingResult {
  /** Unique print job identifier */
  id: string;

  /** Parameters used */
  parameters: BioprintingParams;

  /** Print fidelity (0-1) */
  fidelity: number;

  /** Cell viability post-print (0-1) */
  cellViability: number;

  /** Print duration in seconds */
  duration: number;

  /** Success status */
  success: boolean;

  /** Errors or warnings */
  messages: string[];

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Bioreactor Systems
// ============================================================================

/**
 * Bioreactor types
 */
export type BioreactorType =
  | 'spinner-flask'
  | 'perfusion'
  | 'rotating-wall'
  | 'compression'
  | 'hollow-fiber'
  | 'custom';

/**
 * Bioreactor configuration
 */
export interface BioreactorConfig {
  /** Unique bioreactor identifier */
  id: string;

  /** Bioreactor type */
  type: BioreactorType;

  /** Volume in mL */
  volume: number;

  /** Flow rate in mL/min (if perfusion) */
  flowRate?: number;

  /** Rotation speed in RPM (if rotating) */
  rotationSpeed?: number;

  /** Compression parameters (if mechanical loading) */
  compression?: CompressionParams;

  /** Temperature in celsius */
  temperature: number;

  /** pH setpoint */
  pH: number;

  /** Oxygen tension (pO2) in percentage */
  oxygenLevel: number;

  /** CO2 percentage */
  co2Level: number;

  /** Culture duration in days */
  cultureDuration: number;
}

/**
 * Compression parameters for mechanical loading
 */
export interface CompressionParams {
  /** Frequency in Hz */
  frequency: number;

  /** Strain percentage (0-100) */
  strain: number;

  /** Loading duration in hours per day */
  duration: number;

  /** Rest period in hours */
  restPeriod: number;
}

/**
 * Culture conditions
 */
export interface CultureConditions {
  /** Bioreactor configuration */
  bioreactor: BioreactorConfig;

  /** Culture medium composition */
  medium: CultureMedium;

  /** Feeding schedule */
  feedingSchedule: FeedingSchedule;

  /** Monitoring parameters */
  monitoring: MonitoringParams;
}

/**
 * Culture medium
 */
export interface CultureMedium {
  /** Base medium type */
  baseMedium: string;

  /** Serum type and percentage */
  serum?: {
    type: string;
    percentage: number;
  };

  /** Antibiotics */
  antibiotics: string[];

  /** Growth factors */
  growthFactors?: GrowthFactor[];

  /** Glucose concentration in mM */
  glucose: number;

  /** Buffering system */
  buffer: string;
}

/**
 * Feeding schedule
 */
export interface FeedingSchedule {
  /** Medium exchange frequency in days */
  frequency: number;

  /** Percentage of medium exchanged (0-100) */
  exchangePercentage: number;

  /** Perfusion rate in mL/min (if continuous) */
  perfusionRate?: number;
}

/**
 * Monitoring parameters
 */
export interface MonitoringParams {
  /** Temperature monitoring */
  temperature: SensorConfig;

  /** pH monitoring */
  pH: SensorConfig;

  /** Dissolved oxygen monitoring */
  dissolvedOxygen: SensorConfig;

  /** Glucose monitoring */
  glucose?: SensorConfig;

  /** Lactate monitoring */
  lactate?: SensorConfig;
}

/**
 * Sensor configuration
 */
export interface SensorConfig {
  /** Sensor type */
  type: string;

  /** Sampling frequency in seconds */
  samplingFrequency: number;

  /** Accuracy/tolerance */
  accuracy: number;

  /** Alert thresholds */
  thresholds?: {
    min: number;
    max: number;
  };
}

// ============================================================================
// Tissue Construct
// ============================================================================

/**
 * Tissue construct (scaffold + cells)
 */
export interface TissueConstruct {
  /** Unique construct identifier */
  id: string;

  /** Scaffold */
  scaffold: Scaffold;

  /** Seeded cell type */
  cellType: CellType;

  /** Cell seeding density in cells/cm³ */
  seedingDensity: number;

  /** Seeding method */
  seedingMethod: SeedingMethod;

  /** Cell viability (0-1) */
  cellViability: number;

  /** Culture conditions */
  cultureConditions?: CultureConditions;

  /** Maturation time in days */
  maturationTime: number;

  /** Vascularization status */
  vascularization?: VascularizationStatus;

  /** Quality assessment */
  quality?: QualityAssessment;

  /** Creation date */
  created: Date;

  /** Last updated */
  updated: Date;
}

/**
 * Cell seeding methods
 */
export type SeedingMethod =
  | 'static'
  | 'dynamic'
  | 'vacuum'
  | 'centrifugal'
  | 'bioprinting'
  | 'custom';

/**
 * Vascularization status
 */
export interface VascularizationStatus {
  /** Vascularization strategy used */
  strategy: VascularizationStrategy;

  /** Vessel density (vessels/mm²) */
  vesselDensity?: number;

  /** Average vessel diameter in micrometers */
  vesselDiameter?: number;

  /** Perfusion functional */
  functional: boolean;

  /** Growth factors used */
  growthFactors?: GrowthFactor[];
}

/**
 * Vascularization strategies
 */
export type VascularizationStrategy =
  | 'co-culture'
  | 'microfluidic-channels'
  | 'growth-factor-delivery'
  | 'arteriovenous-loop'
  | 'in-vivo-induction'
  | 'none';

// ============================================================================
// Quality Assessment
// ============================================================================

/**
 * Quality assessment results
 */
export interface QualityAssessment {
  /** Overall quality score (0-1) */
  overallScore: number;

  /** Sterility test */
  sterility: SterilityTest;

  /** Cell viability */
  viability: ViabilityTest;

  /** Mechanical properties */
  mechanical: MechanicalTest;

  /** Biocompatibility */
  biocompatibility: BiocompatibilityTest;

  /** Functional assays */
  functional?: FunctionalAssays;

  /** Assessment date */
  assessmentDate: Date;

  /** Pass/fail status */
  passed: boolean;

  /** Recommendations */
  recommendations: string[];
}

/**
 * Sterility test results
 */
export interface SterilityTest {
  /** Bacterial contamination */
  bacterialContamination: boolean;

  /** Fungal contamination */
  fungalContamination: boolean;

  /** Endotoxin level in EU/mL */
  endotoxinLevel: number;

  /** Mycoplasma detected */
  mycoplasma: boolean;

  /** Test method */
  method: string;

  /** Pass/fail */
  passed: boolean;
}

/**
 * Viability test results
 */
export interface ViabilityTest {
  /** Viability percentage (0-100) */
  viability: number;

  /** Test method */
  method: 'live-dead' | 'mtt' | 'mts' | 'flow-cytometry' | 'custom';

  /** Cell count */
  cellCount?: number;

  /** Proliferation rate */
  proliferationRate?: number;

  /** Pass/fail (>80% threshold) */
  passed: boolean;
}

/**
 * Mechanical test results
 */
export interface MechanicalTest {
  /** Measured properties */
  measured: MechanicalProperties;

  /** Target properties */
  target: MechanicalProperties;

  /** Test method */
  method: 'compression' | 'tensile' | 'shear' | 'custom';

  /** Match percentage (0-100) */
  matchPercentage: number;

  /** Pass/fail (>80% match threshold) */
  passed: boolean;
}

/**
 * Biocompatibility test results
 */
export interface BiocompatibilityTest {
  /** Cytotoxicity (ISO 10993-5) */
  cytotoxicity: {
    viabilityPercentage: number;
    passed: boolean;
  };

  /** Immunogenicity */
  immunogenicity?: {
    cytokineLevels: { [key: string]: number };
    foldIncrease: number;
    passed: boolean;
  };

  /** In vivo response (if tested) */
  inVivo?: {
    inflammationScore: number;
    fibrosisScore: number;
    vascularization: boolean;
    passed: boolean;
  };

  /** Overall biocompatibility */
  passed: boolean;
}

/**
 * Functional assays (tissue-specific)
 */
export interface FunctionalAssays {
  /** Tissue type */
  tissueType: TissueType;

  /** Markers measured */
  markers: { [markerName: string]: number };

  /** Gene expression (fold change) */
  geneExpression?: { [gene: string]: number };

  /** Functional output (e.g., albumin, force) */
  functionalOutput?: { [metric: string]: number };

  /** Pass/fail */
  passed: boolean;
}

// ============================================================================
// Implant Readiness
// ============================================================================

/**
 * Implant readiness assessment
 */
export interface ImplantReadiness {
  /** Tissue construct */
  construct: TissueConstruct;

  /** Quality assessment */
  quality: QualityAssessment;

  /** Regulatory compliance */
  regulatoryCompliance: RegulatoryCompliance;

  /** Surgical compatibility */
  surgicalCompatibility: SurgicalCompatibility;

  /** Ready for implantation */
  ready: boolean;

  /** Risk level */
  riskLevel: 'low' | 'medium' | 'high';

  /** Recommendations */
  recommendations: string[];

  /** Assessment date */
  assessmentDate: Date;
}

/**
 * Regulatory compliance
 */
export interface RegulatoryCompliance {
  /** FDA classification */
  fdaClass?: 'I' | 'II' | 'III';

  /** ISO 10993 compliance */
  iso10993: boolean;

  /** GMP compliance */
  gmpCompliance: boolean;

  /** Documentation complete */
  documentationComplete: boolean;

  /** Batch records */
  batchRecords: boolean;

  /** Overall compliance */
  compliant: boolean;
}

/**
 * Surgical compatibility
 */
export interface SurgicalCompatibility {
  /** Appropriate size for defect */
  appropriateSize: boolean;

  /** Mechanical strength sufficient */
  mechanicalStrength: boolean;

  /** Suture retention strength */
  sutureRetention?: number;

  /** Handling characteristics */
  handling: 'excellent' | 'good' | 'fair' | 'poor';

  /** Sterilization method */
  sterilization: string;

  /** Shelf life in days */
  shelfLife: number;

  /** Compatible */
  compatible: boolean;
}

// ============================================================================
// Protocol Generation
// ============================================================================

/**
 * Tissue engineering protocol
 */
export interface TissueEngineeringProtocol {
  /** Protocol identifier */
  id: string;

  /** Protocol title */
  title: string;

  /** Target tissue type */
  tissueType: TissueType;

  /** Scaffold design */
  scaffold: ScaffoldDesign;

  /** Cell seeding protocol */
  seeding: SeedingProtocol;

  /** Culture protocol */
  culture: CultureProtocol;

  /** Quality control */
  qualityControl: QualityControlProtocol;

  /** Timeline in days */
  timeline: number;

  /** Success rate (0-1) */
  successRate?: number;

  /** References */
  references?: string[];

  /** Created date */
  created: Date;
}

/**
 * Seeding protocol
 */
export interface SeedingProtocol {
  /** Cell type */
  cellType: CellType;

  /** Cell source */
  cellSource: string;

  /** Seeding density */
  density: number;

  /** Seeding method */
  method: SeedingMethod;

  /** Incubation time before culture */
  incubationTime: number;

  /** Step-by-step instructions */
  steps: string[];
}

/**
 * Culture protocol
 */
export interface CultureProtocol {
  /** Culture conditions */
  conditions: CultureConditions;

  /** Duration in days */
  duration: number;

  /** Medium changes */
  mediumChanges: {
    day: number;
    action: string;
  }[];

  /** Monitoring schedule */
  monitoring: {
    parameter: string;
    frequency: string;
  }[];

  /** Expected milestones */
  milestones: {
    day: number;
    milestone: string;
  }[];
}

/**
 * Quality control protocol
 */
export interface QualityControlProtocol {
  /** Sterility testing */
  sterility: {
    frequency: string;
    methods: string[];
  };

  /** Viability testing */
  viability: {
    timepoints: number[];
    method: string;
    acceptanceCriteria: number;
  };

  /** Mechanical testing */
  mechanical?: {
    timepoint: number;
    method: string;
    acceptanceCriteria: MechanicalProperties;
  };

  /** Functional assays */
  functional?: {
    timepoints: number[];
    assays: string[];
  };
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Tissue engineering constants
 */
export const TISSUE_ENGINEERING_CONSTANTS = {
  /** Oxygen diffusion limit in micrometers */
  OXYGEN_DIFFUSION_LIMIT: 200,

  /** Minimum cell viability threshold */
  MIN_CELL_VIABILITY: 0.8,

  /** Minimum porosity for tissue ingrowth (%) */
  MIN_POROSITY: 60,

  /** Maximum porosity for mechanical strength (%) */
  MAX_POROSITY: 95,

  /** Optimal pore interconnectivity (%) */
  OPTIMAL_INTERCONNECTIVITY: 95,

  /** Standard culture temperature (celsius) */
  CULTURE_TEMPERATURE: 37,

  /** Standard pH */
  CULTURE_PH: 7.4,

  /** Standard CO2 level (%) */
  CULTURE_CO2: 5,

  /** Endotoxin limit (EU/mL) */
  ENDOTOXIN_LIMIT: 0.5,

  /** Cell seeding efficiency targets */
  SEEDING_EFFICIENCY: {
    STATIC: 0.35,
    DYNAMIC: 0.55,
    VACUUM: 0.75,
    CENTRIFUGAL: 0.85,
  },

  /** Typical pore sizes (micrometers) */
  PORE_SIZES: {
    OSTEOBLASTS: { min: 100, max: 400 },
    CHONDROCYTES: { min: 100, max: 300 },
    FIBROBLASTS: { min: 50, max: 150 },
    HEPATOCYTES: { min: 20, max: 50 },
    ENDOTHELIAL: { min: 30, max: 100 },
  },

  /** Material degradation times (weeks) */
  DEGRADATION_TIME: {
    COLLAGEN: { min: 2, max: 8 },
    GELATIN: { min: 1, max: 4 },
    CHITOSAN: { min: 4, max: 12 },
    PCL: { min: 24, max: 104 },
    PLA: { min: 24, max: 52 },
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
 * WIA-BIO-006 error codes
 */
export enum BioErrorCode {
  INSUFFICIENT_POROSITY = 'B001',
  LOW_CELL_VIABILITY = 'B002',
  MECHANICAL_FAILURE = 'B003',
  CONTAMINATION = 'B004',
  POOR_VASCULARIZATION = 'B005',
  BIOCOMPATIBILITY_ISSUE = 'B006',
  INVALID_PARAMETERS = 'B007',
  FABRICATION_ERROR = 'B008',
  CULTURE_FAILURE = 'B009',
  QUALITY_FAILURE = 'B010',
}

/**
 * Tissue engineering error
 */
export class TissueEngineeringError extends Error {
  constructor(
    public code: BioErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TissueEngineeringError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  Dimensions3D,
  ScaffoldDesign,
  Scaffold,
  MechanicalProperties,
  Bioink,
  GrowthFactor,
  BioprintingParams,
  BioprintingResult,
  BioreactorConfig,
  CompressionParams,
  CultureConditions,
  CultureMedium,
  FeedingSchedule,
  MonitoringParams,
  SensorConfig,
  TissueConstruct,
  VascularizationStatus,
  QualityAssessment,
  SterilityTest,
  ViabilityTest,
  MechanicalTest,
  BiocompatibilityTest,
  FunctionalAssays,
  ImplantReadiness,
  RegulatoryCompliance,
  SurgicalCompatibility,
  TissueEngineeringProtocol,
  SeedingProtocol,
  CultureProtocol,
  QualityControlProtocol,
};

export {
  TISSUE_ENGINEERING_CONSTANTS,
  BioErrorCode,
  TissueEngineeringError,
};
