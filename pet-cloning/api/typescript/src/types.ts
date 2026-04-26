/**
 * WIA-PET-009 Pet Cloning Standard - TypeScript Type Definitions
 * Version: 2.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Enums
// ============================================================================

export enum Species {
  DOG = 'Canis lupus familiaris',
  CAT = 'Felis catus',
  HORSE = 'Equus ferus caballus',
}

export enum CellType {
  FIBROBLAST = 'fibroblast',
  KERATINOCYTE = 'keratinocyte',
  EPITHELIAL = 'epithelial',
}

export enum SampleType {
  SKIN_BIOPSY = 'skin_biopsy',
  EAR_TISSUE = 'ear_tissue',
  MUSCLE_TISSUE = 'muscle_tissue',
}

export enum OocyteGrade {
  A = 'A', // Optimal
  B = 'B', // Good
  C = 'C', // Marginal
  D = 'D', // Reject
}

export enum BlastocystGrade {
  AA = 'AA', // Excellent
  AB = 'AB', // Good
  BA = 'BA', // Good
  BB = 'BB', // Fair
  BC = 'BC', // Poor
  CB = 'CB', // Poor
  CC = 'CC', // Very poor
}

export enum ProcedureStatus {
  PENDING = 'pending',
  IN_PROGRESS = 'in_progress',
  COMPLETED = 'completed',
  FAILED = 'failed',
  CANCELLED = 'cancelled',
}

export enum PregnancyStatus {
  NOT_PREGNANT = 'not_pregnant',
  CONFIRMED = 'confirmed',
  ONGOING = 'ongoing',
  TERM = 'term',
  LOST = 'lost',
}

// ============================================================================
// Pet Profile
// ============================================================================

export interface PetProfile {
  id: string;
  species: Species;
  breed: string;
  name: string;
  dateOfBirth: Date;
  sex: 'male' | 'female';
  weight?: number; // in kg
  color?: string;
  markings?: string;
  microchipId?: string;
  owner: OwnerInfo;
  healthHistory?: HealthRecord[];
  photos?: string[]; // URLs to photos
}

export interface OwnerInfo {
  id: string;
  name: string;
  email: string;
  phone: string;
  address: Address;
}

export interface Address {
  street: string;
  city: string;
  state: string;
  postalCode: string;
  country: string;
}

export interface HealthRecord {
  date: Date;
  type: 'vaccination' | 'illness' | 'surgery' | 'checkup' | 'other';
  description: string;
  veterinarian?: string;
  documents?: string[];
}

// ============================================================================
// Genetic Sample
// ============================================================================

export interface GeneticSample {
  id: string;
  petId: string;
  sampleType: SampleType;
  cellType: CellType;
  collectionDate: Date;
  collectedBy: string; // Veterinarian name/ID
  collectionLocation: string; // Facility/clinic
  isPostMortem: boolean;
  postMortemHours?: number; // Hours since death (if applicable)

  // Quality Metrics
  viability?: number; // Percentage (0-100)
  dnaIntegrity?: number; // Score (0-1.0)
  chromosomeCount?: number;
  passageNumber?: number;
  contaminationCheck?: ContaminationResult;

  // Storage
  storageLocation?: string; // e.g., "CRYO-VAULT-12-A-034"
  storageTemperature?: number; // in Celsius
  cryopreservationDate?: Date;

  // Status
  status: 'collected' | 'processing' | 'stored' | 'thawed' | 'used' | 'discarded';
  notes?: string;
}

export interface ContaminationResult {
  bacterial: boolean;
  fungal: boolean;
  mycoplasma: boolean;
  percentage: number;
  testDate: Date;
}

// ============================================================================
// Cloning Request
// ============================================================================

export interface CloningRequest {
  id: string;
  petId: string;
  sampleId: string;
  requestDate: Date;
  requestedBy: string; // Owner ID
  priority: 'standard' | 'expedited' | 'research';
  estimatedCompletion?: Date;
  status: ProcedureStatus;

  // Approvals
  ethicsReviewId?: string;
  ethicsApprovalDate?: Date;
  veterinaryApproval?: boolean;

  // Associated Procedures
  scntProcedureIds?: string[];
  embryoTransferId?: string;
  pregnancyId?: string;

  // Outcome
  livebirthId?: string;
  completionDate?: Date;

  // Financial
  quotedCost?: number;
  actualCost?: number;
  currency?: string;
}

// ============================================================================
// SCNT Procedure
// ============================================================================

export interface SCNTProcedure {
  id: string;
  cloningRequestId: string;
  date: Date;
  performedBy: string; // Technician ID
  supervisedBy: string; // Senior embryologist ID

  // Oocytes
  oocyteSource: OocyteSource;
  oocytesCollected: number;
  oocyteQualityDistribution: Record<OocyteGrade, number>;
  oocytesUsed: number;

  // Enucleation
  enucleationAttempts: number;
  enucleationSuccesses: number;
  enucleationRate: number; // Percentage

  // Fusion
  fusionAttempts: number;
  fusionSuccesses: number;
  fusionRate: number; // Percentage
  fusionMethod: 'electrofusion' | 'direct_injection';
  fusionParameters?: ElectrofusionParameters;

  // Activation
  activationMethod: 'calcium_ionophore' | 'electrical' | 'strontium' | 'combination';
  activationProtocol?: string;

  // Results
  reconstructedEmbryos: number;
  embryoIds: string[];

  // Quality Metrics
  overallSuccessRate?: number;
  notes?: string;
}

export interface OocyteSource {
  donorAnimalId: string;
  donorSpecies: Species;
  donorBreed: string;
  donorAge: number; // in years
  collectionDate: Date;
  stimulationProtocol?: string;
}

export interface ElectrofusionParameters {
  alignmentVoltage: number; // V
  alignmentDuration: number; // seconds
  fusionVoltage: number; // kV/cm
  fusionDuration: number; // microseconds
  pulseCount: number;
  pulseInterval: number; // seconds
}

// ============================================================================
// Embryo Development
// ============================================================================

export interface Embryo {
  id: string;
  scntProcedureId: string;
  creationDate: Date;

  // Development Tracking
  developmentLog: DevelopmentCheckpoint[];
  currentStage: EmbryoStage;

  // Quality
  blastocystGrade?: BlastocystGrade;
  qualityScore?: number; // 0-1.0

  // Culture Conditions
  cultureSystem: 'microdrop' | 'microfluidic' | 'group';
  mediaUsed: string[];
  incubatorId: string;

  // Outcome
  transferred: boolean;
  transferDate?: Date;
  transferId?: string;
  cryopreserved: boolean;
  cryopreservationDate?: Date;
  discarded: boolean;
  discardReason?: string;
}

export type EmbryoStage =
  | 'zygote'
  | '2-cell'
  | '4-cell'
  | '8-cell'
  | 'morula'
  | 'early_blastocyst'
  | 'blastocyst'
  | 'expanded_blastocyst'
  | 'hatching_blastocyst'
  | 'arrested';

export interface DevelopmentCheckpoint {
  timestamp: Date;
  hoursPostActivation: number;
  stage: EmbryoStage;
  cellCount?: number;
  morphology: 'excellent' | 'good' | 'fair' | 'poor';
  notes?: string;
  imagePath?: string;
}

// ============================================================================
// Embryo Transfer
// ============================================================================

export interface EmbryoTransfer {
  id: string;
  cloningRequestId: string;
  date: Date;
  performedBy: string; // Veterinarian ID
  assistedBy?: string[];

  // Surrogate
  surrogateId: string;
  surrogateSpecies: Species;
  surrogateAge: number;
  surrogateReproductiveHistory?: string;

  // Synchronization
  synchronizationMethod: 'natural_cycle' | 'hormonal_induction';
  synchronizationProtocol?: string;
  dayOfCycle: number;
  progesteroneLevel?: number; // ng/ml

  // Transfer Details
  transferMethod: 'surgical' | 'nonsurgical';
  embryosTransferred: string[]; // Embryo IDs
  embryoCount: number;
  transferQuality: 'excellent' | 'good' | 'fair' | 'poor';

  // Outcome
  pregnancyEstablished: boolean;
  pregnancyId?: string;
  complications?: string;
  notes?: string;
}

// ============================================================================
// Pregnancy Management
// ============================================================================

export interface Pregnancy {
  id: string;
  embryoTransferId: string;
  surrogateId: string;

  // Confirmation
  confirmationDate: Date;
  confirmationMethod: 'ultrasound' | 'blood_test' | 'palpation';

  // Monitoring
  checkups: PregnancyCheckup[];
  expectedDueDate: Date;
  actualDueDate?: Date;

  // Status
  status: PregnancyStatus;
  fetalCount?: number;

  // Complications
  complications?: PregnancyComplication[];

  // Outcome
  birthMethod?: 'natural' | 'cesarean' | 'assisted';
  offspring?: string[]; // Clone IDs
}

export interface PregnancyCheckup {
  date: Date;
  dayOfPregnancy: number;
  method: 'ultrasound' | 'physical_exam' | 'bloodwork';
  fetalHeartbeat: boolean;
  fetalMovement: boolean;
  placentalStatus: 'normal' | 'abnormal';
  maternalHealth: 'good' | 'fair' | 'concerning';
  findings?: string;
  veterinarianId: string;
}

export interface PregnancyComplication {
  date: Date;
  type: string;
  severity: 'mild' | 'moderate' | 'severe';
  treatment?: string;
  resolved: boolean;
  resolutionDate?: Date;
}

// ============================================================================
// Clone (Offspring)
// ============================================================================

export interface Clone {
  id: string;
  originalPetId: string; // The donor pet
  pregnancyId: string;

  // Birth Information
  birthDate: Date;
  birthWeight: number; // in grams
  birthMethod: 'natural' | 'cesarean';

  // Identification
  microchipId?: string;
  registrationId?: string;
  sex: 'male' | 'female';

  // Neonatal Health
  apgarScore?: number; // If applicable
  neonatalCheckups: NeonatalCheckup[];

  // Development
  milestones: DevelopmentMilestone[];

  // Current Status
  alive: boolean;
  currentAge?: number; // in days/months
  currentWeight?: number; // in kg
  currentOwner?: string; // Owner ID

  // Health
  healthRecords: HealthRecord[];
  geneticPredispositions?: string[];

  // Long-term Tracking
  qualityOfLife?: 'excellent' | 'good' | 'fair' | 'poor';
  behavioralNotes?: string;
}

export interface NeonatalCheckup {
  age: number; // hours or days
  weight: number; // grams
  temperature: number; // Celsius
  nursing: 'excellent' | 'good' | 'poor' | 'none';
  activity: 'active' | 'moderate' | 'lethargic';
  findings?: string;
  veterinarianId: string;
}

export interface DevelopmentMilestone {
  age: number; // in days
  milestone: string;
  achieved: boolean;
  date?: Date;
  notes?: string;
}

// ============================================================================
// Success Prediction
// ============================================================================

export interface SuccessPrediction {
  predictedSuccessRate: number; // Percentage (0-100)
  confidence: number; // Percentage (0-100)

  factors: {
    cellViability: FactorImpact;
    dnaIntegrity: FactorImpact;
    passageNumber: FactorImpact;
    species: FactorImpact;
    technicianExperience: FactorImpact;
    labQuality: FactorImpact;
    oocyteQuality?: FactorImpact;
  };

  recommendations?: string[];
  warnings?: string[];
}

export interface FactorImpact {
  value: number | string;
  impact: 'positive' | 'neutral' | 'negative';
  weight: number; // 0-1.0
  notes?: string;
}

// ============================================================================
// Quality Control
// ============================================================================

export interface QualityControlResult {
  id: string;
  date: Date;
  type: 'media_batch' | 'reagent' | 'equipment' | 'procedure';
  testName: string;

  // Results
  passed: boolean;
  value?: number | string;
  acceptanceCriteria: string;

  // Details
  batchNumber?: string;
  lotNumber?: string;
  equipment?: string;
  performedBy: string;

  // Actions
  actionRequired?: string;
  actionTaken?: string;
  notes?: string;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  meta?: ResponseMeta;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
}

export interface ResponseMeta {
  requestId: string;
  timestamp: Date;
  version: string;
}

export interface PaginatedResponse<T> extends APIResponse<T[]> {
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalItems: number;
  };
}

// ============================================================================
// Configuration
// ============================================================================

export interface SDKConfig {
  apiKey: string;
  environment: 'production' | 'staging' | 'development';
  baseURL?: string;
  timeout?: number; // milliseconds
  retryAttempts?: number;
  debug?: boolean;
}
