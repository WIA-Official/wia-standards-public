/**
 * WIA-CANCER-METABOLISM Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 * © 2025 WIA - World Certification Industry Association
 */

// ============================================================================
// Core Enums
// ============================================================================

export enum CancerType {
  LUNG = 'lung',
  BREAST = 'breast',
  COLORECTAL = 'colorectal',
  PANCREATIC = 'pancreatic',
  LIVER = 'liver',
  GASTRIC = 'gastric',
  PROSTATE = 'prostate',
  LEUKEMIA = 'leukemia',
  LYMPHOMA = 'lymphoma',
  MELANOMA = 'melanoma',
  BRAIN = 'brain',
  KIDNEY = 'kidney',
  BLADDER = 'bladder',
  OVARIAN = 'ovarian',
  THYROID = 'thyroid'
}

export enum MetabolicPathway {
  GLYCOLYSIS = 'glycolysis',
  TCA_CYCLE = 'tca_cycle',
  PENTOSE_PHOSPHATE = 'pentose_phosphate',
  GLUTAMINOLYSIS = 'glutaminolysis',
  LIPID_SYNTHESIS = 'lipid_synthesis',
  OXIDATIVE_PHOSPHORYLATION = 'oxidative_phosphorylation',
  ONE_CARBON = 'one_carbon',
  AMINO_ACID = 'amino_acid',
  NUCLEOTIDE_SYNTHESIS = 'nucleotide_synthesis',
  FATTY_ACID_OXIDATION = 'fatty_acid_oxidation'
}

export enum BiomarkerType {
  GENE_EXPRESSION = 'gene_expression',
  PROTEIN = 'protein',
  METABOLITE = 'metabolite',
  ENZYME_ACTIVITY = 'enzyme_activity',
  MUTATION = 'mutation'
}

export enum ExpressionLevel {
  VERY_LOW = 'very_low',
  LOW = 'low',
  NORMAL = 'normal',
  ELEVATED = 'elevated',
  VERY_ELEVATED = 'very_elevated'
}

export enum WarburgStatus {
  NORMAL = 'normal',
  MILD = 'mild',
  MODERATE = 'moderate',
  ELEVATED = 'elevated',
  SEVERE = 'severe'
}

export enum CertificationLevel {
  BRONZE = 'bronze',
  SILVER = 'silver',
  GOLD = 'gold',
  PLATINUM = 'platinum'
}

// ============================================================================
// Core Interfaces
// ============================================================================

export interface WIAMetadata {
  '@context': string;
  '@type': string;
  version: string;
  timestamp: string;
  philosophy: string;
}

export interface PatientInfo {
  id: string;
  anonymizedId?: string;
  age?: number;
  sex?: 'male' | 'female' | 'other';
  ethnicity?: string;
  consentStatus: boolean;
  consentDate?: string;
}

export interface CancerDiagnosis {
  type: CancerType;
  stage?: string;
  grade?: string;
  histology?: string;
  diagnosisDate?: string;
  primarySite?: string;
  metastatic?: boolean;
  metastaticSites?: string[];
}

export interface Biomarker {
  id: string;
  name: string;
  symbol: string;
  type: BiomarkerType;
  value: number;
  unit: string;
  referenceRange?: {
    low: number;
    high: number;
  };
  expression: ExpressionLevel;
  foldChange?: number;
  pValue?: number;
  methodology?: string;
  timestamp: string;
}

export interface WarburgEffect {
  index: number; // 0-100
  status: WarburgStatus;
  glycolysisRate: number; // mmol/L/h
  lactateLevel: number; // mmol/L
  oxygenConsumption?: number;
  atpProduction?: number;
}

export interface MetaboliteProfile {
  id: string;
  name: string;
  casNumber?: string;
  hmdbId?: string;
  pubchemId?: string;
  concentration: number;
  unit: string;
  pathway: MetabolicPathway;
  detectionMethod?: string;
  qualityScore?: number;
}

export interface PathwayAnalysis {
  pathway: MetabolicPathway;
  activityScore: number; // 0-100
  enrichmentPValue?: number;
  affectedGenes: string[];
  affectedMetabolites: string[];
  therapeuticTargets?: string[];
}

export interface MetabolicProfile {
  warburgEffect: WarburgEffect;
  biomarkers: Biomarker[];
  metabolites: MetaboliteProfile[];
  pathways: PathwayAnalysis[];
  sampleType?: string;
  sampleDate?: string;
  qualityMetrics?: QualityMetrics;
}

export interface QualityMetrics {
  overallScore: number;
  completeness: number;
  accuracy: number;
  reproducibility?: number;
  sampleQuality?: number;
}

export interface ComplianceInfo {
  hipaaCompliant: boolean;
  gdprCompliant: boolean;
  wiaCertified: boolean;
  certificationLevel?: CertificationLevel;
  certificationDate?: string;
  auditTrail?: AuditEntry[];
}

export interface AuditEntry {
  timestamp: string;
  action: string;
  actor: string;
  details?: string;
}

// ============================================================================
// Main Data Structure
// ============================================================================

export interface CancerMetabolismProfile extends WIAMetadata {
  patientId: string;
  patient?: PatientInfo;
  diagnosis: CancerDiagnosis;
  metabolicProfile: MetabolicProfile;
  compliance: ComplianceInfo;
  researchStudy?: ResearchStudyInfo;
}

export interface ResearchStudyInfo {
  studyId: string;
  studyName?: string;
  institution: string;
  principalInvestigator?: string;
  protocol?: string;
  irbApproval?: string;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  metadata: ResponseMetadata;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

export interface ResponseMetadata {
  requestId: string;
  timestamp: string;
  processingTime: number;
  version: string;
}

export interface PaginationParams {
  page?: number;
  limit?: number;
  offset?: number;
  cursor?: string;
}

export interface PaginatedResponse<T> extends APIResponse<T[]> {
  pagination: {
    total: number;
    page: number;
    limit: number;
    hasMore: boolean;
    nextCursor?: string;
  };
}

export interface SearchQuery {
  cancerType?: CancerType;
  pathway?: MetabolicPathway;
  biomarkerName?: string;
  warburgIndexMin?: number;
  warburgIndexMax?: number;
  dateFrom?: string;
  dateTo?: string;
}

// ============================================================================
// Protocol Message Types
// ============================================================================

export interface ProtocolMessage {
  header: MessageHeader;
  payload: unknown;
  signature?: string;
}

export interface MessageHeader {
  version: string;
  messageId: string;
  type: MessageType;
  timestamp: string;
  source: InstitutionInfo;
  target?: InstitutionInfo;
  priority?: 'low' | 'normal' | 'high' | 'urgent';
}

export type MessageType =
  | 'biomarker_query'
  | 'metabolite_update'
  | 'pathway_status'
  | 'treatment_response'
  | 'research_sync'
  | 'compliance_check';

export interface InstitutionInfo {
  id: string;
  name: string;
  type: 'hospital' | 'research' | 'laboratory' | 'biobank';
  country?: string;
  wiaCertified?: boolean;
}

// ============================================================================
// Validation Types
// ============================================================================

export interface ValidationResult {
  valid: boolean;
  errors: ValidationError[];
  warnings: ValidationWarning[];
}

export interface ValidationError {
  path: string;
  code: string;
  message: string;
}

export interface ValidationWarning {
  path: string;
  code: string;
  message: string;
  suggestion?: string;
}

// ============================================================================
// SDK Configuration
// ============================================================================

export interface SDKConfig {
  apiKey: string;
  baseUrl?: string;
  timeout?: number;
  retryAttempts?: number;
  enableLogging?: boolean;
}

export const DEFAULT_CONFIG: Partial<SDKConfig> = {
  baseUrl: 'https://api.wia-cancer-metabolism.org/v1',
  timeout: 30000,
  retryAttempts: 3,
  enableLogging: false
};

// ============================================================================
// Constants
// ============================================================================

export const WIA_CANCER_METABOLISM_VERSION = '1.0.0';
export const WIA_SCHEMA_CONTEXT = 'https://wiastandards.com/schemas/cancer-metabolism/v1';
export const WIA_PHILOSOPHY = '弘益人間 (Benefit All Humanity)';

/**
 * Key cancer metabolism biomarkers
 */
export const KEY_BIOMARKERS = {
  HIF1A: { name: 'Hypoxia-inducible factor 1-alpha', normalRange: [0.5, 2.0] },
  GLUT1: { name: 'Glucose transporter 1', normalRange: [0.8, 1.5] },
  LDHA: { name: 'Lactate dehydrogenase A', normalRange: [0.5, 1.8] },
  PKM2: { name: 'Pyruvate kinase M2', normalRange: [0.6, 1.6] },
  VEGF: { name: 'Vascular endothelial growth factor', normalRange: [0.3, 2.0] },
  MYC: { name: 'MYC proto-oncogene', normalRange: [0.5, 1.5] },
  PTEN: { name: 'Phosphatase and tensin homolog', normalRange: [0.8, 1.2] },
  TP53: { name: 'Tumor protein P53', normalRange: [0.9, 1.1] }
} as const;
