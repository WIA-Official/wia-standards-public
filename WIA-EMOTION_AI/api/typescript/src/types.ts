/**
 * WIA Emotion AI Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIAEmotionAI {
  standard: 'WIA-EMOTION-AI';
  version: string;
  system: EmotionAISystem;
  models: EmotionModel[];
  inputs: EmotionInput[];
  outputs: EmotionOutput[];
  ethics: EthicsFramework;
  privacy: PrivacyConfig;
  validation: ValidationConfig;
  extensions?: Record<string, unknown>;
}

export interface EmotionAISystem {
  id: string;
  name: string;
  type: SystemType;
  status: SystemStatus;
  capabilities: Capability[];
  modalities: Modality[];
  deployment: DeploymentInfo;
  createdAt: string;
}

export type SystemType = 'recognition' | 'generation' | 'analysis' | 'response' | 'multimodal';
export type SystemStatus = 'development' | 'testing' | 'production' | 'deprecated';
export type Modality = 'facial' | 'voice' | 'text' | 'physiological' | 'behavioral' | 'multimodal';

export interface Capability {
  type: CapabilityType;
  supported: boolean;
  accuracy?: number;
  languages?: string[];
}

export type CapabilityType =
  | 'basic-emotions'
  | 'complex-emotions'
  | 'sentiment'
  | 'affect'
  | 'engagement'
  | 'stress'
  | 'empathy'
  | 'personality';

export interface DeploymentInfo {
  environment: 'edge' | 'cloud' | 'hybrid';
  latency: number;
  throughput: number;
  availability: number;
}

// ============================================================================
// Model Types
// ============================================================================

export interface EmotionModel {
  id: string;
  name: string;
  version: string;
  type: ModelType;
  architecture: ModelArchitecture;
  training: TrainingInfo;
  performance: PerformanceMetrics;
  bias: BiasAssessment;
  status: 'training' | 'validated' | 'deployed' | 'retired';
}

export type ModelType = 'classification' | 'regression' | 'generation' | 'embedding' | 'multimodal';

export interface ModelArchitecture {
  framework: string;
  baseModel?: string;
  layers?: number;
  parameters?: number;
  inputSize: InputSpec;
  outputSize: OutputSpec;
}

export interface InputSpec {
  type: string;
  dimensions: number[];
  format: string;
}

export interface OutputSpec {
  type: string;
  classes?: string[];
  dimensions?: number[];
}

export interface TrainingInfo {
  dataset: DatasetInfo;
  epochs: number;
  batchSize: number;
  optimizer: string;
  learningRate: number;
  augmentation: string[];
  completedAt?: string;
}

export interface DatasetInfo {
  name: string;
  size: number;
  demographics: Demographics;
  annotations: AnnotationInfo;
  consent: boolean;
}

export interface Demographics {
  ageGroups: string[];
  genders: string[];
  ethnicities: string[];
  cultures: string[];
  balanced: boolean;
}

export interface AnnotationInfo {
  method: 'expert' | 'crowdsourced' | 'self-reported' | 'hybrid';
  annotators: number;
  agreement: number;
}

export interface PerformanceMetrics {
  accuracy: number;
  precision: number;
  recall: number;
  f1Score: number;
  confusionMatrix?: number[][];
  perClassMetrics?: { [className: string]: { precision: number; recall: number; f1: number } };
}

export interface BiasAssessment {
  tested: boolean;
  demographicParity: number;
  equalizedOdds: number;
  disparateImpact: number;
  findings: BiasFinding[];
  mitigations: string[];
}

export interface BiasFinding {
  category: string;
  severity: 'low' | 'medium' | 'high';
  description: string;
  affected: string[];
}

// ============================================================================
// Input Types
// ============================================================================

export interface EmotionInput {
  id: string;
  type: InputType;
  format: string;
  preprocessing: PreprocessingConfig;
  quality: QualityRequirements;
  consent: ConsentInfo;
}

export type InputType = 'image' | 'video' | 'audio' | 'text' | 'physiological' | 'behavioral';

export interface PreprocessingConfig {
  steps: PreprocessingStep[];
  normalization: boolean;
  augmentation: boolean;
}

export interface PreprocessingStep {
  operation: string;
  parameters: Record<string, unknown>;
  order: number;
}

export interface QualityRequirements {
  minimum: QualitySpec;
  recommended: QualitySpec;
}

export interface QualitySpec {
  resolution?: number[];
  sampleRate?: number;
  bitDepth?: number;
  snr?: number;
}

export interface ConsentInfo {
  required: boolean;
  type: ConsentType;
  granularity: 'session' | 'purpose' | 'data-type';
  revocable: boolean;
}

export type ConsentType = 'explicit' | 'implicit' | 'opt-out';

// ============================================================================
// Output Types
// ============================================================================

export interface EmotionOutput {
  id: string;
  type: OutputType;
  emotions: EmotionPrediction[];
  confidence: number;
  timestamp: string;
  metadata: OutputMetadata;
}

export type OutputType = 'discrete' | 'dimensional' | 'action-units' | 'compound';

export interface EmotionPrediction {
  emotion: EmotionLabel;
  probability: number;
  intensity?: number;
  valence?: number;
  arousal?: number;
  dominance?: number;
}

export type EmotionLabel =
  | 'joy' | 'sadness' | 'anger' | 'fear' | 'surprise' | 'disgust' | 'contempt'
  | 'neutral' | 'anxiety' | 'excitement' | 'frustration' | 'confusion'
  | 'interest' | 'boredom' | 'amusement' | 'pride' | 'shame' | 'guilt';

export interface OutputMetadata {
  modelId: string;
  processingTime: number;
  inputQuality: number;
  explainability?: ExplainabilityInfo;
}

export interface ExplainabilityInfo {
  method: 'gradcam' | 'lime' | 'shap' | 'attention';
  regions?: AttentionRegion[];
  features?: FeatureImportance[];
}

export interface AttentionRegion {
  coordinates: number[];
  importance: number;
  label?: string;
}

export interface FeatureImportance {
  feature: string;
  importance: number;
  direction: 'positive' | 'negative';
}

// ============================================================================
// Ethics Types
// ============================================================================

export interface EthicsFramework {
  principles: EthicsPrinciple[];
  governance: GovernanceStructure;
  audits: AuditRecord[];
  compliance: ComplianceStatus[];
}

export interface EthicsPrinciple {
  name: string;
  description: string;
  implementation: string[];
  monitoring: string[];
}

export interface GovernanceStructure {
  board: GovernanceBoard;
  policies: Policy[];
  escalation: EscalationProcess;
}

export interface GovernanceBoard {
  members: { name: string; role: string; expertise: string }[];
  meetingFrequency: string;
  decisionProcess: string;
}

export interface Policy {
  id: string;
  name: string;
  scope: string;
  requirements: string[];
  lastReviewed: string;
}

export interface EscalationProcess {
  levels: { level: number; criteria: string; responders: string[] }[];
  sla: number;
}

export interface AuditRecord {
  id: string;
  type: 'internal' | 'external' | 'regulatory';
  date: string;
  findings: string[];
  status: 'passed' | 'failed' | 'conditional';
  actions: string[];
}

export interface ComplianceStatus {
  regulation: string;
  status: 'compliant' | 'non-compliant' | 'pending';
  lastAssessed: string;
  gaps?: string[];
}

// ============================================================================
// Privacy Types
// ============================================================================

export interface PrivacyConfig {
  dataMinimization: boolean;
  retention: RetentionPolicy;
  anonymization: AnonymizationConfig;
  rights: DataRights;
  security: SecurityMeasures;
}

export interface RetentionPolicy {
  rawData: number;
  processedData: number;
  aggregated: number;
  unit: 'hours' | 'days' | 'months';
}

export interface AnonymizationConfig {
  techniques: AnonymizationTechnique[];
  kAnonymity?: number;
  differentialPrivacy?: { epsilon: number; delta: number };
}

export type AnonymizationTechnique = 'face-blur' | 'voice-mod' | 'aggregation' | 'pseudonymization' | 'encryption';

export interface DataRights {
  access: boolean;
  rectification: boolean;
  erasure: boolean;
  portability: boolean;
  objection: boolean;
  restriction: boolean;
}

export interface SecurityMeasures {
  encryption: { atRest: boolean; inTransit: boolean; algorithm: string };
  accessControl: { type: string; mfa: boolean };
  audit: boolean;
  incidentResponse: boolean;
}

// ============================================================================
// Validation Types
// ============================================================================

export interface ValidationConfig {
  crossCultural: CrossCulturalValidation;
  clinical: ClinicalValidation;
  realWorld: RealWorldValidation;
}

export interface CrossCulturalValidation {
  cultures: string[];
  adaptations: { culture: string; modifications: string[] }[];
  universality: number;
}

export interface ClinicalValidation {
  populations: string[];
  protocols: string[];
  approvals: string[];
  contraindications: string[];
}

export interface RealWorldValidation {
  environments: string[];
  conditions: string[];
  performance: { environment: string; accuracy: number }[];
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface SystemResponse {
  id: string;
  name: string;
  type: SystemType;
  status: SystemStatus;
  modalities: Modality[];
  links: { self: string };
}

export interface ValidationResult {
  valid: boolean;
  errors?: { path: string; message: string }[];
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: { total: number; limit: number; offset: number; hasMore: boolean };
}
