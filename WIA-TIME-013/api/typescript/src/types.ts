/**
 * WIA-TIME-013: Consciousness Transfer - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Consciousness Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Consciousness Types
// ============================================================================

/**
 * Three-dimensional vector for spatial coordinates
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Complex number representation
 */
export interface ComplexNumber {
  real: number;
  imaginary: number;
}

/**
 * Quantum state representation
 */
export interface QuantumState {
  /** State vector amplitudes (complex numbers) */
  amplitudes: ComplexNumber[];

  /** Basis state labels */
  basisStates: string[];

  /** Entanglement map */
  entanglement?: EntanglementMap;

  /** Fidelity to ideal state */
  fidelity: number;

  /** Quantum coherence measure */
  coherence: number;
}

/**
 * Entanglement between quantum states
 */
export interface EntanglementMap {
  /** Pairs of entangled states */
  entangledPairs: [number, number][];

  /** Entanglement strength (0-1) */
  strength: number[];

  /** Schmidt coefficients */
  schmidtCoefficients?: number[];
}

/**
 * Temporal coordinate in spacetime
 */
export interface TemporalCoordinate {
  /** Time point (ISO 8601) */
  time: Date | string;

  /** Spatial position */
  position: Vector3;

  /** Reference frame */
  referenceFrame?: string;

  /** Timeline identifier */
  timelineId?: string;
}

// ============================================================================
// Neural Pattern Types
// ============================================================================

/**
 * Neural scanning resolution level
 */
export type ScanResolution = 'structural' | 'functional' | 'quantum';

/**
 * Neural scanning parameters
 */
export interface NeuralMappingRequest {
  /** Subject identifier */
  subjectId: string;

  /** Scanning resolution level */
  resolution: ScanResolution;

  /** Include episodic memories */
  includeMemories: boolean;

  /** Include emotional states */
  includeEmotions: boolean;

  /** Scan duration in seconds */
  scanDuration?: number;

  /** Quantum coherence preservation */
  preserveCoherence?: boolean;
}

/**
 * Neural pattern mapping result
 */
export interface NeuralMappingResult {
  /** Unique mapping identifier */
  mappingId: string;

  /** Subject identifier */
  subjectId: string;

  /** Number of neurons mapped */
  neuronCount: number;

  /** Number of synapses mapped */
  synapseCount: number;

  /** Number of quantum states captured */
  quantumStates: number;

  /** Memory capacity in bits */
  memoryCapacity: number;

  /** Consciousness entropy in bits */
  entropy: number;

  /** Mapping fidelity (0-1) */
  fidelity: number;

  /** Total data size in bytes */
  dataSize: number;

  /** Scan completion time in ms */
  scanTime: number;

  /** Timestamp of scan */
  timestamp: Date;

  /** Neural graph data */
  neuralGraph?: NeuralGraph;
}

/**
 * Neural network graph representation
 */
export interface NeuralGraph {
  /** Neuron vertices */
  neurons: Neuron[];

  /** Synapse edges */
  synapses: Synapse[];

  /** Graph metadata */
  metadata: {
    totalNeurons: number;
    totalSynapses: number;
    averageDegree: number;
    clusteringCoefficient: number;
  };
}

/**
 * Individual neuron
 */
export interface Neuron {
  /** Neuron ID */
  id: string;

  /** Neuron type */
  type: 'excitatory' | 'inhibitory' | 'modulatory';

  /** Position in 3D space */
  position: Vector3;

  /** Current activation state */
  activation: number;

  /** Quantum state */
  quantumState?: QuantumState;

  /** Neurotransmitters */
  neurotransmitters?: string[];
}

/**
 * Synaptic connection
 */
export interface Synapse {
  /** Synapse ID */
  id: string;

  /** Source neuron ID */
  source: string;

  /** Target neuron ID */
  target: string;

  /** Synaptic weight (-1 to 1) */
  weight: number;

  /** Plasticity factor */
  plasticity: number;

  /** Neurotransmitter type */
  neurotransmitter: string;

  /** Delay in ms */
  delay?: number;
}

// ============================================================================
// Memory Types
// ============================================================================

/**
 * Memory types
 */
export type MemoryType = 'episodic' | 'semantic' | 'procedural';

/**
 * Memory store containing all memory types
 */
export interface MemoryStore {
  /** Episodic memories (experiences) */
  episodic: EpisodicMemory[];

  /** Semantic memories (knowledge) */
  semantic: SemanticMemory;

  /** Procedural memories (skills) */
  procedural: ProceduralMemory[];

  /** Total memory capacity in bits */
  totalCapacity: number;

  /** Memory integrity score (0-1) */
  integrity: number;
}

/**
 * Episodic memory - autobiographical events
 */
export interface EpisodicMemory {
  /** Memory ID */
  id: string;

  /** Timestamp of event */
  timestamp: Date;

  /** Event description */
  description: string;

  /** Contextual information */
  context: {
    location?: string;
    people?: string[];
    sensory?: SensoryData;
  };

  /** Emotional tag */
  emotion: EmotionalTag;

  /** Vividness (0-1) */
  vividness: number;

  /** Confidence (0-1) */
  confidence: number;
}

/**
 * Semantic memory - knowledge and facts
 */
export interface SemanticMemory {
  /** Concept graph */
  concepts: ConceptNode[];

  /** Associations between concepts */
  associations: ConceptAssociation[];

  /** Knowledge domains */
  domains: string[];
}

/**
 * Concept node in semantic network
 */
export interface ConceptNode {
  /** Concept ID */
  id: string;

  /** Concept label */
  label: string;

  /** Concept category */
  category: string;

  /** Activation strength */
  activation: number;

  /** Related properties */
  properties?: Record<string, unknown>;
}

/**
 * Association between concepts
 */
export interface ConceptAssociation {
  /** Source concept */
  source: string;

  /** Target concept */
  target: string;

  /** Association type */
  type: string;

  /** Association strength (0-1) */
  strength: number;
}

/**
 * Procedural memory - skills and procedures
 */
export interface ProceduralMemory {
  /** Skill ID */
  id: string;

  /** Skill name */
  name: string;

  /** Skill category */
  category: 'motor' | 'cognitive' | 'perceptual';

  /** State-action mapping */
  stateActionMap: StateActionPair[];

  /** Proficiency level (0-1) */
  proficiency: number;

  /** Training time in hours */
  trainingHours?: number;
}

/**
 * State-action pair for procedural memory
 */
export interface StateActionPair {
  /** State description */
  state: Record<string, unknown>;

  /** Action to take */
  action: string;

  /** Expected outcome */
  outcome: string;

  /** Timing constraints */
  timing?: {
    duration: number;
    precision: number;
  };
}

/**
 * Sensory data
 */
export interface SensoryData {
  /** Visual information */
  visual?: string;

  /** Auditory information */
  auditory?: string;

  /** Tactile information */
  tactile?: string;

  /** Olfactory information */
  olfactory?: string;

  /** Gustatory information */
  gustatory?: string;
}

/**
 * Emotional tag
 */
export interface EmotionalTag {
  /** Valence (-1 to 1, negative to positive) */
  valence: number;

  /** Arousal (0 to 1, calm to excited) */
  arousal: number;

  /** Dominance (-1 to 1, submissive to dominant) */
  dominance: number;

  /** Discrete emotion labels */
  labels?: string[];
}

// ============================================================================
// Consciousness Transfer Types
// ============================================================================

/**
 * Compression level for consciousness data
 */
export type CompressionLevel = 'lossless' | 'high-fidelity' | 'balanced';

/**
 * Target substrate for consciousness
 */
export type TargetSubstrate = 'biological' | 'synthetic' | 'quantum-simulation';

/**
 * Transfer method
 */
export type TransferMethod = 'direct' | 'backup-restore' | 'gradual';

/**
 * Consciousness digitization request
 */
export interface DigitizationRequest {
  /** Neural mapping to digitize */
  neuralMapping: NeuralMappingResult;

  /** Compression level */
  compressionLevel: CompressionLevel;

  /** Encryption key for security */
  encryptionKey: CryptoKey | string;

  /** Create backup during digitization */
  includeBackup: boolean;

  /** Preserve quantum coherence */
  preserveCoherence?: boolean;
}

/**
 * Complete consciousness data package
 */
export interface ConsciousnessData {
  /** Unique consciousness identifier */
  id: string;

  /** Creation timestamp */
  timestamp: Date;

  /** Subject identifier */
  subjectId: string;

  /** Quantum state vector */
  stateVector: QuantumState;

  /** Memory store */
  memories: MemoryStore;

  /** Identity signature */
  identity: IdentitySignature;

  /** Metadata */
  metadata: ConsciousnessMetadata;

  /** Data integrity checksum */
  checksum: string;

  /** Encryption status */
  encrypted: boolean;
}

/**
 * Identity signature for verification
 */
export interface IdentitySignature {
  /** Unique identity hash */
  identityHash: string;

  /** Biometric signatures */
  biometrics?: {
    neuralSignature: string;
    behavioralSignature: string;
    cognitiveSignature: string;
  };

  /** Blockchain identity reference */
  blockchainRef?: string;

  /** Cryptographic signature */
  cryptoSignature: string;
}

/**
 * Consciousness metadata
 */
export interface ConsciousnessMetadata {
  /** WIA standard version */
  version: string;

  /** Scan resolution */
  resolution: ScanResolution;

  /** Transfer fidelity */
  fidelity: number;

  /** Data size in bytes */
  dataSize: number;

  /** Compression ratio */
  compressionRatio: number;

  /** Entropy in bits */
  entropy: number;

  /** Original timeline */
  originTimeline?: string;

  /** Provenance chain */
  provenance: ProvenanceEntry[];
}

/**
 * Provenance entry for tracking consciousness history
 */
export interface ProvenanceEntry {
  /** Operation type */
  operation: 'scan' | 'digitize' | 'transfer' | 'backup' | 'restore' | 'modify';

  /** Timestamp */
  timestamp: Date;

  /** Operator/system */
  operator: string;

  /** Operation details */
  details: Record<string, unknown>;

  /** Signature */
  signature: string;
}

// ============================================================================
// Identity Continuity Types
// ============================================================================

/**
 * Identity metrics for continuity verification
 */
export interface IdentityMetrics {
  /** Structural identity (neural pattern similarity) */
  structural: number;

  /** Functional identity (behavioral similarity) */
  functional: number;

  /** Subjective identity (self-recognition) */
  subjective: number;

  /** Memory identity (recall accuracy) */
  memory: number;

  /** Composite identity score */
  composite: number;

  /** Pass/fail status */
  passed: boolean;
}

/**
 * Identity continuity test
 */
export interface IdentityContinuityTest {
  /** Source consciousness state */
  sourceState: ConsciousnessData;

  /** Target consciousness state */
  targetState: ConsciousnessData;

  /** Minimum required fidelity */
  minFidelity: number;

  /** Required identity score */
  minIdentityScore: number;

  /** Include subjective interview */
  includeInterview?: boolean;
}

/**
 * Identity continuity result
 */
export interface IdentityContinuityResult {
  /** Test passed */
  passed: boolean;

  /** Transfer fidelity */
  fidelity: number;

  /** Identity metrics */
  metrics: IdentityMetrics;

  /** Subjective interview results */
  interview?: SubjectiveInterviewResult;

  /** Detected issues */
  issues: string[];

  /** Warnings */
  warnings: string[];

  /** Timestamp */
  timestamp: Date;
}

/**
 * Subjective continuity interview result
 */
export interface SubjectiveInterviewResult {
  /** Responses to standard questions */
  responses: {
    question: string;
    answer: string;
    confidence: number; // 0-100%
  }[];

  /** Overall confidence in continuity */
  overallConfidence: number; // 0-100%

  /** Interviewer assessment */
  assessment: string;

  /** Pass status */
  passed: boolean;
}

// ============================================================================
// Transfer Operation Types
// ============================================================================

/**
 * Consciousness transfer request
 */
export interface TransferRequest {
  /** Source consciousness */
  sourceConsciousness: ConsciousnessData;

  /** Target temporal location */
  targetLocation: TemporalCoordinate;

  /** Target substrate type */
  targetSubstrate: TargetSubstrate;

  /** Transfer method */
  transferMethod: TransferMethod;

  /** Minimum acceptable fidelity */
  minFidelity: number;

  /** Require identity continuity */
  requireContinuity: boolean;

  /** Energy budget in joules */
  energyBudget?: number;

  /** Create backup before transfer */
  createBackup?: boolean;
}

/**
 * Consciousness transfer result
 */
export interface TransferResult {
  /** Unique transfer identifier */
  transferId: string;

  /** Success status */
  success: boolean;

  /** Transfer fidelity achieved */
  fidelity: number;

  /** Identity continuity metrics */
  identityContinuity: IdentityMetrics;

  /** Transfer duration in ms */
  duration: number;

  /** Energy consumed in joules */
  energyUsed: number;

  /** Errors encountered */
  errors: string[];

  /** Warnings */
  warnings: string[];

  /** Timestamp */
  timestamp: Date;

  /** Backup reference (if created) */
  backupId?: string;
}

// ============================================================================
// Temporal Synchronization Types
// ============================================================================

/**
 * Temporal synchronization state
 */
export interface TemporalSyncState {
  /** Internal consciousness time */
  internalTime: Date;

  /** External reference time */
  externalTime: Date;

  /** Time offset in seconds */
  timeOffset: number;

  /** Synchronization status */
  synchronized: boolean;

  /** Desynchronization level (0-1) */
  desynchronization: number;

  /** Last sync timestamp */
  lastSync: Date;
}

/**
 * Timeline information
 */
export interface Timeline {
  /** Unique timeline identifier */
  id: string;

  /** Timeline name/description */
  name?: string;

  /** Branch point from parent */
  branchPoint?: Date;

  /** Parent timeline */
  parentTimeline?: string;

  /** Probability (for MWI) */
  probability?: number;

  /** Active consciousness instances */
  activeInstances: string[];
}

/**
 * Multi-timeline consciousness state
 */
export interface MultiTimelineState {
  /** Primary timeline */
  primaryTimeline: Timeline;

  /** Secondary timelines */
  secondaryTimelines: Timeline[];

  /** Cross-timeline coherence (0-1) */
  coherence: number;

  /** Amplitude distribution across timelines */
  amplitudes: Map<string, number>;

  /** Integration status */
  integrated: boolean;
}

// ============================================================================
// Backup and Restore Types
// ============================================================================

/**
 * Backup configuration
 */
export interface BackupConfig {
  /** Backup frequency */
  frequency: 'realtime' | 'hourly' | 'daily' | 'weekly' | 'monthly' | 'manual';

  /** Compression level */
  compression: CompressionLevel;

  /** Encryption enabled */
  encrypted: boolean;

  /** Redundancy factor */
  redundancy: number;

  /** Storage locations */
  storageNodes: string[];

  /** Retention policy in days */
  retentionDays: number;
}

/**
 * Consciousness backup
 */
export interface ConsciousnessBackup {
  /** Backup identifier */
  backupId: string;

  /** Source consciousness */
  consciousnessId: string;

  /** Subject identifier */
  subjectId: string;

  /** Backup timestamp */
  timestamp: Date;

  /** Consciousness data */
  data: ConsciousnessData;

  /** Backup integrity hash */
  integrityHash: string;

  /** Verification status */
  verified: boolean;

  /** Storage locations */
  storageLocations: string[];

  /** Backup size in bytes */
  size: number;
}

/**
 * Restore request
 */
export interface RestoreRequest {
  /** Backup to restore */
  backupId: string;

  /** Target substrate */
  targetSubstrate: TargetSubstrate;

  /** Target location */
  targetLocation: TemporalCoordinate;

  /** Verify integrity before restore */
  verifyIntegrity: boolean;

  /** Authentication credentials */
  authentication: AuthenticationCredentials;
}

/**
 * Restore result
 */
export interface RestoreResult {
  /** Restore operation ID */
  restoreId: string;

  /** Success status */
  success: boolean;

  /** Restored consciousness ID */
  consciousnessId: string;

  /** Restoration fidelity */
  fidelity: number;

  /** Identity verification */
  identityVerified: boolean;

  /** Restoration duration in ms */
  duration: number;

  /** Errors */
  errors: string[];

  /** Warnings */
  warnings: string[];
}

/**
 * Authentication credentials
 */
export interface AuthenticationCredentials {
  /** Biometric data */
  biometric?: string;

  /** Multi-factor auth tokens */
  mfaTokens: string[];

  /** Legal authorization */
  legalAuthorization?: string;

  /** Cryptographic signature */
  signature: string;
}

// ============================================================================
// Ethical and Safety Types
// ============================================================================

/**
 * Informed consent record
 */
export interface InformedConsent {
  /** Consent ID */
  consentId: string;

  /** Subject ID */
  subjectId: string;

  /** Consent granted */
  granted: boolean;

  /** Consent timestamp */
  timestamp: Date;

  /** Consent document hash */
  documentHash: string;

  /** Witness signatures */
  witnesses: string[];

  /** Video recording reference */
  videoReference?: string;

  /** Blockchain ledger entry */
  blockchainEntry?: string;

  /** Cooling-off period end */
  coolingOffEnd: Date;

  /** Can be revoked */
  revocable: boolean;
}

/**
 * Ethics check result
 */
export interface EthicsCheckResult {
  /** Check passed */
  passed: boolean;

  /** Consent verified */
  consentVerified: boolean;

  /** Rights preserved */
  rightsPreserved: boolean;

  /** Privacy protected */
  privacyProtected: boolean;

  /** No unauthorized duplication */
  noDuplication: boolean;

  /** Violations detected */
  violations: string[];

  /** Recommendations */
  recommendations: string[];

  /** Reviewer */
  reviewer: string;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Safety check
 */
export interface SafetyCheck {
  /** Check name */
  name: string;

  /** Check status */
  status: 'pass' | 'fail' | 'warning' | 'info';

  /** Description */
  description: string;

  /** Measured value */
  value?: number;

  /** Expected value */
  expected?: number;

  /** Threshold */
  threshold?: number;

  /** Corrective action */
  correctiveAction?: string;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for consciousness transfer
 */
export const CONSCIOUSNESS_CONSTANTS = {
  /** Planck constant in J·s */
  PLANCK_CONSTANT: 6.62607015e-34,

  /** Reduced Planck constant in J·s */
  REDUCED_PLANCK: 1.054571817e-34,

  /** Boltzmann constant in J/K */
  BOLTZMANN_CONSTANT: 1.380649e-23,

  /** Speed of light in m/s */
  SPEED_OF_LIGHT: 299792458,

  /** Average neuron count */
  NEURON_COUNT: 86e9,

  /** Average synapse count */
  SYNAPSE_COUNT: 1e15,

  /** Neural processing frequency in Hz */
  NEURAL_FREQUENCY: 200,

  /** Consciousness bandwidth in bits */
  CONSCIOUSNESS_BANDWIDTH: 2.5e25,

  /** Minimum transfer fidelity */
  MIN_FIDELITY: 0.9999,

  /** Minimum identity continuity */
  MIN_IDENTITY: 0.99,

  /** Maximum subjective time freeze in ms */
  MAX_TIME_FREEZE: 100,

  /** Compression ratio (lossless) */
  COMPRESSION_RATIO: 1000,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-TIME-013 error codes
 */
export enum ConsciousnessErrorCode {
  INSUFFICIENT_RESOLUTION = 'C001',
  FIDELITY_BELOW_THRESHOLD = 'C002',
  IDENTITY_CONTINUITY_FAILURE = 'C003',
  MEMORY_CORRUPTION = 'C004',
  UNAUTHORIZED_DUPLICATION = 'C005',
  TEMPORAL_DESYNC = 'C006',
  QUANTUM_DECOHERENCE = 'C007',
  ETHICS_VIOLATION = 'C008',
  INVALID_PARAMETERS = 'C009',
  CONSENT_NOT_GRANTED = 'C010',
}

/**
 * Consciousness transfer error
 */
export class ConsciousnessTransferError extends Error {
  constructor(
    public code: ConsciousnessErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ConsciousnessTransferError';
  }
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
// Export All
// ============================================================================

export type {
  // Core types
  Vector3,
  ComplexNumber,
  QuantumState,
  EntanglementMap,
  TemporalCoordinate,

  // Neural types
  NeuralMappingRequest,
  NeuralMappingResult,
  NeuralGraph,
  Neuron,
  Synapse,

  // Memory types
  MemoryStore,
  EpisodicMemory,
  SemanticMemory,
  ConceptNode,
  ConceptAssociation,
  ProceduralMemory,
  StateActionPair,
  SensoryData,
  EmotionalTag,

  // Consciousness types
  DigitizationRequest,
  ConsciousnessData,
  IdentitySignature,
  ConsciousnessMetadata,
  ProvenanceEntry,

  // Identity types
  IdentityMetrics,
  IdentityContinuityTest,
  IdentityContinuityResult,
  SubjectiveInterviewResult,

  // Transfer types
  TransferRequest,
  TransferResult,

  // Temporal types
  TemporalSyncState,
  Timeline,
  MultiTimelineState,

  // Backup types
  BackupConfig,
  ConsciousnessBackup,
  RestoreRequest,
  RestoreResult,
  AuthenticationCredentials,

  // Ethics types
  InformedConsent,
  EthicsCheckResult,
  SafetyCheck,
};

export { CONSCIOUSNESS_CONSTANTS, ConsciousnessErrorCode, ConsciousnessTransferError };
