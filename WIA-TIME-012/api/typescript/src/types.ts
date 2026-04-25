/**
 * WIA-TIME-012: Matter Transmission - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Matter Types
// ============================================================================

/**
 * Disassembly resolution levels
 */
export enum DisassemblyResolution {
  MOLECULAR = 'molecular',
  ATOMIC = 'atomic',
  SUBATOMIC = 'subatomic',
  QUANTUM = 'quantum',
  PLANCK = 'planck_scale'
}

/**
 * Matter types
 */
export enum MatterType {
  SOLID = 'solid',
  LIQUID = 'liquid',
  GAS = 'gas',
  PLASMA = 'plasma',
  COMPOSITE = 'composite',
  BIOLOGICAL = 'biological',
  QUANTUM = 'quantum'
}

/**
 * Transmission priority levels
 */
export enum TransmissionPriority {
  LOW = 'low',
  NORMAL = 'normal',
  HIGH = 'high',
  CRITICAL = 'critical',
  EMERGENCY = 'emergency'
}

/**
 * Transmission status
 */
export enum TransmissionStatus {
  PENDING = 'pending',
  ANALYZING = 'analyzing',
  DISASSEMBLING = 'disassembling',
  ENCODING = 'encoding',
  TRANSMITTING = 'transmitting',
  REASSEMBLING = 'reassembling',
  VERIFYING = 'verifying',
  COMPLETED = 'completed',
  FAILED = 'failed',
  ABORTED = 'aborted'
}

/**
 * Error correction levels
 */
export enum ErrorCorrectionLevel {
  NONE = 'none',
  MINIMAL = 'minimal',
  STANDARD = 'standard',
  ENHANCED = 'enhanced',
  MAXIMUM = 'maximum'
}

/**
 * Verification levels
 */
export enum VerificationLevel {
  BASIC = 'basic',
  MOLECULAR = 'molecular',
  ATOMIC = 'atomic',
  QUANTUM = 'quantum',
  COMPLETE = 'complete'
}

// ============================================================================
// Object and Entity Types
// ============================================================================

/**
 * Object identifier
 */
export interface ObjectIdentifier {
  /** Unique object ID */
  id: string;

  /** Object name */
  name?: string;

  /** Object type */
  type?: MatterType;

  /** Mass in kilograms */
  mass?: number;

  /** Metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Chemical composition
 */
export interface ChemicalComposition {
  /** Elements present */
  elements: {
    /** Atomic number */
    atomicNumber: number;
    /** Element symbol */
    symbol: string;
    /** Count of atoms */
    count: number;
    /** Percentage by mass */
    massPercent: number;
  }[];

  /** Molecular formulas */
  molecules?: {
    /** Chemical formula */
    formula: string;
    /** Count of molecules */
    count: number;
  }[];

  /** Total atom count */
  totalAtoms: number;

  /** Total mass in kg */
  totalMass: number;
}

/**
 * Spatial coordinates [x, y, z]
 */
export type SpatialCoordinates = [number, number, number];

/**
 * Atom data
 */
export interface Atom {
  /** Unique atom ID */
  id: string;

  /** Atomic number (protons) */
  element: number;

  /** Mass number (protons + neutrons) */
  isotope: number;

  /** Position in picometers [x, y, z] */
  position: SpatialCoordinates;

  /** Velocity in m/s [vx, vy, vz] */
  velocity: SpatialCoordinates;

  /** Nuclear spin */
  spin: number;

  /** Net charge */
  charge: number;

  /** Electron configuration */
  electronConfig: string;

  /** Bonding electrons count */
  bondingElectrons: number;
}

/**
 * Molecular bond
 */
export interface MolecularBond {
  /** First atom ID */
  atomA: string;

  /** Second atom ID */
  atomB: string;

  /** Bond order (1=single, 2=double, 3=triple) */
  bondOrder: number;

  /** Bond length in picometers */
  bondLength: number;

  /** Bond energy in joules */
  bondEnergy: number;

  /** Bond type */
  bondType: 'covalent' | 'ionic' | 'metallic' | 'hydrogen' | 'van_der_waals';
}

// ============================================================================
// Analysis Types
// ============================================================================

/**
 * Analysis options
 */
export interface AnalysisOptions {
  /** Scan resolution */
  resolution?: DisassemblyResolution;

  /** Include quantum state analysis */
  includeQuantumState?: boolean;

  /** Include chemical analysis */
  includeChemistry?: boolean;

  /** Include structural analysis */
  includeStructure?: boolean;

  /** Timeout in seconds */
  timeout?: number;
}

/**
 * Matter analysis request
 */
export interface AnalysisRequest {
  /** Object to analyze */
  object: ObjectIdentifier;

  /** Analysis depth */
  depth: DisassemblyResolution;

  /** Include quantum state */
  includeQuantumState: boolean;

  /** Additional options */
  options?: AnalysisOptions;
}

/**
 * Matter analysis result
 */
export interface AnalysisResult {
  /** Object ID */
  objectId: string;

  /** Total atom count */
  atomCount: number;

  /** Total mass in kg */
  mass: number;

  /** Chemical composition */
  composition: ChemicalComposition;

  /** Complexity score (0-1) */
  complexity: number;

  /** Quantum complexity score (0-1) */
  quantumComplexity: number;

  /** Is object transmissible? */
  transmissible: boolean;

  /** Reason if not transmissible */
  reason?: string;

  /** Estimated energy requirement in joules */
  estimatedEnergy: number;

  /** Estimated transmission duration in seconds */
  estimatedDuration: number;

  /** Quantum state data */
  quantumState?: QuantumState;

  /** Structural data */
  structure?: StructuralData;

  /** Analysis timestamp */
  analyzedAt: Date;
}

/**
 * Structural data
 */
export interface StructuralData {
  /** 3D dimensions in meters [x, y, z] */
  dimensions: SpatialCoordinates;

  /** Volume in cubic meters */
  volume: number;

  /** Density in kg/m³ */
  density: number;

  /** Center of mass [x, y, z] */
  centerOfMass: SpatialCoordinates;

  /** Moment of inertia tensor */
  momentOfInertia: number[][];
}

// ============================================================================
// Quantum State Types
// ============================================================================

/**
 * Quantum state
 */
export interface QuantumState {
  /** Wavefunction representation */
  wavefunction?: WavefunctionData;

  /** Density matrix */
  densityMatrix?: number[][];

  /** Entanglement map */
  entanglementMap: EntanglementPair[];

  /** Superposition states */
  superpositionStates: SuperpositionState[];

  /** Coherence time in seconds */
  coherenceTime: number;

  /** Decoherence rate in 1/seconds */
  decoherenceRate: number;

  /** Quantum fidelity (0-1) */
  fidelity?: number;
}

/**
 * Wavefunction data
 */
export interface WavefunctionData {
  /** Representation basis */
  basis: 'position' | 'momentum' | 'energy';

  /** Number of dimensions */
  dimensions: number;

  /** Wavefunction values at grid points */
  values: {
    /** Position [x, y, z] */
    position: SpatialCoordinates;
    /** Complex amplitude */
    amplitude: {
      real: number;
      imag: number;
      magnitude: number;
      phase: number;
    };
  }[];

  /** Normalization value */
  normalization: number;
}

/**
 * Entanglement pair
 */
export interface EntanglementPair {
  /** First particle ID */
  particleA: string;

  /** Second particle ID */
  particleB: string;

  /** Entanglement strength (0-1) */
  strength: number;

  /** Bell state */
  bellState: 'phi_plus' | 'phi_minus' | 'psi_plus' | 'psi_minus';

  /** Concurrence (entanglement measure) */
  concurrence: number;
}

/**
 * Superposition state
 */
export interface SuperpositionState {
  /** Basis states */
  basisStates: string[];

  /** Complex amplitudes for each basis state */
  amplitudes: {
    real: number;
    imag: number;
  }[];

  /** Phases for each basis state */
  phases: number[];
}

/**
 * Quantum state preservation configuration
 */
export interface QuantumStatePreservation {
  /** Preserve quantum superposition states */
  superposition: boolean;

  /** Preserve quantum entanglement */
  entanglement: boolean;

  /** Quantum coherence time in seconds */
  coherenceTime: number;

  /** Decoherence protection level */
  decoherenceProtection: 'none' | 'minimal' | 'standard' | 'maximum';

  /** Quantum error correction */
  errorCorrection: QuantumErrorCorrection;
}

/**
 * Quantum error correction
 */
export interface QuantumErrorCorrection {
  /** Error correction code */
  code: 'shor' | 'steane' | 'surface' | 'color' | 'none';

  /** Number of physical qubits per logical qubit */
  redundancy: number;

  /** Error detection threshold */
  detectionThreshold: number;

  /** Automatic correction enabled */
  autoCorrect: boolean;
}

// ============================================================================
// Disassembly Types
// ============================================================================

/**
 * Disassembly request
 */
export interface DisassemblyRequest {
  /** Object to disassemble */
  object: string | ObjectIdentifier;

  /** Disassembly method */
  method: 'quantum_deconstruction' | 'molecular_separation' | 'atomic_ionization';

  /** Resolution level */
  resolution: DisassemblyResolution;

  /** Preserve quantum state */
  preserveState: boolean;

  /** Create backup */
  createBackup?: boolean;

  /** Backup redundancy (1-10) */
  backupRedundancy?: number;
}

/**
 * Disassembly result
 */
export interface DisassemblyResult {
  /** Object ID */
  objectId: string;

  /** Encoded matter data */
  encoding: EncodedMatter;

  /** Total atoms disassembled */
  atomCount: number;

  /** Encoding size in bytes */
  encodingSize: number;

  /** Quantum states captured */
  quantumStates: QuantumState[];

  /** Molecular blueprint */
  blueprint: MolecularBlueprint;

  /** Disassembly timestamp */
  disassembledAt: Date;

  /** Verification result */
  verification: DisassemblyVerification;
}

/**
 * Molecular blueprint
 */
export interface MolecularBlueprint {
  /** Version */
  version: string;

  /** Atoms */
  atoms: Atom[];

  /** Bonds */
  bonds: MolecularBond[];

  /** Total mass in kg */
  totalMass: number;

  /** Center of mass */
  centerOfMass: SpatialCoordinates;

  /** Molecular signature (hash) */
  signature: string;

  /** Checksum */
  checksum: string;
}

/**
 * Disassembly verification
 */
export interface DisassemblyVerification {
  /** Mass conserved */
  massConserved: boolean;

  /** Atom count matches */
  atomCountMatch: boolean;

  /** Energy conserved */
  energyConserved: boolean;

  /** Quantum state captured */
  quantumStateCaptured: boolean;

  /** Overall completeness (0-1) */
  completeness: number;

  /** Mass error */
  massError: number;

  /** Energy error */
  energyError: number;
}

// ============================================================================
// Encoding Types
// ============================================================================

/**
 * Encoded matter
 */
export interface EncodedMatter {
  /** Encoding version */
  version: string;

  /** Encoding type */
  encodingType: DisassemblyResolution;

  /** Original object ID */
  objectId: string;

  /** Encoding timestamp */
  timestamp: Date;

  /** Metadata */
  metadata: {
    totalAtoms: number;
    totalMass: number;
    centerOfMass: SpatialCoordinates;
    complexity: number;
  };

  /** Atomic data */
  atomicData: {
    atoms: Atom[];
    bonds: MolecularBond[];
    electronConfigurations: string[];
  };

  /** Quantum state */
  quantumState?: QuantumState;

  /** Molecular signature */
  molecularSignature: string;

  /** Checksum */
  checksum: string;

  /** Compression info */
  compression: {
    algorithm: string;
    ratio: number;
    originalSize: number;
    compressedSize: number;
  };

  /** Error correction info */
  errorCorrection: {
    method: string;
    redundancy: number;
    capability: number;
  };
}

// ============================================================================
// Transmission Types
// ============================================================================

/**
 * Destination coordinates
 */
export interface Destination {
  /** Target time */
  time: Date;

  /** Spatial location [latitude, longitude, altitude] */
  location: SpatialCoordinates;

  /** Spatial uncertainty in meters */
  uncertainty: number;

  /** Temporal uncertainty in seconds */
  temporalUncertainty?: number;

  /** Reference frame */
  referenceFrame?: 'WGS84' | 'ECEF' | 'galactic';
}

/**
 * Transmission request
 */
export interface TransmissionRequest {
  /** Encoded matter to transmit */
  encodedMatter: EncodedMatter;

  /** Destination */
  destination: Destination;

  /** Priority */
  priority: TransmissionPriority;

  /** Error correction level */
  errorCorrection: ErrorCorrectionLevel;

  /** Quantum state preservation */
  quantumStatePreservation?: QuantumStatePreservation;

  /** Planetary motion compensation */
  planetaryCompensation?: boolean;

  /** Callback URL for status updates */
  callbackUrl?: string;
}

/**
 * Transmission result
 */
export interface TransmissionResult {
  /** Transmission ID */
  id: string;

  /** Status */
  status: TransmissionStatus;

  /** Estimated arrival time */
  estimatedArrival: Date;

  /** Energy consumption in joules */
  energyConsumption: number;

  /** Progress (0-100) */
  progress: number;

  /** Atoms transmitted */
  atomsTransmitted: number;

  /** Total atoms */
  totalAtoms: number;

  /** Data integrity (0-100) */
  integrity: number;

  /** Error count */
  errorCount: number;

  /** Transmission started at */
  startedAt: Date;

  /** Transmission completed at */
  completedAt?: Date;
}

/**
 * Transmission progress event
 */
export interface TransmissionProgress {
  /** Transmission ID */
  transmissionId: string;

  /** Percentage complete (0-100) */
  percentage: number;

  /** Atoms transmitted */
  atomsTransmitted: number;

  /** Total atoms */
  totalAtoms: number;

  /** Current phase */
  phase: TransmissionStatus;

  /** Data integrity (0-100) */
  integrity: number;

  /** Estimated time remaining in seconds */
  estimatedTimeRemaining: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Transmission error
 */
export interface TransmissionError {
  /** Error type */
  type: 'data_corruption' | 'quantum_decoherence' | 'energy_failure' | 'destination_hazard' | 'unknown';

  /** Severity */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Error message */
  message: string;

  /** Error code */
  code: string;

  /** Affected atoms */
  affectedAtoms?: number;

  /** Correction applied */
  correctionApplied: boolean;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Reassembly Types
// ============================================================================

/**
 * Reassembly request
 */
export interface ReassemblyRequest {
  /** Transmission ID */
  transmissionId: string;

  /** Verification level */
  verificationLevel: VerificationLevel;

  /** Reconstruct quantum state */
  quantumStateReconstruction: boolean;

  /** Target location (if different from transmission) */
  targetLocation?: SpatialCoordinates;

  /** Timeout in seconds */
  timeout?: number;
}

/**
 * Reassembled object
 */
export interface ReassembledObject {
  /** Object ID */
  id: string;

  /** Original object ID */
  originalId: string;

  /** Mass in kg */
  mass: number;

  /** Atom count */
  atomCount: number;

  /** Position */
  position: SpatialCoordinates;

  /** Quantum state */
  quantumState?: QuantumState;

  /** Reassembly timestamp */
  reassembledAt: Date;

  /** Metadata */
  metadata: Record<string, unknown>;
}

/**
 * Reassembly result
 */
export interface ReassemblyResult {
  /** Success status */
  success: boolean;

  /** Reassembled object */
  object?: ReassembledObject;

  /** Atomic accuracy (0-100) */
  accuracy: number;

  /** Quantum fidelity (0-100) */
  quantumFidelity: number;

  /** Molecular fidelity (0-100) */
  molecularFidelity: number;

  /** Errors */
  errors: string[];

  /** Verification result */
  verification: VerificationResult;

  /** Reassembly log */
  log: ReassemblyLogEntry[];
}

/**
 * Reassembly log entry
 */
export interface ReassemblyLogEntry {
  /** Step name */
  step: string;

  /** Success status */
  success: boolean;

  /** Message */
  message?: string;

  /** Timestamp */
  timestamp: Date;

  /** Metrics */
  metrics?: Record<string, number>;
}

// ============================================================================
// Verification Types
// ============================================================================

/**
 * Verification request
 */
export interface VerificationRequest {
  /** Original disassembly result */
  original: DisassemblyResult;

  /** Reassembled object */
  reassembled: ReassembledObject;

  /** Error tolerance (0-1) */
  tolerance: number;

  /** Verification level */
  level: VerificationLevel;
}

/**
 * Verification result
 */
export interface VerificationResult {
  /** Overall verification status */
  verified: boolean;

  /** Atomic verification */
  atomic: AtomicVerification;

  /** Molecular verification */
  molecular: MolecularVerification;

  /** Quantum verification */
  quantum: QuantumVerification;

  /** Functional verification */
  functional?: FunctionalVerification;

  /** Overall accuracy (0-100) */
  accuracy: number;

  /** Errors found */
  errors: string[];

  /** Warnings */
  warnings: string[];
}

/**
 * Atomic verification
 */
export interface AtomicVerification {
  /** Atomic accuracy (0-100) */
  accuracy: number;

  /** Mass conserved */
  massConserved: boolean;

  /** Mass error */
  massError: number;

  /** Composition match */
  compositionMatch: boolean;

  /** Atom count match */
  atomCountMatch: boolean;
}

/**
 * Molecular verification
 */
export interface MolecularVerification {
  /** Molecular fidelity (0-100) */
  fidelity: number;

  /** Structure match */
  structureMatch: boolean;

  /** Bonds intact */
  bondsIntact: boolean;

  /** Bond accuracy (0-100) */
  bondAccuracy: number;
}

/**
 * Quantum verification
 */
export interface QuantumVerification {
  /** Quantum fidelity (0-100) */
  fidelity: number;

  /** Entanglement preserved */
  entanglementPreserved: boolean;

  /** Superposition maintained */
  superpositionMaintained: boolean;

  /** Coherence time ratio */
  coherenceTimeRatio: number;
}

/**
 * Functional verification
 */
export interface FunctionalVerification {
  /** Operational status */
  operational: boolean;

  /** Performance comparison */
  performance: number;

  /** Functional integrity (0-100) */
  integrity: number;

  /** Test results */
  testResults: Record<string, boolean>;
}

/**
 * Integrity check
 */
export interface IntegrityCheck {
  /** Atomic composition match */
  atomicMatch: boolean;
  accuracy: number;

  /** Molecular structure match */
  molecularMatch: boolean;
  structuralFidelity: number;

  /** Quantum state match */
  quantumMatch: boolean;
  quantumFidelity: number;

  /** Energy conserved */
  energyConserved: boolean;
  energyDelta: number;

  /** Overall verification */
  verified: boolean;
  errors: string[];
}

// ============================================================================
// Backup Types
// ============================================================================

/**
 * Backup request
 */
export interface BackupRequest {
  /** Object to backup */
  object: string | ObjectIdentifier;

  /** Storage location */
  storage: 'local' | 'quantum_vault' | 'distributed';

  /** Redundancy level (1-10) */
  redundancy: number;

  /** Encryption enabled */
  encrypted?: boolean;

  /** Backup label */
  label?: string;
}

/**
 * Backup result
 */
export interface BackupResult {
  /** Backup ID */
  id: string;

  /** Object ID */
  objectId: string;

  /** Storage location */
  storage: string;

  /** Backup size in bytes */
  size: number;

  /** Redundancy level */
  redundancy: number;

  /** Checksum */
  checksum: string;

  /** Created timestamp */
  createdAt: Date;

  /** Expiration timestamp */
  expiresAt?: Date;
}

// ============================================================================
// Living Matter Types
// ============================================================================

/**
 * Living matter subject
 */
export interface LivingMatterSubject {
  /** Subject ID */
  id: string;

  /** Subject type */
  type: 'human' | 'animal' | 'plant' | 'microorganism';

  /** Age */
  age?: number;

  /** Mass in kg */
  mass: number;

  /** Medical history */
  medicalHistory?: string;

  /** Consent given */
  consent: boolean;

  /** Special requirements */
  specialRequirements?: string[];
}

/**
 * Living matter protocol
 */
export interface LivingMatterProtocol {
  /** Neural state backup */
  neuralStateBackup: boolean;

  /** Consciousness preservation level */
  consciousnessPreservation: 'minimal' | 'standard' | 'maximum';

  /** Memory integrity */
  memoryIntegrity: 'partial' | 'complete';

  /** Vital signs monitoring */
  vitalSignsMonitoring: boolean;

  /** Metabolic suspension */
  metabolicSuspension: boolean;

  /** Cellular preservation method */
  cellularPreservation: 'standard' | 'cryogenic' | 'quantum_stasis';

  /** Medical team on standby */
  medicalTeamStandby: boolean;

  /** Emergency abort enabled */
  emergencyAbort: boolean;

  /** Create backup clone */
  backupClone: boolean;

  /** Post-reassembly medical examination */
  medicalExamination: 'basic' | 'comprehensive';

  /** Neurological assessment */
  neurologicalAssessment: boolean;

  /** Psychological support */
  psychologicalSupport: boolean;
}

/**
 * Neural state
 */
export interface NeuralState {
  /** Connectome (neural connection map) */
  connectome: NeuralConnectome;

  /** Synaptic weights */
  synapticWeights: SynapticWeight[];

  /** Neurotransmitter levels */
  neurotransmitterLevels: Record<string, number>;

  /** Electrical patterns */
  electricalPatterns: ElectricalPatterns;

  /** Quantum effects */
  quantumEffects?: QuantumBrainState;
}

/**
 * Neural connectome
 */
export interface NeuralConnectome {
  /** Number of neurons */
  neuronCount: number;

  /** Connections */
  connections: {
    preNeuron: string;
    postNeuron: string;
    strength: number;
  }[];

  /** Brain regions */
  regions: {
    name: string;
    neuronCount: number;
    connections: number;
  }[];
}

/**
 * Synaptic weight
 */
export interface SynapticWeight {
  /** Pre-synaptic neuron ID */
  preNeuron: string;

  /** Post-synaptic neuron ID */
  postNeuron: string;

  /** Synaptic weight */
  weight: number;

  /** Neurotransmitter type */
  neurotransmitter: string;

  /** Plasticity */
  plasticity: number;
}

/**
 * Electrical patterns
 */
export interface ElectricalPatterns {
  /** EEG data */
  eeg: number[][];

  /** Action potentials */
  actionPotentials: ActionPotential[];

  /** Neural oscillations */
  oscillations: {
    frequency: number;
    amplitude: number;
    phase: number;
  }[];
}

/**
 * Action potential
 */
export interface ActionPotential {
  /** Neuron ID */
  neuronId: string;

  /** Timestamp */
  timestamp: number;

  /** Amplitude */
  amplitude: number;

  /** Duration */
  duration: number;
}

/**
 * Quantum brain state
 */
export interface QuantumBrainState {
  /** Quantum coherence in microtubules */
  microtubuleCoherence: number;

  /** Quantum entanglement patterns */
  entanglementPatterns: EntanglementPair[];

  /** Orchestrated objective reduction events */
  orchestratedReduction: number;
}

// ============================================================================
// Statistics and Metrics
// ============================================================================

/**
 * Transmission statistics
 */
export interface TransmissionStats {
  /** Total transmissions */
  totalTransmissions: number;

  /** Successful transmissions */
  successfulTransmissions: number;

  /** Failed transmissions */
  failedTransmissions: number;

  /** Success rate (0-100) */
  successRate: number;

  /** Average accuracy (0-100) */
  avgAccuracy: number;

  /** Average quantum fidelity (0-100) */
  avgQuantumFidelity: number;

  /** Living matter statistics */
  livingMatter: {
    count: number;
    successRate: number;
    avgNeuralFidelity: number;
  };

  /** Total energy consumed (joules) */
  totalEnergy: number;

  /** Average transmission time (seconds) */
  avgTransmissionTime: number;

  /** By matter type */
  byMatterType: Record<MatterType, {
    count: number;
    successRate: number;
  }>;
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * SDK configuration
 */
export interface MatterTransmissionConfig {
  /** API key */
  apiKey: string;

  /** Environment */
  environment: 'development' | 'staging' | 'production';

  /** Base URL */
  baseUrl?: string;

  /** Safety level */
  safety: 'minimal' | 'standard' | 'maximum';

  /** Default error correction */
  defaultErrorCorrection?: ErrorCorrectionLevel;

  /** Default quantum preservation */
  defaultQuantumPreservation?: Partial<QuantumStatePreservation>;

  /** Timeout in seconds */
  timeout?: number;

  /** Retry attempts */
  retryAttempts?: number;

  /** Enable logging */
  logging?: boolean;

  /** Log level */
  logLevel?: 'debug' | 'info' | 'warn' | 'error';
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Event handler type
 */
export type EventHandler<T> = (event: T) => void | Promise<void>;

/**
 * Event emitter interface
 */
export interface EventEmitter<T> {
  on(event: string, handler: EventHandler<T>): void;
  off(event: string, handler: EventHandler<T>): void;
  emit(event: string, data: T): void;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Matter transmission error
 */
export class MatterTransmissionError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'MatterTransmissionError';
  }
}

/**
 * Verification error
 */
export class VerificationError extends MatterTransmissionError {
  constructor(message: string, details?: Record<string, unknown>) {
    super(message, 'VERIFICATION_ERROR', details);
    this.name = 'VerificationError';
  }
}

/**
 * Ethics violation error
 */
export class EthicsViolation extends MatterTransmissionError {
  constructor(message: string, details?: Record<string, unknown>) {
    super(message, 'ETHICS_VIOLATION', details);
    this.name = 'EthicsViolation';
  }
}

// ============================================================================
// Export All
// ============================================================================

export default {
  DisassemblyResolution,
  MatterType,
  TransmissionPriority,
  TransmissionStatus,
  ErrorCorrectionLevel,
  VerificationLevel
};
