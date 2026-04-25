/**
 * WIA-QUA-002: Quantum Algorithm - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Complex Numbers and Linear Algebra
// ============================================================================

/**
 * Complex number representation
 */
export interface Complex {
  real: number;
  imag: number;
}

/**
 * Complex vector (quantum state)
 */
export type ComplexVector = Complex[];

/**
 * Complex matrix (quantum gate/operator)
 */
export type ComplexMatrix = Complex[][];

/**
 * Real-valued vector
 */
export type RealVector = number[];

/**
 * Real-valued matrix
 */
export type RealMatrix = number[][];

// ============================================================================
// Quantum Gates
// ============================================================================

/**
 * Single-qubit gate types
 */
export type SingleQubitGate =
  | 'I'     // Identity
  | 'X'     // Pauli-X (NOT)
  | 'Y'     // Pauli-Y
  | 'Z'     // Pauli-Z
  | 'H'     // Hadamard
  | 'S'     // Phase gate
  | 'T'     // T-gate (π/8)
  | 'Sdg'   // S-dagger
  | 'Tdg';  // T-dagger

/**
 * Parameterized single-qubit gate types
 */
export type ParameterizedSingleGate =
  | 'Rx'    // Rotation around X-axis
  | 'Ry'    // Rotation around Y-axis
  | 'Rz'    // Rotation around Z-axis
  | 'U'     // Universal single-qubit gate
  | 'Phase'; // Phase shift

/**
 * Multi-qubit gate types
 */
export type MultiQubitGate =
  | 'CNOT'    // Controlled-NOT
  | 'CZ'      // Controlled-Z
  | 'SWAP'    // Swap gate
  | 'Toffoli' // Controlled-Controlled-NOT
  | 'Fredkin'; // Controlled-SWAP

/**
 * Gate representation
 */
export interface Gate {
  /** Gate type */
  type: SingleQubitGate | ParameterizedSingleGate | MultiQubitGate;

  /** Target qubit(s) */
  qubits: number[];

  /** Control qubit(s) for controlled gates */
  controls?: number[];

  /** Parameters for parameterized gates */
  params?: number[];

  /** Gate matrix (optional, computed if not provided) */
  matrix?: ComplexMatrix;

  /** Gate label/name */
  label?: string;
}

// ============================================================================
// Quantum Circuits
// ============================================================================

/**
 * Quantum circuit configuration
 */
export interface QuantumCircuit {
  /** Number of qubits */
  numQubits: number;

  /** Number of classical bits (for measurement) */
  numClassicalBits?: number;

  /** Sequence of gates */
  gates: Gate[];

  /** Circuit name/label */
  name?: string;

  /** Circuit metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Quantum state representation
 */
export interface QuantumState {
  /** State vector (2^n dimensional) */
  amplitudes: ComplexVector;

  /** Number of qubits */
  numQubits: number;

  /** Is state normalized? */
  normalized: boolean;

  /** State label */
  label?: string;
}

/**
 * Measurement result
 */
export interface MeasurementResult {
  /** Measured bit string */
  bitString: string;

  /** Decimal representation */
  value: number;

  /** Probability of this outcome */
  probability: number;

  /** Which qubits were measured */
  measuredQubits: number[];
}

/**
 * Circuit execution result
 */
export interface CircuitResult {
  /** Final state vector */
  finalState: QuantumState;

  /** Measurement results (if measured) */
  measurements?: MeasurementResult[];

  /** Execution time in milliseconds */
  executionTime: number;

  /** Number of gates executed */
  gateCount: number;

  /** Success status */
  success: boolean;

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// Quantum Algorithms
// ============================================================================

/**
 * Shor's algorithm parameters
 */
export interface ShorParameters {
  /** Number to factorize */
  N: number;

  /** Random integer a (coprime to N) */
  a?: number;

  /** Number of qubits for first register */
  numQubits1?: number;

  /** Number of qubits for second register */
  numQubits2?: number;

  /** Maximum attempts */
  maxAttempts?: number;
}

/**
 * Shor's algorithm result
 */
export interface ShorResult {
  /** Original number */
  N: number;

  /** Found factors */
  factors: number[];

  /** Period found */
  period?: number;

  /** Random a used */
  a: number;

  /** Number of attempts */
  attempts: number;

  /** Success status */
  success: boolean;

  /** Classical verification */
  verified: boolean;
}

/**
 * Grover's algorithm parameters
 */
export interface GroverParameters {
  /** Size of search space */
  N: number;

  /** Oracle function: returns true for marked items */
  oracle: (x: number) => boolean;

  /** Number of marked items (if known) */
  numMarked?: number;

  /** Number of iterations (auto-calculated if not provided) */
  iterations?: number;

  /** Number of qubits */
  numQubits?: number;
}

/**
 * Grover's algorithm result
 */
export interface GroverResult {
  /** Found item(s) */
  found: number[];

  /** Search space size */
  searchSize: number;

  /** Number of iterations used */
  iterations: number;

  /** Success probability */
  probability: number;

  /** Measurement counts */
  counts: Record<number, number>;

  /** Success status */
  success: boolean;
}

/**
 * VQE (Variational Quantum Eigensolver) parameters
 */
export interface VQEParameters {
  /** Hamiltonian as Pauli string sum */
  hamiltonian: PauliOperator;

  /** Ansatz circuit */
  ansatz: QuantumCircuit | ((params: number[]) => QuantumCircuit);

  /** Initial parameters */
  initialParams?: number[];

  /** Classical optimizer */
  optimizer?: 'COBYLA' | 'BFGS' | 'SPSA' | 'GradientDescent';

  /** Maximum iterations */
  maxIterations?: number;

  /** Convergence tolerance */
  tolerance?: number;

  /** Number of shots per measurement */
  shots?: number;
}

/**
 * VQE result
 */
export interface VQEResult {
  /** Ground state energy */
  energy: number;

  /** Optimal parameters */
  optimalParams: number[];

  /** Ground state wavefunction */
  groundState: QuantumState;

  /** Optimization history */
  energyHistory: number[];

  /** Number of iterations */
  iterations: number;

  /** Convergence status */
  converged: boolean;

  /** Final gradient norm */
  gradientNorm?: number;
}

/**
 * QAOA (Quantum Approximate Optimization Algorithm) parameters
 */
export interface QAOAParameters {
  /** Cost Hamiltonian */
  costHamiltonian: PauliOperator;

  /** Mixer Hamiltonian (optional, defaults to X-mixer) */
  mixerHamiltonian?: PauliOperator;

  /** Number of layers (p) */
  layers: number;

  /** Initial parameters [β, γ] */
  initialParams?: number[];

  /** Classical optimizer */
  optimizer?: 'COBYLA' | 'BFGS' | 'SPSA';

  /** Maximum iterations */
  maxIterations?: number;

  /** Number of shots */
  shots?: number;
}

/**
 * QAOA result
 */
export interface QAOAResult {
  /** Optimal bit string */
  optimalSolution: string;

  /** Cost of optimal solution */
  optimalCost: number;

  /** Optimal parameters */
  optimalParams: number[];

  /** Approximation ratio */
  approximationRatio: number;

  /** All measurement results */
  measurements: Record<string, number>;

  /** Number of layers used */
  layers: number;

  /** Success probability */
  successProbability: number;
}

// ============================================================================
// Quantum Operators
// ============================================================================

/**
 * Pauli string (e.g., "XIYZ")
 */
export type PauliString = string;

/**
 * Pauli term with coefficient
 */
export interface PauliTerm {
  /** Pauli string */
  pauli: PauliString;

  /** Coefficient */
  coefficient: number;

  /** Qubits the operator acts on */
  qubits?: number[];
}

/**
 * Pauli operator (sum of Pauli terms)
 */
export interface PauliOperator {
  /** List of Pauli terms */
  terms: PauliTerm[];

  /** Number of qubits */
  numQubits: number;

  /** Operator name/label */
  label?: string;
}

/**
 * General quantum operator
 */
export interface QuantumOperator {
  /** Operator matrix */
  matrix: ComplexMatrix;

  /** Number of qubits */
  numQubits: number;

  /** Is operator Hermitian? */
  hermitian: boolean;

  /** Is operator unitary? */
  unitary: boolean;

  /** Operator label */
  label?: string;
}

// ============================================================================
// Quantum Error Correction
// ============================================================================

/**
 * Error correction code types
 */
export type ErrorCorrectionCode =
  | 'Shor'      // Shor 9-qubit code
  | 'Steane'    // Steane 7-qubit code
  | 'Surface'   // Surface code
  | 'Bacon-Shor' // Bacon-Shor code
  | 'Color'     // Color code
  | 'Repetition'; // Simple repetition code

/**
 * Error type
 */
export type ErrorType = 'bit-flip' | 'phase-flip' | 'depolarizing' | 'amplitude-damping';

/**
 * Error correction code specification
 */
export interface ErrorCorrectionSpec {
  /** Code type */
  type: ErrorCorrectionCode;

  /** Number of physical qubits */
  physicalQubits: number;

  /** Number of logical qubits */
  logicalQubits: number;

  /** Code distance */
  distance: number;

  /** Stabilizer generators */
  stabilizers: PauliOperator[];

  /** Logical X operators */
  logicalX: PauliOperator[];

  /** Logical Z operators */
  logicalZ: PauliOperator[];

  /** Encoding circuit */
  encodingCircuit?: QuantumCircuit;

  /** Decoding circuit */
  decodingCircuit?: QuantumCircuit;
}

/**
 * Error syndrome
 */
export interface ErrorSyndrome {
  /** Syndrome measurement results */
  syndrome: number[];

  /** Detected error location */
  errorLocation?: number;

  /** Detected error type */
  errorType?: ErrorType;

  /** Correction gate to apply */
  correction?: Gate;
}

/**
 * Error correction result
 */
export interface ErrorCorrectionResult {
  /** Original (noisy) state */
  noisyState: QuantumState;

  /** Corrected state */
  correctedState: QuantumState;

  /** Syndrome measurements */
  syndromes: ErrorSyndrome[];

  /** Fidelity after correction */
  fidelity: number;

  /** Success status */
  success: boolean;
}

// ============================================================================
// Quantum Benchmarking
// ============================================================================

/**
 * Quantum volume parameters
 */
export interface QuantumVolumeParams {
  /** Number of qubits */
  numQubits: number;

  /** Circuit depth */
  depth: number;

  /** Number of trials */
  trials: number;

  /** Success threshold */
  threshold?: number;
}

/**
 * Quantum volume result
 */
export interface QuantumVolumeResult {
  /** Achieved quantum volume */
  quantumVolume: number;

  /** Heavy output probability */
  heavyOutputProbability: number;

  /** Number of qubits tested */
  numQubits: number;

  /** Circuit depth tested */
  depth: number;

  /** Success status */
  passed: boolean;
}

/**
 * Random circuit sampling parameters
 */
export interface RandomCircuitParams {
  /** Number of qubits */
  numQubits: number;

  /** Circuit depth */
  depth: number;

  /** Gate set */
  gateSet: (SingleQubitGate | MultiQubitGate)[];

  /** Number of circuits */
  numCircuits: number;

  /** Number of shots per circuit */
  shots: number;
}

/**
 * Cross-entropy benchmarking result
 */
export interface CrossEntropyResult {
  /** Cross-entropy value */
  crossEntropy: number;

  /** Estimated fidelity */
  fidelity: number;

  /** Number of qubits */
  numQubits: number;

  /** Circuit depth */
  depth: number;

  /** Classical simulation feasible? */
  classicalSimulationFeasible: boolean;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Quantum physical constants
 */
export const QUANTUM_CONSTANTS = {
  /** Planck constant (J·s) */
  PLANCK: 6.62607015e-34,

  /** Reduced Planck constant (J·s) */
  HBAR: 1.054571817e-34,

  /** Rydberg constant (m⁻¹) */
  RYDBERG: 1.0973731568160e7,

  /** Bohr radius (m) */
  BOHR_RADIUS: 5.29177210903e-11,

  /** Fine structure constant */
  FINE_STRUCTURE: 1 / 137.035999084,

  /** Electron mass (kg) */
  ELECTRON_MASS: 9.1093837015e-31,

  /** Proton mass (kg) */
  PROTON_MASS: 1.67262192369e-27,

  /** Elementary charge (C) */
  ELEMENTARY_CHARGE: 1.602176634e-19,

  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,
} as const;

/**
 * Computational precision constants
 */
export const PRECISION = {
  /** Numerical tolerance */
  TOLERANCE: 1e-10,

  /** Probability normalization tolerance */
  NORM_TOLERANCE: 1e-8,

  /** Gate fidelity threshold */
  GATE_FIDELITY_THRESHOLD: 0.99,

  /** Error correction threshold */
  ERROR_THRESHOLD: 0.01,
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

/**
 * Bit string (e.g., "01101")
 */
export type BitString = string;

/**
 * Probability distribution over bit strings
 */
export type Distribution = Record<BitString, number>;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-002 error codes
 */
export enum QuantumErrorCode {
  INVALID_QUBIT_INDEX = 'Q001',
  NON_UNITARY_GATE = 'Q002',
  MEASUREMENT_BEFORE_PREPARATION = 'Q003',
  INSUFFICIENT_QUBITS = 'Q004',
  FACTORIZATION_FAILED = 'Q005',
  ORACLE_NOT_PROVIDED = 'Q006',
  CONVERGENCE_FAILED = 'Q007',
  INVALID_HAMILTONIAN = 'Q008',
  STATE_NOT_NORMALIZED = 'Q009',
  DIMENSION_MISMATCH = 'Q010',
}

/**
 * Quantum algorithm error
 */
export class QuantumError extends Error {
  constructor(
    public code: QuantumErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'QuantumError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Complex numbers
  Complex,
  ComplexVector,
  ComplexMatrix,
  RealVector,
  RealMatrix,

  // Gates
  SingleQubitGate,
  ParameterizedSingleGate,
  MultiQubitGate,
  Gate,

  // Circuits
  QuantumCircuit,
  QuantumState,
  MeasurementResult,
  CircuitResult,

  // Algorithms
  ShorParameters,
  ShorResult,
  GroverParameters,
  GroverResult,
  VQEParameters,
  VQEResult,
  QAOAParameters,
  QAOAResult,

  // Operators
  PauliString,
  PauliTerm,
  PauliOperator,
  QuantumOperator,

  // Error correction
  ErrorCorrectionCode,
  ErrorType,
  ErrorCorrectionSpec,
  ErrorSyndrome,
  ErrorCorrectionResult,

  // Benchmarking
  QuantumVolumeParams,
  QuantumVolumeResult,
  RandomCircuitParams,
  CrossEntropyResult,

  // Utility
  BitString,
  Distribution,
};

export { QUANTUM_CONSTANTS, PRECISION, QuantumErrorCode, QuantumError };
