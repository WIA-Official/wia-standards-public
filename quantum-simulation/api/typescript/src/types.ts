/**
 * WIA-QUA-005: Quantum Simulation - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Complex number representation
 */
export interface Complex {
  real: number;
  imag: number;
}

/**
 * Quantum state representation methods
 */
export type SimulationMethod =
  | 'statevector'
  | 'densitymatrix'
  | 'mps'
  | 'peps'
  | 'clifford'
  | 'stabilizer';

/**
 * Quantum simulator configuration
 */
export interface SimulatorConfig {
  /** Number of qubits */
  numQubits: number;

  /** Simulation method */
  method: SimulationMethod;

  /** Numerical precision */
  precision?: 'single' | 'double';

  /** Enable GPU acceleration */
  useGPU?: boolean;

  /** Random seed for reproducibility */
  seed?: number;

  /** Bond dimension for tensor network methods */
  bondDimension?: number;

  /** Memory limit in bytes */
  memoryLimit?: number;
}

// ============================================================================
// Quantum State
// ============================================================================

/**
 * State vector representation
 */
export interface StateVector {
  /** Number of qubits */
  numQubits: number;

  /** Complex amplitudes (length 2^numQubits) */
  amplitudes: Complex[];

  /** Is state normalized? */
  normalized: boolean;

  /** State metadata */
  metadata?: {
    createdAt: Date;
    label?: string;
  };
}

/**
 * Density matrix representation
 */
export interface DensityMatrix {
  /** Number of qubits */
  numQubits: number;

  /** Matrix elements (2^n × 2^n) */
  matrix: Complex[][];

  /** Purity Tr(ρ²) */
  purity: number;

  /** Is valid density matrix? */
  isValid: boolean;
}

/**
 * Matrix Product State (MPS) representation
 */
export interface MatrixProductState {
  /** Number of qubits */
  numQubits: number;

  /** MPS tensors for each site */
  tensors: Complex[][][]; // [site][left_bond][right_bond][physical_dim]

  /** Bond dimensions */
  bondDimensions: number[];

  /** Canonical form */
  canonicalForm?: 'left' | 'right' | 'mixed';

  /** Center site (for mixed canonical form) */
  centerSite?: number;
}

/**
 * Quantum state (unified interface)
 */
export type QuantumState = StateVector | DensityMatrix | MatrixProductState;

// ============================================================================
// Quantum Gates
// ============================================================================

/**
 * Gate types
 */
export type GateType =
  // Single-qubit gates
  | 'I'
  | 'X'
  | 'Y'
  | 'Z'
  | 'H'
  | 'S'
  | 'T'
  | 'Sdg'
  | 'Tdg'
  | 'Rx'
  | 'Ry'
  | 'Rz'
  | 'U'
  | 'U1'
  | 'U2'
  | 'U3'
  // Two-qubit gates
  | 'CNOT'
  | 'CX'
  | 'CY'
  | 'CZ'
  | 'SWAP'
  | 'iSWAP'
  | 'CRx'
  | 'CRy'
  | 'CRz'
  | 'CU'
  // Three-qubit gates
  | 'CCNOT'
  | 'Toffoli'
  | 'CSWAP'
  | 'Fredkin'
  // Multi-qubit gates
  | 'MCX'
  | 'MCZ';

/**
 * Quantum gate definition
 */
export interface Gate {
  /** Gate type */
  type: GateType;

  /** Qubits this gate acts on */
  qubits: number[];

  /** Gate parameters (e.g., rotation angles) */
  parameters?: number[];

  /** Custom gate matrix (if not standard) */
  matrix?: Complex[][];

  /** Gate label/name */
  label?: string;

  /** Classical control bits */
  condition?: {
    register: string;
    value: number;
  };
}

/**
 * Parameterized gate for variational circuits
 */
export interface ParameterizedGate extends Gate {
  /** Parameter names */
  parameterNames: string[];

  /** Parameter bounds */
  bounds?: Array<[number, number]>;
}

// ============================================================================
// Quantum Circuit
// ============================================================================

/**
 * Quantum circuit
 */
export interface QuantumCircuit {
  /** Number of qubits */
  numQubits: number;

  /** Number of classical bits */
  numClbits?: number;

  /** List of gates */
  gates: Gate[];

  /** Measurements */
  measurements?: Measurement[];

  /** Circuit depth */
  depth?: number;

  /** Circuit name */
  name?: string;

  /** Global phase */
  globalPhase?: number;
}

/**
 * Measurement specification
 */
export interface Measurement {
  /** Qubit to measure */
  qubit: number;

  /** Classical bit to store result */
  clbit?: number;

  /** Measurement basis */
  basis?: 'computational' | 'pauli-x' | 'pauli-y' | 'custom';

  /** Custom measurement operator */
  operator?: Complex[][];
}

// ============================================================================
// Simulation Results
// ============================================================================

/**
 * Circuit simulation result
 */
export interface SimulationResult {
  /** Final quantum state */
  state: QuantumState;

  /** Measurement outcomes (if measured) */
  measurements?: number[];

  /** Measurement counts (for repeated execution) */
  counts?: Record<string, number>;

  /** Execution time in milliseconds */
  executionTime: number;

  /** Memory used in bytes */
  memoryUsed: number;

  /** Success flag */
  success: boolean;

  /** Error message if failed */
  error?: string;
}

/**
 * Expectation value calculation result
 */
export interface ExpectationResult {
  /** Expectation value ⟨O⟩ */
  value: number;

  /** Standard deviation (if sampled) */
  stddev?: number;

  /** Number of samples */
  numSamples?: number;

  /** Observable that was measured */
  observable: Observable;
}

// ============================================================================
// Observables and Operators
// ============================================================================

/**
 * Quantum observable
 */
export interface Observable {
  /** Observable name */
  name?: string;

  /** Operator matrix or Pauli decomposition */
  representation: Complex[][] | PauliString[];

  /** Hermitian check */
  isHermitian: boolean;

  /** Eigenvalues (if known) */
  eigenvalues?: number[];
}

/**
 * Pauli string representation
 */
export interface PauliString {
  /** Pauli operators (I, X, Y, Z) for each qubit */
  paulis: ('I' | 'X' | 'Y' | 'Z')[];

  /** Coefficient */
  coefficient: Complex;

  /** Qubits this string acts on */
  qubits?: number[];
}

/**
 * Hamiltonian operator
 */
export interface Hamiltonian extends Observable {
  /** Pauli decomposition */
  pauliTerms: PauliString[];

  /** Number of terms */
  numTerms: number;

  /** Ground state energy (if known) */
  groundEnergy?: number;
}

// ============================================================================
// Quantum Chemistry
// ============================================================================

/**
 * Molecular structure
 */
export interface Molecule {
  /** Atomic symbols */
  atoms: string[];

  /** Atomic coordinates in Bohr */
  coordinates: number[][];

  /** Molecular charge */
  charge: number;

  /** Spin multiplicity */
  multiplicity: number;

  /** Basis set */
  basis: string;

  /** Symmetry group */
  symmetry?: string;
}

/**
 * Electronic structure result
 */
export interface ElectronicStructure {
  /** Hartree-Fock energy */
  hfEnergy: number;

  /** Correlation energy */
  correlationEnergy?: number;

  /** Total energy */
  totalEnergy: number;

  /** Molecular orbitals */
  orbitals: MolecularOrbital[];

  /** One-electron integrals */
  oneBodyIntegrals: number[][];

  /** Two-electron integrals */
  twoBodyIntegrals: number[][][][];

  /** Number of electrons */
  numElectrons: number;

  /** Number of orbitals */
  numOrbitals: number;
}

/**
 * Molecular orbital
 */
export interface MolecularOrbital {
  /** Orbital index */
  index: number;

  /** Orbital energy */
  energy: number;

  /** Occupation (0, 1, or 2) */
  occupation: number;

  /** Orbital coefficients */
  coefficients: number[];

  /** Orbital type */
  type: 'core' | 'active' | 'virtual';
}

/**
 * VQE (Variational Quantum Eigensolver) configuration
 */
export interface VQEConfig {
  /** Hamiltonian to minimize */
  hamiltonian: Hamiltonian;

  /** Ansatz circuit */
  ansatz: QuantumCircuit | string;

  /** Classical optimizer */
  optimizer: OptimizerConfig;

  /** Initial parameters */
  initialParameters?: number[];

  /** Maximum iterations */
  maxIterations?: number;

  /** Convergence tolerance */
  tolerance?: number;

  /** Number of measurement shots per iteration */
  shots?: number;
}

/**
 * Optimizer configuration
 */
export interface OptimizerConfig {
  /** Optimizer type */
  method: 'COBYLA' | 'BFGS' | 'L-BFGS-B' | 'SLSQP' | 'Nelder-Mead' | 'SPSA';

  /** Maximum function evaluations */
  maxfev?: number;

  /** Tolerance */
  tolerance?: number;

  /** Learning rate (for gradient-based methods) */
  learningRate?: number;

  /** Additional options */
  options?: Record<string, unknown>;
}

/**
 * VQE result
 */
export interface VQEResult {
  /** Optimal energy */
  energy: number;

  /** Optimal parameters */
  parameters: number[];

  /** Number of iterations */
  iterations: number;

  /** Convergence history */
  history: {
    iteration: number;
    energy: number;
    parameters: number[];
  }[];

  /** Success flag */
  converged: boolean;

  /** Final quantum state */
  state?: QuantumState;
}

// ============================================================================
// Noise Models
// ============================================================================

/**
 * Quantum noise model
 */
export interface NoiseModel {
  /** Noise model name */
  name: string;

  /** Gate errors */
  gateErrors: Map<GateType, QuantumError>;

  /** Readout errors */
  readoutErrors?: ReadoutError[];

  /** Thermal relaxation */
  thermalRelaxation?: ThermalRelaxation[];

  /** Custom errors */
  customErrors?: QuantumError[];
}

/**
 * Quantum error (quantum channel)
 */
export interface QuantumError {
  /** Error type */
  type: 'depolarizing' | 'amplitude_damping' | 'phase_damping' | 'pauli' | 'kraus';

  /** Error probability */
  probability: number;

  /** Affected qubits */
  qubits: number[];

  /** Kraus operators (for Kraus representation) */
  krausOperators?: Complex[][][];

  /** Pauli error probabilities (for Pauli channels) */
  pauliProbabilities?: {
    I: number;
    X: number;
    Y: number;
    Z: number;
  };
}

/**
 * Readout error model
 */
export interface ReadoutError {
  /** Qubit index */
  qubit: number;

  /** Confusion matrix */
  confusionMatrix: number[][];

  /** p(0|0), p(0|1), p(1|0), p(1|1) */
  probabilities: [number, number, number, number];
}

/**
 * Thermal relaxation parameters
 */
export interface ThermalRelaxation {
  /** Qubit index */
  qubit: number;

  /** T1 time (amplitude damping) in microseconds */
  T1: number;

  /** T2 time (dephasing) in microseconds */
  T2: number;

  /** Gate time in microseconds */
  gateTime: number;

  /** Temperature in Kelvin */
  temperature?: number;
}

// ============================================================================
// Tensor Networks
// ============================================================================

/**
 * Tensor network configuration
 */
export interface TensorNetworkConfig {
  /** Network type */
  type: 'MPS' | 'PEPS' | 'MERA' | 'TTN';

  /** Maximum bond dimension */
  maxBondDim: number;

  /** Truncation tolerance */
  tolerance: number;

  /** Canonical form to maintain */
  canonical?: 'left' | 'right' | 'mixed';

  /** Compression algorithm */
  compressionMethod?: 'SVD' | 'QR' | 'variational';
}

/**
 * SVD truncation result
 */
export interface SVDTruncation {
  /** Truncated singular values */
  singularValues: number[];

  /** Truncation error */
  error: number;

  /** New bond dimension */
  bondDim: number;

  /** Discarded weight */
  discardedWeight: number;
}

// ============================================================================
// Time Evolution
// ============================================================================

/**
 * Time evolution configuration
 */
export interface TimeEvolutionConfig {
  /** Hamiltonian */
  hamiltonian: Hamiltonian;

  /** Total evolution time */
  time: number;

  /** Time step */
  dt: number;

  /** Evolution method */
  method: 'exact' | 'trotter' | 'krylov' | 'tdvp';

  /** Trotter order (if using Trotter) */
  trotterOrder?: 1 | 2 | 4;

  /** Store intermediate states */
  storeIntermediates?: boolean;
}

/**
 * Time evolution result
 */
export interface TimeEvolutionResult {
  /** Final state */
  finalState: QuantumState;

  /** Intermediate states (if stored) */
  intermediateStates?: QuantumState[];

  /** Time points */
  times?: number[];

  /** Execution time */
  executionTime: number;
}

// ============================================================================
// Quantum Metrics
// ============================================================================

/**
 * Quantum state fidelity
 */
export interface Fidelity {
  /** Fidelity value (0 to 1) */
  value: number;

  /** State 1 */
  state1: QuantumState;

  /** State 2 */
  state2: QuantumState;

  /** Fidelity type */
  type: 'state' | 'process';
}

/**
 * Entanglement measures
 */
export interface EntanglementMeasure {
  /** von Neumann entropy */
  vonNeumannEntropy?: number;

  /** Renyi entropy */
  renyiEntropy?: number;

  /** Entanglement of formation */
  entanglementOfFormation?: number;

  /** Concurrence (for 2-qubit systems) */
  concurrence?: number;

  /** Negativity */
  negativity?: number;
}

/**
 * Purity measure
 */
export interface Purity {
  /** Purity value Tr(ρ²) */
  value: number;

  /** Linear entropy 1 - Tr(ρ²) */
  linearEntropy: number;

  /** Is pure state? */
  isPure: boolean;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for quantum simulation
 */
export const QUANTUM_CONSTANTS = {
  /** Planck constant (J·s) */
  PLANCK: 6.62607015e-34,

  /** Reduced Planck constant (J·s) */
  HBAR: 1.054571817e-34,

  /** Boltzmann constant (J/K) */
  BOLTZMANN: 1.380649e-23,

  /** Elementary charge (C) */
  ELEMENTARY_CHARGE: 1.602176634e-19,

  /** Electron mass (kg) */
  ELECTRON_MASS: 9.1093837015e-31,

  /** Hartree energy (J) */
  HARTREE: 4.3597447222071e-18,

  /** Bohr radius (m) */
  BOHR_RADIUS: 5.29177210903e-11,

  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Fine structure constant */
  FINE_STRUCTURE: 7.2973525693e-3,
} as const;

/**
 * Pauli matrices
 */
export const PAULI_MATRICES = {
  I: [
    [{ real: 1, imag: 0 }, { real: 0, imag: 0 }],
    [{ real: 0, imag: 0 }, { real: 1, imag: 0 }],
  ],
  X: [
    [{ real: 0, imag: 0 }, { real: 1, imag: 0 }],
    [{ real: 1, imag: 0 }, { real: 0, imag: 0 }],
  ],
  Y: [
    [{ real: 0, imag: 0 }, { real: 0, imag: -1 }],
    [{ real: 0, imag: 1 }, { real: 0, imag: 0 }],
  ],
  Z: [
    [{ real: 1, imag: 0 }, { real: 0, imag: 0 }],
    [{ real: 0, imag: 0 }, { real: -1, imag: 0 }],
  ],
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-005 error codes
 */
export enum QuantumErrorCode {
  INVALID_QUBIT = 'Q001',
  NON_UNITARY_GATE = 'Q002',
  STATE_NOT_NORMALIZED = 'Q003',
  MEMORY_EXCEEDED = 'Q004',
  INVALID_MEASUREMENT_BASIS = 'Q005',
  NUMERICAL_INSTABILITY = 'Q006',
  INVALID_CIRCUIT = 'Q007',
  INVALID_HAMILTONIAN = 'Q008',
  OPTIMIZATION_FAILED = 'Q009',
  INVALID_MOLECULE = 'Q010',
}

/**
 * Quantum simulation error
 */
export class QuantumSimulationError extends Error {
  constructor(
    public code: QuantumErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'QuantumSimulationError';
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

/**
 * Bitstring representation
 */
export type Bitstring = string; // e.g., "0110101"

/**
 * Probability distribution over measurement outcomes
 */
export type ProbabilityDistribution = Map<Bitstring, number>;

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core
  Complex,
  SimulationMethod,
  SimulatorConfig,

  // States
  StateVector,
  DensityMatrix,
  MatrixProductState,
  QuantumState,

  // Gates and Circuits
  GateType,
  Gate,
  ParameterizedGate,
  QuantumCircuit,
  Measurement,

  // Results
  SimulationResult,
  ExpectationResult,

  // Observables
  Observable,
  PauliString,
  Hamiltonian,

  // Chemistry
  Molecule,
  ElectronicStructure,
  MolecularOrbital,
  VQEConfig,
  OptimizerConfig,
  VQEResult,

  // Noise
  NoiseModel,
  QuantumError,
  ReadoutError,
  ThermalRelaxation,

  // Tensor Networks
  TensorNetworkConfig,
  SVDTruncation,

  // Time Evolution
  TimeEvolutionConfig,
  TimeEvolutionResult,

  // Metrics
  Fidelity,
  EntanglementMeasure,
  Purity,

  // Utility
  Bitstring,
  ProbabilityDistribution,
};

export { QUANTUM_CONSTANTS, PAULI_MATRICES, QuantumErrorCode, QuantumSimulationError };

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
