/**
 * WIA-QUA-006: Quantum Machine Learning - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum ML Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Quantum ML Types
// ============================================================================

/**
 * Complex number representation
 */
export interface Complex {
  real: number;
  imag: number;
}

/**
 * Quantum gate types
 */
export type QuantumGate = 'RX' | 'RY' | 'RZ' | 'H' | 'X' | 'Y' | 'Z' | 'CNOT' | 'CZ' | 'SWAP' | 'T' | 'S';

/**
 * Pauli operators for measurements
 */
export type PauliOperator = 'X' | 'Y' | 'Z' | 'I';

/**
 * Entanglement patterns for quantum circuits
 */
export type EntanglementPattern = 'linear' | 'full' | 'circular' | 'custom';

/**
 * Data encoding strategies
 */
export type EncodingStrategy = 'amplitude' | 'angle' | 'basis' | 'hamiltonian' | 'custom';

/**
 * Optimizer types for training
 */
export type OptimizerType = 'adam' | 'sgd' | 'cobyla' | 'spsa' | 'lbfgs' | 'nelder-mead';

/**
 * Loss function types
 */
export type LossFunction = 'mse' | 'cross-entropy' | 'hinge' | 'log-loss' | 'custom';

// ============================================================================
// Quantum Neural Network (QNN)
// ============================================================================

/**
 * QNN architecture configuration
 */
export interface QNNConfig {
  /** Number of qubits in the circuit */
  numQubits: number;

  /** Number of variational layers */
  numLayers: number;

  /** Entanglement pattern between qubits */
  entanglementPattern: EntanglementPattern;

  /** Rotation gates to use in each layer */
  rotationGates: QuantumGate[];

  /** Measurement operators */
  measurements: PauliOperator[];

  /** Number of measurement shots */
  shots?: number;

  /** Custom entanglement configuration */
  customEntanglement?: number[][];
}

/**
 * QNN prediction result
 */
export interface QNNResult {
  /** Predicted value(s) */
  prediction: number | number[];

  /** Expectation value of measurement */
  expectationValue: number;

  /** Measurement variance */
  variance: number;

  /** Number of shots used */
  shots: number;

  /** Raw measurement counts */
  counts?: Record<string, number>;
}

/**
 * Quantum Neural Network instance
 */
export interface QuantumNeuralNetwork {
  /** Configuration */
  config: QNNConfig;

  /** Current parameters */
  parameters: number[];

  /** Number of parameters */
  numParameters: number;

  /** Circuit depth */
  circuitDepth: number;

  /** Make prediction */
  predict(input: number[]): QNNResult;

  /** Get circuit representation */
  getCircuit(input?: number[]): QuantumCircuit;
}

// ============================================================================
// Variational Quantum Classifier (VQC)
// ============================================================================

/**
 * VQC configuration
 */
export interface VQCConfig {
  /** Number of qubits */
  numQubits: number;

  /** Feature map for data encoding */
  featureMap: EncodingStrategy;

  /** Variational ansatz type */
  ansatz: 'hardware-efficient' | 'two-local' | 'real-amplitudes' | 'custom';

  /** Number of ansatz layers */
  ansatzLayers?: number;

  /** Measurement operators */
  measurements: PauliOperator[];

  /** Optimizer configuration */
  optimizer?: OptimizerConfig;

  /** Number of shots per measurement */
  shots?: number;
}

/**
 * VQC training result
 */
export interface VQCTrainingResult {
  /** Final training loss */
  finalLoss: number;

  /** Training accuracy */
  accuracy: number;

  /** Optimized parameters */
  parameters: number[];

  /** Loss history per epoch */
  lossHistory: number[];

  /** Accuracy history per epoch */
  accuracyHistory?: number[];

  /** Training time in milliseconds */
  trainingTime: number;

  /** Number of iterations */
  iterations: number;
}

/**
 * Variational Quantum Classifier instance
 */
export interface VariationalQuantumClassifier {
  /** Configuration */
  config: VQCConfig;

  /** Current parameters */
  parameters: number[];

  /** Train the classifier */
  fit(X: number[][], y: number[]): Promise<VQCTrainingResult>;

  /** Make predictions */
  predict(X: number[][]): number[];

  /** Predict probabilities */
  predictProba(X: number[][]): number[][];

  /** Evaluate on test data */
  score(X: number[][], y: number[]): number;
}

// ============================================================================
// Quantum Kernel Methods
// ============================================================================

/**
 * Quantum kernel configuration
 */
export interface QuantumKernelConfig {
  /** Number of qubits */
  numQubits: number;

  /** Feature map for encoding */
  featureMap: EncodingStrategy;

  /** Number of repetitions */
  reps?: number;

  /** Entanglement pattern */
  entanglement?: EntanglementPattern;

  /** Number of shots */
  shots?: number;
}

/**
 * Kernel matrix result
 */
export interface KernelMatrix {
  /** Kernel matrix values */
  values: number[][];

  /** Training data */
  trainingData: number[][];

  /** Test data (optional) */
  testData?: number[][];

  /** Computation time in milliseconds */
  computationTime: number;
}

/**
 * Quantum SVM configuration
 */
export interface QSVMConfig {
  /** Quantum kernel configuration */
  kernel: QuantumKernelConfig;

  /** SVM regularization parameter */
  C?: number;

  /** Kernel cache size */
  cacheSize?: number;
}

/**
 * QSVM instance
 */
export interface QuantumSVM {
  /** Configuration */
  config: QSVMConfig;

  /** Support vectors */
  supportVectors?: number[][];

  /** Dual coefficients */
  dualCoef?: number[];

  /** Train the SVM */
  fit(X: number[][], y: number[]): Promise<void>;

  /** Make predictions */
  predict(X: number[][]): number[];

  /** Get decision function values */
  decisionFunction(X: number[][]): number[];
}

// ============================================================================
// Quantum Generative Models
// ============================================================================

/**
 * Quantum GAN configuration
 */
export interface QGANConfig {
  /** Generator configuration */
  generator: {
    numQubits: number;
    numLayers: number;
    latentDim: number;
  };

  /** Discriminator configuration */
  discriminator: {
    type: 'classical' | 'quantum';
    layers?: number[];
  };

  /** Training configuration */
  training?: {
    epochs: number;
    batchSize: number;
    learningRateG: number;
    learningRateD: number;
  };
}

/**
 * Quantum Boltzmann Machine configuration
 */
export interface QBMConfig {
  /** Number of visible units */
  visibleUnits: number;

  /** Number of hidden units */
  hiddenUnits: number;

  /** Number of Gibbs sampling steps */
  gibbsSteps?: number;

  /** Learning rate */
  learningRate?: number;

  /** Temperature parameter */
  temperature?: number;
}

/**
 * Quantum Circuit Born Machine configuration
 */
export interface QCBMConfig {
  /** Number of qubits */
  numQubits: number;

  /** Number of layers */
  numLayers: number;

  /** Entanglement pattern */
  entanglement: EntanglementPattern;

  /** Training shots */
  shots?: number;
}

// ============================================================================
// Training and Optimization
// ============================================================================

/**
 * Optimizer configuration
 */
export interface OptimizerConfig {
  /** Optimizer type */
  type: OptimizerType;

  /** Learning rate */
  learningRate: number;

  /** Maximum iterations */
  maxIterations?: number;

  /** Convergence tolerance */
  tolerance?: number;

  /** Gradient computation method */
  gradientMethod?: 'parameter-shift' | 'finite-diff' | 'natural';

  /** Additional optimizer-specific options */
  options?: Record<string, unknown>;
}

/**
 * Training configuration
 */
export interface TrainingConfig {
  /** Training data */
  data: number[][];

  /** Training labels */
  labels: number[];

  /** Number of training epochs */
  epochs: number;

  /** Batch size (optional) */
  batchSize?: number;

  /** Optimizer type */
  optimizer: OptimizerType;

  /** Learning rate */
  learningRate: number;

  /** Loss function */
  lossFunction: LossFunction;

  /** Validation split ratio */
  validationSplit?: number;

  /** Early stopping patience */
  earlyStoppingPatience?: number;

  /** Callback functions */
  callbacks?: TrainingCallback[];

  /** Random seed */
  seed?: number;
}

/**
 * Training result
 */
export interface TrainingResult {
  /** Final loss value */
  finalLoss: number;

  /** Training accuracy */
  accuracy: number;

  /** Optimized parameters */
  parameters: number[];

  /** Loss history */
  lossHistory: number[];

  /** Accuracy history */
  accuracyHistory: number[];

  /** Gradient variance history (for barren plateau detection) */
  gradientVariance?: number[];

  /** Training time in milliseconds */
  trainingTime: number;

  /** Number of iterations */
  iterations: number;

  /** Convergence status */
  converged: boolean;
}

/**
 * Training callback interface
 */
export interface TrainingCallback {
  /** Called at the start of training */
  onTrainBegin?(params: { config: TrainingConfig }): void;

  /** Called at the end of training */
  onTrainEnd?(params: { result: TrainingResult }): void;

  /** Called at the start of each epoch */
  onEpochBegin?(params: { epoch: number }): void;

  /** Called at the end of each epoch */
  onEpochEnd?(params: { epoch: number; loss: number; accuracy?: number }): void;

  /** Called on gradient computation */
  onGradient?(params: { gradient: number[]; variance: number }): void;
}

// ============================================================================
// Quantum Circuits
// ============================================================================

/**
 * Quantum circuit representation
 */
export interface QuantumCircuit {
  /** Number of qubits */
  numQubits: number;

  /** Gates in the circuit */
  gates: QuantumGateOperation[];

  /** Parameters (if variational) */
  parameters?: number[];

  /** Circuit depth */
  depth: number;

  /** Get QASM representation */
  toQASM?(): string;

  /** Get circuit diagram */
  toString?(): string;
}

/**
 * Quantum gate operation
 */
export interface QuantumGateOperation {
  /** Gate type */
  gate: QuantumGate;

  /** Target qubit(s) */
  qubits: number[];

  /** Parameter (for parameterized gates) */
  parameter?: number | string;

  /** Control qubits (for controlled gates) */
  controls?: number[];
}

/**
 * Measurement configuration
 */
export interface MeasurementConfig {
  /** Qubits to measure */
  qubits: number[];

  /** Measurement basis */
  basis: PauliOperator[];

  /** Number of shots */
  shots: number;
}

/**
 * Measurement result
 */
export interface MeasurementResult {
  /** Measurement counts */
  counts: Record<string, number>;

  /** Expectation values */
  expectationValues: Record<string, number>;

  /** Standard deviations */
  stdDevs: Record<string, number>;

  /** Number of shots */
  shots: number;
}

// ============================================================================
// Feature Encoding
// ============================================================================

/**
 * Feature map configuration
 */
export interface FeatureMapConfig {
  /** Encoding strategy */
  encoding: EncodingStrategy;

  /** Number of qubits */
  numQubits: number;

  /** Number of repetitions */
  reps?: number;

  /** Entanglement pattern */
  entanglement?: EntanglementPattern;

  /** Custom feature map function */
  customMap?: (data: number[]) => QuantumCircuit;
}

/**
 * Encoded data representation
 */
export interface EncodedData {
  /** Original classical data */
  classicalData: number[];

  /** Quantum circuit for encoding */
  encodingCircuit: QuantumCircuit;

  /** Feature map used */
  featureMap: FeatureMapConfig;
}

// ============================================================================
// Barren Plateau Detection
// ============================================================================

/**
 * Barren plateau detection configuration
 */
export interface BarrenPlateauConfig {
  /** Threshold for gradient variance */
  varianceThreshold: number;

  /** Number of samples for variance estimation */
  numSamples?: number;

  /** Detection frequency (check every N epochs) */
  checkFrequency?: number;
}

/**
 * Barren plateau detection result
 */
export interface BarrenPlateauResult {
  /** Is barren plateau detected? */
  detected: boolean;

  /** Gradient variance */
  gradientVariance: number;

  /** Threshold used */
  threshold: number;

  /** Recommended actions */
  recommendations: string[];

  /** Severity level */
  severity: 'low' | 'medium' | 'high' | 'critical';
}

// ============================================================================
// Quantum Advantage Metrics
// ============================================================================

/**
 * Quantum advantage metrics
 */
export interface QuantumAdvantageMetrics {
  /** Classical baseline accuracy */
  classicalAccuracy: number;

  /** Quantum model accuracy */
  quantumAccuracy: number;

  /** Relative improvement */
  improvement: number;

  /** Statistical significance (p-value) */
  pValue?: number;

  /** Circuit resources used */
  resources: {
    numQubits: number;
    circuitDepth: number;
    gateCount: number;
    shots: number;
  };
}

// ============================================================================
// Error and Validation
// ============================================================================

/**
 * WIA-QUA-006 error codes
 */
export enum QuantumMLErrorCode {
  INVALID_QUBIT_COUNT = 'Q001',
  BARREN_PLATEAU = 'Q002',
  TRAINING_DIVERGENCE = 'Q003',
  MEASUREMENT_ERROR = 'Q004',
  INVALID_ENCODING = 'Q005',
  CIRCUIT_TOO_DEEP = 'Q006',
  OPTIMIZER_FAILURE = 'Q007',
  INVALID_DATA = 'Q008',
  DIMENSION_MISMATCH = 'Q009',
  RESOURCE_LIMIT = 'Q010',
}

/**
 * Quantum ML error
 */
export class QuantumMLError extends Error {
  constructor(
    public code: QuantumMLErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'QuantumMLError';
  }
}

/**
 * Validation result
 */
export interface ValidationResult {
  /** Is valid? */
  isValid: boolean;

  /** Validation errors */
  errors: string[];

  /** Warnings */
  warnings: string[];

  /** Recommendations */
  recommendations: string[];
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
 * Matrix type
 */
export type Matrix = number[][];

/**
 * Vector type
 */
export type Vector = number[];

/**
 * Quantum state vector (complex amplitudes)
 */
export type StateVector = Complex[];

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Quantum ML constants
 */
export const QUANTUM_ML_CONSTANTS = {
  /** Default number of measurement shots */
  DEFAULT_SHOTS: 1024,

  /** Parameter shift value for gradient computation */
  PARAMETER_SHIFT: Math.PI / 2,

  /** Default learning rate */
  DEFAULT_LEARNING_RATE: 0.01,

  /** Barren plateau variance threshold */
  BARREN_PLATEAU_THRESHOLD: 1e-6,

  /** Maximum recommended circuit depth */
  MAX_CIRCUIT_DEPTH: 100,

  /** Minimum required accuracy improvement */
  MIN_ACCURACY_IMPROVEMENT: 0.01,

  /** Default convergence tolerance */
  CONVERGENCE_TOLERANCE: 1e-4,

  /** Maximum training iterations */
  MAX_ITERATIONS: 1000,
} as const;

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Complex,
  QuantumGate,
  PauliOperator,
  EntanglementPattern,
  EncodingStrategy,
  OptimizerType,
  LossFunction,

  // QNN
  QNNConfig,
  QNNResult,
  QuantumNeuralNetwork,

  // VQC
  VQCConfig,
  VQCTrainingResult,
  VariationalQuantumClassifier,

  // Quantum Kernels
  QuantumKernelConfig,
  KernelMatrix,
  QSVMConfig,
  QuantumSVM,

  // Generative Models
  QGANConfig,
  QBMConfig,
  QCBMConfig,

  // Training
  OptimizerConfig,
  TrainingConfig,
  TrainingResult,
  TrainingCallback,

  // Circuits
  QuantumCircuit,
  QuantumGateOperation,
  MeasurementConfig,
  MeasurementResult,

  // Feature Encoding
  FeatureMapConfig,
  EncodedData,

  // Barren Plateau
  BarrenPlateauConfig,
  BarrenPlateauResult,

  // Metrics
  QuantumAdvantageMetrics,

  // Validation
  ValidationResult,

  // Utility
  Matrix,
  Vector,
  StateVector,
};

export { QUANTUM_ML_CONSTANTS, QuantumMLErrorCode, QuantumMLError };

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
