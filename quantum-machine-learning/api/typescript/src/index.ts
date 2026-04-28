/**
 * WIA-QUA-006: Quantum Machine Learning SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum ML Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for quantum machine learning including:
 * - Quantum Neural Networks (QNN)
 * - Variational Quantum Classifiers (VQC)
 * - Quantum Support Vector Machines (QSVM)
 * - Quantum Generative Models (QGAN, QBM)
 * - Quantum Feature Encoding
 * - Hybrid Quantum-Classical Optimization
 */

import {
  QNNConfig,
  QNNResult,
  QuantumNeuralNetwork,
  VQCConfig,
  VQCTrainingResult,
  VariationalQuantumClassifier,
  QuantumKernelConfig,
  KernelMatrix,
  QSVMConfig,
  QuantumSVM,
  TrainingConfig,
  TrainingResult,
  OptimizerConfig,
  QuantumCircuit,
  QuantumGateOperation,
  MeasurementResult,
  FeatureMapConfig,
  EncodedData,
  BarrenPlateauConfig,
  BarrenPlateauResult,
  QuantumAdvantageMetrics,
  ValidationResult,
  QUANTUM_ML_CONSTANTS,
  QuantumMLErrorCode,
  QuantumMLError,
  PauliOperator,
  Matrix,
  Vector,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-QUA-006 Quantum Machine Learning SDK
 */
export class QuantumMLSDK {
  private version = '1.0.0';
  private initialized = false;

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Create a Quantum Neural Network
   */
  createQNN(config: QNNConfig): QuantumNeuralNetwork {
    this.validateQNNConfig(config);

    const numParameters = this.calculateQNNParameters(config);
    const parameters = this.initializeParameters(numParameters);
    const circuitDepth = this.calculateCircuitDepth(config);

    return {
      config,
      parameters,
      numParameters,
      circuitDepth,
      predict: (input: number[]) => this.qnnPredict(config, parameters, input),
      getCircuit: (input?: number[]) => this.buildQNNCircuit(config, parameters, input),
    };
  }

  /**
   * Train a Quantum Neural Network
   */
  async trainQNN(
    qnn: QuantumNeuralNetwork,
    trainingConfig: TrainingConfig
  ): Promise<TrainingResult> {
    const startTime = Date.now();

    const { data, labels, epochs, optimizer, learningRate, lossFunction } = trainingConfig;

    // Validate training data
    this.validateTrainingData(data, labels, qnn.config.numQubits);

    const lossHistory: number[] = [];
    const accuracyHistory: number[] = [];
    const gradientVariance: number[] = [];

    let parameters = [...qnn.parameters];
    let bestLoss = Infinity;
    let bestParams = [...parameters];

    // Training loop
    for (let epoch = 0; epoch < epochs; epoch++) {
      // Compute predictions
      const predictions = data.map((x) => {
        const result = this.qnnPredict(qnn.config, parameters, x);
        return result.expectationValue;
      });

      // Compute loss
      const loss = this.computeLoss(predictions, labels, lossFunction);
      lossHistory.push(loss);

      // Compute accuracy
      const accuracy = this.computeAccuracy(predictions, labels);
      accuracyHistory.push(accuracy);

      // Compute gradients
      const gradients = this.computeGradients(qnn.config, parameters, data, labels, lossFunction);

      // Check for barren plateau
      const variance = this.computeVariance(gradients);
      gradientVariance.push(variance);

      if (variance < QUANTUM_ML_CONSTANTS.BARREN_PLATEAU_THRESHOLD) {
        console.warn(`Barren plateau detected at epoch ${epoch}. Gradient variance: ${variance}`);
      }

      // Update parameters
      parameters = this.updateParameters(parameters, gradients, learningRate, optimizer);

      // Track best parameters
      if (loss < bestLoss) {
        bestLoss = loss;
        bestParams = [...parameters];
      }

      // Early stopping check
      if (
        trainingConfig.earlyStoppingPatience &&
        epoch > trainingConfig.earlyStoppingPatience
      ) {
        const recentLosses = lossHistory.slice(-trainingConfig.earlyStoppingPatience);
        const improving = recentLosses.some((l, i) => i > 0 && l < recentLosses[i - 1]);
        if (!improving) {
          console.log(`Early stopping at epoch ${epoch}`);
          break;
        }
      }
    }

    // Update QNN with best parameters
    qnn.parameters = bestParams;

    const trainingTime = Date.now() - startTime;

    return {
      finalLoss: bestLoss,
      accuracy: accuracyHistory[accuracyHistory.length - 1],
      parameters: bestParams,
      lossHistory,
      accuracyHistory,
      gradientVariance,
      trainingTime,
      iterations: lossHistory.length,
      converged: bestLoss < QUANTUM_ML_CONSTANTS.CONVERGENCE_TOLERANCE,
    };
  }

  /**
   * Create a Variational Quantum Classifier
   */
  createVQC(config: VQCConfig): VariationalQuantumClassifier {
    this.validateVQCConfig(config);

    const numLayers = config.ansatzLayers || 2;
    const numParameters = this.calculateVQCParameters(config, numLayers);
    let parameters = this.initializeParameters(numParameters);

    const vqc: VariationalQuantumClassifier = {
      config,
      parameters,

      fit: async (X: number[][], y: number[]) => {
        const trainingConfig: TrainingConfig = {
          data: X,
          labels: y,
          epochs: config.optimizer?.maxIterations || 100,
          optimizer: config.optimizer?.type || 'adam',
          learningRate: config.optimizer?.learningRate || 0.05,
          lossFunction: 'cross-entropy',
        };

        const result = await this.trainVQC(vqc, trainingConfig);
        parameters = result.parameters;
        vqc.parameters = parameters;

        return {
          finalLoss: result.finalLoss,
          accuracy: result.accuracy,
          parameters: result.parameters,
          lossHistory: result.lossHistory,
          accuracyHistory: result.accuracyHistory,
          trainingTime: result.trainingTime,
          iterations: result.iterations,
        };
      },

      predict: (X: number[][]) => {
        return X.map((x) => {
          const probs = this.vqcPredictProba(config, parameters, x);
          return probs[0] > 0.5 ? 0 : 1;
        });
      },

      predictProba: (X: number[][]) => {
        return X.map((x) => this.vqcPredictProba(config, parameters, x));
      },

      score: (X: number[][], y: number[]) => {
        const predictions = vqc.predict(X);
        const correct = predictions.filter((p, i) => p === y[i]).length;
        return correct / y.length;
      },
    };

    return vqc;
  }

  /**
   * Compute quantum kernel matrix
   */
  computeQuantumKernel(
    config: QuantumKernelConfig,
    X_train: number[][],
    X_test?: number[][]
  ): KernelMatrix {
    const startTime = Date.now();

    const X2 = X_test || X_train;
    const n1 = X_train.length;
    const n2 = X2.length;

    const kernelMatrix: number[][] = Array(n1)
      .fill(0)
      .map(() => Array(n2).fill(0));

    // Compute kernel for all pairs
    for (let i = 0; i < n1; i++) {
      for (let j = 0; j < n2; j++) {
        kernelMatrix[i][j] = this.quantumKernelValue(config, X_train[i], X2[j]);
      }
    }

    const computationTime = Date.now() - startTime;

    return {
      values: kernelMatrix,
      trainingData: X_train,
      testData: X_test,
      computationTime,
    };
  }

  /**
   * Create Quantum SVM
   */
  createQSVM(config: QSVMConfig): QuantumSVM {
    let supportVectors: number[][] | undefined;
    let dualCoef: number[] | undefined;

    return {
      config,
      supportVectors,
      dualCoef,

      fit: async (X: number[][], y: number[]) => {
        // Compute kernel matrix
        const kernelMatrix = this.computeQuantumKernel(config.kernel, X);

        // Train classical SVM with quantum kernel
        const svmResult = this.trainClassicalSVM(kernelMatrix.values, y, config.C || 1.0);

        supportVectors = svmResult.supportVectors;
        dualCoef = svmResult.dualCoef;
      },

      predict: (X: number[][]) => {
        if (!supportVectors || !dualCoef) {
          throw new QuantumMLError(
            QuantumMLErrorCode.INVALID_DATA,
            'QSVM not trained. Call fit() first.'
          );
        }

        return X.map((x) => {
          const kernelValues = supportVectors!.map((sv) =>
            this.quantumKernelValue(config.kernel, x, sv)
          );

          const decision = kernelValues.reduce(
            (sum, k, i) => sum + dualCoef![i] * k,
            0
          );

          return decision > 0 ? 1 : -1;
        });
      },

      decisionFunction: (X: number[][]) => {
        if (!supportVectors || !dualCoef) {
          throw new QuantumMLError(
            QuantumMLErrorCode.INVALID_DATA,
            'QSVM not trained. Call fit() first.'
          );
        }

        return X.map((x) => {
          const kernelValues = supportVectors!.map((sv) =>
            this.quantumKernelValue(config.kernel, x, sv)
          );

          return kernelValues.reduce((sum, k, i) => sum + dualCoef![i] * k, 0);
        });
      },
    };
  }

  /**
   * Encode classical data into quantum feature map
   */
  encodeData(config: FeatureMapConfig, data: number[]): EncodedData {
    const circuit = this.buildFeatureMap(config, data);

    return {
      classicalData: data,
      encodingCircuit: circuit,
      featureMap: config,
    };
  }

  /**
   * Detect barren plateau
   */
  detectBarrenPlateau(
    gradients: number[],
    config?: BarrenPlateauConfig
  ): BarrenPlateauResult {
    const threshold = config?.varianceThreshold || QUANTUM_ML_CONSTANTS.BARREN_PLATEAU_THRESHOLD;
    const variance = this.computeVariance(gradients);

    const detected = variance < threshold;

    const recommendations: string[] = [];
    let severity: 'low' | 'medium' | 'high' | 'critical' = 'low';

    if (detected) {
      if (variance < threshold * 0.1) {
        severity = 'critical';
        recommendations.push('Use layer-wise training');
        recommendations.push('Reduce circuit depth significantly');
        recommendations.push('Consider different ansatz architecture');
      } else if (variance < threshold * 0.5) {
        severity = 'high';
        recommendations.push('Use local cost functions');
        recommendations.push('Apply correlated parameter initialization');
      } else {
        severity = 'medium';
        recommendations.push('Monitor gradient variance closely');
        recommendations.push('Consider parameter sharing');
      }
    }

    return {
      detected,
      gradientVariance: variance,
      threshold,
      recommendations,
      severity,
    };
  }

  /**
   * Evaluate quantum advantage
   */
  evaluateQuantumAdvantage(
    quantumModel: VariationalQuantumClassifier,
    classicalAccuracy: number,
    X_test: number[][],
    y_test: number[]
  ): QuantumAdvantageMetrics {
    const quantumAccuracy = quantumModel.score(X_test, y_test);
    const improvement = quantumAccuracy - classicalAccuracy;

    return {
      classicalAccuracy,
      quantumAccuracy,
      improvement,
      resources: {
        numQubits: quantumModel.config.numQubits,
        circuitDepth: this.calculateVQCCircuitDepth(quantumModel.config),
        gateCount: this.estimateGateCount(quantumModel.config),
        shots: quantumModel.config.shots || QUANTUM_ML_CONSTANTS.DEFAULT_SHOTS,
      },
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Validate QNN configuration
   */
  private validateQNNConfig(config: QNNConfig): void {
    if (config.numQubits < 1) {
      throw new QuantumMLError(
        QuantumMLErrorCode.INVALID_QUBIT_COUNT,
        'Number of qubits must be at least 1'
      );
    }

    if (config.numLayers < 1) {
      throw new QuantumMLError(
        QuantumMLErrorCode.INVALID_DATA,
        'Number of layers must be at least 1'
      );
    }

    const depth = config.numLayers * (config.rotationGates.length + 1);
    if (depth > QUANTUM_ML_CONSTANTS.MAX_CIRCUIT_DEPTH) {
      throw new QuantumMLError(
        QuantumMLErrorCode.CIRCUIT_TOO_DEEP,
        `Circuit depth (${depth}) exceeds maximum (${QUANTUM_ML_CONSTANTS.MAX_CIRCUIT_DEPTH})`
      );
    }
  }

  /**
   * Calculate number of parameters in QNN
   */
  private calculateQNNParameters(config: QNNConfig): number {
    return config.numQubits * config.numLayers * config.rotationGates.length;
  }

  /**
   * Calculate circuit depth
   */
  private calculateCircuitDepth(config: QNNConfig): number {
    return config.numLayers * (config.rotationGates.length + 1); // +1 for entanglement
  }

  /**
   * Initialize parameters randomly
   */
  private initializeParameters(count: number): number[] {
    return Array(count)
      .fill(0)
      .map(() => Math.random() * 2 * Math.PI);
  }

  /**
   * Build QNN circuit
   */
  private buildQNNCircuit(
    config: QNNConfig,
    parameters: number[],
    input?: number[]
  ): QuantumCircuit {
    const gates: QuantumGateOperation[] = [];
    let paramIndex = 0;

    // Input encoding (if provided)
    if (input) {
      for (let i = 0; i < Math.min(input.length, config.numQubits); i++) {
        gates.push({
          gate: 'RY',
          qubits: [i],
          parameter: input[i],
        });
      }
    }

    // Variational layers
    for (let layer = 0; layer < config.numLayers; layer++) {
      // Rotation gates
      for (const gateType of config.rotationGates) {
        for (let q = 0; q < config.numQubits; q++) {
          gates.push({
            gate: gateType,
            qubits: [q],
            parameter: parameters[paramIndex++],
          });
        }
      }

      // Entanglement
      const entanglementGates = this.getEntanglementGates(
        config.numQubits,
        config.entanglementPattern
      );
      gates.push(...entanglementGates);
    }

    return {
      numQubits: config.numQubits,
      gates,
      parameters,
      depth: this.calculateCircuitDepth(config),
    };
  }

  /**
   * Get entanglement gates
   */
  private getEntanglementGates(
    numQubits: number,
    pattern: string
  ): QuantumGateOperation[] {
    const gates: QuantumGateOperation[] = [];

    switch (pattern) {
      case 'linear':
        for (let i = 0; i < numQubits - 1; i++) {
          gates.push({ gate: 'CNOT', qubits: [i, i + 1] });
        }
        break;

      case 'full':
        for (let i = 0; i < numQubits; i++) {
          for (let j = i + 1; j < numQubits; j++) {
            gates.push({ gate: 'CNOT', qubits: [i, j] });
          }
        }
        break;

      case 'circular':
        for (let i = 0; i < numQubits; i++) {
          gates.push({ gate: 'CNOT', qubits: [i, (i + 1) % numQubits] });
        }
        break;
    }

    return gates;
  }

  /**
   * QNN prediction
   */
  private qnnPredict(config: QNNConfig, parameters: number[], input: number[]): QNNResult {
    const circuit = this.buildQNNCircuit(config, parameters, input);
    const measurement = this.simulateMeasurement(circuit, config.measurements, config.shots);

    const expectationValue = measurement.expectationValues['Z'] || 0;
    const variance = measurement.stdDevs['Z'] || 0;

    return {
      prediction: expectationValue,
      expectationValue,
      variance: variance * variance,
      shots: measurement.shots,
      counts: measurement.counts,
    };
  }

  /**
   * Simulate quantum measurement
   */
  private simulateMeasurement(
    circuit: QuantumCircuit,
    measurements: PauliOperator[],
    shots = QUANTUM_ML_CONSTANTS.DEFAULT_SHOTS
  ): MeasurementResult {
    // Simplified simulation: return mock results
    // In real implementation, this would use actual quantum simulator

    const counts: Record<string, number> = {
      '0'.repeat(circuit.numQubits): Math.floor(shots * 0.6),
      '1'.repeat(circuit.numQubits): Math.floor(shots * 0.4),
    };

    const expectationValues: Record<string, number> = {};
    const stdDevs: Record<string, number> = {};

    measurements.forEach((op) => {
      expectationValues[op] = Math.random() * 2 - 1; // Random value in [-1, 1]
      stdDevs[op] = 0.1;
    });

    return {
      counts,
      expectationValues,
      stdDevs,
      shots,
    };
  }

  /**
   * Validate training data
   */
  private validateTrainingData(data: number[][], labels: number[], numQubits: number): void {
    if (data.length !== labels.length) {
      throw new QuantumMLError(
        QuantumMLErrorCode.DIMENSION_MISMATCH,
        'Data and labels must have same length'
      );
    }

    if (data.length === 0) {
      throw new QuantumMLError(QuantumMLErrorCode.INVALID_DATA, 'Training data is empty');
    }
  }

  /**
   * Compute loss
   */
  private computeLoss(predictions: number[], labels: number[], lossFunction: string): number {
    switch (lossFunction) {
      case 'mse':
        return (
          predictions.reduce((sum, pred, i) => sum + Math.pow(pred - labels[i], 2), 0) /
          predictions.length
        );

      case 'cross-entropy':
        return (
          -predictions.reduce((sum, pred, i) => {
            const p = 1 / (1 + Math.exp(-pred)); // Sigmoid
            const y = labels[i];
            return sum + y * Math.log(p + 1e-10) + (1 - y) * Math.log(1 - p + 1e-10);
          }, 0) / predictions.length
        );

      default:
        return (
          predictions.reduce((sum, pred, i) => sum + Math.abs(pred - labels[i]), 0) /
          predictions.length
        );
    }
  }

  /**
   * Compute accuracy
   */
  private computeAccuracy(predictions: number[], labels: number[]): number {
    const correct = predictions.filter((pred, i) => {
      const predicted = pred > 0 ? 1 : 0;
      return predicted === labels[i];
    }).length;

    return correct / predictions.length;
  }

  /**
   * Compute gradients using parameter shift rule
   */
  private computeGradients(
    config: QNNConfig,
    parameters: number[],
    data: number[][],
    labels: number[],
    lossFunction: string
  ): number[] {
    const gradients: number[] = [];
    const shift = QUANTUM_ML_CONSTANTS.PARAMETER_SHIFT;

    for (let i = 0; i < parameters.length; i++) {
      // Forward shift
      const paramsPlus = [...parameters];
      paramsPlus[i] += shift;
      const predictionsPlus = data.map((x) => this.qnnPredict(config, paramsPlus, x).expectationValue);
      const lossPlus = this.computeLoss(predictionsPlus, labels, lossFunction);

      // Backward shift
      const paramsMinus = [...parameters];
      paramsMinus[i] -= shift;
      const predictionsMinus = data.map((x) => this.qnnPredict(config, paramsMinus, x).expectationValue);
      const lossMinus = this.computeLoss(predictionsMinus, labels, lossFunction);

      // Gradient
      gradients[i] = (lossPlus - lossMinus) / (2 * shift);
    }

    return gradients;
  }

  /**
   * Compute variance
   */
  private computeVariance(values: number[]): number {
    const mean = values.reduce((sum, v) => sum + v, 0) / values.length;
    const variance =
      values.reduce((sum, v) => sum + Math.pow(v - mean, 2), 0) / values.length;
    return variance;
  }

  /**
   * Update parameters
   */
  private updateParameters(
    parameters: number[],
    gradients: number[],
    learningRate: number,
    optimizer: string
  ): number[] {
    // Simple gradient descent
    return parameters.map((p, i) => p - learningRate * gradients[i]);
  }

  /**
   * Validate VQC configuration
   */
  private validateVQCConfig(config: VQCConfig): void {
    if (config.numQubits < 1) {
      throw new QuantumMLError(
        QuantumMLErrorCode.INVALID_QUBIT_COUNT,
        'Number of qubits must be at least 1'
      );
    }
  }

  /**
   * Calculate VQC parameters
   */
  private calculateVQCParameters(config: VQCConfig, numLayers: number): number {
    return config.numQubits * numLayers * 3; // 3 rotation angles per qubit per layer
  }

  /**
   * Calculate VQC circuit depth
   */
  private calculateVQCCircuitDepth(config: VQCConfig): number {
    const numLayers = config.ansatzLayers || 2;
    return numLayers * 4; // Approximate
  }

  /**
   * Estimate gate count
   */
  private estimateGateCount(config: VQCConfig): number {
    const numLayers = config.ansatzLayers || 2;
    return config.numQubits * numLayers * 5; // Approximate
  }

  /**
   * Train VQC
   */
  private async trainVQC(
    vqc: VariationalQuantumClassifier,
    trainingConfig: TrainingConfig
  ): Promise<TrainingResult> {
    // Use QNN training infrastructure
    const qnnConfig: QNNConfig = {
      numQubits: vqc.config.numQubits,
      numLayers: vqc.config.ansatzLayers || 2,
      entanglementPattern: 'linear',
      rotationGates: ['RY', 'RZ'],
      measurements: vqc.config.measurements,
      shots: vqc.config.shots,
    };

    const qnn = this.createQNN(qnnConfig);
    qnn.parameters = vqc.parameters;

    return this.trainQNN(qnn, trainingConfig);
  }

  /**
   * VQC predict probability
   */
  private vqcPredictProba(config: VQCConfig, parameters: number[], input: number[]): number[] {
    // Build circuit and measure
    const expectation = Math.random(); // Simplified
    const prob1 = (expectation + 1) / 2; // Map [-1, 1] to [0, 1]
    return [prob1, 1 - prob1];
  }

  /**
   * Quantum kernel value
   */
  private quantumKernelValue(
    config: QuantumKernelConfig,
    x1: number[],
    x2: number[]
  ): number {
    // Build feature map circuits
    const circuit1 = this.buildFeatureMap(
      { encoding: config.featureMap, numQubits: config.numQubits, reps: config.reps },
      x1
    );

    const circuit2 = this.buildFeatureMap(
      { encoding: config.featureMap, numQubits: config.numQubits, reps: config.reps },
      x2
    );

    // Simplified kernel computation
    const similarity = this.computeStateSimilarity(x1, x2);
    return similarity;
  }

  /**
   * Compute state similarity
   */
  private computeStateSimilarity(x1: number[], x2: number[]): number {
    // Simplified: use classical similarity
    const dotProduct = x1.reduce((sum, val, i) => sum + val * x2[i], 0);
    const norm1 = Math.sqrt(x1.reduce((sum, val) => sum + val * val, 0));
    const norm2 = Math.sqrt(x2.reduce((sum, val) => sum + val * val, 0));

    return Math.abs(dotProduct / (norm1 * norm2 + 1e-10));
  }

  /**
   * Build feature map
   */
  private buildFeatureMap(config: FeatureMapConfig, data: number[]): QuantumCircuit {
    const gates: QuantumGateOperation[] = [];
    const reps = config.reps || 1;

    for (let rep = 0; rep < reps; rep++) {
      switch (config.encoding) {
        case 'angle':
          for (let i = 0; i < Math.min(data.length, config.numQubits); i++) {
            gates.push({ gate: 'RY', qubits: [i], parameter: data[i] });
          }
          break;

        case 'amplitude':
          // Simplified amplitude encoding
          for (let i = 0; i < Math.min(data.length, config.numQubits); i++) {
            gates.push({ gate: 'RY', qubits: [i], parameter: Math.asin(data[i]) });
          }
          break;

        case 'basis':
          // Binary encoding
          for (let i = 0; i < Math.min(data.length, config.numQubits); i++) {
            if (data[i] > 0.5) {
              gates.push({ gate: 'X', qubits: [i] });
            }
          }
          break;
      }

      // Add entanglement if specified
      if (config.entanglement) {
        gates.push(...this.getEntanglementGates(config.numQubits, config.entanglement));
      }
    }

    return {
      numQubits: config.numQubits,
      gates,
      depth: gates.length,
    };
  }

  /**
   * Train classical SVM with kernel matrix
   */
  private trainClassicalSVM(
    kernelMatrix: number[][],
    labels: number[],
    C: number
  ): { supportVectors: number[][]; dualCoef: number[] } {
    // Simplified SVM training
    // In real implementation, use optimization library

    const n = labels.length;
    const supportVectors: number[][] = [];
    const dualCoef: number[] = [];

    // Simplified: use all points as support vectors
    for (let i = 0; i < n; i++) {
      supportVectors.push(Array(kernelMatrix[i].length).fill(0).map((_, j) => kernelMatrix[i][j]));
      dualCoef.push(labels[i] * C / n);
    }

    return { supportVectors, dualCoef };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create QNN (standalone function)
 */
export function createQNN(config: QNNConfig): QuantumNeuralNetwork {
  const sdk = new QuantumMLSDK();
  return sdk.createQNN(config);
}

/**
 * Train QNN (standalone function)
 */
export async function trainQNN(
  qnn: QuantumNeuralNetwork,
  config: TrainingConfig
): Promise<TrainingResult> {
  const sdk = new QuantumMLSDK();
  return sdk.trainQNN(qnn, config);
}

/**
 * Create VQC (standalone function)
 */
export function createVQC(config: VQCConfig): VariationalQuantumClassifier {
  const sdk = new QuantumMLSDK();
  return sdk.createVQC(config);
}

/**
 * Compute quantum kernel (standalone function)
 */
export function quantumKernel(
  config: QuantumKernelConfig,
  X_train: number[][],
  X_test?: number[][]
): KernelMatrix {
  const sdk = new QuantumMLSDK();
  return sdk.computeQuantumKernel(config, X_train, X_test);
}

/**
 * Angle encoding feature map
 */
export function angleEncoding(data: number[], numQubits: number): FeatureMapConfig {
  return {
    encoding: 'angle',
    numQubits,
    reps: 1,
  };
}

/**
 * Amplitude encoding feature map
 */
export function amplitudeEncoding(data: number[], numQubits: number): FeatureMapConfig {
  return {
    encoding: 'amplitude',
    numQubits,
    reps: 1,
  };
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { QuantumMLSDK };
export default QuantumMLSDK;

/**
 * 弘익人間 (홍익인간) · Benefit All Humanity
 */
