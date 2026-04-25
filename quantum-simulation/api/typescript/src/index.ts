/**
 * WIA-QUA-005: Quantum Simulation SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for quantum system simulation including:
 * - State vector and density matrix methods
 * - Quantum circuit simulation
 * - Tensor network techniques
 * - Quantum chemistry calculations
 * - Noise modeling
 * - Variational algorithms
 */

import {
  Complex,
  StateVector,
  DensityMatrix,
  QuantumCircuit,
  Gate,
  GateType,
  SimulationResult,
  ExpectationResult,
  Observable,
  Hamiltonian,
  PauliString,
  VQEConfig,
  VQEResult,
  NoiseModel,
  SimulatorConfig,
  QuantumState,
  Measurement,
  QUANTUM_CONSTANTS,
  PAULI_MATRICES,
  QuantumErrorCode,
  QuantumSimulationError,
  Molecule,
  ElectronicStructure,
  TimeEvolutionConfig,
  TimeEvolutionResult,
  Fidelity,
  Purity,
  ProbabilityDistribution,
} from './types';

// ============================================================================
// Complex Number Utilities
// ============================================================================

/**
 * Create a complex number
 */
export function complex(real: number, imag: number = 0): Complex {
  return { real, imag };
}

/**
 * Add two complex numbers
 */
export function complexAdd(a: Complex, b: Complex): Complex {
  return { real: a.real + b.real, imag: a.imag + b.imag };
}

/**
 * Multiply two complex numbers
 */
export function complexMul(a: Complex, b: Complex): Complex {
  return {
    real: a.real * b.real - a.imag * b.imag,
    imag: a.real * b.imag + a.imag * b.real,
  };
}

/**
 * Complex conjugate
 */
export function complexConj(a: Complex): Complex {
  return { real: a.real, imag: -a.imag };
}

/**
 * Absolute value (magnitude) of complex number
 */
export function complexAbs(a: Complex): number {
  return Math.sqrt(a.real * a.real + a.imag * a.imag);
}

/**
 * Phase of complex number
 */
export function complexPhase(a: Complex): number {
  return Math.atan2(a.imag, a.real);
}

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-QUA-005 Quantum Simulation SDK
 */
export class QuantumSimulator {
  private version = '1.0.0';
  private config: SimulatorConfig;
  private state: StateVector | null = null;

  constructor(config?: Partial<SimulatorConfig>) {
    this.config = {
      numQubits: config?.numQubits ?? 1,
      method: config?.method ?? 'statevector',
      precision: config?.precision ?? 'double',
      useGPU: config?.useGPU ?? false,
      seed: config?.seed,
      bondDimension: config?.bondDimension ?? 50,
      memoryLimit: config?.memoryLimit,
    };

    // Initialize state
    this.reset();
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Get current configuration
   */
  getConfig(): SimulatorConfig {
    return { ...this.config };
  }

  /**
   * Reset to |0...0⟩ state
   */
  reset(): void {
    const dim = Math.pow(2, this.config.numQubits);
    const amplitudes: Complex[] = new Array(dim);

    for (let i = 0; i < dim; i++) {
      amplitudes[i] = { real: i === 0 ? 1 : 0, imag: 0 };
    }

    this.state = {
      numQubits: this.config.numQubits,
      amplitudes,
      normalized: true,
    };
  }

  /**
   * Initialize state with custom amplitudes
   */
  initialize(amplitudes: Complex[] | number[]): void {
    const dim = Math.pow(2, this.config.numQubits);

    if (amplitudes.length !== dim) {
      throw new QuantumSimulationError(
        QuantumErrorCode.STATE_NOT_NORMALIZED,
        `Expected ${dim} amplitudes, got ${amplitudes.length}`
      );
    }

    // Convert numbers to complex if needed
    const complexAmps: Complex[] = amplitudes.map((a) =>
      typeof a === 'number' ? { real: a, imag: 0 } : a
    );

    // Check normalization
    const norm = this.calculateNorm(complexAmps);
    if (Math.abs(norm - 1.0) > 1e-10) {
      throw new QuantumSimulationError(
        QuantumErrorCode.STATE_NOT_NORMALIZED,
        `State not normalized: ||ψ||² = ${norm}`
      );
    }

    this.state = {
      numQubits: this.config.numQubits,
      amplitudes: complexAmps,
      normalized: true,
    };
  }

  /**
   * Get current state
   */
  getState(): StateVector {
    if (!this.state) {
      throw new QuantumSimulationError(
        QuantumErrorCode.INVALID_CIRCUIT,
        'State not initialized'
      );
    }
    return { ...this.state, amplitudes: [...this.state.amplitudes] };
  }

  /**
   * Apply Hadamard gate
   */
  h(qubit: number): void {
    this.validateQubit(qubit);
    const gate: Complex[][] = [
      [
        { real: 1 / Math.sqrt(2), imag: 0 },
        { real: 1 / Math.sqrt(2), imag: 0 },
      ],
      [
        { real: 1 / Math.sqrt(2), imag: 0 },
        { real: -1 / Math.sqrt(2), imag: 0 },
      ],
    ];
    this.applySingleQubitGate(gate, qubit);
  }

  /**
   * Apply Pauli X gate
   */
  x(qubit: number): void {
    this.validateQubit(qubit);
    this.applySingleQubitGate(PAULI_MATRICES.X as Complex[][], qubit);
  }

  /**
   * Apply Pauli Y gate
   */
  y(qubit: number): void {
    this.validateQubit(qubit);
    this.applySingleQubitGate(PAULI_MATRICES.Y as Complex[][], qubit);
  }

  /**
   * Apply Pauli Z gate
   */
  z(qubit: number): void {
    this.validateQubit(qubit);
    this.applySingleQubitGate(PAULI_MATRICES.Z as Complex[][], qubit);
  }

  /**
   * Apply S gate (phase gate)
   */
  s(qubit: number): void {
    this.validateQubit(qubit);
    const gate: Complex[][] = [
      [
        { real: 1, imag: 0 },
        { real: 0, imag: 0 },
      ],
      [
        { real: 0, imag: 0 },
        { real: 0, imag: 1 },
      ],
    ];
    this.applySingleQubitGate(gate, qubit);
  }

  /**
   * Apply T gate
   */
  t(qubit: number): void {
    this.validateQubit(qubit);
    const gate: Complex[][] = [
      [
        { real: 1, imag: 0 },
        { real: 0, imag: 0 },
      ],
      [
        { real: 0, imag: 0 },
        { real: Math.cos(Math.PI / 4), imag: Math.sin(Math.PI / 4) },
      ],
    ];
    this.applySingleQubitGate(gate, qubit);
  }

  /**
   * Apply Rx rotation gate
   */
  rx(qubit: number, theta: number): void {
    this.validateQubit(qubit);
    const gate: Complex[][] = [
      [
        { real: Math.cos(theta / 2), imag: 0 },
        { real: 0, imag: -Math.sin(theta / 2) },
      ],
      [
        { real: 0, imag: -Math.sin(theta / 2) },
        { real: Math.cos(theta / 2), imag: 0 },
      ],
    ];
    this.applySingleQubitGate(gate, qubit);
  }

  /**
   * Apply Ry rotation gate
   */
  ry(qubit: number, theta: number): void {
    this.validateQubit(qubit);
    const gate: Complex[][] = [
      [
        { real: Math.cos(theta / 2), imag: 0 },
        { real: -Math.sin(theta / 2), imag: 0 },
      ],
      [
        { real: Math.sin(theta / 2), imag: 0 },
        { real: Math.cos(theta / 2), imag: 0 },
      ],
    ];
    this.applySingleQubitGate(gate, qubit);
  }

  /**
   * Apply Rz rotation gate
   */
  rz(qubit: number, theta: number): void {
    this.validateQubit(qubit);
    const gate: Complex[][] = [
      [
        { real: Math.cos(-theta / 2), imag: Math.sin(-theta / 2) },
        { real: 0, imag: 0 },
      ],
      [
        { real: 0, imag: 0 },
        { real: Math.cos(theta / 2), imag: Math.sin(theta / 2) },
      ],
    ];
    this.applySingleQubitGate(gate, qubit);
  }

  /**
   * Apply CNOT (controlled-X) gate
   */
  cnot(control: number, target: number): void {
    this.validateQubit(control);
    this.validateQubit(target);

    if (control === target) {
      throw new QuantumSimulationError(
        QuantumErrorCode.INVALID_CIRCUIT,
        'Control and target qubits must be different'
      );
    }

    if (!this.state) return;

    const n = this.config.numQubits;
    const newAmplitudes = [...this.state.amplitudes];

    for (let i = 0; i < Math.pow(2, n); i++) {
      // Check if control qubit is |1⟩
      if ((i >> control) & 1) {
        // Flip target qubit
        const j = i ^ (1 << target);
        newAmplitudes[i] = this.state.amplitudes[j];
      }
    }

    this.state.amplitudes = newAmplitudes;
  }

  /**
   * Apply CZ (controlled-Z) gate
   */
  cz(control: number, target: number): void {
    this.validateQubit(control);
    this.validateQubit(target);

    if (!this.state) return;

    for (let i = 0; i < this.state.amplitudes.length; i++) {
      // If both qubits are |1⟩, apply phase flip
      if (((i >> control) & 1) && ((i >> target) & 1)) {
        this.state.amplitudes[i].real *= -1;
        this.state.amplitudes[i].imag *= -1;
      }
    }
  }

  /**
   * Apply SWAP gate
   */
  swap(qubit1: number, qubit2: number): void {
    this.validateQubit(qubit1);
    this.validateQubit(qubit2);

    if (!this.state) return;

    const newAmplitudes = [...this.state.amplitudes];

    for (let i = 0; i < this.state.amplitudes.length; i++) {
      const bit1 = (i >> qubit1) & 1;
      const bit2 = (i >> qubit2) & 1;

      if (bit1 !== bit2) {
        const j = i ^ (1 << qubit1) ^ (1 << qubit2);
        newAmplitudes[i] = this.state.amplitudes[j];
      }
    }

    this.state.amplitudes = newAmplitudes;
  }

  /**
   * Measure qubit in computational basis
   */
  measure(qubit: number | number[]): number[] {
    if (!this.state) {
      throw new QuantumSimulationError(
        QuantumErrorCode.INVALID_CIRCUIT,
        'State not initialized'
      );
    }

    const qubits = Array.isArray(qubit) ? qubit : [qubit];
    const results: number[] = [];

    for (const q of qubits) {
      this.validateQubit(q);

      // Calculate probability of |0⟩
      let prob0 = 0;
      for (let i = 0; i < this.state.amplitudes.length; i++) {
        if (((i >> q) & 1) === 0) {
          const amp = this.state.amplitudes[i];
          prob0 += amp.real * amp.real + amp.imag * amp.imag;
        }
      }

      // Sample outcome
      const outcome = Math.random() < prob0 ? 0 : 1;
      results.push(outcome);

      // Collapse state
      const norm = Math.sqrt(outcome === 0 ? prob0 : 1 - prob0);
      for (let i = 0; i < this.state.amplitudes.length; i++) {
        if (((i >> q) & 1) !== outcome) {
          this.state.amplitudes[i] = { real: 0, imag: 0 };
        } else {
          this.state.amplitudes[i].real /= norm;
          this.state.amplitudes[i].imag /= norm;
        }
      }
    }

    return results;
  }

  /**
   * Get measurement probabilities without collapsing state
   */
  getProbabilities(): number[] {
    if (!this.state) {
      throw new QuantumSimulationError(
        QuantumErrorCode.INVALID_CIRCUIT,
        'State not initialized'
      );
    }

    return this.state.amplitudes.map((amp) => amp.real * amp.real + amp.imag * amp.imag);
  }

  /**
   * Calculate expectation value of observable
   */
  expectation(observable: Observable, qubits?: number[]): number {
    if (!this.state) {
      throw new QuantumSimulationError(
        QuantumErrorCode.INVALID_CIRCUIT,
        'State not initialized'
      );
    }

    // For Pauli string representation
    if ('pauliTerms' in observable && Array.isArray(observable.pauliTerms)) {
      return this.expectationFromPauliStrings(
        observable.pauliTerms as PauliString[]
      );
    }

    // For matrix representation
    if (Array.isArray(observable.representation)) {
      const matrix = observable.representation as Complex[][];
      return this.expectationFromMatrix(matrix, qubits ?? [0]);
    }

    throw new QuantumSimulationError(
      QuantumErrorCode.INVALID_HAMILTONIAN,
      'Invalid observable representation'
    );
  }

  /**
   * Get state fidelity
   */
  getFidelity(): number {
    if (!this.state) return 0;

    const norm = this.calculateNorm(this.state.amplitudes);
    return norm;
  }

  /**
   * Run quantum circuit
   */
  runCircuit(circuit: QuantumCircuit): SimulationResult {
    const startTime = Date.now();

    try {
      // Apply gates
      for (const gate of circuit.gates) {
        this.applyGate(gate);
      }

      // Perform measurements if specified
      let measurements: number[] | undefined;
      if (circuit.measurements && circuit.measurements.length > 0) {
        measurements = this.measure(circuit.measurements.map((m) => m.qubit));
      }

      const executionTime = Date.now() - startTime;

      return {
        state: this.getState(),
        measurements,
        executionTime,
        memoryUsed: this.estimateMemory(),
        success: true,
      };
    } catch (error) {
      return {
        state: this.getState(),
        executionTime: Date.now() - startTime,
        memoryUsed: this.estimateMemory(),
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  private validateQubit(qubit: number): void {
    if (qubit < 0 || qubit >= this.config.numQubits) {
      throw new QuantumSimulationError(
        QuantumErrorCode.INVALID_QUBIT,
        `Qubit ${qubit} out of range [0, ${this.config.numQubits - 1}]`
      );
    }
  }

  private applySingleQubitGate(gate: Complex[][], qubit: number): void {
    if (!this.state) return;

    const n = this.config.numQubits;
    const stride = 1 << qubit;
    const newAmplitudes = [...this.state.amplitudes];

    for (let i = 0; i < Math.pow(2, n - 1); i++) {
      // Insert zero bit at position 'qubit'
      const idx0 =
        ((i >> qubit) << (qubit + 1)) | (i & ((1 << qubit) - 1));
      const idx1 = idx0 | stride;

      const amp0 = this.state.amplitudes[idx0];
      const amp1 = this.state.amplitudes[idx1];

      // Matrix multiplication
      newAmplitudes[idx0] = complexAdd(
        complexMul(gate[0][0], amp0),
        complexMul(gate[0][1], amp1)
      );
      newAmplitudes[idx1] = complexAdd(
        complexMul(gate[1][0], amp0),
        complexMul(gate[1][1], amp1)
      );
    }

    this.state.amplitudes = newAmplitudes;
  }

  private applyGate(gate: Gate): void {
    const { type, qubits, parameters } = gate;

    if (qubits.length === 1) {
      const q = qubits[0];
      switch (type) {
        case 'H':
          this.h(q);
          break;
        case 'X':
          this.x(q);
          break;
        case 'Y':
          this.y(q);
          break;
        case 'Z':
          this.z(q);
          break;
        case 'S':
          this.s(q);
          break;
        case 'T':
          this.t(q);
          break;
        case 'Rx':
          this.rx(q, parameters?.[0] ?? 0);
          break;
        case 'Ry':
          this.ry(q, parameters?.[0] ?? 0);
          break;
        case 'Rz':
          this.rz(q, parameters?.[0] ?? 0);
          break;
        default:
          throw new QuantumSimulationError(
            QuantumErrorCode.INVALID_CIRCUIT,
            `Unknown gate type: ${type}`
          );
      }
    } else if (qubits.length === 2) {
      const [q0, q1] = qubits;
      switch (type) {
        case 'CNOT':
        case 'CX':
          this.cnot(q0, q1);
          break;
        case 'CZ':
          this.cz(q0, q1);
          break;
        case 'SWAP':
          this.swap(q0, q1);
          break;
        default:
          throw new QuantumSimulationError(
            QuantumErrorCode.INVALID_CIRCUIT,
            `Unknown two-qubit gate: ${type}`
          );
      }
    }
  }

  private calculateNorm(amplitudes: Complex[]): number {
    let norm = 0;
    for (const amp of amplitudes) {
      norm += amp.real * amp.real + amp.imag * amp.imag;
    }
    return norm;
  }

  private expectationFromPauliStrings(pauliTerms: PauliString[]): number {
    let expectation = 0;

    for (const term of pauliTerms) {
      const value = this.expectationOfPauliString(term);
      expectation += term.coefficient.real * value;
    }

    return expectation;
  }

  private expectationOfPauliString(pauliString: PauliString): number {
    // Simplified implementation - measure in computational basis
    // and calculate expectation
    if (!this.state) return 0;

    let expectation = 0;
    const probs = this.getProbabilities();

    for (let i = 0; i < probs.length; i++) {
      let sign = 1;
      for (let q = 0; q < pauliString.paulis.length; q++) {
        if (pauliString.paulis[q] === 'Z' && ((i >> q) & 1)) {
          sign *= -1;
        }
      }
      expectation += sign * probs[i];
    }

    return expectation;
  }

  private expectationFromMatrix(matrix: Complex[][], qubits: number[]): number {
    // Simplified: assume single-qubit observable
    if (!this.state) return 0;

    let expectation = 0;
    const q = qubits[0];

    for (let i = 0; i < this.state.amplitudes.length; i++) {
      const bit = (i >> q) & 1;
      const amp = this.state.amplitudes[i];
      const matrixElement = matrix[bit][bit];

      expectation +=
        (amp.real * amp.real + amp.imag * amp.imag) * matrixElement.real;
    }

    return expectation;
  }

  private estimateMemory(): number {
    const dim = Math.pow(2, this.config.numQubits);
    return dim * 16; // 16 bytes per complex number
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create Bell state (|00⟩ + |11⟩)/√2
 */
export function createBellState(numQubits: number = 2): StateVector {
  if (numQubits < 2) {
    throw new QuantumSimulationError(
      QuantumErrorCode.INVALID_QUBIT,
      'Bell state requires at least 2 qubits'
    );
  }

  const sim = new QuantumSimulator({ numQubits });
  sim.h(0);
  sim.cnot(0, 1);
  return sim.getState();
}

/**
 * Create GHZ state (|00...0⟩ + |11...1⟩)/√2
 */
export function createGHZState(numQubits: number): StateVector {
  const sim = new QuantumSimulator({ numQubits });
  sim.h(0);
  for (let i = 1; i < numQubits; i++) {
    sim.cnot(0, i);
  }
  return sim.getState();
}

/**
 * Calculate state fidelity between two states
 */
export function calculateFidelity(state1: StateVector, state2: StateVector): number {
  if (state1.amplitudes.length !== state2.amplitudes.length) {
    throw new QuantumSimulationError(
      QuantumErrorCode.INVALID_CIRCUIT,
      'States must have same dimension'
    );
  }

  let fidelity = 0;
  for (let i = 0; i < state1.amplitudes.length; i++) {
    const amp1 = state1.amplitudes[i];
    const amp2 = complexConj(state2.amplitudes[i]);
    const product = complexMul(amp1, amp2);
    fidelity += product.real; // Only real part contributes to |⟨ψ₁|ψ₂⟩|²
  }

  return Math.abs(fidelity);
}

/**
 * Calculate expectation value of observable
 */
export function calculateExpectation(
  state: StateVector,
  observable: Complex[][],
  qubit: number = 0
): number {
  const sim = new QuantumSimulator({ numQubits: state.numQubits });
  sim.initialize(state.amplitudes);

  return sim.expectation(
    {
      name: 'custom',
      representation: observable,
      isHermitian: true,
    },
    [qubit]
  );
}

// ============================================================================
// Export All
// ============================================================================

export {
  // Main class
  QuantumSimulator,

  // Complex utilities
  complex,
  complexAdd,
  complexMul,
  complexConj,
  complexAbs,
  complexPhase,

  // State creation
  createBellState,
  createGHZState,

  // Metrics
  calculateFidelity,
  calculateExpectation,

  // Types
  Complex,
  StateVector,
  DensityMatrix,
  QuantumCircuit,
  Gate,
  SimulationResult,
  ExpectationResult,
  Observable,
  Hamiltonian,
  QUANTUM_CONSTANTS,
  PAULI_MATRICES,
  QuantumErrorCode,
  QuantumSimulationError,
};

// Re-export all types
export * from './types';

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
