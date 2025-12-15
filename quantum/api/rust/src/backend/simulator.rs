//! Local quantum state vector simulator

use async_trait::async_trait;
use num_complex::Complex64;
use std::collections::HashMap;
use std::time::Instant;

use crate::backend::QuantumBackend;
use crate::circuit::{Gate, QuantumCircuit};
use crate::error::{QuantumError, Result};
use crate::types::{
    BackendInfo, BackendType, ExecutionConfig, ExecutionResult, Job, StateVector,
};

/// State vector simulator backend
pub struct SimulatorBackend {
    max_qubits: usize,
}

impl SimulatorBackend {
    /// Create new simulator with default max qubits (20)
    pub fn new() -> Self {
        Self { max_qubits: 20 }
    }

    /// Create simulator with custom max qubits
    pub fn with_max_qubits(max_qubits: usize) -> Self {
        Self { max_qubits }
    }

    /// Apply single-qubit gate to state vector
    fn apply_single_gate(
        state: &mut [Complex64],
        qubit: usize,
        num_qubits: usize,
        matrix: [[Complex64; 2]; 2],
    ) {
        let n = 1 << num_qubits;
        let mask = 1 << qubit;

        for i in 0..n {
            if i & mask == 0 {
                let j = i | mask;
                let a = state[i];
                let b = state[j];
                state[i] = matrix[0][0] * a + matrix[0][1] * b;
                state[j] = matrix[1][0] * a + matrix[1][1] * b;
            }
        }
    }

    /// Apply CNOT gate
    fn apply_cx(state: &mut [Complex64], control: usize, target: usize, num_qubits: usize) {
        let n = 1 << num_qubits;
        let control_mask = 1 << control;
        let target_mask = 1 << target;

        for i in 0..n {
            if (i & control_mask != 0) && (i & target_mask == 0) {
                let j = i | target_mask;
                state.swap(i, j);
            }
        }
    }

    /// Apply CZ gate
    fn apply_cz(state: &mut [Complex64], control: usize, target: usize, num_qubits: usize) {
        let n = 1 << num_qubits;
        let control_mask = 1 << control;
        let target_mask = 1 << target;

        for i in 0..n {
            if (i & control_mask != 0) && (i & target_mask != 0) {
                state[i] = -state[i];
            }
        }
    }

    /// Apply SWAP gate
    fn apply_swap(state: &mut [Complex64], qubit1: usize, qubit2: usize, num_qubits: usize) {
        let n = 1 << num_qubits;
        let mask1 = 1 << qubit1;
        let mask2 = 1 << qubit2;

        for i in 0..n {
            if (i & mask1 != 0) != (i & mask2 != 0) {
                let j = i ^ mask1 ^ mask2;
                if i < j {
                    state.swap(i, j);
                }
            }
        }
    }

    /// Apply Toffoli (CCX) gate
    fn apply_ccx(
        state: &mut [Complex64],
        control1: usize,
        control2: usize,
        target: usize,
        num_qubits: usize,
    ) {
        let n = 1 << num_qubits;
        let c1_mask = 1 << control1;
        let c2_mask = 1 << control2;
        let t_mask = 1 << target;

        for i in 0..n {
            if (i & c1_mask != 0) && (i & c2_mask != 0) && (i & t_mask == 0) {
                let j = i | t_mask;
                state.swap(i, j);
            }
        }
    }

    /// Measure qubit and collapse state
    fn measure_qubit(
        state: &mut [Complex64],
        qubit: usize,
        num_qubits: usize,
        rng: &mut impl FnMut() -> f64,
    ) -> u8 {
        let n = 1 << num_qubits;
        let mask = 1 << qubit;

        // Calculate probability of measuring |1⟩
        let mut prob_one = 0.0;
        for i in 0..n {
            if i & mask != 0 {
                prob_one += state[i].norm_sqr();
            }
        }

        // Determine measurement result
        let result = if rng() < prob_one { 1 } else { 0 };

        // Collapse state
        let norm = if result == 1 {
            prob_one.sqrt()
        } else {
            (1.0 - prob_one).sqrt()
        };

        for i in 0..n {
            if (i & mask != 0) != (result != 0) {
                state[i] = Complex64::new(0.0, 0.0);
            } else if norm > 1e-10 {
                state[i] /= norm;
            }
        }

        result
    }

    /// Simulate circuit
    fn simulate(
        &self,
        circuit: &QuantumCircuit,
        config: &ExecutionConfig,
    ) -> Result<(HashMap<String, u32>, StateVector)> {
        let num_qubits = circuit.num_qubits();
        let num_clbits = circuit.num_clbits();
        let n = 1 << num_qubits;

        if num_qubits > self.max_qubits {
            return Err(QuantumError::BackendError(format!(
                "Circuit has {} qubits, but max is {}",
                num_qubits, self.max_qubits
            )));
        }

        let mut counts: HashMap<String, u32> = HashMap::new();
        let mut final_state = vec![Complex64::new(0.0, 0.0); n];

        // Use seed for reproducibility
        let mut rng_state = config.seed.unwrap_or(42);
        let mut rng = || {
            // Simple LCG RNG
            rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
            (rng_state >> 33) as f64 / (1u64 << 31) as f64
        };

        for shot in 0..config.shots {
            // Initialize state to |0...0⟩
            let mut state = vec![Complex64::new(0.0, 0.0); n];
            state[0] = Complex64::new(1.0, 0.0);

            // Classical register
            let mut clbits = vec![0u8; num_clbits];

            // Apply gates
            for gate in &circuit.circuit.gates {
                match gate {
                    Gate::H { qubits } | Gate::X { qubits } | Gate::Y { qubits }
                    | Gate::Z { qubits } | Gate::S { qubits } | Gate::T { qubits }
                    | Gate::Rx { qubits, .. } | Gate::Ry { qubits, .. }
                    | Gate::Rz { qubits, .. } | Gate::U { qubits, .. } => {
                        if let Some(matrix) = gate.matrix() {
                            Self::apply_single_gate(&mut state, qubits[0], num_qubits, matrix);
                        }
                    }
                    Gate::Cx { qubits } => {
                        Self::apply_cx(&mut state, qubits[0], qubits[1], num_qubits);
                    }
                    Gate::Cz { qubits } => {
                        Self::apply_cz(&mut state, qubits[0], qubits[1], num_qubits);
                    }
                    Gate::Swap { qubits } => {
                        Self::apply_swap(&mut state, qubits[0], qubits[1], num_qubits);
                    }
                    Gate::Ccx { qubits } => {
                        Self::apply_ccx(&mut state, qubits[0], qubits[1], qubits[2], num_qubits);
                    }
                    Gate::Measure { qubits, clbits: cbits } => {
                        let result = Self::measure_qubit(&mut state, qubits[0], num_qubits, &mut rng);
                        if let Some(cb) = cbits.first() {
                            if *cb < num_clbits {
                                clbits[*cb] = result;
                            }
                        }
                    }
                    Gate::Reset { qubits } => {
                        // Measure then flip if needed
                        let result = Self::measure_qubit(&mut state, qubits[0], num_qubits, &mut rng);
                        if result == 1 {
                            if let Some(matrix) = Gate::x(qubits[0]).matrix() {
                                Self::apply_single_gate(&mut state, qubits[0], num_qubits, matrix);
                            }
                        }
                    }
                    Gate::Barrier { .. } => {
                        // No-op
                    }
                }
            }

            // Record measurement outcome
            let bitstring: String = clbits.iter().rev().map(|b| (*b + b'0') as char).collect();
            *counts.entry(bitstring).or_insert(0) += 1;

            // Save last state for statevector output
            if shot == config.shots - 1 {
                final_state = state;
            }
        }

        let statevector = StateVector {
            real: final_state.iter().map(|c| c.re).collect(),
            imag: final_state.iter().map(|c| c.im).collect(),
        };

        Ok((counts, statevector))
    }
}

impl Default for SimulatorBackend {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl QuantumBackend for SimulatorBackend {
    fn name(&self) -> &str {
        "wia_simulator"
    }

    fn provider(&self) -> &str {
        "wia_quantum"
    }

    fn max_qubits(&self) -> usize {
        self.max_qubits
    }

    fn is_simulator(&self) -> bool {
        true
    }

    async fn run(&self, circuit: &QuantumCircuit, config: &ExecutionConfig) -> Result<ExecutionResult> {
        let start = Instant::now();

        let (counts, statevector) = self.simulate(circuit, config)?;

        let execution_time_ms = start.elapsed().as_secs_f64() * 1000.0;

        let backend_info = BackendInfo {
            provider: self.provider().to_string(),
            name: self.name().to_string(),
            backend_type: BackendType::Simulator,
            num_qubits: self.max_qubits,
        };

        let job = Job::new(circuit.circuit_id, backend_info);
        let mut result = ExecutionResult::new(&job, counts, execution_time_ms);
        result.statevector = Some(statevector);

        Ok(result)
    }
}
