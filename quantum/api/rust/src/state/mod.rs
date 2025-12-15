//! Quantum state module

use crate::error::{QuantumError, Result};
use crate::types::{StateVector, WIA_QUANTUM_VERSION};
use num_complex::Complex64;
use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

/// State type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StateType {
    Statevector,
    DensityMatrix,
    Stabilizer,
}

/// Basis type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Basis {
    Computational,
    Pauli,
    Custom,
}

/// Quantum state wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuantumState {
    pub wia_quantum_version: String,
    #[serde(rename = "type")]
    pub data_type: String,
    pub state_type: StateType,
    pub num_qubits: usize,
    pub data: StateVector,
    #[serde(default)]
    pub basis: Basis,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub purity: Option<f64>,
}

impl QuantumState {
    /// Create a new state initialized to |0...0⟩
    pub fn new(num_qubits: usize) -> Self {
        let size = 1 << num_qubits;
        Self {
            wia_quantum_version: WIA_QUANTUM_VERSION.to_string(),
            data_type: "state".to_string(),
            state_type: StateType::Statevector,
            num_qubits,
            data: StateVector::new(size),
            basis: Basis::Computational,
            purity: Some(1.0),
        }
    }

    /// Create state from amplitudes
    pub fn from_amplitudes(num_qubits: usize, real: Vec<f64>, imag: Vec<f64>) -> Result<Self> {
        let expected_size = 1 << num_qubits;
        if real.len() != expected_size || imag.len() != expected_size {
            return Err(QuantumError::StateError(format!(
                "Expected {} amplitudes, got {} real and {} imag",
                expected_size,
                real.len(),
                imag.len()
            )));
        }

        // Check normalization
        let norm: f64 = real
            .iter()
            .zip(imag.iter())
            .map(|(r, i)| r * r + i * i)
            .sum();

        if (norm - 1.0).abs() > 1e-6 {
            return Err(QuantumError::StateError(format!(
                "State is not normalized: norm = {}",
                norm
            )));
        }

        Ok(Self {
            wia_quantum_version: WIA_QUANTUM_VERSION.to_string(),
            data_type: "state".to_string(),
            state_type: StateType::Statevector,
            num_qubits,
            data: StateVector { real, imag },
            basis: Basis::Computational,
            purity: Some(1.0),
        })
    }

    /// Get amplitude at basis state index
    pub fn amplitude(&self, index: usize) -> Complex64 {
        self.data.amplitude(index)
    }

    /// Get probability of measuring basis state
    pub fn probability(&self, index: usize) -> f64 {
        self.data.probability(index)
    }

    /// Get all probabilities
    pub fn probabilities(&self) -> Vec<f64> {
        (0..self.data.len())
            .map(|i| self.probability(i))
            .collect()
    }

    /// Get the most probable basis state
    pub fn most_probable_state(&self) -> (usize, f64) {
        self.probabilities()
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, p)| (i, *p))
            .unwrap_or((0, 0.0))
    }

    /// Create Bell state |Φ+⟩ = (|00⟩ + |11⟩) / √2
    pub fn bell_phi_plus() -> Self {
        let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
        Self::from_amplitudes(
            2,
            vec![inv_sqrt2, 0.0, 0.0, inv_sqrt2],
            vec![0.0, 0.0, 0.0, 0.0],
        )
        .expect("Bell state creation should not fail")
    }

    /// Create GHZ state (|00...0⟩ + |11...1⟩) / √2
    pub fn ghz(num_qubits: usize) -> Self {
        let size = 1 << num_qubits;
        let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
        let mut real = vec![0.0; size];
        real[0] = inv_sqrt2;
        real[size - 1] = inv_sqrt2;

        Self::from_amplitudes(num_qubits, real, vec![0.0; size])
            .expect("GHZ state creation should not fail")
    }

    /// Calculate overlap with another state
    pub fn overlap(&self, other: &QuantumState) -> Result<Complex64> {
        if self.num_qubits != other.num_qubits {
            return Err(QuantumError::StateError(
                "States have different number of qubits".to_string(),
            ));
        }

        let mut overlap = Complex64::new(0.0, 0.0);
        for i in 0..self.data.len() {
            let a = self.amplitude(i).conj();
            let b = other.amplitude(i);
            overlap += a * b;
        }

        Ok(overlap)
    }

    /// Calculate fidelity with another state
    pub fn fidelity(&self, other: &QuantumState) -> Result<f64> {
        let overlap = self.overlap(other)?;
        Ok(overlap.norm_sqr())
    }

    /// Serialize to JSON
    pub fn to_json(&self) -> Result<String> {
        serde_json::to_string_pretty(self).map_err(Into::into)
    }

    /// Deserialize from JSON
    pub fn from_json(json: &str) -> Result<Self> {
        serde_json::from_str(json).map_err(Into::into)
    }
}

impl Default for Basis {
    fn default() -> Self {
        Basis::Computational
    }
}
