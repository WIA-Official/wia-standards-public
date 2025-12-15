//! Quantum gate definitions

use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Quantum gate types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "name", rename_all = "lowercase")]
pub enum Gate {
    /// Hadamard gate
    H { qubits: Vec<usize> },
    /// Pauli-X gate
    X { qubits: Vec<usize> },
    /// Pauli-Y gate
    Y { qubits: Vec<usize> },
    /// Pauli-Z gate
    Z { qubits: Vec<usize> },
    /// S gate (sqrt(Z))
    S { qubits: Vec<usize> },
    /// T gate (sqrt(S))
    T { qubits: Vec<usize> },
    /// Rotation around X-axis
    Rx { qubits: Vec<usize>, params: Vec<f64> },
    /// Rotation around Y-axis
    Ry { qubits: Vec<usize>, params: Vec<f64> },
    /// Rotation around Z-axis
    Rz { qubits: Vec<usize>, params: Vec<f64> },
    /// Universal single-qubit gate
    U { qubits: Vec<usize>, params: Vec<f64> },
    /// CNOT gate
    Cx { qubits: Vec<usize> },
    /// CZ gate
    Cz { qubits: Vec<usize> },
    /// SWAP gate
    Swap { qubits: Vec<usize> },
    /// Toffoli gate (CCX)
    Ccx { qubits: Vec<usize> },
    /// Measurement
    Measure { qubits: Vec<usize>, clbits: Vec<usize> },
    /// Reset qubit to |0⟩
    Reset { qubits: Vec<usize> },
    /// Barrier (no-op for optimization boundary)
    Barrier { qubits: Vec<usize> },
}

impl Gate {
    /// Create Hadamard gate
    pub fn h(qubit: usize) -> Self {
        Gate::H { qubits: vec![qubit] }
    }

    /// Create Pauli-X gate
    pub fn x(qubit: usize) -> Self {
        Gate::X { qubits: vec![qubit] }
    }

    /// Create Pauli-Y gate
    pub fn y(qubit: usize) -> Self {
        Gate::Y { qubits: vec![qubit] }
    }

    /// Create Pauli-Z gate
    pub fn z(qubit: usize) -> Self {
        Gate::Z { qubits: vec![qubit] }
    }

    /// Create S gate
    pub fn s(qubit: usize) -> Self {
        Gate::S { qubits: vec![qubit] }
    }

    /// Create T gate
    pub fn t(qubit: usize) -> Self {
        Gate::T { qubits: vec![qubit] }
    }

    /// Create Rx gate
    pub fn rx(qubit: usize, theta: f64) -> Self {
        Gate::Rx {
            qubits: vec![qubit],
            params: vec![theta],
        }
    }

    /// Create Ry gate
    pub fn ry(qubit: usize, theta: f64) -> Self {
        Gate::Ry {
            qubits: vec![qubit],
            params: vec![theta],
        }
    }

    /// Create Rz gate
    pub fn rz(qubit: usize, theta: f64) -> Self {
        Gate::Rz {
            qubits: vec![qubit],
            params: vec![theta],
        }
    }

    /// Create U gate (universal single-qubit)
    pub fn u(qubit: usize, theta: f64, phi: f64, lambda: f64) -> Self {
        Gate::U {
            qubits: vec![qubit],
            params: vec![theta, phi, lambda],
        }
    }

    /// Create CNOT gate
    pub fn cx(control: usize, target: usize) -> Self {
        Gate::Cx {
            qubits: vec![control, target],
        }
    }

    /// Create CZ gate
    pub fn cz(control: usize, target: usize) -> Self {
        Gate::Cz {
            qubits: vec![control, target],
        }
    }

    /// Create SWAP gate
    pub fn swap(qubit1: usize, qubit2: usize) -> Self {
        Gate::Swap {
            qubits: vec![qubit1, qubit2],
        }
    }

    /// Create Toffoli gate
    pub fn ccx(control1: usize, control2: usize, target: usize) -> Self {
        Gate::Ccx {
            qubits: vec![control1, control2, target],
        }
    }

    /// Create measurement
    pub fn measure(qubit: usize, clbit: usize) -> Self {
        Gate::Measure {
            qubits: vec![qubit],
            clbits: vec![clbit],
        }
    }

    /// Create reset
    pub fn reset(qubit: usize) -> Self {
        Gate::Reset { qubits: vec![qubit] }
    }

    /// Create barrier
    pub fn barrier(qubits: Vec<usize>) -> Self {
        Gate::Barrier { qubits }
    }

    /// Get target qubits
    pub fn qubits(&self) -> &[usize] {
        match self {
            Gate::H { qubits } => qubits,
            Gate::X { qubits } => qubits,
            Gate::Y { qubits } => qubits,
            Gate::Z { qubits } => qubits,
            Gate::S { qubits } => qubits,
            Gate::T { qubits } => qubits,
            Gate::Rx { qubits, .. } => qubits,
            Gate::Ry { qubits, .. } => qubits,
            Gate::Rz { qubits, .. } => qubits,
            Gate::U { qubits, .. } => qubits,
            Gate::Cx { qubits } => qubits,
            Gate::Cz { qubits } => qubits,
            Gate::Swap { qubits } => qubits,
            Gate::Ccx { qubits } => qubits,
            Gate::Measure { qubits, .. } => qubits,
            Gate::Reset { qubits } => qubits,
            Gate::Barrier { qubits } => qubits,
        }
    }

    /// Check if this is a measurement
    pub fn is_measurement(&self) -> bool {
        matches!(self, Gate::Measure { .. })
    }

    /// Get gate matrix (2x2 for single-qubit gates)
    pub fn matrix(&self) -> Option<[[num_complex::Complex64; 2]; 2]> {
        use num_complex::Complex64;
        let i = Complex64::i();
        let zero = Complex64::new(0.0, 0.0);
        let one = Complex64::new(1.0, 0.0);
        let sqrt2_inv = Complex64::new(1.0 / 2.0_f64.sqrt(), 0.0);

        match self {
            Gate::H { .. } => Some([
                [sqrt2_inv, sqrt2_inv],
                [sqrt2_inv, -sqrt2_inv],
            ]),
            Gate::X { .. } => Some([
                [zero, one],
                [one, zero],
            ]),
            Gate::Y { .. } => Some([
                [zero, -i],
                [i, zero],
            ]),
            Gate::Z { .. } => Some([
                [one, zero],
                [zero, -one],
            ]),
            Gate::S { .. } => Some([
                [one, zero],
                [zero, i],
            ]),
            Gate::T { .. } => Some([
                [one, zero],
                [zero, Complex64::from_polar(1.0, PI / 4.0)],
            ]),
            Gate::Rx { params, .. } => {
                let theta = params.first().copied().unwrap_or(0.0);
                let cos = Complex64::new((theta / 2.0).cos(), 0.0);
                let sin = Complex64::new(0.0, -(theta / 2.0).sin());
                Some([
                    [cos, sin],
                    [sin, cos],
                ])
            }
            Gate::Ry { params, .. } => {
                let theta = params.first().copied().unwrap_or(0.0);
                let cos = Complex64::new((theta / 2.0).cos(), 0.0);
                let sin = Complex64::new((theta / 2.0).sin(), 0.0);
                Some([
                    [cos, -sin],
                    [sin, cos],
                ])
            }
            Gate::Rz { params, .. } => {
                let theta = params.first().copied().unwrap_or(0.0);
                Some([
                    [Complex64::from_polar(1.0, -theta / 2.0), zero],
                    [zero, Complex64::from_polar(1.0, theta / 2.0)],
                ])
            }
            _ => None, // Multi-qubit gates don't have 2x2 matrix
        }
    }
}
