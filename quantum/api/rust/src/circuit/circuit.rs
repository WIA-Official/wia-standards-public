//! Quantum circuit definition

use crate::circuit::Gate;
use crate::error::{QuantumError, Result};
use crate::types::WIA_QUANTUM_VERSION;
use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

/// Quantum circuit
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuantumCircuit {
    pub wia_quantum_version: String,
    #[serde(rename = "type")]
    pub data_type: String,
    pub circuit_id: Uuid,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub created_at: DateTime<Utc>,
    pub circuit: CircuitData,
}

/// Circuit data containing qubits, clbits, and gates
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CircuitData {
    pub num_qubits: usize,
    pub num_clbits: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub qasm: Option<String>,
    pub gates: Vec<Gate>,
}

impl QuantumCircuit {
    /// Create a new quantum circuit
    pub fn new(num_qubits: usize, num_clbits: usize) -> Self {
        Self {
            wia_quantum_version: WIA_QUANTUM_VERSION.to_string(),
            data_type: "circuit".to_string(),
            circuit_id: Uuid::new_v4(),
            name: None,
            description: None,
            created_at: Utc::now(),
            circuit: CircuitData {
                num_qubits,
                num_clbits,
                qasm: None,
                gates: Vec::new(),
            },
        }
    }

    /// Create circuit with name
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Create circuit with description
    pub fn with_description(mut self, description: impl Into<String>) -> Self {
        self.description = Some(description.into());
        self
    }

    /// Get number of qubits
    pub fn num_qubits(&self) -> usize {
        self.circuit.num_qubits
    }

    /// Get number of classical bits
    pub fn num_clbits(&self) -> usize {
        self.circuit.num_clbits
    }

    /// Get gate count
    pub fn num_gates(&self) -> usize {
        self.circuit.gates.len()
    }

    /// Validate qubit index
    fn validate_qubit(&self, qubit: usize) -> Result<()> {
        if qubit >= self.circuit.num_qubits {
            Err(QuantumError::InvalidQubit(qubit, self.circuit.num_qubits - 1))
        } else {
            Ok(())
        }
    }

    /// Validate classical bit index
    fn validate_clbit(&self, clbit: usize) -> Result<()> {
        if clbit >= self.circuit.num_clbits {
            Err(QuantumError::InvalidClbit(clbit, self.circuit.num_clbits - 1))
        } else {
            Ok(())
        }
    }

    /// Add Hadamard gate
    pub fn h(&mut self, qubit: usize) -> Result<&mut Self> {
        self.validate_qubit(qubit)?;
        self.circuit.gates.push(Gate::h(qubit));
        Ok(self)
    }

    /// Add Pauli-X gate
    pub fn x(&mut self, qubit: usize) -> Result<&mut Self> {
        self.validate_qubit(qubit)?;
        self.circuit.gates.push(Gate::x(qubit));
        Ok(self)
    }

    /// Add Pauli-Y gate
    pub fn y(&mut self, qubit: usize) -> Result<&mut Self> {
        self.validate_qubit(qubit)?;
        self.circuit.gates.push(Gate::y(qubit));
        Ok(self)
    }

    /// Add Pauli-Z gate
    pub fn z(&mut self, qubit: usize) -> Result<&mut Self> {
        self.validate_qubit(qubit)?;
        self.circuit.gates.push(Gate::z(qubit));
        Ok(self)
    }

    /// Add S gate
    pub fn s(&mut self, qubit: usize) -> Result<&mut Self> {
        self.validate_qubit(qubit)?;
        self.circuit.gates.push(Gate::s(qubit));
        Ok(self)
    }

    /// Add T gate
    pub fn t(&mut self, qubit: usize) -> Result<&mut Self> {
        self.validate_qubit(qubit)?;
        self.circuit.gates.push(Gate::t(qubit));
        Ok(self)
    }

    /// Add Rx gate
    pub fn rx(&mut self, qubit: usize, theta: f64) -> Result<&mut Self> {
        self.validate_qubit(qubit)?;
        self.circuit.gates.push(Gate::rx(qubit, theta));
        Ok(self)
    }

    /// Add Ry gate
    pub fn ry(&mut self, qubit: usize, theta: f64) -> Result<&mut Self> {
        self.validate_qubit(qubit)?;
        self.circuit.gates.push(Gate::ry(qubit, theta));
        Ok(self)
    }

    /// Add Rz gate
    pub fn rz(&mut self, qubit: usize, theta: f64) -> Result<&mut Self> {
        self.validate_qubit(qubit)?;
        self.circuit.gates.push(Gate::rz(qubit, theta));
        Ok(self)
    }

    /// Add CNOT gate
    pub fn cx(&mut self, control: usize, target: usize) -> Result<&mut Self> {
        self.validate_qubit(control)?;
        self.validate_qubit(target)?;
        if control == target {
            return Err(QuantumError::CircuitError(
                "Control and target qubits must be different".to_string(),
            ));
        }
        self.circuit.gates.push(Gate::cx(control, target));
        Ok(self)
    }

    /// Add CZ gate
    pub fn cz(&mut self, control: usize, target: usize) -> Result<&mut Self> {
        self.validate_qubit(control)?;
        self.validate_qubit(target)?;
        self.circuit.gates.push(Gate::cz(control, target));
        Ok(self)
    }

    /// Add SWAP gate
    pub fn swap(&mut self, qubit1: usize, qubit2: usize) -> Result<&mut Self> {
        self.validate_qubit(qubit1)?;
        self.validate_qubit(qubit2)?;
        self.circuit.gates.push(Gate::swap(qubit1, qubit2));
        Ok(self)
    }

    /// Add Toffoli gate
    pub fn ccx(&mut self, control1: usize, control2: usize, target: usize) -> Result<&mut Self> {
        self.validate_qubit(control1)?;
        self.validate_qubit(control2)?;
        self.validate_qubit(target)?;
        self.circuit.gates.push(Gate::ccx(control1, control2, target));
        Ok(self)
    }

    /// Add measurement
    pub fn measure(&mut self, qubit: usize, clbit: usize) -> Result<&mut Self> {
        self.validate_qubit(qubit)?;
        self.validate_clbit(clbit)?;
        self.circuit.gates.push(Gate::measure(qubit, clbit));
        Ok(self)
    }

    /// Measure all qubits to corresponding classical bits
    pub fn measure_all(&mut self) -> Result<&mut Self> {
        let n = self.circuit.num_qubits.min(self.circuit.num_clbits);
        for i in 0..n {
            self.circuit.gates.push(Gate::measure(i, i));
        }
        Ok(self)
    }

    /// Add reset
    pub fn reset(&mut self, qubit: usize) -> Result<&mut Self> {
        self.validate_qubit(qubit)?;
        self.circuit.gates.push(Gate::reset(qubit));
        Ok(self)
    }

    /// Add barrier on all qubits
    pub fn barrier(&mut self) -> &mut Self {
        let qubits: Vec<usize> = (0..self.circuit.num_qubits).collect();
        self.circuit.gates.push(Gate::barrier(qubits));
        self
    }

    /// Generate OpenQASM 3.0 representation
    pub fn to_qasm(&self) -> String {
        let mut qasm = String::new();
        qasm.push_str("OPENQASM 3.0;\n");
        qasm.push_str("include \"stdgates.inc\";\n\n");
        qasm.push_str(&format!("qubit[{}] q;\n", self.circuit.num_qubits));
        qasm.push_str(&format!("bit[{}] c;\n\n", self.circuit.num_clbits));

        for gate in &self.circuit.gates {
            let line = match gate {
                Gate::H { qubits } => format!("h q[{}];", qubits[0]),
                Gate::X { qubits } => format!("x q[{}];", qubits[0]),
                Gate::Y { qubits } => format!("y q[{}];", qubits[0]),
                Gate::Z { qubits } => format!("z q[{}];", qubits[0]),
                Gate::S { qubits } => format!("s q[{}];", qubits[0]),
                Gate::T { qubits } => format!("t q[{}];", qubits[0]),
                Gate::Rx { qubits, params } => {
                    format!("rx({}) q[{}];", params[0], qubits[0])
                }
                Gate::Ry { qubits, params } => {
                    format!("ry({}) q[{}];", params[0], qubits[0])
                }
                Gate::Rz { qubits, params } => {
                    format!("rz({}) q[{}];", params[0], qubits[0])
                }
                Gate::U { qubits, params } => {
                    format!("u({}, {}, {}) q[{}];", params[0], params[1], params[2], qubits[0])
                }
                Gate::Cx { qubits } => format!("cx q[{}], q[{}];", qubits[0], qubits[1]),
                Gate::Cz { qubits } => format!("cz q[{}], q[{}];", qubits[0], qubits[1]),
                Gate::Swap { qubits } => format!("swap q[{}], q[{}];", qubits[0], qubits[1]),
                Gate::Ccx { qubits } => {
                    format!("ccx q[{}], q[{}], q[{}];", qubits[0], qubits[1], qubits[2])
                }
                Gate::Measure { qubits, clbits } => {
                    format!("c[{}] = measure q[{}];", clbits[0], qubits[0])
                }
                Gate::Reset { qubits } => format!("reset q[{}];", qubits[0]),
                Gate::Barrier { qubits } => {
                    let q_str: Vec<String> = qubits.iter().map(|q| format!("q[{}]", q)).collect();
                    format!("barrier {};", q_str.join(", "))
                }
            };
            qasm.push_str(&line);
            qasm.push('\n');
        }

        qasm
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
