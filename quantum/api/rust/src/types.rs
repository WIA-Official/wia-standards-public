//! Core types for WIA Quantum SDK

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;
use chrono::{DateTime, Utc};

/// WIA Quantum format version
pub const WIA_QUANTUM_VERSION: &str = "1.0.0";

/// Job status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JobStatus {
    Queued,
    Running,
    Completed,
    Failed,
    Cancelled,
}

/// Backend type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BackendType {
    Simulator,
    Hardware,
}

/// Backend information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BackendInfo {
    pub provider: String,
    pub name: String,
    #[serde(rename = "type")]
    pub backend_type: BackendType,
    pub num_qubits: usize,
}

/// Execution configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecutionConfig {
    #[serde(default = "default_shots")]
    pub shots: u32,
    pub seed: Option<u64>,
    #[serde(default)]
    pub optimization_level: u8,
}

fn default_shots() -> u32 {
    1024
}

impl Default for ExecutionConfig {
    fn default() -> Self {
        Self {
            shots: 1024,
            seed: None,
            optimization_level: 1,
        }
    }
}

/// Job request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Job {
    pub wia_quantum_version: String,
    #[serde(rename = "type")]
    pub data_type: String,
    pub job_id: Uuid,
    pub circuit_id: Uuid,
    pub status: JobStatus,
    pub created_at: DateTime<Utc>,
    pub backend: BackendInfo,
    pub execution: ExecutionConfig,
}

impl Job {
    pub fn new(circuit_id: Uuid, backend: BackendInfo) -> Self {
        Self {
            wia_quantum_version: WIA_QUANTUM_VERSION.to_string(),
            data_type: "job".to_string(),
            job_id: Uuid::new_v4(),
            circuit_id,
            status: JobStatus::Queued,
            created_at: Utc::now(),
            backend,
            execution: ExecutionConfig::default(),
        }
    }
}

/// Execution result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecutionResult {
    pub wia_quantum_version: String,
    #[serde(rename = "type")]
    pub data_type: String,
    pub result_id: Uuid,
    pub job_id: Uuid,
    pub circuit_id: Uuid,
    pub status: JobStatus,
    pub success: bool,
    pub created_at: DateTime<Utc>,
    pub counts: HashMap<String, u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub statevector: Option<StateVector>,
    pub metadata: ResultMetadata,
}

/// State vector representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StateVector {
    pub real: Vec<f64>,
    pub imag: Vec<f64>,
}

impl StateVector {
    pub fn new(size: usize) -> Self {
        let mut real = vec![0.0; size];
        real[0] = 1.0; // |0...0⟩ state
        Self {
            real,
            imag: vec![0.0; size],
        }
    }

    pub fn len(&self) -> usize {
        self.real.len()
    }

    pub fn is_empty(&self) -> bool {
        self.real.is_empty()
    }

    /// Get amplitude at index
    pub fn amplitude(&self, index: usize) -> num_complex::Complex64 {
        num_complex::Complex64::new(
            self.real.get(index).copied().unwrap_or(0.0),
            self.imag.get(index).copied().unwrap_or(0.0),
        )
    }

    /// Get probability at index
    pub fn probability(&self, index: usize) -> f64 {
        self.amplitude(index).norm_sqr()
    }
}

/// Result metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResultMetadata {
    pub shots: u32,
    pub execution_time_ms: f64,
    pub backend: String,
}

impl ExecutionResult {
    pub fn new(job: &Job, counts: HashMap<String, u32>, execution_time_ms: f64) -> Self {
        Self {
            wia_quantum_version: WIA_QUANTUM_VERSION.to_string(),
            data_type: "result".to_string(),
            result_id: Uuid::new_v4(),
            job_id: job.job_id,
            circuit_id: job.circuit_id,
            status: JobStatus::Completed,
            success: true,
            created_at: Utc::now(),
            counts,
            statevector: None,
            metadata: ResultMetadata {
                shots: job.execution.shots,
                execution_time_ms,
                backend: job.backend.name.clone(),
            },
        }
    }
}
