//! Quantum provider traits and implementations

use crate::circuit::QuantumCircuit;
use crate::error::{QuantumError, Result};
use crate::types::{BackendInfo, BackendType, ExecutionResult, JobStatus};
use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Provider configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ProviderConfig {
    /// API key for authentication
    pub api_key: Option<String>,
    /// API URL override
    pub api_url: Option<String>,
    /// Default backend ID
    pub default_backend: Option<String>,
    /// Connection timeout in milliseconds
    pub timeout_ms: Option<u64>,
    /// Provider-specific options
    #[serde(default)]
    pub options: HashMap<String, serde_json::Value>,
}

/// Provider capabilities
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ProviderCapabilities {
    /// Supports simulators
    pub simulators: bool,
    /// Supports real hardware
    pub real_hardware: bool,
    /// Supports error mitigation
    pub error_mitigation: bool,
    /// Supports dynamic circuits
    pub dynamic_circuits: bool,
    /// Maximum qubits
    pub max_qubits: usize,
    /// Maximum shots per job
    pub max_shots: usize,
    /// Supported basis gates
    pub basis_gates: Vec<String>,
    /// PQC enabled
    pub pqc_enabled: bool,
}

/// Job request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JobRequest {
    /// Circuit to execute
    pub circuit: serde_json::Value,
    /// Backend ID
    pub backend_id: Option<String>,
    /// Number of shots
    pub shots: usize,
    /// Job name
    pub name: Option<String>,
    /// Additional options
    #[serde(default)]
    pub options: HashMap<String, serde_json::Value>,
}

impl JobRequest {
    /// Create job request from circuit
    pub fn from_circuit(circuit: &QuantumCircuit) -> Self {
        Self {
            circuit: serde_json::to_value(circuit.to_data()).unwrap_or_default(),
            backend_id: None,
            shots: 1024,
            name: circuit.name.clone(),
            options: HashMap::new(),
        }
    }

    /// Set backend
    pub fn with_backend(mut self, backend_id: &str) -> Self {
        self.backend_id = Some(backend_id.to_string());
        self
    }

    /// Set shots
    pub fn with_shots(mut self, shots: usize) -> Self {
        self.shots = shots;
        self
    }
}

/// Job handle
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JobHandle {
    /// Job ID
    pub job_id: String,
    /// Provider ID
    pub provider_id: String,
    /// Backend ID
    pub backend_id: String,
    /// Submission time
    pub submitted_at: i64,
}

/// Job result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JobResult {
    /// Job ID
    pub job_id: String,
    /// Execution result
    pub result: ExecutionResult,
    /// Execution metadata
    pub metadata: JobMetadata,
}

/// Job metadata
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct JobMetadata {
    /// Queue time in milliseconds
    pub queue_time_ms: Option<f64>,
    /// Execution time in milliseconds
    pub execution_time_ms: Option<f64>,
    /// Backend name
    pub backend_name: Option<String>,
    /// Shots executed
    pub shots: usize,
}

/// Quantum provider trait
#[async_trait]
pub trait QuantumProvider: Send + Sync {
    /// Get provider ID
    fn id(&self) -> &str;

    /// Get provider name
    fn name(&self) -> &str;

    /// Get provider version
    fn version(&self) -> &str;

    /// Connect to the provider
    async fn connect(&mut self, config: ProviderConfig) -> Result<()>;

    /// Disconnect from the provider
    async fn disconnect(&mut self) -> Result<()>;

    /// Check if connected
    fn is_connected(&self) -> bool;

    /// Get available backends
    async fn get_backends(&self) -> Result<Vec<BackendInfo>>;

    /// Get specific backend
    async fn get_backend(&self, id: &str) -> Result<Option<BackendInfo>>;

    /// Submit a job
    async fn submit_job(&self, job: JobRequest) -> Result<JobHandle>;

    /// Get job status
    async fn get_job_status(&self, job_id: &str) -> Result<JobStatus>;

    /// Get job result
    async fn get_job_result(&self, job_id: &str) -> Result<JobResult>;

    /// Cancel a job
    async fn cancel_job(&self, job_id: &str) -> Result<()>;

    /// Get provider capabilities
    fn capabilities(&self) -> ProviderCapabilities;
}

/// Local simulator provider
pub struct LocalSimulatorProvider {
    connected: bool,
    config: ProviderConfig,
    max_qubits: usize,
}

impl LocalSimulatorProvider {
    /// Create new local simulator provider
    pub fn new() -> Self {
        Self {
            connected: false,
            config: ProviderConfig::default(),
            max_qubits: 30,
        }
    }

    /// Set maximum qubits
    pub fn with_max_qubits(mut self, max_qubits: usize) -> Self {
        self.max_qubits = max_qubits;
        self
    }
}

impl Default for LocalSimulatorProvider {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl QuantumProvider for LocalSimulatorProvider {
    fn id(&self) -> &str {
        "local"
    }

    fn name(&self) -> &str {
        "WIA Local Simulator"
    }

    fn version(&self) -> &str {
        env!("CARGO_PKG_VERSION")
    }

    async fn connect(&mut self, config: ProviderConfig) -> Result<()> {
        self.config = config;
        self.connected = true;
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<()> {
        self.connected = false;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    async fn get_backends(&self) -> Result<Vec<BackendInfo>> {
        Ok(vec![BackendInfo {
            name: "statevector_simulator".to_string(),
            backend_type: BackendType::Simulator,
            num_qubits: self.max_qubits,
            status: "online".to_string(),
            provider: "WIA".to_string(),
        }])
    }

    async fn get_backend(&self, id: &str) -> Result<Option<BackendInfo>> {
        let backends = self.get_backends().await?;
        Ok(backends.into_iter().find(|b| b.name == id))
    }

    async fn submit_job(&self, job: JobRequest) -> Result<JobHandle> {
        if !self.connected {
            return Err(QuantumError::ConnectionError(
                "Provider not connected".to_string(),
            ));
        }

        // For local simulator, we execute immediately
        let job_id = uuid::Uuid::new_v4().to_string();

        Ok(JobHandle {
            job_id,
            provider_id: self.id().to_string(),
            backend_id: job
                .backend_id
                .unwrap_or_else(|| "statevector_simulator".to_string()),
            submitted_at: chrono::Utc::now().timestamp_millis(),
        })
    }

    async fn get_job_status(&self, _job_id: &str) -> Result<JobStatus> {
        // Local simulator completes immediately
        Ok(JobStatus::Completed)
    }

    async fn get_job_result(&self, job_id: &str) -> Result<JobResult> {
        // Mock result for local simulator
        Ok(JobResult {
            job_id: job_id.to_string(),
            result: ExecutionResult::default(),
            metadata: JobMetadata {
                execution_time_ms: Some(10.0),
                shots: 1024,
                ..Default::default()
            },
        })
    }

    async fn cancel_job(&self, _job_id: &str) -> Result<()> {
        Ok(())
    }

    fn capabilities(&self) -> ProviderCapabilities {
        ProviderCapabilities {
            simulators: true,
            real_hardware: false,
            error_mitigation: false,
            dynamic_circuits: true,
            max_qubits: self.max_qubits,
            max_shots: 1_000_000,
            basis_gates: vec![
                "id".to_string(),
                "x".to_string(),
                "y".to_string(),
                "z".to_string(),
                "h".to_string(),
                "s".to_string(),
                "t".to_string(),
                "cx".to_string(),
                "cz".to_string(),
                "swap".to_string(),
            ],
            pqc_enabled: true,
        }
    }
}

/// IBM Quantum provider (mock implementation)
pub struct IBMProvider {
    connected: bool,
    config: ProviderConfig,
}

impl IBMProvider {
    /// Create new IBM provider
    pub fn new() -> Self {
        Self {
            connected: false,
            config: ProviderConfig::default(),
        }
    }
}

impl Default for IBMProvider {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl QuantumProvider for IBMProvider {
    fn id(&self) -> &str {
        "ibm"
    }

    fn name(&self) -> &str {
        "IBM Quantum"
    }

    fn version(&self) -> &str {
        "1.0.0"
    }

    async fn connect(&mut self, config: ProviderConfig) -> Result<()> {
        if config.api_key.is_none() {
            return Err(QuantumError::AuthError(
                "IBM Quantum requires API key".to_string(),
            ));
        }
        self.config = config;
        self.connected = true;
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<()> {
        self.connected = false;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    async fn get_backends(&self) -> Result<Vec<BackendInfo>> {
        // Mock IBM backends
        Ok(vec![
            BackendInfo {
                name: "ibm_brisbane".to_string(),
                backend_type: BackendType::Hardware,
                num_qubits: 127,
                status: "online".to_string(),
                provider: "IBM".to_string(),
            },
            BackendInfo {
                name: "ibm_kyoto".to_string(),
                backend_type: BackendType::Hardware,
                num_qubits: 127,
                status: "online".to_string(),
                provider: "IBM".to_string(),
            },
            BackendInfo {
                name: "ibmq_qasm_simulator".to_string(),
                backend_type: BackendType::Simulator,
                num_qubits: 32,
                status: "online".to_string(),
                provider: "IBM".to_string(),
            },
        ])
    }

    async fn get_backend(&self, id: &str) -> Result<Option<BackendInfo>> {
        let backends = self.get_backends().await?;
        Ok(backends.into_iter().find(|b| b.name == id))
    }

    async fn submit_job(&self, job: JobRequest) -> Result<JobHandle> {
        if !self.connected {
            return Err(QuantumError::ConnectionError(
                "Provider not connected".to_string(),
            ));
        }

        let job_id = format!("ibm_{}", uuid::Uuid::new_v4());

        Ok(JobHandle {
            job_id,
            provider_id: self.id().to_string(),
            backend_id: job
                .backend_id
                .unwrap_or_else(|| "ibmq_qasm_simulator".to_string()),
            submitted_at: chrono::Utc::now().timestamp_millis(),
        })
    }

    async fn get_job_status(&self, _job_id: &str) -> Result<JobStatus> {
        Ok(JobStatus::Queued)
    }

    async fn get_job_result(&self, job_id: &str) -> Result<JobResult> {
        Ok(JobResult {
            job_id: job_id.to_string(),
            result: ExecutionResult::default(),
            metadata: JobMetadata::default(),
        })
    }

    async fn cancel_job(&self, _job_id: &str) -> Result<()> {
        Ok(())
    }

    fn capabilities(&self) -> ProviderCapabilities {
        ProviderCapabilities {
            simulators: true,
            real_hardware: true,
            error_mitigation: true,
            dynamic_circuits: true,
            max_qubits: 127,
            max_shots: 100_000,
            basis_gates: vec![
                "id".to_string(),
                "rz".to_string(),
                "sx".to_string(),
                "x".to_string(),
                "cx".to_string(),
            ],
            pqc_enabled: false,
        }
    }
}

/// Amazon Braket provider (mock implementation)
pub struct AmazonBraketProvider {
    connected: bool,
    config: ProviderConfig,
}

impl AmazonBraketProvider {
    /// Create new Braket provider
    pub fn new() -> Self {
        Self {
            connected: false,
            config: ProviderConfig::default(),
        }
    }
}

impl Default for AmazonBraketProvider {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl QuantumProvider for AmazonBraketProvider {
    fn id(&self) -> &str {
        "braket"
    }

    fn name(&self) -> &str {
        "Amazon Braket"
    }

    fn version(&self) -> &str {
        "1.0.0"
    }

    async fn connect(&mut self, config: ProviderConfig) -> Result<()> {
        self.config = config;
        self.connected = true;
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<()> {
        self.connected = false;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    async fn get_backends(&self) -> Result<Vec<BackendInfo>> {
        Ok(vec![
            BackendInfo {
                name: "sv1".to_string(),
                backend_type: BackendType::Simulator,
                num_qubits: 34,
                status: "online".to_string(),
                provider: "AWS".to_string(),
            },
            BackendInfo {
                name: "tn1".to_string(),
                backend_type: BackendType::Simulator,
                num_qubits: 50,
                status: "online".to_string(),
                provider: "AWS".to_string(),
            },
            BackendInfo {
                name: "ionq_harmony".to_string(),
                backend_type: BackendType::Hardware,
                num_qubits: 11,
                status: "online".to_string(),
                provider: "IonQ".to_string(),
            },
        ])
    }

    async fn get_backend(&self, id: &str) -> Result<Option<BackendInfo>> {
        let backends = self.get_backends().await?;
        Ok(backends.into_iter().find(|b| b.name == id))
    }

    async fn submit_job(&self, job: JobRequest) -> Result<JobHandle> {
        if !self.connected {
            return Err(QuantumError::ConnectionError(
                "Provider not connected".to_string(),
            ));
        }

        let job_id = format!("arn:aws:braket:us-east-1:job/{}", uuid::Uuid::new_v4());

        Ok(JobHandle {
            job_id,
            provider_id: self.id().to_string(),
            backend_id: job.backend_id.unwrap_or_else(|| "sv1".to_string()),
            submitted_at: chrono::Utc::now().timestamp_millis(),
        })
    }

    async fn get_job_status(&self, _job_id: &str) -> Result<JobStatus> {
        Ok(JobStatus::Queued)
    }

    async fn get_job_result(&self, job_id: &str) -> Result<JobResult> {
        Ok(JobResult {
            job_id: job_id.to_string(),
            result: ExecutionResult::default(),
            metadata: JobMetadata::default(),
        })
    }

    async fn cancel_job(&self, _job_id: &str) -> Result<()> {
        Ok(())
    }

    fn capabilities(&self) -> ProviderCapabilities {
        ProviderCapabilities {
            simulators: true,
            real_hardware: true,
            error_mitigation: false,
            dynamic_circuits: false,
            max_qubits: 50,
            max_shots: 100_000,
            basis_gates: vec![
                "h".to_string(),
                "x".to_string(),
                "y".to_string(),
                "z".to_string(),
                "cnot".to_string(),
            ],
            pqc_enabled: false,
        }
    }
}
