//! Protocol message types

use crate::circuit::QuantumCircuit;
use crate::types::{ExecutionConfig, ExecutionResult};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;

/// Message type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    Connect,
    ConnectAck,
    Disconnect,
    SubmitJob,
    JobQueued,
    JobRunning,
    JobCompleted,
    JobFailed,
    JobCancelled,
    CancelJob,
    GetJobStatus,
    JobStatus,
    Subscribe,
    SubscribeAck,
    Unsubscribe,
    BackendStatus,
    Calibration,
    Error,
    Ping,
    Pong,
}

/// Base protocol message
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Message {
    pub protocol: String,
    pub version: String,
    pub message_id: String,
    pub timestamp: i64,
    #[serde(rename = "type")]
    pub message_type: MessageType,
    pub payload: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub session_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,
}

impl Message {
    /// Create a new message with the given type and payload
    pub fn new<T: Serialize>(message_type: MessageType, payload: T) -> Self {
        Self {
            protocol: super::PROTOCOL_ID.to_string(),
            version: super::PROTOCOL_VERSION.to_string(),
            message_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now().timestamp_millis(),
            message_type,
            payload: serde_json::to_value(payload).unwrap_or_default(),
            session_id: None,
            correlation_id: None,
        }
    }

    /// Create a message with session ID
    pub fn with_session(mut self, session_id: String) -> Self {
        self.session_id = Some(session_id);
        self
    }

    /// Create a message with correlation ID
    pub fn with_correlation(mut self, correlation_id: String) -> Self {
        self.correlation_id = Some(correlation_id);
        self
    }

    /// Serialize to JSON
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }

    /// Deserialize from JSON
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }
}

/// Connect request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectPayload {
    pub client_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub client_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub client_version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capabilities: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auth: Option<AuthInfo>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub options: Option<ConnectOptions>,
}

/// Authentication info
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AuthInfo {
    #[serde(rename = "type")]
    pub auth_type: String,
    pub token: String,
}

/// Connect options
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectOptions {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferred_backend: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compression: Option<bool>,
}

/// Connect acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectAckPayload {
    pub success: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub session_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub server_info: Option<ServerInfo>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub available_backends: Option<Vec<BackendInfo>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quotas: Option<Quotas>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// Server info
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ServerInfo {
    pub name: String,
    pub version: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub region: Option<String>,
}

/// Backend info
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BackendInfo {
    pub backend_id: String,
    #[serde(rename = "type")]
    pub backend_type: String,
    pub provider: String,
    pub num_qubits: usize,
    pub status: BackendStatusType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub queue_length: Option<usize>,
}

/// Backend status type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum BackendStatusType {
    Online,
    Offline,
    Maintenance,
    Calibrating,
}

/// User quotas
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Quotas {
    pub max_qubits: usize,
    pub max_shots: usize,
    pub max_jobs_per_day: usize,
    pub remaining_jobs: usize,
}

/// Submit job request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SubmitJobPayload {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub job_name: Option<String>,
    pub backend_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub priority: Option<JobPriority>,
    pub circuit: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<JobConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tags: Option<Vec<String>>,
}

/// Job priority
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum JobPriority {
    Low,
    Normal,
    High,
}

impl Default for JobPriority {
    fn default() -> Self {
        JobPriority::Normal
    }
}

/// Job configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct JobConfig {
    #[serde(default = "default_shots")]
    pub shots: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub seed: Option<u64>,
    #[serde(default = "default_optimization_level")]
    pub optimization_level: u8,
    #[serde(default)]
    pub error_mitigation: bool,
    #[serde(default)]
    pub dynamic_decoupling: bool,
    #[serde(default)]
    pub resilience_level: u8,
}

fn default_shots() -> usize { 1024 }
fn default_optimization_level() -> u8 { 1 }

impl Default for JobConfig {
    fn default() -> Self {
        Self {
            shots: 1024,
            seed: None,
            optimization_level: 1,
            error_mitigation: false,
            dynamic_decoupling: false,
            resilience_level: 0,
        }
    }
}

/// Job queued payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct JobQueuedPayload {
    pub job_id: String,
    pub position: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub estimated_start_time: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub estimated_duration: Option<i64>,
}

/// Job running payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct JobRunningPayload {
    pub job_id: String,
    pub backend_id: String,
    pub started_at: i64,
    #[serde(default)]
    pub progress: u8,
}

/// Job completed payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct JobCompletedPayload {
    pub job_id: String,
    pub result: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timing: Option<JobTiming>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub usage: Option<JobUsage>,
}

/// Job timing info
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct JobTiming {
    pub queued_at: i64,
    pub started_at: i64,
    pub completed_at: i64,
    pub execution_time: f64,
}

/// Job usage info
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct JobUsage {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quantum_seconds: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub credits_used: Option<f64>,
}

/// Job failed payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct JobFailedPayload {
    pub job_id: String,
    pub error: ErrorPayload,
    pub recoverable: bool,
}

/// Subscribe payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SubscribePayload {
    pub topics: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filters: Option<SubscribeFilters>,
}

/// Subscribe filters
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SubscribeFilters {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub job_ids: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub backend_ids: Option<Vec<String>>,
}

/// Error payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ErrorPayload {
    pub code: u32,
    pub name: String,
    pub message: String,
    #[serde(default)]
    pub recoverable: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub related_message_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retry_after: Option<i64>,
}

/// Backend status payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BackendStatusPayload {
    pub backend_id: String,
    pub status: BackendStatusType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub queue_length: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub estimated_queue_time: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub properties: Option<BackendProperties>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_calibration: Option<i64>,
}

/// Backend properties
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BackendProperties {
    pub num_qubits: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub basis_gates: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coupling_map: Option<Vec<[usize; 2]>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub t1: Option<Vec<f64>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub t2: Option<Vec<f64>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub readout_error: Option<Vec<f64>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gate_error: Option<HashMap<String, Vec<f64>>>,
}

/// Calibration payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CalibrationPayload {
    pub backend_id: String,
    pub calibration_id: String,
    pub timestamp: i64,
    pub data: CalibrationData,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub next_calibration: Option<i64>,
}

/// Calibration data
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CalibrationData {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub qubit_properties: Option<Vec<QubitCalibration>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gate_properties: Option<Vec<GateCalibration>>,
}

/// Qubit calibration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct QubitCalibration {
    pub qubit: usize,
    pub t1_us: f64,
    pub t2_us: f64,
    pub frequency_ghz: f64,
    pub readout_error: f64,
}

/// Gate calibration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct GateCalibration {
    pub gate: String,
    pub qubits: Vec<usize>,
    pub error: f64,
    pub duration_ns: f64,
}

/// Ping payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PingPayload {}

/// Pong payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PongPayload {
    pub server_time: i64,
}
