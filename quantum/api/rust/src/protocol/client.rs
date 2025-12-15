//! Quantum protocol client

use crate::circuit::QuantumCircuit;
use crate::error::{QuantumError, Result};
use crate::types::ExecutionResult;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{Mutex, RwLock};
use uuid::Uuid;

use super::message::*;
use super::transport::{ConnectionState, MockTransport, Transport, TransportConfig};
use super::{PROTOCOL_ID, PROTOCOL_VERSION};

/// Client configuration
#[derive(Debug, Clone)]
pub struct ClientConfig {
    /// Server URL
    pub url: String,
    /// API key for authentication
    pub api_key: Option<String>,
    /// Client ID
    pub client_id: String,
    /// Client name
    pub client_name: Option<String>,
    /// Transport configuration
    pub transport: TransportConfig,
}

impl Default for ClientConfig {
    fn default() -> Self {
        Self {
            url: "wss://quantum.wia.live/wia-quantum".to_string(),
            api_key: None,
            client_id: Uuid::new_v4().to_string(),
            client_name: Some("WIA Quantum SDK".to_string()),
            transport: TransportConfig::default(),
        }
    }
}

/// Job status
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum JobState {
    Queued { position: usize },
    Running { progress: u8 },
    Completed,
    Failed { error: String },
    Cancelled,
}

/// Submitted job handle
#[derive(Debug, Clone)]
pub struct JobHandle {
    pub job_id: String,
    pub backend_id: String,
    pub submitted_at: i64,
}

/// Quantum protocol client
pub struct Client<T: Transport = MockTransport> {
    config: ClientConfig,
    transport: Arc<Mutex<T>>,
    session_id: RwLock<Option<String>>,
    pending_jobs: RwLock<HashMap<String, JobState>>,
    available_backends: RwLock<Vec<BackendInfo>>,
}

impl Client<MockTransport> {
    /// Create a new client with mock transport (for testing)
    pub fn new_mock(config: ClientConfig) -> Self {
        Self {
            config,
            transport: Arc::new(Mutex::new(MockTransport::new())),
            session_id: RwLock::new(None),
            pending_jobs: RwLock::new(HashMap::new()),
            available_backends: RwLock::new(Vec::new()),
        }
    }
}

impl<T: Transport> Client<T> {
    /// Create a new client with custom transport
    pub fn with_transport(config: ClientConfig, transport: T) -> Self {
        Self {
            config,
            transport: Arc::new(Mutex::new(transport)),
            session_id: RwLock::new(None),
            pending_jobs: RwLock::new(HashMap::new()),
            available_backends: RwLock::new(Vec::new()),
        }
    }

    /// Connect to the quantum backend
    pub async fn connect(&self) -> Result<()> {
        let mut transport = self.transport.lock().await;

        // Connect transport
        transport.connect(&self.config.url).await?;

        // Send connect message
        let auth = self.config.api_key.as_ref().map(|key| AuthInfo {
            auth_type: "api_key".to_string(),
            token: key.clone(),
        });

        let connect_payload = ConnectPayload {
            client_id: self.config.client_id.clone(),
            client_name: self.config.client_name.clone(),
            client_version: Some(env!("CARGO_PKG_VERSION").to_string()),
            capabilities: Some(vec![
                "circuits".to_string(),
                "jobs".to_string(),
                "calibration".to_string(),
            ]),
            auth,
            options: None,
        };

        let message = Message::new(MessageType::Connect, connect_payload);
        transport.send(&message).await?;

        // In a real implementation, we would wait for connect_ack
        // For now, we just set connected state

        Ok(())
    }

    /// Disconnect from the quantum backend
    pub async fn disconnect(&self) -> Result<()> {
        let mut transport = self.transport.lock().await;

        // Send disconnect message
        let message = Message::new(MessageType::Disconnect, serde_json::json!({}));
        transport.send(&message).await?;

        // Disconnect transport
        transport.disconnect().await?;

        // Clear session
        *self.session_id.write().await = None;

        Ok(())
    }

    /// Check if connected
    pub async fn is_connected(&self) -> bool {
        let transport = self.transport.lock().await;
        transport.is_connected()
    }

    /// Get connection state
    pub async fn state(&self) -> ConnectionState {
        let transport = self.transport.lock().await;
        transport.state()
    }

    /// Submit a quantum job
    pub async fn submit_job(
        &self,
        circuit: &QuantumCircuit,
        backend_id: &str,
        config: Option<JobConfig>,
    ) -> Result<JobHandle> {
        let transport = self.transport.lock().await;

        if !transport.is_connected() {
            return Err(QuantumError::ConnectionError(
                "Not connected to backend".to_string(),
            ));
        }

        let submit_payload = SubmitJobPayload {
            job_name: circuit.name.clone(),
            backend_id: backend_id.to_string(),
            priority: Some(JobPriority::Normal),
            circuit: serde_json::to_value(circuit.to_data()).unwrap_or_default(),
            config,
            tags: None,
        };

        let message = Message::new(MessageType::SubmitJob, submit_payload);
        transport.send(&message).await?;

        // In a real implementation, we would wait for job_queued response
        // For now, create a mock job handle
        let job_id = Uuid::new_v4().to_string();
        let handle = JobHandle {
            job_id: job_id.clone(),
            backend_id: backend_id.to_string(),
            submitted_at: chrono::Utc::now().timestamp_millis(),
        };

        // Track job
        self.pending_jobs
            .write()
            .await
            .insert(job_id, JobState::Queued { position: 0 });

        Ok(handle)
    }

    /// Get job status
    pub async fn get_job_status(&self, job_id: &str) -> Result<JobState> {
        let jobs = self.pending_jobs.read().await;
        jobs.get(job_id)
            .cloned()
            .ok_or_else(|| QuantumError::JobError(format!("Job not found: {}", job_id)))
    }

    /// Cancel a job
    pub async fn cancel_job(&self, job_id: &str) -> Result<()> {
        let transport = self.transport.lock().await;

        if !transport.is_connected() {
            return Err(QuantumError::ConnectionError(
                "Not connected to backend".to_string(),
            ));
        }

        let message = Message::new(
            MessageType::CancelJob,
            serde_json::json!({ "jobId": job_id }),
        );
        transport.send(&message).await?;

        // Update local state
        self.pending_jobs
            .write()
            .await
            .insert(job_id.to_string(), JobState::Cancelled);

        Ok(())
    }

    /// Subscribe to topics
    pub async fn subscribe(&self, topics: Vec<String>) -> Result<()> {
        let transport = self.transport.lock().await;

        if !transport.is_connected() {
            return Err(QuantumError::ConnectionError(
                "Not connected to backend".to_string(),
            ));
        }

        let subscribe_payload = SubscribePayload {
            topics,
            filters: None,
        };

        let message = Message::new(MessageType::Subscribe, subscribe_payload);
        transport.send(&message).await?;

        Ok(())
    }

    /// Get available backends
    pub async fn get_backends(&self) -> Vec<BackendInfo> {
        self.available_backends.read().await.clone()
    }

    /// Send ping
    pub async fn ping(&self) -> Result<()> {
        let transport = self.transport.lock().await;

        if !transport.is_connected() {
            return Err(QuantumError::ConnectionError(
                "Not connected to backend".to_string(),
            ));
        }

        let message = Message::new(MessageType::Ping, PingPayload {});
        transport.send(&message).await?;

        Ok(())
    }

    /// Handle incoming message
    pub async fn handle_message(&self, message: Message) -> Result<()> {
        match message.message_type {
            MessageType::ConnectAck => {
                if let Ok(payload) = serde_json::from_value::<ConnectAckPayload>(message.payload) {
                    if payload.success {
                        *self.session_id.write().await = payload.session_id;
                        if let Some(backends) = payload.available_backends {
                            *self.available_backends.write().await = backends;
                        }
                    }
                }
            }
            MessageType::JobQueued => {
                if let Ok(payload) = serde_json::from_value::<JobQueuedPayload>(message.payload) {
                    self.pending_jobs.write().await.insert(
                        payload.job_id,
                        JobState::Queued {
                            position: payload.position,
                        },
                    );
                }
            }
            MessageType::JobRunning => {
                if let Ok(payload) = serde_json::from_value::<JobRunningPayload>(message.payload) {
                    self.pending_jobs.write().await.insert(
                        payload.job_id,
                        JobState::Running {
                            progress: payload.progress,
                        },
                    );
                }
            }
            MessageType::JobCompleted => {
                if let Ok(payload) = serde_json::from_value::<JobCompletedPayload>(message.payload)
                {
                    self.pending_jobs
                        .write()
                        .await
                        .insert(payload.job_id, JobState::Completed);
                }
            }
            MessageType::JobFailed => {
                if let Ok(payload) = serde_json::from_value::<JobFailedPayload>(message.payload) {
                    self.pending_jobs.write().await.insert(
                        payload.job_id,
                        JobState::Failed {
                            error: payload.error.message,
                        },
                    );
                }
            }
            MessageType::JobCancelled => {
                if let Some(job_id) = message.payload.get("jobId").and_then(|v| v.as_str()) {
                    self.pending_jobs
                        .write()
                        .await
                        .insert(job_id.to_string(), JobState::Cancelled);
                }
            }
            MessageType::BackendStatus => {
                // Update backend status
            }
            MessageType::Error => {
                if let Ok(error) = serde_json::from_value::<ErrorPayload>(message.payload) {
                    return Err(QuantumError::ProtocolError(format!(
                        "{}: {}",
                        error.name, error.message
                    )));
                }
            }
            _ => {}
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_client_connect() {
        let config = ClientConfig::default();
        let client = Client::new_mock(config);

        client.connect().await.unwrap();
        assert!(client.is_connected().await);

        client.disconnect().await.unwrap();
        assert!(!client.is_connected().await);
    }

    #[tokio::test]
    async fn test_submit_job() {
        let config = ClientConfig::default();
        let client = Client::new_mock(config);

        client.connect().await.unwrap();

        let mut circuit = QuantumCircuit::new(2, 2);
        circuit.h(0).unwrap();
        circuit.cx(0, 1).unwrap();
        circuit.measure_all().unwrap();

        let handle = client
            .submit_job(&circuit, "simulator_statevector", None)
            .await
            .unwrap();

        assert!(!handle.job_id.is_empty());
        assert_eq!(handle.backend_id, "simulator_statevector");
    }
}
