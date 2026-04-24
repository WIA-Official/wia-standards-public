//! SOAR Platform Integration
//!
//! Splunk Phantom, Palo Alto XSOAR, Swimlane

use crate::types::WiaSecurityEvent;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ============================================================================
// Types
// ============================================================================

/// SOAR platform configuration
#[derive(Debug, Clone)]
pub struct SoarConfig {
    pub url: String,
    pub api_key: String,
    pub verify_tls: bool,
}

/// SOAR action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SoarAction {
    pub name: String,
    pub app: String,
    pub parameters: HashMap<String, serde_json::Value>,
}

/// Playbook execution request
#[derive(Debug, Clone)]
pub struct PlaybookRequest {
    pub playbook_name: String,
    pub container_id: String,
    pub parameters: HashMap<String, serde_json::Value>,
}

/// Playbook execution result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlaybookResult {
    pub run_id: String,
    pub status: PlaybookStatus,
    pub message: Option<String>,
}

/// Playbook status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlaybookStatus {
    Pending,
    Running,
    Success,
    Failed,
    Cancelled,
}

/// Result type for SOAR operations
pub type SoarResult<T> = Result<T, SoarError>;

/// SOAR integration error
#[derive(Debug)]
pub enum SoarError {
    ApiError { status: u16, message: String },
    PlaybookNotFound(String),
    ContainerNotFound(String),
    NetworkError(String),
    AuthenticationError(String),
}

impl std::fmt::Display for SoarError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ApiError { status, message } => write!(f, "API error ({}): {}", status, message),
            Self::PlaybookNotFound(name) => write!(f, "Playbook not found: {}", name),
            Self::ContainerNotFound(id) => write!(f, "Container not found: {}", id),
            Self::NetworkError(msg) => write!(f, "Network error: {}", msg),
            Self::AuthenticationError(msg) => write!(f, "Authentication error: {}", msg),
        }
    }
}

impl std::error::Error for SoarError {}

// ============================================================================
// Splunk Phantom Client
// ============================================================================

/// Splunk Phantom configuration
#[derive(Debug, Clone)]
pub struct PhantomConfig {
    pub url: String,
    pub api_key: String,
    pub verify_tls: bool,
}

/// Splunk Phantom client
pub struct PhantomClient {
    _config: PhantomConfig,
}

impl PhantomClient {
    /// Create a new Phantom client
    pub fn new(config: PhantomConfig) -> Self {
        Self { _config: config }
    }

    /// Create container from WIA Security event
    pub async fn create_container(&self, event: &WiaSecurityEvent) -> SoarResult<String> {
        let container = serde_json::json!({
            "name": self.extract_title(event),
            "description": self.extract_description(event),
            "label": "wia_security",
            "severity": self.map_severity(event.severity),
            "status": "new",
            "source_data_identifier": event.id.to_string(),
            "custom_fields": {
                "wia_event_type": event.event_type.to_string(),
                "wia_severity": event.severity,
            }
        });

        // In production, make HTTP request
        // POST {url}/rest/container
        let _ = container;
        Ok(format!(
            "phantom-container-{}",
            chrono::Utc::now().timestamp()
        ))
    }

    /// Run playbook on container
    pub async fn run_playbook(
        &self,
        playbook_name: &str,
        container_id: &str,
    ) -> SoarResult<PlaybookResult> {
        let _body = serde_json::json!({
            "playbook_id": playbook_name,
            "container_id": container_id,
            "scope": "new",
        });

        // In production, make HTTP request
        // POST {url}/rest/playbook_run
        Ok(PlaybookResult {
            run_id: format!("run-{}", chrono::Utc::now().timestamp()),
            status: PlaybookStatus::Running,
            message: None,
        })
    }

    /// Add artifact to container
    pub async fn add_artifact(
        &self,
        container_id: &str,
        artifact: PhantomArtifact,
    ) -> SoarResult<String> {
        let _body = serde_json::json!({
            "container_id": container_id,
            "name": artifact.name,
            "label": artifact.label,
            "severity": artifact.severity,
            "cef": artifact.cef,
        });

        // In production, make HTTP request
        // POST {url}/rest/artifact
        Ok(format!("artifact-{}", chrono::Utc::now().timestamp()))
    }

    /// Get playbook run status
    pub async fn get_playbook_status(&self, run_id: &str) -> SoarResult<PlaybookResult> {
        // In production, make HTTP request
        // GET {url}/rest/playbook_run/{run_id}
        Ok(PlaybookResult {
            run_id: run_id.to_string(),
            status: PlaybookStatus::Success,
            message: None,
        })
    }

    /// Run action
    pub async fn run_action(&self, container_id: &str, action: SoarAction) -> SoarResult<String> {
        let _body = serde_json::json!({
            "action": action.name,
            "app": action.app,
            "container_id": container_id,
            "parameters": [action.parameters],
        });

        // In production, make HTTP request
        // POST {url}/rest/action_run
        Ok(format!("action-{}", chrono::Utc::now().timestamp()))
    }

    fn extract_title(&self, event: &WiaSecurityEvent) -> String {
        event
            .data
            .get("title")
            .and_then(|v| v.as_str())
            .unwrap_or("WIA Security Event")
            .to_string()
    }

    fn extract_description(&self, event: &WiaSecurityEvent) -> String {
        event
            .data
            .get("description")
            .and_then(|v| v.as_str())
            .unwrap_or("")
            .to_string()
    }

    fn map_severity(&self, severity: f64) -> &'static str {
        if severity >= 9.0 {
            "critical"
        } else if severity >= 7.0 {
            "high"
        } else if severity >= 4.0 {
            "medium"
        } else {
            "low"
        }
    }
}

/// Phantom artifact
#[derive(Debug, Clone)]
pub struct PhantomArtifact {
    pub name: String,
    pub label: String,
    pub severity: String,
    pub cef: HashMap<String, serde_json::Value>,
}

// ============================================================================
// Palo Alto XSOAR Client
// ============================================================================

/// XSOAR configuration
#[derive(Debug, Clone)]
pub struct XsoarConfig {
    pub url: String,
    pub api_key: String,
    pub verify_tls: bool,
}

/// Palo Alto XSOAR client
pub struct XsoarClient {
    _config: XsoarConfig,
}

impl XsoarClient {
    /// Create a new XSOAR client
    pub fn new(config: XsoarConfig) -> Self {
        Self { _config: config }
    }

    /// Create incident from WIA Security event
    pub async fn create_incident(&self, event: &WiaSecurityEvent) -> SoarResult<String> {
        let incident = serde_json::json!({
            "name": self.extract_title(event),
            "type": "WIA Security",
            "severity": self.map_severity(event.severity),
            "labels": [
                { "type": "wia_event_type", "value": event.event_type.to_string() },
            ],
            "customFields": {
                "wiaeventid": event.id.to_string(),
                "wiaseverity": event.severity,
            },
            "rawJSON": serde_json::to_string(&event.data).unwrap_or_default(),
        });

        // In production, make HTTP request
        // POST {url}/incident
        let _ = incident;
        Ok(format!("xsoar-incident-{}", chrono::Utc::now().timestamp()))
    }

    /// Run playbook on incident
    pub async fn run_playbook(
        &self,
        playbook_id: &str,
        incident_id: &str,
    ) -> SoarResult<PlaybookResult> {
        let _body = serde_json::json!({
            "playbookId": playbook_id,
            "incidentId": incident_id,
        });

        // In production, make HTTP request
        // POST {url}/inv-playbook
        Ok(PlaybookResult {
            run_id: format!("run-{}", chrono::Utc::now().timestamp()),
            status: PlaybookStatus::Running,
            message: None,
        })
    }

    /// Add entry to incident war room
    pub async fn add_entry(&self, incident_id: &str, entry: XsoarEntry) -> SoarResult<String> {
        let _body = serde_json::json!({
            "investigationId": incident_id,
            "data": entry.data,
            "categories": entry.categories,
        });

        // In production, make HTTP request
        // POST {url}/entry
        Ok(format!("entry-{}", chrono::Utc::now().timestamp()))
    }

    /// Search incidents
    pub async fn search_incidents(
        &self,
        query: &str,
        size: usize,
    ) -> SoarResult<Vec<XsoarIncident>> {
        let _body = serde_json::json!({
            "filter": {
                "query": query,
            },
            "size": size,
        });

        // In production, make HTTP request
        // POST {url}/incidents/search
        Ok(vec![])
    }

    /// Close incident
    pub async fn close_incident(
        &self,
        incident_id: &str,
        close_reason: &str,
        close_notes: &str,
    ) -> SoarResult<()> {
        let _body = serde_json::json!({
            "id": incident_id,
            "closeReason": close_reason,
            "closeNotes": close_notes,
        });

        // In production, make HTTP request
        // POST {url}/incident/close
        Ok(())
    }

    fn extract_title(&self, event: &WiaSecurityEvent) -> String {
        event
            .data
            .get("title")
            .and_then(|v| v.as_str())
            .unwrap_or("WIA Security Event")
            .to_string()
    }

    fn map_severity(&self, severity: f64) -> u8 {
        if severity >= 9.0 {
            4 // Critical
        } else if severity >= 7.0 {
            3 // High
        } else if severity >= 4.0 {
            2 // Medium
        } else {
            1 // Low
        }
    }
}

/// XSOAR entry
#[derive(Debug, Clone)]
pub struct XsoarEntry {
    pub data: String,
    pub categories: Vec<String>,
}

/// XSOAR incident
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct XsoarIncident {
    pub id: String,
    pub name: String,
    pub severity: u8,
    pub status: String,
    pub created: String,
}

// ============================================================================
// Swimlane Client
// ============================================================================

/// Swimlane configuration
#[derive(Debug, Clone)]
pub struct SwimlaneConfig {
    pub url: String,
    pub api_key: String,
}

/// Swimlane client
pub struct SwimlaneClient {
    _config: SwimlaneConfig,
}

impl SwimlaneClient {
    /// Create a new Swimlane client
    pub fn new(config: SwimlaneConfig) -> Self {
        Self { _config: config }
    }

    /// Create record in application
    pub async fn create_record(
        &self,
        app_id: &str,
        event: &WiaSecurityEvent,
    ) -> SoarResult<String> {
        let _record = serde_json::json!({
            "applicationId": app_id,
            "values": {
                "title": self.extract_title(event),
                "description": self.extract_description(event),
                "severity": event.severity,
                "eventType": event.event_type.to_string(),
                "eventId": event.id.to_string(),
            }
        });

        // In production, make HTTP request
        // POST {url}/api/app/{app_id}/record
        Ok(format!(
            "swimlane-record-{}",
            chrono::Utc::now().timestamp()
        ))
    }

    /// Execute workflow
    pub async fn execute_workflow(&self, workflow_id: &str, record_id: &str) -> SoarResult<String> {
        let _body = serde_json::json!({
            "workflowId": workflow_id,
            "recordId": record_id,
        });

        // In production, make HTTP request
        Ok(format!("workflow-run-{}", chrono::Utc::now().timestamp()))
    }

    fn extract_title(&self, event: &WiaSecurityEvent) -> String {
        event
            .data
            .get("title")
            .and_then(|v| v.as_str())
            .unwrap_or("WIA Security Event")
            .to_string()
    }

    fn extract_description(&self, event: &WiaSecurityEvent) -> String {
        event
            .data
            .get("description")
            .and_then(|v| v.as_str())
            .unwrap_or("")
            .to_string()
    }
}

// ============================================================================
// Factory Functions
// ============================================================================

/// Create Phantom client
pub fn create_phantom_client(config: PhantomConfig) -> PhantomClient {
    PhantomClient::new(config)
}

/// Create XSOAR client
pub fn create_xsoar_client(config: XsoarConfig) -> XsoarClient {
    XsoarClient::new(config)
}

/// Create Swimlane client
pub fn create_swimlane_client(config: SwimlaneConfig) -> SwimlaneClient {
    SwimlaneClient::new(config)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_playbook_status() {
        let result = PlaybookResult {
            run_id: "test-123".to_string(),
            status: PlaybookStatus::Success,
            message: Some("Completed".to_string()),
        };
        assert_eq!(result.status, PlaybookStatus::Success);
    }
}
