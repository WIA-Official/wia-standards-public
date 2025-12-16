//! WIA Security Event Types
//!
//! Core type definitions for the WIA Security Standard.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;

// ============================================================================
// Enums
// ============================================================================

/// Event types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EventType {
    Alert,
    ThreatIntel,
    Vulnerability,
    Incident,
    NetworkEvent,
    EndpointEvent,
    AuthEvent,
}

impl std::fmt::Display for EventType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EventType::Alert => write!(f, "alert"),
            EventType::ThreatIntel => write!(f, "threat_intel"),
            EventType::Vulnerability => write!(f, "vulnerability"),
            EventType::Incident => write!(f, "incident"),
            EventType::NetworkEvent => write!(f, "network_event"),
            EventType::EndpointEvent => write!(f, "endpoint_event"),
            EventType::AuthEvent => write!(f, "auth_event"),
        }
    }
}

/// Source types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceType {
    Siem,
    Edr,
    Ids,
    Firewall,
    Scanner,
    Custom,
}

/// Priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Priority {
    Critical,
    High,
    Medium,
    Low,
    Info,
}

/// Direction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Direction {
    Inbound,
    Outbound,
    Internal,
}

/// Cloud providers
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CloudProvider {
    Aws,
    Azure,
    Gcp,
}

/// Alert categories
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AlertCategory {
    Malware,
    Intrusion,
    Policy,
    Reconnaissance,
    Other,
}

/// Alert status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AlertStatus {
    New,
    Investigating,
    Resolved,
    FalsePositive,
    Closed,
}

/// Threat types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ThreatType {
    Malware,
    Apt,
    Campaign,
    Botnet,
    Ransomware,
    Phishing,
}

/// Threat status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ThreatStatus {
    Active,
    Inactive,
    Unknown,
}

/// CVSS severity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CvssSeverity {
    Critical,
    High,
    Medium,
    Low,
    None,
}

/// Incident categories
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IncidentCategory {
    Malware,
    Phishing,
    Ransomware,
    DataBreach,
    Ddos,
    UnauthorizedAccess,
    InsiderThreat,
    Apt,
    Other,
}

/// Incident status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IncidentStatus {
    New,
    Triaging,
    Investigating,
    Containing,
    Eradicating,
    Recovering,
    Closed,
}

/// Network event types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NetworkEventType {
    Connection,
    Dns,
    Http,
    Tls,
    Smtp,
    Ftp,
    Ssh,
    Rdp,
    Custom,
}

/// Network protocols
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NetworkProtocol {
    TCP,
    UDP,
    ICMP,
    GRE,
    ESP,
}

/// Auth event types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthEventType {
    LoginSuccess,
    LoginFailure,
    Logout,
    PasswordChange,
    AccountLocked,
    AccountUnlocked,
    MfaSuccess,
    MfaFailure,
    PrivilegeEscalation,
}

/// Auth result
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthResult {
    Success,
    Failure,
}

// ============================================================================
// Structs
// ============================================================================

/// Event source
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Source {
    #[serde(rename = "type")]
    pub source_type: SourceType,
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vendor: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
}

impl Source {
    pub fn new(source_type: SourceType, name: impl Into<String>) -> Self {
        Self {
            source_type,
            name: name.into(),
            vendor: None,
            version: None,
        }
    }

    pub fn with_vendor(mut self, vendor: impl Into<String>) -> Self {
        self.vendor = Some(vendor.into());
        self
    }

    pub fn with_version(mut self, version: impl Into<String>) -> Self {
        self.version = Some(version.into());
        self
    }
}

/// Operating system info
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OperatingSystem {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
}

/// Host information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Host {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hostname: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ip: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mac: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub os: Option<OperatingSystem>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub domain: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub agent_id: Option<String>,
}

/// User information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct User {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub domain: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub email: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub employee_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub groups: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub roles: Option<Vec<String>>,
}

/// Cloud context
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Cloud {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub provider: Option<CloudProvider>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub account_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub region: Option<String>,
}

/// Network endpoint
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NetworkEndpoint {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ip: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub port: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hostname: Option<String>,
}

/// Network context
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NetworkContext {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<NetworkEndpoint>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub destination: Option<NetworkEndpoint>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub protocol: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub direction: Option<Direction>,
}

/// Event context
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Context {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub host: Option<Host>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub network: Option<NetworkContext>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub user: Option<User>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cloud: Option<Cloud>,
}

/// MITRE ATT&CK mapping
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Mitre {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tactic: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tactic_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub technique: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub technique_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sub_technique: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sub_technique_name: Option<String>,
}

/// Event metadata
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Meta {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tags: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub labels: Option<HashMap<String, String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub raw: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub custom: Option<serde_json::Value>,
}

/// Indicator
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Indicator {
    #[serde(rename = "type")]
    pub indicator_type: String,
    pub value: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub context: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub first_seen: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_seen: Option<String>,
}

// ============================================================================
// Event Data Types
// ============================================================================

/// Alert data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlertData {
    pub alert_id: String,
    pub title: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub category: String,
    pub status: String,
    pub priority: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub assignee: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub detection_rule: Option<DetectionRule>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub indicators: Option<Vec<Indicator>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub first_seen: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_seen: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub count: Option<u32>,
}

/// Detection rule
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionRule {
    pub id: String,
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
}

/// Threat intel data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThreatIntelData {
    pub threat_type: String,
    pub threat_name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub threat_family: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aliases: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub first_seen: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_seen: Option<String>,
    pub status: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub indicators: Option<Vec<serde_json::Value>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ttps: Option<Vec<serde_json::Value>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_sectors: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_countries: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub references: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub report_id: Option<String>,
}

/// CVSS score
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Cvss {
    pub version: String,
    pub score: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vector: Option<String>,
    pub severity: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub base_score: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temporal_score: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub environmental_score: Option<f64>,
}

/// Vulnerability data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VulnerabilityData {
    pub vuln_id: String,
    pub title: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub cvss: Cvss,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cwe: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub affected_products: Option<Vec<serde_json::Value>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exploit_available: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exploit_details: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub patch_available: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub patch_details: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub references: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub published: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub modified: Option<String>,
}

/// Incident data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IncidentData {
    pub incident_id: String,
    pub title: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub category: String,
    pub status: String,
    pub priority: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub impact: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeline: Option<Vec<serde_json::Value>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub affected_assets: Option<Vec<serde_json::Value>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub iocs: Option<Vec<serde_json::Value>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub response_actions: Option<Vec<serde_json::Value>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub root_cause: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lessons_learned: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub updated_at: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub closed_at: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lead_analyst: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub team: Option<Vec<String>>,
}

/// Network event data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NetworkEventData {
    pub event_type: String,
    pub protocol: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub direction: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub action: Option<String>,
    pub source: serde_json::Value,
    pub destination: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bytes_sent: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bytes_received: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub packets_sent: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub packets_received: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration_ms: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rule_matched: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub application: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub http: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dns: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tls: Option<serde_json::Value>,
}

/// Endpoint event data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EndpointEventData {
    pub event_type: String,
    pub host: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub process: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub file: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub registry: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub network_connection: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dll: Option<serde_json::Value>,
}

/// Auth event data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuthEventData {
    pub event_type: String,
    pub result: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub failure_reason: Option<String>,
    pub user: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<serde_json::Value>,
    pub target: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auth_method: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mfa_used: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mfa_method: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub session_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub logon_type: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attempt_count: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub previous_login: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub risk_score: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub risk_factors: Option<Vec<String>>,
}

// ============================================================================
// Main Event
// ============================================================================

/// WIA Security Event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaSecurityEvent {
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,
    pub version: String,
    pub id: Uuid,
    #[serde(rename = "type")]
    pub event_type: EventType,
    pub timestamp: String,
    pub severity: f64,
    pub source: Source,
    pub data: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub context: Option<Context>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mitre: Option<Mitre>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub meta: Option<Meta>,
}

impl WiaSecurityEvent {
    /// Create a new event
    pub fn new(event_type: EventType, source: Source, data: serde_json::Value) -> Self {
        Self {
            schema: Some(crate::SCHEMA_URL.to_string()),
            version: crate::VERSION.to_string(),
            id: Uuid::new_v4(),
            event_type,
            timestamp: Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string(),
            severity: 5.0,
            source,
            data,
            context: None,
            mitre: None,
            meta: None,
        }
    }

    /// Convert to JSON string
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Parse from JSON string
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }
}
