//! WRP message types and structures

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};

/// WRP Message - the core message type for WIA Robot Protocol
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WrpMessage {
    /// Protocol identifier (always "wia-robot")
    pub protocol: String,
    /// Protocol version (SemVer)
    pub version: String,
    /// Unique message ID (UUID v4)
    pub message_id: String,
    /// UTC timestamp
    pub timestamp: DateTime<Utc>,
    /// Message sequence number
    pub sequence: u64,
    /// Message type
    #[serde(rename = "type")]
    pub message_type: MessageType,
    /// Message priority
    pub priority: Priority,
    /// Source endpoint
    pub source: Endpoint,
    /// Destination endpoint
    pub destination: Endpoint,
    /// Safety information
    pub safety: SafetyInfo,
    /// Message payload
    pub payload: serde_json::Value,
    /// CRC32 checksum (hex)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub checksum: Option<String>,
}

/// Message type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    Handshake,
    HandshakeAck,
    Heartbeat,
    Telemetry,
    Control,
    ControlAck,
    EmergencyStop,
    EmergencyStopAck,
    SafetyAlert,
    Status,
    Config,
    ConfigAck,
    Error,
    Log,
}

impl MessageType {
    /// Get default priority for this message type
    pub fn default_priority(&self) -> Priority {
        match self {
            MessageType::EmergencyStop | MessageType::EmergencyStopAck => Priority::Emergency,
            MessageType::SafetyAlert => Priority::Critical,
            MessageType::Control | MessageType::ControlAck |
            MessageType::Config | MessageType::ConfigAck |
            MessageType::Handshake | MessageType::HandshakeAck |
            MessageType::Error => Priority::High,
            MessageType::Telemetry | MessageType::Status | MessageType::Heartbeat => Priority::Normal,
            MessageType::Log => Priority::Low,
        }
    }

    /// Check if this message type requires acknowledgment by default
    pub fn requires_ack(&self) -> bool {
        matches!(
            self,
            MessageType::Control |
            MessageType::Config |
            MessageType::EmergencyStop
        )
    }
}

/// Message priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Priority {
    Low = 0,
    Normal = 1,
    High = 2,
    Critical = 3,
    Emergency = 4,
}

impl Default for Priority {
    fn default() -> Self {
        Priority::Normal
    }
}

/// Endpoint representing a device or system
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Endpoint {
    /// Device identifier
    pub device_id: String,
    /// Device type
    pub device_type: String,
    /// Optional location
    #[serde(skip_serializing_if = "Option::is_none")]
    pub location: Option<Location>,
}

impl Endpoint {
    /// Create a new endpoint
    pub fn new(device_id: &str, device_type: &str) -> Self {
        Self {
            device_id: device_id.to_string(),
            device_type: device_type.to_string(),
            location: None,
        }
    }

    /// Create a broadcast endpoint
    pub fn broadcast() -> Self {
        Self {
            device_id: "broadcast".to_string(),
            device_type: "all".to_string(),
            location: None,
        }
    }

    /// Add location to endpoint
    pub fn with_location(mut self, lat: f64, lon: f64) -> Self {
        self.location = Some(Location {
            latitude: lat,
            longitude: lon,
            altitude_m: None,
        });
        self
    }
}

/// Geographic location
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Location {
    /// Latitude in degrees
    pub latitude: f64,
    /// Longitude in degrees
    pub longitude: f64,
    /// Altitude in meters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub altitude_m: Option<f64>,
}

/// Safety information attached to every message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyInfo {
    /// Emergency stop active
    pub emergency_stop: bool,
    /// Current safety level
    pub safety_level: SafetyLevel,
    /// Whether acknowledgment is required
    #[serde(default)]
    pub requires_ack: bool,
    /// Acknowledgment timeout in milliseconds
    #[serde(default = "default_ack_timeout")]
    pub ack_timeout_ms: u64,
}

fn default_ack_timeout() -> u64 {
    1000
}

impl Default for SafetyInfo {
    fn default() -> Self {
        Self {
            emergency_stop: false,
            safety_level: SafetyLevel::Normal,
            requires_ack: false,
            ack_timeout_ms: 1000,
        }
    }
}

impl SafetyInfo {
    /// Create safe default info
    pub fn safe() -> Self {
        Self::default()
    }

    /// Create emergency stop info
    pub fn emergency() -> Self {
        Self {
            emergency_stop: true,
            safety_level: SafetyLevel::Emergency,
            requires_ack: true,
            ack_timeout_ms: 100,
        }
    }
}

/// Safety levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum SafetyLevel {
    Normal = 0,
    Warning = 1,
    Caution = 2,
    Critical = 3,
    Emergency = 4,
}

impl Default for SafetyLevel {
    fn default() -> Self {
        SafetyLevel::Normal
    }
}

impl SafetyLevel {
    /// Check if this level requires immediate attention
    pub fn is_urgent(&self) -> bool {
        matches!(self, SafetyLevel::Critical | SafetyLevel::Emergency)
    }
}

/// E-Stop source codes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EStopReason {
    UserButton,
    SoftwareCommand,
    FallDetected,
    ForceExceeded,
    ThermalExceeded,
    BatteryCritical,
    CommunicationLost,
    ExternalTrigger,
    WatchdogTimeout,
    CollisionDetected,
    WorkspaceViolation,
    Unknown,
}

impl WrpMessage {
    /// Calculate CRC32 checksum
    pub fn calculate_checksum(&mut self) {
        let mut msg = self.clone();
        msg.checksum = None;
        let serialized = serde_json::to_string(&msg).unwrap_or_default();
        let checksum = crc32fast::hash(serialized.as_bytes());
        self.checksum = Some(format!("{:08x}", checksum));
    }

    /// Verify checksum
    pub fn verify_checksum(&self) -> bool {
        match &self.checksum {
            Some(expected) => {
                let mut msg = self.clone();
                msg.checksum = None;
                let serialized = serde_json::to_string(&msg).unwrap_or_default();
                let actual = format!("{:08x}", crc32fast::hash(serialized.as_bytes()));
                &actual == expected
            }
            None => true, // No checksum = pass
        }
    }

    /// Check if this is an emergency message
    pub fn is_emergency(&self) -> bool {
        self.safety.emergency_stop || self.priority == Priority::Emergency
    }

    /// Check if acknowledgment is required
    pub fn requires_ack(&self) -> bool {
        self.safety.requires_ack
    }

    /// Serialize to JSON
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }

    /// Serialize to pretty JSON
    pub fn to_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Deserialize from JSON
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_type_priority() {
        assert_eq!(MessageType::EmergencyStop.default_priority(), Priority::Emergency);
        assert_eq!(MessageType::Control.default_priority(), Priority::High);
        assert_eq!(MessageType::Telemetry.default_priority(), Priority::Normal);
        assert_eq!(MessageType::Log.default_priority(), Priority::Low);
    }

    #[test]
    fn test_safety_level_urgent() {
        assert!(!SafetyLevel::Normal.is_urgent());
        assert!(!SafetyLevel::Warning.is_urgent());
        assert!(SafetyLevel::Critical.is_urgent());
        assert!(SafetyLevel::Emergency.is_urgent());
    }

    #[test]
    fn test_endpoint_creation() {
        let endpoint = Endpoint::new("exo-001", "exoskeleton");
        assert_eq!(endpoint.device_id, "exo-001");
        assert_eq!(endpoint.device_type, "exoskeleton");
        assert!(endpoint.location.is_none());

        let with_location = endpoint.with_location(37.5665, 126.978);
        assert!(with_location.location.is_some());
    }

    #[test]
    fn test_broadcast_endpoint() {
        let broadcast = Endpoint::broadcast();
        assert_eq!(broadcast.device_id, "broadcast");
        assert_eq!(broadcast.device_type, "all");
    }

    #[test]
    fn test_safety_info_defaults() {
        let safe = SafetyInfo::safe();
        assert!(!safe.emergency_stop);
        assert_eq!(safe.safety_level, SafetyLevel::Normal);

        let emergency = SafetyInfo::emergency();
        assert!(emergency.emergency_stop);
        assert_eq!(emergency.safety_level, SafetyLevel::Emergency);
        assert!(emergency.requires_ack);
    }
}
