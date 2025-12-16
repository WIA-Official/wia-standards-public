//! WIA ecosystem bridge for cross-system communication

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;

use crate::types::{PoseData, SignLanguageCode};

/// WIA system identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum WiaSystem {
    VoiceSign,
    Exoskeleton,
    BionicEye,
}

impl std::fmt::Display for WiaSystem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::VoiceSign => write!(f, "voice-sign"),
            Self::Exoskeleton => write!(f, "exoskeleton"),
            Self::BionicEye => write!(f, "bionic-eye"),
        }
    }
}

/// Sign gesture command for exoskeleton
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignGestureCommand {
    /// Unique command ID
    pub command_id: String,

    /// Source request ID from voice-sign
    pub source_request_id: String,

    /// Sign being produced
    pub sign: SignInfo,

    /// Target joint angles
    pub joint_targets: Vec<JointTarget>,

    /// Timing information
    pub timing: GestureTiming,

    /// Safety parameters
    pub safety: GestureSafety,
}

/// Sign information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignInfo {
    /// Gloss representation
    pub gloss: String,

    /// Notation (HamNoSys)
    pub notation: Option<String>,

    /// Hand shape
    pub hand_shape: HandShape,

    /// Movement type
    pub movement_type: MovementType,
}

/// Hand shape
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HandShape {
    Open,
    Closed,
    Pointing,
    Flat,
    Curved,
    Custom,
}

/// Movement type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MovementType {
    Static,
    Linear,
    Circular,
    Oscillating,
    Complex,
}

/// Joint target
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointTarget {
    /// Joint identifier
    pub joint_id: String,

    /// Target angle in degrees
    pub angle_degrees: f32,

    /// Maximum velocity
    pub velocity_limit: f32,

    /// Maximum torque
    pub torque_limit: f32,
}

/// Gesture timing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GestureTiming {
    /// Start delay in milliseconds
    pub start_delay_ms: u32,

    /// Duration in milliseconds
    pub duration_ms: u32,

    /// Transition type
    pub transition: TransitionType,
}

/// Transition type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum TransitionType {
    Smooth,
    Sharp,
    Hold,
}

/// Gesture safety parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GestureSafety {
    /// Maximum force in Newtons
    pub max_force_n: f32,

    /// Emergency stop enabled
    pub emergency_stop: bool,

    /// User can override
    pub user_override: bool,

    /// Collision detection enabled
    pub collision_detection: bool,
}

impl Default for GestureSafety {
    fn default() -> Self {
        Self {
            max_force_n: 10.0,
            emergency_stop: true,
            user_override: true,
            collision_detection: true,
        }
    }
}

/// Display request for bionic eye
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BionicEyeDisplayRequest {
    /// Request ID
    pub request_id: String,

    /// Display content
    pub content: DisplayContent,

    /// Display parameters
    pub parameters: DisplayParameters,

    /// Priority
    pub priority: DisplayPriority,
}

/// Display content
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum DisplayContent {
    /// Sign language avatar
    SignAvatar {
        pose_data: PoseData,
        loop_count: u32,
    },

    /// Gloss text overlay
    GlossText {
        glosses: Vec<String>,
        highlight_current: bool,
    },

    /// Caption text
    Caption { text: String, language: String },

    /// Combined display
    Combined {
        avatar: Option<PoseData>,
        gloss: Option<Vec<String>>,
        caption: Option<String>,
    },
}

/// Display parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisplayParameters {
    /// Position in visual field
    pub position: DisplayPosition,

    /// Size (percentage of field)
    pub size_percent: f32,

    /// Opacity (0.0 - 1.0)
    pub opacity: f32,

    /// Duration in ms (0 = until dismissed)
    pub duration_ms: u32,
}

impl Default for DisplayParameters {
    fn default() -> Self {
        Self {
            position: DisplayPosition::BottomRight,
            size_percent: 25.0,
            opacity: 1.0,
            duration_ms: 0,
        }
    }
}

/// Display position
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisplayPosition {
    Center,
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,
    Peripheral,
}

/// Display priority
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DisplayPriority {
    Background,
    Normal,
    Important,
    Urgent,
    Emergency,
}

/// WIA Bridge for cross-system communication
pub struct WiaBridge {
    config: BridgeConfig,
    exoskeleton_client: Option<Arc<dyn ExoskeletonClient>>,
    bionic_eye_client: Option<Arc<dyn BionicEyeClient>>,
}

/// Bridge configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BridgeConfig {
    /// Enable exoskeleton integration
    pub exoskeleton_enabled: bool,

    /// Enable bionic eye integration
    pub bionic_eye_enabled: bool,

    /// Event bus URL
    pub event_bus_url: Option<String>,

    /// Timeout in milliseconds
    pub timeout_ms: u64,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            exoskeleton_enabled: false,
            bionic_eye_enabled: false,
            event_bus_url: None,
            timeout_ms: 5000,
        }
    }
}

impl WiaBridge {
    /// Create a new WIA bridge
    pub fn new(config: BridgeConfig) -> Self {
        Self {
            config,
            exoskeleton_client: None,
            bionic_eye_client: None,
        }
    }

    /// Set exoskeleton client
    pub fn with_exoskeleton(mut self, client: Arc<dyn ExoskeletonClient>) -> Self {
        self.exoskeleton_client = Some(client);
        self
    }

    /// Set bionic eye client
    pub fn with_bionic_eye(mut self, client: Arc<dyn BionicEyeClient>) -> Self {
        self.bionic_eye_client = Some(client);
        self
    }

    /// Send gesture command to exoskeleton
    pub async fn send_gesture_command(
        &self,
        command: SignGestureCommand,
    ) -> Result<GestureAck, BridgeError> {
        if !self.config.exoskeleton_enabled {
            return Err(BridgeError::SystemDisabled(WiaSystem::Exoskeleton));
        }

        let client = self
            .exoskeleton_client
            .as_ref()
            .ok_or(BridgeError::ClientNotConfigured(WiaSystem::Exoskeleton))?;

        client.send_gesture(command).await
    }

    /// Send display request to bionic eye
    pub async fn send_display_request(
        &self,
        request: BionicEyeDisplayRequest,
    ) -> Result<DisplayAck, BridgeError> {
        if !self.config.bionic_eye_enabled {
            return Err(BridgeError::SystemDisabled(WiaSystem::BionicEye));
        }

        let client = self
            .bionic_eye_client
            .as_ref()
            .ok_or(BridgeError::ClientNotConfigured(WiaSystem::BionicEye))?;

        client.send_display(request).await
    }

    /// Broadcast emergency to all connected systems
    pub async fn broadcast_emergency(
        &self,
        message: &str,
        urgency: EmergencyUrgency,
    ) -> BroadcastResult {
        let mut results = BroadcastResult::default();

        // Send to exoskeleton
        if self.config.exoskeleton_enabled {
            if let Some(client) = &self.exoskeleton_client {
                match client.emergency_alert(message, urgency).await {
                    Ok(_) => results.exoskeleton = Some(true),
                    Err(_) => results.exoskeleton = Some(false),
                }
            }
        }

        // Send to bionic eye
        if self.config.bionic_eye_enabled {
            if let Some(client) = &self.bionic_eye_client {
                let request = BionicEyeDisplayRequest {
                    request_id: uuid::Uuid::new_v4().to_string(),
                    content: DisplayContent::Caption {
                        text: format!("[EMERGENCY] {}", message),
                        language: "en".to_string(),
                    },
                    parameters: DisplayParameters {
                        position: DisplayPosition::Center,
                        size_percent: 50.0,
                        opacity: 1.0,
                        duration_ms: 10000,
                    },
                    priority: DisplayPriority::Emergency,
                };

                match client.send_display(request).await {
                    Ok(_) => results.bionic_eye = Some(true),
                    Err(_) => results.bionic_eye = Some(false),
                }
            }
        }

        results
    }
}

/// Emergency urgency level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum EmergencyUrgency {
    Low,
    Medium,
    High,
    Critical,
}

/// Broadcast result
#[derive(Debug, Clone, Default)]
pub struct BroadcastResult {
    pub exoskeleton: Option<bool>,
    pub bionic_eye: Option<bool>,
}

impl BroadcastResult {
    /// Check if all systems were notified
    pub fn all_systems_notified(&self) -> bool {
        self.exoskeleton.unwrap_or(true) && self.bionic_eye.unwrap_or(true)
    }
}

/// Gesture acknowledgment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GestureAck {
    pub accepted: bool,
    pub command_id: String,
    pub estimated_completion_ms: Option<u32>,
}

/// Display acknowledgment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisplayAck {
    pub accepted: bool,
    pub request_id: String,
}

/// Exoskeleton client trait
#[async_trait::async_trait]
pub trait ExoskeletonClient: Send + Sync {
    /// Send gesture command
    async fn send_gesture(&self, command: SignGestureCommand) -> Result<GestureAck, BridgeError>;

    /// Send emergency alert
    async fn emergency_alert(
        &self,
        message: &str,
        urgency: EmergencyUrgency,
    ) -> Result<(), BridgeError>;

    /// Check connection status
    async fn is_connected(&self) -> bool;
}

/// Bionic eye client trait
#[async_trait::async_trait]
pub trait BionicEyeClient: Send + Sync {
    /// Send display request
    async fn send_display(&self, request: BionicEyeDisplayRequest)
        -> Result<DisplayAck, BridgeError>;

    /// Check connection status
    async fn is_connected(&self) -> bool;
}

/// Bridge error
#[derive(Debug, thiserror::Error)]
pub enum BridgeError {
    #[error("System disabled: {0}")]
    SystemDisabled(WiaSystem),

    #[error("Client not configured for: {0}")]
    ClientNotConfigured(WiaSystem),

    #[error("Connection error: {0}")]
    ConnectionError(String),

    #[error("Timeout")]
    Timeout,

    #[error("Permission denied: {0}")]
    PermissionDenied(String),

    #[error("Invalid request: {0}")]
    InvalidRequest(String),
}

/// Mock exoskeleton client for testing
pub struct MockExoskeletonClient;

#[async_trait::async_trait]
impl ExoskeletonClient for MockExoskeletonClient {
    async fn send_gesture(&self, command: SignGestureCommand) -> Result<GestureAck, BridgeError> {
        Ok(GestureAck {
            accepted: true,
            command_id: command.command_id,
            estimated_completion_ms: Some(command.timing.duration_ms),
        })
    }

    async fn emergency_alert(
        &self,
        _message: &str,
        _urgency: EmergencyUrgency,
    ) -> Result<(), BridgeError> {
        Ok(())
    }

    async fn is_connected(&self) -> bool {
        true
    }
}

/// Mock bionic eye client for testing
pub struct MockBionicEyeClient;

#[async_trait::async_trait]
impl BionicEyeClient for MockBionicEyeClient {
    async fn send_display(
        &self,
        request: BionicEyeDisplayRequest,
    ) -> Result<DisplayAck, BridgeError> {
        Ok(DisplayAck {
            accepted: true,
            request_id: request.request_id,
        })
    }

    async fn is_connected(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_wia_bridge_gesture() {
        let config = BridgeConfig {
            exoskeleton_enabled: true,
            ..Default::default()
        };

        let bridge = WiaBridge::new(config)
            .with_exoskeleton(Arc::new(MockExoskeletonClient));

        let command = SignGestureCommand {
            command_id: "cmd-001".to_string(),
            source_request_id: "req-001".to_string(),
            sign: SignInfo {
                gloss: "HELLO".to_string(),
                notation: None,
                hand_shape: HandShape::Open,
                movement_type: MovementType::Static,
            },
            joint_targets: vec![],
            timing: GestureTiming {
                start_delay_ms: 0,
                duration_ms: 500,
                transition: TransitionType::Smooth,
            },
            safety: GestureSafety::default(),
        };

        let result = bridge.send_gesture_command(command).await;
        assert!(result.is_ok());
        assert!(result.unwrap().accepted);
    }

    #[tokio::test]
    async fn test_wia_bridge_display() {
        let config = BridgeConfig {
            bionic_eye_enabled: true,
            ..Default::default()
        };

        let bridge = WiaBridge::new(config).with_bionic_eye(Arc::new(MockBionicEyeClient));

        let request = BionicEyeDisplayRequest {
            request_id: "req-001".to_string(),
            content: DisplayContent::GlossText {
                glosses: vec!["HELLO".to_string()],
                highlight_current: true,
            },
            parameters: DisplayParameters::default(),
            priority: DisplayPriority::Normal,
        };

        let result = bridge.send_display_request(request).await;
        assert!(result.is_ok());
        assert!(result.unwrap().accepted);
    }
}
