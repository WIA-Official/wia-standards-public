//! WebSocket protocol handler for real-time communication

use serde::{Deserialize, Serialize};
use crate::types::Timestamp;
use super::message::{MessageType, MessageAck, AckStatus};

/// WebSocket connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting,
    Failed,
}

/// WebSocket endpoint types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WsEndpoint {
    /// Care events stream for a recipient
    CareEvents,
    /// Family dashboard live updates
    FamilyDashboard,
    /// Device control channel
    DeviceControl,
}

impl WsEndpoint {
    /// Get the WebSocket URL path for this endpoint
    pub fn path(&self, id: &str) -> String {
        match self {
            WsEndpoint::CareEvents => format!("/v1/care/{}", id),
            WsEndpoint::FamilyDashboard => format!("/v1/family/{}", id),
            WsEndpoint::DeviceControl => format!("/v1/device/{}", id),
        }
    }

    /// Get full WebSocket URL
    pub fn url(&self, host: &str, id: &str, secure: bool) -> String {
        let protocol = if secure { "wss" } else { "ws" };
        format!("{}://{}{}", protocol, host, self.path(id))
    }
}

/// WebSocket connection configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WsConfig {
    /// Server host
    pub host: String,
    /// Use secure WebSocket (wss)
    pub secure: bool,
    /// Reconnect on disconnect
    pub auto_reconnect: bool,
    /// Maximum reconnection attempts
    pub max_reconnect_attempts: u32,
    /// Reconnection delay in milliseconds
    pub reconnect_delay_ms: u32,
    /// Ping interval in seconds
    pub ping_interval_secs: u32,
    /// Connection timeout in seconds
    pub connect_timeout_secs: u32,
}

impl Default for WsConfig {
    fn default() -> Self {
        Self {
            host: "stream.wia-carebot.org".to_string(),
            secure: true,
            auto_reconnect: true,
            max_reconnect_attempts: 10,
            reconnect_delay_ms: 1000,
            ping_interval_secs: 30,
            connect_timeout_secs: 10,
        }
    }
}

/// WebSocket session manager
#[derive(Debug, Clone)]
pub struct WsSession {
    /// Session ID
    pub session_id: String,
    /// Endpoint type
    pub endpoint: WsEndpoint,
    /// Target ID (recipient_id, family_id, or device_id)
    pub target_id: String,
    /// Connection state
    pub state: ConnectionState,
    /// Last ping timestamp
    pub last_ping: Option<Timestamp>,
    /// Last pong timestamp
    pub last_pong: Option<Timestamp>,
    /// Messages awaiting acknowledgment
    pub pending_acks: Vec<PendingAck>,
    /// Reconnection attempts
    pub reconnect_attempts: u32,
    /// Session statistics
    pub stats: SessionStats,
}

impl WsSession {
    /// Create a new WebSocket session
    pub fn new(endpoint: WsEndpoint, target_id: &str) -> Self {
        Self {
            session_id: format!("ws-{}", uuid::Uuid::new_v4()),
            endpoint,
            target_id: target_id.to_string(),
            state: ConnectionState::Disconnected,
            last_ping: None,
            last_pong: None,
            pending_acks: Vec::new(),
            reconnect_attempts: 0,
            stats: SessionStats::default(),
        }
    }

    /// Mark a message as pending acknowledgment
    pub fn add_pending_ack(&mut self, message_id: &str, message_type: MessageType) {
        self.pending_acks.push(PendingAck {
            message_id: message_id.to_string(),
            message_type,
            sent_at: Timestamp::now(),
            retry_count: 0,
        });
    }

    /// Process acknowledgment
    pub fn process_ack(&mut self, ack: &MessageAck) -> bool {
        if let Some(pos) = self
            .pending_acks
            .iter()
            .position(|p| p.message_id == ack.message_id)
        {
            self.pending_acks.remove(pos);
            if ack.status == AckStatus::Success {
                self.stats.messages_acked += 1;
            } else {
                self.stats.messages_failed += 1;
            }
            true
        } else {
            false
        }
    }

    /// Get messages that need retry
    pub fn get_retry_messages(&self, timeout_secs: u32) -> Vec<&PendingAck> {
        let now = chrono::Utc::now();
        self.pending_acks
            .iter()
            .filter(|p| {
                let elapsed = (now - p.sent_at.0).num_seconds() as u32;
                elapsed > timeout_secs
            })
            .collect()
    }

    /// Update statistics for sent message
    pub fn record_sent(&mut self, message_type: MessageType) {
        self.stats.messages_sent += 1;
        match message_type {
            MessageType::SafetyEmergency => self.stats.emergency_messages += 1,
            _ => {}
        }
    }

    /// Update statistics for received message
    pub fn record_received(&mut self) {
        self.stats.messages_received += 1;
    }

    /// Check if connection is healthy
    pub fn is_healthy(&self) -> bool {
        if self.state != ConnectionState::Connected {
            return false;
        }

        // Check if we've received pong within expected interval
        if let (Some(ping), Some(pong)) = (&self.last_ping, &self.last_pong) {
            pong.0 >= ping.0
        } else {
            true // No ping/pong yet, assume healthy
        }
    }
}

/// Pending acknowledgment tracker
#[derive(Debug, Clone)]
pub struct PendingAck {
    pub message_id: String,
    pub message_type: MessageType,
    pub sent_at: Timestamp,
    pub retry_count: u32,
}

/// Session statistics
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SessionStats {
    /// Total messages sent
    pub messages_sent: u64,
    /// Total messages received
    pub messages_received: u64,
    /// Messages successfully acknowledged
    pub messages_acked: u64,
    /// Messages failed
    pub messages_failed: u64,
    /// Emergency messages sent
    pub emergency_messages: u64,
    /// Total reconnections
    pub reconnections: u32,
    /// Bytes sent
    pub bytes_sent: u64,
    /// Bytes received
    pub bytes_received: u64,
}

/// Video call signaling for WebRTC
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VideoCallSignaling {
    /// Signaling type
    #[serde(rename = "type")]
    pub signal_type: SignalingType,
    /// From device/app
    pub from: String,
    /// To device/app
    pub to: String,
    /// Session ID
    pub session_id: String,
    /// SDP offer/answer
    pub sdp: Option<String>,
    /// ICE candidates
    pub ice_candidates: Vec<IceCandidate>,
    /// Accessibility settings
    pub accessibility: CallAccessibility,
}

impl VideoCallSignaling {
    /// Create an offer signal
    pub fn offer(from: &str, to: &str, sdp: &str) -> Self {
        Self {
            signal_type: SignalingType::Offer,
            from: from.to_string(),
            to: to.to_string(),
            session_id: format!("call-{}", uuid::Uuid::new_v4()),
            sdp: Some(sdp.to_string()),
            ice_candidates: Vec::new(),
            accessibility: CallAccessibility::default(),
        }
    }

    /// Create an answer signal
    pub fn answer(from: &str, to: &str, session_id: &str, sdp: &str) -> Self {
        Self {
            signal_type: SignalingType::Answer,
            from: from.to_string(),
            to: to.to_string(),
            session_id: session_id.to_string(),
            sdp: Some(sdp.to_string()),
            ice_candidates: Vec::new(),
            accessibility: CallAccessibility::default(),
        }
    }

    /// Create an ICE candidate signal
    pub fn ice_candidate(from: &str, to: &str, session_id: &str, candidate: IceCandidate) -> Self {
        Self {
            signal_type: SignalingType::IceCandidate,
            from: from.to_string(),
            to: to.to_string(),
            session_id: session_id.to_string(),
            sdp: None,
            ice_candidates: vec![candidate],
            accessibility: CallAccessibility::default(),
        }
    }

    /// Set accessibility options for elderly users
    pub fn with_accessibility(mut self) -> Self {
        self.accessibility = CallAccessibility {
            auto_answer: true,
            loud_ringtone: true,
            visual_indicator: "screen_flash".to_string(),
            simplified_ui: true,
        };
        self
    }
}

/// Signaling message types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignalingType {
    Offer,
    Answer,
    IceCandidate,
    Hangup,
    Reject,
    Busy,
}

/// ICE candidate
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IceCandidate {
    pub candidate: String,
    pub sdp_mid: Option<String>,
    pub sdp_m_line_index: Option<u32>,
}

/// Accessibility settings for video calls
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CallAccessibility {
    /// Auto-answer incoming calls
    pub auto_answer: bool,
    /// Use loud ringtone
    pub loud_ringtone: bool,
    /// Visual indicator type
    pub visual_indicator: String,
    /// Use simplified UI
    pub simplified_ui: bool,
}

impl Default for CallAccessibility {
    fn default() -> Self {
        Self {
            auto_answer: false,
            loud_ringtone: false,
            visual_indicator: "normal".to_string(),
            simplified_ui: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ws_endpoint_url() {
        let endpoint = WsEndpoint::CareEvents;
        let url = endpoint.url("stream.wia-carebot.org", "recipient-001", true);
        assert_eq!(url, "wss://stream.wia-carebot.org/v1/care/recipient-001");
    }

    #[test]
    fn test_ws_session_creation() {
        let session = WsSession::new(WsEndpoint::CareEvents, "recipient-001");
        assert!(session.session_id.starts_with("ws-"));
        assert_eq!(session.state, ConnectionState::Disconnected);
    }

    #[test]
    fn test_video_call_signaling() {
        let signal = VideoCallSignaling::offer("carebot-001", "family-app-001", "sdp-data")
            .with_accessibility();

        assert_eq!(signal.signal_type, SignalingType::Offer);
        assert!(signal.accessibility.auto_answer);
        assert!(signal.accessibility.loud_ringtone);
    }
}
