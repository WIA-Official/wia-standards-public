//! Protocol message handler

use crate::error::{RobotError, RobotResult};
use crate::protocol::message::*;
use std::collections::HashMap;
use std::sync::Arc;

/// Message handler callback type
pub type MessageCallback = Box<dyn Fn(&WrpMessage) -> RobotResult<()> + Send + Sync>;

/// Protocol handler for processing WRP messages
pub struct ProtocolHandler {
    callbacks: HashMap<MessageType, Vec<Arc<MessageCallback>>>,
    emergency_callback: Option<Arc<MessageCallback>>,
    default_callback: Option<Arc<MessageCallback>>,
}

impl Default for ProtocolHandler {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolHandler {
    /// Create a new protocol handler
    pub fn new() -> Self {
        Self {
            callbacks: HashMap::new(),
            emergency_callback: None,
            default_callback: None,
        }
    }

    /// Register a callback for a specific message type
    pub fn on<F>(&mut self, msg_type: MessageType, callback: F)
    where
        F: Fn(&WrpMessage) -> RobotResult<()> + Send + Sync + 'static,
    {
        self.callbacks
            .entry(msg_type)
            .or_default()
            .push(Arc::new(Box::new(callback)));
    }

    /// Register emergency stop callback (highest priority)
    pub fn on_emergency<F>(&mut self, callback: F)
    where
        F: Fn(&WrpMessage) -> RobotResult<()> + Send + Sync + 'static,
    {
        self.emergency_callback = Some(Arc::new(Box::new(callback)));
    }

    /// Register default callback for unhandled messages
    pub fn on_default<F>(&mut self, callback: F)
    where
        F: Fn(&WrpMessage) -> RobotResult<()> + Send + Sync + 'static,
    {
        self.default_callback = Some(Arc::new(Box::new(callback)));
    }

    /// Handle an incoming message
    pub fn handle(&self, message: &WrpMessage) -> RobotResult<()> {
        // Verify checksum first
        if !message.verify_checksum() {
            return Err(RobotError::CommunicationError(
                "Checksum verification failed".into(),
            ));
        }

        // Emergency messages get priority handling
        if message.is_emergency() {
            if let Some(ref callback) = self.emergency_callback {
                callback(message)?;
            }
        }

        // Call registered callbacks for this message type
        if let Some(callbacks) = self.callbacks.get(&message.message_type) {
            for callback in callbacks {
                callback(message)?;
            }
        } else if let Some(ref default) = self.default_callback {
            // Call default callback if no specific handler
            default(message)?;
        }

        Ok(())
    }

    /// Handle a batch of messages, sorted by priority
    pub fn handle_batch(&self, messages: &mut [WrpMessage]) -> Vec<RobotResult<()>> {
        // Sort by priority (highest first)
        messages.sort_by(|a, b| b.priority.cmp(&a.priority));

        messages.iter().map(|msg| self.handle(msg)).collect()
    }
}

/// Connection state machine
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Active,
    Inactive,
    Reconnecting,
    Stopped,
}

impl Default for ConnectionState {
    fn default() -> Self {
        ConnectionState::Disconnected
    }
}

/// Connection manager for WRP
pub struct ConnectionManager {
    state: ConnectionState,
    device_id: String,
    device_type: String,
    heartbeat_interval_ms: u64,
    last_heartbeat: Option<chrono::DateTime<chrono::Utc>>,
    reconnect_attempts: u32,
    max_reconnect_attempts: u32,
}

impl ConnectionManager {
    /// Create a new connection manager
    pub fn new(device_id: &str, device_type: &str) -> Self {
        Self {
            state: ConnectionState::Disconnected,
            device_id: device_id.to_string(),
            device_type: device_type.to_string(),
            heartbeat_interval_ms: 1000,
            last_heartbeat: None,
            reconnect_attempts: 0,
            max_reconnect_attempts: 5,
        }
    }

    /// Get current state
    pub fn state(&self) -> ConnectionState {
        self.state
    }

    /// Set heartbeat interval
    pub fn set_heartbeat_interval(&mut self, interval_ms: u64) {
        self.heartbeat_interval_ms = interval_ms;
    }

    /// Transition to connecting state
    pub fn connect(&mut self) -> RobotResult<()> {
        match self.state {
            ConnectionState::Disconnected | ConnectionState::Reconnecting => {
                self.state = ConnectionState::Connecting;
                Ok(())
            }
            ConnectionState::Stopped => {
                Err(RobotError::CommunicationError(
                    "Cannot connect from stopped state".into(),
                ))
            }
            _ => Ok(()), // Already connecting or connected
        }
    }

    /// Handle successful handshake
    pub fn on_handshake_complete(&mut self) {
        self.state = ConnectionState::Active;
        self.reconnect_attempts = 0;
        self.update_heartbeat();
    }

    /// Update last heartbeat time
    pub fn update_heartbeat(&mut self) {
        self.last_heartbeat = Some(chrono::Utc::now());
        if self.state == ConnectionState::Inactive {
            self.state = ConnectionState::Active;
        }
    }

    /// Check heartbeat timeout
    pub fn check_heartbeat(&mut self) -> bool {
        if let Some(last) = self.last_heartbeat {
            let elapsed = chrono::Utc::now()
                .signed_duration_since(last)
                .num_milliseconds() as u64;

            let timeout = self.heartbeat_interval_ms * 3;

            if elapsed > timeout {
                self.state = ConnectionState::Inactive;
                return false;
            }
        }
        true
    }

    /// Handle connection error
    pub fn on_error(&mut self) {
        match self.state {
            ConnectionState::Active | ConnectionState::Inactive => {
                self.state = ConnectionState::Reconnecting;
                self.reconnect_attempts += 1;
            }
            ConnectionState::Reconnecting => {
                self.reconnect_attempts += 1;
                if self.reconnect_attempts >= self.max_reconnect_attempts {
                    self.state = ConnectionState::Stopped;
                }
            }
            _ => {}
        }
    }

    /// Handle emergency stop
    pub fn on_emergency_stop(&mut self) {
        self.state = ConnectionState::Stopped;
    }

    /// Disconnect
    pub fn disconnect(&mut self) {
        self.state = ConnectionState::Disconnected;
        self.last_heartbeat = None;
    }

    /// Get device endpoint
    pub fn endpoint(&self) -> Endpoint {
        Endpoint::new(&self.device_id, &self.device_type)
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        matches!(self.state, ConnectionState::Active | ConnectionState::Inactive)
    }
}

/// Safety watchdog for monitoring communication
pub struct SafetyWatchdog {
    enabled: bool,
    timeout_ms: u64,
    last_activity: chrono::DateTime<chrono::Utc>,
    warning_threshold_ms: u64,
    critical_threshold_ms: u64,
}

impl Default for SafetyWatchdog {
    fn default() -> Self {
        Self::new(10000)
    }
}

impl SafetyWatchdog {
    /// Create a new watchdog with timeout
    pub fn new(timeout_ms: u64) -> Self {
        Self {
            enabled: true,
            timeout_ms,
            last_activity: chrono::Utc::now(),
            warning_threshold_ms: timeout_ms / 2,
            critical_threshold_ms: timeout_ms * 3 / 4,
        }
    }

    /// Enable or disable watchdog
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Feed the watchdog (reset timer)
    pub fn feed(&mut self) {
        self.last_activity = chrono::Utc::now();
    }

    /// Check watchdog status
    pub fn check(&self) -> SafetyLevel {
        if !self.enabled {
            return SafetyLevel::Normal;
        }

        let elapsed = chrono::Utc::now()
            .signed_duration_since(self.last_activity)
            .num_milliseconds() as u64;

        if elapsed >= self.timeout_ms {
            SafetyLevel::Emergency
        } else if elapsed >= self.critical_threshold_ms {
            SafetyLevel::Critical
        } else if elapsed >= self.warning_threshold_ms {
            SafetyLevel::Warning
        } else {
            SafetyLevel::Normal
        }
    }

    /// Check if watchdog has timed out
    pub fn is_timed_out(&self) -> bool {
        self.check() == SafetyLevel::Emergency
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::builder::MessageBuilder;
    use std::sync::atomic::{AtomicU32, Ordering};

    #[test]
    fn test_protocol_handler() {
        let mut handler = ProtocolHandler::new();
        let counter = Arc::new(AtomicU32::new(0));
        let counter_clone = counter.clone();

        handler.on(MessageType::Telemetry, move |_msg| {
            counter_clone.fetch_add(1, Ordering::SeqCst);
            Ok(())
        });

        let msg = MessageBuilder::telemetry()
            .from_device("test", "test")
            .to_device("test", "test")
            .build();

        handler.handle(&msg).unwrap();
        assert_eq!(counter.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn test_emergency_priority() {
        let mut handler = ProtocolHandler::new();
        let emergency_called = Arc::new(AtomicU32::new(0));
        let emergency_clone = emergency_called.clone();

        handler.on_emergency(move |_msg| {
            emergency_clone.fetch_add(1, Ordering::SeqCst);
            Ok(())
        });

        let msg = MessageBuilder::emergency_stop()
            .from_device("test", "test")
            .build();

        handler.handle(&msg).unwrap();
        assert_eq!(emergency_called.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn test_connection_state_machine() {
        let mut conn = ConnectionManager::new("test-001", "exoskeleton");
        assert_eq!(conn.state(), ConnectionState::Disconnected);

        conn.connect().unwrap();
        assert_eq!(conn.state(), ConnectionState::Connecting);

        conn.on_handshake_complete();
        assert_eq!(conn.state(), ConnectionState::Active);
        assert!(conn.is_connected());

        conn.on_emergency_stop();
        assert_eq!(conn.state(), ConnectionState::Stopped);
        assert!(!conn.is_connected());
    }

    #[test]
    fn test_safety_watchdog() {
        let watchdog = SafetyWatchdog::new(100);
        assert_eq!(watchdog.check(), SafetyLevel::Normal);
        assert!(!watchdog.is_timed_out());
    }

    #[test]
    fn test_checksum_validation() {
        let handler = ProtocolHandler::new();

        let mut msg = MessageBuilder::telemetry()
            .from_device("test", "test")
            .to_device("test", "test")
            .build();

        // Valid checksum should pass
        assert!(handler.handle(&msg).is_ok());

        // Invalid checksum should fail
        msg.checksum = Some("invalid!".to_string());
        assert!(handler.handle(&msg).is_err());
    }
}
