//! WIA Auto Ecosystem Integration modules
//!
//! This module provides integration with WIA ecosystem assistive devices
//! including smart wheelchairs, eye gaze systems, BCI, AAC, and fleet systems.

pub mod wheelchair;
pub mod eye_gaze;
pub mod bci;
pub mod aac;
pub mod fleet;

// Re-exports
pub use wheelchair::*;
pub use eye_gaze::*;
pub use bci::*;
pub use aac::*;
pub use fleet::*;

use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::error::Result;

/// Integration priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum IntegrationPriority {
    /// Low priority (comfort features)
    Low = 0,
    /// Normal priority (standard operations)
    Normal = 1,
    /// High priority (important functions)
    High = 2,
    /// Critical priority (safety-related)
    Critical = 3,
}

/// Command from any integration source
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IntegrationCommand {
    /// Unique command ID
    pub id: Uuid,
    /// Source of the command
    pub source: IntegrationSource,
    /// Command type
    pub command_type: CommandType,
    /// Priority level
    pub priority: IntegrationPriority,
    /// Timestamp
    pub timestamp: chrono::DateTime<chrono::Utc>,
    /// Whether confirmation is required
    pub requires_confirmation: bool,
    /// Command parameters
    pub parameters: serde_json::Value,
}

impl IntegrationCommand {
    /// Create a new integration command
    pub fn new(source: IntegrationSource, command_type: CommandType) -> Self {
        Self {
            id: Uuid::new_v4(),
            source,
            command_type,
            priority: IntegrationPriority::Normal,
            timestamp: chrono::Utc::now(),
            requires_confirmation: false,
            parameters: serde_json::Value::Null,
        }
    }

    /// Set priority
    pub fn with_priority(mut self, priority: IntegrationPriority) -> Self {
        self.priority = priority;
        self
    }

    /// Set confirmation requirement
    pub fn with_confirmation(mut self) -> Self {
        self.requires_confirmation = true;
        self
    }

    /// Set parameters
    pub fn with_params<T: Serialize>(mut self, params: &T) -> Self {
        self.parameters = serde_json::to_value(params).unwrap_or_default();
        self
    }
}

/// Source of integration commands
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntegrationSource {
    /// Smart wheelchair
    Wheelchair,
    /// Eye gaze system
    EyeGaze,
    /// Brain-computer interface
    Bci,
    /// AAC/Voice
    Aac,
    /// Fleet management
    Fleet,
    /// Direct user input
    Direct,
}

/// Types of commands that can be issued
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandType {
    // Navigation
    /// Set destination
    SetDestination,
    /// Cancel current trip
    CancelTrip,
    /// Request pull over
    PullOver,
    /// Resume trip
    ResumeTrip,

    // Emergency
    /// Emergency stop
    EmergencyStop,
    /// Request assistance
    RequestAssistance,

    // Comfort
    /// Adjust temperature
    AdjustTemperature,
    /// Control window
    ControlWindow,
    /// Control entertainment
    ControlEntertainment,

    // Wheelchair specific
    /// Request docking
    RequestDocking,
    /// Release securement
    ReleaseSecurement,

    // Information
    /// Query status
    QueryStatus,
    /// Query ETA
    QueryEta,
    /// Query location
    QueryLocation,
}

/// Command arbiter that resolves conflicts between multiple input sources
#[derive(Debug)]
pub struct CommandArbiter {
    /// Pending commands
    pending: Vec<IntegrationCommand>,
    /// Active command sources
    active_sources: Vec<IntegrationSource>,
}

impl CommandArbiter {
    /// Create a new command arbiter
    pub fn new() -> Self {
        Self {
            pending: Vec::new(),
            active_sources: Vec::new(),
        }
    }

    /// Submit a command
    pub fn submit(&mut self, command: IntegrationCommand) {
        self.pending.push(command);
        // Sort ascending so pop() returns highest priority
        self.pending.sort_by(|a, b| a.priority.cmp(&b.priority));
    }

    /// Get the highest priority command
    pub fn next_command(&mut self) -> Option<IntegrationCommand> {
        self.pending.pop()
    }

    /// Get all pending commands
    pub fn pending_commands(&self) -> &[IntegrationCommand] {
        &self.pending
    }

    /// Clear all pending commands
    pub fn clear(&mut self) {
        self.pending.clear();
    }

    /// Register an active source
    pub fn register_source(&mut self, source: IntegrationSource) {
        if !self.active_sources.contains(&source) {
            self.active_sources.push(source);
        }
    }

    /// Unregister a source
    pub fn unregister_source(&mut self, source: IntegrationSource) {
        self.active_sources.retain(|s| *s != source);
    }

    /// Check if a source is active
    pub fn is_source_active(&self, source: IntegrationSource) -> bool {
        self.active_sources.contains(&source)
    }
}

impl Default for CommandArbiter {
    fn default() -> Self {
        Self::new()
    }
}

/// Integration hub that manages all device integrations
#[async_trait]
pub trait IntegrationHub: Send + Sync {
    /// Initialize all integrations
    async fn initialize(&mut self) -> Result<()>;

    /// Shutdown all integrations
    async fn shutdown(&mut self) -> Result<()>;

    /// Process incoming commands
    async fn process_commands(&mut self) -> Result<Vec<IntegrationCommand>>;

    /// Send feedback to a specific integration
    async fn send_feedback(&self, source: IntegrationSource, feedback: &str) -> Result<()>;

    /// Get status of all integrations
    async fn status(&self) -> IntegrationStatus;
}

/// Status of all integrations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IntegrationStatus {
    /// Wheelchair integration status
    pub wheelchair: ConnectionStatus,
    /// Eye gaze integration status
    pub eye_gaze: ConnectionStatus,
    /// BCI integration status
    pub bci: ConnectionStatus,
    /// AAC integration status
    pub aac: ConnectionStatus,
    /// Fleet integration status
    pub fleet: ConnectionStatus,
}

impl Default for IntegrationStatus {
    fn default() -> Self {
        Self {
            wheelchair: ConnectionStatus::Disconnected,
            eye_gaze: ConnectionStatus::Disconnected,
            bci: ConnectionStatus::Disconnected,
            aac: ConnectionStatus::Disconnected,
            fleet: ConnectionStatus::Disconnected,
        }
    }
}

/// Connection status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConnectionStatus {
    /// Not connected
    Disconnected,
    /// Connecting
    Connecting,
    /// Connected and operational
    Connected,
    /// Connected but degraded
    Degraded,
    /// Error state
    Error,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command_priority() {
        let mut arbiter = CommandArbiter::new();

        let low = IntegrationCommand::new(IntegrationSource::Aac, CommandType::AdjustTemperature)
            .with_priority(IntegrationPriority::Low);
        let high = IntegrationCommand::new(IntegrationSource::Bci, CommandType::EmergencyStop)
            .with_priority(IntegrationPriority::Critical);
        let normal = IntegrationCommand::new(IntegrationSource::EyeGaze, CommandType::SetDestination)
            .with_priority(IntegrationPriority::Normal);

        arbiter.submit(low);
        arbiter.submit(high);
        arbiter.submit(normal);

        // Critical should come first
        let cmd = arbiter.next_command().unwrap();
        assert_eq!(cmd.priority, IntegrationPriority::Critical);

        // Then normal
        let cmd = arbiter.next_command().unwrap();
        assert_eq!(cmd.priority, IntegrationPriority::Normal);

        // Then low
        let cmd = arbiter.next_command().unwrap();
        assert_eq!(cmd.priority, IntegrationPriority::Low);
    }

    #[test]
    fn test_source_registration() {
        let mut arbiter = CommandArbiter::new();

        arbiter.register_source(IntegrationSource::Wheelchair);
        arbiter.register_source(IntegrationSource::EyeGaze);

        assert!(arbiter.is_source_active(IntegrationSource::Wheelchair));
        assert!(arbiter.is_source_active(IntegrationSource::EyeGaze));
        assert!(!arbiter.is_source_active(IntegrationSource::Bci));

        arbiter.unregister_source(IntegrationSource::Wheelchair);
        assert!(!arbiter.is_source_active(IntegrationSource::Wheelchair));
    }
}
