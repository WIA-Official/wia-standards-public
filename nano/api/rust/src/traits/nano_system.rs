//! Base trait for all nano systems

use crate::error::NanoResult;
use crate::types::{NanoMessage, NanoSystemType, Environment, Scale};
use async_trait::async_trait;

/// Base trait for all nanoscale systems
#[async_trait]
pub trait NanoSystem: Send + Sync {
    /// Get the system type
    fn system_type(&self) -> NanoSystemType;

    /// Get unique system identifier
    fn id(&self) -> &str;

    /// Get current system status
    fn status(&self) -> SystemStatus;

    /// Initialize the system
    async fn initialize(&mut self) -> NanoResult<()>;

    /// Shutdown the system gracefully
    async fn shutdown(&mut self) -> NanoResult<()>;

    /// Reset to initial state
    async fn reset(&mut self) -> NanoResult<()>;

    /// Get current environment
    fn environment(&self) -> Option<&Environment>;

    /// Set operating environment
    fn set_environment(&mut self, env: Environment);

    /// Get scale information
    fn scale(&self) -> Option<&Scale>;

    /// Generate a status message
    fn to_message(&self) -> NanoResult<NanoMessage>;

    /// Get energy level (in femtojoules)
    fn energy_level(&self) -> Option<f64> {
        None
    }

    /// Check if system is operational
    fn is_operational(&self) -> bool {
        matches!(self.status(), SystemStatus::Active | SystemStatus::Idle)
    }
}

/// System operational status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemStatus {
    /// System is initializing
    Initializing,
    /// System is idle and ready
    Idle,
    /// System is actively operating
    Active,
    /// System is in low-power mode
    LowPower,
    /// System has encountered an error
    Error,
    /// System is shutting down
    ShuttingDown,
    /// System is offline
    Offline,
}

impl SystemStatus {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Initializing => "initializing",
            Self::Idle => "idle",
            Self::Active => "active",
            Self::LowPower => "low_power",
            Self::Error => "error",
            Self::ShuttingDown => "shutting_down",
            Self::Offline => "offline",
        }
    }
}

/// Power management trait for nano systems
pub trait PowerManaged {
    /// Get current power consumption in femtowatts
    fn power_consumption(&self) -> f64;

    /// Get remaining energy in femtojoules
    fn remaining_energy(&self) -> f64;

    /// Get estimated runtime in seconds
    fn estimated_runtime(&self) -> f64 {
        let power = self.power_consumption();
        if power > 0.0 {
            self.remaining_energy() / power
        } else {
            f64::INFINITY
        }
    }

    /// Enter low-power mode
    fn enter_low_power_mode(&mut self) -> NanoResult<()>;

    /// Exit low-power mode
    fn exit_low_power_mode(&mut self) -> NanoResult<()>;

    /// Harvest energy from environment (if capable)
    fn harvest_energy(&mut self, _amount: f64) -> NanoResult<f64> {
        Ok(0.0) // Default: no harvesting capability
    }
}

/// Diagnostic capabilities for nano systems
pub trait Diagnosable {
    /// Run self-diagnostic
    fn run_diagnostic(&mut self) -> DiagnosticResult;

    /// Get last diagnostic result
    fn last_diagnostic(&self) -> Option<&DiagnosticResult>;

    /// Check component health
    fn component_health(&self) -> Vec<ComponentHealth>;
}

/// Result of a diagnostic check
#[derive(Debug, Clone)]
pub struct DiagnosticResult {
    pub timestamp: String,
    pub overall_status: HealthStatus,
    pub components: Vec<ComponentHealth>,
    pub messages: Vec<String>,
}

/// Health status of a component
#[derive(Debug, Clone)]
pub struct ComponentHealth {
    pub component: String,
    pub status: HealthStatus,
    pub details: Option<String>,
}

/// Health status enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthStatus {
    Healthy,
    Degraded,
    Warning,
    Critical,
    Unknown,
}
