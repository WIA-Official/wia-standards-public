//! Error types for WIA Nano SDK

use std::fmt;
use std::error::Error;

/// Result type alias for WIA Nano operations
pub type NanoResult<T> = Result<T, NanoError>;

/// Main error type for WIA Nano SDK
#[derive(Debug)]
pub enum NanoError {
    /// Serialization/deserialization error
    Serialization(serde_json::Error),

    /// Invalid parameter or configuration
    InvalidParameter(String),

    /// Resource not found
    NotFound(String),

    /// Operation timeout
    Timeout(String),

    /// Communication error
    Communication(String),

    /// Insufficient energy or power
    InsufficientEnergy { required: f64, available: f64 },

    /// Position out of bounds
    OutOfBounds { position: [f64; 3], bounds: [f64; 6] },

    /// Assembly operation failed
    AssemblyFailed(String),

    /// Sensor error
    SensorError { sensor_id: String, message: String },

    /// Navigation error
    NavigationError(String),

    /// Mission aborted
    MissionAborted { reason: String, recovery: Option<String> },

    /// Collision detected
    CollisionDetected { object_id: Option<String>, position: [f64; 3] },

    /// IO error
    Io(std::io::Error),

    /// Custom error
    Custom(String),
}

impl fmt::Display for NanoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Serialization(e) => write!(f, "Serialization error: {}", e),
            Self::InvalidParameter(msg) => write!(f, "Invalid parameter: {}", msg),
            Self::NotFound(resource) => write!(f, "Not found: {}", resource),
            Self::Timeout(op) => write!(f, "Operation timeout: {}", op),
            Self::Communication(msg) => write!(f, "Communication error: {}", msg),
            Self::InsufficientEnergy { required, available } => {
                write!(f, "Insufficient energy: required {} fJ, available {} fJ", required, available)
            }
            Self::OutOfBounds { position, bounds } => {
                write!(f, "Position {:?} out of bounds {:?}", position, bounds)
            }
            Self::AssemblyFailed(reason) => write!(f, "Assembly failed: {}", reason),
            Self::SensorError { sensor_id, message } => {
                write!(f, "Sensor {} error: {}", sensor_id, message)
            }
            Self::NavigationError(msg) => write!(f, "Navigation error: {}", msg),
            Self::MissionAborted { reason, recovery } => {
                match recovery {
                    Some(rec) => write!(f, "Mission aborted: {}. Recovery: {}", reason, rec),
                    None => write!(f, "Mission aborted: {}", reason),
                }
            }
            Self::CollisionDetected { object_id, position } => {
                match object_id {
                    Some(id) => write!(f, "Collision with {} at {:?}", id, position),
                    None => write!(f, "Collision at {:?}", position),
                }
            }
            Self::Io(e) => write!(f, "IO error: {}", e),
            Self::Custom(msg) => write!(f, "{}", msg),
        }
    }
}

impl Error for NanoError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Serialization(e) => Some(e),
            Self::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<serde_json::Error> for NanoError {
    fn from(err: serde_json::Error) -> Self {
        Self::Serialization(err)
    }
}

impl From<std::io::Error> for NanoError {
    fn from(err: std::io::Error) -> Self {
        Self::Io(err)
    }
}

impl From<String> for NanoError {
    fn from(msg: String) -> Self {
        Self::Custom(msg)
    }
}

impl From<&str> for NanoError {
    fn from(msg: &str) -> Self {
        Self::Custom(msg.to_string())
    }
}
