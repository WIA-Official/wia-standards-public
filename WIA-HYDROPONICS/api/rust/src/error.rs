//! Error types for WIA-HYDROPONICS SDK
//!
//! 弘益人間 - Clear error handling for sustainable agriculture

use thiserror::Error;

/// Result type alias
pub type Result<T> = std::result::Result<T, HydroponicsError>;

/// WIA-HYDROPONICS error types
#[derive(Error, Debug)]
pub enum HydroponicsError {
    #[error("Invalid URL: {0}")]
    InvalidUrl(String),

    #[error("Invalid API key: {0}")]
    InvalidApiKey(String),

    #[error("HTTP error: {0}")]
    HttpError(reqwest::StatusCode),

    #[error("Network error: {0}")]
    NetworkError(#[from] reqwest::Error),

    #[error("JSON error: {0}")]
    JsonError(#[from] serde_json::Error),

    #[error("System not found: {0}")]
    SystemNotFound(String),

    #[error("Plant not found: {0}")]
    PlantNotFound(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("API error: {0}")]
    ApiError(String),

    #[error("Invalid pH value: {0}")]
    InvalidPh(f64),

    #[error("Invalid EC value: {0}")]
    InvalidEc(f64),

    #[error("System malfunction: {0}")]
    SystemMalfunction(String),

    #[error("Sensor error: {0}")]
    SensorError(String),

    #[error("Other error: {0}")]
    Other(String),
}

impl HydroponicsError {
    pub fn is_retryable(&self) -> bool {
        matches!(self, HydroponicsError::NetworkError(_) | HydroponicsError::SensorError(_))
    }

    pub fn error_code(&self) -> &str {
        match self {
            HydroponicsError::InvalidUrl(_) => "INVALID_URL",
            HydroponicsError::InvalidApiKey(_) => "INVALID_API_KEY",
            HydroponicsError::HttpError(_) => "HTTP_ERROR",
            HydroponicsError::NetworkError(_) => "NETWORK_ERROR",
            HydroponicsError::JsonError(_) => "JSON_ERROR",
            HydroponicsError::SystemNotFound(_) => "SYSTEM_NOT_FOUND",
            HydroponicsError::PlantNotFound(_) => "PLANT_NOT_FOUND",
            HydroponicsError::ValidationError(_) => "VALIDATION_ERROR",
            HydroponicsError::ApiError(_) => "API_ERROR",
            HydroponicsError::InvalidPh(_) => "INVALID_PH",
            HydroponicsError::InvalidEc(_) => "INVALID_EC",
            HydroponicsError::SystemMalfunction(_) => "SYSTEM_MALFUNCTION",
            HydroponicsError::SensorError(_) => "SENSOR_ERROR",
            HydroponicsError::Other(_) => "OTHER",
        }
    }
}
