//! Error types
use thiserror::Error;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Error, Debug)]
pub enum Error {
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
    #[error("Validation error: {0}")]
    ValidationError(String),
    #[error("API error: {0}")]
    ApiError(String),
    #[error("Other error: {0}")]
    Other(String),
}
