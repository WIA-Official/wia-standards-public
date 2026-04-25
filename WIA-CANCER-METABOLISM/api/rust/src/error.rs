//! Error types for Cancer Metabolism Research SDK

use thiserror::Error;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Error, Debug)]
pub enum Error {
    #[error("Network error: {0}")]
    NetworkError(String),
    #[error("API error: {0}")]
    ApiError(String),
    #[error("Parse error: {0}")]
    ParseError(String),
    #[error("Invalid input: {0}")]
    InvalidInput(String),
    #[error("Analysis error: {0}")]
    AnalysisError(String),
}

impl Error {
    pub fn is_retryable(&self) -> bool {
        matches!(self, Error::NetworkError(_))
    }
}
