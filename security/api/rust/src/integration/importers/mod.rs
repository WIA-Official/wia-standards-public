//! Data Importers
//!
//! Import security data from various sources into WIA Security format.

pub mod nvd;
pub mod nessus;
pub mod openvas;
pub mod stix;

pub use nvd::*;
pub use nessus::*;
pub use openvas::*;
pub use stix::*;

use thiserror::Error;

/// Import error types
#[derive(Debug, Error)]
pub enum ImportError {
    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Authentication error: {0}")]
    AuthError(String),

    #[error("Rate limit exceeded, retry after {retry_after} seconds")]
    RateLimitError { retry_after: u64 },

    #[error("Invalid data format: {0}")]
    InvalidFormat(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
}

/// Import result type
pub type ImportResult<T> = Result<T, ImportError>;
