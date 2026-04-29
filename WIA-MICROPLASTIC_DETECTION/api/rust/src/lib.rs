//! WIA Microplastic Detection Standard SDK
//!
//! 弘益人間 (Benefit All Humanity)
//!
//! This SDK provides tools for detecting and analyzing microplastics in marine environments.

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::MicroplasticDetectionClient;
pub use error::{Result, MicroplasticDetectionError};
pub use types::*;

/// SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert!(!VERSION.is_empty());
    }
}
