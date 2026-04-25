//! WIA Ocean Deep Sea Exploration Standard SDK
//!
//! 弘益人間 (Benefit All Humanity)
//!
//! This SDK provides tools for deep sea exploration, mission planning, and data collection.

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::DeepSeaExplorationClient;
pub use error::{Result, DeepSeaExplorationError};
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
