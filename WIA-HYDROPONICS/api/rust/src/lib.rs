//! # WIA-HYDROPONICS: Smart Hydroponic System Management
//!
//! 弘益人間 (홍익인간) - Benefit All Humanity through sustainable agriculture
//!
//! This SDK provides a comprehensive Rust interface for the WIA-HYDROPONICS standard,
//! enabling smart hydroponic system management, plant monitoring, and nutrient optimization.
//!
//! ## Features
//!
//! - System monitoring and control
//! - Plant growth tracking
//! - Nutrient management and pH control
//! - Environmental monitoring
//! - Harvest prediction and optimization
//!
//! ## Example
//!
//! ```no_run
//! use wia_hydroponics::HydroponicsClient;
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = HydroponicsClient::new("https://api.wia-hydroponics.org", "your-api-key")?;
//!
//!     // Monitor system status
//!     let systems = client.list_systems().await?;
//!     println!("Found {} systems", systems.len());
//!
//!     Ok(())
//! }
//! ```

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::HydroponicsClient;
pub use error::{HydroponicsError, Result};
pub use types::*;

/// WIA-HYDROPONICS SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Philosophy: 弘益人間 (Benefit All Humanity)
pub const PHILOSOPHY: &str = "弘益人間 (홍익인간) - Benefit All Humanity through sustainable agriculture";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert!(!VERSION.is_empty());
    }

    #[test]
    fn test_philosophy() {
        assert!(PHILOSOPHY.contains("弘益人間"));
    }
}
