//! # WIA-HOME: Smart Home Automation Standard
//!
//! 弘益人間 (홍익인간) - Benefit All Humanity
//!
//! This SDK provides a comprehensive Rust interface for the WIA-HOME standard,
//! enabling smart home automation, device control, and energy management.
//!
//! ## Features
//!
//! - Device discovery and management
//! - Real-time control and monitoring
//! - Energy optimization
//! - Security and access control
//! - Scene automation
//!
//! ## Example
//!
//! ```no_run
//! use wia_home::{HomeClient, DeviceType};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = HomeClient::new("https://api.wia-home.org", "your-api-key")?;
//!
//!     // Discover devices
//!     let devices = client.discover_devices().await?;
//!     println!("Found {} devices", devices.len());
//!
//!     Ok(())
//! }
//! ```

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::HomeClient;
pub use error::{HomeError, Result};
pub use types::*;

/// WIA-HOME SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Philosophy: 弘益人間 (Benefit All Humanity)
pub const PHILOSOPHY: &str = "弘益人間 (홍익인간) - Benefit All Humanity";

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
