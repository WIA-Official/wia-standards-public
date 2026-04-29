//! # WIA-HEALTH SDK for Rust
//!
//! A Rust SDK for interacting with the WIA-HEALTH Standard API.
//!
//! ## 弘益人間 (Benefit All Humanity)
//!
//! Advancing global health through standardized medical data and protocols
//!
//! ## Features
//!
//! - **Data Validation** - Validate data against WIA-HEALTH standards
//! - **API Client** - Easy-to-use client for WIA-HEALTH API
//! - **Type Safety** - Strong typing for all API interactions
//! - **Async Support** - Built on Tokio for high performance
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use wia_health_sdk::{HealthClient, Config};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = HealthClient::new(Config {
//!         api_key: "your-api-key".to_string(),
//!         base_url: None,
//!         timeout_secs: 30,
//!     });
//!
//!     let result = client.validate(&data).await?;
//!     println!("Validation result: {:?}", result);
//!     Ok(())
//! }
//! ```

#![warn(missing_docs)]

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

// Re-export main types
pub use types::*;
pub use client::HealthClient;
pub use error::{Error, Result};

/// SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA-HEALTH Standard version
pub const STANDARD_VERSION: &str = "1.0.0";

/// Philosophy
pub const PHILOSOPHY: &str = "弘益人間 (Benefit All Humanity)";
