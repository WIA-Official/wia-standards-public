//! # WIA-DDOS_PROTECTION SDK for Rust
//!
//! A Rust SDK for interacting with the WIA-DDOS_PROTECTION Standard API.
//!
//! ## 弘益人間 (Benefit All Humanity)
//!
//! Protecting digital infrastructure and users through standardized security protocols
//!
//! ## Features
//!
//! - **Data Validation** - Validate data against WIA-DDOS_PROTECTION standards
//! - **API Client** - Easy-to-use client for WIA-DDOS_PROTECTION API
//! - **Type Safety** - Strong typing for all API interactions
//! - **Async Support** - Built on Tokio for high performance
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use wia_ddos_protection_sdk::{DdosProtectionClient, Config};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = DdosProtectionClient::new(Config {
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
pub use client::DdosProtectionClient;
pub use error::{Error, Result};

/// SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA-DDOS_PROTECTION Standard version
pub const STANDARD_VERSION: &str = "1.0.0";

/// Philosophy
pub const PHILOSOPHY: &str = "弘益人間 (Benefit All Humanity)";
