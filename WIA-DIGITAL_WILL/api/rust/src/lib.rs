//! # WIA-DIGITAL_WILL SDK for Rust
//!
//! A Rust SDK for interacting with the WIA-DIGITAL_WILL Standard API.
//!
//! ## 弘益人間 (Benefit All Humanity)
//!
//! Ensuring digital continuity and rights protection in the modern age
//!
//! ## Features
//!
//! - **Data Validation** - Validate data against WIA-DIGITAL_WILL standards
//! - **API Client** - Easy-to-use client for WIA-DIGITAL_WILL API
//! - **Type Safety** - Strong typing for all API interactions
//! - **Async Support** - Built on Tokio for high performance
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use wia_digital_will_sdk::{DigitalWillClient, Config};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = DigitalWillClient::new(Config {
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
pub use client::DigitalWillClient;
pub use error::{Error, Result};

/// SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA-DIGITAL_WILL Standard version
pub const STANDARD_VERSION: &str = "1.0.0";

/// Philosophy
pub const PHILOSOPHY: &str = "弘益人間 (Benefit All Humanity)";
