//! # 5g-6g-spectrum SDK for Rust
//!
//! A Rust SDK for interacting with the 5g-6g-spectrum Standard API.
//!
//! ## 弘益人間 (Benefit All Humanity)
//!
//! Enabling next-generation communication for global connectivity
//!
//! ## Features
//!
//! - **Data Validation** - Validate data against 5g-6g-spectrum standards
//! - **API Client** - Easy-to-use client for 5g-6g-spectrum API
//! - **Type Safety** - Strong typing for all API interactions
//! - **Async Support** - Built on Tokio for high performance
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use 5g_6g_spectrum_sdk::{FiveGSixGSpectrumClient, Config};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = FiveGSixGSpectrumClient::new(Config {
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
pub use client::FiveGSixGSpectrumClient;
pub use error::{Error, Result};

/// SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// 5g-6g-spectrum Standard version
pub const STANDARD_VERSION: &str = "1.0.0";

/// Philosophy
pub const PHILOSOPHY: &str = "弘益人間 (Benefit All Humanity)";
