//! # WIA-DEFI SDK for Rust
//!
//! A Rust SDK for interacting with the WIA-DEFI Standard API.
//!
//! ## 弘益人間 (Benefit All Humanity)
//!
//! Financial technology that benefits all humanity through transparent, secure, and accessible services
//!
//! ## Features
//!
//! - **Data Validation** - Validate data against WIA-DEFI standards
//! - **API Client** - Easy-to-use client for WIA-DEFI API
//! - **Type Safety** - Strong typing for all API interactions
//! - **Async Support** - Built on Tokio for high performance
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use wia_defi_sdk::{DefiClient, Config};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = DefiClient::new(Config {
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
pub use client::DefiClient;
pub use error::{Error, Result};

/// SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA-DEFI Standard version
pub const STANDARD_VERSION: &str = "1.0.0";

/// Philosophy
pub const PHILOSOPHY: &str = "弘益人間 (Benefit All Humanity)";
