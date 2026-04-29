//! # WIA-CHILD-010-predator-detection SDK for Rust
//!
//! A Rust SDK for interacting with the WIA-CHILD-010-predator-detection Standard API.
//!
//! ## 弘益人間 (Benefit All Humanity)
//!
//! Creating a safer digital environment for children worldwide
//!
//! ## Features
//!
//! - **Data Validation** - Validate data against WIA-CHILD-010-predator-detection standards
//! - **API Client** - Easy-to-use client for WIA-CHILD-010-predator-detection API
//! - **Type Safety** - Strong typing for all API interactions
//! - **Async Support** - Built on Tokio for high performance
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use wia_child_010_predator_detection_sdk::{Child010PredatorDetectionClient, Config};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = Child010PredatorDetectionClient::new(Config {
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
pub use client::Child010PredatorDetectionClient;
pub use error::{Error, Result};

/// SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA-CHILD-010-predator-detection Standard version
pub const STANDARD_VERSION: &str = "1.0.0";

/// Philosophy
pub const PHILOSOPHY: &str = "弘益人間 (Benefit All Humanity)";
