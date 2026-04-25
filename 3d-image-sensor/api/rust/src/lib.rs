//! # 3d-image-sensor SDK for Rust
//!
//! A Rust SDK for interacting with the 3d-image-sensor Standard API.
//!
//! ## 弘益人間 (Benefit All Humanity)
//!
//! Standardizing hardware interfaces for universal accessibility
//!
//! ## Features
//!
//! - **Data Validation** - Validate data against 3d-image-sensor standards
//! - **API Client** - Easy-to-use client for 3d-image-sensor API
//! - **Type Safety** - Strong typing for all API interactions
//! - **Async Support** - Built on Tokio for high performance
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use 3d_image_sensor_sdk::{ThreeDImageSensorClient, Config};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = ThreeDImageSensorClient::new(Config {
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
pub use client::ThreeDImageSensorClient;
pub use error::{Error, Result};

/// SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// 3d-image-sensor Standard version
pub const STANDARD_VERSION: &str = "1.0.0";

/// Philosophy
pub const PHILOSOPHY: &str = "弘益人間 (Benefit All Humanity)";
