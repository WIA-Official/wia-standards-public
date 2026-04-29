//! # WIA-CHILD-006-child-data-privacy SDK for Rust
//!
//! A Rust SDK for interacting with the WIA-CHILD-006-child-data-privacy Standard API.
//!
//! ## 弘益人間 (Benefit All Humanity)
//!
//! Creating a safer digital environment for children worldwide
//!
//! ## Features
//!
//! - **Data Validation** - Validate data against WIA-CHILD-006-child-data-privacy standards
//! - **API Client** - Easy-to-use client for WIA-CHILD-006-child-data-privacy API
//! - **Type Safety** - Strong typing for all API interactions
//! - **Async Support** - Built on Tokio for high performance
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use wia_child_006_child_data_privacy_sdk::{Child006ChildDataPrivacyClient, Config};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = Child006ChildDataPrivacyClient::new(Config {
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
pub use client::Child006ChildDataPrivacyClient;
pub use error::{Error, Result};

/// SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA-CHILD-006-child-data-privacy Standard version
pub const STANDARD_VERSION: &str = "1.0.0";

/// Philosophy
pub const PHILOSOPHY: &str = "弘益人間 (Benefit All Humanity)";
