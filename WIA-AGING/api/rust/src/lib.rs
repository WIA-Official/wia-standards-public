//! # WIA-AGING SDK for Rust
//!
//! A Rust SDK for interacting with the WIA-AGING Standard API.
//!
//! ## 弘益人間 (Benefit All Humanity)
//!
//! The WIA-AGING Standard enables dignified aging through technology by standardizing
//! data formats, APIs, and protocols for aging-related health information.
//!
//! ## Features
//!
//! - **Biological Age Assessment** - Calculate biological age using various methods
//! - **Biomarker Management** - Submit and retrieve biomarker data
//! - **Real-time Streaming** - WebSocket support for wearable device data
//! - **Profile Management** - Create and manage aging profiles
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use wia_aging_sdk::{WiaAgingClient, Config, CreateAssessmentRequest, BiologicalAgeMethod};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = WiaAgingClient::new(Config {
//!         api_key: "your-api-key".to_string(),
//!         environment: Environment::Production,
//!         ..Default::default()
//!     });
//!
//!     let assessment = client.assessments()
//!         .create("profile_id", CreateAssessmentRequest {
//!             method: BiologicalAgeMethod::PhenotypicLevine,
//!             biomarkers: vec![
//!                 Biomarker::new("WIA-AGE-001", 0.8, "mg/L"),
//!             ],
//!         })
//!         .await?;
//!
//!     println!("Biological Age: {}", assessment.biological_age.value);
//!     Ok(())
//! }
//! ```

#![warn(missing_docs)]
#![warn(rustdoc::missing_doc_code_examples)]

pub mod types;
pub mod client;
pub mod error;
pub mod streaming;

// Re-export main types
pub use types::*;
pub use client::WiaAgingClient;
pub use error::{Error, Result};

/// SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA-AGING Standard version
pub const STANDARD_VERSION: &str = "1.0.0";

/// Philosophy
pub const PHILOSOPHY: &str = "弘益人間 (Benefit All Humanity)";
