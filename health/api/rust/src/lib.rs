//! # WIA Health Standard
//!
//! Rust implementation of the WIA Health & Longevity Data Standard.
//!
//! This crate provides comprehensive types and utilities for working with
//! health and longevity data, including:
//!
//! - **Biomarkers**: Inflammatory, metabolic, hormonal markers and aging clocks
//! - **Genomics**: Sequencing, variants, pharmacogenomics, polygenic risk scores
//! - **Epigenetics**: Methylation age, senescence markers, reprogramming history
//! - **Telomeres**: Length measurements, telomerase activity, interventions
//! - **Digital Twin**: Multi-modal health simulation and predictions
//! - **Interventions**: Longevity treatment tracking and outcomes
//!
//! ## Quick Start
//!
//! ```rust
//! use wia_health::prelude::*;
//! use uuid::Uuid;
//! use chrono::NaiveDate;
//!
//! // Create a subject
//! let subject = Subject {
//!     id: Uuid::new_v4(),
//!     anonymized_id: Some("patient-001".to_string()),
//!     birth_year: Some(1985),
//!     biological_sex: Some(BiologicalSex::Male),
//!     ethnicity: None,
//!     consent: Some(Consent {
//!         data_sharing: true,
//!         research: Some(true),
//!         consent_date: NaiveDate::from_ymd_opt(2025, 1, 1).unwrap(),
//!         version: Some("1.0".to_string()),
//!         expiration_date: None,
//!     }),
//! };
//!
//! // Build a health profile
//! let profile = HealthProfileBuilder::new()
//!     .subject(subject)
//!     .build()
//!     .expect("Failed to build profile");
//!
//! println!("Profile ID: {}", profile.id);
//! ```
//!
//! ## Features
//!
//! - `wasm` - WebAssembly support for browser usage
//! - `python` - Python bindings via PyO3
//! - `nodejs` - Node.js bindings via Neon
//!
//! ## License
//!
//! MIT License - This standard belongs to humanity.
//!
//! 弘益人間 - Benefit All Humanity

#![warn(missing_docs)]
#![warn(rustdoc::missing_crate_level_docs)]

pub mod adapters;
pub mod core;
pub mod error;
pub mod integration;
pub mod protocol;
pub mod transport;
pub mod types;

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::adapters::*;
    pub use crate::core::*;
    pub use crate::error::*;
    pub use crate::integration::*;
    pub use crate::protocol::*;
    pub use crate::transport::*;
    pub use crate::types::*;
}

// Re-export commonly used items at crate root
pub use crate::core::HealthProfileBuilder;
pub use crate::error::{HealthError, Result};
pub use crate::types::HealthProfile;

/// Crate version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Standard version
pub const STANDARD_VERSION: &str = "1.0.0";

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::NaiveDate;
    use uuid::Uuid;

    #[test]
    fn test_version() {
        assert_eq!(VERSION, "1.0.0");
        assert_eq!(STANDARD_VERSION, "1.0.0");
    }

    #[test]
    fn test_prelude_imports() {
        use crate::prelude::*;

        let subject = Subject {
            id: Uuid::new_v4(),
            anonymized_id: None,
            birth_year: Some(1990),
            biological_sex: Some(BiologicalSex::Female),
            ethnicity: None,
            consent: Some(Consent {
                data_sharing: true,
                research: None,
                consent_date: NaiveDate::from_ymd_opt(2025, 1, 1).unwrap(),
                version: None,
                expiration_date: None,
            }),
        };

        let profile = HealthProfileBuilder::new().subject(subject).build();

        assert!(profile.is_ok());
    }
}
