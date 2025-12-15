//! # WIA Bio Ecosystem Module
//!
//! Integration with external biotechnology ecosystem services.
//!
//! This module provides adapters for:
//! - GA4GH (DRS, WES, TES, Beacon)
//! - HL7 FHIR Genomics
//! - LIMS systems
//! - ELN platforms
//! - Bioinformatics databases (NCBI, UniProt)
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────┐
//! │         EcosystemManager            │
//! ├─────────────────────────────────────┤
//! │  ┌─────────┬─────────┬─────────┐   │
//! │  │ GA4GH   │  FHIR   │  NCBI   │   │
//! │  │ Adapter │ Adapter │ Adapter │   │
//! │  └─────────┴─────────┴─────────┘   │
//! └─────────────────────────────────────┘
//! ```
//!
//! ## Example
//!
//! ```rust,no_run
//! use wia_bio::ecosystem::*;
//!
//! #[tokio::main]
//! async fn main() {
//!     let mut manager = EcosystemManager::new();
//!     manager.register(Box::new(MockAdapter::new()));
//!
//!     let config = AdapterConfig::default();
//!     manager.initialize_all(&[config]).await.unwrap();
//! }
//! ```

mod base;
mod config;
mod manager;
mod mock;
mod ga4gh;
mod fhir;
mod database;

pub use base::*;
pub use config::*;
pub use manager::*;
pub use mock::*;
pub use ga4gh::*;
pub use fhir::*;
pub use database::*;
