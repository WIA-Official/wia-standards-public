//! Integration module for WIA Material ecosystem connectivity
//!
//! This module provides:
//! - Data providers for external material databases (OPTIMADE, Materials Project)
//! - File format exporters/importers (CIF, POSCAR, XYZ)
//! - Unified integration manager for all integration features
//!
//! # Example
//!
//! ```rust,ignore
//! use wia_material::integration::{IntegrationManager, ProviderQuery, ExportFormat};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let mut manager = IntegrationManager::new();
//!
//!     // Search for materials
//!     let query = ProviderQuery {
//!         elements: Some(vec!["Fe".to_string(), "O".to_string()]),
//!         ..Default::default()
//!     };
//!
//!     let materials = manager.search(&query).await?;
//!
//!     // Export to CIF format
//!     if let Some(material) = materials.first() {
//!         let cif = manager.export(ExportFormat::Cif, material)?;
//!         println!("{}", cif);
//!     }
//!
//!     Ok(())
//! }
//! ```

pub mod converters;
pub mod error;
pub mod exporters;
pub mod manager;
pub mod providers;

// Re-export main types
pub use error::{IntegrationError, IntegrationResult};
pub use exporters::{
    CifExporter, ExportFormat, ExportOptions, Exporter, ExporterRegistry, ImportOptions,
    PoscarExporter, XyzExporter,
};
pub use manager::{IntegrationManager, IntegrationManagerBuilder};
pub use providers::{
    DataProvider, MaterialsProjectProvider, MockProvider, OptimadeProvider, ProviderConfig,
    ProviderQuery, ProviderRegistry, ProviderStatus,
};

/// Integration module version
pub const INTEGRATION_VERSION: &str = "1.0.0";
