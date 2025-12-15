//! # WIA Material SDK
//!
//! Rust SDK for WIA Material Science Standards.
//!
//! This library provides a standardized interface for working with material science data
//! including superconductors, metamaterials, memristors, topological insulators,
//! holographic storage, and programmable matter.
//!
//! ## Quick Start
//!
//! ```rust
//! use wia_material::{WiaMaterial, MaterialBuilder, MaterialType};
//!
//! #[tokio::main]
//! async fn main() {
//!     // Create a client
//!     let client = WiaMaterial::new();
//!
//!     // Create a material using the builder
//!     let material = MaterialBuilder::new()
//!         .material_type(MaterialType::Superconductor)
//!         .name("YBCO")
//!         .formula("YBa2Cu3O7-x")
//!         .confidence(0.95)
//!         .build()
//!         .unwrap();
//!
//!     // Store the material
//!     let stored = client.create_material(material).await.unwrap();
//!     println!("Created: {}", stored.material_id);
//! }
//! ```
//!
//! ## Features
//!
//! - **Type-safe data structures**: All material types are strongly typed with validation
//! - **Async support**: Built on Tokio for efficient async operations
//! - **Extensible adapters**: Support for various data sources (simulators, databases, APIs)
//! - **Domain-specific types**: Specialized types for each material science domain
//!
//! ## Material Types
//!
//! - `Superconductor`: High-temperature and room-temperature superconductors
//! - `Metamaterial`: Electromagnetic, acoustic, and mechanical metamaterials
//! - `Memristor`: Resistive switching devices for neuromorphic computing
//! - `TopologicalInsulator`: Quantum materials with topological surface states
//! - `HolographicStorage`: Volumetric data storage media
//! - `ProgrammableMatter`: Self-reconfiguring materials (claytronics)
//!
//! ## License
//!
//! MIT License - This standard belongs to humanity.
//!
//! 弘益人間 (Benefit All Humanity)

// Re-export lazy_static and regex for internal use
#[macro_use]
extern crate lazy_static;
extern crate regex;

pub mod adapters;
pub mod core;
pub mod error;
pub mod integration;
pub mod protocol;
pub mod transport;
pub mod types;

// Re-export main types for convenience
pub use adapters::{MaterialAdapter, MaterialQuery, SimulatorAdapter};
pub use core::{MaterialBuilder, MaterialConfig, MaterialEvent, MaterialEventHandler, MaterialStatistics, WiaMaterial};
pub use error::{MaterialError, MaterialResult};
pub use protocol::{
    Filter, FilterBuilder, FilterCondition, FilterOperator, Message, MessageBuilder, MessageType,
    Pagination, QueryPayload, SortConfig, SortOrder, SubscriptionChannel,
};
pub use transport::{
    ConnectionState, HttpTransport, StreamingTransport, Transport, TransportConfig,
    WebSocketConfig, WebSocketTransport,
};
pub use types::*;

// Integration module exports
pub use integration::{
    CifExporter, DataProvider, ExportFormat, ExportOptions, Exporter, ExporterRegistry,
    ImportOptions, IntegrationError, IntegrationManager, IntegrationManagerBuilder,
    IntegrationResult, MaterialsProjectProvider, MockProvider, OptimadeProvider,
    PoscarExporter, ProviderConfig, ProviderQuery, ProviderRegistry, ProviderStatus,
    XyzExporter,
};

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Schema base URL
pub const SCHEMA_BASE_URL: &str = "https://wia.live/material/v1";

/// Get the schema URL for a specific material type
pub fn schema_url(material_type: MaterialType) -> String {
    match material_type {
        MaterialType::Superconductor => format!("{}/superconductor.schema.json", SCHEMA_BASE_URL),
        MaterialType::Metamaterial => format!("{}/metamaterial.schema.json", SCHEMA_BASE_URL),
        MaterialType::ProgrammableMatter => {
            format!("{}/programmable-matter.schema.json", SCHEMA_BASE_URL)
        }
        MaterialType::HolographicStorage => {
            format!("{}/holographic-storage.schema.json", SCHEMA_BASE_URL)
        }
        MaterialType::Memristor => format!("{}/memristor.schema.json", SCHEMA_BASE_URL),
        MaterialType::TopologicalInsulator => {
            format!("{}/topological-insulator.schema.json", SCHEMA_BASE_URL)
        }
        MaterialType::Custom => format!("{}/schema.json", SCHEMA_BASE_URL),
    }
}

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::adapters::{MaterialAdapter, MaterialQuery, SimulatorAdapter};
    pub use crate::core::{MaterialBuilder, MaterialConfig, WiaMaterial};
    pub use crate::error::{MaterialError, MaterialResult};
    pub use crate::integration::{
        CifExporter, DataProvider, ExportFormat, ExportOptions, Exporter, ImportOptions,
        IntegrationError, IntegrationManager, IntegrationManagerBuilder, IntegrationResult,
        PoscarExporter, ProviderConfig, ProviderQuery, ProviderStatus, XyzExporter,
    };
    pub use crate::protocol::{
        Filter, FilterBuilder, FilterCondition, FilterOperator, Message, MessageBuilder,
        MessageType, Pagination, QueryPayload, SortConfig, SortOrder, SubscriptionChannel,
    };
    pub use crate::transport::{
        ConnectionState, HttpTransport, StreamingTransport, Transport, TransportConfig,
        WebSocketConfig, WebSocketTransport,
    };
    pub use crate::types::{
        CrystalSystem, ElectricalProperties, Identity, LatticeParameters, MagneticProperties,
        MaterialData, MaterialType, Measurement, MechanicalProperties, Meta, OpticalProperties,
        Properties, Provenance, Structure, ThermalProperties, Timestamp,
    };
    pub use crate::types::{
        HolographicStorageProperties, MemristorProperties, MetamaterialProperties,
        ProgrammableMatterProperties, SuperconductorProperties, TopologicalInsulatorProperties,
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert_eq!(VERSION, "1.0.0");
    }

    #[test]
    fn test_schema_url() {
        assert!(schema_url(MaterialType::Superconductor).contains("superconductor"));
        assert!(schema_url(MaterialType::TopologicalInsulator).contains("topological-insulator"));
    }

    #[tokio::test]
    async fn test_full_workflow() {
        use crate::prelude::*;

        // Create client
        let client = WiaMaterial::new();

        // Build material
        let material = MaterialBuilder::new()
            .material_type(MaterialType::Superconductor)
            .name("Test Superconductor")
            .formula("TestFormula")
            .confidence(0.9)
            .build()
            .unwrap();

        // Store
        let stored = client.create_material(material).await.unwrap();
        assert!(stored.material_id.starts_with("wia-mat-"));

        // Retrieve
        let retrieved = client.get_material(&stored.material_id).await.unwrap();
        assert_eq!(retrieved.identity.name, "Test Superconductor");

        // Statistics
        let stats = client.get_statistics().await;
        assert_eq!(stats.total_count, 1);
        assert_eq!(stats.superconductor_count, 1);
    }
}
