//! # WIA Space Standard
//!
//! Rust implementation of the WIA Space Standard for advanced space technology data formats.
//!
//! ## Overview
//!
//! This crate provides data structures and APIs for six key space technology areas:
//!
//! - **Dyson Sphere**: Stellar energy harvesting megastructures
//! - **Mars Terraforming**: Planetary environment modification
//! - **Warp Drive**: Spacetime propulsion systems
//! - **Space Elevator**: Orbital access infrastructure
//! - **Asteroid Mining**: Space resource extraction
//! - **Interstellar Travel**: Interstellar mission systems
//!
//! ## Quick Start
//!
//! ```rust
//! use wia_space::prelude::*;
//!
//! // Create an asteroid mining project
//! let asteroid = TargetAsteroid::new("16 Psyche", AsteroidType::MType)
//!     .with_diameter_km(226.0)
//!     .with_mass_kg(2.72e19);
//!
//! let project = AsteroidMiningSpec::new("mining-001", asteroid);
//! ```
//!
//! ## Features
//!
//! - `wasm`: WebAssembly support for browser environments
//! - `python`: Python bindings via PyO3
//!
//! ## 弘益人間 - Benefit All Humanity

pub mod types;
pub mod core;
pub mod adapters;
pub mod error;
pub mod protocol;
pub mod transport;
pub mod output;

// Re-exports for convenience
pub use types::*;
pub use core::*;
pub use error::{SpaceError, SpaceResult};

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::types::*;
    pub use crate::core::*;
    pub use crate::error::{SpaceError, SpaceResult};
    pub use crate::protocol::{
        WspMessage, MessageType, Endpoint, EndpointType,
        MessageBuilder, Priority, MessageMetadata,
    };
    pub use crate::transport::{
        Transport, TransportType, TransportConfig,
        MockTransport, LatencyConfig, LatencySimulator,
    };
    pub use crate::output::{
        OutputAdapter, OutputType, OutputFormat, OutputConfig, OutputError,
        OutputManager, OutputData, OutputResult, OutputMetadata,
        CcsdsOemExporter, CcsdsOpmExporter, JsonExporter, CsvExporter,
        GmatScriptExporter, CzmlExporter,
    };
}

/// WIA Space standard version
pub const WIA_VERSION: &str = "1.0.0";

/// Schema base URL
pub const SCHEMA_BASE_URL: &str = "https://wia.live/schemas/space/";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert_eq!(WIA_VERSION, "1.0.0");
    }
}
