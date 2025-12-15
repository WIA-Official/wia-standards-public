//! WIA Space Output Layer (Phase 4)
//!
//! This module provides output adapters for integrating WIA Space Standard
//! with external systems like visualization tools, data exporters, and dashboards.
//!
//! ## Output Types
//!
//! - `Visualization`: 3D visualization (CesiumJS, Three.js)
//! - `Export`: Data export (CCSDS, JSON, CSV)
//! - `Dashboard`: Real-time dashboards (OpenMCT)
//! - `Alert`: Notification systems (WebHook, Email)

mod adapter;
mod manager;
mod exporter;
mod data;

pub use adapter::*;
pub use manager::*;
pub use exporter::*;
pub use data::*;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_output_type() {
        assert!(OutputType::Visualization.is_visual());
        assert!(OutputType::Export.is_export());
        assert!(!OutputType::Alert.is_visual());
    }
}
