//! WIA Physics Ecosystem Integration Module
//!
//! This module provides adapters for integrating WIA Physics data with external systems:
//! - Control systems (EPICS, TANGO)
//! - Data archives (HDF5)
//! - Time-series databases (InfluxDB)
//! - Visualization (Grafana)

pub mod traits;
pub mod manager;
pub mod hdf5;
pub mod influxdb;
pub mod epics;

pub use traits::*;
pub use manager::*;
pub use hdf5::*;
pub use influxdb::*;
pub use epics::*;

/// Integration module version
pub const INTEGRATION_VERSION: &str = "1.0.0";
