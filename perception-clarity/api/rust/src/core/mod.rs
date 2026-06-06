//! Core functionality for the WIA Perception Clarity SDK.
//!
//! - [`pci`] — the PCI computation, the PCI → state mapping, and the
//!   state → safe-action mapping.
//! - [`builders`] — ergonomic builders for [`crate::types::Sensor`] and
//!   [`crate::types::SensorClarityReport`].

mod pci;
mod builders;

pub use pci::*;
pub use builders::*;
