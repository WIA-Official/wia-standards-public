//! Adapters for different sensor backends.
//!
//! - [`simulator`] — a deterministic mock sensor that produces synthetic PCI
//!   readings for testing and development.

mod simulator;

pub use simulator::*;
