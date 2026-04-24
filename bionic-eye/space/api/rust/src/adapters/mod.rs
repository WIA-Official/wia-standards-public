//! Adapters for external systems and simulators
//!
//! This module provides adapters for integrating with various simulation
//! tools and external data sources.

mod simulator;
mod io;

pub use simulator::*;
pub use io::*;
