//! Adapters for XR Platform Integration
//!
//! This module provides adapters for various XR platforms and devices.

pub mod simulator;
pub mod platform;
pub mod wia;

pub use simulator::*;
pub use platform::*;
pub use wia::*;
