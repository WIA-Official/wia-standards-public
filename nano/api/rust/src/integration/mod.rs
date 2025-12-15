//! WIA Ecosystem Integration
//!
//! This module provides integration with the broader WIA ecosystem including:
//! - WIA Gateway for cross-domain communication
//! - Device registry for discovery and management
//! - Event system for real-time notifications
//! - Interoperability with other WIA domains (AAC, Climate, etc.)

mod gateway;
mod registry;
mod events;
mod interop;

pub use gateway::*;
pub use registry::*;
pub use events::*;
pub use interop::*;
