//! WIA ecosystem integration module
//!
//! This module provides integration with other WIA standards
//! including Exoskeleton and Bionic Eye systems.

pub mod wia_bridge;
pub mod events;
pub mod auth;

pub use wia_bridge::*;
pub use events::*;
pub use auth::*;
