//! WIA Robot Protocol (WRP) implementation
//!
//! This module provides the core protocol types and handlers for
//! communication between WIA Robot devices.

pub mod message;
pub mod builder;
pub mod handler;
pub mod error;

pub use message::*;
pub use builder::*;
pub use handler::*;
pub use error::*;
