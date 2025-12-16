//! Robot type adapters
//!
//! This module provides specialized implementations for each robot type.

pub mod exoskeleton;
pub mod prosthetics;
pub mod rehabilitation;
pub mod care;
pub mod surgical;
pub mod mobility;

pub use exoskeleton::*;
pub use prosthetics::*;
pub use rehabilitation::*;
pub use care::*;
pub use surgical::*;
pub use mobility::*;
