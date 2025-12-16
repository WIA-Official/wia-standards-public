//! Transport layer for WIA Robot Protocol
//!
//! This module provides transport abstractions and implementations
//! for sending WRP messages over various protocols.

pub mod base;
pub mod mock;

pub use base::*;
pub use mock::*;
