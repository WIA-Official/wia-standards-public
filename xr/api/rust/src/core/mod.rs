//! Core XR Accessibility Engine
//!
//! This module provides the main accessibility engine and profile management.

pub mod xr;
pub mod profile;
pub mod adaptation;
pub mod session;

pub use xr::*;
pub use profile::*;
pub use adaptation::*;
pub use session::*;
