//! WIA Integration Adapters
//!
//! This module provides specific adapters for different WIA standards.

mod aac;
mod bci;
mod voice;
mod tts;
mod braille;

pub use aac::*;
pub use bci::*;
pub use voice::*;
pub use tts::*;
pub use braille::*;
