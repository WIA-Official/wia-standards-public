//! WIA Smart Home Communication Protocol
//! 弘益人間 - Benefit All Humanity
//!
//! This module implements the WIA Smart Home communication protocol,
//! built on top of Matter with accessibility extensions.

pub mod accessibility;
pub mod discovery;
pub mod matter;

mod messages;

pub use accessibility::*;
pub use discovery::*;
pub use matter::*;
pub use messages::*;
