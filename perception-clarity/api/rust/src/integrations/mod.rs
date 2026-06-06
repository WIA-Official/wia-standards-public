//! Integrations that turn clarity reports into downstream behavior.
//!
//! Each submodule consumes [`crate::types::Sensor`] / [`crate::types::ClarityState`]
//! and produces a domain-specific decision:
//!
//! - [`fusion`] — down-weight a degraded sensor in a fusion stack.
//! - [`auto`] — SAE J3016 fallback when a sensor goes blind.
//! - [`drone`] — return-to-home when a sensor is obstructed.
//! - [`vision_ai`] — gate inference on a minimum PCI.

pub mod fusion;
pub mod auto;
pub mod drone;
pub mod vision_ai;

pub use fusion::*;
pub use auto::*;
pub use drone::*;
pub use vision_ai::*;
