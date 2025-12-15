//! WIA Health Integration Module
//!
//! Integration adapters for external systems including FHIR, wearables, and dashboards

mod adapter;
mod fhir;
mod wearable;
mod dashboard;
mod export;
mod manager;

pub use adapter::*;
pub use fhir::*;
pub use wearable::*;
pub use dashboard::*;
pub use export::*;
pub use manager::*;
