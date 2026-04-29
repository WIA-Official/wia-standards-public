//! Ecosystem integration module for WIA CareBot
//!
//! This module provides integrations with:
//! - Healthcare systems (HL7 FHIR)
//! - Smart home platforms (Matter, HomeKit)
//! - Emergency services (119, 112)
//! - Family mobile apps
//! - Cloud platforms

pub mod fhir;
pub mod smart_home;
pub mod export;

pub use fhir::*;
pub use smart_home::*;
pub use export::*;
