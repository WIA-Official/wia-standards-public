//! Medical system integration adapters
//!
//! This module provides exporters for medical system integration:
//!
//! - **HL7 FHIR**: Fast Healthcare Interoperability Resources

pub mod fhir;

pub use fhir::FhirExporter;
