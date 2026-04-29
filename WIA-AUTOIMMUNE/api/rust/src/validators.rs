//! Validation functions for autoimmune disease data

use crate::error::{Error, Result};
use crate::types::*;

pub fn validate_patient(patient: &Patient) -> Result<()> {
    if patient.symptom_severity < 0.0 || patient.symptom_severity > 10.0 {
        return Err(Error::InvalidInput("Severity must be 0-10".into()));
    }
    Ok(())
}

pub fn validate_symptom_log(log: &SymptomLog) -> Result<()> {
    if log.symptoms.is_empty() {
        return Err(Error::InvalidInput("Must have at least one symptom".into()));
    }
    if log.severity < 0.0 || log.severity > 10.0 {
        return Err(Error::InvalidInput("Severity must be 0-10".into()));
    }
    Ok(())
}
