//! Validation functions for chronic pain data

use crate::error::{Error, Result};
use crate::types::*;

pub fn validate_pain_profile(profile: &PainProfile) -> Result<()> {
    if profile.severity < 0.0 || profile.severity > 10.0 {
        return Err(Error::InvalidInput("Pain severity must be 0-10".into()));
    }
    if profile.location.is_empty() {
        return Err(Error::InvalidInput("Pain location cannot be empty".into()));
    }
    Ok(())
}

pub fn validate_pain_log(log: &PainLog) -> Result<()> {
    if log.pain_level < 0.0 || log.pain_level > 10.0 {
        return Err(Error::InvalidInput("Pain level must be 0-10".into()));
    }
    Ok(())
}
