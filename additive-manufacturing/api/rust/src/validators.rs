//! Validation functions for additive-manufacturing

use crate::error::{Result, AdditiveManufacturingError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AdditiveManufacturingError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
