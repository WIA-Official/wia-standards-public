//! Validation functions for adas

use crate::error::{Result, AdasError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AdasError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
