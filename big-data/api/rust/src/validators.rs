//! Validation functions for big-data

use crate::error::{Result, BigDataError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(BigDataError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
