//! Validation functions for access-control

use crate::error::{Result, AccessControlError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AccessControlError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
