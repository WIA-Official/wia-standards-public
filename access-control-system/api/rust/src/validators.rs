//! Validation functions for access-control-system

use crate::error::{Result, AccessControlSystemError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AccessControlSystemError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
