//! Validation functions for anti-gravity

use crate::error::{Result, AntiGravityError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AntiGravityError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
