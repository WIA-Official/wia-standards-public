//! Validation functions for alzheimers

use crate::error::{Result, AlzheimersError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AlzheimersError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
