//! Validation functions for amr

use crate::error::{Result, AmrError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AmrError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
