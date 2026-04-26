//! Validation functions for ai-chip

use crate::error::{Result, AiChipError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AiChipError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
