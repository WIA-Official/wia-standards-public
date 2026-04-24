//! Validation functions for ai-city

use crate::error::{Result, AiCityError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AiCityError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
