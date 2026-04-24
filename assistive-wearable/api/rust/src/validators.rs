//! Validation functions for assistive-wearable

use crate::error::{Result, AssistiveWearableError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AssistiveWearableError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
