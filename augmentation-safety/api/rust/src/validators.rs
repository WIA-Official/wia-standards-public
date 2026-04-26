//! Validation functions for augmentation-safety

use crate::error::{Result, AugmentationSafetyError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AugmentationSafetyError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
