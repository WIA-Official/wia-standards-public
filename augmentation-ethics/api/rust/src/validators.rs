//! Validation functions for augmentation-ethics

use crate::error::{Result, AugmentationEthicsError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AugmentationEthicsError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
