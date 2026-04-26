//! Validation functions for beauty-tech

use crate::error::{Result, BeautyTechError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(BeautyTechError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
