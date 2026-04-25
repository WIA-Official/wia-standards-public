//! Validation functions for bim

use crate::error::{Result, BimError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(BimError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
