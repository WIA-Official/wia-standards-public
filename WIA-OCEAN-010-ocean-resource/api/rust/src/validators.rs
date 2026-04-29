//! Validation functions for ocean resource

use crate::error::{Result, OceanResourceError};
use crate::types::*;

pub fn validate_item(item: &OceanResource) -> Result<()> {
    if item.name.is_empty() {
        return Err(OceanResourceError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
