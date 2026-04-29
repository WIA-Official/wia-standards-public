//! Validation functions for dementia care

use crate::error::{Result, DementiaCareError};
use crate::types::*;

pub fn validate_item(item: &DementiaCare) -> Result<()> {
    if item.name.is_empty() {
        return Err(DementiaCareError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
