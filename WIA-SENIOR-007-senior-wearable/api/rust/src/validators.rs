//! Validation functions for senior wearable

use crate::error::{Result, SeniorWearableError};
use crate::types::*;

pub fn validate_item(item: &SeniorWearable) -> Result<()> {
    if item.name.is_empty() {
        return Err(SeniorWearableError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
