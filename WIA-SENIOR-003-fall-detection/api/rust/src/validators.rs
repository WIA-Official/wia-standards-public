//! Validation functions for fall detection

use crate::error::{Result, FallDetectionError};
use crate::types::*;

pub fn validate_item(item: &FallDetection) -> Result<()> {
    if item.name.is_empty() {
        return Err(FallDetectionError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
