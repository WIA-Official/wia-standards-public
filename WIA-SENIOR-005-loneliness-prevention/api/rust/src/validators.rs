//! Validation functions for loneliness prevention

use crate::error::{Result, LonelinessPreventionError};
use crate::types::*;

pub fn validate_item(item: &LonelinessPrevention) -> Result<()> {
    if item.name.is_empty() {
        return Err(LonelinessPreventionError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
