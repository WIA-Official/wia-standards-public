//! Validation functions for time management 008

use crate::error::{Result, TimeManagementError};
use crate::types::*;

pub fn validate_item(item: &TimeManagement) -> Result<()> {
    if item.name.is_empty() {
        return Err(TimeManagementError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
