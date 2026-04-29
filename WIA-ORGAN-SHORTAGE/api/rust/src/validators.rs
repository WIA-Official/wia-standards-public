//! Validation functions for organ shortage

use crate::error::{Result, OrganShortageError};
use crate::types::*;

pub fn validate_item(item: &OrganShortage) -> Result<()> {
    if item.name.is_empty() {
        return Err(OrganShortageError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
