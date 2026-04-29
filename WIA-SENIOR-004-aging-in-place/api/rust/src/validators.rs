//! Validation functions for aging in place

use crate::error::{Result, AgingInPlaceError};
use crate::types::*;

pub fn validate_item(item: &AgingInPlace) -> Result<()> {
    if item.name.is_empty() {
        return Err(AgingInPlaceError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
