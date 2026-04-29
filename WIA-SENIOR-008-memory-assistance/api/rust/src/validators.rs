//! Validation functions for memory assistance

use crate::error::{Result, MemoryAssistanceError};
use crate::types::*;

pub fn validate_item(item: &MemoryAssistance) -> Result<()> {
    if item.name.is_empty() {
        return Err(MemoryAssistanceError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
