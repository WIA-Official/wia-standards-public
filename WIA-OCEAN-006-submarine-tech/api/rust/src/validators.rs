//! Validation functions for submarine technology

use crate::error::{Result, SubmarineTechError};
use crate::types::*;

pub fn validate_item(item: &SubmarineTech) -> Result<()> {
    if item.name.is_empty() {
        return Err(SubmarineTechError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
