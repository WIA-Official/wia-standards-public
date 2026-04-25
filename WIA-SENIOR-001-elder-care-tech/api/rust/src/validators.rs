//! Validation functions for elder care technology

use crate::error::{Result, ElderCareError};
use crate::types::*;

pub fn validate_item(item: &ElderCare) -> Result<()> {
    if item.name.is_empty() {
        return Err(ElderCareError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
