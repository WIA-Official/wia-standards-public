//! Validation functions for intergenerational

use crate::error::{Result, IntergenerationalError};
use crate::types::*;

pub fn validate_item(item: &Intergenerational) -> Result<()> {
    if item.name.is_empty() {
        return Err(IntergenerationalError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
