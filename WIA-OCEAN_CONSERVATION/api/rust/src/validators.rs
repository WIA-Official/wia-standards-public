//! Validation functions for ocean conservation

use crate::error::{Result, OceanConservationError};
use crate::types::*;

pub fn validate_item(item: &OceanConservation) -> Result<()> {
    if item.name.is_empty() {
        return Err(OceanConservationError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
