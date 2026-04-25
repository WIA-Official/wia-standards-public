//! Validation functions for marine sensor

use crate::error::{Result, MarineSensorError};
use crate::types::*;

pub fn validate_item(item: &MarineSensor) -> Result<()> {
    if item.name.is_empty() {
        return Err(MarineSensorError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
